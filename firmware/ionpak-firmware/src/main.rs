#![feature(asm, used, const_fn, core_float, use_extern_macros, naked_functions,
    core_intrinsics)]
#![no_std]
#![allow(unused_mut)]

extern crate cortex_m;
extern crate cortex_m_rt;
extern crate tm4c129x;

use core::cell::{Cell, RefCell};
use cortex_m::exception::Handlers as ExceptionHandlers;
use cortex_m::interrupt::Mutex;
use tm4c129x::interrupt::Interrupt;
use tm4c129x::interrupt::Handlers as InterruptHandlers;
use cortex_m::ctxt::Context;

extern crate smoltcp;
use smoltcp::Error;
use smoltcp::wire::{EthernetAddress, IpAddress};
use smoltcp::iface::{ArpCache, SliceArpCache, EthernetInterface};
use smoltcp::socket::{AsSocket, SocketSet};
use smoltcp::socket::{TcpSocket, TcpSocketBuffer};

use board::UART0;
use board::complete_fmt_write;

const LEDD1: u8 = 0x1; // PN0
const LEDD2: u8 = 0x2; // PN1

#[macro_export]
macro_rules! print {
    ($($arg:tt)*) => ({
        use core::fmt::Write;
        write!($crate::UART0, $($arg)*).unwrap()
    })
}

#[macro_export]
macro_rules! println {
    ($fmt:expr) => (print!(concat!($fmt, "\n")));
    ($fmt:expr, $($arg:tt)*) => (print!(concat!($fmt, "\n"), $($arg)*));
}

#[macro_use]
mod board;
mod pid;
mod loop_anode;
mod loop_cathode;
mod electrometer;
mod ethmac;

static TIME: Mutex<Cell<u64>> = Mutex::new(Cell::new(0));

fn get_time() -> u64 {
    cortex_m::interrupt::free(|cs| {
        TIME.borrow(cs).get()
    })
}

static LOOP_ANODE: Mutex<RefCell<loop_anode::Controller>> = Mutex::new(RefCell::new(
    loop_anode::Controller::new()));

static LOOP_CATHODE: Mutex<RefCell<loop_cathode::Controller>> = Mutex::new(RefCell::new(
    loop_cathode::Controller::new()));

static ELECTROMETER: Mutex<RefCell<electrometer::Electrometer>> = Mutex::new(RefCell::new(
    electrometer::Electrometer::new()));

fn address<T>(r: *const T) -> usize {
    r as usize
}

//#[no_mangle]
//pub extern fn main() -> i32 {
fn main() {

    // Enable the FPU
    unsafe {
        asm!("
            PUSH {R0, R1}
            LDR.W R0, =0xE000ED88
            LDR R1, [R0]
            ORR R1, R1, #(0xF << 20)
            STR R1, [R0]
            DSB
            ISB
            POP {R0, R1}
        ");
    }
    // Beware of the compiler inserting FPU instructions
    // in the prologue of functions before the FPU is enabled!
    main_with_fpu();
}

#[inline(never)]
fn main_with_fpu() {

    board::init();

    cortex_m::interrupt::free(|cs| {
        let nvic = tm4c129x::NVIC.borrow(cs);
        nvic.enable(Interrupt::TIMER0A);
    });

    let flashctl = tm4c129x::FLASH_CTRL.get();
    let userreg0 = unsafe { (*flashctl).userreg0.read().bits() };
    let userreg1 = unsafe { (*flashctl).userreg1.read().bits() };

    let     hardware_addr = EthernetAddress([userreg0 as u8, (userreg0 >> 8) as u8, (userreg0 >> 16) as u8, userreg1 as u8, (userreg1 >> 8) as u8, (userreg1 >> 16) as u8]);
    println!("using IC MAC address {}", hardware_addr);

    let mut protocol_addrs = [IpAddress::v4(192, 168, 3, 200)];
    println!("Using default IP address {}", protocol_addrs[0]);

    let mut arp_cache_entries: [_; 8] = Default::default();
    let mut arp_cache = SliceArpCache::new(&mut arp_cache_entries[..]);
    
    ethmac::init(hardware_addr.0);
    let mut device = ethmac::EthernetDevice;

    let mut iface = EthernetInterface::new(
        &mut device, &mut arp_cache as &mut ArpCache,
        hardware_addr, &mut protocol_addrs[..]);

    let server_socket = {
        // It is not strictly necessary to use a `static mut` and unsafe code here, but
        // on embedded systems that smoltcp targets it is far better to allocate the data
        // statically to verify that it fits into RAM rather than get undefined behavior
        // when stack overflows.
        static mut TCP_SERVER_RX_DATA: [u8; 1024] = [0; 1024];
        static mut TCP_SERVER_TX_DATA: [u8; 1024] = [0; 1024];
        let tcp_rx_buffer = TcpSocketBuffer::new(unsafe { &mut TCP_SERVER_RX_DATA[..] });
        let tcp_tx_buffer = TcpSocketBuffer::new(unsafe { &mut TCP_SERVER_TX_DATA[..] });
        TcpSocket::new(tcp_rx_buffer, tcp_tx_buffer)
    };
    
    let mut socket_set_entries: [_; 2] = Default::default();
    let mut socket_set = SocketSet::new(&mut socket_set_entries[..]);
    let server_handle = socket_set.add(server_socket);
    
    println!("Starting main loop...");

    let mut next_info = 0;
    loop {
        let time_ms = get_time();
        if time_ms > next_info {
            ethmac::info();
            next_info = next_info + 3000;
        }

        match iface.poll(&mut socket_set, time_ms) {
            Ok(()) | Err(Error::Exhausted) => (),
            Err(e) => println!("poll error: {}", e)
        }

    };

    cortex_m::interrupt::free(|cs| {
        let nvic = tm4c129x::NVIC.borrow(cs);
        nvic.enable(Interrupt::ADC0SS0);

        let mut loop_anode = LOOP_ANODE.borrow(cs).borrow_mut();
        let mut loop_cathode = LOOP_CATHODE.borrow(cs).borrow_mut();
        /*
        // ZJ-10
        let anode = 165.0;
        let cathode_bias = 50.0;
        let emission = 0.5e-3;
        */
        /*
        // ZJ-27
        let anode = 225.0;
        let cathode_bias = 25.0;
        let emission = 1.0e-3;
        */
        // ZJ-12
        let anode : f64 = 200.0;
        let cathode_bias : f64 = 50.0;
        let emission : f64 = 4.0e-3;
/*
        loop_anode.set_target(anode);
        loop_cathode.set_emission_target(emission);
        loop_cathode.set_bias_target(cathode_bias);
*/
    });

    let mut next_blink = 0;
    let mut next_info = 0;
    let mut led_state = true;
    let mut latch_reset_time = None;
    loop {
        board::process_errors();

        let time = get_time();

        if time > next_blink {
            led_state = !led_state;
            next_blink = time + 100;
            board::set_led(1, led_state);
        }

        if time > next_info {
            // FIXME: done in ISR now because of FPU snafu
            /*cortex_m::interrupt::free(|cs| {
                LOOP_CATHODE.borrow(cs).borrow().debug_print();
            });*/
            next_info = next_info + 300;
        }

        if board::error_latched() {
            match latch_reset_time {
                None => {
                    println!("Protection latched");
                    latch_reset_time = Some(time + 1000);
                }
                Some(t) => if time > t {
                    latch_reset_time = None;
                    cortex_m::interrupt::free(|cs| {
                        // reset PID loops as they have accumulated large errors
                        // while the protection was active, which would cause
                        // unnecessary overshoots.
                        LOOP_ANODE.borrow(cs).borrow_mut().reset();
                        LOOP_CATHODE.borrow(cs).borrow_mut().reset();
                        board::reset_error();
                    });
                    println!("Protection reset");
                }
            }
        }
    }
}

use tm4c129x::interrupt::TIMER0A;
extern fn timer0_a(_ctxt: TIMER0A) {
    cortex_m::interrupt::free(|cs| {
        let timer0 = tm4c129x::TIMER0.borrow(cs);
        timer0.icr.write(|w| w.tatocint().bit(true)); // Clear time-out interrupt bit

        // Increment 1ms timer
        let time = TIME.borrow(cs);
        time.set(time.get() + 1);
    });
}

use tm4c129x::interrupt::ADC0SS0;
extern fn adc0_ss0(_ctxt: ADC0SS0) {
    cortex_m::interrupt::free(|cs| {
        let adc0 = tm4c129x::ADC0.borrow(cs);
        if adc0.ostat.read().ov0().bit() {
            panic!("ADC FIFO overflowed")
        }
        adc0.isc.write(|w| w.in0().bit(true));

        let ic_sample  = adc0.ssfifo0.read().data().bits();
        let fbi_sample = adc0.ssfifo0.read().data().bits();
        let fv_sample  = adc0.ssfifo0.read().data().bits();
        let fd_sample  = adc0.ssfifo0.read().data().bits();
        let av_sample  = adc0.ssfifo0.read().data().bits();
        let fbv_sample = adc0.ssfifo0.read().data().bits();

        let mut loop_anode = LOOP_ANODE.borrow(cs).borrow_mut();
        let mut loop_cathode = LOOP_CATHODE.borrow(cs).borrow_mut();
        let mut electrometer = ELECTROMETER.borrow(cs).borrow_mut();
        loop_anode.adc_input(av_sample);
        loop_cathode.adc_input(fbi_sample, fd_sample, fv_sample, fbv_sample);
        electrometer.adc_input(ic_sample);

/*
        if time.get() % 300 == 0 {
            println!("");
            loop_anode.get_status().debug_print();
            loop_cathode.get_status().debug_print();
            electrometer.get_status().debug_print();
        }
*/
    });
}

#[used]
#[link_section = ".rodata.exceptions"]
pub static EXCEPTIONS: ExceptionHandlers = ExceptionHandlers {
    nmi: custom_handler,
    hard_fault: custom_handler,
    mem_manage: custom_handler,
    bus_fault: custom_handler,
    usage_fault: custom_handler,
    svcall: custom_handler,
    ..cortex_m::exception::DEFAULT_HANDLERS
};

#[used]
#[link_section = ".rodata.interrupts"]
pub static INTERRUPTS: InterruptHandlers = InterruptHandlers {
    TIMER0A: timer0_a,
    ADC0SS0: adc0_ss0,
    ..tm4c129x::interrupt::DEFAULT_HANDLERS
};

#[naked]
pub extern "C" fn custom_handler<T>(_token: T)
where
    T: Context,
{
    // This is the actual exception handler. `_sr` is a pointer to the previous
    // stack frame
    #[cfg(target_arch = "arm")]
    extern "C" fn handler(_sr: &StackedRegisters) -> ! {
        // printing the exception currently being serviced
        println!("EXCEPTION {:?} @ PC=0x{:08x} LR=0x{:08x} XPSR=0x{:08x}", Exception::current(), _sr.pc, _sr.lr, _sr.xpsr);
        println!("R0=0x{:08x} R1=0x{:08x} R2=0x{:08x} R3=0x{:08x} R12=0x{:08x}", _sr.r0, _sr.r1, _sr.r2, _sr.r3, _sr.r12);
        
        cortex_m::asm::bkpt();

        loop {}
    }

    match () {
        #[cfg(target_arch = "arm")]
        () => {
            unsafe {
                // "trampoline" to get to the real exception handler.
                asm!("mrs r0, MSP
                  ldr r1, [r0, #20]
                  b $0"
                     :
                     : "i"(handler as extern "C" fn(&StackedRegisters) -> !)
                     :
                     : "volatile");

                ::core::intrinsics::unreachable()
            }
        }
        #[cfg(not(target_arch = "arm"))]
        () => {}
    }
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum Exception {
    /// i.e. currently not servicing an exception
    ThreadMode,
    /// Non-maskable interrupt.
    Nmi,
    /// All class of fault.
    HardFault,
    /// Memory management.
    MemoryManagementFault,
    /// Pre-fetch fault, memory access fault.
    BusFault,
    /// Undefined instruction or illegal state.
    UsageFault,
    /// System service call via SWI instruction
    SVCall,
    /// Pendable request for system service
    PendSV,
    /// System tick timer
    Systick,
    /// An interrupt
    Interrupt(u8),
    // Unreachable variant
    #[doc(hidden)]
    Reserved,
}

impl Exception {
    /// Returns the kind of exception that's currently being serviced
    pub fn current() -> Exception {
        match unsafe { (*cortex_m::peripheral::SCB.get()).icsr.read() } as u8 {
            0 => Exception::ThreadMode,
            2 => Exception::Nmi,
            3 => Exception::HardFault,
            4 => Exception::MemoryManagementFault,
            5 => Exception::BusFault,
            6 => Exception::UsageFault,
            11 => Exception::SVCall,
            14 => Exception::PendSV,
            15 => Exception::Systick,
            n if n >= 16 => Exception::Interrupt(n - 16),
            _ => Exception::Reserved,
        }
    }
}

/// Registers stacked during an exception
#[derive(Clone, Copy, Debug)]
#[repr(C)]
pub struct StackedRegisters {
    /// (General purpose) Register 0
    pub r0: u32,
    /// (General purpose) Register 1
    pub r1: u32,
    /// (General purpose) Register 2
    pub r2: u32,
    /// (General purpose) Register 3
    pub r3: u32,
    /// (General purpose) Register 12
    pub r12: u32,
    /// Linker Register
    pub lr: u32,
    /// Program Counter
    pub pc: u32,
    /// Program Status Register
    pub xpsr: u32,
}

// panic print
#[no_mangle]
pub unsafe extern "C" fn rust_begin_unwind(
    _args: ::core::fmt::Arguments,
    _file: &'static str,
    _line: u32,
) -> ! {
    println!("panicked at {}:{}: {}", _file, _line, _args);

    println!("halting.");
    loop {}
}