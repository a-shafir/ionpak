#![feature(asm, used, const_fn, core_float, use_extern_macros, naked_functions, core_intrinsics)]
#![no_std]

extern crate cortex_m;
extern crate cortex_m_rt;
extern crate tm4c129x;

use core::cell::{Cell, RefCell};
use cortex_m::exception::Handlers as ExceptionHandlers;
use cortex_m::interrupt::Mutex;
use tm4c129x::interrupt::Interrupt;
use tm4c129x::interrupt::Handlers as InterruptHandlers;
use cortex_m::ctxt::Context;

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

fn main() {
    board::init();

    //enabling FPU in the priveleged mode
    unsafe { asm!("svc 1")};

    println!("Ready.");

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

        let time = TIME.borrow(cs);
        time.set(time.get() + 1);

        if time.get() % 300 == 0 {
            println!("");
            loop_anode.get_status().debug_print();
            loop_cathode.get_status().debug_print();
            electrometer.get_status().debug_print();
        }
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
    svcall: svcall_handler,
    ..cortex_m::exception::DEFAULT_HANDLERS
};

#[used]
#[link_section = ".rodata.interrupts"]
pub static INTERRUPTS: InterruptHandlers = InterruptHandlers {
    ADC0SS0: adc0_ss0,
    ..tm4c129x::interrupt::DEFAULT_HANDLERS
};

#[naked]
pub extern "C" fn svcall_handler<T>(_token: T)
where
    T: Context,
{
    // This is the actual exception handler. `_sr` is a pointer to the previous
    // stack frame
    #[cfg(target_arch = "arm")]
    extern "C" fn handler(_sr: &StackedRegisters) {
        // Verifying the svcall exception
        if 11 == unsafe { (*cortex_m::peripheral::SCB.get()).icsr.read() } as u8 {
            // getting the SVC code (https://stackoverflow.com/questions/16156404/writing-own-svc-calls-arm-assembly)
            let scode = unsafe { *((_sr.pc-2) as *mut u8) };
            match scode {
                // asm SVC 1 as FPU enable
                1 => {
                    let scb = tm4c129x::SCB.get();
                    unsafe { (*scb).cpacr.write(0x00F00000u32) }
                }
                _ => {
                }
            }
        }
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
                     : "i"(handler as extern "C" fn(&StackedRegisters))
                     :
                     : "volatile");

                ::core::intrinsics::unreachable()
            }
        }
        #[cfg(not(target_arch = "arm"))]
        () => {}
    }
}

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
        
        let points_at = unsafe { *((_sr.pc-2) as *mut u8) };

        println!("X=0x{:08x}", points_at);

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
    println!("panicked at {}:{}", _file, _line);

    #[cfg(target_arch = "arm")]
    asm!("bkpt" :::: "volatile");

    loop {}
}
