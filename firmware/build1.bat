del target\thumbv7em-none-eabihf\release\ionpak-firmware
del src\version.ascii
echo Compiled at: %date%-%time% >src\version.ascii
xargo build --release --features divsclk 2>1
arm-none-eabi-objcopy.exe -O binary target\thumbv7em-none-eabihf\release\ionpak-firmware image.bin