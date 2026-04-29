Script to flash combined UART bootloader and firmware using Black Magic Debugger

1. Convert `.bin` to `.elf` using `bin2elf.sh`
2. Flash using `flash-combined-uartboot-fw.sh`

Edit config file `elf_bmp_flash.gdb` to set serial port for Black Magic 

Requires ARM toolchain and gdb debugger

- To intall ARM toolchain on Ubuntu follow: https://developer.arm.com/documentation/110477/221/Installation

- To intall gdb debugger:

`sudo apt install gdb-multiarch`
`sudo ln -s /usr/bin/gdb-multiarch /usr/bin/arm-none-eabi-gdb`

