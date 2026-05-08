# Firmware

```{tip}
Most users do not need to compile their own firmware. Pre-built firmware can be downloaded from GitHub.
```

## Updating firmware

### emonPi3

**The emonPi3 runs a UART bootloader, it can only be updated via a RaspberryPi using the GPIO UART. Unlike the emonTx6 which runs  USB bootloader which can be updated via USB**

#### Web Interface

The easiest way to update the emonPi3's firmware is to use the firmware upload tool.

1. In your local emonPi3 web interface, navigate to `Setup > Admin > Update > Firmware`.
2. Select serial port `ttyAMA0` and then select `emonPi3` from hardware.

#### emonPi Command Line

To update the emonPi3 using the command line, you can use the [BOSSA](https://github.com/shumatech/BOSSA) application. The following steps are taken:

- Connect to the emonPi3 via SSH, it's disabled by default, use the push button & LCD to enabled it, see [emonSD download page](https://docs.openenergymonitor.org/emonsd/download.html) for credentials
- Open a serial connection using, for example, `minicom` to `/dev/ttyAMA0` (`minicom -D /dev/ttyAMA0 -b 115200`)
- Enter 'e' and then press Enter.
- You will be prompted to reboot to enter the bootloader. Any unsaved configuration changes at this point will be lost. Press 'y' to continue, or any other key to cancel.
- Exit minicom using `CTRL + X` then `Y`
- The emonPi3's LED will slowly pulse green indicating it is in the bootloader.
- To check the bootloader is responsive, run `bossac -p /dev/ttyAMA0 -i`.
- To upload the compiled firware, run `bossac -p /dev/ttyAMA0 -e -w -v -R --offset=0x2000 path/to/.bin`.

Pre-compiled `.bin` firmware for the emonPi3 can be downloaded from the [emon32-fw releases page](https://github.com/openenergymonitor/emon32-fw/releases)

- If BOSSA is not installed, it can be installed by:
- `git clone https://github.com/openenergymonitor/BOSSA`
- `cd BOSSA`
- `make bossac`

### emonTx6

**The emonTx6 runs a USB bootloader, it can be updated using the USB port**

*The emonTx6 shares the [same firmware](https://github.com/openenergymonitor/emon32-fw/releases) as the emonPi3, they also share the same PCB. The unit auto-detects when a RaspberryPi is not connected and enables emonTx6 function i.e transmitting via RFM*

To update the emonTx6's firmware:

The emonTx6 firmware can be updated using a PC or laptop, Windows, Linux and Mac OS are supported. 

- Connect the emonTx6 to a computer using a USB cable 

#### Enable bootloader mode 

Either by 
- Double pressing the button the emonTx6/emonPi2 PCB
Or
- Send `e` via serial e.g `minicom` (e.g. `minicom -D /dev/ttyACM0 -b 115200`) or the Arduino serial monitor.

Once in bootloader mode:
- The emonTx6's LED will slowly pulse red and a disk drive called `EMONBOOT` will appear in the file manager.
- Drag and drop the firmware image ending `.uf2` to the `EMONBOOT` folder.
- The emonTx6 will reboot and run the new firmware

## Development

Contributions are welcome. Small PRs can be accepted at any time. Please get in touch before making large changes to see if it's going to fit. This is an open source project, so PRs and issues may not be addressed quickly. This is not a comment on the quality of the contribution, and you are free to fork the project at any point.

```{tip}
Run the install-hooks.sh script to enable the pre-commit hooks, which include autoformatting. A clang-format pattern is included. You may need to install clang-format and ruff using your OS's package manager.
```

### How to compile firmware

There is a single unified firmware for the emonPi3 and emonTx6. The firmware is self contained, with no external libraries required and does not require any frameworks like Arduino or Platform.io.

Compiling the firmware requires the the [Arm gcc toolchain](https://developer.arm.com/Tools%20and%20Software/GNU%20Toolchain). This also may be available as a package in your distribution. The Makefile is for Arm Cortex-M0+ based microcontrollers, specifically the Microchip ATSAMD21J17 ([datasheet](https://www.microchip.com/en-us/product/ATSAMD21J17), [errata](https://ww1.microchip.com/downloads/en/DeviceDoc/SAMD21-%20Family-Silicon-%20Errata-and-DataSheet-Clarification-DS80000760C.pdf)).

To install the ARM toolchain on Linux see: https://developer.arm.com/documentation/110477/221/Installation

Ensure the toolchain is available on the path by running:

```{bash}
arm-none-eabi-gcc --version
```

Clone the `emon32-fw` repo and compile the firmware:

```{bash}
git clone https://github.com/openenergymonitor/emon32-fw
cd emon32-fw
make -j
```

Images in `.bin`, `.hex`, `.elf`, and `.uf2` formats will be in the `bin/` folder. The image names include the version and the git commit hash for traceability.

### Flash and debug

The emonPi3/Tx6 exposes a standard Arm Cortex-M 10-pin SWD connector. You can flash and debug the microcontroller using a CMSIS-DAP compatible debugger using `openocd` and `arm-none-eabi-gdb`. There is no requirement to use the vendor IDE and tools.
