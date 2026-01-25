# Firmware

```{tip}
Most users do not need to compile their own firmware. Pre-built firmware can be downloaded from GitHub.
```

## Updating firmware

### emonPi3

#### Web Interface

The easiest way to update the emonPi3's firmware is to use the firmware upload tool.

1. In your local emonPi3 web interface, navigate to `Setup > Admin > Update > Firmware`.
2. Select serial port `ttyS0` and then select `emonPi3` from hardware.

#### Command Line

To update the emonPi3 using the command line, you can use the [BOSSA](https://github.com/shumatech/BOSSA) application. The following steps are taken:

1. Open a serial connection using, for example, `minicom` to `/dev/ttyS0` (`minicom -D /dev/ttyS0 -b 115200`)
2. Enter 'e' and then press Enter.
3. You will be prompted to reboot to enter the bootloader. Any unsaved configuration changes at this point will be lost. Press 'y' to continue, or any other key to cancel.
4. The emonPi3's LED will slowly pulse green indicating it is in the bootloader.
5. To check the bootloader is responsive, run `bossac -p /dev/ttyS0 -i`.
6. To upload the compiled firmare, run `bossac -p /dev/ttyS0 -e -w -v -R --offset=0x2000 path/to/bin`.

If BOSSA is not installed, it can be installed by:

```{bash}
git clone https://github.com/shumatech/BOSSA
cd BOSSA
make bossac
```

### emonTx6

To update the emonTx6's firmware from a desktop, the following steps are taken:

1. Connect a USB cable to the emonTx6's USB-C socket.
2. Open a serial connection using, for example, `minicom` (e.g. `minicom -D /dev/ttyACM0 -b 115200`) or the Arduino serial monitor.
3. Enter 'e' and then press Enter.
4. You will be prompted to reboot to enter the bootloader. Any unsaved configuration changes at this point will be lost. Press 'y' to continue, or any other key to cancel.
5. The emonTx6's LED will slowly pulse red and a drive called `EMONBOOT` will appear in the file manager.
6. Drag and drop the firmware image ending `.uf2` to the `EMONBOOT` folder.
7. The emonTx6 will reboot and enter the application.

You can also update from a command line environment in Linux with the following steps:

1. Connect a USB cable to the emonTx6's USB-C socket.
2. In `emon32-fw/scripts`, run `flash-emontx6.sh`.

By default, this will use a locally built UF2. If you want to use a different UF2 file, for example downloaded from OEM, you should run:

```{bash}
flash-emontx6.sh --uf2 <path/to/file>
```

### Changing the bootloader

When configured as an emonPi3, the bootloader is accessed through the internal serial port. When configured as an emonTx6, the bootloader is accessed through the USB-C port. To change between the two modes, the bootloader needs to be updated.

#### To change from an emonPi3 to an emonTx6

1. Open a serial connection using, for example, `minicom` to `/dev/ttyS0` (`minicom -D /dev/ttyS0 -b 115200`)
2. Enter 'e' and then press Enter.
3. You will be prompted to reboot to enter the bootloader. Any unsaved configuration changes at this point will be lost. Press 'y' to continue, or any other key to cancel.
4. The emonPi3's LED will slowly pulse green indicating it is in the bootloader.
5. To check the bootloader is responsive, run `bossac -p /dev/ttyS0 -i`.
6. To upload the compiled firmare, in the `emon32-fw` folder run `bossac -p /dev/ttyS0 -e -w -v -R --offset=0x2000 bin/bootloaders/change-bootloader-usb.bin`.
7. After connecting a cable to the USB-C port, you can now upload the firmware as described above, skipping steps 2-4 as it will already be in the bootloader.

#### To change from an emonTx6 to an emonPi3

1. Connect a USB cable to the emonTx6's USB-C socket.
2. Open a serial connection using, for example, `minicom` (e.g. `minicom -D /dev/ttyACM0 -b 115200`) or the Arduino serial monitor.
3. Enter 'e' and then press Enter.
4. You will be prompted to reboot to enter the bootloader. Any unsaved configuration changes at this point will be lost. Press 'y' to continue, or any other key to cancel.
5. The emonTx6's LED will slowly pulse red and a drive called `EMONBOOT` will appear in the file manager.
6. Drag and drop `bin/bootloaders/bootloader-change-uart.uf2` to `EMONBOOT`.
7. You can now upload the firmware as described above, skipping steps 1-4 as it will already be in the bootloader.

## Development

Contributions are welcome. Small PRs can be accepted at any time. Please get in touch before making large changes to see if it's going to fit. This is an open source project, so PRs and issues may not be addressed quickly. This is not a comment on the quality of the contribution, and you are free to fork the project at any point.

```{tip}
Run the install-hooks.sh script to enable the pre-commit hooks, which include autoformatting. A clang-format pattern is included. You may need to install clang-format and ruff using your OS's package manager.
```

### How to compile firmware

There is a single unified firmware for the emonPi3 and emonTx6. The firmware is self contained, with no external libraries required and does not require any frameworks like Arduino or Platform.io.

Compiling the firmware requires the the [Arm gcc toolchain](https://developer.arm.com/Tools%20and%20Software/GNU%20Toolchain). This also may be available as a package in your distribution. The Makefile is for Arm Cortex-M0+ based microcontrollers, specifically the Microchip ATSAMD21J17 ([datasheet](https://www.microchip.com/en-us/product/ATSAMD21J17), [errata](https://ww1.microchip.com/downloads/en/DeviceDoc/SAMD21-%20Family-Silicon-%20Errata-and-DataSheet-Clarification-DS80000760C.pdf)).

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

To update from the repository, in the `emon32-fw` folder, run:

```{bash}
git pull
make clean
make -j
```

### Flash and debug

The emonPi3/Tx6 exposes a standard Arm Cortex-M 10-pin SWD connector. You can flash and debug the microcontroller using a CMSIS-DAP compatible debugger using `openocd` and `arm-none-eabi-gdb`. There is no requirement to use the vendor IDE and tools.
