# _emon32_ Firmware

This describes the firmware provided for the [_emon32_ energy monitoring](https://github.com/awjlogan/emon32) system. The software is modular, and should be easily portable to other microcontrollers and implementations.

This firmware is intended to be used with the [OpenEnergyMonitor](https://openenergymonitor.org) platform. Hardware systems are available directly from them.

## Getting in contact

### Problems

Issues can be reported:

- As a [GitHub issue](https://github.com/awjlogan/emon32-fw/issues)
- On the [OpenEnergyMonitor forums](https://community.openenergymonitor.org/)

Please include as much information as possible (run the `v` command on the serial link), including at least:

- The emon32 hardware that you using and the emon32-fw version (run the 'v' command on the serial link)
- All settings (run the `l` command on the serial link)
- A full description, including a reproduction if possible, of the issue

### Contributing

Contributions are welcome! Small PRs can be accepted at any time. Please get in touch before making _large_ changes to see if it's going to fit before spending too much time on things.

> [!TIP]
> Run the `install-hooks.sh` script to install the pre-commit hook autoformatter. A [clang-format](https://clang.llvm.org/docs/ClangFormat.html) autoformat pattern is included in the repository.  You may need to install `clang-format` using your OS's package manager. You may need to install [ruff](https://docs.astral.sh/ruff/) using your OS's package manager.

> [!NOTE]
> Please bear in mind that this is an open source project and PRs and enhancements may not be addressed quickly, or at all. This is no comment on the quality of the contribution, and please feel free to fork as you like!

## Functional Description

### Default Configuration

Most default parameters can be adjusted in `emon32.h`.

> [!NOTE]
> This section assumes you are using the [emonPi3](https://github.com/awjlogan/emon32).

The following default values are set out of the box.

| Parameter           | Value     | Comment                                   |
|---------------------|-----------|-------------------------------------------|
| DELTA_WH_STORE_DEF  | 200       | Minimum accumulation in Wh before saving  |
| NODE_ID_DEF         | 17        | Node ID used.                             |
| GROUP_ID_DEF        | 210       | Fixed for OpenEnergyMonitor               |
| MAINS_FREQ_DEF      | 50        | Mains frequency in Hz                     |
| REPORT_TIME_DEF     | 9.8       | Time between reports in seconds           |
| NUM_CT_ACTIVE_DEF   | 6         | Only onboard CT sensors in use            |

OPA1 is configured as a pulse input and OPA2 is configured as a OneWire input.

### Version information

The firmware version numbering follows [semantic versioning](https://semver.org/). That is, for version `X.Y.Z`:

- `X` : major version with no guaranteed backward compatibility with previous major versions.
- `Y` : minor version where any added functionality has backward compatibility.
- `Z` : improvements and bug fixes.

Any firmware with `X == 0` is considered unstable and subject to change without notice.

> [!NOTE]
> Build information, including compiler version and commit, is generated during the build process and included in the binary.

### USB Serial Connection

The emonPi3 will enumerate as a normal serial device and show as **emonPi3**.

### Hardware serial connection

A dedicated UART is used for debug, configuration, and data transmission. It has the following UART configuration:

- 115200 baud
- 8N1

It is available on:

- Raspberry Pi:
  - GPIO 14 (UART TX _from_ Raspberry Pi)
  - GPIO 15 (UART RX _to_ the Raspberry Pi)

### Run time configuration

The _emon32_ firmware is compatible with the OpenEnergyMonitor [emonPi2 configuration](https://docs.openenergymonitor.org/emonpi2/configuration.html) options, which can be accessed through the debug serial link.

> [!NOTE]
> All options can be listed by entering `?`.

The following options are added:

|Command      |Definition                                             |
|-------------|-------------------------------------------------------|
|b            |Print the configuration as JSON on serial              |
|o&lt;_x_&gt; |Auto calibrate CT lead for channel _x_                 |
|t            |Trigger a data set processing event                    |
|v            |Print firmware and board information                   |
|x&lt;_n_&gt; |Set 433.00 MHz compatibility, _n_ = 1                  |

### Data acquisition

The ADC is triggered by a dedicated timer (`TIMER_ADC`) with no intervention from the processor. Data are accumulated by DMA into a ping-pong buffer - when one sample set is being processed, another is being captured in the background.

Raw data from the ADC are downsampled (if configured) and then injected into the energy and power calculation routines. As there is a single ADC, CT values are interpolated between the appropriate voltage samples.

### Data transmission

When a full report is ready, the following actions take place:

- 1 s _before_ the report is due, any temperature sensors present are triggered to record a value.
  - The DS18B20 temperature sensor takes 750 ms to take a measurement in the default 12bit mode.
  - A report can also be triggered with the command `t` on the serial link.
- At the report time, the following values are calculated:
  - Power for each CT.
  - Accumulated energy for each CT.
  - Mains frequency.
  - Power factor
- Data are packed into two formats:
  - Key:value pairs for serial transmission.
  - Packed structure for transmission by the RFM module.
- Data are sent over the configured interface.
  - It is configurable whether data are always echoed on the debug console.

> [!WARNING]
> The RFM69 transmitter will be damaged if it is run at maximum power without an antenna.

## Compiling and uploading

### Compiling

Compiling the firmware requires the the [Arm gcc toolchain](https://developer.arm.com/Tools%20and%20Software/GNU%20Toolchain) (may be available as a package in your distribution). The Makefile is for a Cortex-M0+ based microcontrollers, specifically the Microchip ATSAMD21J17 ([datasheet](https://www.microchip.com/en-us/product/ATSAMD21J17), [errata](https://ww1.microchip.com/downloads/en/DeviceDoc/SAMD21-%20Family-Silicon-%20Errata-and-DataSheet-Clarification-DS80000760C.pdf)).

`sudo apt install gcc-arm-none-eabi`

> [!NOTE]
> To find which version, if any, of the toolchain is on your path, enter `arm-none-eabi-gcc --version`. You can set the path to a compiler off your path by setting the `TC_PATH` variable in `Makefile`.

To build the firmware:

  `> make -j`

In `bin/`, the following binary files will be generated:

- `emon32-vX.Y.Z-(commit[-dirty]).bin`
- `emon32-vX.Y.Z-(commit[-dirty]).elf`
- `emon32-vX.Y.Z-(commit[-dirty]).hex`
- `emon32-vX.Y.Z-(commit[-dirty]).uf2`

The `-dirty` tag (if present) indicates that there are uncommitted changes when the binaries are built.

### Uploading

The emonPi3 comes preloaded with a [UF2 bootloader](https://microsoft.github.io/uf2/). This allows the firmware to be updated over USB without any specialised hardware. The following procedure should be used:

1. Connect to the host computer through the USB-C port.
2. Quickly double press the `RESET` button.
   - The LED will pulse slowly to indicate it has entered bootloader mode,
   - The drive `EMONBOOT` will appear on the host computer.
3. Copy `bin/emon32-vX.Y.Z-(commit[-dirty]).uf2` to the `EMONBOOT` drive. The board will reset and enter the main program.

#### Updating the bootloader

The bootloader can be updated by the same process as for uploading normal firmware. This will not normally need to be done.

#### Restoring the bootloader

If, for whatever reason, the bootloader is corrupted it can be flashed back to the board with the included binary. The bootloader binary is included in `bin/bootloader-emonPi3-v*` as a `.bin` and `.elf` file. You will need a suitable SWD programmer to do this.

## Modifications

### Helper scripts

> [!NOTE]
> A Python virtual environment shoulde be setup by running `python3 -m venv venv && source venv/bin/activate && pip3 install -r requirements.txt` in `./scripts/`.

- `a2l.sh`: converts a hex address to a file line. Usage: `a2l.sh <address>`
- `elf-size.sh`: this script decomposes the built `.elf` file into functions with their sizes.
- `filter.py`: generates the half band filter coefficients. Described in "Digital Filter".

### Floating point support

The Cortex-M0+ does not have a hardware floating point unit, so all floating point operations are done in software. The gcc built in floating point functions are quite large and slow, and have been replaced with the [Qfplib](https://www.quinapalus.com/qfplib.html) library. All floating point operations, including type conversions, should use these functions.

### Compile Time Configuration

Below is a list of the compile time options, grouped by location. The value for emonPi3 is given in bold, and the allowed range in general is given:

- `src/board_def.h`; values mostly constrained by the physical arrangement.
  - **NUM_CT**: The number of CT channels. These must be contiguous from the lowest index above the voltage channels, but can be less than the number of physical channels. **12** \[1..12\]
  - **NUM_V**: The number of physical voltage channels. Due to the ADC and software architecture, this must always be the physical number of voltage channels even when only using a single phase. **3**, \[1..3\]
  - **SAMPLE_RATE**: Sample rate, in Hz, for each channel _before_ any downsampling. This is typically restricted by the -3dB point of the anti-aliasing filter. The total ADC sampling rate is (**SAMPLE_RATE** \* (**NUM_V** + **NUM_CT**)). **4800**, \[4800\]

### Digital filter

The base configuration has an oversampling factor of 2X to ease the anti-aliasing requirments. Samples are then low pass filtered and reduced to _f/2_ with a half band filter. Filter coefficients can be generated using the **filter.py** script (_./scripts/filter.py_). This generates a header file (`emon_CM_coeffs.h`) which can be copied to the `<emon32-fw>/src/` folder for use.

> [!NOTE]
> It is recommended to use an odd number of taps, as the filter can be made symmetric in this manner.

You will need [**SciPy**](https://scipy.org/) and [**Matplotlib**](https://matplotlib.org/) to use the filter designer. These are included in the virtual environment described in "Helper Scripts".

### Assertions

Assertions are [implemented](https://interrupt.memfault.com/blog/asserts-in-embedded-systems) by the **EMON32_ASSERT(_condition_)** macro. The microcontroller will enter a breakpoint when an assertion fails and the PC is stored in the `g_assert_info` variable. The PC is used to find the file and line where the assertion failed using `arm-none-eabi-addr2line`.

### Tests

Test programs are available for the `emon_CM` and `eeprom` modules, abstracted from the underlying hardware. In _./tests_, run `make cm` or `make eeprom` followed by `./cm.test` or `./eeprom.test` respectively.

## Hardware Description

### Peripherals (SAMD21)

The following table lists the peripherals used in the SAMD21.

|Peripheral       | Alias           | Description                   | Usage                             |
|-----------------|-----------------|-------------------------------|-----------------------------------|
|ADC              |                 |Analog-to-digital converter    |Acquire analog signals             |
|DMAC             |                 |DMA Controller                 |ADC->buffer and UART TX            |
|EIC              |                 |External interrupt controller  |External device sense              |
|EVSYS            |                 |Event System                   |Asynchronous event handling        |
|PORT             |                 |GPIO handling                  |                                   |
|SERCOM2          |SERCOM_UART      |UART                           |Configuration and data UART        |
|SERCOM3          |SERCOM_I2CM      |I2C (internal)                 |I2C for internal peripherals       |
|SERCOM4          |SERCOM_SPI       |SPI                            |Drives RFM module                  |
|SERCOM5          |SERCOM_I2M_EXT   |I2C (external)                 |Drives display module              |
|TC3              |TIMER_ADC        |Timer/Counter (16bit)          |ADC sample trigger                 |
|TC4+5            |TIMER_DELAY      |Timer/Counter (32bit)          |Delay timer                        |
|TC6+7            |TIMER_TICK       |Timer/Counter (32bit)          |Global time (micro/millisecond)    |
|USB              |                 |USB interface                  |USB CDC (serial) emulation         |

### Designing a new board

The files `/src/board_def.h` and `/src/board_def.c` contain options for configuring the microcontroller for a given board. Pin mappings and peripheral usage will need to be adjusted to your design.

### Porting to different microcontroller

Within the top level loop, there are no direct calls to low level hardware. You must provide functions that handle the hardware specific to the microcontroller you are using.

All peripheral drivers are in header/source pairs named **driver_\<PERIPHERAL\>**. For example, the ADC driver is in **driver_ADC.\***. If you are porting to a new microcontroller, you will need to provide implementations of all the functions exposed in **driver_\<PERIPHERAL\>.h** and any internal functions within **driver_\<PERIPHERAL\>.c**. If your microcontroller does not support a particular function (for example, it doesn't have a DMA), then either no operation or an alternative must be provided.

You will also need to ensure that the vendor's headers are included and visible to the compiler.

## Acknowledgements

### Third party libraries and tools

- [mcu-starter-projects](https://github.com/ataradov/mcu-starter-projects) - good starting point for build chains for microcontrollers.
- [printf](https://github.com/eyalroz/printf) - embedded `printf` implementation.
- [pid.codes](https://pid.codes) - reuse of a retired USB VID for open source projects.
- [Qfplib](https://www.quinapalus.com/qfplib.html) - soft floating point library for Arm Cortex-M0.
- [RFM69](https://github.com/LowPowerLab/RFM69) - RFM69 driver from Low Power Labs used as reference.
- [SSD1306 library](https://github.com/Matiasus/SSD1306/tree/master) - used as a reference for this implementation.
- [tinyUSB](https://github.com/hathach/tinyusb) - USB library (derived from [_236aa9622_](https://github.com/hathach/tinyusb/commit/236aa9622a31b8c4727c98c6d683cee011fb8f9b)).
- [Using Asserts in Embedded Systems](https://interrupt.memfault.com/blog/asserts-in-embedded-systems) - custom assertions from _Interrupt by Memfault_.
- [Wintertools](https://github.com/https://github.com/wntrblm/wintertools) - various build and linker scripts from Winterbloom.

### Others

- Glyn Hudson @ [OpenEnergyMonitor](https://openenergymonitor.org/)
- Rob Wall @ [OpenEnergyMonitor forums](https://community.openenergymonitor.org/) for discussions around all aspects of energy monitoring.
- Trystan Lea @ [OpenEnergyMonitor](https://openenergymonitor.org/)
