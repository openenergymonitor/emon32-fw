# Overview

The emonPi3 and emonTx6 are an upgrade of the existing OpenEnergyMonitor emonPi2 and emonTx5, sharing the same PCB profile and the same inputs for up to six Current Transformer (CT) sensors. The emonPi3 and emonTx6 focus on upgrading the microcontroller at the heart of the system, which provides an improved ADC, integrates more peripherals, and greatly increased compute performance.

When combined with an emonVs voltage sensor and CT sensors it provides a fully integrated system for monitoring home energy consumption, solar generation, EV charging, heat pumps, battery storage, and other applications.

Throughout the documentation, if there are steps and options specific to either the emonPi3 or emonTx6 these will be specifically described.

The emonPi3 and emonTx6 share the same firmware. Most users will not need to compile their own firmware.

The emonPi3 and emonTx6 are fully open source, and you can find the hardware files [here](https://github.com/openenergymonitor/emon32).

## Key features

- 6x clip-on CT current sensor inputs (333 mV voltage output or, optionally, current output)
- emonVS precision voltage sensor and power input
- Support for single and 3-phase voltage
- Continuous sampling of real and reactive power
- Cumulative energy consumption, persisted over reboots (this is stored every 200 Wh)
- USB serial interface for configuration and output
- ISM band 433.92 MHz radio transceiver
- 40pin Raspberry Pi GPIO header
- Wall-mount aluminium enclosure

## Key Components

- Microcontroller: [Microchip ATSAMD21J17](https://www.microchip.com/en-us/product/atsamd21j17)
- Precision voltage reference: [Microchip MCP1501](https://www.microchip.com/en-us/product/MCP1501)
- Radio module: [RFM69CW](https://www.hoperf.com/modules/rf_transceiver/RFM69CW.html) @ 433.92 MHz

## Accuracy

The emonPi3 builds on the same improvements as the emonPi2. The precision voltage reference is used continuously as the ADC reference, rather than for calibration and the ADC is pseudo-differential offering improved performance.
