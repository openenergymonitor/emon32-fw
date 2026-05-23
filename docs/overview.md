# Overview

The emonPi3 and emonTx6 are an upgrade of the existing OpenEnergyMonitor emonPi2 and emonTx5, sharing the same PCB profile and the same inputs for up to six Current Transformer (CT) sensors. The emonPi3 and emonTx6 focus on upgrading the microcontroller at the heart of the system, which provides an improved ADC, integrates more peripherals, and greatly increased compute performance.

When combined with an emonVs voltage sensor and CT sensors it provides a fully integrated system for monitoring home energy consumption, solar generation, EV charging, heat pumps, battery storage, and other applications.

Throughout the documentation, if there are steps and options specific to either the emonPi3 or emonTx6 these will be specifically described.

The emonPi3 and emonTx6 are fully open source, with both the [hardware](https://github.com/openenergymonitor/emon32) and the [firmware](https://github.com/openenergymonitor/emon32-fw) available.

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
- Fully supported in [ESPHome](https://esphome.io/components/sensor/emontx/) for [Home Assistant](https://www.home-assistant.io/) integration

## Selecting the emonPi3 or the emonTx6

The emonPi3 and emonTx6 share a common hardware design and the same firmware.

The emonPi3 is fitted with a Raspberry Pi running emonCMS to handle the energy monitoring data, connected external sensors, and other OpenEnergyMonitor products such as additional emonTx units and emonTH sensors. The radio module operates as a receiver in the emonPi3.

The emonTx6 sends data using any of the following:

- An RFM69 radio module, typically to an OpenEnergyMonitor base.
- Serial through the USB-C port.serial.
- WiFi using an [emonWiFi adapter](https://github.com/openenergymonitor/emonwifi).

The following are the common options:

- Setting up a new OpenEnergyMonitor system: **emonPi3**
- Expanding an existing OpenEnergyMonitorSystem: **emonTx6**
- Use without the radio module (e.g. Home Assistant, MQTT): **emonTx6** with [emonWiFi](https://github.com/openenergymonitor/emonwifi) expander
