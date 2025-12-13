# Configuration

```{note}
The emonPi3 and emonTx6, by default, use 433.92 MHz for RF communication, rather than 433.00 MHz, to stay within the ISM regulated band. If you are using OpenEnergyMonitor emonTH2, emonPi2, emonTx4, or emonTx5 units you may need to enable 433.00 MHz compatibility mode.
```

## Through EmonCMS

The emonPi3 is usually pre-configured in the shop as part of the order process but can be re-configured at any point using the Serial Config tool available from the emonPi3 local EmonCMS interface.

- Navigate to `Setup > Admin > Serial Config`
- Click on `Stop EmonHub` to temporarily stop the EmonHub service while we perform calibration.
- Select serial port `/dev/ttyAMA0` and click `Connect`.
- After a couple of seconds the emonPi3 will print out its current configuration which will populate the interface (if it does not do this type `l` and click `Send` to reload the calibration details from the emonPi3 measurement board).
- Adjust any settings you need to change.
- Click on `Save Changes` to ensure that the new configuration is recorded such that it persists when you power cycle the board.
- When finished, click on `Stop Serial` to disconnect the serial configuration tool and then `Start EmonHub` to restart the EmonHub service.

![emonPi3_serial_config.png](img/emonPi3_serial_config.png)

## Directly via serial

It is possible to configure the emonPi3/Tx6 directly through the USB serial port or UART using `minicom` or other similar tool. When using a UART, the settings are 115200, 8N1, `\r\n` line ending.

The following details the available commands and their function.

- **?** show this text again
- **a\<n\>** sets the assumed RMS voltage (V)
- **b** pack the configuration into JSON and send on serial for backup
- **c\<n\>** n = 0 for OFF, n = 1 for ON, enable log to serial
- **d\<x.x\>** a floating point number for the data logging period (s)
- **e** enter the bootloader
- **f\<n\>** the line frequency, normally either 50 or 60 (Hz)
- **g\<n\>** sets the network group, OpenEnergyMonitor is normally 210
- **j\<n\>** n = 0 for OFF, n = 1 for ON, use JSON format in serial data output
- **k\<x\> \<a\> \<y.y\> \<z.z\> v1 v2**
  - Calibrate an analogue input channel:
  - x = a single numeral: 1-3 = voltage calibration, 4 = ct1 calibration, 5 = ct2 calibration, etc
  - a : a = 0 for disabled, a = 1 for enabled, channel active
  - y.y : a floating point number for the voltage/current calibration constant
  - z.z : a floating point number for the phase calibration for this CT (z is not needed, or ignored if supplied, when x = 0)
  - v1 : the voltage channel associated with this CT
  - v2 : the second voltage channel associated with this CT for 3-phase only
  - e.g. k1 1 101.3
  - k4 1 20.0 3.20 1 1
- **l** list the settings
- **m\<v\> \<w\> \<x\> \<y\> \<z\>** OneWire and pulse configuration:
  - v : channel index
  - w : 0 for DISABLED, 1 for ENABLED. If DISABLED, no other arguments needed.
  - x : function selection. \[b, f, r\]: pulse, o: OneWire
  - y : 0 for no pull up, 1 for pull up (ignored for OneWire)
  - z : minimum pulse period (ms) (ignored for OneWire)
- **n\<n\>** sets the node ID \[1..60\]
- **p\<n\>** sets the RF power level
- **r** restore default settings
- **s** save settings to EEPROM
- **t** trigger a report on the next mains cycle
- **v** show firmware and board information
- **w\<n\>** n = 0 for OFF, n= 1 for ON, enable wireless transmission
- **x\<n\>** n = 0 for 433.92 MHz, n = 1 for 433.00 MHz compatibility
- **z** zero energy accumulators
