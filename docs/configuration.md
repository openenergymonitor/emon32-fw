# Configuration

```{note}
The emonPi3 and emonTx6, by default, use 433.92 MHz for RF communication, rather than 433.00 MHz, to stay within the ISM regulated band. If you are using OpenEnergyMonitor emonTH2, emonPi2, emonTx4, or emonTx5 units you should update their firmware to support 433.92 MHz, or enable 433.00 MHz compatibility mode in the emonPi3/Tx6.
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

The following table details the available commands and their function.

| Command | Description |
|---------|-------------|
| **?** | Show help text |
| **a\<n>** | Set the assumed RMS voltage as integer (when no AC voltage detected)<br>Example: `a230` sets assumed voltage to 230V |
| **b** | Backup configuration to serial |
| **c\<n>** | Log to serial output<br>- `c0`: Disable serial logging<br>- `c1`: Enable serial logging |
| **d\<x.x>** | Set data log period in seconds<br>Example: `d10.0` sets logging period to 10 seconds |
| **e** | Enter bootloader mode for firmware updates |
| **f\<n>** | Set line frequency in Hz<br>- `f50`: 50 Hz (Europe, UK, etc.)<br>- `f60`: 60 Hz (US, Canada, etc.) |
| **g\<n>** | Set network group for RF communication (default = 210)<br>Example: `g210` |
| **j\<n>** | JSON serial format<br>- `j0`: Disable JSON format<br>- `j1`: Enable JSON format |
| **k\<x> \<a> \<y.y> \<z.z> \<v1> \<v2>** | Configure an analog input (voltage or current)<br>Parameters:<br>- `x`: Channel number (1-3 = Voltage; 4+ = CT)<br>- `a`: Active status (0 = DISABLED, 1 = ENABLED)<br>- `y.y`: V/CT calibration constant<br>- `z.z`: CT phase calibration value (degrees)<br>- `v1`: CT voltage channel 1 (reference)<br>- `v2`: CT voltage channel 2 (for L-L loads)<br>Example: `k4 1 90.0 1.5 1 1` |
| **l** | List current settings (displays all configuration) |
| **m\<v> \<w> \<x> \<y> \<z>** | Configure a OneWire/pulse input<br>Parameters:<br>- `v`: Channel index<br>- `w`: Active status (0 = DISABLED, 1 = ENABLED)<br>- `x`: Function select<br>&nbsp;&nbsp;- `b`: Both edges (pulse)<br>&nbsp;&nbsp;- `f`: Falling edge (pulse)<br>&nbsp;&nbsp;- `r`: Rising edge (pulse)<br>&nbsp;&nbsp;- `o`: OneWire (temperature sensor)<br>- `y`: Pull-up resistor (0 = OFF, 1 = ON)<br>- `z`: Minimum time, in milliseconds, between pulses<br>Example: `m1 1 r 1 50` |
| **n\<n>** | Set node ID [1..60]<br>Example: `n5` sets node ID to 5 |
| **o<x>** | OneWire configuration<br>Options:<br>- `x` = `ca`: clear all saved OneWire addresses<br>- `x` = `c<n>`: clear saved OneWire address for channel `n`<br>- `x` = `f`: reset and find OneWire devices<br>- `x` = `h`: hold the found OneWire addresses (overrides any manually configured addresses)<br>- `x` = `l`: list OneWire devices<br>- `x` = `n`: list all saved OneWire addresses<br>- `x` = integer, `n`: move an address to position `n` (see Examples)<br>- `x` = `r <a> <b>`, remap found sensor `<a>` to index `<b>` |
| **p\<n>** | Set the RF power level<br>Example: `p7` |
| **q** | Reset the system (confirmation required) |
| **r** | Restore default settings (unsaved changes will be lost) |
| **rs** | Restore saved settings (unsaved changes will be lost) |
| **s** | Save settings to NVM (non-volatile memory)<br>Must be used after making configuration changes |
| **t** | Trigger report on next cycle (force immediate data transmission) |
| **v** | Show firmware and board information |
| **w\<n>** | RF module active<br>- `w0`: Disable RF<br>- `w1`: Enable RF |
| **x\<n>** | 433 MHz RF frequency compatibility<br>- `x0`: 433.92 MHz (standard)<br>- `x1`: 433.00 MHz (legacy compatibility) |
| **z** | Zero energy/pulse accumulators (reset Wh/pulse counters)<br>- `z`: Zero all accumulators (E1-E12, pulse1-3) with confirmation<br>- `ze1` to `ze12`: Zero individual energy accumulator (e.g., `ze3` zeros E3 only)<br>- `zp1` to `zp2`: Zero individual pulse accumulator (e.g., `zp1` zeros pulse1 only)<br>All commands require 'y' confirmation |

## EmonHub Node Decoder Configuration

The emonTx6 transmits data via RF that needs to be decoded by EmonHub. The node decoder configuration defines how the raw data packets are interpreted. Configuration depends on whether the emonTx6 is operating in single-phase or three-phase mode.

### Single-Phase Mode

When only voltage channel V1 is active, the emonTx6 operates in single-phase mode and transmits a single voltage reading along with power and energy data.

The decoder configuration for node 20 (main CT1-6 channels) in single-phase mode is:

```
[[20]]
nodename = emonTx6_20
[[[rx]]]
names =     MSG, Vrms, P1, P2, P3, P4, P5, P6, E1, E2, E3, E4, E5, E6
datacodes = L, h, h, h, h, h, h, h, l, l, l, l, l, l
scales =    1.0, 0.01, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0
units =     n, V, W, W, W, W, W, W, Wh, Wh, Wh, Wh, Wh, Wh
```

**Data fields:**
- `MSG`: Message counter
- `Vrms`: Single RMS voltage (scaled by 0.01)
- `P1-P6`: Real power for CT channels 1-6 (watts)
- `E1-E6`: Energy accumulator for CT channels 1-6 (watt-hours)

### Three-Phase Mode

When all three voltage channels (V1, V2, V3) are active on the emonTx6, it operates in three-phase mode and transmits three-phase voltage readings along with power and energy data.

The decoder configuration for node 20 (main CT1-6 channels) in three-phase mode is:

```
[[20]]
nodename = emonTx6_20
[[[rx]]]
names =     MSG, Vrms1, Vrms2, Vrms3, P1, P2, P3, P4, P5, P6, E1, E2, E3, E4, E5, E6
datacodes = L, h, h, h, h, h, h, h, h, h, l, l, l, l, l, l
scales =    1.0, 0.01, 0.01, 0.01, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0
units =     n, V, V, V, W, W, W, W, W, W, Wh, Wh, Wh, Wh, Wh, Wh
```

**Data fields:**
- `MSG`: Message counter
- `Vrms1, Vrms2, Vrms3`: Three-phase RMS voltages (scaled by 0.01)
- `P1-P6`: Real power for CT channels 1-6 (watts)
- `E1-E6`: Energy accumulator for CT channels 1-6 (watt-hours)

### Additional Nodes

Both single-phase and three-phase modes use additional nodes for supplementary data:

- **Node N+1**: Temperature and pulse data (node 21 if N=20)
- **Node N+2**: Extended CT channels 7-12 (node 22 if N=20)

### Configuration Steps

1. Determine your emonTx6's operating mode by checking which voltage channels are active (use the `l` command in serial configuration).
2. Select the appropriate decoder configuration above (single-phase or three-phase).
3. Adjust the node number `[[20]]` to match your emonTx6's configured node ID.
4. Add the configuration to your `emonhub.conf` file in the appropriate section.
