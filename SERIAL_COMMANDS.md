# emon32 Serial Commands Reference

This document lists all available serial commands for the emon32 firmware (emonPi3/emonTx6).

## Serial Connection Settings

### USB CDC (Virtual Serial Port)
- **Baud rate**: Not applicable (USB CDC automatically negotiates)
- **Data format**: 8 data bits, no parity, 1 stop bit (8N1)
- **Line termination**: `\r\n` or `\n`
- **Flow control**: Not required (optional on Windows for some terminal applications)

### Hardware UART
- **Baud rate**: 115200
- **Data format**: 8 data bits, no parity, 1 stop bit (8N1)
- **Line termination**: `\r\n` or `\n`
- **Flow control**: Not used

## Quick Start

To test if your serial connection is working, try these commands first:

- **v** - Shows firmware version and board info
- **l** - Lists all current configuration settings
- **t** - Triggers an immediate data report
- **?** - Shows the built-in help text

## Command Reference

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
| **m\<v> \<w> \<x> \<y> \<z>** | Configure a OneWire/pulse input<br>Parameters:<br>- `v`: Channel index<br>- `w`: Active status (0 = DISABLED, 1 = ENABLED)<br>- `x`: Function select<br>&nbsp;&nbsp;- `b`: Both edges (pulse)<br>&nbsp;&nbsp;- `f`: Falling edge (pulse)<br>&nbsp;&nbsp;- `r`: Rising edge (pulse)<br>&nbsp;&nbsp;- `o`: OneWire (temperature sensor)<br>- `y`: Pull-up resistor (0 = OFF, 1 = ON)<br>- `z`: Minimum period in ms (debounce)<br>Example: `m1 1 r 1 50` |
| **n\<n>** | Set node ID [1..60]<br>Example: `n5` sets node ID to 5 |
| **o** | Reserved for auto calibration (not yet implemented) |
| **p\<n>** | Set the RF power level<br>Example: `p7` |
| **r** | Restore default settings (WARNING: overwrites configuration) |
| **s** | Save settings to NVM (non-volatile memory)<br>Must be used after making configuration changes |
| **t** | Trigger report on next cycle (force immediate data transmission) |
| **v** | Show firmware and board information |
| **w\<n>** | RF module active<br>- `w0`: Disable RF<br>- `w1`: Enable RF |
| **x\<n>** | 433 MHz RF frequency compatibility<br>- `x0`: 433.92 MHz (standard)<br>- `x1`: 433.00 MHz (legacy compatibility) |
| **z** | Zero energy accumulators (reset Wh counters) |

## Configuration Workflow

When making configuration changes:

1. Make your changes using the appropriate commands (e.g., `k`, `m`, `f`, etc.)
2. Verify with `l` (list settings)
3. Save with `s` (save to NVM)
4. Settings that require a reset will automatically trigger a reset when saved

## Examples

### Enable Voltage Channels V2 and V3 (for 3-phase monitoring)
```
k2 1 100.0           # Enable V2 with default calibration (100.0)
k3 1 100.0           # Enable V3 with default calibration (100.0)
s                    # Save configuration
```

**Note:** For voltage channels (1-3), you only need:
- Channel number (2 or 3)
- Active status (1 = enabled)
- Calibration value (typically 100.0, range: 25.0-150.0)

The phase and voltage reference parameters (v1, v2) are only required for CT channels.

### Configure a CT channel
```
k4 1 90.0 1.5 1 1    # Enable CT4, cal=90.0, phase=1.5°, refs V1-V1
s                     # Save configuration
```

### Configure a 3-phase CT on L1-L2
```
k4 1 90.0 4.2 1 2    # Enable CT4, references V1 and V2 (L1-L2 load)
s                     # Save configuration
```

### Configure a pulse input
```
m1 1 r 1 50          # Enable OPA1, rising edge, pull-up on, 50ms debounce
s                     # Save configuration
```

### Enable serial logging with JSON
```
c1                    # Enable serial logging
j1                    # Enable JSON format
s                     # Save configuration
```

### Set line frequency to 50 Hz
```
f50                   # Set to 50 Hz
s                     # Save (will automatically reset)
```

## Troubleshooting

### No Output on USB CDC
If you're not seeing any output on USB:

1. Check that your serial terminal is connected (COM port open)
2. On Windows, some terminal applications may require DTR/RTS flow control enabled
3. Try sending `v` to request board information
4. Try sending `t` to trigger an immediate data report
5. Try sending `l` to list configuration
6. Check if serial logging is enabled with `c1` then `s`

### No Output on Hardware UART
If you're not seeing any output on the hardware UART:

1. Verify the UART configuration: 115200 baud, 8N1
2. **Check Rx/Tx connections** - ensure they are the correct way around:
   - Board TX → Your device RX
   - Board RX → Your device TX
3. Try sending `v` to request board information
4. Verify the cable is properly connected and not loose
5. Check if serial logging is enabled with `c1` then `s`

## Notes

- Commands are case-sensitive (lowercase only)
- Line termination: Either `\r\n` or `\n` is accepted
- USB CDC baud rate doesn't matter (automatically negotiated)
- Hardware UART uses 115200 baud, 8N1
- Configuration changes are only temporary until saved with `s`
- Settings that require a reset will automatically trigger a reset when saved
