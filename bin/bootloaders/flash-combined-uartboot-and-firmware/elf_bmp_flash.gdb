# 1. Specify the ELF file first so GDB has the "exec file" context
file emon32-latest.elf

# 2. Connect to the hardware
target extended-remote /dev/ttyACM0
monitor swdp_scan
attach 1

# 3. Flash
load

# 5. Verify (Now it knows which file to compare against)
compare-sections

# 6. Finish
monitor reset
quit
