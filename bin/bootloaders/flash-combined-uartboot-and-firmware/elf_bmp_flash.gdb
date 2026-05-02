# 1. Specify the ELF file first so GDB has the "exec file" context
file bl_serial-emon32.elf

# 2. Connect to the hardware
target extended-remote /dev/ttyACM0
monitor swdp_scan
attach 1

# 3. SAMD21 Preparation
monitor unlock_bootprot
monitor erase_mass

# 4. Flash
load

# 5. Verify (Now it knows which file to compare against)
compare-sections

# 6. Finish
monitor lock_bootprot 3
monitor reset
quit
