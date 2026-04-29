arm-none-eabi-objcopy -I binary -O elf32-littlearm -B arm \
    --rename-section .data=.text,contents,alloc,load,readonly,code \
    --change-section-address .data=0 \
    bl_serial-emon32-v1.0.bin bl_serial-emon32-v1.0.elf
