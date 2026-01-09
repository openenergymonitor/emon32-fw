##############################################################################
BUILD = build
BIN = emon32
OUT = bin
##############################################################################
.PHONY: all directory clean size

# Path to toolchain, e.g. /path/to/bin/ Leave empty if already on path.
TC_PATH =
CC = $(TC_PATH)arm-none-eabi-gcc
OBJCOPY = $(TC_PATH)arm-none-eabi-objcopy
SIZE = $(TC_PATH)arm-none-eabi-size

ifeq ($(OS), Windows_NT)
  MKDIR = gmkdir
else
  MKDIR = mkdir
endif

CFLAGS += -W -Wall -Wextra -Wpedantic --std=c17 -Os -g3
CFLAGS += -fno-diagnostics-show-caret -fno-common
CFLAGS += -fdata-sections -ffunction-sections
CFLAGS += -funsigned-char -funsigned-bitfields
CFLAGS += -Wuninitialized
CFLAGS += -Wshadow -Wdouble-promotion -Wundef
CFLAGS += -mcpu=cortex-m0plus -mthumb
CFLAGS += -Wconversion -Wsign-conversion
CFLAGS += -MD -MP -MT $(BUILD)/$(*F).o -MF $(BUILD)/$(@F).d

LDFLAGS += -mcpu=cortex-m0plus -mthumb
LDFLAGS += -Wl,--gc-sections
LDFLAGS += -Wl,--print-memory-usage
LDFLAGS += -Wl,--script=./linker/samd21j17.ld

INCLUDES += \
  -isystem ./include/samd21 \
  -isystem ./third_party/printf \
  -isystem ./third_party/qfplib \
  -isystem ./third_party/RFM69 \
  -isystem ./third_party/tinyusb/src \
  -isystem ./third_party/tinyusb/src/device \
  -I./src/

SRCS += $(wildcard ./src/*.c) \
  $(wildcard ./third_party/printf/*.c) \
  $(wildcard ./third_party/tinyusb/src/*.c) \
  $(wildcard ./third_party/tinyusb/src/class/cdc/*.c) \
  $(wildcard ./third_party/tinyusb/src/common/*.c) \
  $(wildcard ./third_party/tinyusb/src/device/*.c) \
  $(wildcard ./third_party/tinyusb/src/portable/microchip/samd/*.c)

DEFINES += \
  -D__SAMD21J17A__ \
  -DDONT_USE_CMSIS_INIT \
  -DCFG_TUSB_MCU=OPT_MCU_SAMD21

CFLAGS += $(INCLUDES) $(DEFINES)

OBJS = $(addprefix $(BUILD)/, $(notdir %/$(subst .c,.o, $(SRCS))))
OBJS += $(BUILD)/qfplib-m0-full.o

# Always update the build information. This forces this to run every time. Exit
# if this fails - likely to be a path of Python version issue.
BUILD_INFO := $(shell python3 ./scripts/build_info.py ./src/emon32_build_info.c)
ifeq ($(strip $(BUILD_INFO)), )
$(error 1)
endif

VERSION_INFO := $(shell python3 ./scripts/version_info.py)

all: directory $(BUILD)/$(BIN).elf $(BUILD)/$(BIN).hex $(BUILD)/$(BIN).bin $(BUILD)/$(BIN).uf2 size

$(BUILD)/$(BIN).elf: $(OBJS)
	@echo LD $@
	@$(CC) $(LDFLAGS) $(OBJS) $(LIBS) -o $@
	@cp $@ $(OUT)/$(VERSION_INFO).elf

$(BUILD)/$(BIN).hex: $(BUILD)/$(BIN).elf
	@echo OBJCOPY $@
	@$(OBJCOPY) -O ihex $^ $@
	@cp $@ $(OUT)/$(VERSION_INFO).hex

$(BUILD)/$(BIN).bin: $(BUILD)/$(BIN).elf
	@echo OBJCOPY $@
	@$(OBJCOPY) -O binary $^ $@
	@cp $@ $(OUT)/$(VERSION_INFO).bin

$(BUILD)/$(BIN).uf2: $(BUILD)/$(BIN).bin
	@echo BIN_TO_UF2 $@
	@python3 ./scripts/bin_to_uf2.py $(BUILD)/$(BIN).bin $(BUILD)/$(BIN).uf2
	@[ -f $@ ] && cp $@ $(OUT)/$(VERSION_INFO).uf2 || true

$(BUILD)/qfplib-m0-full.o:
	@echo AS $@
	@$(CC) $(CFLAGS) third_party/qfplib/qfplib-m0-full.s -c -o $@

%.o:
	@echo CC $@
	@$(CC) $(CFLAGS) $(filter %/$(subst .o,.c,$(notdir $@)), $(SRCS)) -c -o $@

directory:
	@$(MKDIR) -p $(BUILD)
	@$(MKDIR) -p $(OUT)

size: $(BUILD)/$(BIN).elf
	@echo size:
	@$(SIZE) -t $^

clean:
	@echo clean
	@-rm -rf $(BUILD)
	@-rm -f $(OUT)/emon32*

-include $(wildcard $(BUILD)/*.d)
