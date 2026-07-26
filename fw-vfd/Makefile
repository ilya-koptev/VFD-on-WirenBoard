# Сборка загрузчика VFD-модуля. Раскладка и стиль — как в wb-embedded-controller:
# system/ + include/ + src/, CMSIS без вендорского HAL, -Os.
#
# На Windows-стенде make отсутствует, поэтому рядом лежит build.py с теми же
# флагами; на контроллере и на Linux собирается этой цели.

TARGET   ?= boot
MCU      ?= STM32F411xE
CPU      ?= cortex-m4

PREFIX   ?= arm-none-eabi-
CC        = $(PREFIX)gcc
OBJCOPY   = $(PREFIX)objcopy
SIZE      = $(PREFIX)size

# CMSIS: заголовки ядра и описание периферии, идут из пакета PlatformIO
CUBE     ?= $(HOME)/.platformio/packages/framework-stm32cubef4
CMSIS     = $(CUBE)/Drivers/CMSIS

BUILD     = build
SRC       = src/boot/main.c system/startup.c
LDSCRIPT  = ldscripts/stm32f411_boot.ld

INCLUDES  = -Iinclude -I$(CMSIS)/Include -I$(CMSIS)/Device/ST/STM32F4xx/Include

CFLAGS    = -mcpu=$(CPU) -mthumb -Os -std=gnu11 \
            -ffunction-sections -fdata-sections -fno-common \
            -Wall -Wextra -D$(MCU) $(INCLUDES)

LDFLAGS   = -mcpu=$(CPU) -mthumb -T$(LDSCRIPT) -nostartfiles \
            -Wl,--gc-sections -Wl,-Map=$(BUILD)/$(TARGET).map

OBJ       = $(addprefix $(BUILD)/,$(notdir $(SRC:.c=.o)))
vpath %.c src/boot system

all: $(BUILD)/$(TARGET).bin

$(BUILD)/%.o: %.c | $(BUILD)
	$(CC) $(CFLAGS) -c $< -o $@

$(BUILD)/$(TARGET).elf: $(OBJ)
	$(CC) $(LDFLAGS) $^ -o $@
	$(SIZE) $@

$(BUILD)/$(TARGET).bin: $(BUILD)/$(TARGET).elf
	$(OBJCOPY) -O binary $< $@

$(BUILD):
	mkdir -p $(BUILD)

clean:
	rm -rf $(BUILD)

.PHONY: all clean
