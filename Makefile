BOOTLOADER_DIR := submodules/bootloader
LIBOPENCM3_DIR := submodules/libopencm3
LIBCANARD_DIR := submodules/libcanard
LDSCRIPT := boards/stm32f302k8/app.ld
BL_LDSCRIPT := boards/stm32f302k8/bl.ld
BL_CONFIG_FILE := boards/board_jc_esc.c

ARCH_FLAGS := -mthumb -mcpu=cortex-m4 -mfloat-abi=hard -mfpu=fpv4-sp-d16

LDFLAGS := --static -nostartfiles -L$(LIBOPENCM3_DIR)/lib -L $(dir $(LDSCRIPT)) -T$(LDSCRIPT) -Wl,--gc-sections --specs=nano.specs -u printf_float

LDLIBS := -lopencm3_stm32f3 -lm -Wl,--start-group -lc -lgcc -lrdimon -Wl,--end-group

CFLAGS += -std=gnu11 -O3 -ffast-math -g -Wdouble-promotion -Wextra -Wshadow -Werror=implicit-function-declaration -Wredundant-decls -Wmissing-prototypes -Wstrict-prototypes -fsingle-precision-constant -fno-common -ffunction-sections -fdata-sections -MD -Wall -Wundef -Isrc -I$(LIBOPENCM3_DIR)/include -I$(LIBCANARD_DIR) -I$(BOOTLOADER_DIR)/include -DSTM32F3 -D"CANARD_ASSERT(x)"="do {} while(0)" -DGIT_HASH=0x$(shell git rev-parse --short=8 HEAD) -fshort-wchar

COMMON_OBJS := $(addprefix build/,$(addsuffix .o,$(basename $(shell find src/esc -name "*.c"))))
COMMON_OBJS += $(addprefix build/,$(addsuffix .o,$(basename $(shell find $(BOOTLOADER_DIR)/shared -name "*.c"))))

BIN := build/bin/main.elf build/bin/main.bin

.PHONY: all
all: $(LIBOPENCM3_DIR) $(BIN)

.PRECIOUS: src/esc/ekf.c src/esc/ekf.h
src/esc/ekf.h src/esc/ekf.c: tools/ekf/ekf_generator.py
	python tools/ekf/ekf_generator.py src/esc/ekf.h src/esc/ekf.c

build/bin/%.elf: $(COMMON_OBJS) build/canard.o build/src/esc/ekf.o
	@echo "### BUILDING $@"
	@mkdir -p "$(dir $@)"
	@arm-none-eabi-gcc $(LDFLAGS) $(ARCH_FLAGS) $^ $(LDLIBS) -o $@
	@arm-none-eabi-size $@

build/bin/%.bin: build/bin/%.elf
	@echo "### BUILDING $@"
	@mkdir -p "$(dir $@)"
	@arm-none-eabi-objcopy -O binary $< $@
	@python $(BOOTLOADER_DIR)/tools/crc_binary.py $@ $@

.PRECIOUS: build/%.o
build/%.o: %.c $(LIBOPENCM3_DIR)
	@echo "### BUILDING $@"
	@mkdir -p "$(dir $@)"
	@arm-none-eabi-gcc $(CFLAGS) $(ARCH_FLAGS) -S $< -o $(patsubst %.o,%.S,$@)
	@arm-none-eabi-gcc $(CFLAGS) $(ARCH_FLAGS) -c $< -o $@

build/canard.o: $(LIBCANARD_DIR)/canard.c
	@echo "### BUILDING $@"
	@mkdir -p "$(dir $@)"
	@arm-none-eabi-gcc $(CFLAGS) $(ARCH_FLAGS) -c $< -o $@

.PHONY: $(LIBOPENCM3_DIR)
$(LIBOPENCM3_DIR):
	@echo "### BUILDING $@"
	@$(MAKE) -C $(LIBOPENCM3_DIR) CFLAGS="-fshort-wchar"

upload: build/bin/main.elf build/bin/main.bin
	@echo "### UPLOADING"
	@openocd -f openocd.cfg -c "program $< verify reset exit"

.PHONY: clean
clean:
	@$(MAKE) -C $(LIBOPENCM3_DIR) clean
	@rm -rf build
	@rm -f src/esc/ekf.h
	@rm -f src/esc/ekf.c

bootloader:
	@echo "### BUILDING BOOTLOADER..."
	@$(MAKE) -C $(BOOTLOADER_DIR) LDSCRIPT=$(abspath $(BL_LDSCRIPT)) BOARD_CONFIG_FILE=$(abspath $(BL_CONFIG_FILE)) clean
	@$(MAKE) -C $(BOOTLOADER_DIR) LDSCRIPT=$(abspath $(BL_LDSCRIPT)) BOARD_CONFIG_FILE=$(abspath $(BL_CONFIG_FILE))

bootloader-upload:
	@echo "### BUILDING AND UPLOADING BOOTLOADER..."
	@$(MAKE) -C $(BOOTLOADER_DIR) LDSCRIPT=$(abspath $(BL_LDSCRIPT)) BOARD_CONFIG_FILE=$(abspath $(BL_CONFIG_FILE)) clean
	@$(MAKE) -C $(BOOTLOADER_DIR) LDSCRIPT=$(abspath $(BL_LDSCRIPT)) BOARD_CONFIG_FILE=$(abspath $(BL_CONFIG_FILE)) upload
