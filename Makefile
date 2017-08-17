BOARDS_DIR := boards
APP_SRC_DIR := src
BL_SRC_DIR := modules/bootloader
LIBOPENCM3_DIR := modules/libopencm3
LIBCANARD_DIR := modules/libcanard
COMMON_DIR := modules/common

include $(COMMON_DIR)/include.mk
BOARDS := $(notdir $(wildcard $(BOARDS_DIR)/*))
$(foreach board,$(BOARDS),$(eval -include $(BOARDS_DIR)/$(board)/board.mk))

$(eval $(call common_setup_standard))

all: com.proficnc.gnss_1.0

# add libopencm3
$(eval $(call common_add_libopencm3))

# add libcanard for each board
$(foreach board,$(BOARDS),$(eval $(call common_add_canard,build/$(board)_app/libcanard)))
$(foreach board,$(BOARDS),$(eval $(call common_add_canard,build/$(board)_bl/libcanard)))

# add common for each board
$(foreach board,$(BOARDS),$(eval $(call common_add_source_directory,$(COMMON_DIR)/src,build/$(board)_app/common)))
$(foreach board,$(BOARDS),$(eval $(call common_add_source_directory,$(COMMON_DIR)/src,build/$(board)_bl/common)))
CFLAGS += -I $(COMMON_DIR)/include

# add src for each board
$(foreach board,$(BOARDS),$(eval $(call common_add_source_directory,$(APP_SRC_DIR),build/$(board)_app/src)))
$(foreach board,$(BOARDS),$(eval $(call common_add_source_directory,$(BL_SRC_DIR)/src,build/$(board)_bl/src)))

-include $(DEP)

# TODO: templatize

$(OBJ): libopencm3

.PHONY: com.proficnc.gnss_1.0 com.proficnc.gnss_1.0_app com.proficnc.gnss_1.0_bl
com.proficnc.gnss_1.0: com.proficnc.gnss_1.0_app com.proficnc.gnss_1.0_bl
com.proficnc.gnss_1.0_app: build/com.proficnc.gnss_1.0_app/bin/main.elf
com.proficnc.gnss_1.0_bl: build/com.proficnc.gnss_1.0_bl/bin/main.elf

build/com.proficnc.gnss_1.0/bin/main.bin: build/com.proficnc.gnss_1.0/bin/main.elf
	@echo "### BUILDING $@"
	$(Q) mkdir -p "$(dir $@)"
	$(Q) $(OBJCOPY_com.proficnc.gnss_1.0) -O binary $< $@
	$(Q) python $(COMMON_DIR)/tools/crc_binary.py $@ $@

build/com.proficnc.gnss_1.0_bl/bin/main.elf: $(filter build/com.proficnc.gnss_1.0_bl/%.o, $(OBJ)) $(BL_LDSCRIPT_com.proficnc.gnss_1.0)
build/com.proficnc.gnss_1.0_app/bin/main.elf: $(filter build/com.proficnc.gnss_1.0_app/%.o, $(OBJ)) $(APP_LDSCRIPT_com.proficnc.gnss_1.0)

build/com.proficnc.gnss_1.0_app/bin/main.elf build/com.proficnc.gnss_1.0_bl/bin/main.elf:
	@echo "### BUILDING $@"
	$(Q) mkdir -p $(dir $@)
	$(Q) $(CC_com.proficnc.gnss_1.0) $(LDFLAGS) $(LDFLAGS_com.proficnc.gnss_1.0) $(CFLAGS) $(filter %.o,$^) $(LDLIBS) $(LDLIBS_com.proficnc.gnss_1.0) $(CFLAGS) $(CFLAGS_com.proficnc.gnss_1.0) -T $(notdir $(filter %.ld,$^)) -o $@
	$(Q) $(SIZE_com.proficnc.gnss_1.0) $@

build/com.proficnc.gnss_1.0_app/%.d:
	$(Q) mkdir -p $(dir $@)
	$(Q) $(CC_com.proficnc.gnss_1.0) $(CFLAGS) $(CFLAGS_com.proficnc.gnss_1.0) -c $< -MP -MM -MF $@ -MT $(@:.d=.o)

build/com.proficnc.gnss_1.0_app/%.o:
	@echo "### BUILDING $@"
	$(Q) $(CC_com.proficnc.gnss_1.0) $(CFLAGS) $(CFLAGS_com.proficnc.gnss_1.0) -c $< -o $@

clean:
	$(Q) rm -rf build
#
# $(call add_source_directory,../common,common)
#
# BOOTLOADER_DIR := ../bootloader
# LIBOPENCM3_DIR := omd_common/libopencm3
# LIBCANARD_DIR := omd_common/libcanard
# LDSCRIPT := boards/stm32f302k8/app.ld
# BL_LDSCRIPT := boards/stm32f302k8/bl.ld
# BOARD_CONFIG_HEADER := boards/board_can_gps.h
#
# ARCH_FLAGS := -mthumb -mcpu=cortex-m4 -mfloat-abi=hard -mfpu=fpv4-sp-d16
#
# LDFLAGS := --static -nostartfiles -L$(LIBOPENCM3_DIR)/lib -L $(dir $(LDSCRIPT)) -T$(LDSCRIPT) -Wl,--gc-sections --specs=nano.specs -u printf_float -Wl,--no-wchar-size-warning
#
# LDLIBS := -lopencm3_stm32f3 -lm -Wl,--start-group -lc -lgcc -lrdimon -Wl,--end-group
#
# CFLAGS += -std=gnu11 -O3 -ffast-math -g -Wdouble-promotion -Wextra -Wshadow -Werror=implicit-function-declaration -Wredundant-decls -Wmissing-prototypes -Wstrict-prototypes -fsingle-precision-constant -fno-common -ffunction-sections -fdata-sections -MD -Wall -Wundef -Isrc -I$(LIBOPENCM3_DIR)/include -I$(LIBCANARD_DIR) -I$(BOOTLOADER_DIR)/include -DSTM32F3 -D"CANARD_ASSERT(x)"="do {} while(0)" -DGIT_HASH=0x$(shell git rev-parse --short=8 HEAD) -fshort-wchar -include $(BOARD_CONFIG_HEADER)
#
# COMMON_OBJS := $(addprefix build/,$(addsuffix .o,$(basename $(shell find src -name "*.c"))))
# COMMON_OBJS += $(addprefix build/,$(addsuffix .o,$(basename $(shell find $(BOOTLOADER_DIR)/shared -name "*.c"))))
#
# BIN := build/bin/main.elf build/bin/main.bin
#
# .PHONY: all
# all: $(LIBOPENCM3_DIR) $(BIN)
#
# build/bin/%.elf: $(COMMON_OBJS) build/canard.o
# 	@echo "### BUILDING $@"
# 	@mkdir -p "$(dir $@)"
# 	@arm-none-eabi-gcc $(LDFLAGS) $(ARCH_FLAGS) $^ $(LDLIBS) -o $@
# 	@arm-none-eabi-size $@
#
# build/bin/%.bin: build/bin/%.elf
# 	@echo "### BUILDING $@"
# 	@mkdir -p "$(dir $@)"
# 	@arm-none-eabi-objcopy -O binary $< $@
# 	@python $(BOOTLOADER_DIR)/tools/crc_binary.py $@ $@
#
# .PRECIOUS: build/%.o
# build/%.o: %.c $(LIBOPENCM3_DIR)
# 	@echo "### BUILDING $@"
# 	@mkdir -p "$(dir $@)"
# 	@arm-none-eabi-gcc $(CFLAGS) $(ARCH_FLAGS) -S $< -o $(patsubst %.o,%.S,$@)
# 	@arm-none-eabi-gcc $(CFLAGS) $(ARCH_FLAGS) -c $< -o $@
#
# build/canard.o: $(LIBCANARD_DIR)/canard.c
# 	@echo "### BUILDING $@"
# 	@mkdir -p "$(dir $@)"
# 	@arm-none-eabi-gcc $(CFLAGS) $(ARCH_FLAGS) -c $< -o $@
#
# .PHONY: $(LIBOPENCM3_DIR)
# $(LIBOPENCM3_DIR):
# 	@echo "### BUILDING $@"
# 	@$(MAKE) -C $(LIBOPENCM3_DIR) CFLAGS="-fshort-wchar"
#
# upload: build/bin/main.elf build/bin/main.bin
# 	@echo "### UPLOADING"
# 	@openocd -f openocd.cfg -c "program $< verify reset exit"
#
# .PHONY: clean
# clean:
# 	@$(MAKE) -C $(LIBOPENCM3_DIR) clean
# 	@rm -rf build
#
# .PHONY: bootloader
# bootloader:
# 	@echo "### BUILDING BOOTLOADER..."
# 	@$(MAKE) -C $(BOOTLOADER_DIR) LDSCRIPT=$(abspath $(BL_LDSCRIPT)) BOARD_CONFIG_HEADER=$(abspath $(BOARD_CONFIG_HEADER)) clean
# 	@$(MAKE) -C $(BOOTLOADER_DIR) LDSCRIPT=$(abspath $(BL_LDSCRIPT)) BOARD_CONFIG_HEADER=$(abspath $(BOARD_CONFIG_HEADER)) USE_LTO=1
#
# .PHONY: bootloader-upload
# bootloader-upload: bootloader
# 	@echo "### UPLOADING BOOTLOADER..."
# 	@$(MAKE) -C $(BOOTLOADER_DIR) LDSCRIPT=$(abspath $(BL_LDSCRIPT)) BOARD_CONFIG_HEADER=$(abspath $(BOARD_CONFIG_HEADER)) upload
