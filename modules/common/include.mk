COMMON_DIR := $(patsubst %/,%,$(dir $(lastword $(MAKEFILE_LIST))))
LIBOPENCM3_DIR ?= $(COMMON_DIR)/../libopencm3
LIBCANARD_DIR ?= $(COMMON_DIR)/../libcanard

# common flags
CFLAGS += -DGIT_HASH=0x$(shell git rev-parse --short=8 HEAD)
LDFLAGS += --static -nostartfiles -Wl,--gc-sections --specs=nano.specs -u printf_float -Wl,--no-wchar-size-warning

# STM32F3
COMMON_STM32F3_CC := arm-none-eabi-gcc
COMMON_STM32F3_OBJCOPY := arm-none-eabi-objcopy
COMMON_STM32F3_SIZE := arm-none-eabi-size
COMMON_STM32F3_CFLAGS := -DSTM32F3 -mthumb -mcpu=cortex-m4 -mfloat-abi=hard -mfpu=fpv4-sp-d16
COMMON_STM32F3_LDLIBS = $(LDLIB_OPENCM3_STM32F3) -lm -Wl,--start-group -lc -lgcc -lrdimon -Wl,--end-group

# Link-time optimization
ifdef USE_LTO
  LDFLAGS += -flto
  CFLAGS += -flto
  LIBOPENCM3_MAKE_ARGS = CFLAGS="-fshort-wchar -flto" LDFLAGS="-flto" AR="arm-none-eabi-gcc-ar"
else
  LIBOPENCM3_MAKE_ARGS = CFLAGS="-fshort-wchar"
endif

define common_add_source_directory
DIR_SRC := $(wildcard $(1)/*.c)
DIR_SRC_REL := $$(patsubst $(1)/%,%, $$(DIR_SRC))
DIR_OBJ := $$(addprefix $(2)/, $$(DIR_SRC_REL:.c=.o))

SRC := $$(SRC) $$(DIR_SRC)
OBJ := $$(OBJ) $$(DIR_OBJ)
DEP := $$(DEP) $$(DIR_OBJ:.o=.d)

ifneq ($(3),)
$(3) := $$($(3)) $$(DIR_OBJ)
endif

$$(foreach srcfile,$$(DIR_SRC_REL),$$(eval $(2)/$$(srcfile:.c=.d):$(1)/$$(srcfile)))
endef

####### common_add_libopencm3 #######
define common_add_libopencm3
LDFLAGS += -L $$(LIBOPENCM3_DIR)/lib
CFLAGS += -I $$(LIBOPENCM3_DIR)/include
LDLIB_OPENCM3_STM32F3 := -lopencm3_stm32f3

.PHONY: libopencm3
libopencm3:
	@echo "### BUILDING $$@"
	@$(MAKE) -C $$(LIBOPENCM3_DIR) $$(LIBOPENCM3_MAKE_ARGS)

clean: libopencm3_clean

.PHONY: libopencm3_clean
libopencm3_clean:
	@$(MAKE) -C $$(LIBOPENCM3_DIR) $$(LIBOPENCM3_MAKE_ARGS) clean
endef
####### common_add_libopencm3 #######

define common_add_canard
$(call common_add_source_directory,$(LIBCANARD_DIR),$(1))
CFLAGS += -D"CANARD_ASSERT(x)"="{}" -I$$(LIBCANARD_DIR)
endef

define common_setup_standard
CFLAGS += -std=gnu11 -ffast-math -g -Wdouble-promotion -Wextra -Wshadow -Werror=implicit-function-declaration -Wredundant-decls -Wmissing-prototypes -Wstrict-prototypes -fsingle-precision-constant -fno-common -ffunction-sections -fdata-sections -Wall -Wundef -fshort-wchar
endef
