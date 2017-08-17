THIS_DIR := $(patsubst %/,%,$(dir $(lastword $(MAKEFILE_LIST))))

BOARDNAME = com.proficnc.gnss_1.0

APP_LDSCRIPT_com.proficnc.gnss_1.0 = $(COMMON_DIR)/ldscripts/stm32f3_12K_50K_2K/app.ld
BL_LDSCRIPT_com.proficnc.gnss_1.0 = $(COMMON_DIR)/ldscripts/stm32f3_12K_50K_2K/bl.ld

BOARD_CONFIG_HEADER_com.proficnc.gnss_1.0 := -include $(THIS_DIR)/board.h
CFLAGS_com.proficnc.gnss_1.0 = $(COMMON_STM32F3_CFLAGS) $(BOARD_CONFIG_HEADER_com.proficnc.gnss_1.0)
LDFLAGS_com.proficnc.gnss_1.0 = $(COMMON_STM32F3_LDFLAGS) -L$(dir $(APP_LDSCRIPT_com.proficnc.gnss_1.0))
LDLIBS_com.proficnc.gnss_1.0 = $(COMMON_STM32F3_LDLIBS)

CC_com.proficnc.gnss_1.0 = $(COMMON_STM32F3_CC)
OBJCOPY_com.proficnc.gnss_1.0 = $(COMMON_STM32F3_OBJCOPY)
SIZE_com.proficnc.gnss_1.0 = $(COMMON_STM32F3_SIZE)
