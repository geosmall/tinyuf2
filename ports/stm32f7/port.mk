UF2_FAMILY_ID = 0x53b80f00
CROSS_COMPILE = arm-none-eabi-

ST_HAL_DRIVER = lib/mcu/st/stm32f7xx_hal_driver
ST_CMSIS = lib/mcu/st/cmsis_device_f7
CMSIS_5 = lib/CMSIS_5

# Port Compiler Flags
CFLAGS += \
  -flto \
  -mthumb \
  -mabi=aapcs \
  -mcpu=cortex-m7 \
  -mfloat-abi=hard \
  -mfpu=fpv5-d16 \
  -nostdlib -nostartfiles \
  -DCFG_TUSB_MCU=OPT_MCU_STM32F7

# suppress warning caused by vendor mcu driver
CFLAGS += -Wno-error=cast-align -Wno-error=unused-parameter -Wno-error=shadow

# default linker file
ifdef BUILD_APPLICATION
  LD_FILES ?= $(PORT_DIR)/linker/stm32f7_app.ld
else
  LD_FILES ?= $(PORT_DIR)/linker/stm32f7_boot.ld
endif

# Port source
SRC_C += \
	ports/stm32f7/boards.c \
	ports/stm32f7/board_flash.c \
	$(ST_CMSIS)/Source/Templates/system_stm32f7xx.c \
	$(ST_HAL_DRIVER)/Src/stm32f7xx_hal.c \
	$(ST_HAL_DRIVER)/Src/stm32f7xx_hal_cortex.c \
	$(ST_HAL_DRIVER)/Src/stm32f7xx_hal_rcc.c \
	$(ST_HAL_DRIVER)/Src/stm32f7xx_hal_gpio.c \
	$(ST_HAL_DRIVER)/Src/stm32f7xx_hal_flash.c \
	$(ST_HAL_DRIVER)/Src/stm32f7xx_hal_flash_ex.c \
	$(ST_HAL_DRIVER)/Src/stm32f7xx_hal_pwr.c \
	$(ST_HAL_DRIVER)/Src/stm32f7xx_hal_pwr_ex.c \
	$(ST_HAL_DRIVER)/Src/stm32f7xx_hal_uart.c

ifndef BUILD_NO_TINYUSB
SRC_C += lib/tinyusb/src/portable/synopsys/dwc2/dcd_dwc2.c
SRC_C += lib/tinyusb/src/portable/synopsys/dwc2/dwc2_common.c
endif

# Port include
INC += \
	$(TOP)/$(CMSIS_5)/CMSIS/Core/Include \
	$(TOP)/$(ST_CMSIS)/Include \
	$(TOP)/$(ST_HAL_DRIVER)/Inc
