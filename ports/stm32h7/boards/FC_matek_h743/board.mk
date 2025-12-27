# MATEK H743-WLITE Flight Controller
# STM32H743VIT6 - 2MB Flash, 1MB SRAM

CFLAGS += \
  -DSTM32H743xx \
  -DHSE_VALUE=8000000U \
  -DBOARD_PFLASH_EN

# Use standalone H743 linker script (internal flash)
# Override LD_FILES to use single standalone script instead of combined approach
LD_FILES = $(PORT_DIR)/linker/h743xx.ld

# Startup file
SRC_S += $(ST_CMSIS)/Source/Templates/gcc/startup_stm32h743xx.s

# For flash-jlink target
JLINK_DEVICE = stm32h743vi

flash: flash-jlink
erase: erase-jlink
