CFLAGS += \
  -DSTM32F722xx \
  -DHSE_VALUE=8000000U

SRC_S += \
  $(ST_CMSIS)/Source/Templates/gcc/startup_stm32f722xx.s

# For flash-jlink target
JLINK_DEVICE = stm32f722re

flash: flash-jlink
erase: erase-jlink
