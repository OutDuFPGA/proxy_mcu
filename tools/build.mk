# Standalone build (run from anywhere: make -f tools/build.mk -j8)
# Standalone build reproducing the STM32CubeIDE Debug configuration on Linux.
# The generated Debug/ makefiles hardcode Windows paths (E:/OutDu_Hardware/...),
# so they cannot run here. Flags below are copied verbatim from
# Debug/Application/User/Core/subdir.mk and Debug/makefile.
P   := $(abspath $(dir $(lastword $(MAKEFILE_LIST)))..)

# Image sensor: IMX568 (default - PC3 is the sensor's FLASH input) or IMX678
# (PC3 is an MCU output from boot). Each sensor is a separate build with its
# own output folder and file names, so both images can sit side by side:
#   make -f tools/build.mk -j8              -> Debug_linux/proxy-mcu_Secure.elf
#   make -f tools/build.mk SENSOR=IMX678 -j8
#   (or: make -f tools/build.mk imx678)     -> Debug_linux_imx678/proxy-mcu_Secure_imx678.elf
SENSOR ?= IMX568
ifeq ($(SENSOR),IMX568)
OUT  := $(P)/STM32CubeIDE/Secure/Debug_linux
NAME := proxy-mcu_Secure
else ifeq ($(SENSOR),IMX678)
OUT  := $(P)/STM32CubeIDE/Secure/Debug_linux_imx678
NAME := proxy-mcu_Secure_imx678
else
$(error SENSOR must be IMX568 or IMX678)
endif

CORE := $(P)/Secure/Core/Src
APP  := $(P)/STM32CubeIDE/Secure/Application/User/Core
HAL  := $(P)/Drivers/STM32H5xx_HAL_Driver/Src
LD   := $(P)/STM32CubeIDE/Secure/STM32H563RGVX_FLASH.ld

CSRC := \
  $(CORE)/gpio.c $(CORE)/gtzc_s.c $(CORE)/icache.c $(CORE)/ir_led_cli.c \
  $(CORE)/ir_led_driver.c $(CORE)/dac.c $(CORE)/main.c $(CORE)/secure_nsc.c \
  $(CORE)/stm32h5xx_hal_msp.c $(CORE)/stm32h5xx_it.c $(CORE)/tim.c \
  $(CORE)/usart.c $(CORE)/system_stm32h5xx_s.c \
  $(APP)/TMC2300.c $(APP)/syscalls.c $(APP)/sysmem.c \
  $(HAL)/stm32h5xx_hal.c $(HAL)/stm32h5xx_hal_cortex.c $(HAL)/stm32h5xx_hal_dma.c \
  $(HAL)/stm32h5xx_hal_dma_ex.c $(HAL)/stm32h5xx_hal_exti.c $(HAL)/stm32h5xx_hal_flash.c \
  $(HAL)/stm32h5xx_hal_flash_ex.c $(HAL)/stm32h5xx_hal_gpio.c $(HAL)/stm32h5xx_hal_gtzc.c \
  $(HAL)/stm32h5xx_hal_icache.c $(HAL)/stm32h5xx_hal_pwr.c $(HAL)/stm32h5xx_hal_pwr_ex.c \
  $(HAL)/stm32h5xx_hal_rcc.c $(HAL)/stm32h5xx_hal_rcc_ex.c $(HAL)/stm32h5xx_hal_tim.c \
  $(HAL)/stm32h5xx_hal_tim_ex.c $(HAL)/stm32h5xx_hal_uart.c $(HAL)/stm32h5xx_hal_uart_ex.c

ASRC := $(P)/STM32CubeIDE/Secure/Application/User/Startup/startup_stm32h563rgvx.s

INC := -I$(P)/Secure/Core/Inc -I$(P)/Secure_nsclib \
       -I$(P)/Drivers/STM32H5xx_HAL_Driver/Inc \
       -I$(P)/Drivers/CMSIS/Device/ST/STM32H5xx/Include \
       -I$(P)/Drivers/STM32H5xx_HAL_Driver/Inc/Legacy \
       -I$(P)/Drivers/CMSIS/Include

STAMP  := $(OUT)/sensor-$(SENSOR).stamp

CPU   := -mcpu=cortex-m33 -mfpu=fpv5-sp-d16 -mfloat-abi=hard -mthumb
CFLAGS := $(CPU) -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32H563xx $(INC) \
          -O0 -ffunction-sections -fdata-sections -Wall -mcmse --specs=nano.specs \
          -DIR_SENSOR=IR_SENSOR_$(SENSOR)
LDFLAGS := $(CPU) -T$(LD) --specs=nosys.specs --specs=nano.specs \
           -Wl,-Map=$(OUT)/$(NAME).map -Wl,--gc-sections -static \
           -Wl,--cmse-implib -Wl,--out-implib=$(OUT)/secure_nsclib.o \
           -Wl,--start-group -lc -lm -Wl,--end-group

OBJ := $(addprefix $(OUT)/,$(notdir $(CSRC:.c=.o))) $(OUT)/startup.o
vpath %.c $(sort $(dir $(CSRC)))

$(OUT)/$(NAME).elf: $(OBJ)
	arm-none-eabi-gcc -o $@ $(OBJ) $(LDFLAGS)
	arm-none-eabi-objcopy -O binary $@ $(OUT)/$(NAME).bin
	arm-none-eabi-objcopy -O ihex   $@ $(OUT)/$(NAME).hex
	arm-none-eabi-size $@

# Shortcuts: "make -f tools/build.mk imx678" / "imx568" / "both".
.PHONY: imx568 imx678 both
imx568:
	$(MAKE) -f $(lastword $(MAKEFILE_LIST)) SENSOR=IMX568
imx678:
	$(MAKE) -f $(lastword $(MAKEFILE_LIST)) SENSOR=IMX678
both: imx568 imx678

# Objects carry the sensor choice: switching SENSOR rebuilds everything.
$(OUT)/%.o: %.c $(STAMP) | $(OUT)
	arm-none-eabi-gcc $(CFLAGS) -c $< -o $@

$(STAMP): | $(OUT)
	rm -f $(OUT)/sensor-*.stamp
	touch $@

$(OUT)/startup.o: $(ASRC) | $(OUT)
	arm-none-eabi-gcc $(CPU) -g3 -c -x assembler-with-cpp $< -o $@

$(OUT):
	mkdir -p $(OUT)
