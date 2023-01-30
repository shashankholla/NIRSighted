######################################
# paths
######################################

ifneq ($(wildcard paths.mk),)
  include paths.mk
else
  $(info Copying file default.paths.mk into paths.mk.)
  $(info Please customize its variables appropriately for your computer.)
  $(shell cp default.paths.mk paths.mk)
  include paths.mk
endif

# Check to make sure that paths.mk defined all of the variables that we needed.
ifeq ($(wildcard $(CUBE_PATH)/.),)
  $(info Please edit paths.mk to point the variable CUBE_PATH to the root directory of the STM32CubeL4 SDK.)
  $(info The STM32CubeL4 SDK can be downloaded from https://github.com/STMicroelectronics/STM32CubeL4)
  $(error CUBE_PATH path not defined)
endif

######################################
# target
######################################
TARGET = gtcam-rev2b


######################################
# building variables
######################################
# debug build?
DEBUG = 1
# optimization
OPT = -Og

#######################################
# paths
#######################################
# Build path
BUILD_DIR = build

######################################
# source
######################################
# C sources for system code
CSRC =  \
Src/main.c \
Src/freertos.c \
Src/stm32l4xx_it.c \
Src/stm32l4xx_hal_msp.c \
Src/stm32l4xx_hal_timebase_tim.c \
Src/system_stm32l4xx.c \

# C sources for code we've written
CSRC+= Src/uart_spam_task.c
CSRC+= Src/sd_driver_task.c
CSRC+= Src/sd_test_task.c
CSRC+= Src/camera_read_task.c
CSRC+= Src/mlx_read_task.c
CSRC+= Src/runtime_stats_task.c
CSRC+= Src/MLX90640_I2C_Driver.c
CSRC+= Src/MLX90640_API.c
CSRC+= Src/system_health_indicator_task.c
CSRC+= Src/annotator_task.c

#CSRC+= $(CUBE_PATH)/Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_sd.c
CSRC+= Src/_stm32l4xx_hal_sd.c
CSRC+= $(CUBE_PATH)/Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_ll_sdmmc.c
#CSRC+= Src/_stm32l4xx_ll_sdmmc.c

# C sources for saeclib
CSRC+= saeclib/src/saeclib_circular_buffer.c

# C sources for LL and HAL drivers
CSRC+= $(CUBE_PATH)/Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_ll_utils.c \
$(CUBE_PATH)/Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_ll_exti.c \
$(CUBE_PATH)/Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_ll_gpio.c \
$(CUBE_PATH)/Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_ll_adc.c \
$(CUBE_PATH)/Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_ll_dma.c \
$(CUBE_PATH)/Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal.c \
$(CUBE_PATH)/Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_i2c.c \
$(CUBE_PATH)/Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_i2c_ex.c \
$(CUBE_PATH)/Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_rcc.c \
$(CUBE_PATH)/Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_rcc_ex.c \
$(CUBE_PATH)/Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_flash.c \
$(CUBE_PATH)/Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_flash_ex.c \
$(CUBE_PATH)/Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_flash_ramfunc.c \
$(CUBE_PATH)/Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_gpio.c \
$(CUBE_PATH)/Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_dma.c \
$(CUBE_PATH)/Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_dma_ex.c \
$(CUBE_PATH)/Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_pwr.c \
$(CUBE_PATH)/Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_pwr_ex.c \
$(CUBE_PATH)/Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_cortex.c \
$(CUBE_PATH)/Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_exti.c \
$(CUBE_PATH)/Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_dcmi.c \
$(CUBE_PATH)/Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_ll_i2c.c \
$(CUBE_PATH)/Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_sd_ex.c \
$(CUBE_PATH)/Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_ll_spi.c \
$(CUBE_PATH)/Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_tim.c \
$(CUBE_PATH)/Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_tim_ex.c \
$(CUBE_PATH)/Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_ll_tim.c \
$(CUBE_PATH)/Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_ll_usart.c \
$(CUBE_PATH)/Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_ll_rcc.c \
$(CUBE_PATH)/Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_pcd.c \
$(CUBE_PATH)/Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_pcd_ex.c \
$(CUBE_PATH)/Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_uart.c \
$(CUBE_PATH)/Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_uart_ex.c \
$(CUBE_PATH)/Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_usart.c \
$(CUBE_PATH)/Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_usart_ex.c \
$(CUBE_PATH)/Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_ll_usb.c \
$(CUBE_PATH)/Middlewares/Third_Party/FreeRTOS/Source/croutine.c \
$(CUBE_PATH)/Middlewares/Third_Party/FreeRTOS/Source/event_groups.c \
$(CUBE_PATH)/Middlewares/Third_Party/FreeRTOS/Source/list.c \
$(CUBE_PATH)/Middlewares/Third_Party/FreeRTOS/Source/queue.c \
$(CUBE_PATH)/Middlewares/Third_Party/FreeRTOS/Source/stream_buffer.c \
$(CUBE_PATH)/Middlewares/Third_Party/FreeRTOS/Source/tasks.c \
$(CUBE_PATH)/Middlewares/Third_Party/FreeRTOS/Source/timers.c \
$(CUBE_PATH)/Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS/cmsis_os.c \
$(CUBE_PATH)/Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F/port.c \

# ASM sources
ASM_SOURCES =  \
startup_stm32l4s9xx.s


#######################################
# binaries
#######################################
PREFIX = arm-none-eabi-
# The gcc compiler bin path can be either defined in make command via GCC_PATH variable (> make GCC_PATH=xxx)
# either it can be added to the PATH environment variable.
ifdef GCC_PATH
CC = $(GCC_PATH)/$(PREFIX)gcc
AS = $(GCC_PATH)/$(PREFIX)gcc -x assembler-with-cpp
CP = $(GCC_PATH)/$(PREFIX)objcopy
SZ = $(GCC_PATH)/$(PREFIX)size
else
CC = $(PREFIX)gcc
AS = $(PREFIX)gcc -x assembler-with-cpp
CP = $(PREFIX)objcopy
SZ = $(PREFIX)size
endif
HEX = $(CP) -O ihex
BIN = $(CP) -O binary -S

#######################################
# CFLAGS
#######################################
# cpu
CPU = -mcpu=cortex-m4

# fpu
FPU = -mfpu=fpv4-sp-d16

# float-abi
FLOAT-ABI = -mfloat-abi=hard

# mcu
MCU = $(CPU) -mthumb $(FPU) $(FLOAT-ABI)

# macros for gcc
# AS defines
AS_DEFS =

# C defines
C_DEFS =  \
-DUSE_FULL_LL_DRIVER \
-DUSE_HAL_DRIVER \
-DSTM32L4S9xx


# AS includes
AS_INCLUDES =  \
-IInc

# C includes
CINCS =  \
-IInc \
-I$(CUBE_PATH)/Drivers/STM32L4xx_HAL_Driver/Inc \
-I$(CUBE_PATH)/Drivers/STM32L4xx_HAL_Driver/Inc/Legacy \
-I$(CUBE_PATH)/Middlewares/Third_Party/FreeRTOS/Source/include \
-I$(CUBE_PATH)/Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS \
-I$(CUBE_PATH)/Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F \
-I$(CUBE_PATH)/Drivers/CMSIS/Device/ST/STM32L4xx/Include \
-I$(CUBE_PATH)/Drivers/CMSIS/Include \
-I$(CUBE_PATH)/Drivers/CMSIS/Include \


# compile gcc flags
ASFLAGS = $(MCU) $(AS_DEFS) $(AS_INCLUDES) $(OPT) -Wall -fdata-sections -ffunction-sections

CFLAGS = $(MCU) $(C_DEFS) $(CINCS) $(OPT) -Wall -fdata-sections -ffunction-sections

ifeq ($(DEBUG), 1)
CFLAGS += -g -gdwarf-2
endif


# Generate dependency information
CFLAGS += -MMD -MP -MF"$(@:%.o=%.d)"


#######################################
# LDFLAGS
#######################################
# link script
LDSCRIPT = STM32L4S9ZIJx_FLASH.ld

# libraries
LIBS = -lc -lm -lnosys
LIBDIR =
LDFLAGS = $(MCU) -specs=nano.specs -T$(LDSCRIPT) $(LIBDIR) $(LIBS) -Wl,-Map=$(BUILD_DIR)/$(TARGET).map,--cref -Wl,--gc-sections

# default action: build all
all: $(BUILD_DIR)/$(TARGET).elf $(BUILD_DIR)/$(TARGET).hex $(BUILD_DIR)/$(TARGET).bin


#######################################
# build the application
#######################################
# list of objects
OBJECTS = $(addprefix $(BUILD_DIR)/,$(notdir $(CSRC:.c=.o)))
vpath %.c $(sort $(dir $(CSRC)))

# list of ASM program objects
OBJECTS += $(addprefix $(BUILD_DIR)/,$(notdir $(ASM_SOURCES:.s=.o)))
vpath %.s $(sort $(dir $(ASM_SOURCES)))

$(info $(OBJECTS))

$(BUILD_DIR)/%.o: %.c Makefile | $(BUILD_DIR)
	$(CC) -c $(CFLAGS) -Wa,-a,-ad,-alms=$(BUILD_DIR)/$(notdir $(<:.c=.lst)) $< -o $@

$(BUILD_DIR)/%.o: %.s Makefile | $(BUILD_DIR)
	$(AS) -c $(CFLAGS) $< -o $@

$(BUILD_DIR)/$(TARGET).elf: $(OBJECTS) Makefile
	$(CC) $(OBJECTS) $(LDFLAGS) -o $@
	$(SZ) $@

$(BUILD_DIR)/%.hex: $(BUILD_DIR)/%.elf | $(BUILD_DIR)
	$(HEX) $< $@

$(BUILD_DIR)/%.bin: $(BUILD_DIR)/%.elf | $(BUILD_DIR)
	$(BIN) $< $@

$(BUILD_DIR):
	mkdir $@

#######################################
# clean up
#######################################
clean:
	-rm -fR $(BUILD_DIR)

#######################################
# dependencies
#######################################
-include $(wildcard $(BUILD_DIR)/*.d)

# *** EOF ***
