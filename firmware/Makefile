#======================================================================#
#Output files
EXECUTABLE=colin_imu.elf
BIN_IMAGE=colin_imu.bin

#======================================================================#
#Cross Compiler
CC=arm-none-eabi-gcc
OBJCOPY=arm-none-eabi-objcopy
GDB=arm-none-eabi-gdb

#======================================================================#
#Flags
CFLAGS=-g -mlittle-endian -mthumb
CFLAGS+=-mcpu=cortex-m4
CFLAGS+=-mfpu=fpv4-sp-d16 -mfloat-abi=hard
CFLAGS+=--specs=nano.specs --specs=nosys.specs
CFLAGS+=-Wall
CFLAGS+=-D USE_STDPERIPH_DRIVER \
	-D STM32F429_439xx \
	-D __FPU_PRESENT=1 \
	-D __FPU_USED=1 \
	-D ARM_MATH_CM4
CFLAGS+=-I./

#stm32-flash
CFLAGS+=-Wl,-T,stm32_flash.ld

ARCH=CM4F

LDLIBS+=-Wl,--start-group -lm -Wl,--end-group

#======================================================================#
#Libraries

#Stm32 libraries
ST_LIB=./lib/STM32F4xx_StdPeriph_Driver
CFLAGS+=-I./lib/CMSIS/ST/STM32F4xx/Include

#CMSIS libraries
CFLAGS+=-I./lib/CMSIS/Include

#StdPeriph includes
CFLAGS+=-I$(ST_LIB)/inc

#Freertos
FREERTOS=./lib/FreeRTOS

#Major program
COMMON=./common
DRIVER=./driver
AHRS=./ahrs

#======================================================================#
#Source code
SRC=./lib/CMSIS/system_stm32f4xx.c

#StdPeriph
CFLAGS+=-I$(ST_LIB)/inc
SRC+=$(ST_LIB)/src/misc.c \
	$(ST_LIB)/src/stm32f4xx_rcc.c \
	$(ST_LIB)/src/stm32f4xx_dma.c \
	$(ST_LIB)/src/stm32f4xx_flash.c \
	$(ST_LIB)/src/stm32f4xx_gpio.c \
	$(ST_LIB)/src/stm32f4xx_usart.c \
	$(ST_LIB)/src/stm32f4xx_tim.c\
	$(ST_LIB)/src/stm32f4xx_spi.c\
	$(ST_LIB)/src/stm32f4xx_i2c.c \
	$(ST_LIB)/src/stm32f4xx_adc.c

#Freertos
CFLAGS+=-I$(FREERTOS)/Source/include \
	-I$(FREERTOS)/Source/portable/GCC/ARM_$(ARCH)
SRC+=$(FREERTOS)/Source/croutine.c \
	$(FREERTOS)/Source/list.c \
	$(FREERTOS)/Source/queue.c \
	$(FREERTOS)/Source/tasks.c \
	$(FREERTOS)/Source/timers.c \
	$(FREERTOS)/Source/portable/MemMang/heap_4.c \
	$(FREERTOS)/Source/portable/GCC/ARM_$(ARCH)/port.c

#Major programs
CFLAGS+=-I$(COMMON)
SRC+=$(COMMON)/stdlib.c \
	$(COMMON)/string.c \
	$(COMMON)/delay.c

CFLAGS+=-I$(DRIVER)/peripheral/
SRC+=$(DRIVER)/peripheral/usart.c \
	$(DRIVER)/peripheral/i2c.c \
	$(DRIVER)/peripheral/led.c

CFLAGS+=-I$(DRIVER)/device/
SRC+=$(DRIVER)/device/mpu6050.c

CFLAGS+=-I$(AHRS)
SRC+=$(AHRS)/main.c \
	$(AHRS)/moving_average.c

CFLAGS+=-I./

#======================================================================#
#STM32 startup file
STARTUP=./startup_stm32f4xx.s

#======================================================================#
#Make rules

#Make all
all:$(BIN_IMAGE)

$(BIN_IMAGE):$(EXECUTABLE)
	$(OBJCOPY) -O binary $^ $@

STARTUP_OBJ = startup_stm32f4xx.o

$(STARTUP_OBJ): $(STARTUP) 
	$(CC) $(CFLAGS) $^ $(LDLIBS) -c $(STARTUP)

$(EXECUTABLE):$(SRC) $(STARTUP_OBJ)
	$(CC) $(CFLAGS) $^ $(LDLIBS) -o $@

#Make clean
clean:
	rm -rf $(EXECUTABLE)
	rm -rf $(BIN_IMAGE)
#Make flash
flash:
	st-flash write $(BIN_IMAGE) 0x8000000

#Make openocd
openocd:
	openocd -s /opt/openocd/share/openocd/scripts/ -f ./debug/openocd.cfg

#Make cgdb
gdbauto:
	cgdb -d $(GDB) -x ./debug/openocd_gdb.gdb

#======================================================================
.PHONY:all clean flash
