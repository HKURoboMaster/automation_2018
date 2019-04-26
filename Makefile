.PHONY: all clean flash debug
print-%  : ; @echo $* = $($*)
rwildcard=$(foreach d,$(wildcard $1*),$(call rwildcard,$d/,$2) $(filter $(subst *,%,$2),$d))
sources := $(filter-out ./Middlewares/Third_Party/FreeRTOS/Source/portable/RVDS/ARM_CM4F/port.c, $(call rwildcard, ./, *.c))
objects := $(patsubst %.c,%.o,$(sources))
sources += ./MDK-ARM/startup_stm32f427xx_gcc.s
objects += ./MDK-ARM/startup_stm32f427xx_gcc.o
ifeq ($(CC),cc)
    CC := arm-none-eabi-gcc
endif
ifeq ($(OBJCOPY),)
    OBJCOPY := arm-none-eabi-objcopy
endif
CFLAGS += -O0 -g
CFLAGS += -march=armv7e-m -mcpu=cortex-m4 -mthumb -std=c11 -mfloat-abi=hard -mfpu=fpv4-sp-d16 -fsingle-precision-constant -finline-functions -ffunction-sections -fdata-sections
LDFLAGS += -march=armv7e-m -mcpu=cortex-m4 -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16 -lm -lc -lgcc -ffunction-sections -fdata-sections -specs=nosys.specs -Wl,--gc-sections,-Tstm32.ld,-Map,rm_infantry.map,-orm_infantry.elf,--no-wchar-size-warning
CFLAGS += -I ./Inc -I ./Drivers/STM32F4xx_HAL_Driver/Inc -I ./Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I ./Middlewares/Third_Party/FatFs/src/drivers -I ./Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -I ./Drivers/CMSIS/Device/ST/STM32F4xx/Include -I ./Middlewares/Third_Party/FatFs/src -I ./Middlewares/Third_Party/FreeRTOS/Source/include -I ./Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS -I ./Drivers/CMSIS/Include -I ./Drivers/CMSIS/Device -I ./MDK-ARM/app -I ./MDK-ARM/bsp -I ./MDK-ARM/rmlib -I ./MDK-ARM/other_lib
CFLAGS += -D USE_HAL_DRIVER -D STM32F427xx -D ARM_MATH_CM4 -D __FPU_PRESENT

all: rm_infantry.elf rm_infantry.hex

rm_infantry.hex: rm_infantry.elf
	$(OBJCOPY) -O ihex rm_infantry.elf rm_infantry.hex

rm_infantry.elf: $(objects)
	$(CC) $(objects) $(LDFLAGS)

%.o: %.c
	$(CC) $(CFLAGS) -c $^ -o $@

%.o: %.s
	$(CC) $(CFLAGS) -c $^ -o $@

clean:
	rm -rf rm_infantry.elf rm_infantry.map rm_infantry.hex $(objects)

flash:
	sudo JLinkExe -device STM32F427II -if SWD -speed 4000 -CommanderScript flash.jlink

debug:
	sudo JLinkGDBServer -select USB -device STM32F427II -endian little -if SWD -speed 4000 -halt -rtos /opt/SEGGER/JLink/GDBServer/RTOSPlugin_FreeRTOS.so
