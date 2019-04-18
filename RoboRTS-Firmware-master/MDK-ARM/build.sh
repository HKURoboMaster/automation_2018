#!/bin/bash

INCLUDE_FLAGS="-I ../Inc -I ../Drivers/STM32F4xx_HAL_Driver/Inc -I ../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I ../Middlewares/Third_Party/FatFs/src/drivers -I ../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -I ../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I ../Middlewares/Third_Party/FatFs/src -I ../Middlewares/Third_Party/FreeRTOS/Source/include -I ../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS -I ../Drivers/CMSIS/Include -I ./app -I ./bsp -I ./rmlib -I ./other_lib -I ../Drivers/CMSIS/Device"

DEFINE_FLAGS="-D USE_HAL_DRIVER -D STM32F427xx -D ARM_MATH_CM4 -D __FPU_PRESENT"

compile() {
    echo "==> Compiling $1";
    arm-none-eabi-gcc -g -O0 -c $1 $INCLUDE_FLAGS $DEFINE_FLAGS -march=armv7e-m -mcpu=cortex-m4 -mthumb -std=c11 -mfloat-abi=hard -mfpu=fpv4-sp-d16 -specs=nano.specs -specs=nosys.specs -fsingle-precision-constant -finline-functions -ffunction-sections -fdata-sections
}

threads=4
for i in $(find .. | grep '\.c$') ./startup_stm32f427xx.s;
do
    ((counter=counter%threads)); ((counter++==0)) && wait
    compile $i &
done
wait
echo "==> Linking";
arm-none-eabi-gcc *.o -g -O0 -march=armv7e-m -mcpu=cortex-m4 -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16 -lm -lc -lgcc -ffunction-sections -fdata-sections -specs=nano.specs -specs=nosys.specs -Wl,--gc-sections,-Tstm32.ld,-Map,output.map,-ooutput.elf
echo "==> Generating HEX";
arm-none-eabi-objcopy -O ihex output.elf output.hex
echo "==> Generating BIN";
arm-none-eabi-objcopy -O binary output.elf output.bin
