#!/bin/bash
for i in $(find .. | grep '\.c$') ./startup_stm32f427xx.s;
do
    echo "==> Compiling $i";
    arm-none-eabi-gcc -c $i -I ../Inc -I ../Drivers/STM32F4xx_HAL_Driver/Inc -I ../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I ../Middlewares/Third_Party/FatFs/src/drivers -I ../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -I ../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I ../Middlewares/Third_Party/FatFs/src -I ../Middlewares/Third_Party/FreeRTOS/Source/include -I ../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS -I ../Drivers/CMSIS/Include -I ./app -I ./bsp -I ./rmlib -I ./other_lib -I ../Drivers/CMSIS/Device -D USE_HAL_DRIVER -D STM32F427xx -D ARM_MATH_CM4 -D __FPU_PRESENT -march=armv7e-m -mcpu=cortex-m4 -mthumb -std=c11 -mfloat-abi=hard -mfpu=fpv4-sp-d16 -specs=nano.specs -specs=nosys.specs;
done
echo "==> Linking";
arm-none-eabi-gcc *.o -march=armv7e-m -mcpu=cortex-m4 -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16 -nostartfiles -Xlinker -ooutput.elf -Xlinker -M -Xlinker -Map=output.map -lm -specs=nano.specs -specs=nosys.specs

