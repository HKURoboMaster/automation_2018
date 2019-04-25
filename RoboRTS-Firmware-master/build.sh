#!/bin/bash

INCLUDE_FLAGS="-I ./Inc -I ./Drivers/STM32F4xx_HAL_Driver/Inc -I ./Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I ./Middlewares/Third_Party/FatFs/src/drivers -I ./Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -I ./Drivers/CMSIS/Device/ST/STM32F4xx/Include -I ./Middlewares/Third_Party/FatFs/src -I ./Middlewares/Third_Party/FreeRTOS/Source/include -I ./Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS -I ./Drivers/CMSIS/Include -I ./Drivers/CMSIS/Device -I ./MDK-ARM/app -I ./MDK-ARM/bsp -I ./MDK-ARM/rmlib -I ./MDK-ARM/other_lib"

DEFINE_FLAGS="-D USE_HAL_DRIVER -D STM32F427xx -D ARM_MATH_CM4 -D __FPU_PRESENT"

if [ -z "$TOOLCHAIN_PREFIX" ]
then
    TOOLCHAIN_PREFIX=arm-none-eabi-
fi

[ ! -d "./build" ] && mkdir build

compile() {
    echo "==> Compiling $1";
    "$TOOLCHAIN_PREFIX"gcc -g -O0 -c $1 -o build/$(echo $1 | sed 's/\.[cs]$/.o/g' | rev | cut -d'/' -f1 | rev) $INCLUDE_FLAGS $DEFINE_FLAGS -march=armv7e-m -mcpu=cortex-m4 -mthumb -std=c11 -mfloat-abi=hard -mfpu=fpv4-sp-d16 -fsingle-precision-constant -finline-functions -ffunction-sections -fdata-sections
}

threads=1
if [ -x "$(command -v git)" ]; then
    threads=$(nproc)
fi
for i in $(find . | grep '\.c$') ./MDK-ARM/startup_stm32f427xx_gcc.s;
do
    ((counter=counter%threads)); ((counter++==0)) && wait
    compile $i &
done
wait
echo "==> Linking";
"$TOOLCHAIN_PREFIX"gcc build/*.o -g -O0 -march=armv7e-m -mcpu=cortex-m4 -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16 -lm -lc -lgcc -ffunction-sections -fdata-sections -specs=nosys.specs -Wl,--gc-sections,-Tstm32.ld,-Map,output.map,-ooutput.elf,--no-wchar-size-warning
echo "==> Generating HEX";
"$TOOLCHAIN_PREFIX"objcopy -O ihex output.elf output.hex
echo "==> Generating BIN";
"$TOOLCHAIN_PREFIX"objcopy -O binary output.elf output.bin
