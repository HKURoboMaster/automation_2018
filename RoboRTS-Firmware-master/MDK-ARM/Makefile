.PHONY: all clean flash debug

all:
	./build.sh

clean:
	rm -rf output.elf output.map output.hex *.o

flash:
	sudo /opt/SEGGER/JLink/JLinkExe -device STM32F427II -if SWD -speed 4000 

debug:
	sudo /opt/SEGGER/JLink/JLinkGDBServer -select USB -device STM32F427II -endian little -if SWD -speed 4000 -halt -rtos /opt/SEGGER/JLink/GDBServer/RTOSPlugin_FreeRTOS.so
