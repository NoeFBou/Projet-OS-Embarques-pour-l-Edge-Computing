INCLUDES=-I.
INCLUDES+= -Iarduino-libs-and-core/cores/arduino
INCLUDES+= -Iarduino-libs-and-core/libraries/Wire/src
INCLUDES+= -Iarduino-libs-and-core/libraries/Wire/src/utility
INCLUDES+= -Iarduino-libs-and-core/variants/standard
INCLUDES+= -IFreeRTOS-Kernel/include
INCLUDES+= -IFreeRTOS-Kernel/portable/GCC/ATMega328/


vpath %.cpp src
vpath %.cpp arduino-libs-and-core/libraries/Wire/src
vpath %c arduino-libs-and-core/libraries/Wire/src/utility
# List C source files here. (C dependencies are automatically generated.)
vpath %c FreeRTOS-Kernel/
vpath %c FreeRTOS-Kernel/portable/MemMang
vpath %c FreeRTOS-Kernel/portable/GCC/ATMega328/

BUILD_DIR=build

CC=avr-gcc
CPP=avr-g++

MMCU=-mmcu=atmega328p

CFLAGS= -g -Os -w -std=gnu11 -ffunction-sections -fdata-sections -MMD -flto -fno-fat-lto-objects ${MMCU} -DF_CPU=16000000L -DARDUINO=10812 -DARDUINO_AVR_UNO -DARDUINO_ARCH_AVR
CPPFLAGS= -g -Os -w -std=gnu++11 -fpermissive -fno-exceptions -ffunction-sections -fdata-sections -fno-threadsafe-statics -Wno-error=narrowing -MMD -flto -w -x c++ -CC ${MMCU} -DF_CPU=16000000L -DARDUINO=10812 -DARDUINO_AVR_UNO -DARDUINO_ARCH_AVR

PROGRAM=coffre_fort


all: ${PROGRAM}.elf ${PROGRAM}.hex 
	rm -f *.d


${PROGRAM}.elf: ${BUILD_DIR}/tasks.o ${BUILD_DIR}/queue.o ${BUILD_DIR}/list.o ${BUILD_DIR}/croutine.o ${BUILD_DIR}/heap_1.o ${BUILD_DIR}/port.o ${BUILD_DIR}/Wire.o ${BUILD_DIR}/main.o ${BUILD_DIR}/twi.o 
	${CPP} -w -Os -g -flto -fuse-linker-plugin -Wl,--gc-sections ${INCLUDES} ${MMCU} $^ -o ${BUILD_DIR}/$@ -static -L./arduino-libs-and-core -lres


${BUILD_DIR}/%.o: %.c
	mkdir -p ${BUILD_DIR}/
	${CC} -c ${CFLAGS} ${INCLUDES} $^
	mv *.o ${BUILD_DIR}/

${BUILD_DIR}/%.o: %.cpp
	mkdir -p ${BUILD_DIR}/
	${CPP} -c ${CPPFLAGS} ${INCLUDES} $^
	mv *.o ${BUILD_DIR}/

OBJCOPY=avr-objcopy
 
${PROGRAM}.hex: ${BUILD_DIR}/${PROGRAM}.elf
	${OBJCOPY} -O ihex -R .eeprom  ${BUILD_DIR}/${PROGRAM}.elf ${BUILD_DIR}/$@

PORT=/dev/ttyACM0
 
upload: ${BUILD_DIR}/${PROGRAM}.hex
	avrdude -F -V -c arduino -p ATMEGA328P -P ${PORT} -b 115200 -U flash:w:${BUILD_DIR}/${PROGRAM}.hex;

clean:
	rm -Rf ${BUILD_DIR}
