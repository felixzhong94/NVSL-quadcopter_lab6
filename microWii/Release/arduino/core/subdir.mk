################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
/home/garza/.arduino15/packages/arduino/hardware/avr/1.6.10/cores/arduino/CDC.cpp \
/home/garza/.arduino15/packages/arduino/hardware/avr/1.6.10/cores/arduino/HardwareSerial.cpp \
/home/garza/.arduino15/packages/arduino/hardware/avr/1.6.10/cores/arduino/HardwareSerial0.cpp \
/home/garza/.arduino15/packages/arduino/hardware/avr/1.6.10/cores/arduino/HardwareSerial1.cpp \
/home/garza/.arduino15/packages/arduino/hardware/avr/1.6.10/cores/arduino/HardwareSerial2.cpp \
/home/garza/.arduino15/packages/arduino/hardware/avr/1.6.10/cores/arduino/HardwareSerial3.cpp \
/home/garza/.arduino15/packages/arduino/hardware/avr/1.6.10/cores/arduino/IPAddress.cpp \
/home/garza/.arduino15/packages/arduino/hardware/avr/1.6.10/cores/arduino/PluggableUSB.cpp \
/home/garza/.arduino15/packages/arduino/hardware/avr/1.6.10/cores/arduino/Print.cpp \
/home/garza/.arduino15/packages/arduino/hardware/avr/1.6.10/cores/arduino/Stream.cpp \
/home/garza/.arduino15/packages/arduino/hardware/avr/1.6.10/cores/arduino/Tone.cpp \
/home/garza/.arduino15/packages/arduino/hardware/avr/1.6.10/cores/arduino/USBCore.cpp \
/home/garza/.arduino15/packages/arduino/hardware/avr/1.6.10/cores/arduino/WMath.cpp \
/home/garza/.arduino15/packages/arduino/hardware/avr/1.6.10/cores/arduino/WString.cpp \
/home/garza/.arduino15/packages/arduino/hardware/avr/1.6.10/cores/arduino/abi.cpp \
/home/garza/.arduino15/packages/arduino/hardware/avr/1.6.10/cores/arduino/main.cpp \
/home/garza/.arduino15/packages/arduino/hardware/avr/1.6.10/cores/arduino/new.cpp 

C_SRCS += \
/home/garza/.arduino15/packages/arduino/hardware/avr/1.6.10/cores/arduino/WInterrupts.c \
/home/garza/.arduino15/packages/arduino/hardware/avr/1.6.10/cores/arduino/hooks.c \
/home/garza/.arduino15/packages/arduino/hardware/avr/1.6.10/cores/arduino/wiring.c \
/home/garza/.arduino15/packages/arduino/hardware/avr/1.6.10/cores/arduino/wiring_analog.c \
/home/garza/.arduino15/packages/arduino/hardware/avr/1.6.10/cores/arduino/wiring_digital.c \
/home/garza/.arduino15/packages/arduino/hardware/avr/1.6.10/cores/arduino/wiring_pulse.c \
/home/garza/.arduino15/packages/arduino/hardware/avr/1.6.10/cores/arduino/wiring_shift.c 

S_UPPER_SRCS += \
/home/garza/.arduino15/packages/arduino/hardware/avr/1.6.10/cores/arduino/wiring_pulse.S 

C_DEPS += \
./arduino/WInterrupts.c.d \
./arduino/hooks.c.d \
./arduino/wiring.c.d \
./arduino/wiring_analog.c.d \
./arduino/wiring_digital.c.d \
./arduino/wiring_pulse.c.d \
./arduino/wiring_shift.c.d 

S_UPPER_DEPS += \
./arduino/wiring_pulse.S.d 

AR_OBJ += \
./arduino/CDC.cpp.o \
./arduino/HardwareSerial.cpp.o \
./arduino/HardwareSerial0.cpp.o \
./arduino/HardwareSerial1.cpp.o \
./arduino/HardwareSerial2.cpp.o \
./arduino/HardwareSerial3.cpp.o \
./arduino/IPAddress.cpp.o \
./arduino/PluggableUSB.cpp.o \
./arduino/Print.cpp.o \
./arduino/Stream.cpp.o \
./arduino/Tone.cpp.o \
./arduino/USBCore.cpp.o \
./arduino/WInterrupts.c.o \
./arduino/WMath.cpp.o \
./arduino/WString.cpp.o \
./arduino/abi.cpp.o \
./arduino/hooks.c.o \
./arduino/main.cpp.o \
./arduino/new.cpp.o \
./arduino/wiring.c.o \
./arduino/wiring_analog.c.o \
./arduino/wiring_digital.c.o \
./arduino/wiring_pulse.S.o \
./arduino/wiring_pulse.c.o \
./arduino/wiring_shift.c.o 

CPP_DEPS += \
./arduino/CDC.cpp.d \
./arduino/HardwareSerial.cpp.d \
./arduino/HardwareSerial0.cpp.d \
./arduino/HardwareSerial1.cpp.d \
./arduino/HardwareSerial2.cpp.d \
./arduino/HardwareSerial3.cpp.d \
./arduino/IPAddress.cpp.d \
./arduino/PluggableUSB.cpp.d \
./arduino/Print.cpp.d \
./arduino/Stream.cpp.d \
./arduino/Tone.cpp.d \
./arduino/USBCore.cpp.d \
./arduino/WMath.cpp.d \
./arduino/WString.cpp.d \
./arduino/abi.cpp.d \
./arduino/main.cpp.d \
./arduino/new.cpp.d 


# Each subdirectory must supply rules for building sources it contributes
arduino/CDC.cpp.o: /home/garza/.arduino15/packages/arduino/hardware/avr/1.6.10/cores/arduino/CDC.cpp
	@echo 'Building file: $<'
	@echo 'Starting C++ compile'
	"/home/garza/arduino-1.6.8/hardware/tools/avr/bin/avr-g++" -c -g -Os -fno-exceptions -ffunction-sections -fdata-sections -fno-threadsafe-statics -MMD -mmcu=atmega128rfa1 -DF_CPU=16000000L -DARDUINO=10608 -DARDUINO_ATMEGA128RFA1_DEV_BOARD -DARDUINO_ARCH_AVR     -I"/home/garza/.arduino15/packages/arduino/hardware/avr/1.6.10/cores/arduino" -I"/home/garza/Arduino/hardware/sparkfun/avr/variants/rf128" -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -D__IN_ECLIPSE__=1 -x c++ "$<"  -o  "$@"   -Wall
	@echo 'Finished building: $<'
	@echo ' '

arduino/HardwareSerial.cpp.o: /home/garza/.arduino15/packages/arduino/hardware/avr/1.6.10/cores/arduino/HardwareSerial.cpp
	@echo 'Building file: $<'
	@echo 'Starting C++ compile'
	"/home/garza/arduino-1.6.8/hardware/tools/avr/bin/avr-g++" -c -g -Os -fno-exceptions -ffunction-sections -fdata-sections -fno-threadsafe-statics -MMD -mmcu=atmega128rfa1 -DF_CPU=16000000L -DARDUINO=10608 -DARDUINO_ATMEGA128RFA1_DEV_BOARD -DARDUINO_ARCH_AVR     -I"/home/garza/.arduino15/packages/arduino/hardware/avr/1.6.10/cores/arduino" -I"/home/garza/Arduino/hardware/sparkfun/avr/variants/rf128" -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -D__IN_ECLIPSE__=1 -x c++ "$<"  -o  "$@"   -Wall
	@echo 'Finished building: $<'
	@echo ' '

arduino/HardwareSerial0.cpp.o: /home/garza/.arduino15/packages/arduino/hardware/avr/1.6.10/cores/arduino/HardwareSerial0.cpp
	@echo 'Building file: $<'
	@echo 'Starting C++ compile'
	"/home/garza/arduino-1.6.8/hardware/tools/avr/bin/avr-g++" -c -g -Os -fno-exceptions -ffunction-sections -fdata-sections -fno-threadsafe-statics -MMD -mmcu=atmega128rfa1 -DF_CPU=16000000L -DARDUINO=10608 -DARDUINO_ATMEGA128RFA1_DEV_BOARD -DARDUINO_ARCH_AVR     -I"/home/garza/.arduino15/packages/arduino/hardware/avr/1.6.10/cores/arduino" -I"/home/garza/Arduino/hardware/sparkfun/avr/variants/rf128" -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -D__IN_ECLIPSE__=1 -x c++ "$<"  -o  "$@"   -Wall
	@echo 'Finished building: $<'
	@echo ' '

arduino/HardwareSerial1.cpp.o: /home/garza/.arduino15/packages/arduino/hardware/avr/1.6.10/cores/arduino/HardwareSerial1.cpp
	@echo 'Building file: $<'
	@echo 'Starting C++ compile'
	"/home/garza/arduino-1.6.8/hardware/tools/avr/bin/avr-g++" -c -g -Os -fno-exceptions -ffunction-sections -fdata-sections -fno-threadsafe-statics -MMD -mmcu=atmega128rfa1 -DF_CPU=16000000L -DARDUINO=10608 -DARDUINO_ATMEGA128RFA1_DEV_BOARD -DARDUINO_ARCH_AVR     -I"/home/garza/.arduino15/packages/arduino/hardware/avr/1.6.10/cores/arduino" -I"/home/garza/Arduino/hardware/sparkfun/avr/variants/rf128" -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -D__IN_ECLIPSE__=1 -x c++ "$<"  -o  "$@"   -Wall
	@echo 'Finished building: $<'
	@echo ' '

arduino/HardwareSerial2.cpp.o: /home/garza/.arduino15/packages/arduino/hardware/avr/1.6.10/cores/arduino/HardwareSerial2.cpp
	@echo 'Building file: $<'
	@echo 'Starting C++ compile'
	"/home/garza/arduino-1.6.8/hardware/tools/avr/bin/avr-g++" -c -g -Os -fno-exceptions -ffunction-sections -fdata-sections -fno-threadsafe-statics -MMD -mmcu=atmega128rfa1 -DF_CPU=16000000L -DARDUINO=10608 -DARDUINO_ATMEGA128RFA1_DEV_BOARD -DARDUINO_ARCH_AVR     -I"/home/garza/.arduino15/packages/arduino/hardware/avr/1.6.10/cores/arduino" -I"/home/garza/Arduino/hardware/sparkfun/avr/variants/rf128" -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -D__IN_ECLIPSE__=1 -x c++ "$<"  -o  "$@"   -Wall
	@echo 'Finished building: $<'
	@echo ' '

arduino/HardwareSerial3.cpp.o: /home/garza/.arduino15/packages/arduino/hardware/avr/1.6.10/cores/arduino/HardwareSerial3.cpp
	@echo 'Building file: $<'
	@echo 'Starting C++ compile'
	"/home/garza/arduino-1.6.8/hardware/tools/avr/bin/avr-g++" -c -g -Os -fno-exceptions -ffunction-sections -fdata-sections -fno-threadsafe-statics -MMD -mmcu=atmega128rfa1 -DF_CPU=16000000L -DARDUINO=10608 -DARDUINO_ATMEGA128RFA1_DEV_BOARD -DARDUINO_ARCH_AVR     -I"/home/garza/.arduino15/packages/arduino/hardware/avr/1.6.10/cores/arduino" -I"/home/garza/Arduino/hardware/sparkfun/avr/variants/rf128" -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -D__IN_ECLIPSE__=1 -x c++ "$<"  -o  "$@"   -Wall
	@echo 'Finished building: $<'
	@echo ' '

arduino/IPAddress.cpp.o: /home/garza/.arduino15/packages/arduino/hardware/avr/1.6.10/cores/arduino/IPAddress.cpp
	@echo 'Building file: $<'
	@echo 'Starting C++ compile'
	"/home/garza/arduino-1.6.8/hardware/tools/avr/bin/avr-g++" -c -g -Os -fno-exceptions -ffunction-sections -fdata-sections -fno-threadsafe-statics -MMD -mmcu=atmega128rfa1 -DF_CPU=16000000L -DARDUINO=10608 -DARDUINO_ATMEGA128RFA1_DEV_BOARD -DARDUINO_ARCH_AVR     -I"/home/garza/.arduino15/packages/arduino/hardware/avr/1.6.10/cores/arduino" -I"/home/garza/Arduino/hardware/sparkfun/avr/variants/rf128" -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -D__IN_ECLIPSE__=1 -x c++ "$<"  -o  "$@"   -Wall
	@echo 'Finished building: $<'
	@echo ' '

arduino/PluggableUSB.cpp.o: /home/garza/.arduino15/packages/arduino/hardware/avr/1.6.10/cores/arduino/PluggableUSB.cpp
	@echo 'Building file: $<'
	@echo 'Starting C++ compile'
	"/home/garza/arduino-1.6.8/hardware/tools/avr/bin/avr-g++" -c -g -Os -fno-exceptions -ffunction-sections -fdata-sections -fno-threadsafe-statics -MMD -mmcu=atmega128rfa1 -DF_CPU=16000000L -DARDUINO=10608 -DARDUINO_ATMEGA128RFA1_DEV_BOARD -DARDUINO_ARCH_AVR     -I"/home/garza/.arduino15/packages/arduino/hardware/avr/1.6.10/cores/arduino" -I"/home/garza/Arduino/hardware/sparkfun/avr/variants/rf128" -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -D__IN_ECLIPSE__=1 -x c++ "$<"  -o  "$@"   -Wall
	@echo 'Finished building: $<'
	@echo ' '

arduino/Print.cpp.o: /home/garza/.arduino15/packages/arduino/hardware/avr/1.6.10/cores/arduino/Print.cpp
	@echo 'Building file: $<'
	@echo 'Starting C++ compile'
	"/home/garza/arduino-1.6.8/hardware/tools/avr/bin/avr-g++" -c -g -Os -fno-exceptions -ffunction-sections -fdata-sections -fno-threadsafe-statics -MMD -mmcu=atmega128rfa1 -DF_CPU=16000000L -DARDUINO=10608 -DARDUINO_ATMEGA128RFA1_DEV_BOARD -DARDUINO_ARCH_AVR     -I"/home/garza/.arduino15/packages/arduino/hardware/avr/1.6.10/cores/arduino" -I"/home/garza/Arduino/hardware/sparkfun/avr/variants/rf128" -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -D__IN_ECLIPSE__=1 -x c++ "$<"  -o  "$@"   -Wall
	@echo 'Finished building: $<'
	@echo ' '

arduino/Stream.cpp.o: /home/garza/.arduino15/packages/arduino/hardware/avr/1.6.10/cores/arduino/Stream.cpp
	@echo 'Building file: $<'
	@echo 'Starting C++ compile'
	"/home/garza/arduino-1.6.8/hardware/tools/avr/bin/avr-g++" -c -g -Os -fno-exceptions -ffunction-sections -fdata-sections -fno-threadsafe-statics -MMD -mmcu=atmega128rfa1 -DF_CPU=16000000L -DARDUINO=10608 -DARDUINO_ATMEGA128RFA1_DEV_BOARD -DARDUINO_ARCH_AVR     -I"/home/garza/.arduino15/packages/arduino/hardware/avr/1.6.10/cores/arduino" -I"/home/garza/Arduino/hardware/sparkfun/avr/variants/rf128" -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -D__IN_ECLIPSE__=1 -x c++ "$<"  -o  "$@"   -Wall
	@echo 'Finished building: $<'
	@echo ' '

arduino/Tone.cpp.o: /home/garza/.arduino15/packages/arduino/hardware/avr/1.6.10/cores/arduino/Tone.cpp
	@echo 'Building file: $<'
	@echo 'Starting C++ compile'
	"/home/garza/arduino-1.6.8/hardware/tools/avr/bin/avr-g++" -c -g -Os -fno-exceptions -ffunction-sections -fdata-sections -fno-threadsafe-statics -MMD -mmcu=atmega128rfa1 -DF_CPU=16000000L -DARDUINO=10608 -DARDUINO_ATMEGA128RFA1_DEV_BOARD -DARDUINO_ARCH_AVR     -I"/home/garza/.arduino15/packages/arduino/hardware/avr/1.6.10/cores/arduino" -I"/home/garza/Arduino/hardware/sparkfun/avr/variants/rf128" -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -D__IN_ECLIPSE__=1 -x c++ "$<"  -o  "$@"   -Wall
	@echo 'Finished building: $<'
	@echo ' '

arduino/USBCore.cpp.o: /home/garza/.arduino15/packages/arduino/hardware/avr/1.6.10/cores/arduino/USBCore.cpp
	@echo 'Building file: $<'
	@echo 'Starting C++ compile'
	"/home/garza/arduino-1.6.8/hardware/tools/avr/bin/avr-g++" -c -g -Os -fno-exceptions -ffunction-sections -fdata-sections -fno-threadsafe-statics -MMD -mmcu=atmega128rfa1 -DF_CPU=16000000L -DARDUINO=10608 -DARDUINO_ATMEGA128RFA1_DEV_BOARD -DARDUINO_ARCH_AVR     -I"/home/garza/.arduino15/packages/arduino/hardware/avr/1.6.10/cores/arduino" -I"/home/garza/Arduino/hardware/sparkfun/avr/variants/rf128" -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -D__IN_ECLIPSE__=1 -x c++ "$<"  -o  "$@"   -Wall
	@echo 'Finished building: $<'
	@echo ' '

arduino/WInterrupts.c.o: /home/garza/.arduino15/packages/arduino/hardware/avr/1.6.10/cores/arduino/WInterrupts.c
	@echo 'Building file: $<'
	@echo 'Starting C compile'
	"/home/garza/arduino-1.6.8/hardware/tools/avr/bin/avr-gcc" -c -g -Os -ffunction-sections -fdata-sections -MMD -mmcu=atmega128rfa1 -DF_CPU=16000000L -DARDUINO=10608 -DARDUINO_ATMEGA128RFA1_DEV_BOARD -DARDUINO_ARCH_AVR     -I"/home/garza/.arduino15/packages/arduino/hardware/avr/1.6.10/cores/arduino" -I"/home/garza/Arduino/hardware/sparkfun/avr/variants/rf128" -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -D__IN_ECLIPSE__=1 "$<"  -o  "$@"   -Wall
	@echo 'Finished building: $<'
	@echo ' '

arduino/WMath.cpp.o: /home/garza/.arduino15/packages/arduino/hardware/avr/1.6.10/cores/arduino/WMath.cpp
	@echo 'Building file: $<'
	@echo 'Starting C++ compile'
	"/home/garza/arduino-1.6.8/hardware/tools/avr/bin/avr-g++" -c -g -Os -fno-exceptions -ffunction-sections -fdata-sections -fno-threadsafe-statics -MMD -mmcu=atmega128rfa1 -DF_CPU=16000000L -DARDUINO=10608 -DARDUINO_ATMEGA128RFA1_DEV_BOARD -DARDUINO_ARCH_AVR     -I"/home/garza/.arduino15/packages/arduino/hardware/avr/1.6.10/cores/arduino" -I"/home/garza/Arduino/hardware/sparkfun/avr/variants/rf128" -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -D__IN_ECLIPSE__=1 -x c++ "$<"  -o  "$@"   -Wall
	@echo 'Finished building: $<'
	@echo ' '

arduino/WString.cpp.o: /home/garza/.arduino15/packages/arduino/hardware/avr/1.6.10/cores/arduino/WString.cpp
	@echo 'Building file: $<'
	@echo 'Starting C++ compile'
	"/home/garza/arduino-1.6.8/hardware/tools/avr/bin/avr-g++" -c -g -Os -fno-exceptions -ffunction-sections -fdata-sections -fno-threadsafe-statics -MMD -mmcu=atmega128rfa1 -DF_CPU=16000000L -DARDUINO=10608 -DARDUINO_ATMEGA128RFA1_DEV_BOARD -DARDUINO_ARCH_AVR     -I"/home/garza/.arduino15/packages/arduino/hardware/avr/1.6.10/cores/arduino" -I"/home/garza/Arduino/hardware/sparkfun/avr/variants/rf128" -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -D__IN_ECLIPSE__=1 -x c++ "$<"  -o  "$@"   -Wall
	@echo 'Finished building: $<'
	@echo ' '

arduino/abi.cpp.o: /home/garza/.arduino15/packages/arduino/hardware/avr/1.6.10/cores/arduino/abi.cpp
	@echo 'Building file: $<'
	@echo 'Starting C++ compile'
	"/home/garza/arduino-1.6.8/hardware/tools/avr/bin/avr-g++" -c -g -Os -fno-exceptions -ffunction-sections -fdata-sections -fno-threadsafe-statics -MMD -mmcu=atmega128rfa1 -DF_CPU=16000000L -DARDUINO=10608 -DARDUINO_ATMEGA128RFA1_DEV_BOARD -DARDUINO_ARCH_AVR     -I"/home/garza/.arduino15/packages/arduino/hardware/avr/1.6.10/cores/arduino" -I"/home/garza/Arduino/hardware/sparkfun/avr/variants/rf128" -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -D__IN_ECLIPSE__=1 -x c++ "$<"  -o  "$@"   -Wall
	@echo 'Finished building: $<'
	@echo ' '

arduino/hooks.c.o: /home/garza/.arduino15/packages/arduino/hardware/avr/1.6.10/cores/arduino/hooks.c
	@echo 'Building file: $<'
	@echo 'Starting C compile'
	"/home/garza/arduino-1.6.8/hardware/tools/avr/bin/avr-gcc" -c -g -Os -ffunction-sections -fdata-sections -MMD -mmcu=atmega128rfa1 -DF_CPU=16000000L -DARDUINO=10608 -DARDUINO_ATMEGA128RFA1_DEV_BOARD -DARDUINO_ARCH_AVR     -I"/home/garza/.arduino15/packages/arduino/hardware/avr/1.6.10/cores/arduino" -I"/home/garza/Arduino/hardware/sparkfun/avr/variants/rf128" -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -D__IN_ECLIPSE__=1 "$<"  -o  "$@"   -Wall
	@echo 'Finished building: $<'
	@echo ' '

arduino/main.cpp.o: /home/garza/.arduino15/packages/arduino/hardware/avr/1.6.10/cores/arduino/main.cpp
	@echo 'Building file: $<'
	@echo 'Starting C++ compile'
	"/home/garza/arduino-1.6.8/hardware/tools/avr/bin/avr-g++" -c -g -Os -fno-exceptions -ffunction-sections -fdata-sections -fno-threadsafe-statics -MMD -mmcu=atmega128rfa1 -DF_CPU=16000000L -DARDUINO=10608 -DARDUINO_ATMEGA128RFA1_DEV_BOARD -DARDUINO_ARCH_AVR     -I"/home/garza/.arduino15/packages/arduino/hardware/avr/1.6.10/cores/arduino" -I"/home/garza/Arduino/hardware/sparkfun/avr/variants/rf128" -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -D__IN_ECLIPSE__=1 -x c++ "$<"  -o  "$@"   -Wall
	@echo 'Finished building: $<'
	@echo ' '

arduino/new.cpp.o: /home/garza/.arduino15/packages/arduino/hardware/avr/1.6.10/cores/arduino/new.cpp
	@echo 'Building file: $<'
	@echo 'Starting C++ compile'
	"/home/garza/arduino-1.6.8/hardware/tools/avr/bin/avr-g++" -c -g -Os -fno-exceptions -ffunction-sections -fdata-sections -fno-threadsafe-statics -MMD -mmcu=atmega128rfa1 -DF_CPU=16000000L -DARDUINO=10608 -DARDUINO_ATMEGA128RFA1_DEV_BOARD -DARDUINO_ARCH_AVR     -I"/home/garza/.arduino15/packages/arduino/hardware/avr/1.6.10/cores/arduino" -I"/home/garza/Arduino/hardware/sparkfun/avr/variants/rf128" -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -D__IN_ECLIPSE__=1 -x c++ "$<"  -o  "$@"   -Wall
	@echo 'Finished building: $<'
	@echo ' '

arduino/wiring.c.o: /home/garza/.arduino15/packages/arduino/hardware/avr/1.6.10/cores/arduino/wiring.c
	@echo 'Building file: $<'
	@echo 'Starting C compile'
	"/home/garza/arduino-1.6.8/hardware/tools/avr/bin/avr-gcc" -c -g -Os -ffunction-sections -fdata-sections -MMD -mmcu=atmega128rfa1 -DF_CPU=16000000L -DARDUINO=10608 -DARDUINO_ATMEGA128RFA1_DEV_BOARD -DARDUINO_ARCH_AVR     -I"/home/garza/.arduino15/packages/arduino/hardware/avr/1.6.10/cores/arduino" -I"/home/garza/Arduino/hardware/sparkfun/avr/variants/rf128" -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -D__IN_ECLIPSE__=1 "$<"  -o  "$@"   -Wall
	@echo 'Finished building: $<'
	@echo ' '

arduino/wiring_analog.c.o: /home/garza/.arduino15/packages/arduino/hardware/avr/1.6.10/cores/arduino/wiring_analog.c
	@echo 'Building file: $<'
	@echo 'Starting C compile'
	"/home/garza/arduino-1.6.8/hardware/tools/avr/bin/avr-gcc" -c -g -Os -ffunction-sections -fdata-sections -MMD -mmcu=atmega128rfa1 -DF_CPU=16000000L -DARDUINO=10608 -DARDUINO_ATMEGA128RFA1_DEV_BOARD -DARDUINO_ARCH_AVR     -I"/home/garza/.arduino15/packages/arduino/hardware/avr/1.6.10/cores/arduino" -I"/home/garza/Arduino/hardware/sparkfun/avr/variants/rf128" -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -D__IN_ECLIPSE__=1 "$<"  -o  "$@"   -Wall
	@echo 'Finished building: $<'
	@echo ' '

arduino/wiring_digital.c.o: /home/garza/.arduino15/packages/arduino/hardware/avr/1.6.10/cores/arduino/wiring_digital.c
	@echo 'Building file: $<'
	@echo 'Starting C compile'
	"/home/garza/arduino-1.6.8/hardware/tools/avr/bin/avr-gcc" -c -g -Os -ffunction-sections -fdata-sections -MMD -mmcu=atmega128rfa1 -DF_CPU=16000000L -DARDUINO=10608 -DARDUINO_ATMEGA128RFA1_DEV_BOARD -DARDUINO_ARCH_AVR     -I"/home/garza/.arduino15/packages/arduino/hardware/avr/1.6.10/cores/arduino" -I"/home/garza/Arduino/hardware/sparkfun/avr/variants/rf128" -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -D__IN_ECLIPSE__=1 "$<"  -o  "$@"   -Wall
	@echo 'Finished building: $<'
	@echo ' '

arduino/wiring_pulse.S.o: /home/garza/.arduino15/packages/arduino/hardware/avr/1.6.10/cores/arduino/wiring_pulse.S
	@echo 'Building file: $<'
	@echo 'Starting S compile'
	"/home/garza/arduino-1.6.8/hardware/tools/avr/bin/avr-gcc" -c -g -x assembler-with-cpp -mmcu=atmega128rfa1 -DF_CPU=16000000L -DARDUINO=10608 -DARDUINO_ATMEGA128RFA1_DEV_BOARD -DARDUINO_ARCH_AVR     -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -D__IN_ECLIPSE__=1 "$<"  -o  "$@"   -Wall
	@echo 'Finished building: $<'
	@echo ' '

arduino/wiring_pulse.c.o: /home/garza/.arduino15/packages/arduino/hardware/avr/1.6.10/cores/arduino/wiring_pulse.c
	@echo 'Building file: $<'
	@echo 'Starting C compile'
	"/home/garza/arduino-1.6.8/hardware/tools/avr/bin/avr-gcc" -c -g -Os -ffunction-sections -fdata-sections -MMD -mmcu=atmega128rfa1 -DF_CPU=16000000L -DARDUINO=10608 -DARDUINO_ATMEGA128RFA1_DEV_BOARD -DARDUINO_ARCH_AVR     -I"/home/garza/.arduino15/packages/arduino/hardware/avr/1.6.10/cores/arduino" -I"/home/garza/Arduino/hardware/sparkfun/avr/variants/rf128" -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -D__IN_ECLIPSE__=1 "$<"  -o  "$@"   -Wall
	@echo 'Finished building: $<'
	@echo ' '

arduino/wiring_shift.c.o: /home/garza/.arduino15/packages/arduino/hardware/avr/1.6.10/cores/arduino/wiring_shift.c
	@echo 'Building file: $<'
	@echo 'Starting C compile'
	"/home/garza/arduino-1.6.8/hardware/tools/avr/bin/avr-gcc" -c -g -Os -ffunction-sections -fdata-sections -MMD -mmcu=atmega128rfa1 -DF_CPU=16000000L -DARDUINO=10608 -DARDUINO_ATMEGA128RFA1_DEV_BOARD -DARDUINO_ARCH_AVR     -I"/home/garza/.arduino15/packages/arduino/hardware/avr/1.6.10/cores/arduino" -I"/home/garza/Arduino/hardware/sparkfun/avr/variants/rf128" -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -D__IN_ECLIPSE__=1 "$<"  -o  "$@"   -Wall
	@echo 'Finished building: $<'
	@echo ' '


