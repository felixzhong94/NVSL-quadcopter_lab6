################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../.ino.cpp \
../Alarms.cpp \
../EEPROM.cpp \
../IMU.cpp \
../MultiWii.cpp \
../Output.cpp \
../Protocol.cpp \
../RX.cpp \
../Sensors.cpp \
../Serial.cpp 

INO_SRCS += \
../MultiWii.ino 

INO_DEPS += \
./MultiWii.ino.d 

CPP_DEPS += \
./.ino.cpp.d \
./Alarms.cpp.d \
./EEPROM.cpp.d \
./IMU.cpp.d \
./MultiWii.cpp.d \
./Output.cpp.d \
./Protocol.cpp.d \
./RX.cpp.d \
./Sensors.cpp.d \
./Serial.cpp.d 

LINK_OBJ += \
./.ino.cpp.o \
./Alarms.cpp.o \
./EEPROM.cpp.o \
./IMU.cpp.o \
./MultiWii.cpp.o \
./Output.cpp.o \
./Protocol.cpp.o \
./RX.cpp.o \
./Sensors.cpp.o \
./Serial.cpp.o 


# Each subdirectory must supply rules for building sources it contributes
.ino.cpp.o: ../.ino.cpp
	@echo 'Building file: $<'
	@echo 'Starting C++ compile'
	"/home/garza/arduino-1.6.8/hardware/tools/avr/bin/avr-g++" -c -g -Os -fno-exceptions -ffunction-sections -fdata-sections -fno-threadsafe-statics -MMD -mmcu=atmega128rfa1 -DF_CPU=16000000L -DARDUINO=10608 -DARDUINO_ATMEGA128RFA1_DEV_BOARD -DARDUINO_ARCH_AVR     -I"/home/garza/.arduino15/packages/arduino/hardware/avr/1.6.10/cores/arduino" -I"/home/garza/Arduino/hardware/sparkfun/avr/variants/rf128" -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -D__IN_ECLIPSE__=1 -x c++ "$<"  -o  "$@"   -Wall
	@echo 'Finished building: $<'
	@echo ' '

Alarms.cpp.o: ../Alarms.cpp
	@echo 'Building file: $<'
	@echo 'Starting C++ compile'
	"/home/garza/arduino-1.6.8/hardware/tools/avr/bin/avr-g++" -c -g -Os -fno-exceptions -ffunction-sections -fdata-sections -fno-threadsafe-statics -MMD -mmcu=atmega128rfa1 -DF_CPU=16000000L -DARDUINO=10608 -DARDUINO_ATMEGA128RFA1_DEV_BOARD -DARDUINO_ARCH_AVR     -I"/home/garza/.arduino15/packages/arduino/hardware/avr/1.6.10/cores/arduino" -I"/home/garza/Arduino/hardware/sparkfun/avr/variants/rf128" -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -D__IN_ECLIPSE__=1 -x c++ "$<"  -o  "$@"   -Wall
	@echo 'Finished building: $<'
	@echo ' '

EEPROM.cpp.o: ../EEPROM.cpp
	@echo 'Building file: $<'
	@echo 'Starting C++ compile'
	"/home/garza/arduino-1.6.8/hardware/tools/avr/bin/avr-g++" -c -g -Os -fno-exceptions -ffunction-sections -fdata-sections -fno-threadsafe-statics -MMD -mmcu=atmega128rfa1 -DF_CPU=16000000L -DARDUINO=10608 -DARDUINO_ATMEGA128RFA1_DEV_BOARD -DARDUINO_ARCH_AVR     -I"/home/garza/.arduino15/packages/arduino/hardware/avr/1.6.10/cores/arduino" -I"/home/garza/Arduino/hardware/sparkfun/avr/variants/rf128" -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -D__IN_ECLIPSE__=1 -x c++ "$<"  -o  "$@"   -Wall
	@echo 'Finished building: $<'
	@echo ' '

IMU.cpp.o: ../IMU.cpp
	@echo 'Building file: $<'
	@echo 'Starting C++ compile'
	"/home/garza/arduino-1.6.8/hardware/tools/avr/bin/avr-g++" -c -g -Os -fno-exceptions -ffunction-sections -fdata-sections -fno-threadsafe-statics -MMD -mmcu=atmega128rfa1 -DF_CPU=16000000L -DARDUINO=10608 -DARDUINO_ATMEGA128RFA1_DEV_BOARD -DARDUINO_ARCH_AVR     -I"/home/garza/.arduino15/packages/arduino/hardware/avr/1.6.10/cores/arduino" -I"/home/garza/Arduino/hardware/sparkfun/avr/variants/rf128" -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -D__IN_ECLIPSE__=1 -x c++ "$<"  -o  "$@"   -Wall
	@echo 'Finished building: $<'
	@echo ' '

MultiWii.cpp.o: ../MultiWii.cpp
	@echo 'Building file: $<'
	@echo 'Starting C++ compile'
	"/home/garza/arduino-1.6.8/hardware/tools/avr/bin/avr-g++" -c -g -Os -fno-exceptions -ffunction-sections -fdata-sections -fno-threadsafe-statics -MMD -mmcu=atmega128rfa1 -DF_CPU=16000000L -DARDUINO=10608 -DARDUINO_ATMEGA128RFA1_DEV_BOARD -DARDUINO_ARCH_AVR     -I"/home/garza/.arduino15/packages/arduino/hardware/avr/1.6.10/cores/arduino" -I"/home/garza/Arduino/hardware/sparkfun/avr/variants/rf128" -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -D__IN_ECLIPSE__=1 -x c++ "$<"  -o  "$@"   -Wall
	@echo 'Finished building: $<'
	@echo ' '

MultiWii.o: ../MultiWii.ino
	@echo 'Building file: $<'
	@echo 'Starting C++ compile'
	"/home/garza/arduino-1.6.8/hardware/tools/avr/bin/avr-g++" -c -g -Os -fno-exceptions -ffunction-sections -fdata-sections -fno-threadsafe-statics -MMD -mmcu=atmega128rfa1 -DF_CPU=16000000L -DARDUINO=10608 -DARDUINO_ATMEGA128RFA1_DEV_BOARD -DARDUINO_ARCH_AVR     -I"/home/garza/.arduino15/packages/arduino/hardware/avr/1.6.10/cores/arduino" -I"/home/garza/Arduino/hardware/sparkfun/avr/variants/rf128" -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -D__IN_ECLIPSE__=1 -x c++ "$<"  -o  "$@"   -Wall
	@echo 'Finished building: $<'
	@echo ' '

Output.cpp.o: ../Output.cpp
	@echo 'Building file: $<'
	@echo 'Starting C++ compile'
	"/home/garza/arduino-1.6.8/hardware/tools/avr/bin/avr-g++" -c -g -Os -fno-exceptions -ffunction-sections -fdata-sections -fno-threadsafe-statics -MMD -mmcu=atmega128rfa1 -DF_CPU=16000000L -DARDUINO=10608 -DARDUINO_ATMEGA128RFA1_DEV_BOARD -DARDUINO_ARCH_AVR     -I"/home/garza/.arduino15/packages/arduino/hardware/avr/1.6.10/cores/arduino" -I"/home/garza/Arduino/hardware/sparkfun/avr/variants/rf128" -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -D__IN_ECLIPSE__=1 -x c++ "$<"  -o  "$@"   -Wall
	@echo 'Finished building: $<'
	@echo ' '

Protocol.cpp.o: ../Protocol.cpp
	@echo 'Building file: $<'
	@echo 'Starting C++ compile'
	"/home/garza/arduino-1.6.8/hardware/tools/avr/bin/avr-g++" -c -g -Os -fno-exceptions -ffunction-sections -fdata-sections -fno-threadsafe-statics -MMD -mmcu=atmega128rfa1 -DF_CPU=16000000L -DARDUINO=10608 -DARDUINO_ATMEGA128RFA1_DEV_BOARD -DARDUINO_ARCH_AVR     -I"/home/garza/.arduino15/packages/arduino/hardware/avr/1.6.10/cores/arduino" -I"/home/garza/Arduino/hardware/sparkfun/avr/variants/rf128" -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -D__IN_ECLIPSE__=1 -x c++ "$<"  -o  "$@"   -Wall
	@echo 'Finished building: $<'
	@echo ' '

RX.cpp.o: ../RX.cpp
	@echo 'Building file: $<'
	@echo 'Starting C++ compile'
	"/home/garza/arduino-1.6.8/hardware/tools/avr/bin/avr-g++" -c -g -Os -fno-exceptions -ffunction-sections -fdata-sections -fno-threadsafe-statics -MMD -mmcu=atmega128rfa1 -DF_CPU=16000000L -DARDUINO=10608 -DARDUINO_ATMEGA128RFA1_DEV_BOARD -DARDUINO_ARCH_AVR     -I"/home/garza/.arduino15/packages/arduino/hardware/avr/1.6.10/cores/arduino" -I"/home/garza/Arduino/hardware/sparkfun/avr/variants/rf128" -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -D__IN_ECLIPSE__=1 -x c++ "$<"  -o  "$@"   -Wall
	@echo 'Finished building: $<'
	@echo ' '

Sensors.cpp.o: ../Sensors.cpp
	@echo 'Building file: $<'
	@echo 'Starting C++ compile'
	"/home/garza/arduino-1.6.8/hardware/tools/avr/bin/avr-g++" -c -g -Os -fno-exceptions -ffunction-sections -fdata-sections -fno-threadsafe-statics -MMD -mmcu=atmega128rfa1 -DF_CPU=16000000L -DARDUINO=10608 -DARDUINO_ATMEGA128RFA1_DEV_BOARD -DARDUINO_ARCH_AVR     -I"/home/garza/.arduino15/packages/arduino/hardware/avr/1.6.10/cores/arduino" -I"/home/garza/Arduino/hardware/sparkfun/avr/variants/rf128" -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -D__IN_ECLIPSE__=1 -x c++ "$<"  -o  "$@"   -Wall
	@echo 'Finished building: $<'
	@echo ' '

Serial.cpp.o: ../Serial.cpp
	@echo 'Building file: $<'
	@echo 'Starting C++ compile'
	"/home/garza/arduino-1.6.8/hardware/tools/avr/bin/avr-g++" -c -g -Os -fno-exceptions -ffunction-sections -fdata-sections -fno-threadsafe-statics -MMD -mmcu=atmega128rfa1 -DF_CPU=16000000L -DARDUINO=10608 -DARDUINO_ATMEGA128RFA1_DEV_BOARD -DARDUINO_ARCH_AVR     -I"/home/garza/.arduino15/packages/arduino/hardware/avr/1.6.10/cores/arduino" -I"/home/garza/Arduino/hardware/sparkfun/avr/variants/rf128" -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -D__IN_ECLIPSE__=1 -x c++ "$<"  -o  "$@"   -Wall
	@echo 'Finished building: $<'
	@echo ' '


