################################################################################
# Automatically-generated file. Do not edit!
################################################################################

-include ../makefile.init

RM := rm -f

# All of the sources participating in the build are defined here
-include sources.mk
-include subdir.mk
-include arduino/core/subdir.mk
-include objects.mk

ifneq ($(MAKECMDGOALS),clean)
ifneq ($(strip $(C++_DEPS)),)
-include $(C++_DEPS)
endif
ifneq ($(strip $(ASM_DEPS)),)
-include $(ASM_DEPS)
endif
ifneq ($(strip $(C_DEPS)),)
-include $(C_DEPS)
endif
ifneq ($(strip $(CC_DEPS)),)
-include $(CC_DEPS)
endif
ifneq ($(strip $(INO_DEPS)),)
-include $(INO_DEPS)
endif
ifneq ($(strip $(CPP_DEPS)),)
-include $(CPP_DEPS)
endif
ifneq ($(strip $(CXX_DEPS)),)
-include $(CXX_DEPS)
endif
ifneq ($(strip $(C_UPPER_DEPS)),)
-include $(C_UPPER_DEPS)
endif
ifneq ($(strip $(S_UPPER_DEPS)),)
-include $(S_UPPER_DEPS)
endif
endif

-include ../makefile.defs

# Add inputs and outputs from these tool invocations to the build variables 
AR += \
arduino.ar \

ELF += \
MultiWii.elf \

EEP += \
MultiWii.eep \

FLASH_IMAGE += \
MultiWii.hex \


# All Target
all: MultiWii

# Tool invocations
MultiWii: $(FLASH_IMAGE)
	@echo 'Building target: $@'
	@echo 'Printing size:'
	"/home/garza/arduino-1.6.8/hardware/tools/avr/bin/avr-size" -A "/home/garza/eclipseArduino_workspace/MultiWii/Release/MultiWii.elf"
	@echo 'Finished building target: $@'
	@echo ' '

arduino.ar: $(AR_OBJ)
	@echo 'Starting archiver'
	"/home/garza/arduino-1.6.8/hardware/tools/avr/bin/avr-ar" rcs  "/home/garza/eclipseArduino_workspace/MultiWii/Release/arduino.ar"   $(AR_OBJ)
	@echo 'Finished building: $@'
	@echo ' '

MultiWii.elf: $(LINK_OBJ) $(AR)
	@echo 'Starting combiner'
	"/home/garza/arduino-1.6.8/hardware/tools/avr/bin/avr-gcc" -Os -Wl,--gc-sections -mmcu=atmega128rfa1  -o "/home/garza/eclipseArduino_workspace/MultiWii/Release/MultiWii.elf"   $(LINK_OBJ) $(AR)  "/home/garza/eclipseArduino_workspace/MultiWii/Release/arduino.ar" "-L/home/garza/eclipseArduino_workspace/MultiWii/Release" -lm
	@echo 'Finished building: $@'
	@echo ' '

MultiWii.eep: $(ELF)
	@echo 'Create eeprom image'
	"/home/garza/arduino-1.6.8/hardware/tools/avr/bin/avr-objcopy" -O ihex -j .eeprom --set-section-flags=.eeprom=alloc,load --no-change-warnings --change-section-lma .eeprom=0  "/home/garza/eclipseArduino_workspace/MultiWii/Release/MultiWii.elf" "/home/garza/eclipseArduino_workspace/MultiWii/Release/MultiWii.eep"
	@echo 'Finished building: $@'
	@echo ' '

MultiWii.hex: $(EEP)
	@echo 'Create Flash image (ihex format)'
	"/home/garza/arduino-1.6.8/hardware/tools/avr/bin/avr-objcopy" -O ihex -R .eeprom  "/home/garza/eclipseArduino_workspace/MultiWii/Release/MultiWii.elf" "/home/garza/eclipseArduino_workspace/MultiWii/Release/MultiWii.hex"
	@echo 'Finished building: $@'
	@echo ' '

# Other Targets
clean:
	-$(RM) $(C_DEPS)$(AR)$(FLASH_IMAGE)$(EEP)$(CXX_DEPS)$(S_UPPER_DEPS)$(ELF)$(C++_DEPS)$(ASM_DEPS)$(CC_DEPS)$(INO_DEPS)$(AR_OBJ)$(CPP_DEPS)$(LINK_OBJ)$(C_UPPER_DEPS)$(SIZEDUMMY) MultiWii
	-@echo ' '

.PHONY: all clean dependents
.SECONDARY:

-include ../makefile.targets
