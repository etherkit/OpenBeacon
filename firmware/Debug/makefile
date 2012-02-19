################################################################################
# Automatically-generated file. Do not edit!
################################################################################

-include ../makefile.init

RM := rm -rf

# All of the sources participating in the build are defined here
-include sources.mk
-include usbdrv/subdir.mk
-include subdir.mk
-include objects.mk

ifneq ($(MAKECMDGOALS),clean)
ifneq ($(strip $(C_DEPS)),)
-include $(C_DEPS)
endif
ifneq ($(strip $(ASM_DEPS)),)
-include $(ASM_DEPS)
endif
ifneq ($(strip $(S_DEPS)),)
-include $(S_DEPS)
endif
ifneq ($(strip $(S_UPPER_DEPS)),)
-include $(S_UPPER_DEPS)
endif
endif

-include ../makefile.defs

# Add inputs and outputs from these tool invocations to the build variables 
LSS += \
OpenBeacon.lss \

FLASH_IMAGE += \
OpenBeacon.hex \

SIZEDUMMY += \
sizedummy \


# All Target
all: OpenBeacon.elf secondary-outputs

# Tool invocations
OpenBeacon.elf: $(OBJS) $(USER_OBJS)
	@echo 'Building target: $@'
	@echo 'Invoking: AVR C Linker'
	avr-gcc -Wl,-Map,OpenBeacon.map -mmcu=attiny85 -o "OpenBeacon.elf" $(OBJS) $(USER_OBJS) $(LIBS)
	@echo 'Finished building target: $@'
	@echo ' '

OpenBeacon.lss: OpenBeacon.elf
	@echo 'Invoking: AVR Create Extended Listing'
	-avr-objdump -h -S OpenBeacon.elf  >"OpenBeacon.lss"
	@echo 'Finished building: $@'
	@echo ' '

OpenBeacon.hex: OpenBeacon.elf
	@echo 'Create Flash image (ihex format)'
	-avr-objcopy -R .eeprom -O ihex OpenBeacon.elf  "OpenBeacon.hex"
	@echo 'Finished building: $@'
	@echo ' '

sizedummy: OpenBeacon.elf
	@echo 'Invoking: Print Size'
	-avr-size --format=avr --mcu=attiny85 OpenBeacon.elf
	@echo 'Finished building: $@'
	@echo ' '

# Other Targets
clean:
	-$(RM) $(OBJS)$(C_DEPS)$(ASM_DEPS)$(FLASH_IMAGE)$(ELFS)$(LSS)$(S_DEPS)$(SIZEDUMMY)$(S_UPPER_DEPS) OpenBeacon.elf
	-@echo ' '

secondary-outputs: $(LSS) $(FLASH_IMAGE) $(SIZEDUMMY)

.PHONY: all clean dependents
.SECONDARY:

-include ../makefile.targets
