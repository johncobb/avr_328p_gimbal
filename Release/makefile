################################################################################
# Automatically-generated file. Do not edit!
################################################################################

-include ../makefile.init

RM := rm -rf

# All of the sources participating in the build are defined here
-include sources.mk
-include util/subdir.mk
-include pwm/subdir.mk
-include math/subdir.mk
-include imu/subdir.mk
-include i2c/twi/subdir.mk
-include i2c/subdir.mk
-include gimbal/subdir.mk
-include eeprom/subdir.mk
-include complimentary/subdir.mk
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
avr_328p_gimbal.lss \

FLASH_IMAGE += \
avr_328p_gimbal.hex \

EEPROM_IMAGE += \
avr_328p_gimbal.eep \

SIZEDUMMY += \
sizedummy \


# All Target
all: avr_328p_gimbal.elf secondary-outputs

# Tool invocations
avr_328p_gimbal.elf: $(OBJS) $(USER_OBJS)
	@echo 'Building target: $@'
	@echo 'Invoking: AVR C Linker'
	avr-gcc -Wl,-Map,avr_328p_gimbal.map -uvfprintf -mmcu=atmega328p -o "avr_328p_gimbal.elf" $(OBJS) $(USER_OBJS) $(LIBS)
	@echo 'Finished building target: $@'
	@echo ' '

avr_328p_gimbal.lss: avr_328p_gimbal.elf
	@echo 'Invoking: AVR Create Extended Listing'
	-avr-objdump -h -S avr_328p_gimbal.elf  >"avr_328p_gimbal.lss"
	@echo 'Finished building: $@'
	@echo ' '

avr_328p_gimbal.hex: avr_328p_gimbal.elf
	@echo 'Create Flash image (ihex format)'
	-avr-objcopy -R .eeprom -R .fuse -R .lock -R .signature -O ihex avr_328p_gimbal.elf  "avr_328p_gimbal.hex"
	@echo 'Finished building: $@'
	@echo ' '

avr_328p_gimbal.eep: avr_328p_gimbal.elf
	@echo 'Create eeprom image (ihex format)'
	-avr-objcopy -j .eeprom --no-change-warnings --change-section-lma .eeprom=0 -O ihex avr_328p_gimbal.elf  "avr_328p_gimbal.eep"
	@echo 'Finished building: $@'
	@echo ' '

sizedummy: avr_328p_gimbal.elf
	@echo 'Invoking: Print Size'
	-avr-size --format=avr --mcu=atmega328p avr_328p_gimbal.elf
	@echo 'Finished building: $@'
	@echo ' '

# Other Targets
clean:
	-$(RM) $(OBJS)$(C_DEPS)$(ASM_DEPS)$(EEPROM_IMAGE)$(FLASH_IMAGE)$(ELFS)$(LSS)$(S_DEPS)$(SIZEDUMMY)$(S_UPPER_DEPS) avr_328p_gimbal.elf
	-@echo ' '

secondary-outputs: $(LSS) $(FLASH_IMAGE) $(EEPROM_IMAGE) $(SIZEDUMMY)

.PHONY: all clean dependents
.SECONDARY:

-include ../makefile.targets
