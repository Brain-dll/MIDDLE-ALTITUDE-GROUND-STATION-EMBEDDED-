################################################################################
# Automatically-generated file. Do not edit!
################################################################################

-include ../makefile.init

RM := rm -rf

# All of the sources participating in the build are defined here
-include sources.mk
-include Drivers/STM32F3xx_HAL_Driver/Src/subdir.mk
-include Core/Startup/subdir.mk
-include Core/Src/subdir.mk
-include subdir.mk
-include objects.mk

ifneq ($(MAKECMDGOALS),clean)
ifneq ($(strip $(S_DEPS)),)
-include $(S_DEPS)
endif
ifneq ($(strip $(S_UPPER_DEPS)),)
-include $(S_UPPER_DEPS)
endif
ifneq ($(strip $(C_DEPS)),)
-include $(C_DEPS)
endif
endif

-include ../makefile.defs

# Add inputs and outputs from these tool invocations to the build variables 
EXECUTABLES += \
F334_MAIN_GROUNDSTATION.elf \

SIZE_OUTPUT += \
default.size.stdout \

OBJDUMP_LIST += \
F334_MAIN_GROUNDSTATION.list \

OBJCOPY_BIN += \
F334_MAIN_GROUNDSTATION.bin \


# All Target
all: F334_MAIN_GROUNDSTATION.elf secondary-outputs

# Tool invocations
F334_MAIN_GROUNDSTATION.elf: $(OBJS) $(USER_OBJS) C:\Users\Beytu\Documents\ST_PROJECTS\F334_MAIN_GROUNDSTATION\STM32F334C8TX_FLASH.ld
	arm-none-eabi-gcc -o "F334_MAIN_GROUNDSTATION.elf" @"objects.list" $(USER_OBJS) $(LIBS) -mcpu=cortex-m4 -T"C:\Users\Beytu\Documents\ST_PROJECTS\F334_MAIN_GROUNDSTATION\STM32F334C8TX_FLASH.ld" --specs=nosys.specs -Wl,-Map="F334_MAIN_GROUNDSTATION.map" -Wl,--gc-sections -static -u _printf_float --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -Wl,--start-group -lc -lm -Wl,--end-group
	@echo 'Finished building target: $@'
	@echo ' '

default.size.stdout: $(EXECUTABLES)
	arm-none-eabi-size  $(EXECUTABLES)
	@echo 'Finished building: $@'
	@echo ' '

F334_MAIN_GROUNDSTATION.list: $(EXECUTABLES)
	arm-none-eabi-objdump -h -S $(EXECUTABLES) > "F334_MAIN_GROUNDSTATION.list"
	@echo 'Finished building: $@'
	@echo ' '

F334_MAIN_GROUNDSTATION.bin: $(EXECUTABLES)
	arm-none-eabi-objcopy  -O binary $(EXECUTABLES) "F334_MAIN_GROUNDSTATION.bin"
	@echo 'Finished building: $@'
	@echo ' '

# Other Targets
clean:
	-$(RM) *
	-@echo ' '

secondary-outputs: $(SIZE_OUTPUT) $(OBJDUMP_LIST) $(OBJCOPY_BIN)

.PHONY: all clean dependents
.SECONDARY:

-include ../makefile.targets
