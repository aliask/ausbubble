################################################################################
# Automatically-generated file. Do not edit!
################################################################################

-include ../makefile.init

RM := rm -rf

# All of the sources participating in the build are defined here
-include sources.mk
-include src/usb/subdir.mk
-include src/system/subdir.mk
-include src/drivers/subdir.mk
-include src/subdir.mk
-include lib/STM32_USB_OTG_Driver/src/subdir.mk
-include lib/STM32_USB_Device_Library/Core/src/subdir.mk
-include lib/STM32_USB_Device_Library/Class/cdc/src/subdir.mk
-include lib/STM32F4xx_StdPeriph_Driver/src/subdir.mk
-include lib/FreeRTOS/Source/portable/MemMang/subdir.mk
-include lib/FreeRTOS/Source/portable/GCC/ARM_CM4F/subdir.mk
-include lib/FreeRTOS/Source/subdir.mk
-include subdir.mk
-include objects.mk

ifneq ($(MAKECMDGOALS),clean)
ifneq ($(strip $(C++_DEPS)),)
-include $(C++_DEPS)
endif
ifneq ($(strip $(C_DEPS)),)
-include $(C_DEPS)
endif
ifneq ($(strip $(ASM_DEPS)),)
-include $(ASM_DEPS)
endif
ifneq ($(strip $(CC_DEPS)),)
-include $(CC_DEPS)
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
SECONDARY_FLASH += \
AusBubble.hex \

SECONDARY_LIST += \
AusBubble.lst \

SECONDARY_SIZE += \
AusBubble.siz \


# All Target
all: AusBubble.elf secondary-outputs

# Tool invocations
AusBubble.elf: $(OBJS) $(USER_OBJS)
	@echo 'Building target: $@'
	@echo 'Invoking: ARM Yagarto Windows GCC C++ Linker'
	arm-none-eabi-g++ -T"C:/Users/Nick/workspace/AusBubble/stm32_flash.ld" -nostartfiles -Xlinker --gc-sections -Wl,-Map,AusBubble.map -v -mcpu=cortex-m4 -mthumb -mfloat-abi=softfp -mfpu=fpv4-sp-d16 -g3 -gdwarf-2 -o "AusBubble.elf" $(OBJS) $(USER_OBJS) $(LIBS)
	@echo 'Finished building target: $@'
	@echo ' '

AusBubble.hex: AusBubble.elf
	@echo 'Invoking: ARM Yagarto Windows GNU Create Flash Image'
	arm-none-eabi-objcopy -O ihex AusBubble.elf  "AusBubble.hex"
	@echo 'Finished building: $@'
	@echo ' '

AusBubble.lst: AusBubble.elf
	@echo 'Invoking: ARM Yagarto Windows GNU Create Listing'
	arm-none-eabi-objdump -h -S AusBubble.elf > "AusBubble.lst"
	@echo 'Finished building: $@'
	@echo ' '

AusBubble.siz: AusBubble.elf
	@echo 'Invoking: ARM Yagarto Windows GNU Print Size'
	arm-none-eabi-size  --format=berkeley AusBubble.elf
	@echo 'Finished building: $@'
	@echo ' '

# Other Targets
clean:
	-$(RM) $(OBJS)$(C_DEPS)$(SECONDARY_FLASH)$(CXX_DEPS)$(S_UPPER_DEPS)$(SECONDARY_LIST)$(C++_DEPS)$(SECONDARY_SIZE)$(ASM_DEPS)$(CC_DEPS)$(CPP_DEPS)$(EXECUTABLES)$(C_UPPER_DEPS) AusBubble.elf
	-@echo ' '

secondary-outputs: $(SECONDARY_FLASH) $(SECONDARY_LIST) $(SECONDARY_SIZE)

.PHONY: all clean dependents
.SECONDARY:

-include ../makefile.targets
