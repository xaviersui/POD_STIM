################################################################################
# Automatically-generated file. Do not edit!
################################################################################

-include ../makefile.init

RM := rm -rf

# All of the sources participating in the build are defined here
-include sources.mk
-include gecko_sdk_4.4.0/platform/service/system/src/subdir.mk
-include gecko_sdk_4.4.0/platform/service/sleeptimer/src/subdir.mk
-include gecko_sdk_4.4.0/platform/service/device_init/src/subdir.mk
-include gecko_sdk_4.4.0/platform/peripheral/src/subdir.mk
-include gecko_sdk_4.4.0/platform/emlib/src/subdir.mk
-include gecko_sdk_4.4.0/platform/emdrv/uartdrv/src/subdir.mk
-include gecko_sdk_4.4.0/platform/emdrv/spidrv/src/subdir.mk
-include gecko_sdk_4.4.0/platform/emdrv/gpiointerrupt/src/subdir.mk
-include gecko_sdk_4.4.0/platform/emdrv/dmadrv/src/subdir.mk
-include gecko_sdk_4.4.0/platform/common/toolchain/src/subdir.mk
-include gecko_sdk_4.4.0/platform/common/src/subdir.mk
-include gecko_sdk_4.4.0/platform/Device/SiliconLabs/EFM32PG23/Source/subdir.mk
-include autogen/subdir.mk
-include PLM/subdir.mk
-include BSP/subdir.mk
-include App/subdir.mk
-include subdir.mk
-include objects.mk

ifneq ($(MAKECMDGOALS),clean)
ifneq ($(strip $(C_DEPS)),)
-include $(C_DEPS)
endif
endif

-include ../makefile.defs

OPTIONAL_TOOL_DEPS := \
$(wildcard ../makefile.defs) \
$(wildcard ../makefile.init) \
$(wildcard ../makefile.targets) \


BUILD_ARTIFACT_NAME := POD_STIM
BUILD_ARTIFACT_EXTENSION := axf
BUILD_ARTIFACT_PREFIX :=
BUILD_ARTIFACT := $(BUILD_ARTIFACT_PREFIX)$(BUILD_ARTIFACT_NAME)$(if $(BUILD_ARTIFACT_EXTENSION),.$(BUILD_ARTIFACT_EXTENSION),)

# Add inputs and outputs from these tool invocations to the build variables 

# All Target
all: main-build

# Main-build Target
main-build: POD_STIM.axf

# Tool invocations
POD_STIM.axf: $(OBJS) $(USER_OBJS) makefile objects.mk $(OPTIONAL_TOOL_DEPS)
	@echo 'Building target: $@'
	@echo 'Invoking: GNU ARM C Linker'
	arm-none-eabi-gcc -g -gdwarf-2 -mcpu=cortex-m33 -mthumb -T "C:\Users\ebt\Documents\Xavier\Firmware\Simplicity\POD_STIM\autogen\linkerfile.ld" -Xlinker --gc-sections -Xlinker -Map="POD_STIM.map" -mfpu=fpv5-sp-d16 -mfloat-abi=hard --specs=nano.specs -o POD_STIM.axf -Wl,--start-group "./App/app.o" "./App/biofeedback.o" "./App/error.o" "./App/fsm.o" "./App/main.o" "./App/stimulation.o" "./BSP/bio_management.o" "./BSP/bsp.o" "./BSP/srl_comm_management.o" "./BSP/stim_management.o" "./PLM/flash.o" "./PLM/i2c.o" "./PLM/iadc.o" "./PLM/spi.o" "./PLM/timer.o" "./PLM/uart.o" "./autogen/sl_device_init_clocks.o" "./autogen/sl_event_handler.o" "./autogen/sl_spidrv_init.o" "./autogen/sl_uartdrv_init.o" "./gecko_sdk_4.4.0/platform/Device/SiliconLabs/EFM32PG23/Source/startup_efm32pg23.o" "./gecko_sdk_4.4.0/platform/Device/SiliconLabs/EFM32PG23/Source/system_efm32pg23.o" "./gecko_sdk_4.4.0/platform/common/src/sl_assert.o" "./gecko_sdk_4.4.0/platform/common/src/sl_slist.o" "./gecko_sdk_4.4.0/platform/common/src/sl_syscalls.o" "./gecko_sdk_4.4.0/platform/common/toolchain/src/sl_memory.o" "./gecko_sdk_4.4.0/platform/emdrv/dmadrv/src/dmadrv.o" "./gecko_sdk_4.4.0/platform/emdrv/gpiointerrupt/src/gpiointerrupt.o" "./gecko_sdk_4.4.0/platform/emdrv/spidrv/src/spidrv.o" "./gecko_sdk_4.4.0/platform/emdrv/uartdrv/src/uartdrv.o" "./gecko_sdk_4.4.0/platform/emlib/src/em_burtc.o" "./gecko_sdk_4.4.0/platform/emlib/src/em_cmu.o" "./gecko_sdk_4.4.0/platform/emlib/src/em_core.o" "./gecko_sdk_4.4.0/platform/emlib/src/em_emu.o" "./gecko_sdk_4.4.0/platform/emlib/src/em_eusart.o" "./gecko_sdk_4.4.0/platform/emlib/src/em_gpio.o" "./gecko_sdk_4.4.0/platform/emlib/src/em_i2c.o" "./gecko_sdk_4.4.0/platform/emlib/src/em_iadc.o" "./gecko_sdk_4.4.0/platform/emlib/src/em_ldma.o" "./gecko_sdk_4.4.0/platform/emlib/src/em_msc.o" "./gecko_sdk_4.4.0/platform/emlib/src/em_prs.o" "./gecko_sdk_4.4.0/platform/emlib/src/em_system.o" "./gecko_sdk_4.4.0/platform/emlib/src/em_timer.o" "./gecko_sdk_4.4.0/platform/emlib/src/em_usart.o" "./gecko_sdk_4.4.0/platform/peripheral/src/peripheral_sysrtc.o" "./gecko_sdk_4.4.0/platform/service/device_init/src/sl_device_init_dcdc_s2.o" "./gecko_sdk_4.4.0/platform/service/device_init/src/sl_device_init_emu_s2.o" "./gecko_sdk_4.4.0/platform/service/device_init/src/sl_device_init_nvic.o" "./gecko_sdk_4.4.0/platform/service/sleeptimer/src/sl_sleeptimer.o" "./gecko_sdk_4.4.0/platform/service/sleeptimer/src/sl_sleeptimer_hal_burtc.o" "./gecko_sdk_4.4.0/platform/service/sleeptimer/src/sl_sleeptimer_hal_sysrtc.o" "./gecko_sdk_4.4.0/platform/service/sleeptimer/src/sl_sleeptimer_hal_timer.o" "./gecko_sdk_4.4.0/platform/service/system/src/sl_system_init.o" "./gecko_sdk_4.4.0/platform/service/system/src/sl_system_process_action.o" -lgcc -lc -lm -lnosys -Wl,--end-group -Wl,--start-group -lgcc -lc -lnosys -Wl,--end-group
	@echo 'Finished building target: $@'
	@echo ' '

	@echo 'Building out file: POD_STIM.out'
	arm-none-eabi-objcopy "POD_STIM.axf" "POD_STIM.out"
	@echo ' '

	@echo 'Building hex file: POD_STIM.hex'
	arm-none-eabi-objcopy -O ihex "POD_STIM.axf" "POD_STIM.hex"
	@echo ' '

	@echo 'Building bin file: POD_STIM.bin'
	arm-none-eabi-objcopy -O binary "POD_STIM.axf" "POD_STIM.bin"
	@echo ' '

	@echo 'Building s37 file: POD_STIM.s37'
	arm-none-eabi-objcopy -O srec "POD_STIM.axf" "POD_STIM.s37"
	@echo ' '

	@echo 'Running size tool'
	arm-none-eabi-size "POD_STIM.axf" -A
	@echo ' '

# Other Targets
clean:
	-@$(foreach FILE, $(EXECUTABLES), $(shell $(RM) $(FILE)) )
	-@$(foreach FILE, $(OBJS), $(shell $(RM) $(FILE)) )
	-@$(foreach FILE, $(C_DEPS), $(shell $(RM) $(FILE)) )
	-$(RM) POD_STIM.axf
	-@echo ' '

.PHONY: all clean dependents main-build

-include ../makefile.targets
