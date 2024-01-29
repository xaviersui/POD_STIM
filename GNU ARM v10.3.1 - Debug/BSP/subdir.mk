################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../BSP/bio_management.c \
../BSP/bsp.c \
../BSP/srl_comm_management.c \
../BSP/stim_management.c 

OBJS += \
./BSP/bio_management.o \
./BSP/bsp.o \
./BSP/srl_comm_management.o \
./BSP/stim_management.o 

C_DEPS += \
./BSP/bio_management.d \
./BSP/bsp.d \
./BSP/srl_comm_management.d \
./BSP/stim_management.d 


# Each subdirectory must supply rules for building sources it contributes
BSP/bio_management.o: ../BSP/bio_management.c BSP/subdir.mk
	@echo 'Building file: $<'
	@echo 'Invoking: GNU ARM C Compiler'
	arm-none-eabi-gcc -g -gdwarf-2 -mcpu=cortex-m33 -mthumb -std=c99 '-DDEBUG=1' '-DDEBUG_EFM=1' '-DEFM32PG23B310F512IM48=1' '-DSL_COMPONENT_CATALOG_PRESENT=1' -I"C:\Users\ebt\Documents\Xavier\Firmware\Simplicity\POD_STIM\App" -I"C:\Users\ebt\Documents\Xavier\Firmware\Simplicity\POD_STIM\BSP" -I"C:\Users\ebt\Documents\Xavier\Firmware\Simplicity\POD_STIM\PLM" -I"C:\Users\ebt\Documents\Xavier\Firmware\Simplicity\POD_STIM\config" -I"C:\Users\ebt\Documents\Xavier\Firmware\Simplicity\POD_STIM\autogen" -I"C:\Users\ebt\Documents\Xavier\Firmware\Simplicity\POD_STIM" -I"C:\Users\ebt\Documents\Xavier\Firmware\Simplicity\POD_STIM\gecko_sdk_4.4.0\platform\Device\SiliconLabs\EFM32PG23\Include" -I"C:\Users\ebt\Documents\Xavier\Firmware\Simplicity\POD_STIM\gecko_sdk_4.4.0\platform\common\inc" -I"C:\Users\ebt\Documents\Xavier\Firmware\Simplicity\POD_STIM\gecko_sdk_4.4.0\platform\CMSIS\Core\Include" -I"C:\Users\ebt\Documents\Xavier\Firmware\Simplicity\POD_STIM\gecko_sdk_4.4.0\platform\service\device_init\inc" -I"C:\Users\ebt\Documents\Xavier\Firmware\Simplicity\POD_STIM\gecko_sdk_4.4.0\platform\emdrv\dmadrv\inc" -I"C:\Users\ebt\Documents\Xavier\Firmware\Simplicity\POD_STIM\gecko_sdk_4.4.0\platform\emdrv\common\inc" -I"C:\Users\ebt\Documents\Xavier\Firmware\Simplicity\POD_STIM\gecko_sdk_4.4.0\platform\emlib\inc" -I"C:\Users\ebt\Documents\Xavier\Firmware\Simplicity\POD_STIM\gecko_sdk_4.4.0\platform\emdrv\gpiointerrupt\inc" -I"C:\Users\ebt\Documents\Xavier\Firmware\Simplicity\POD_STIM\gecko_sdk_4.4.0\platform\peripheral\inc" -I"C:\Users\ebt\Documents\Xavier\Firmware\Simplicity\POD_STIM\gecko_sdk_4.4.0\platform\common\toolchain\inc" -I"C:\Users\ebt\Documents\Xavier\Firmware\Simplicity\POD_STIM\gecko_sdk_4.4.0\platform\service\system\inc" -I"C:\Users\ebt\Documents\Xavier\Firmware\Simplicity\POD_STIM\gecko_sdk_4.4.0\platform\service\sleeptimer\inc" -I"C:\Users\ebt\Documents\Xavier\Firmware\Simplicity\POD_STIM\gecko_sdk_4.4.0\platform\emdrv\spidrv\inc" -I"C:\Users\ebt\Documents\Xavier\Firmware\Simplicity\POD_STIM\gecko_sdk_4.4.0\platform\emdrv\uartdrv\inc" -Os -Wall -Wextra -mno-sched-prolog -fno-builtin -ffunction-sections -fdata-sections -imacrossl_gcc_preinclude.h -mfpu=fpv5-sp-d16 -mfloat-abi=hard -mcmse --specs=nano.specs -c -fmessage-length=0 -MMD -MP -MF"BSP/bio_management.d" -MT"$@" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '

BSP/bsp.o: ../BSP/bsp.c BSP/subdir.mk
	@echo 'Building file: $<'
	@echo 'Invoking: GNU ARM C Compiler'
	arm-none-eabi-gcc -g -gdwarf-2 -mcpu=cortex-m33 -mthumb -std=c99 '-DDEBUG=1' '-DDEBUG_EFM=1' '-DEFM32PG23B310F512IM48=1' '-DSL_COMPONENT_CATALOG_PRESENT=1' -I"C:\Users\ebt\Documents\Xavier\Firmware\Simplicity\POD_STIM\App" -I"C:\Users\ebt\Documents\Xavier\Firmware\Simplicity\POD_STIM\BSP" -I"C:\Users\ebt\Documents\Xavier\Firmware\Simplicity\POD_STIM\PLM" -I"C:\Users\ebt\Documents\Xavier\Firmware\Simplicity\POD_STIM\config" -I"C:\Users\ebt\Documents\Xavier\Firmware\Simplicity\POD_STIM\autogen" -I"C:\Users\ebt\Documents\Xavier\Firmware\Simplicity\POD_STIM" -I"C:\Users\ebt\Documents\Xavier\Firmware\Simplicity\POD_STIM\gecko_sdk_4.4.0\platform\Device\SiliconLabs\EFM32PG23\Include" -I"C:\Users\ebt\Documents\Xavier\Firmware\Simplicity\POD_STIM\gecko_sdk_4.4.0\platform\common\inc" -I"C:\Users\ebt\Documents\Xavier\Firmware\Simplicity\POD_STIM\gecko_sdk_4.4.0\platform\CMSIS\Core\Include" -I"C:\Users\ebt\Documents\Xavier\Firmware\Simplicity\POD_STIM\gecko_sdk_4.4.0\platform\service\device_init\inc" -I"C:\Users\ebt\Documents\Xavier\Firmware\Simplicity\POD_STIM\gecko_sdk_4.4.0\platform\emdrv\dmadrv\inc" -I"C:\Users\ebt\Documents\Xavier\Firmware\Simplicity\POD_STIM\gecko_sdk_4.4.0\platform\emdrv\common\inc" -I"C:\Users\ebt\Documents\Xavier\Firmware\Simplicity\POD_STIM\gecko_sdk_4.4.0\platform\emlib\inc" -I"C:\Users\ebt\Documents\Xavier\Firmware\Simplicity\POD_STIM\gecko_sdk_4.4.0\platform\emdrv\gpiointerrupt\inc" -I"C:\Users\ebt\Documents\Xavier\Firmware\Simplicity\POD_STIM\gecko_sdk_4.4.0\platform\peripheral\inc" -I"C:\Users\ebt\Documents\Xavier\Firmware\Simplicity\POD_STIM\gecko_sdk_4.4.0\platform\common\toolchain\inc" -I"C:\Users\ebt\Documents\Xavier\Firmware\Simplicity\POD_STIM\gecko_sdk_4.4.0\platform\service\system\inc" -I"C:\Users\ebt\Documents\Xavier\Firmware\Simplicity\POD_STIM\gecko_sdk_4.4.0\platform\service\sleeptimer\inc" -I"C:\Users\ebt\Documents\Xavier\Firmware\Simplicity\POD_STIM\gecko_sdk_4.4.0\platform\emdrv\spidrv\inc" -I"C:\Users\ebt\Documents\Xavier\Firmware\Simplicity\POD_STIM\gecko_sdk_4.4.0\platform\emdrv\uartdrv\inc" -Os -Wall -Wextra -mno-sched-prolog -fno-builtin -ffunction-sections -fdata-sections -imacrossl_gcc_preinclude.h -mfpu=fpv5-sp-d16 -mfloat-abi=hard -mcmse --specs=nano.specs -c -fmessage-length=0 -MMD -MP -MF"BSP/bsp.d" -MT"$@" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '

BSP/srl_comm_management.o: ../BSP/srl_comm_management.c BSP/subdir.mk
	@echo 'Building file: $<'
	@echo 'Invoking: GNU ARM C Compiler'
	arm-none-eabi-gcc -g -gdwarf-2 -mcpu=cortex-m33 -mthumb -std=c99 '-DDEBUG=1' '-DDEBUG_EFM=1' '-DEFM32PG23B310F512IM48=1' '-DSL_COMPONENT_CATALOG_PRESENT=1' -I"C:\Users\ebt\Documents\Xavier\Firmware\Simplicity\POD_STIM\App" -I"C:\Users\ebt\Documents\Xavier\Firmware\Simplicity\POD_STIM\BSP" -I"C:\Users\ebt\Documents\Xavier\Firmware\Simplicity\POD_STIM\PLM" -I"C:\Users\ebt\Documents\Xavier\Firmware\Simplicity\POD_STIM\config" -I"C:\Users\ebt\Documents\Xavier\Firmware\Simplicity\POD_STIM\autogen" -I"C:\Users\ebt\Documents\Xavier\Firmware\Simplicity\POD_STIM" -I"C:\Users\ebt\Documents\Xavier\Firmware\Simplicity\POD_STIM\gecko_sdk_4.4.0\platform\Device\SiliconLabs\EFM32PG23\Include" -I"C:\Users\ebt\Documents\Xavier\Firmware\Simplicity\POD_STIM\gecko_sdk_4.4.0\platform\common\inc" -I"C:\Users\ebt\Documents\Xavier\Firmware\Simplicity\POD_STIM\gecko_sdk_4.4.0\platform\CMSIS\Core\Include" -I"C:\Users\ebt\Documents\Xavier\Firmware\Simplicity\POD_STIM\gecko_sdk_4.4.0\platform\service\device_init\inc" -I"C:\Users\ebt\Documents\Xavier\Firmware\Simplicity\POD_STIM\gecko_sdk_4.4.0\platform\emdrv\dmadrv\inc" -I"C:\Users\ebt\Documents\Xavier\Firmware\Simplicity\POD_STIM\gecko_sdk_4.4.0\platform\emdrv\common\inc" -I"C:\Users\ebt\Documents\Xavier\Firmware\Simplicity\POD_STIM\gecko_sdk_4.4.0\platform\emlib\inc" -I"C:\Users\ebt\Documents\Xavier\Firmware\Simplicity\POD_STIM\gecko_sdk_4.4.0\platform\emdrv\gpiointerrupt\inc" -I"C:\Users\ebt\Documents\Xavier\Firmware\Simplicity\POD_STIM\gecko_sdk_4.4.0\platform\peripheral\inc" -I"C:\Users\ebt\Documents\Xavier\Firmware\Simplicity\POD_STIM\gecko_sdk_4.4.0\platform\common\toolchain\inc" -I"C:\Users\ebt\Documents\Xavier\Firmware\Simplicity\POD_STIM\gecko_sdk_4.4.0\platform\service\system\inc" -I"C:\Users\ebt\Documents\Xavier\Firmware\Simplicity\POD_STIM\gecko_sdk_4.4.0\platform\service\sleeptimer\inc" -I"C:\Users\ebt\Documents\Xavier\Firmware\Simplicity\POD_STIM\gecko_sdk_4.4.0\platform\emdrv\spidrv\inc" -I"C:\Users\ebt\Documents\Xavier\Firmware\Simplicity\POD_STIM\gecko_sdk_4.4.0\platform\emdrv\uartdrv\inc" -Os -Wall -Wextra -mno-sched-prolog -fno-builtin -ffunction-sections -fdata-sections -imacrossl_gcc_preinclude.h -mfpu=fpv5-sp-d16 -mfloat-abi=hard -mcmse --specs=nano.specs -c -fmessage-length=0 -MMD -MP -MF"BSP/srl_comm_management.d" -MT"$@" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '

BSP/stim_management.o: ../BSP/stim_management.c BSP/subdir.mk
	@echo 'Building file: $<'
	@echo 'Invoking: GNU ARM C Compiler'
	arm-none-eabi-gcc -g -gdwarf-2 -mcpu=cortex-m33 -mthumb -std=c99 '-DDEBUG=1' '-DDEBUG_EFM=1' '-DEFM32PG23B310F512IM48=1' '-DSL_COMPONENT_CATALOG_PRESENT=1' -I"C:\Users\ebt\Documents\Xavier\Firmware\Simplicity\POD_STIM\App" -I"C:\Users\ebt\Documents\Xavier\Firmware\Simplicity\POD_STIM\BSP" -I"C:\Users\ebt\Documents\Xavier\Firmware\Simplicity\POD_STIM\PLM" -I"C:\Users\ebt\Documents\Xavier\Firmware\Simplicity\POD_STIM\config" -I"C:\Users\ebt\Documents\Xavier\Firmware\Simplicity\POD_STIM\autogen" -I"C:\Users\ebt\Documents\Xavier\Firmware\Simplicity\POD_STIM" -I"C:\Users\ebt\Documents\Xavier\Firmware\Simplicity\POD_STIM\gecko_sdk_4.4.0\platform\Device\SiliconLabs\EFM32PG23\Include" -I"C:\Users\ebt\Documents\Xavier\Firmware\Simplicity\POD_STIM\gecko_sdk_4.4.0\platform\common\inc" -I"C:\Users\ebt\Documents\Xavier\Firmware\Simplicity\POD_STIM\gecko_sdk_4.4.0\platform\CMSIS\Core\Include" -I"C:\Users\ebt\Documents\Xavier\Firmware\Simplicity\POD_STIM\gecko_sdk_4.4.0\platform\service\device_init\inc" -I"C:\Users\ebt\Documents\Xavier\Firmware\Simplicity\POD_STIM\gecko_sdk_4.4.0\platform\emdrv\dmadrv\inc" -I"C:\Users\ebt\Documents\Xavier\Firmware\Simplicity\POD_STIM\gecko_sdk_4.4.0\platform\emdrv\common\inc" -I"C:\Users\ebt\Documents\Xavier\Firmware\Simplicity\POD_STIM\gecko_sdk_4.4.0\platform\emlib\inc" -I"C:\Users\ebt\Documents\Xavier\Firmware\Simplicity\POD_STIM\gecko_sdk_4.4.0\platform\emdrv\gpiointerrupt\inc" -I"C:\Users\ebt\Documents\Xavier\Firmware\Simplicity\POD_STIM\gecko_sdk_4.4.0\platform\peripheral\inc" -I"C:\Users\ebt\Documents\Xavier\Firmware\Simplicity\POD_STIM\gecko_sdk_4.4.0\platform\common\toolchain\inc" -I"C:\Users\ebt\Documents\Xavier\Firmware\Simplicity\POD_STIM\gecko_sdk_4.4.0\platform\service\system\inc" -I"C:\Users\ebt\Documents\Xavier\Firmware\Simplicity\POD_STIM\gecko_sdk_4.4.0\platform\service\sleeptimer\inc" -I"C:\Users\ebt\Documents\Xavier\Firmware\Simplicity\POD_STIM\gecko_sdk_4.4.0\platform\emdrv\spidrv\inc" -I"C:\Users\ebt\Documents\Xavier\Firmware\Simplicity\POD_STIM\gecko_sdk_4.4.0\platform\emdrv\uartdrv\inc" -Os -Wall -Wextra -mno-sched-prolog -fno-builtin -ffunction-sections -fdata-sections -imacrossl_gcc_preinclude.h -mfpu=fpv5-sp-d16 -mfloat-abi=hard -mcmse --specs=nano.specs -c -fmessage-length=0 -MMD -MP -MF"BSP/stim_management.d" -MT"$@" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


