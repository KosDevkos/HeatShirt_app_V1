################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../app/bluetooth/common/util/infrastructure.c 

OBJS += \
./app/bluetooth/common/util/infrastructure.o 

C_DEPS += \
./app/bluetooth/common/util/infrastructure.d 


# Each subdirectory must supply rules for building sources it contributes
app/bluetooth/common/util/infrastructure.o: ../app/bluetooth/common/util/infrastructure.c
	@echo 'Building file: $<'
	@echo 'Invoking: GNU ARM C Compiler'
	arm-none-eabi-gcc -g -gdwarf-2 -mcpu=cortex-m4 -mthumb -std=c99 '-DHAL_CONFIG=1' '-D__HEAP_SIZE=0xD00' '-D__STACK_SIZE=0x800' '-D__StackLimit=0x20000000' '-DBGM13S32F512GA=1' -I"/Users/ilyakosvin/SimplicityStudio/v4_workspace/MLX90632_V1/platform/emlib/src" -I"/Users/ilyakosvin/SimplicityStudio/v4_workspace/MLX90632_V1/hardware/module/config" -I"/Users/ilyakosvin/SimplicityStudio/v4_workspace/MLX90632_V1/platform/radio/rail_lib/common" -I"/Users/ilyakosvin/SimplicityStudio/v4_workspace/MLX90632_V1/platform/CMSIS/Include" -I"/Users/ilyakosvin/SimplicityStudio/v4_workspace/MLX90632_V1/platform/emlib/inc" -I"/Users/ilyakosvin/SimplicityStudio/v4_workspace/MLX90632_V1/protocol/bluetooth/ble_stack/inc/common" -I"/Users/ilyakosvin/SimplicityStudio/v4_workspace/MLX90632_V1/platform/Device/SiliconLabs/BGM13/Include" -I"/Users/ilyakosvin/SimplicityStudio/v4_workspace/MLX90632_V1/hardware/kit/common/halconfig" -I"/Users/ilyakosvin/SimplicityStudio/v4_workspace/MLX90632_V1/platform/emdrv/uartdrv/inc" -I"/Users/ilyakosvin/SimplicityStudio/v4_workspace/MLX90632_V1/hardware/kit/common/drivers" -I"/Users/ilyakosvin/SimplicityStudio/v4_workspace/MLX90632_V1/app/bluetooth/common/util" -I"/Users/ilyakosvin/SimplicityStudio/v4_workspace/MLX90632_V1/hardware/kit/BGM13_BRD4305A/config" -I"/Users/ilyakosvin/SimplicityStudio/v4_workspace/MLX90632_V1/protocol/bluetooth/ble_stack/inc/soc" -I"/Users/ilyakosvin/SimplicityStudio/v4_workspace/MLX90632_V1/hardware/kit/common/bsp" -I"/Users/ilyakosvin/SimplicityStudio/v4_workspace/MLX90632_V1/platform/Device/SiliconLabs/BGM13/Source/GCC" -I"/Users/ilyakosvin/SimplicityStudio/v4_workspace/MLX90632_V1/platform/emdrv/sleep/src" -I"/Users/ilyakosvin/SimplicityStudio/v4_workspace/MLX90632_V1/platform/emdrv/gpiointerrupt/inc" -I"/Users/ilyakosvin/SimplicityStudio/v4_workspace/MLX90632_V1/platform/emdrv/sleep/inc" -I"/Users/ilyakosvin/SimplicityStudio/v4_workspace/MLX90632_V1/platform/radio/rail_lib/protocol/ble" -I"/Users/ilyakosvin/SimplicityStudio/v4_workspace/MLX90632_V1/platform/radio/rail_lib/protocol/ieee802154" -I"/Users/ilyakosvin/SimplicityStudio/v4_workspace/MLX90632_V1/platform/bootloader/api" -I"/Users/ilyakosvin/SimplicityStudio/v4_workspace/MLX90632_V1/platform/radio/rail_lib/chip/efr32/efr32xg1x" -I"/Users/ilyakosvin/SimplicityStudio/v4_workspace/MLX90632_V1/platform/emdrv/common/inc" -I"/Users/ilyakosvin/SimplicityStudio/v4_workspace/MLX90632_V1/platform/halconfig/inc/hal-config" -I"/Users/ilyakosvin/SimplicityStudio/v4_workspace/MLX90632_V1/platform/Device/SiliconLabs/BGM13/Source" -I"/Users/ilyakosvin/SimplicityStudio/v4_workspace/MLX90632_V1" -I"/Users/ilyakosvin/SimplicityStudio/v4_workspace/MLX90632_V1/platform/bootloader" -O2 -Wall -c -fmessage-length=0 -ffunction-sections -fdata-sections -mfpu=fpv4-sp-d16 -mfloat-abi=softfp -MMD -MP -MF"app/bluetooth/common/util/infrastructure.d" -MT"app/bluetooth/common/util/infrastructure.o" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


