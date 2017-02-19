################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Utilities/Components/ov2640/ov2640.c 

OBJS += \
./Utilities/Components/ov2640/ov2640.o 

C_DEPS += \
./Utilities/Components/ov2640/ov2640.d 


# Each subdirectory must supply rules for building sources it contributes
Utilities/Components/ov2640/%.o: ../Utilities/Components/ov2640/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: MCU GCC Compiler'
	@echo %cd%
	arm-none-eabi-gcc -mcpu=cortex-m4 -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16 -DSTM32F407VGTx -DSTM32F4 -DSTM32F4DISCOVERY -DSTM32 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F407xx -I"C:/Users/WANG/workspace/CanRotor_IMU/inc" -I"C:/Users/WANG/workspace/CanRotor_IMU/CMSIS/core" -I"C:/Users/WANG/workspace/CanRotor_IMU/CMSIS/device" -I"C:/Users/WANG/workspace/CanRotor_IMU/HAL_Driver/Inc/Legacy" -I"C:/Users/WANG/workspace/CanRotor_IMU/HAL_Driver/Inc" -I"C:/Users/WANG/workspace/CanRotor_IMU/Utilities/Components/ampire480272" -I"C:/Users/WANG/workspace/CanRotor_IMU/Utilities/Components/ampire640480" -I"C:/Users/WANG/workspace/CanRotor_IMU/Utilities/Components/Common" -I"C:/Users/WANG/workspace/CanRotor_IMU/Utilities/Components/cs43l22" -I"C:/Users/WANG/workspace/CanRotor_IMU/Utilities/Components/exc7200" -I"C:/Users/WANG/workspace/CanRotor_IMU/Utilities/Components/ft6x06" -I"C:/Users/WANG/workspace/CanRotor_IMU/Utilities/Components/ili9325" -I"C:/Users/WANG/workspace/CanRotor_IMU/Utilities/Components/ili9341" -I"C:/Users/WANG/workspace/CanRotor_IMU/Utilities/Components/l3gd20" -I"C:/Users/WANG/workspace/CanRotor_IMU/Utilities/Components/lis302dl" -I"C:/Users/WANG/workspace/CanRotor_IMU/Utilities/Components/lis3dsh" -I"C:/Users/WANG/workspace/CanRotor_IMU/Utilities/Components/ls016b8uy" -I"C:/Users/WANG/workspace/CanRotor_IMU/Utilities/Components/lsm303dlhc" -I"C:/Users/WANG/workspace/CanRotor_IMU/Utilities/Components/mfxstm32l152" -I"C:/Users/WANG/workspace/CanRotor_IMU/Utilities/Components/n25q128a" -I"C:/Users/WANG/workspace/CanRotor_IMU/Utilities/Components/n25q256a" -I"C:/Users/WANG/workspace/CanRotor_IMU/Utilities/Components/n25q512a" -I"C:/Users/WANG/workspace/CanRotor_IMU/Utilities/Components/otm8009a" -I"C:/Users/WANG/workspace/CanRotor_IMU/Utilities/Components/ov2640" -I"C:/Users/WANG/workspace/CanRotor_IMU/Utilities/Components/s25fl512s" -I"C:/Users/WANG/workspace/CanRotor_IMU/Utilities/Components/s5k5cag" -I"C:/Users/WANG/workspace/CanRotor_IMU/Utilities/Components/st7735" -I"C:/Users/WANG/workspace/CanRotor_IMU/Utilities/Components/st7789h2" -I"C:/Users/WANG/workspace/CanRotor_IMU/Utilities/Components/stmpe1600" -I"C:/Users/WANG/workspace/CanRotor_IMU/Utilities/Components/stmpe811" -I"C:/Users/WANG/workspace/CanRotor_IMU/Utilities/Components/ts3510" -I"C:/Users/WANG/workspace/CanRotor_IMU/Utilities/Components/wm8994" -I"C:/Users/WANG/workspace/CanRotor_IMU/Utilities" -I"C:/Users/WANG/workspace/CanRotor_IMU/Utilities/STM32F4-Discovery" -O0 -g3 -Wall -fmessage-length=0 -ffunction-sections -c -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


