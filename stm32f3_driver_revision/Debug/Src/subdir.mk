################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Src/led_toggle.c 

OBJS += \
./Src/led_toggle.o 

C_DEPS += \
./Src/led_toggle.d 


# Each subdirectory must supply rules for building sources it contributes
Src/led_toggle.o: ../Src/led_toggle.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DSTM32 -DSTM32F3 -DDEBUG -DSTM32F303RETx -DNUCLEO_F303RE -c -I../Inc -I"C:/Users/Chirag Goyal/Desktop/stm workspace/githubtest/stm32f3_driver_revision/driver/inc" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Src/led_toggle.d" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

