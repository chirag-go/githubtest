################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../driver/src/gpio_driver.c 

OBJS += \
./driver/src/gpio_driver.o 

C_DEPS += \
./driver/src/gpio_driver.d 


# Each subdirectory must supply rules for building sources it contributes
driver/src/gpio_driver.o: ../driver/src/gpio_driver.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DSTM32 -DSTM32F3 -DDEBUG -DSTM32F303RETx -DNUCLEO_F303RE -c -I../Inc -I"C:/Users/Chirag Goyal/Desktop/stm workspace/githubtest/stm32f3_driver_revision/driver/inc" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"driver/src/gpio_driver.d" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

