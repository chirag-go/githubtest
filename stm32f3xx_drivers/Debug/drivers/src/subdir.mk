################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../drivers/src/stm32f3xx_gpio_driver.c \
../drivers/src/stm32f3xx_spi.c 

OBJS += \
./drivers/src/stm32f3xx_gpio_driver.o \
./drivers/src/stm32f3xx_spi.o 

C_DEPS += \
./drivers/src/stm32f3xx_gpio_driver.d \
./drivers/src/stm32f3xx_spi.d 


# Each subdirectory must supply rules for building sources it contributes
drivers/src/stm32f3xx_gpio_driver.o: ../drivers/src/stm32f3xx_gpio_driver.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DSTM32 -DSTM32F3 -DDEBUG -DSTM32F303RETx -DNUCLEO_F303RE -c -I../Inc -I"C:/Users/Chirag Goyal/Desktop/stm workspace/githubtest/stm32f3xx_drivers/drivers/inc" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"drivers/src/stm32f3xx_gpio_driver.d" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"
drivers/src/stm32f3xx_spi.o: ../drivers/src/stm32f3xx_spi.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DSTM32 -DSTM32F3 -DDEBUG -DSTM32F303RETx -DNUCLEO_F303RE -c -I../Inc -I"C:/Users/Chirag Goyal/Desktop/stm workspace/githubtest/stm32f3xx_drivers/drivers/inc" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"drivers/src/stm32f3xx_spi.d" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

