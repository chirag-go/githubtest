################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Src/005spi_tx_only_arudino.c 

OBJS += \
./Src/005spi_tx_only_arudino.o 

C_DEPS += \
./Src/005spi_tx_only_arudino.d 


# Each subdirectory must supply rules for building sources it contributes
Src/005spi_tx_only_arudino.o: ../Src/005spi_tx_only_arudino.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DSTM32 -DSTM32F3 -DDEBUG -DSTM32F303RETx -DNUCLEO_F303RE -c -I../Inc -I"C:/Users/Chirag Goyal/Desktop/stm workspace/githubtest/stm32f3xx_drivers/drivers/inc" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Src/005spi_tx_only_arudino.d" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

