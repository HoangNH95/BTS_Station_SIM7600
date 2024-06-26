################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Utilities/buff.c \
../Utilities/utilities.c 

OBJS += \
./Utilities/buff.o \
./Utilities/utilities.o 

C_DEPS += \
./Utilities/buff.d \
./Utilities/utilities.d 


# Each subdirectory must supply rules for building sources it contributes
Utilities/buff.o: C:/Users/HoangNH/Desktop/SmartTech/bts-sim7600/BTS_Station/Utilities/buff.c Utilities/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F401xE -c -I../Core/Inc -I"C:/Users/HoangNH/Desktop/SmartTech/bts-sim7600/BTS_Station/Middle" -I"C:/Users/HoangNH/Desktop/SmartTech/bts-sim7600/BTS_Station/Utilities" -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
Utilities/utilities.o: C:/Users/HoangNH/Desktop/SmartTech/bts-sim7600/BTS_Station/Utilities/utilities.c Utilities/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F401xE -c -I../Core/Inc -I"C:/Users/HoangNH/Desktop/SmartTech/bts-sim7600/BTS_Station/Middle" -I"C:/Users/HoangNH/Desktop/SmartTech/bts-sim7600/BTS_Station/Utilities" -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Utilities

clean-Utilities:
	-$(RM) ./Utilities/buff.cyclo ./Utilities/buff.d ./Utilities/buff.o ./Utilities/buff.su ./Utilities/utilities.cyclo ./Utilities/utilities.d ./Utilities/utilities.o ./Utilities/utilities.su

.PHONY: clean-Utilities

