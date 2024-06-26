################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Middle/button.c \
../Middle/dht.c \
../Middle/door_sensor.c \
../Middle/eventButtonCb.c \
../Middle/eventSerialCb.c \
../Middle/fanctrl.c \
../Middle/irctrl.c \
../Middle/led.c \
../Middle/lm35.c \
../Middle/mq135.c \
../Middle/rainwater.c \
../Middle/rs233.c \
../Middle/simcom.c \
../Middle/timer.c 

OBJS += \
./Middle/button.o \
./Middle/dht.o \
./Middle/door_sensor.o \
./Middle/eventButtonCb.o \
./Middle/eventSerialCb.o \
./Middle/fanctrl.o \
./Middle/irctrl.o \
./Middle/led.o \
./Middle/lm35.o \
./Middle/mq135.o \
./Middle/rainwater.o \
./Middle/rs233.o \
./Middle/simcom.o \
./Middle/timer.o 

C_DEPS += \
./Middle/button.d \
./Middle/dht.d \
./Middle/door_sensor.d \
./Middle/eventButtonCb.d \
./Middle/eventSerialCb.d \
./Middle/fanctrl.d \
./Middle/irctrl.d \
./Middle/led.d \
./Middle/lm35.d \
./Middle/mq135.d \
./Middle/rainwater.d \
./Middle/rs233.d \
./Middle/simcom.d \
./Middle/timer.d 


# Each subdirectory must supply rules for building sources it contributes
Middle/%.o Middle/%.su Middle/%.cyclo: ../Middle/%.c Middle/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F401xE -c -I../Core/Inc -I"C:/Users/HoangNH/Desktop/SmartTech/bts-sim7600/BTS_Station/Middle" -I"C:/Users/HoangNH/Desktop/SmartTech/bts-sim7600/BTS_Station/Utilities" -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Middle

clean-Middle:
	-$(RM) ./Middle/button.cyclo ./Middle/button.d ./Middle/button.o ./Middle/button.su ./Middle/dht.cyclo ./Middle/dht.d ./Middle/dht.o ./Middle/dht.su ./Middle/door_sensor.cyclo ./Middle/door_sensor.d ./Middle/door_sensor.o ./Middle/door_sensor.su ./Middle/eventButtonCb.cyclo ./Middle/eventButtonCb.d ./Middle/eventButtonCb.o ./Middle/eventButtonCb.su ./Middle/eventSerialCb.cyclo ./Middle/eventSerialCb.d ./Middle/eventSerialCb.o ./Middle/eventSerialCb.su ./Middle/fanctrl.cyclo ./Middle/fanctrl.d ./Middle/fanctrl.o ./Middle/fanctrl.su ./Middle/irctrl.cyclo ./Middle/irctrl.d ./Middle/irctrl.o ./Middle/irctrl.su ./Middle/led.cyclo ./Middle/led.d ./Middle/led.o ./Middle/led.su ./Middle/lm35.cyclo ./Middle/lm35.d ./Middle/lm35.o ./Middle/lm35.su ./Middle/mq135.cyclo ./Middle/mq135.d ./Middle/mq135.o ./Middle/mq135.su ./Middle/rainwater.cyclo ./Middle/rainwater.d ./Middle/rainwater.o ./Middle/rainwater.su ./Middle/rs233.cyclo ./Middle/rs233.d ./Middle/rs233.o ./Middle/rs233.su ./Middle/simcom.cyclo ./Middle/simcom.d ./Middle/simcom.o ./Middle/simcom.su ./Middle/timer.cyclo ./Middle/timer.d ./Middle/timer.o ./Middle/timer.su

.PHONY: clean-Middle

