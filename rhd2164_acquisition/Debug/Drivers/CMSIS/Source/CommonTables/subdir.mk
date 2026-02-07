################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Drivers/CMSIS/Source/CommonTables/CommonTables.c \
../Drivers/CMSIS/Source/CommonTables/arm_const_structs.c 

OBJS += \
./Drivers/CMSIS/Source/CommonTables/CommonTables.o \
./Drivers/CMSIS/Source/CommonTables/arm_const_structs.o 

C_DEPS += \
./Drivers/CMSIS/Source/CommonTables/CommonTables.d \
./Drivers/CMSIS/Source/CommonTables/arm_const_structs.d 


# Each subdirectory must supply rules for building sources it contributes
Drivers/CMSIS/Source/CommonTables/%.o Drivers/CMSIS/Source/CommonTables/%.su Drivers/CMSIS/Source/CommonTables/%.cyclo: ../Drivers/CMSIS/Source/CommonTables/%.c Drivers/CMSIS/Source/CommonTables/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m7 -std=gnu11 -g3 -DDEBUG -DSTM32H723xx -DUSE_HAL_DRIVER -DUSE_PWR_LDO_SUPPLY -c -I../Core/Inc -I../Drivers/STM32H7xx_HAL_Driver/Inc -I../Drivers/CMSIS/Device/ST/STM32H7xx/Include -I../Drivers/CMSIS/Include -I../Drivers/STM32H7xx_HAL_Driver/Inc/Legacy -I../Middlewares/ST/ARM/DSP/Inc -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Drivers-2f-CMSIS-2f-Source-2f-CommonTables

clean-Drivers-2f-CMSIS-2f-Source-2f-CommonTables:
	-$(RM) ./Drivers/CMSIS/Source/CommonTables/CommonTables.cyclo ./Drivers/CMSIS/Source/CommonTables/CommonTables.d ./Drivers/CMSIS/Source/CommonTables/CommonTables.o ./Drivers/CMSIS/Source/CommonTables/CommonTables.su ./Drivers/CMSIS/Source/CommonTables/arm_const_structs.cyclo ./Drivers/CMSIS/Source/CommonTables/arm_const_structs.d ./Drivers/CMSIS/Source/CommonTables/arm_const_structs.o ./Drivers/CMSIS/Source/CommonTables/arm_const_structs.su

.PHONY: clean-Drivers-2f-CMSIS-2f-Source-2f-CommonTables

