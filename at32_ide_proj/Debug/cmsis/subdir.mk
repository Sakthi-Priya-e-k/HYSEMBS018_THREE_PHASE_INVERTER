################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
C:/Users/Admin/Downloads/ARTERY_PROGRAM_TIMER_12_11/libraries/cmsis/cm4/device_support/system_at32f403a_407.c 

OBJS += \
./cmsis/system_at32f403a_407.o 

C_DEPS += \
./cmsis/system_at32f403a_407.d 


# Each subdirectory must supply rules for building sources it contributes
cmsis/system_at32f403a_407.o: C:/Users/Admin/Downloads/ARTERY_PROGRAM_TIMER_12_11/libraries/cmsis/cm4/device_support/system_at32f403a_407.c cmsis/subdir.mk
	@echo 'Building file: $<'
	@echo 'Invoking: GNU Arm Cross C Compiler'
	arm-none-eabi-gcc -mcpu=cortex-m4 -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16 -O0 -ffunction-sections  -g -DAT32F407VGT7 -DUSE_STDPERIPH_DRIVER -DAT_START_F407_V1 -DTRACE -DOS_USE_TRACE_SEMIHOSTING_DEBUG -I"../../../../../../../libraries/drivers/inc" -I"../../../../../../../libraries/cmsis/cm4/core_support" -I"../../../../../../../libraries/cmsis/cm4/device_support" -I"../../inc" -I"../../../../../../at32f403a_407_board" -I"../../../../../../../middlewares/3rd_party/lwip_2.1.2/port" -I"../../../../../../../middlewares/3rd_party/lwip_2.1.2/port/arch" -I"../../../../../../../middlewares/3rd_party/lwip_2.1.2/src/include" -I"../../../../../../../middlewares/3rd_party/lwip_2.1.2/src/include/lwip" -I"../../../../../../../middlewares/3rd_party/lwip_2.1.2/src/apps/ping" -std=c99 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -c -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


