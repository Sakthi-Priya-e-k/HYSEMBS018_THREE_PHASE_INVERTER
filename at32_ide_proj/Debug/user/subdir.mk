################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
C:/Users/Admin/Downloads/ARTERY_PROGRAM_TIMER_12_11/project/at_start_f407/examples/emac/tcp_server/src/at32_emac.c \
C:/Users/Admin/Downloads/ARTERY_PROGRAM_TIMER_12_11/project/at_start_f407/examples/emac/tcp_server/src/at32f403a_407_clock.c \
C:/Users/Admin/Downloads/ARTERY_PROGRAM_TIMER_12_11/project/at_start_f407/examples/emac/tcp_server/src/at32f403a_407_int.c \
C:/Users/Admin/Downloads/ARTERY_PROGRAM_TIMER_12_11/project/at_start_f407/examples/emac/tcp_server/src/main.c \
C:/Users/Admin/Downloads/ARTERY_PROGRAM_TIMER_12_11/project/at_start_f407/examples/emac/tcp_server/src/netconf.c \
C:/Users/Admin/Downloads/ARTERY_PROGRAM_TIMER_12_11/project/at_start_f407/examples/emac/tcp_server/src/tcp_server.c 

OBJS += \
./user/at32_emac.o \
./user/at32f403a_407_clock.o \
./user/at32f403a_407_int.o \
./user/main.o \
./user/netconf.o \
./user/tcp_server.o 

C_DEPS += \
./user/at32_emac.d \
./user/at32f403a_407_clock.d \
./user/at32f403a_407_int.d \
./user/main.d \
./user/netconf.d \
./user/tcp_server.d 


# Each subdirectory must supply rules for building sources it contributes
user/at32_emac.o: C:/Users/Admin/Downloads/ARTERY_PROGRAM_TIMER_12_11/project/at_start_f407/examples/emac/tcp_server/src/at32_emac.c user/subdir.mk
	@echo 'Building file: $<'
	@echo 'Invoking: GNU Arm Cross C Compiler'
	arm-none-eabi-gcc -mcpu=cortex-m4 -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16 -O0 -ffunction-sections  -g -DAT32F407VGT7 -DUSE_STDPERIPH_DRIVER -DAT_START_F407_V1 -DTRACE -DOS_USE_TRACE_SEMIHOSTING_DEBUG -I"../../../../../../../libraries/drivers/inc" -I"../../../../../../../libraries/cmsis/cm4/core_support" -I"../../../../../../../libraries/cmsis/cm4/device_support" -I"../../inc" -I"../../../../../../at32f403a_407_board" -I"../../../../../../../middlewares/3rd_party/lwip_2.1.2/port" -I"../../../../../../../middlewares/3rd_party/lwip_2.1.2/port/arch" -I"../../../../../../../middlewares/3rd_party/lwip_2.1.2/src/include" -I"../../../../../../../middlewares/3rd_party/lwip_2.1.2/src/include/lwip" -I"../../../../../../../middlewares/3rd_party/lwip_2.1.2/src/apps/ping" -std=c99 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -c -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '

user/at32f403a_407_clock.o: C:/Users/Admin/Downloads/ARTERY_PROGRAM_TIMER_12_11/project/at_start_f407/examples/emac/tcp_server/src/at32f403a_407_clock.c user/subdir.mk
	@echo 'Building file: $<'
	@echo 'Invoking: GNU Arm Cross C Compiler'
	arm-none-eabi-gcc -mcpu=cortex-m4 -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16 -O0 -ffunction-sections  -g -DAT32F407VGT7 -DUSE_STDPERIPH_DRIVER -DAT_START_F407_V1 -DTRACE -DOS_USE_TRACE_SEMIHOSTING_DEBUG -I"../../../../../../../libraries/drivers/inc" -I"../../../../../../../libraries/cmsis/cm4/core_support" -I"../../../../../../../libraries/cmsis/cm4/device_support" -I"../../inc" -I"../../../../../../at32f403a_407_board" -I"../../../../../../../middlewares/3rd_party/lwip_2.1.2/port" -I"../../../../../../../middlewares/3rd_party/lwip_2.1.2/port/arch" -I"../../../../../../../middlewares/3rd_party/lwip_2.1.2/src/include" -I"../../../../../../../middlewares/3rd_party/lwip_2.1.2/src/include/lwip" -I"../../../../../../../middlewares/3rd_party/lwip_2.1.2/src/apps/ping" -std=c99 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -c -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '

user/at32f403a_407_int.o: C:/Users/Admin/Downloads/ARTERY_PROGRAM_TIMER_12_11/project/at_start_f407/examples/emac/tcp_server/src/at32f403a_407_int.c user/subdir.mk
	@echo 'Building file: $<'
	@echo 'Invoking: GNU Arm Cross C Compiler'
	arm-none-eabi-gcc -mcpu=cortex-m4 -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16 -O0 -ffunction-sections  -g -DAT32F407VGT7 -DUSE_STDPERIPH_DRIVER -DAT_START_F407_V1 -DTRACE -DOS_USE_TRACE_SEMIHOSTING_DEBUG -I"../../../../../../../libraries/drivers/inc" -I"../../../../../../../libraries/cmsis/cm4/core_support" -I"../../../../../../../libraries/cmsis/cm4/device_support" -I"../../inc" -I"../../../../../../at32f403a_407_board" -I"../../../../../../../middlewares/3rd_party/lwip_2.1.2/port" -I"../../../../../../../middlewares/3rd_party/lwip_2.1.2/port/arch" -I"../../../../../../../middlewares/3rd_party/lwip_2.1.2/src/include" -I"../../../../../../../middlewares/3rd_party/lwip_2.1.2/src/include/lwip" -I"../../../../../../../middlewares/3rd_party/lwip_2.1.2/src/apps/ping" -std=c99 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -c -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '

user/main.o: C:/Users/Admin/Downloads/ARTERY_PROGRAM_TIMER_12_11/project/at_start_f407/examples/emac/tcp_server/src/main.c user/subdir.mk
	@echo 'Building file: $<'
	@echo 'Invoking: GNU Arm Cross C Compiler'
	arm-none-eabi-gcc -mcpu=cortex-m4 -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16 -O0 -ffunction-sections  -g -DAT32F407VGT7 -DUSE_STDPERIPH_DRIVER -DAT_START_F407_V1 -DTRACE -DOS_USE_TRACE_SEMIHOSTING_DEBUG -I"../../../../../../../libraries/drivers/inc" -I"../../../../../../../libraries/cmsis/cm4/core_support" -I"../../../../../../../libraries/cmsis/cm4/device_support" -I"../../inc" -I"../../../../../../at32f403a_407_board" -I"../../../../../../../middlewares/3rd_party/lwip_2.1.2/port" -I"../../../../../../../middlewares/3rd_party/lwip_2.1.2/port/arch" -I"../../../../../../../middlewares/3rd_party/lwip_2.1.2/src/include" -I"../../../../../../../middlewares/3rd_party/lwip_2.1.2/src/include/lwip" -I"../../../../../../../middlewares/3rd_party/lwip_2.1.2/src/apps/ping" -std=c99 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -c -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '

user/netconf.o: C:/Users/Admin/Downloads/ARTERY_PROGRAM_TIMER_12_11/project/at_start_f407/examples/emac/tcp_server/src/netconf.c user/subdir.mk
	@echo 'Building file: $<'
	@echo 'Invoking: GNU Arm Cross C Compiler'
	arm-none-eabi-gcc -mcpu=cortex-m4 -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16 -O0 -ffunction-sections  -g -DAT32F407VGT7 -DUSE_STDPERIPH_DRIVER -DAT_START_F407_V1 -DTRACE -DOS_USE_TRACE_SEMIHOSTING_DEBUG -I"../../../../../../../libraries/drivers/inc" -I"../../../../../../../libraries/cmsis/cm4/core_support" -I"../../../../../../../libraries/cmsis/cm4/device_support" -I"../../inc" -I"../../../../../../at32f403a_407_board" -I"../../../../../../../middlewares/3rd_party/lwip_2.1.2/port" -I"../../../../../../../middlewares/3rd_party/lwip_2.1.2/port/arch" -I"../../../../../../../middlewares/3rd_party/lwip_2.1.2/src/include" -I"../../../../../../../middlewares/3rd_party/lwip_2.1.2/src/include/lwip" -I"../../../../../../../middlewares/3rd_party/lwip_2.1.2/src/apps/ping" -std=c99 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -c -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '

user/tcp_server.o: C:/Users/Admin/Downloads/ARTERY_PROGRAM_TIMER_12_11/project/at_start_f407/examples/emac/tcp_server/src/tcp_server.c user/subdir.mk
	@echo 'Building file: $<'
	@echo 'Invoking: GNU Arm Cross C Compiler'
	arm-none-eabi-gcc -mcpu=cortex-m4 -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16 -O0 -ffunction-sections  -g -DAT32F407VGT7 -DUSE_STDPERIPH_DRIVER -DAT_START_F407_V1 -DTRACE -DOS_USE_TRACE_SEMIHOSTING_DEBUG -I"../../../../../../../libraries/drivers/inc" -I"../../../../../../../libraries/cmsis/cm4/core_support" -I"../../../../../../../libraries/cmsis/cm4/device_support" -I"../../inc" -I"../../../../../../at32f403a_407_board" -I"../../../../../../../middlewares/3rd_party/lwip_2.1.2/port" -I"../../../../../../../middlewares/3rd_party/lwip_2.1.2/port/arch" -I"../../../../../../../middlewares/3rd_party/lwip_2.1.2/src/include" -I"../../../../../../../middlewares/3rd_party/lwip_2.1.2/src/include/lwip" -I"../../../../../../../middlewares/3rd_party/lwip_2.1.2/src/apps/ping" -std=c99 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -c -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


