################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../src/ConfigParameters.c \
../src/ConfigParams.c \
../src/FOCCurrentControl.c \
../src/FOCCurrentControl_data.c \
../src/FOCSpeedControl.c \
../src/hal_entry.c \
../src/rtGetInf.c \
../src/rtGetNaN.c \
../src/rt_nonfinite.c 

C_DEPS += \
./src/ConfigParameters.d \
./src/ConfigParams.d \
./src/FOCCurrentControl.d \
./src/FOCCurrentControl_data.d \
./src/FOCSpeedControl.d \
./src/hal_entry.d \
./src/rtGetInf.d \
./src/rtGetNaN.d \
./src/rt_nonfinite.d 

OBJS += \
./src/ConfigParameters.o \
./src/ConfigParams.o \
./src/FOCCurrentControl.o \
./src/FOCCurrentControl_data.o \
./src/FOCSpeedControl.o \
./src/hal_entry.o \
./src/rtGetInf.o \
./src/rtGetNaN.o \
./src/rt_nonfinite.o 

SREC += \
000.srec 

MAP += \
000.map 


# Each subdirectory must supply rules for building sources it contributes
src/%.o: ../src/%.c
	$(file > $@.in,-mcpu=cortex-m33 -mthumb -mfloat-abi=hard -mfpu=fpv5-sp-d16 -O2 -fmessage-length=0 -fsigned-char -ffunction-sections -fdata-sections -fno-strict-aliasing -Wunused -Wuninitialized -Wall -Wextra -Wmissing-declarations -Wconversion -Wpointer-arith -Wshadow -Wlogical-op -Waggregate-return -Wfloat-equal -g -D_RENESAS_RA_ -D_RA_CORE=CM33 -D_RA_ORDINAL=1 -I"C:/Users/ikeha/e2_studio/workspace/000/src" -I"." -I"C:/Users/ikeha/e2_studio/workspace/000/ra/fsp/inc" -I"C:/Users/ikeha/e2_studio/workspace/000/ra/fsp/inc/api" -I"C:/Users/ikeha/e2_studio/workspace/000/ra/fsp/inc/instances" -I"C:/Users/ikeha/e2_studio/workspace/000/ra/arm/CMSIS_6/CMSIS/Core/Include" -I"C:/Users/ikeha/e2_studio/workspace/000/ra_gen" -I"C:/Users/ikeha/e2_studio/workspace/000/ra_cfg/fsp_cfg/bsp" -I"C:/Users/ikeha/e2_studio/workspace/000/ra_cfg/fsp_cfg" -std=c99 -Wno-stringop-overflow -Wno-format-truncation --param=min-pagesize=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -c -o "$@" -x c "$<")
	@echo Building file: $< && arm-none-eabi-gcc @"$@.in"

