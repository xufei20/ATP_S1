################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../BSP/Src/Focus.c \
../BSP/Src/ImageModule.c \
../BSP/Src/RangefinderModule.c \
../BSP/Src/ServoModule.c \
../BSP/Src/cmdPC.c \
../BSP/Src/tcpecho.c 

OBJS += \
./BSP/Src/Focus.o \
./BSP/Src/ImageModule.o \
./BSP/Src/RangefinderModule.o \
./BSP/Src/ServoModule.o \
./BSP/Src/cmdPC.o \
./BSP/Src/tcpecho.o 

C_DEPS += \
./BSP/Src/Focus.d \
./BSP/Src/ImageModule.d \
./BSP/Src/RangefinderModule.d \
./BSP/Src/ServoModule.d \
./BSP/Src/cmdPC.d \
./BSP/Src/tcpecho.d 


# Each subdirectory must supply rules for building sources it contributes
BSP/Src/%.o BSP/Src/%.su BSP/Src/%.cyclo: ../BSP/Src/%.c BSP/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m7 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32H743xx -c -I../Core/Inc -I../BSP/Inc -I../Drivers/STM32H7xx_HAL_Driver/Inc -I../Drivers/STM32H7xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32H7xx/Include -I../Drivers/CMSIS/Include -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -I../LWIP/App -I../LWIP/Target -I../Middlewares/Third_Party/LwIP/src/include -I../Middlewares/Third_Party/LwIP/system -I../Drivers/BSP/Components/lan8742 -I../Middlewares/Third_Party/LwIP/src/include/netif/ppp -I../Middlewares/Third_Party/LwIP/src/include/lwip -I../Middlewares/Third_Party/LwIP/src/include/lwip/apps -I../Middlewares/Third_Party/LwIP/src/include/lwip/priv -I../Middlewares/Third_Party/LwIP/src/include/lwip/prot -I../Middlewares/Third_Party/LwIP/src/include/netif -I../Middlewares/Third_Party/LwIP/src/include/compat/posix -I../Middlewares/Third_Party/LwIP/src/include/compat/posix/arpa -I../Middlewares/Third_Party/LwIP/src/include/compat/posix/net -I../Middlewares/Third_Party/LwIP/src/include/compat/posix/sys -I../Middlewares/Third_Party/LwIP/src/include/compat/stdc -I../Middlewares/Third_Party/LwIP/system/arch -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-BSP-2f-Src

clean-BSP-2f-Src:
	-$(RM) ./BSP/Src/Focus.cyclo ./BSP/Src/Focus.d ./BSP/Src/Focus.o ./BSP/Src/Focus.su ./BSP/Src/ImageModule.cyclo ./BSP/Src/ImageModule.d ./BSP/Src/ImageModule.o ./BSP/Src/ImageModule.su ./BSP/Src/RangefinderModule.cyclo ./BSP/Src/RangefinderModule.d ./BSP/Src/RangefinderModule.o ./BSP/Src/RangefinderModule.su ./BSP/Src/ServoModule.cyclo ./BSP/Src/ServoModule.d ./BSP/Src/ServoModule.o ./BSP/Src/ServoModule.su ./BSP/Src/cmdPC.cyclo ./BSP/Src/cmdPC.d ./BSP/Src/cmdPC.o ./BSP/Src/cmdPC.su ./BSP/Src/tcpecho.cyclo ./BSP/Src/tcpecho.d ./BSP/Src/tcpecho.o ./BSP/Src/tcpecho.su

.PHONY: clean-BSP-2f-Src

