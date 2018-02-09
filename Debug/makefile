################################################################################
# Automatically-generated file. Do not edit!
################################################################################

CG_TOOL_ROOT := /home/v125/Programming/ti/ccsv7/tools/compiler/ti-cgt-arm_16.9.3.LTS

GEN_OPTS__FLAG := 
GEN_CMDS__FLAG := 

ORDERED_OBJS += \
"./main.obj" \
"./tm4c1294ncpdt_startup_ccs.obj" \
"./uartstdio.obj" \
"./HAL/tm4c1294/hal_common_tm4c.obj" \
"./HAL/tm4c1294/hal_mpu_i2c_tm4c.obj" \
"./HAL/tm4c1294/hal_mpu_spi_tm4c.obj" \
"./libs/myLib.obj" \
"./mpu9250/MahonyAHRS.obj" \
"./mpu9250/api_mpu9250.obj" \
"./mpu9250/dmp_mpu9250.obj" \
"./mpu9250/nondmp_mpu9250.obj" \
"./mpu9250/eMPL/inv_mpu.obj" \
"./mpu9250/eMPL/inv_mpu_dmp_motion_driver.obj" \
"./serialPort/uartHW.obj" \
"../tm4c1294ncpdt.cmd" \
$(GEN_CMDS__FLAG) \
-llibc.a \
-l"/media/v125/shared/Programming/ti/TivaWare_C_Series-2.1.4.178/driverlib/ccs/Debug/driverlib.lib" \

-include ../makefile.init

RM := rm -rf
RMDIR := rm -rf

# All of the sources participating in the build are defined here
-include sources.mk
-include subdir_vars.mk
-include HAL/tm4c1294/subdir_vars.mk
-include libs/subdir_vars.mk
-include mpu9250/subdir_vars.mk
-include mpu9250/eMPL/subdir_vars.mk
-include serialPort/subdir_vars.mk
-include subdir_rules.mk
-include HAL/tm4c1294/subdir_rules.mk
-include libs/subdir_rules.mk
-include mpu9250/subdir_rules.mk
-include mpu9250/eMPL/subdir_rules.mk
-include serialPort/subdir_rules.mk
-include objects.mk

ifneq ($(MAKECMDGOALS),clean)
ifneq ($(strip $(C55_DEPS)),)
-include $(C55_DEPS)
endif
ifneq ($(strip $(C_UPPER_DEPS)),)
-include $(C_UPPER_DEPS)
endif
ifneq ($(strip $(S67_DEPS)),)
-include $(S67_DEPS)
endif
ifneq ($(strip $(S62_DEPS)),)
-include $(S62_DEPS)
endif
ifneq ($(strip $(S_DEPS)),)
-include $(S_DEPS)
endif
ifneq ($(strip $(OPT_DEPS)),)
-include $(OPT_DEPS)
endif
ifneq ($(strip $(C??_DEPS)),)
-include $(C??_DEPS)
endif
ifneq ($(strip $(ASM_UPPER_DEPS)),)
-include $(ASM_UPPER_DEPS)
endif
ifneq ($(strip $(S??_DEPS)),)
-include $(S??_DEPS)
endif
ifneq ($(strip $(C64_DEPS)),)
-include $(C64_DEPS)
endif
ifneq ($(strip $(CXX_DEPS)),)
-include $(CXX_DEPS)
endif
ifneq ($(strip $(S64_DEPS)),)
-include $(S64_DEPS)
endif
ifneq ($(strip $(INO_DEPS)),)
-include $(INO_DEPS)
endif
ifneq ($(strip $(CLA_DEPS)),)
-include $(CLA_DEPS)
endif
ifneq ($(strip $(S55_DEPS)),)
-include $(S55_DEPS)
endif
ifneq ($(strip $(SV7A_DEPS)),)
-include $(SV7A_DEPS)
endif
ifneq ($(strip $(C62_DEPS)),)
-include $(C62_DEPS)
endif
ifneq ($(strip $(C67_DEPS)),)
-include $(C67_DEPS)
endif
ifneq ($(strip $(PDE_DEPS)),)
-include $(PDE_DEPS)
endif
ifneq ($(strip $(K_DEPS)),)
-include $(K_DEPS)
endif
ifneq ($(strip $(C_DEPS)),)
-include $(C_DEPS)
endif
ifneq ($(strip $(CC_DEPS)),)
-include $(CC_DEPS)
endif
ifneq ($(strip $(C++_DEPS)),)
-include $(C++_DEPS)
endif
ifneq ($(strip $(C43_DEPS)),)
-include $(C43_DEPS)
endif
ifneq ($(strip $(S43_DEPS)),)
-include $(S43_DEPS)
endif
ifneq ($(strip $(ASM_DEPS)),)
-include $(ASM_DEPS)
endif
ifneq ($(strip $(S_UPPER_DEPS)),)
-include $(S_UPPER_DEPS)
endif
ifneq ($(strip $(CPP_DEPS)),)
-include $(CPP_DEPS)
endif
ifneq ($(strip $(SA_DEPS)),)
-include $(SA_DEPS)
endif
endif

-include ../makefile.defs

# Add inputs and outputs from these tool invocations to the build variables 
EXE_OUTPUTS += \
tm4c_mpu9250.out \

EXE_OUTPUTS__QUOTED += \
"tm4c_mpu9250.out" \

BIN_OUTPUTS += \
tm4c_mpu9250.hex \

BIN_OUTPUTS__QUOTED += \
"tm4c_mpu9250.hex" \


# All Target
all: $(OBJS) $(CMD_SRCS) $(GEN_CMDS)
	@$(MAKE) --no-print-directory -Onone "tm4c_mpu9250.out"

# Tool invocations
tm4c_mpu9250.out: $(OBJS) $(CMD_SRCS) $(GEN_CMDS)
	@echo 'Building target: $@'
	@echo 'Invoking: ARM Linker'
	"/home/v125/Programming/ti/ccsv7/tools/compiler/ti-cgt-arm_16.9.3.LTS/bin/armcl" -mv7M4 --code_state=16 --float_support=FPv4SPD16 -me --define=ccs="ccs" --define=PART_TM4C1294NCPDT -g --gcc --diag_warning=225 --diag_wrap=off --display_error_number --abi=eabi -z -m"tm4c_mpu9250.map" --heap_size=0 --stack_size=512 -i"/home/v125/Programming/ti/ccsv7/tools/compiler/ti-cgt-arm_16.9.3.LTS/lib" -i"/home/v125/Programming/ti/ccsv7/tools/compiler/ti-cgt-arm_16.9.3.LTS/include" --reread_libs --diag_wrap=off --display_error_number --warn_sections --xml_link_info="tm4c_mpu9250_linkInfo.xml" --rom_model -o "tm4c_mpu9250.out" $(ORDERED_OBJS)
	@echo 'Finished building target: $@'
	@echo ' '

tm4c_mpu9250.hex: $(EXE_OUTPUTS)
	@echo 'Invoking: ARM Hex Utility'
	"/home/v125/Programming/ti/ccsv7/tools/compiler/ti-cgt-arm_16.9.3.LTS/bin/armhex"  -o "tm4c_mpu9250.hex" $(EXE_OUTPUTS__QUOTED)
	@echo 'Finished building: $@'
	@echo ' '

# Other Targets
clean:
	-$(RM) $(BIN_OUTPUTS__QUOTED)$(EXE_OUTPUTS__QUOTED)
	-$(RM) "main.obj" "tm4c1294ncpdt_startup_ccs.obj" "uartstdio.obj" "HAL/tm4c1294/hal_common_tm4c.obj" "HAL/tm4c1294/hal_mpu_i2c_tm4c.obj" "HAL/tm4c1294/hal_mpu_spi_tm4c.obj" "libs/myLib.obj" "mpu9250/MahonyAHRS.obj" "mpu9250/api_mpu9250.obj" "mpu9250/dmp_mpu9250.obj" "mpu9250/nondmp_mpu9250.obj" "mpu9250/eMPL/inv_mpu.obj" "mpu9250/eMPL/inv_mpu_dmp_motion_driver.obj" "serialPort/uartHW.obj" 
	-$(RM) "tm4c1294ncpdt_startup_ccs.d" "uartstdio.d" "HAL/tm4c1294/hal_common_tm4c.d" "HAL/tm4c1294/hal_mpu_i2c_tm4c.d" "HAL/tm4c1294/hal_mpu_spi_tm4c.d" "libs/myLib.d" "mpu9250/api_mpu9250.d" "mpu9250/eMPL/inv_mpu.d" "mpu9250/eMPL/inv_mpu_dmp_motion_driver.d" 
	-$(RM) "main.d" "mpu9250/MahonyAHRS.d" "mpu9250/dmp_mpu9250.d" "mpu9250/nondmp_mpu9250.d" "serialPort/uartHW.d" 
	-@echo 'Finished clean'
	-@echo ' '

.PHONY: all clean dependents
.SECONDARY:

-include ../makefile.targets

