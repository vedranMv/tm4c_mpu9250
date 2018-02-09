################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Each subdirectory must supply rules for building sources it contributes
HAL/tm4c1294/hal_common_tm4c.obj: ../HAL/tm4c1294/hal_common_tm4c.c $(GEN_OPTS) | $(GEN_HDRS)
	@echo 'Building file: $<'
	@echo 'Invoking: ARM Compiler'
	"/home/v125/Programming/ti/ccsv7/tools/compiler/ti-cgt-arm_16.9.3.LTS/bin/armcl" -mv7M4 --code_state=16 --float_support=FPv4SPD16 -me --include_path="/home/v125/Programming/ti/workspace1/tm4c_mpu9250" --include_path="/media/v125/shared/Programming/ti/TivaWare_C_Series-2.1.4.178" --include_path="../../../../../media/v125/shared/Programming/ti/TivaWare_C_Series-2.1.4.178/utils" --include_path="/home/v125/Programming/ti/ccsv7/tools/compiler/ti-cgt-arm_16.9.3.LTS/include" --define=ccs="ccs" --define=PART_TM4C1294NCPDT -g --gcc --diag_warning=225 --diag_wrap=off --display_error_number --abi=eabi --preproc_with_compile --preproc_dependency="HAL/tm4c1294/hal_common_tm4c.d" --obj_directory="HAL/tm4c1294" $(GEN_OPTS__FLAG) "$(shell echo $<)"
	@echo 'Finished building: $<'
	@echo ' '

HAL/tm4c1294/hal_mpu_i2c_tm4c.obj: ../HAL/tm4c1294/hal_mpu_i2c_tm4c.c $(GEN_OPTS) | $(GEN_HDRS)
	@echo 'Building file: $<'
	@echo 'Invoking: ARM Compiler'
	"/home/v125/Programming/ti/ccsv7/tools/compiler/ti-cgt-arm_16.9.3.LTS/bin/armcl" -mv7M4 --code_state=16 --float_support=FPv4SPD16 -me --include_path="/home/v125/Programming/ti/workspace1/tm4c_mpu9250" --include_path="/media/v125/shared/Programming/ti/TivaWare_C_Series-2.1.4.178" --include_path="../../../../../media/v125/shared/Programming/ti/TivaWare_C_Series-2.1.4.178/utils" --include_path="/home/v125/Programming/ti/ccsv7/tools/compiler/ti-cgt-arm_16.9.3.LTS/include" --define=ccs="ccs" --define=PART_TM4C1294NCPDT -g --gcc --diag_warning=225 --diag_wrap=off --display_error_number --abi=eabi --preproc_with_compile --preproc_dependency="HAL/tm4c1294/hal_mpu_i2c_tm4c.d" --obj_directory="HAL/tm4c1294" $(GEN_OPTS__FLAG) "$(shell echo $<)"
	@echo 'Finished building: $<'
	@echo ' '

HAL/tm4c1294/hal_mpu_spi_tm4c.obj: ../HAL/tm4c1294/hal_mpu_spi_tm4c.c $(GEN_OPTS) | $(GEN_HDRS)
	@echo 'Building file: $<'
	@echo 'Invoking: ARM Compiler'
	"/home/v125/Programming/ti/ccsv7/tools/compiler/ti-cgt-arm_16.9.3.LTS/bin/armcl" -mv7M4 --code_state=16 --float_support=FPv4SPD16 -me --include_path="/home/v125/Programming/ti/workspace1/tm4c_mpu9250" --include_path="/media/v125/shared/Programming/ti/TivaWare_C_Series-2.1.4.178" --include_path="../../../../../media/v125/shared/Programming/ti/TivaWare_C_Series-2.1.4.178/utils" --include_path="/home/v125/Programming/ti/ccsv7/tools/compiler/ti-cgt-arm_16.9.3.LTS/include" --define=ccs="ccs" --define=PART_TM4C1294NCPDT -g --gcc --diag_warning=225 --diag_wrap=off --display_error_number --abi=eabi --preproc_with_compile --preproc_dependency="HAL/tm4c1294/hal_mpu_spi_tm4c.d" --obj_directory="HAL/tm4c1294" $(GEN_OPTS__FLAG) "$(shell echo $<)"
	@echo 'Finished building: $<'
	@echo ' '


