################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Each subdirectory must supply rules for building sources it contributes
mpu9250/MahonyAHRS.obj: ../mpu9250/MahonyAHRS.cpp $(GEN_OPTS) | $(GEN_HDRS)
	@echo 'Building file: $<'
	@echo 'Invoking: ARM Compiler'
	"/home/v125/Programming/ti/ccsv7/tools/compiler/ti-cgt-arm_16.9.3.LTS/bin/armcl" -mv7M4 --code_state=16 --float_support=FPv4SPD16 -me --include_path="/home/v125/Programming/ti/workspace1/tm4c_mpu9250" --include_path="/media/v125/shared/Programming/ti/TivaWare_C_Series-2.1.4.178" --include_path="../../../../../media/v125/shared/Programming/ti/TivaWare_C_Series-2.1.4.178/utils" --include_path="/home/v125/Programming/ti/ccsv7/tools/compiler/ti-cgt-arm_16.9.3.LTS/include" --define=ccs="ccs" --define=PART_TM4C1294NCPDT -g --gcc --diag_warning=225 --diag_wrap=off --display_error_number --abi=eabi --preproc_with_compile --preproc_dependency="mpu9250/MahonyAHRS.d" --obj_directory="mpu9250" $(GEN_OPTS__FLAG) "$(shell echo $<)"
	@echo 'Finished building: $<'
	@echo ' '

mpu9250/api_mpu9250.obj: ../mpu9250/api_mpu9250.c $(GEN_OPTS) | $(GEN_HDRS)
	@echo 'Building file: $<'
	@echo 'Invoking: ARM Compiler'
	"/home/v125/Programming/ti/ccsv7/tools/compiler/ti-cgt-arm_16.9.3.LTS/bin/armcl" -mv7M4 --code_state=16 --float_support=FPv4SPD16 -me --include_path="/home/v125/Programming/ti/workspace1/tm4c_mpu9250" --include_path="/media/v125/shared/Programming/ti/TivaWare_C_Series-2.1.4.178" --include_path="../../../../../media/v125/shared/Programming/ti/TivaWare_C_Series-2.1.4.178/utils" --include_path="/home/v125/Programming/ti/ccsv7/tools/compiler/ti-cgt-arm_16.9.3.LTS/include" --define=ccs="ccs" --define=PART_TM4C1294NCPDT -g --gcc --diag_warning=225 --diag_wrap=off --display_error_number --abi=eabi --preproc_with_compile --preproc_dependency="mpu9250/api_mpu9250.d" --obj_directory="mpu9250" $(GEN_OPTS__FLAG) "$(shell echo $<)"
	@echo 'Finished building: $<'
	@echo ' '

mpu9250/dmp_mpu9250.obj: ../mpu9250/dmp_mpu9250.cpp $(GEN_OPTS) | $(GEN_HDRS)
	@echo 'Building file: $<'
	@echo 'Invoking: ARM Compiler'
	"/home/v125/Programming/ti/ccsv7/tools/compiler/ti-cgt-arm_16.9.3.LTS/bin/armcl" -mv7M4 --code_state=16 --float_support=FPv4SPD16 -me --include_path="/home/v125/Programming/ti/workspace1/tm4c_mpu9250" --include_path="/media/v125/shared/Programming/ti/TivaWare_C_Series-2.1.4.178" --include_path="../../../../../media/v125/shared/Programming/ti/TivaWare_C_Series-2.1.4.178/utils" --include_path="/home/v125/Programming/ti/ccsv7/tools/compiler/ti-cgt-arm_16.9.3.LTS/include" --define=ccs="ccs" --define=PART_TM4C1294NCPDT -g --gcc --diag_warning=225 --diag_wrap=off --display_error_number --abi=eabi --preproc_with_compile --preproc_dependency="mpu9250/dmp_mpu9250.d" --obj_directory="mpu9250" $(GEN_OPTS__FLAG) "$(shell echo $<)"
	@echo 'Finished building: $<'
	@echo ' '

mpu9250/nondmp_mpu9250.obj: ../mpu9250/nondmp_mpu9250.cpp $(GEN_OPTS) | $(GEN_HDRS)
	@echo 'Building file: $<'
	@echo 'Invoking: ARM Compiler'
	"/home/v125/Programming/ti/ccsv7/tools/compiler/ti-cgt-arm_16.9.3.LTS/bin/armcl" -mv7M4 --code_state=16 --float_support=FPv4SPD16 -me --include_path="/home/v125/Programming/ti/workspace1/tm4c_mpu9250" --include_path="/media/v125/shared/Programming/ti/TivaWare_C_Series-2.1.4.178" --include_path="../../../../../media/v125/shared/Programming/ti/TivaWare_C_Series-2.1.4.178/utils" --include_path="/home/v125/Programming/ti/ccsv7/tools/compiler/ti-cgt-arm_16.9.3.LTS/include" --define=ccs="ccs" --define=PART_TM4C1294NCPDT -g --gcc --diag_warning=225 --diag_wrap=off --display_error_number --abi=eabi --preproc_with_compile --preproc_dependency="mpu9250/nondmp_mpu9250.d" --obj_directory="mpu9250" $(GEN_OPTS__FLAG) "$(shell echo $<)"
	@echo 'Finished building: $<'
	@echo ' '


