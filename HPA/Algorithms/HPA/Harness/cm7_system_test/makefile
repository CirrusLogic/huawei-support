##############################################################################
#
# Makefile for Haptics Processing Algorithm System Test
#
##############################################################################
# TODO:  enter license info
#
#
#
#

# Variables for shell commands
RM              = rm -rf

# Variables for primary paths
ROOT_PATH       = $(abspath ../../../../../)
REPO_PATH       = $(abspath ../../../../)
BUILD_DIR       = $(REPO_PATH)/build
MAKEFILE_PATH   = $(PWD)

# Initialize source/dependency variables
ASM_SRCS =
C_SRCS =
OBJS =
ASM_OBJS =
INCLUDES =
LIBS =

# Initialize toolchain arguments
HPA_LIB_ARCH_FLAGS = -mthumb -mcpu=cortex-m7 -mfloat-abi=hard -mfpu=fpv5-d16
HPA_LIB_CFLAGS = $(HPA_LIB_ARCH_FLAGS) -gdwarf-2 -mlong-calls 
HPA_LIB_CFLAGS += -D__CORTEX_M7=1 -D__CMSIS_RTOS=1 -DARMCM7_DP -D__FPU_PRESENT=1 -D__STRICT_ANSI__
HPA_LIB_CFLAGS += -Wall -Werror -Wno-strict-aliasing -Wno-endif-labels -fno-common -ffunction-sections
HPA_LIB_CFLAGS += -Os

HPA_LIB_ASM_FLAGS = $(HPA_LIB_ARCH_FLAGS) -gdwarf-2 -mlong-calls
HPA_LIB_ASM_FLAGS += -D__CORTEX_M7=1 -D__CMSIS_RTOS=1 -DARMCM7_DP -D__FPU_PRESENT=1 

# Process command line variables per make goal

# Assign toolchain, shell executable names required per target
ifeq ($(MAKECMDGOALS), baremetal)
    CC          = arm-none-eabi-gcc
    LD          = arm-none-eabi-ld
    AR          = arm-none-eabi-ar
    AS          = arm-none-eabi-gcc
    OBJCOPY     = arm-none-eabi-objcopy
    OBJDUMP     = arm-none-eabi-objdump
    SIZE        = arm-none-eabi-size
    IS_VALID_BUILD = YES
endif

# Assign compiler, assembler, linker flags required per target
DEBUG_OPTIONS += -DDEBUG
DEBUG_OPTIONS += -g3

# use = or += not := else dependencies expand to nil
CFLAGS = $(HPA_LIB_CFLAGS)
CFLAGS += --specs=nano.specs
CFLAGS += -DUSE_HAL_DRIVER -DSTM32F767xx
CFLAGS += $(DEBUG_OPTIONS)
OUTPUT_NAME =
OUTPUT_EXT  =
ARFLAGS     = rcs
ASM_FLAGS = $(HPA_LIB_ASM_FLAGS)
ASM_FLAGS += --specs=nano.specs
ASM_FLAGS += $(DEBUG_OPTIONS)
LDFLAGS = $(HPA_LIB_ARCH_FLAGS)
LDFLAGS += -T"$(APP_PATH)/STM32F767ZITX_FLASH.ld"
LDFLAGS += --specs=nosys.specs --specs=nano.specs
LDFLAGS += -Wl,-Map="$(BUILD_DIR)/baremetal.map"
LDFLAGS += -Wl,--gc-sections
LDFLAGS += -static
LDFLAGS += -Wl,--start-group -lc -lm -Wl,--end-group

# Assign sources, includes, paths for driver and library
ALGO_LIBRARY_ALGORITHM_PATH = $(REPO_PATH)/Algorithms/HPA/Algorithm
ALGO_LIBRARY_MATH_PATH = $(REPO_PATH)/Algorithms/HPA/Algorithm/Math_Lib
ALGO_LIBRARY_API_PATH = $(REPO_PATH)/Algorithms/HPA/Api
INCLUDES += -I$(ALGO_LIBRARY_ALGORITHM_PATH)/Include -I$(ALGO_LIBRARY_API_PATH)/Include -I$(ALGO_LIBRARY_MATH_PATH)/Include
ALGO_LIBRARY = $(BUILD_DIR)/Algorithms/HPA/Algorithm/libhpa.a
LIBS += $(ALGO_LIBRARY)
#CFLAGS += -DHARD_CODED_CH

# Assign sources, includes, paths for Bullseye
BULLSEYE_PATH = $(REPO_PATH)/third_party/bullseye
BULLSEYE_SRCS = $BULLSEYE_PATH)/src/libcov-cirrusrtt.c
BULLSEYE_INC = -I$(REPO_PATH)/third_party/bullseye/inc

# Assign list of application source files required per target
APP_PATH = $(REPO_PATH)/Algorithms/HPA/Harness/cm7_system_test
STCUBE_PATH = $(REPO_PATH)/Algorithms/third_party/st/STM32Cube_FW_F7_V1.15.0/Drivers
STM32_EEPROM_EMU_PATH =$(REPO_PATH)/Algorithms/third_party/st/STM32Cube_FW_F7_V1.15.0/Projects/STM32F767ZI-Nucleo/Applications/EEPROM/EEPROM_Emulation
HAL_PATH = $(STCUBE_PATH)/STM32F7xx_HAL_Driver/Src
C_SRCS += $(APP_PATH)/main.c
C_SRCS += $(APP_PATH)/haptic_tables.c
C_SRCS += $(APP_PATH)/system_test_bsp.c
C_SRCS += $(APP_PATH)/stm32f7xx_it.c
C_SRCS += $(APP_PATH)/syscalls.c
C_SRCS += $(APP_PATH)/sysmem.c
C_SRCS += $(HAL_PATH)/stm32f7xx_hal_cortex.c
C_SRCS += $(HAL_PATH)/stm32f7xx_hal_gpio.c
C_SRCS += $(HAL_PATH)/stm32f7xx_hal_i2c_ex.c
C_SRCS += $(HAL_PATH)/stm32f7xx_hal_i2c.c
C_SRCS += $(HAL_PATH)/stm32f7xx_hal_pwr_ex.c
C_SRCS += $(HAL_PATH)/stm32f7xx_hal_pwr.c
C_SRCS += $(HAL_PATH)/stm32f7xx_hal_rcc_ex.c
C_SRCS += $(HAL_PATH)/stm32f7xx_hal_rcc.c
C_SRCS += $(HAL_PATH)/stm32f7xx_hal_tim_ex.c
C_SRCS += $(HAL_PATH)/stm32f7xx_hal_tim.c
C_SRCS += $(HAL_PATH)/stm32f7xx_hal_dma_ex.c
C_SRCS += $(HAL_PATH)/stm32f7xx_hal_dma.c
C_SRCS += $(HAL_PATH)/stm32f7xx_hal_sai_ex.c
C_SRCS += $(HAL_PATH)/stm32f7xx_hal_sai.c
C_SRCS += $(HAL_PATH)/stm32f7xx_hal_flash_ex.c
C_SRCS += $(HAL_PATH)/stm32f7xx_hal_flash.c
C_SRCS += $(HAL_PATH)/stm32f7xx_hal.c
C_SRCS += $(STCUBE_PATH)/CMSIS/Device/ST/STM32F7xx/Source/Templates/system_stm32f7xx.c
C_SRCS += $(STM32_EEPROM_EMU_PATH)/Src/eeprom.c
ifeq ($(BULLSEYE), 1)
    C_SRCS += $(REPO_PATH)/third_party/bullseye/src/libcov-cirrusrtt.c
# Also include the Segger RTT code
    C_SRCS += $(REPO_PATH)/third_party/rtt/gcc_io.c
    C_SRCS += $(REPO_PATH)/third_party/rtt/gcc_io_rtt.c
    C_SRCS += $(REPO_PATH)/third_party/rtt/SEGGER_RTT.c
    C_SRCS += $(REPO_PATH)/third_party/rtt/SEGGER_RTT_printf.c    
endif    
ASM_SRCS += $(STCUBE_PATH)/CMSIS/Device/ST/STM32F7xx/Source/Templates/gcc/startup_stm32f767xx.s
INCLUDES += -I$(APP_PATH)
INCLUDES += -I$(REPO_PATH)/Algorithms/third_party/arm/CMSIS_5-5.4.0/CMSIS/Core/Include
INCLUDES += -I$(STCUBE_PATH)/CMSIS/Device/ST/STM32F7xx/Include
INCLUDES += -I$(STCUBE_PATH)/STM32F7xx_HAL_Driver/Inc
INCLUDES += -I$(STM32_EEPROM_EMU_PATH)/Inc
ifeq ($(BULLSEYE), 1)
    INCLUDES += -I$(REPO_PATH)/third_party/bullseye/inc
# Also include the Segger RTT code
    INCLUDES += -I$(REPO_PATH)/third_party/rtt    
endif

# Defines after all target-specific variables set
OBJS = $(subst $(REPO_PATH),$(BUILD_DIR),$(C_SRCS:.c=.o))
ASM_OBJS = $(subst $(REPO_PATH),$(BUILD_DIR),$(ASM_SRCS:.s=.o))
BUILD_PATHS = $(sort $(dir $(ALGO_LIB_OBJS)))
BUILD_PATHS += $(sort $(dir $(OBJS)))
BUILD_PATHS += $(sort $(dir $(ASM_OBJS)))

# Print info, only if not cleaning
# don't delete this, it's ugly, but it needs 2 blank lines
define newline


endef

ifdef IS_VALID_BUILD
$(info )
$(info ===============================================================================)
$(info PATH:)
$(info $(PATH))
$(info -------------------------------------------------------------------------------)
$(info ROOT_PATH:       $(ROOT_PATH))
$(info REPO_PATH:       $(REPO_PATH))
$(info ALGO_LIBRARY_ALGORITHM_PATH:     $(ALGO_LIBRARY_ALGORITHM_PATH))
$(info ALGO_LIBRARY_MATH_PATH:     $(ALGO_LIBRARY_MATH_PATH))
$(info ALGO_LIBRARY_API_PATH:     $(ALGO_LIBRARY_API_PATH))
$(info APP_PATH:        $(APP_PATH))
$(info BUILD_DIR:       $(BUILD_DIR))
$(info BUILD_PATHS:     $(addprefix $(newline),$(BUILD_PATHS)))
$(info -------------------------------------------------------------------------------)
$(info INCLUDES: $(addprefix $(newline),$(INCLUDES)))
$(info -------------------------------------------------------------------------------)
$(info C_SRCS: $(addprefix $(newline),$(C_SRCS)))
$(info -------------------------------------------------------------------------------)
$(info ASM_SRCS: $(addprefix $(newline),$(ASM_SRCS)))
$(info -------------------------------------------------------------------------------)
$(info OBJS: $(addprefix $(newline),$(OBJS)))
$(info -------------------------------------------------------------------------------)
$(info ASM_OBJS: $(addprefix $(newline),$(ASM_OBJS)))
$(info -------------------------------------------------------------------------------)
$(info DEFINES: $(addprefix $(newline),$(DEFINES)))
$(info -------------------------------------------------------------------------------)
$(info CFLAGS: $(addprefix $(newline),$(CFLAGS)))
$(info -------------------------------------------------------------------------------)
$(info ASM_FLAGS: $(addprefix $(newline),$(ASM_FLAGS)))
$(info -------------------------------------------------------------------------------)
$(info LDFLAGS: $(addprefix $(newline),$(LDFLAGS)))
$(info -------------------------------------------------------------------------------)
$(info BULLSEYE: $(BULLSEYE))
$(info ===============================================================================)
$(info )
endif

# Now for all the targets

.PHONY: default all build_path clean
default: print_targets
#all: build_path $(BUILD_PATHS)

# Create the 'build' directory
build_path:
	@echo -------------------------------------------------------------------------------
	@echo CREATING build folders.
	@mkdir -p $(BUILD_DIR)

# Create directories for the build detritus
define mkdir_rule
.PHONY: $(1)
$(1):
	@mkdir -p $(1)
endef
$(foreach path,$(BUILD_PATHS),$(eval $(call mkdir_rule,$(path))))

# Create a target for each .o files, depending on its corresponding .c file
define obj_rule
$(1): $(2) $(MAKEFILE_PATH)/makefile
	@echo -------------------------------------------------------------------------------
	@echo COMPILING $(2) #to $(1)
	$(CC) $(CFLAGS) $(INCLUDES) -MD -MP -MT $(1) -MF $(subst .o,.d,$(1)) -c $(2) -o $(1)
endef
$(foreach obj,$(OBJS),$(eval $(call obj_rule,$(obj),$(subst $(BUILD_DIR),$(REPO_PATH),$(obj:.o=.c)))))

# Create a target for each .o file, depending on its corresponding .s file
define asm_obj_rule
$(1): $(2) $(MAKEFILE_PATH)/makefile
	@echo -------------------------------------------------------------------------------
	@echo ASSEMBLING $(2) #to $(1)
	$(AS) $(ASM_FLAGS) $(INCLUDES) -MD -MP -MT $(1) -MF $(subst .o,.d,$(1)) -c $(2) -o $(1)
endef
$(foreach obj,$(ASM_OBJS),$(eval $(call asm_obj_rule,$(obj),$(subst $(BUILD_DIR),$(REPO_PATH),$(obj:.o=.s)))))

# Enable bullseye build
bullseye_on:
	@echo -------------------------------------------------------------------------------
	@echo BULLSEYE ON
	cov01.exe --on
	covselect --deleteAll
	covselect --add $(ALGO_LIBRARY_ALGORITHM_PATH)/
	covselect --add $(ALGO_LIBRARY_MATH_PATH)/
	covselect --add $(ALGO_LIBRARY_API_PATH)/
	covselect --add !$(REPO_PATH)/Algorithms/COMMON/
	covselect --add !$(REPO_PATH)/Algorithms/HPA/Harness/
	covselect --add !$(REPO_PATH)/Algorithms/third_party/
	covselect --add !$(REPO_PATH)/build/

# Disable bullseye build
bullseye_off:
	@echo -------------------------------------------------------------------------------
	@echo BULLSEYE OFF
	cov01.exe --off

# Create the driver static library and print the resource usage
hpa_lib_cm7:
	$(MAKE) -C ../../ hpa_lib_cm7

baremetal: build_path $(BUILD_PATHS) $(BULLSEYE_ON_CMD) hpa_lib_cm7 $(OBJS) $(ASM_OBJS) $(BULLSEYE_OFF_CMD)
	@echo -------------------------------------------------------------------------------
	@echo LINKING $@
	$(CC) $(LDFLAGS) $(OBJS) $(ASM_OBJS) $(LIBS) -o $(BUILD_DIR)/baremetal.elf
	@echo -------------------------------------------------------------------------------
	@echo SIZE of baremetal.elf
	$(SIZE) -t $(BUILD_DIR)/baremetal.elf
	@echo -------------------------------------------------------------------------------
	$(OBJCOPY) -O ihex $(BUILD_DIR)/baremetal.elf $(BUILD_DIR)/baremetal.hex

print_targets:
	@echo ERROR:  No valid target specified.
	@echo
	@echo Valid targets:
	@echo       hpa_lib_cm7
	@echo       baremetal
#	@echo
#	@echo Valid command line variables:
#	@echo		BULLSEYE=0	\(will disable Bullseye coverage\)
#	@echo		BULLSEYE=1	\(will enable Bullseye coverage\)

clean:
	$(RM) $(BUILD_DIR)
