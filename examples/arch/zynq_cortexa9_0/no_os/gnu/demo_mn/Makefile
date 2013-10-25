#  (c) Bernecker + Rainer Industrie-Elektronik Ges.m.b.H.
#      A-5142 Eggelsberg, B&R Strasse 1
#      www.br-automation.com
#  (c) Kalycito Infotech Private Ltd.
#
# Project       : POWERLINK Xilinx Examples
# Module        : build system
# Autor         : chidrupaya
# Date          : 13.05.2013
# File          : Makefile
# contents      : Controls the build of the Xilinx Zynq examples
################################################################################

# Initial version of Makefile , Compiles Project software files for ps7_cortexa9_0 processor
# Yet to add hardware compilation
##
VERSION = 0.3

include makefile.settings

##################################
# Directory structure
APP_DIR=.
OBJDIR=obj

##################################
#translate dual_processor variable
ifeq ($(DUAL_PROCESSOR),yes)
    DESIGN=dual
else
    DESIGN=single
endif

##################################
# Specific for Zynq to enable prints
ifeq ($(BOARD_NAME),zynq)
	MACROS+= -D__ZYNQ__
endif

##################################
#general compile defines

ifeq ($(PROCESSOR_NAME),ps7_cortexa9_0)
	CC=arm-xilinx-eabi-gcc
	AR=arm-xilinx-eabi-ar
	SIZE=arm-xilinx-eabi-size
	COPY=arm-xilinx-eabi-objcopy
	PROC=ARM_xilinx
else
	CC=mb-gcc
	AR=mb-ar
	SIZE=mb-size
	COPY=mb-objcopy
	PROC=MicroBlaze
endif
CP=cp
RM=rm
PERL=xilperl

EXECUTABLE=demo_mn
LSSCRIPT=lscript_$(BOARD_NAME)_$(BUS_INTERFACE)_$(DESIGN).ld
MACROS+=-D${DBG_MODE} -DDEF_DEBUG_LVL=${DEF_DEBUG_LVL} 
CFLAGS=$(DEBUG_FLAG) -Wall -fmessage-length=0 ${MACROS} -D__ZYNQ__
CPPFLAGS=-MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)"
LDFLAGS=-Wl,-T -Wl,${LSSCRIPT} -Wl,-Map,${EXECUTABLE}.map

ifeq ($(PROCESSOR_NAME),ps7_cortexa9_0)
	CFLAGS+= $(PROC_FEATURES_ARM)
	LDFLAGS+= $(PROC_FEATURES_ARM)
else
	CFLAGS+= $(PROC_FEATURES_MB)
	LDFLAGS+= $(PROC_FEATURES_MB)
endif

#################################
# Flash defines
FLASH_IMAGE_NAME= flash_image
BOOTLOADER_NAME= mem_bootloader
BOOT_BUILD_DIR= build
OBJCPYFLAGS=-O srec
BITSTREAM= download


##################################
# Include directory
INCLUDES=-I${APP_DIR} \
-I${STACK_ROOT}/include \
-I${STACK_ROOT}/libs/dualprocshm \
-I${STACK_ROOT}/libs/circbuf \
-I${STACK_ROOT}/libs/sharedbuff \
-I${STACK_ROOT}/libs \
-I${STACK_ROOT}/stack/src \
-I${STACK_ROOT}/objdicts/CiA302-4_MN \
-I$(BSP_PATH)/${PROCESSOR_NAME}/include \
-I$(HW_SPEC)

##################################
# Dependencies 
DEPS = ${APP_DIR}/EplCfg.h \
${APP_DIR}/makefile.settings \
${APP_DIR}/systemComponents.h \
${STACK_ROOT}/include \
${STACK_ROOT}/libs/dualprocshm \
${STACK_ROOT}/libs/circbuf \
${STACK_ROOT}/stack/include/target/openmac \
${STACK_ROOT}/stack/src \
${STACK_ROOT}/objdicts/CiA302-4_MN \
$(BSP_PATH)/${PROCESSOR_NAME}

##################################
# Source files

SRCFILES=${STACK_ROOT}/examples/arch/zynq_cortexa9_0/no_os/gnu/demo_mn/systemComponents.c \
${STACK_ROOT}/examples/arch/zynq_cortexa9_0/no_os/gnu/demo_mn/demo_main.c \
${STACK_ROOT}/examples/demo_mn_console/event.c \
${STACK_ROOT}/libs/sd_fat16/ff.c \
${STACK_ROOT}/libs/sd_fat16/mmc.c \
${STACK_ROOT}/libs/sd_fat16/fs_sdcard.c \
${STACK_ROOT}/libs/console/printlog.c \
${STACK_ROOT}/libs/ami/amiarm.c \
${STACK_ROOT}/libs/dualprocshm/dualprocshm-zynq.c \
${STACK_ROOT}/libs/dualprocshm/dualprocshm-noos.c \
${STACK_ROOT}/libs/dualprocshm/dualprocshm-intr.c \
${STACK_ROOT}/libs/circbuf/circbuffer.c \
${STACK_ROOT}/libs/circbuf/circbuf-noosdual.c \
${STACK_ROOT}/libs/trace/trace-printf.c \
${STACK_ROOT}/stack/src/user/api/processimage-cia302.c \
${STACK_ROOT}/stack/src/arch/xilinx_arm/target-arm.c \
${STACK_ROOT}/stack/src/common/timer/timer-generic.c \
${STACK_ROOT}/stack/src/common/debug.c \
${STACK_ROOT}/stack/src/common/errstring.c \
${STACK_ROOT}/stack/src/common/event/event.c \
${STACK_ROOT}/stack/src/user/api/generic.c \
${STACK_ROOT}/stack/src/user/api/processimage.c \
${STACK_ROOT}/stack/src/user/ctrl/ctrlu.c \
${STACK_ROOT}/stack/src/user/ctrl/ctrlucal-noosdual.c \
${STACK_ROOT}/stack/src/user/dll/dllucal.c \
${STACK_ROOT}/stack/src/user/dll/dllucal-circbuf.c \
${STACK_ROOT}/stack/src/user/errhnd/errhndu.c \
${STACK_ROOT}/stack/src/user/errhnd/errhnducal-noosdual.c \
${STACK_ROOT}/stack/src/user/event/eventu.c \
${STACK_ROOT}/stack/src/user/event/eventucal-noosdual.c \
${STACK_ROOT}/stack/src/user/event/eventucalintf-circbuf.c \
${STACK_ROOT}/stack/src/user/nmt/identu.c \
${STACK_ROOT}/stack/src/user/nmt/nmtcnu.c \
${STACK_ROOT}/stack/src/user/nmt/nmtmnu.c \
${STACK_ROOT}/stack/src/user/nmt/nmtu.c \
${STACK_ROOT}/stack/src/user/nmt/statusu.c \
${STACK_ROOT}/stack/src/user/nmt/syncu.c \
${STACK_ROOT}/stack/src/user/obd/obd.c \
${STACK_ROOT}/stack/src/user/obd/obdcreate.c \
${STACK_ROOT}/stack/src/user/obd/obdcdc.c \
${STACK_ROOT}/stack/src/user/pdo/pdou.c \
${STACK_ROOT}/stack/src/user/pdo/pdoucal.c \
${STACK_ROOT}/stack/src/user/pdo/pdoucal-triplebufshm.c \
${STACK_ROOT}/stack/src/user/pdo/pdoucalmem-noosdual.c \
${STACK_ROOT}/stack/src/user/pdo/pdoucalsync-noosdual.c \
${STACK_ROOT}/stack/src/user/sdo/sdo-asndu.c \
${STACK_ROOT}/stack/src/user/sdo/sdo-asysequ.c \
${STACK_ROOT}/stack/src/user/sdo/sdo-comu.c \
${STACK_ROOT}/stack/src/user/cfmu.c \
${STACK_ROOT}/stack/src/user/ledu.c 

LIBRARIES = libxil.a

SRC_DIR = $(subst ${STACK_ROOT},,$(SRCFILES))
SRC_FILES = $(subst ./,,$(SRC_DIR))
VPATH=$(sort $(patsubst " ", :, $(dir $(SRC_FILES))))

###################################
# set Optimization level macro
ifeq ($(OPT_LEVEL),-O0)
	MACROS+= -DXIL_NO_OPT_LEVEL
endif
ifeq ($(OPT_LEVEL),-O1)
	MACROS+= -DXIL_OPT_LEVEL_1
endif
ifeq ($(OPT_LEVEL),-O2)
	MACROS+= -DXIL_OPT_LEVEL_2
endif
ifeq ($(OPT_LEVEL),-O3)
	MACROS+= -DXIL_OPT_LEVEL_3
endif
ifeq ($(OPT_LEVEL),-Os)
	MACROS+= -DXIL_OPT_LEVEL_SIZE
endif

###################################
# set endian according to bus interface
ifeq ($(PROCESSOR_NAME),pcp)
ifeq ($(BUS_INTERFACE),axi)
	CFLAGS+= -mlittle-endian
	LDFLAGS+= -mlittle-endian
	SRCFILES+= ${STACK_ROOT}/libs/ami/amiarm.c
else
	CFLAGS+= -mbig-endian
	LDFLAGS+= -mbig-endian
	SRCFILES+= ${STACK_ROOT}/libs/ami/amibe.c
endif
endif

###################################
# libraries
LIBSPATH=-L$(BSP_PATH)/${PROCESSOR_NAME}/lib 

# Note: ARM gcc reports undefined reference unless nested
LIBS=-Wl,--start-group,-lxil,-lc,--end-group

###################################
# objects
# objects


OBJ = $(addprefix $(OBJDIR)/,$(patsubst %.c, %.o, $(SRC_FILES)))
OBJD = $(addprefix $(OBJDIR)/,$(patsubst %.c, %.d, $(SRC_FILES)))

OUTPUT_OPTION = -o $@

	
CFLAGS += ${INCLUDES}

.PHONY:	header
header:
	@echo ""
	@echo "================================================================================"
	@echo " Host-PCP - Build System (Xilinx)"
	@echo "================================================================================"
	@echo ""
	@echo " Copyright (c) 2012 B&R"
	@echo " Version $(VERSION)"
	@echo "================================================================================"
	@echo ""
	@echo "Write 'make all' to generate the bitstream and Direct I/O software example"
	@echo ""
	@echo "Write 'make application' to generate the Direct I/O software example"
	@echo "Write 'make bitstream' to generate the bitstream"
	@echo ""
	@echo "Write 'make download-all' to download the bitstream and the .elf file to the target"
	@echo "Write 'make download-bits' to download the bitstream to the target"
	@echo "Write 'make download-elf' to download the .elf file to the target"
	@echo ""
	@echo "Write 'make clean_sw' to clean the application"
	@echo "Write 'make clean_hw' to clean the bitstream"
	@echo "Write 'make clean_all' to clean the all generated files"
	@echo ""
	@echo "Change 'makefile.setting' to configure the build system"

.PHONY: all
all: application

####################################################
# A P P L I C A T I O N
####################################################
.PHONY: application
application: builddir ${EXECUTABLE}.elf ${EXECUTABLE}.size ${EXECUTABLE}.elfcheck ${EXECUTABLE}.mem

.PHONY: builddir
builddir:
	$(shell mkdir $(OBJDIR))
	
$(OBJDIR)/%.o: ${STACK_ROOT}%.c $(DEPS)
	@echo Building file Stack: $<
	@$(shell mkdir $(subst /,\,$(@D)))
	$(CC) -c ${OPT_LEVEL} $(CFLAGS) $(CPPFLAGS) $< $(OUTPUT_OPTION)
	@echo Finished building: $<
	@echo ' '
# extra rule for usleep to set opt level to zero
$(OBJDIR)/xilinx_usleep.o: xilinx_usleep.c xilinx_usleep.h $(BSP_PATH)/$(PROCESSOR_NAME)
	@echo Building file: $<
	@echo Invoking: ARM gcc compiler
	$(CC) -c -O0 $(CFLAGS) $(CPPFLAGS) $< ${OUTPUT_OPTION}
	@echo Finished building: $<
	@echo ' '

${EXECUTABLE}.elf: $(BSP_PATH)/$(PROCESSOR_NAME) ${OBJ} ${LSSCRIPT}
	@echo Building target: $@
	@echo Invoking: ARM gcc linker
	$(CC) $(LDFLAGS) ${OPT_LEVEL} $(LIBSPATH) ${OBJ} ${LIBS} -o ${EXECUTABLE}.elf	
	@echo Finished building target: $@
	@echo ' '

${EXECUTABLE}.size: ${EXECUTABLE}.elf
	@echo Invoking: ARM Print Size
	$(SIZE) ${EXECUTABLE}.elf  |tee "${EXECUTABLE}.size"
	@echo Finished building: $@
	@echo ' '

${EXECUTABLE}.elfcheck: ${EXECUTABLE}.elf
	@echo Invoking: Xilinx ELF Check
	elfcheck ${EXECUTABLE}.elf -hw ${HW_SPEC}/system.xml -pe ${PROCESSOR_NAME}  |tee "${EXECUTABLE}.elfcheck"
	@echo Finished building: $@
	@echo ' '

${EXECUTABLE}.srec: ${EXECUTABLE}.elf
	@echo Invoking: ARM objcopy
	$(COPY) $(OBJCPYFLAGS) ${EXECUTABLE}.elf ${EXECUTABLE}.srec
	@echo Finished building: $@
	@echo ' '

${EXECUTABLE}.mem: ${EXECUTABLE}.elf
	@echo Invoking: Data2MEM
	Data2MEM -bd ${EXECUTABLE}.elf -d -o m ${EXECUTABLE}.mem
	@echo Finished building: $@
	@echo ' '

####################################################
# C L E A N
####################################################
.PHONY: clean
clean: clean_sw

.PHONY: clean_all
clean_all: clean_sw clean_hw

.PHONY: clean_sw
clean_sw:
	-@echo "Cleaning software parts..."
	 @touch $(EXECUTABLE)
	-${RM} -rf $(EXECUTABLE).* $(OBJ) $(OBJD)
	 ${RM} -rf $(OBJDIR)
	-@echo ' '

.PHONY: clean_hw
clean_hw:
	-@echo "Cleaning bitstream parts..."
	$(MAKE) -C $(XPS_DIR) -f system.make clean
	-${RM} -rf $(XPS_DIR)/system.make
	-@echo ' '


####################################################
# H A R D W A R E
####################################################

$(XPS_DIR)/system.make :
	$(MAKE) -C $(XPS_DIR) bitstream

$(XPS_DIR)/implementation/system.bit :
	$(MAKE) -C $(XPS_DIR) -f system.make bits

$(HW_SPEC)/system.xml :
	$(MAKE) -C $(XPS_DIR) -f system.make exporttosdk

.PHONY: bitstream
bitstream: $(XPS_DIR)/system.make $(XPS_DIR)/implementation/system.bit $(HW_SPEC)/system.xml


####################################################
# D O W N L O A D
####################################################
.PHONY: download-all
download-all: download-bits download-elf

.PHONY: download-bits
download-bits: $(XPS_DIR)/system.make
	$(MAKE) -C $(XPS_DIR) -f system.make download

.PHONY: download-elf
download-elf: application
	xmd -hw $(HW_SPEC)/system.xml -tcl download-elf.tcl

-include $(OBJD)
