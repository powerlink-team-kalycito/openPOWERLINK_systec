#  (c) Bernecker + Rainer Industrie-Elektronik Ges.m.b.H.
#      A-5142 Eggelsberg, B&R Strasse 1
#      www.br-automation.com
#  (c) Kalycito Infotech Private Ltd.
#
# Project       : POWERLINK Xilinx Examples
# Module        : build system
# Autor         : gaurav
# Date          : 13.05.2013
# File          : Makefile
# contents      : Controls the build of the Xilinx Zynq examples
################################################################################

# Initial version of Makefile , Compiles Project software files for PCP processor
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
EXECUTABLE=mn_pcp

LSSCRIPT=lscript_$(BOARD_NAME)_$(BUS_INTERFACE)_$(DESIGN).ld
MACROS+=-D${DBG_MODE} -DDEF_DEBUG_LVL=${DEF_DEBUG_LVL}
CFLAGS=$(DEBUG_FLAG) -Wall -fmessage-length=0 ${MACROS}
CPPFLAGS=-MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)"
LDFLAGS=-Wl,-T -Wl,${LSSCRIPT} -Wl,-Map,${EXECUTABLE}.map

ifeq ($(PROCESSOR_NAME),ps7_cortexa9_0)
	CFLAGS+= $(PROC_FEATURES_ARM)
	LDFLAGS+= $(PROC_FEATURES_ARM)
else
	CFLAGS+= $(PROC_FEATURES_MB)
	LDFLAGS+= $(PROC_FEATURES_MB)
endif

##################################
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
-I${STACK_ROOT}/libs/omethlib \
-I${STACK_ROOT}/stack/include/target/openmac \
-I${STACK_ROOT}/stack/src \
-I${STACK_ROOT}/stack/src/arch/xilinx_microblaze \
-I${BSP_PATH}/${PROCESSOR_NAME}/include 

##################################
# Source files
SRCFILES=${STACK_ROOT}/libs/omethlib/omethlib.c \
${STACK_ROOT}/libs/omethlib/omethlibint.c \
${STACK_ROOT}/libs/omethlib/omethlib_phycfg_generic.c \
${STACK_ROOT}/libs/trace/trace-printf.c \
${STACK_ROOT}/libs/dualprocshm/dualprocshm-zynq.c \
${STACK_ROOT}/libs/dualprocshm/dualprocshm-noos.c \
${STACK_ROOT}/libs/dualprocshm/dualprocshm-intr.c \
${STACK_ROOT}/libs/circbuf/circbuffer.c \
${STACK_ROOT}/libs/circbuf/circbuf-noosdual.c \
${STACK_ROOT}/stack/src/arch/xilinx_microblaze/xilinx_irq.c \
${STACK_ROOT}/stack/src/arch/xilinx_microblaze/xilinx_usleep.c \
${STACK_ROOT}/stack/src/arch/xilinx_microblaze/target-microblaze.c \
${STACK_ROOT}/stack/src/common/event/event.c \
${STACK_ROOT}/stack/src/common/errstring.c \
${STACK_ROOT}/stack/src/common/debug.c \
${STACK_ROOT}/stack/src/kernel/ctrl/ctrlk.c \
${STACK_ROOT}/stack/src/kernel/ctrl/ctrlkcal-noosdual.c \
${STACK_ROOT}/stack/src/kernel/dll/dllk.c \
${STACK_ROOT}/stack/src/kernel/dll/dllkfilter.c \
${STACK_ROOT}/stack/src/kernel/dll/dllkstatemachine.c \
${STACK_ROOT}/stack/src/kernel/dll/dllkframe.c \
${STACK_ROOT}/stack/src/kernel/dll/dllkevent.c \
${STACK_ROOT}/stack/src/kernel/dll/dllkcal.c \
${STACK_ROOT}/stack/src/kernel/dll/dllkcal-circbuf.c \
${STACK_ROOT}/stack/src/kernel/edrv/edrvcyclic-openmac.c \
${STACK_ROOT}/stack/src/kernel/edrv/edrv-openmac.c \
${STACK_ROOT}/stack/src/kernel/errhnd/errhndk.c \
${STACK_ROOT}/stack/src/kernel/errhnd/errhndkcal-noosdual.c \
${STACK_ROOT}/stack/src/kernel/event/eventk.c \
${STACK_ROOT}/stack/src/kernel/event/eventkcal-noosdual.c \
${STACK_ROOT}/stack/src/kernel/event/eventkcalintf-circbuf.c \
${STACK_ROOT}/stack/src/kernel/hrtimer/hrtimer-openmac.c \
${STACK_ROOT}/stack/src/kernel/nmt/nmtk.c \
${STACK_ROOT}/stack/src/kernel/pdo/pdok.c \
${STACK_ROOT}/stack/src/kernel/pdo/pdokcal.c \
${STACK_ROOT}/stack/src/kernel/pdo/pdokcal-triplebufshm.c \
${STACK_ROOT}/stack/src/kernel/pdo/pdokcalmem-noosdual.c \
${STACK_ROOT}/stack/src/kernel/pdo/pdokcalsync-noosdual.c \
${STACK_ROOT}/stack/src/kernel/timestamp/timestamp-openmac.c \
${APP_DIR}/daemon.c \
${APP_DIR}/systemComponents.c 

#####################################
# Dependencies
DEPS = ${APP_DIR}/EplCfg.h \
${APP_DIR}/makefile.settings \
${APP_DIR}/systemComponents.h \
${STACK_ROOT}/include \
${STACK_ROOT}/libs/sharedbuff \
${STACK_ROOT}/stack/include/target/openmac \
${STACK_ROOT}/stack/src \
${STACK_ROOT}/objdicts/CiA302-4_MN \
$(BSP_PATH)/${PROCESSOR_NAME}

LIBRARIES = libxil.a

VPATH=$(sort $(patsubst " ", :, $(dir $(SRCFILES))))


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
###################################
# objects
OBJ = $(addprefix $(OBJDIR)/,$(notdir $(patsubst %.c, %.o, $(SRCFILES))))
OBJD = $(addprefix $(OBJDIR)/,$(notdir $(patsubst %.c, %.d, $(SRCFILES))))

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
	@echo "Write 'make all' to generate the mn_pcp example"
	@echo ""
	@echo "Write 'make application' to generate the mn_pcp example"
	@echo "Write 'make bitstream' to generate the bitstream"
	@echo ""
	@echo "Write 'make build-sd' to create BOOT.BIN file "
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

$(OBJDIR)/%.o: %.c $(DEPS)
	@echo Building file: $<
	@echo Invoking: MicroBlaze gcc compiler
	$(CC) -c ${OPT_LEVEL} $(CFLAGS) $(CPPFLAGS) $< $(OUTPUT_OPTION)
	@echo Finished building: $<
	@echo ' '

# extra rule for usleep to set opt level to zero
$(OBJDIR)/xilinx_usleep.o: xilinx_usleep.c xilinx_usleep.h $(BSP_PATH)/$(PROCESSOR_NAME)
	@echo Building file: $<
	@echo Invoking: MicroBlaze gcc compiler
	$(CC) -c -O0 $(CFLAGS) $(CPPFLAGS) $< ${OUTPUT_OPTION}
	@echo Finished building: $<
	@echo ' '

${EXECUTABLE}.elf: $(BSP_PATH)/$(PROCESSOR_NAME) ${OBJ} ${LSSCRIPT}
	@echo Building target: $@
	@echo Invoking: MicroBlaze gcc linker
	$(CC) $(LDFLAGS) $(LIBSPATH) ${OBJ} $(LIBS) -o ${EXECUTABLE}.elf
	@echo Finished building target: $@
	@echo ' '

${EXECUTABLE}.size: ${EXECUTABLE}.elf
	@echo Invoking: MicroBlaze Print Size
	$(SIZE) ${EXECUTABLE}.elf  |tee "${EXECUTABLE}.size"
	@echo Finished building: $@
	@echo ' '

${EXECUTABLE}.elfcheck: ${EXECUTABLE}.elf
	@echo Invoking: Xilinx ELF Check
	elfcheck ${EXECUTABLE}.elf -hw ${HW_SPEC}/system.xml -pe ${PROCESSOR_NAME}  |tee "${EXECUTABLE}.elfcheck"
	@echo Finished building: $@
	@echo ' '

${EXECUTABLE}.srec: ${EXECUTABLE}.elf
	@echo Invoking: Microblaze objcopy
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
	
	
####################################################
# SD_IMAGE
####################################################
.PHONY: transfer
transfer: 
	-@echo "Transfering"
	$(MAKE) -C $(SDCARD_DIR) transfer
	-@echo "BOOT.bin Transferred Succesfully"
	
.PHONY: build-sd
build-sd: 
	-@echo "Creating Sd Image..."
	$(MAKE) -C $(SDCARD_DIR) build
	-@echo "BOOT.bin Created Succesfully"
	
#$(MAKE) -C $(SDCARD_DIR) transfer
-include $(OBJD)
