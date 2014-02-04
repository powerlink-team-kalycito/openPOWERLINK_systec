#  (c) Bernecker + Rainer Industrie-Elektronik Ges.m.b.H.
#      A-5142 Eggelsberg, B&R Strasse 1
#      www.br-automation.com
#  (c) Kalycito Infotech Private Ltd.
#
# Project       : POWERLINK Xilinx Examples
# Module        : build system
# Autor         : gaurav
# Date          : 18.11.2013
# File          : Makefile
# contents      : Controls the build of the Xilinx examples
################################################################################

VERSION = 0.4

include makefile.settings

APP_DIR_ARCH=$(STACKROOT_DIR)/drivers/xilinx/microblaze/mn_pcp

OBJDICT_DIR=$(STACKROOT_DIR)/objdicts/cia401_mn

LIBS_DIR=$(STACKROOT_DIR)/contrib
KERNEL_DIR=$(STACKROOT_DIR)/stack/src/kernel
USER_DIR=$(STACKROOT_DIR)/stack/src/user
COMMON_DIR=$(STACKROOT_DIR)/stack/src/common
TARGET_DIR=$(STACKROOT_DIR)/stack/src/arch/xilinx_microblaze

IP_CORE_REPO=$(STACKROOT_DIR)/hardware/ipcore/xilinx
IP_CORE_REPOR = J:/Xilinx-MB-Project/Ref_Src/Hw/MyCodes/xilinx_mn/ipcore/release/xilinx/library/
IP_CORE_DRV=$(STACKROOT_DIR)/hardware/ipcore/drivers

HW_SPEC=$(XPS_DIR)/SDK/SDK_Export/hw
BSP_PATH=$(APP_DIR_ARCH)

OBJDIR=obj

##################################
#translate dual_processor variable
ifeq ($(DUAL_PROCESSOR),yes)
	DESIGN=dual
else
	DESIGN=single
endif

##################################
#general compile defines
CC=mb-gcc
AR=mb-ar
CP=cp
RM=rm -rf
PERL=xilperl
EXECUTABLE=mn_pcp

LSSCRIPT=lscript_$(BOARD_NAME)_$(BUS_INTERFACE)_$(DESIGN).ld
MSS_FILE=system_$(BOARD_NAME)_$(BUS_INTERFACE)_$(DESIGN).mss
MACROS=-D$(DBG_MODE) -DDEF_DEBUG_LVL=$(DEF_DEBUG_LVL)
CFLAGS=$(DEBUG_FLAG) -Wall -fmessage-length=0 $(PROC_FEATURES) $(MACROS) -DXIL_PLK_MN
CPPFLAGS=-MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)"
LDFLAGS=-Wl,-T -Wl,$(LSSCRIPT) $(PROC_FEATURES) -Wl,-Map,$(EXECUTABLE).map



##################################
# Include directory
INCLUDES=-I${APP_DIR} \
${STACKROOT_DIR}/include \
-I$(IP_CORE_DRV)/openmac \
-I$(IP_CORE_DRV)/hostinterface \
-I$(STACKROOT_DIR)/stack/src/common/circbuf \
-I${STACKROOT_DIR}/stack/src \
-I${STACKROOT_DIR}/stack/include \
-I${STACKROOT_DIR}/stack/include/target \
-I${STACKROOT_DIR}/stack/src/arch/xilinx_microblaze \
-I${PROCESSOR_NAME}/include \
-I$(APP_DIR_ARCH) \

##################################
# Source files
SRCFILES=daemon.c \
${STACKROOT_DIR}/hardware/ipcore/drivers/hostinterface/hostiflib.c \
${STACKROOT_DIR}/hardware/ipcore/drivers/hostinterface/hostiflib_l.c \
${STACKROOT_DIR}/hardware/ipcore/drivers/hostinterface/lfqueue.c \
${STACKROOT_DIR}/hardware/ipcore/drivers/hostinterface/linmem.c \
${STACKROOT_DIR}/hardware/ipcore/drivers/hostinterface/hostiflib_microblaze.c \
${STACKROOT_DIR}/hardware/ipcore/drivers/openmac/omethlib.c \
${STACKROOT_DIR}/hardware/ipcore/drivers/openmac/omethlibint.c \
${STACKROOT_DIR}/hardware/ipcore/drivers/openmac/omethlib_phycfg_generic.c \
${STACKROOT_DIR}/stack/src/common/circbuf/circbuffer.c \
${STACKROOT_DIR}/stack/src/common/circbuf/circbuf-noos.c \
${STACKROOT_DIR}/contrib/trace/trace-printf.c \
${STACKROOT_DIR}/stack/src/arch/xilinx_microblaze/target-microblaze.c \
${STACKROOT_DIR}/stack/src/arch/xilinx_microblaze/systemtimer.c \
${STACKROOT_DIR}/stack/src/arch/xilinx_microblaze/usleep.c \
${STACKROOT_DIR}/stack/src/arch/xilinx_microblaze/openmac-microblaze.c \
${STACKROOT_DIR}/stack/src/arch/xilinx_microblaze/lock-dualprocnoos.c \
${STACKROOT_DIR}/stack/src/common/dll/dllcal-hostif.c \
${STACKROOT_DIR}/stack/src/common/event/event.c \
${STACKROOT_DIR}/stack/src/common/errstring.c \
${STACKROOT_DIR}/stack/src/kernel/ctrl/ctrlk.c \
${STACKROOT_DIR}/stack/src/kernel/ctrl/ctrlkcal-hostif.c \
${STACKROOT_DIR}/stack/src/kernel/dll/dllk.c \
${STACKROOT_DIR}/stack/src/kernel/dll/dllkfilter.c \
${STACKROOT_DIR}/stack/src/kernel/dll/dllkstatemachine.c \
${STACKROOT_DIR}/stack/src/kernel/dll/dllkframe.c \
${STACKROOT_DIR}/stack/src/kernel/dll/dllkevent.c \
${STACKROOT_DIR}/stack/src/kernel/dll/dllkcal.c \
${STACKROOT_DIR}/stack/src/kernel/edrv/edrvcyclic-openmac.c \
${STACKROOT_DIR}/stack/src/kernel/edrv/edrv-openmac.c \
${STACKROOT_DIR}/stack/src/kernel/errhnd/errhndk.c \
${STACKROOT_DIR}/stack/src/kernel/errhnd/errhndkcal-hostif.c \
${STACKROOT_DIR}/stack/src/kernel/event/eventk.c \
${STACKROOT_DIR}/stack/src/kernel/event/eventkcal-nooshostif.c \
${STACKROOT_DIR}/stack/src/kernel/event/eventkcalintf-hostif.c \
${STACKROOT_DIR}/stack/src/kernel/hrtimer/hrtimer-openmac.c \
${STACKROOT_DIR}/stack/src/kernel/nmt/nmtk.c \
${STACKROOT_DIR}/stack/src/kernel/pdo/pdok.c \
${STACKROOT_DIR}/stack/src/kernel/pdo/pdokcal.c \
${STACKROOT_DIR}/stack/src/kernel/pdo/pdokcal-triplebufshm.c \
${STACKROOT_DIR}/stack/src/kernel/pdo/pdokcalmem-hostif.c \
${STACKROOT_DIR}/stack/src/kernel/pdo/pdokcalsync-hostif.c \
${STACKROOT_DIR}/stack/src/kernel/timestamp/timestamp-openmac.c 

#####################################
# Dependencies
DEPS = ${APP_DIR}/EplCfg.h \
${APP_DIR}/makefile.settings \
${APP_DIR}/systemComponents.h \
${STACKROOT_DIR}/include \
${STACKROOT_DIR}/stack/include/target/openmac \
${STACKROOT_DIR}/stack/src \
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
ifeq ($(BUS_INTERFACE),axi)
	PROC_FEATURES+= -mlittle-endian
	SRCFILES+= $(COMMON_DIR)/ami/amile.c
else
	PROC_FEATURES+= -mbig-endian
	SRCFILES+= $(COMMON_DIR)/ami/amibe.c
endif

###################################
# libraries
LIBSPATH=$(BSP_PATH)/$(PROCESSOR_NAME)/lib
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
	@echo " Copyright (c) 2013 Kalycito"
	@echo " Version $(VERSION)"
	@echo "================================================================================"
	@echo ""
	@echo "Write 'make all' to generate the bitstream and mn_pcp example"
	@echo ""
	@echo "Write 'make bsp' to generate the board support package"
	@echo "Write 'make app' to generate the mn_pcp example"
	@echo "Write 'make bitstream' to generate the bitstream"
	@echo ""
	@echo "Write 'make download-all' to download the bitstream and the .elf file to the target"
	@echo "Write 'make download-bits' to download the bitstream to the target"
	@echo "Write 'make download-elf' to download the .elf file to the target"
	@echo ""
	@echo "Write 'make clean-bsp' to clean the board support package"
	@echo "Write 'make clean-app' to clean the application"
	@echo "Write 'make clean-sw' to clean the whole software"
	@echo ""
	@echo "Write 'make clean-hw' to clean the bitstream"
	@echo "Write 'make clean-all' to clean the all generated files"
	@echo ""
	@echo "Change 'makefile.setting' to configure the build system"

.PHONY: all
#all: bitstream app
all: app

####################################################
# A P P L I C A T I O N
####################################################
.PHONY: app
app: builddir bsp $(EXECUTABLE).elf $(EXECUTABLE).size $(EXECUTABLE).elfcheck


bsp: $(LIBSPATH)/libxil.a

$(LIBSPATH)/libxil.a: $(HW_SPEC)/system.xml $(MSS_FILE)
	@echo Invoking: libgen
	libgen -hw $(HW_SPEC)/system.xml -lp $(IP_CORE_REPO) -lp $(IP_CORE_REPOR) -pe $(PROCESSOR_NAME) -od $(BSP_PATH) -log libgen.log $(MSS_FILE)
	@echo Finished building: board support package
	@echo ' '

.PHONY: builddir
builddir:
	$(shell mkdir $(OBJDIR))

$(OBJDIR)/%.o: %.c $(BSP_PATH)/$(PROCESSOR_NAME)
	@echo Building file: $<
	@echo Invoking: MicroBlaze gcc compiler
	$(CC) -c ${OPT_LEVEL} $(CFLAGS) $(CPPFLAGS) $< $(OUTPUT_OPTION)
	@echo Finished building: $<
	@echo ' '

# extra rule for usleep to set opt level to zero
$(OBJDIR)/usleep.o: usleep.c usleep.h $(BSP_PATH)/$(PROCESSOR_NAME)
	@echo Building file: $<
	@echo Invoking: MicroBlaze gcc compiler
	$(CC) -c -O0 $(CFLAGS) $(CPPFLAGS) $< ${OUTPUT_OPTION}
	@echo Finished building: $<
	@echo ' '

$(EXECUTABLE).elf: $(OBJ) $(LSSCRIPT) $(LIBSPATH)/libxil.a
	@echo Building target: $@
	@echo Invoking: MicroBlaze gcc linker
	$(CC) $(LDFLAGS) -L$(LIBSPATH) $(OBJ) -o $(EXECUTABLE).elf
	@echo Finished building target: $@
	@echo ' '

$(EXECUTABLE).size: $(EXECUTABLE).elf
	@echo Invoking: MicroBlaze Print Size
	mb-size $(EXECUTABLE).elf  |tee "$(EXECUTABLE).size"
	@echo Finished building: $@
	@echo ' '

$(EXECUTABLE).elfcheck: $(EXECUTABLE).elf
	@echo Invoking: Xilinx ELF Check
	elfcheck $(EXECUTABLE).elf -hw $(HW_SPEC)/system.xml -pe $(PROCESSOR_NAME)  |tee "$(EXECUTABLE).elfcheck"
	@echo Finished building: $@
	@echo ' '

####################################################
# C L E A N
####################################################
.PHONY: clean
clean: clean-sw

.PHONY: clean-all
clean-all: clean-sw clean-hw

.PHONY: clean-bsp
clean-bsp:
	-@echo "Cleaning board support package..."
	-$(RM) $(PROCESSOR_NAME) libgen.log
	-@echo ' '

.PHONY: clean-app
clean-app:
	-@echo "Cleaning software parts..."
	@touch $(EXECUTABLE)
	-$(RM) $(EXECUTABLE).* $(OBJ) $(OBJD) $(OBJDIR)
	
	-@echo ' '

.PHONY: clean-sw
clean-sw: clean-bsp clean-app

.PHONY: clean-hw
clean-hw: $(XPS_DIR)/system.make
	-@echo "Cleaning bitstream parts..."
	$(MAKE) -C $(XPS_DIR) -f system.make clean
	-$(RM) $(XPS_DIR)/system.make $(HW_SPEC)
	-@echo ' '


####################################################
# H A R D W A R E
####################################################

$(XPS_DIR)/system.make :
	$(MAKE) -C $(XPS_DIR) all

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
download-elf: app
	xmd -hw $(HW_SPEC)/system.xml -tcl download-elf.tcl $(EXECUTABLE) $(VERIFY_ELF)

-include $(OBJD)
	
-include $(OBJD)
