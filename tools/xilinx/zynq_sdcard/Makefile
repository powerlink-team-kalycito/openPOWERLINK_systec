#  (c) Kalycito Infotech Private Ltd.
#      www.kalycito.com
#
# Project       : POWERLINK Zynq Example
# Module        : SD Card generation
# Autor         : vinod
# Date          : 23.09.2013
# File          : Makefile
# contents      : Generates the BOOT.bin for Zynq.
################################################################################


DRV_NAME = G

.PHONY: build
build:
	@echo "Build Sd Card Image"

#@rm BOOT.BIN 
	@cp ../../../fpga/boards/xilinx/xilinx_z702/Zynq_ap_pcp_intaxi-axi/implementation/system.bit .
	@cp ../../../Workspace/zynq_fsbl_dualProcessor/Debug/zynq_fsbl_dualProcessor.elf .
	@cp ../../../examples/arch/zynq_cortexa9_0/no_os/gnu/demo_mn/demo_mn.elf .
	@cp ../../../stack/make/driver/xilinx/zynq/mn_pcp/mn_pcp.elf .
	@arm-xilinx-eabi-objcopy -I elf32-little -O binary mn_pcp.elf mn_pcp.bin
	@bootgen -image bootimage_zynq_mn.bif -o i BOOT.BIN -w on 
	@rm *.elf *.bit mn_pcp.bin 
	@echo Finished building: SdCard
	@echo ' '
.PHONY: transfer
transfer:
	cp BOOT.BIN ${DRV_NAME}:
	