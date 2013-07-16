#  (c) Bernecker + Rainer Industrie-Elektronik Ges.m.b.H.
#      A-5142 Eggelsberg, B&R Strasse 1
#      www.br-automation.com
#
# Project       : POWERLINK Xilinx Examples
# Module        : Makefile generation
# Autor         : mairt
# Date          : 13.02.2012
# File          : Makefile
# contents      : Creates the Makefile in the design folder of the example.
################################################################################


DRV_NAME = G

.PHONY: build
build:
	@echo "Build Sd Card Image"
	createBoot_zynq_mn.bat
	@echo Finished building: SdCard
	@echo ' '
.PHONY: transfer
transfer:
	cp BOOT.BIN ${DRV_NAME}:
	windrive -e ${DRV_NAME}