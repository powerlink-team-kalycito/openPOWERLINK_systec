#  (c) Bernecker + Rainer Industrie-Elektronik Ges.m.b.H.
#      A-5142 Eggelsberg, B&R Strasse 1
#      www.br-automation.com
#
# Project       : POWERLINK Xilinx Examples
# Module        : download .elf file
# Autor         : mairt
# Date          : 13.08.2012
# File          : tcl script
# contents      : connects to mb, download the elf file and starts execution
################################################################################

connect mb mdm -debugdevice cpunr 1
rst -processor
dow demo_mn_embedded.elf
run
disconnect 0
exit;
