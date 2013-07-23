Xilinx Zynq Managing Node {#zynq-mn}
============================

## Introduction

The Xilinx Zynq Managing Node uses the POWERLINK IP-Core,
which consists of an optimized MAC for POWERLINK (openMAC) and
a hub for daisy chaining several nodes.
Additionally, the MN demos apply two separate processors named:
- POWERLINK Communication Processor (Pcp)
- Host Processor

Host interface is used for synchronizing and data sharing between Host
and Powerlink Communication Processor (PCP). Host Interface have following features
    - Data sharing between Host & PCP
    - Address translation :-Access to Dynamic memory allocation at PCP memory location by AP
    - Interrupts
    - Control & Status Registers to synchronization
    - Interface for parallel interface for host if needed
        - Less no of address lines (16bit) needed for accessing host memory address space
        
In case of Zynq designs (in general where both Host & PCP is running from same memory or host has
direct access to PCP's full memory space) we can avoid special hardware for host interface by
providing a shared memory location in external memory where both Host & PCP can access.

## Contents

- FPGA design for dual processor (Host - PCP)
- SDK repository for First stage bootloader for AMP systems.
- scripts for BOOT.bin genreration.
- openCONFIGURATOR project for 10 CNs

## Requirements

- Zynq 702 Eval board KIT
- Avnet FMC Ethernet Connector [Connect this to FMC 1 of Board] 
- Xilinx ISE Design Suite Embeded Edition 14.4.

**NOTE**: Steps for Creating First stage Boot loader for AMP processors may not be \
          valid in case of higher ISE versions.

## Hardware Setup

1. Zc7020 Setup
    - SW16 state 1,2,5 - low & 3,4 - High for booting from SDCARD

2. ISMNET FMC Connector Setup

    - Install jumper on JP1 pins 2-3
    - Install jumper on JP2 pins 1-2
    - Install jumper on JP5 & JP10 pins 7-8
    - Install jumper on JP4 & JP9 

## Creating Hardware Platform & BSP for the design

1. Create directory `Workspace` in stack root directory `{~/openPOWERLINK-V2.0.0}`
2. Open Xilinx SDK and enter `<rootDir>/Workspace`
3. Set repositories
      - select *Xilinx tools* -> *repositories*
      - select *New* for Local repositories
      - browse to and select `<rootDir>/tools/xilinx/zynq_fsbl_repo`
      - browse to and select `<rootDir>/fpga/ipcore/xilinx`
      - Click *Ok*
4. Creating Hardware Platform & BSP for the design
  - Open Workspace.
  - Select *File* -> *New* -> *Board support package*.
  - Select system.xml file from SDK/SDK_Export `{<rootDir>/fpga/boards/xilinx/xilinx_z702/Zynq_ap_pcp_intaxi-axi/SDK/SDK_Export/hw}`
    when asked to specify hardware platform file.\n
    (*Please make sure that there wont be any hardware platform open on the worksapce*).
  - Modify the hardware platform name as *hw_platform_zynq_intaxi-axi* for dual processor design.
  - Create two board support packages for dual Processor design. \n
    Use name *standalone_bsp_zynq_intaxi-pcp-axi* for *PCP* processor \n
    and *standalone_bsp_zynq_intaxi-host-axi* for ps7_cortexa9_0 processor.

## Creating First stage Boot loader for AMP
  
1. Select *File* -> *new* -> *Xilinx* -> *Application project*.
2. Give the name as *zynq_fsbl_dualProcessor* for Dual processor design.
3. Change processor to 'ps7_cortexa9_0' and use exiting Board support package *standalone_bsp_zynq_intaxi-host-axi*.
4. Click *Next* and select existing template 'Zynq FSBL for AMP'. Verify the description starts with 'AMP Modified'.
5. Click *Finish* to add *zynq_fsbl_dualProcessor* project.

## How to import the demo projects into Xilinx Software Development Kit

1. Select menu *File -> Import...*
2. Select the import source *General* -> *Existing Projects into Workspace*
3. Browse to `<rootDir>/examples/arch/zynq_cortexa9_0/no_os/gnu/demo_mn` (via the button *Browse...*)
4. Press the button *Finish*.
5. Browse to `<rootDir>/stack/make/driver/xilinx/zynq/mn_pcp` (via the button *Browse...*)
6. Press the button *Finish*.

**NOTE**: Use makefile.settings for respective projects present in mn_pcp or demo_mn for dual \n
          processor design to modify settings such as debug level, BSP_PATH etc.

## Generating bitstream and BOOT.bin

User can use `make targets` (*Window -> Show View -> Make Target*) available for mn_pcp projects \n
to generate bitstream and BOOT.bin files.

- bitstream : It will invoke the xps command line utility to generate fpga configuration file `system.bit` used to \n
              create `BOOT.bin`
- build-sd  : Generates the `BOOT.bin` file that contains the bit file, FSBL, demo-mn elf (Host application binary),\n
              and mn_pcp elf (PCP binary).

## How to run the demo

1. Generate the fpga configuration file `system.bit` using the make target **bitstream**.
2. Compile the projects by Selecting *Projects* -> *Build all* which will produce the elf files for the projects.\n
   (*demo_mn.elf*, *mn_pcp.elf* & *zynq_fsbl_dualProcessor.elf*)
3. Build the `BOOT.bin` file using the **build-sd** make target.
4. Copy the generated `BOOT.bin` file to the SD card.
5. Plug the SD card into the zc702 then power up the board.

The demo starts running once the system gets booted.

## Troubleshooting and Debugging on Zynq

Normally, SDK will automatically start XMD in the background when starting to debug an application. XMD \n
provides a command shell and GDB server which is connected to the CPU via the JTAG cable. For this \n
example design, XMD will be manually started in order to connect to both CPU0 and the Microblaze. Then, \n
SDK will be instructed to connect to both external GDB servers.
    
Since FSBL was used to boot the design, there is no need to re-initialize the PS registers and care must be \n
taken not to reset the full PS since both CPUs will be debugged simultaneously.
1. From SDK, start xmd and connect to both CPUs\n
    - In SDK, open a Xilinx command shell with *Xilinx_Tools -> Launch_shell*
    - In the new command shell enter **xmd**
    - At the XMD prompt, enter the command **connect arm hw** 
    - XMD should respond with the TCP port number 1234
    - Enter the command **connect mb mdm**
    - XMD should respond with the TCP port number 1235
   Now, two GDB servers are running and listening to TCP ports 1234 and 1235.
2. Start Debugging CPU0(Application Processor) in SDK for Dual Processor design
    - In SDK project explorer window, right click on demo_mn and select *debug_as -> debug_configurations*
    - Highlight *Xilinx C/C++ ELF* and select the *New launch configuration* icon at the top left
    - The name will be automatically set to *demo_mn Default*
    - Select *Device Initialization* tab and clear out the *Path to initialization TCL file* if it enabled.\n
      Initialization has already been done by Linux and FSBL
    - Change the reset type to No Reset\n
    - Select the *Remote Debug* tab\n
    - Instruct SDK to connect to the externally created GDB server by selecting *Connect to gdbserver on a different machine*.\n
      The IP Address should default to *localhost* and the port should be '1234'
    - *Apply*
    - *Debug*
    - The application will be downloaded then executed. The app will stop at a breakpoint at the first executable line\n
      in main(). There are times when the app may not automatically stop at the beginning of main so the pause button (suspend)\n
      may have to be pressed.
    - Press resume, single step, etc to continue running the app.
3. While debugging CPU0, Start Debugging the Microblaze (Powerlink Communication Processor or PCP) in SDK
    - If SDK has already started a debug session, the view will be *Debug*. In the upper right area of SDK, select the C/C+ View.
    - In SDK project explorer window, right click on mn_pcp and select *debug_as -> debug_configurations*
    - Highlight *Xilinx C/C++ ELF* and select the *New launch configuration* icon at the top left
    - The name will be automatically set to *mn_pcp Default* as you selected project
    - Select *Device Initialization* tab and clear out the *Path to initialization TCL file*.\n
    Initialization has already been done by Linux and FSBL\n
    - Change the reset type to No Reset
    - Select the *Remote Debug* tab
    - Instruct SDK to connect to the externally created GDB server by selecting *Connect to gdbserver on a different machine*.\n
      The IP Address should default to *localhost* and the port should be set to '1235' \n
    - *Apply*
    - *Debug*
    - The application will be downloaded then executed. The app will stop at a breakpoint at the first executable \n
      line in main(). There are times when the app may not automatically stop at the beginning of main so the pause\n
      button (*suspend*) may have to be pressed.
    - Press resume, single step, etc to continue running the app.

At any point, while in the *Debug view*, the focus can be switched between CPU0 and the Microblaze debug by selecting \n
the listed function, under *Thread* on the debug tab. As each function is selected, the visible source will change.