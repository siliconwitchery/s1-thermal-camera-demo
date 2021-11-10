# S1 I2C Verilog Demo
# -------------------------
#
# Copyright 2021 Silicon Witchery AB
#
# Permission to use, copy, modify, and/or distribute this 
# software for any purpose with or without fee is hereby
# granted, provided that the above copyright notice and this
# permission notice appear in all copies.
#
# THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS
# ALL WARRANTIES WITH REGARD TO THIS SOFTWARE INCLUDING ALL 
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS. IN NO 
# EVENT SHALL THE AUTHOR BE LIABLE FOR ANY SPECIAL, DIRECT, 
# INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES WHATSOEVER 
# RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN 
# ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, 
# ARISING OUT OF OR IN CONNECTION WITH THE USE OR PERFORMANCE 
# OF THIS SOFTWARE.
#
# -------------------------


PROJECT_NAME = i2c_verilog_demo
SIM_DIRECTORY = .sim

# Put the nRF SDK path here. If you don't have it, download it here:
# https://www.nordicsemi.com/Products/Development-software/nRF5-SDK
NRF_SDK_PATH ?= 

# Put your arm GCC path here. If you don't have it, download it here:
# https://developer.arm.com/tools-and-software/open-source-software/developer-tools/gnu-toolchain/gnu-rm/downloads
GNU_INSTALL_ROOT ?= /Applications/ARM/bin/
GNU_PREFIX ?= arm-none-eabi

# Source files
SRC_FILES += \
  main.c \

# Include paths
INC_FOLDERS += \
  . \

# Add compile time options
OPT += -flto 	# Link time optimization
OPT += -g3 		# Debugging information (takes way more flash)

# This is where the magic happens.
include s1-sdk/s1.mk

# Build task for the verilog project
build-verilog:
	@echo Creating the build directory.
	@mkdir -p $(OUTPUT_DIRECTORY)

	@echo "Synthesizing all the verilog files using yosys."
	@yosys -p "synth_ice40 -json $(OUTPUT_DIRECTORY)/hardware.json" -f verilog ice_40_top.v

	@echo "Placement and routing for the iCE40 uwg30 chip package."
	@nextpnr-ice40 --up5k --package uwg30 --json $(OUTPUT_DIRECTORY)/hardware.json --asc $(OUTPUT_DIRECTORY)/hardware.asc --pcf s1.pcf -q
	
	@echo "Generating the FPGA binary file."
	@icepack $(OUTPUT_DIRECTORY)/hardware.asc $(OUTPUT_DIRECTORY)/fpga_binfile.bin

	@echo "Converting the bin file into a .h file for the nRF build process."
	@cd $(OUTPUT_DIRECTORY) && xxd -i fpga_binfile.bin fpga_binfile_ram.h

	@echo "Appending the .h file to store binary in flash instead of ram."
	@sed '1s/^/const /' $(OUTPUT_DIRECTORY)/fpga_binfile_ram.h > fpga_binfile.h


# Build task to simulate the verilog using a test bench
sim-verilog:
	@echo Creating the sim directory.
	@mkdir -p $(SIM_DIRECTORY)

	@echo "Building sim executable with iVerilog."
	@iverilog -o .sim/i2c_controller_tb.out i2c_controller_tb.v

	@echo "Running vvp."
	@vvp .sim/i2c_controller_tb.out -lxt2
	
	@echo "Opening results."
	@gtkwave .sim/test_i2c_controller.lxt test_i2c_controller.gtkw