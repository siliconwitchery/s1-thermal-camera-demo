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


PROJECT_NAME = thermal_camera_demo
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
  c-code/main.c \

# Include paths
INC_FOLDERS += \
  c-code \

# Add compile time options
OPT += -flto 	# Link time optimization
OPT += -g3 		# Debugging information (takes way more flash)

# This is where the magic happens.
include s1-sdk/s1.mk

# Build task for the verilog project
build-verilog:
	@mkdir -p $(OUTPUT_DIRECTORY)
	@echo "\n---\nSynthesizing.\n"
	@yosys -p "synth_ice40 -json $(OUTPUT_DIRECTORY)/hardware.json" -q -Wall -f verilog verilog-code/top.v
	@echo "\n---\nPlace and route.\n"
	@nextpnr-ice40 --up5k --package uwg30 --json $(OUTPUT_DIRECTORY)/hardware.json --asc $(OUTPUT_DIRECTORY)/hardware.asc --pcf s1-sdk/s1.pcf -q
	@icepack $(OUTPUT_DIRECTORY)/hardware.asc $(OUTPUT_DIRECTORY)/fpga_binfile.bin
	@cd $(OUTPUT_DIRECTORY) && xxd -i fpga_binfile.bin fpga_binfile_ram.h
	@sed '1s/^/const /' $(OUTPUT_DIRECTORY)/fpga_binfile_ram.h > c-code/fpga_binfile.h


# Build task to simulate the verilog using a test bench
sim-i2c-controller:
	@mkdir -p $(SIM_DIRECTORY)
	@echo "\n---\nSynthesizing with iVerilog.\n"
	@iverilog -Wall -Iverilog-code -o .sim/i2c_controller_tb.out verilog-code/testbenches/i2c_controller_tb.v
	@vvp .sim/i2c_controller_tb.out -lxt2
	@gtkwave .sim/i2c_controller_tb.lxt verilog-code/testbenches/i2c_controller_tb.gtkw

# Remove the simulation folder
clean-simulations:
	rm -rf .sim