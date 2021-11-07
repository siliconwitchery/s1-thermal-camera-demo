BUILD_DIRECTORY ?= .build
SIM_DIRECTORY ?= .sim

# Build task for the verilog project
build-verilog:
	@echo Creating the build directory.
	@mkdir -p $(BUILD_DIRECTORY)

	@echo "Synthesizing all the verilog files using yosys."
	@yosys -p "synth_ice40 -json $(BUILD_DIRECTORY)/hardware.json" -f verilog -q ice_40_top.v

	@echo "Placement and routing for the iCE40 uwg30 chip package."
	@nextpnr-ice40 --up5k --package uwg30 --json $(BUILD_DIRECTORY)/hardware.json --asc $(BUILD_DIRECTORY)/hardware.asc --pcf s1.pcf -q
	
	@echo "Generating the FPGA binary file."
	@icepack $(BUILD_DIRECTORY)/hardware.asc $(BUILD_DIRECTORY)/fpga_binfile.bin

	@echo "Converting the bin file into a .h file for the nRF build process."
	@cd $(BUILD_DIRECTORY) && xxd -i fpga_binfile.bin fpga_binfile_ram.h

	@echo "Appending the .h file to store binary in flash instead of ram."
	@sed '1s/^/const /' $(BUILD_DIRECTORY)/fpga_binfile_ram.h > fpga_binfile.h


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