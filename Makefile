# S1 Thermal MLX90640 camera demo
# ------------------
#
# Copyright 2021 Silicon Witchery AB
#
# Permission to use, copy, modify, and/or distribute this software for any 
# purpose with or without fee is hereby granted, provided that the above
# copyright notice and this permission notice appear in all copies.
#
# THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES WITH
# REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF MERCHANTABILITY 
# AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY SPECIAL, DIRECT, 
# INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES WHATSOEVER RESULTING FROM 
# LOSS OF USE, DATA OR PROFITS, WHETHER IN AN ACTION OF CONTRACT, NEGLIGENCE OR
# OTHER TORTIOUS ACTION, ARISING OUT OF OR IN CONNECTION WITH THE USE OR 
# PERFORMANCE OF THIS SOFTWARE.
#
# ------------------


PROJECT_NAME = thermal_camera_demo

# Put the nRF SDK path here. If you don't have it, download it here:
# https://www.nordicsemi.com/Products/Development-software/nRF5-SDK
NRF_SDK_PATH ?=

# Put your arm GCC path here. If you don't have it, download it here:
# https://developer.arm.com/tools-and-software/open-source-software/developer-tools/gnu-toolchain/gnu-rm/downloads
GNU_INSTALL_ROOT ?=

# Source files
SRC_FILES += \
  c-code/main.c \
  $(NRF_SDK_PATH)/components/ble/ble_advertising/ble_advertising.c \
  $(NRF_SDK_PATH)/components/ble/common/ble_advdata.c \
  $(NRF_SDK_PATH)/components/ble/common/ble_conn_params.c \
  $(NRF_SDK_PATH)/components/ble/common/ble_srv_common.c \
  $(NRF_SDK_PATH)/components/ble/nrf_ble_gatt/nrf_ble_gatt.c \
  $(NRF_SDK_PATH)/components/libraries/pwr_mgmt/nrf_pwr_mgmt.c \
  $(NRF_SDK_PATH)/components/softdevice/common/nrf_sdh_ble.c \
  $(NRF_SDK_PATH)/components/softdevice/common/nrf_sdh.c \

# Include paths
INC_FOLDERS += \
  c-code \
  $(NRF_SDK_PATH)/components/ble/ble_advertising \
  $(NRF_SDK_PATH)/components/ble/common \
  $(NRF_SDK_PATH)/components/ble/nrf_ble_gatt \
  $(NRF_SDK_PATH)/components/ble/peer_manager \
  $(NRF_SDK_PATH)/components/libraries/mutex \
  $(NRF_SDK_PATH)/components/libraries/pwr_mgmt \
  $(NRF_SDK_PATH)/components/softdevice/common \
  $(NRF_SDK_PATH)/components/softdevice/s112/headers \
  $(NRF_SDK_PATH)/components/softdevice/s112/headers/nrf52 \

# Use the S112 bluetooth stack linker file
# LINKER_FILE = c-code/thermal_camera_demo_linker.ld
# CFLAGS += -DBLUETOOTH_ENABLED

# Additional C flags
CFLAGS += -DBLE_STACK_SUPPORT_REQD
CFLAGS += -DNRF_SD_BLE_API_VERSION=7
CFLAGS += -DS112
CFLAGS += -DSOFTDEVICE_PRESENT

# Additional assembler flags
ASMFLAGS += -DNRF_SD_BLE_API_VERSION=7
ASMFLAGS += -DS112
ASMFLAGS += -DSOFTDEVICE_PRESENT

# This is where the magic happens.
include s1-sdk/s1.mk

# Build task for the verilog project
build-verilog:
	@mkdir -p $(OUTPUT_DIRECTORY)
	@echo "\n---\nChecking with iverilog.\n"
	@iverilog -Wall -Iverilog-code -o $(OUTPUT_DIRECTORY)/temp_top.out -i verilog-code/top.v
	@echo "\n---\nSynthesizing.\n"
	@yosys -p "synth_ice40 -json $(OUTPUT_DIRECTORY)/hardware.json" -q -Wall verilog-code/top.v
	@echo "\n---\nPlace and route.\n"
	@nextpnr-ice40 --up5k --package uwg30 -q --json $(OUTPUT_DIRECTORY)/hardware.json --asc $(OUTPUT_DIRECTORY)/hardware.asc --pcf s1-sdk/s1.pcf
	@icepack $(OUTPUT_DIRECTORY)/hardware.asc $(OUTPUT_DIRECTORY)/fpga_binfile.bin
	@cd $(OUTPUT_DIRECTORY) && xxd -i fpga_binfile.bin fpga_binfile_ram.h
	@sed '1s/^/const /' $(OUTPUT_DIRECTORY)/fpga_binfile_ram.h > c-code/fpga_binfile.h

# Build task to simulate the i2c controller
sim-i2c-controller:
	@mkdir -p $(SIM_DIRECTORY)
	@echo "\n---\nSynthesizing with iVerilog.\n"
	@iverilog -Wall -Iverilog-code -o $(SIM_DIRECTORY)/i2c_controller_tb.out verilog-code/testbenches/i2c_controller_tb.v
	@vvp $(SIM_DIRECTORY)/i2c_controller_tb.out -lxt2
	@gtkwave $(SIM_DIRECTORY)/i2c_controller_tb.lxt verilog-code/testbenches/i2c_controller_tb.gtkw

# Build task to simulate the spi controller
sim-spi-controller:
	@mkdir -p $(SIM_DIRECTORY)
	@echo "\n---\nSynthesizing with iVerilog.\n"
	@iverilog -Wall -Iverilog-code -o $(SIM_DIRECTORY)/spi_controller_tb.out verilog-code/testbenches/spi_controller_tb.v
	@vvp $(SIM_DIRECTORY)/spi_controller_tb.out -lxt2
	@gtkwave $(SIM_DIRECTORY)/spi_controller_tb.lxt verilog-code/testbenches/spi_controller_tb.gtkw

sim-int16-to-float:
	@mkdir -p $(SIM_DIRECTORY)
	@echo "\n---\nSynthesizing with iVerilog.\n"
	@iverilog -Wall -Iverilog-code -o $(SIM_DIRECTORY)/int16_to_float_tb.out verilog-code/testbenches/int16_to_float_tb.v
	@vvp $(SIM_DIRECTORY)/int16_to_float_tb.out -lxt2
	@gtkwave $(SIM_DIRECTORY)/int16_to_float_tb.lxt verilog-code/testbenches/int16_to_float_tb.gtkw

# Remove the simulation folder
clean-simulations:
	rm -rf $(SIM_DIRECTORY)

# "make flash-s112-softdevice" will flash the bluetooth stack software to the nRF chip
flash-s112-softdevice:
	nrfjprog -f nrf52 --program $(NRF_SDK_PATH)/components/softdevice/s112/hex/*.hex --sectorerase -r

# "make nrf_sdk_config" will start the nRF SDK configuration utility for editing sdk_config.h
nrf_sdk_config:
	java -jar $(NRF_SDK_PATH)/external_tools/cmsisconfig/CMSIS_Configuration_Wizard.jar c-code/sdk_config.h