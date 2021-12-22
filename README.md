# S1 Thermal Camera Demo

This is an example project demonstrating use of the FPGA to read a sensor, and send the data over to a web app using BLE. You can find the corresponding code in the folders [verilog-code](https://github.com/siliconwitchery/s1-thermal-camera-demo/tree/main/verilog-code), [c-code](https://github.com/siliconwitchery/s1-thermal-camera-demo/tree/main/c-code), and [web-code](https://github.com/siliconwitchery/s1-thermal-camera-demo/tree/main/web-code).

Photo of device with labels

`MLX90640 <--I2C--> iCE40 <--SPI--> nRF52811 <--BLE--> web app`

## Build and Flash

To build and flash the application, you need to do the following steps. The easiest way is to run 
``` bash
cp .vscode/tasks_template.jsonc .vscode/tasks.json
``` 
and run the corresponding tasks from VSCode.

### Build FPGA binary:

``` bash
make build-verilog NRF_SDK_PATH={YOUR_NRF_SDK_PATH} GNU_INSTALL_ROOT={YOUR_GNU_INSTALL_ROOT}
```

### Build nRF binary:

``` bash
make NRF_SDK_PATH={YOUR_NRF_SDK_PATH} GNU_INSTALL_ROOT={YOUR_GNU_INSTALL_ROOT}
```

### Flash Softdevice:
``` bash
make flash-s112-softdevice NRF_SDK_PATH={YOUR_NRF_SDK_PATH} GNU_INSTALL_ROOT={YOUR_GNU_INSTALL_ROOT}
```

### Flash nRF with FPGA binary:
In the [Makefile](https://github.com/siliconwitchery/s1-thermal-camera-demo/blob/main/Makefile), comment out the line `CFLAGS += -DBLUETOOTH_ENABLED`. This selects the main function `main_fpga_flasher_app()` which writes the FPGA binary from onboard flash into external flash. We need to this because there is not enough space on the onboard flash of the nRF chip for both the S112 Softdevice and the FPGA binary. In your application, you can choose to upload the FPGA binary over BLE instead.

### Flash nRF chip:
Uncomment `CFLAGS += -DBLUETOOTH_ENABLED` from the [Makefile](https://github.com/siliconwitchery/s1-thermal-camera-demo/blob/main/Makefile)
``` bash
make flash NRF_SDK_PATH={YOUR_NRF_SDK_PATH} GNU_INSTALL_ROOT={YOUR_GNU_INSTALL_ROOT}
```

### Run web app:
Open [web-code/index.html](https://github.com/siliconwitchery/s1-thermal-camera-demo/blob/main/web-code/index.html) in Chrome. Click the "Connect" buttons and select the S1-Module from the popup menu.

That's it! The device will begin streaming and you should see something like this
![web application screenshot](https://github.com/siliconwitchery/s1-thermal-camera-demo/blob/main/images/web-app-screenshot.png)

## Licence

**This design is released under the [Creative Commons Attribution 4.0 International](https://creativecommons.org/licenses/by/4.0/) Licence.**

This is a human-readable summary of (and not a substitute for) the [license](https://creativecommons.org/licenses/by/4.0/legalcode).

### You are free to:

**Share** — copy and redistribute the material in any medium or format

**Adapt** — remix, transform, and build upon the material
for any purpose, even commercially.

The licensor cannot revoke these freedoms as long as you follow the license terms.

### Under the following terms:

**Attribution** — You must give appropriate credit, provide a link to the license, and indicate if changes were made. You may do so in any reasonable manner, but not in any way that suggests the licensor endorses you or your use.

**No additional restrictions** — You may not apply legal terms or technological measures that legally restrict others from doing anything the license permits.

### Notices:

You do not have to comply with the license for elements of the material in the public domain or where your use is permitted by an applicable exception or limitation.

No warranties are given. The license may not give you all of the permissions necessary for your intended use. For example, other rights such as publicity, privacy, or moral rights may limit how you use the material.