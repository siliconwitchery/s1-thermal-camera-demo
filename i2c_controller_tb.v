// This gives us a 50MHz tik-tok
`timescale 10ns / 10ns

// 2400ns equals a single SCL clock period
`define SCL_PERIOD 240

// Include the I2C controller, and clock divider modules
`include "i2c_controller.v"
`include "clock_divider.v"

module test_i2c_controller;

    // Generate a 25MHz clock (Note that the real chip will be 24MHz)
    reg hf_clk = 0;
    initial begin : clk_25MHz
        forever #2 hf_clk <= ~hf_clk;
    end

    // The I2C clock will run at 800kHz (twice the I2C frequency)
    wire i2c_clock;

    // Global reset line
    reg reset;

    // The I2C wires
    wire scl;
    wire sda_in;
    wire sda_out;
    wire sda_oe;

    // Address, TX and RX data registers.
    reg [6:0] address; 
    reg [7:0] transmit_data; 
    wire [7:0] received_data;

    // Option for writing or reading. Write if high, read otherwise
    reg read_write;

    // Flag to begin the transfer process
    reg enable_transfer;

    // Register for pushing out simulated peripheral data
    reg sda_peripheral_data = 'bZ;

    // Flag for simulated peripheral ack
    reg sda_peripheral_ack = 0;

    // This mechanism issues the peripheral ack or data
    assign sda_in = sda_peripheral_ack ? 0 : sda_peripheral_data;
    
    // Instantiate the 24MHz-800kHz clock divider (Note we run it here at 25MHz)
    clock_divider clock_divider (
        .sys_clk(hf_clk),
        .slow_clk(i2c_clk)
    );

    // Instantiate the I2C controller
    i2c_controller i2c_controller (
        .clk(i2c_clk),
        .reset(reset),
        .idle(idle),
        .ack(ack),
        .nack(nack),
        .address(address),
        .read_write(read_write),
        .transmit_data(transmit_data),
        .received_data(received_data),
        .enable_transfer(enable_transfer),
        .sda_in(sda_in),
        .sda_out(sda_out),
        .sda_oe(sda_oe),
        .scl(scl)
    );

    // This is where the simulation output file is saved
    initial begin
        $dumpfile(".sim/test_i2c_controller.lxt");
        $dumpvars(0, test_i2c_controller);
    end

    // The test routine is here
    initial begin

        //////////////////////////
        //      WRITE TEST      //
        //////////////////////////

        // Reset the controller and wait a bit (just for a nice visual gap in the output waveform)
        reset = 1;
        # `SCL_PERIOD;
        reset = 0;
        # (`SCL_PERIOD * 2);
        
        // Test writing 2 bytes to a device with address 0x55
        address = 'h55;
        transmit_data = 'hDB;
        read_write = 0;
        enable_transfer = 1;

        // After some time, issue the first peripheral ack for address
        # (`SCL_PERIOD * 10.25);
        sda_peripheral_ack = 1;
        # `SCL_PERIOD
        sda_peripheral_ack = 0;
        
        // Issue the second peripheral ack for data, and then set new data
        # (`SCL_PERIOD * 9);
        sda_peripheral_ack = 1;
        transmit_data = 'h6C;
        # `SCL_PERIOD
        sda_peripheral_ack = 0;

        // Issue the third peripheral ack for data and clear write pending flag to end the transfer
        # (`SCL_PERIOD * 9);
        sda_peripheral_ack = 1;
        # `SCL_PERIOD
        sda_peripheral_ack = 0;

        // Disable the transfer
        enable_transfer = 0;

        // Wait a couple periods before starting the read operation (just for a visual gap)
        # `SCL_PERIOD;
        read_write = 1;
        # `SCL_PERIOD;

        /////////////////////////
        //      READ TEST      //
        /////////////////////////

        // Now read 2 bytes from device address 0x2A
        address = 8'h2A;
        enable_transfer = 1;

        // After some time we should get the address ack from the peripheral
        # (`SCL_PERIOD * 10.5);
        sda_peripheral_ack = 1;
        # `SCL_PERIOD
        sda_peripheral_ack = 0;

        // Issue simulated data from the peripheral on the SDA
        sda_peripheral_data = 1;
        # `SCL_PERIOD
        sda_peripheral_data = 0;
        # `SCL_PERIOD
        sda_peripheral_data = 1;
        # `SCL_PERIOD
        sda_peripheral_data = 1;
        # `SCL_PERIOD
        sda_peripheral_data = 0;
        # `SCL_PERIOD
        sda_peripheral_data = 0;
        # `SCL_PERIOD
        sda_peripheral_data = 1;
        # `SCL_PERIOD
        sda_peripheral_data = 1;
        # `SCL_PERIOD
        sda_peripheral_data = 'bZ;

        // Issue second set of data after the controller sends an ack
        # (`SCL_PERIOD * 2)
        sda_peripheral_data = 0;
        # `SCL_PERIOD
        sda_peripheral_data = 1;
        # `SCL_PERIOD
        sda_peripheral_data = 1;
        # `SCL_PERIOD
        sda_peripheral_data = 1;
        # `SCL_PERIOD
        sda_peripheral_data = 0;
        # `SCL_PERIOD
        sda_peripheral_data = 1;
        # `SCL_PERIOD
        sda_peripheral_data = 0;
        # `SCL_PERIOD
        sda_peripheral_data = 0;
        # `SCL_PERIOD
        sda_peripheral_data = 'bZ;

        // Release the enable line to stop transfer and wait for stop sequence
        enable_transfer = 0;
        # `SCL_PERIOD

        // Done!
        # (`SCL_PERIOD * 3);
        $finish;
    end

endmodule