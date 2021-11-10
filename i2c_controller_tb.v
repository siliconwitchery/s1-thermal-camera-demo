// This gives us a 50MHz tik-tok
`timescale 10ns / 10ns

`define SCL_PERIOD 240

// Include the I2C controller, and clock divider
`include "i2c_controller.v"
`include "clock_divider.v"

module test_i2c_controller;

    // Generate a 5MHz clock (the real chip is 6MHz)
    reg hf_clk = 0;
    initial begin : clk_25MHz
        forever #2 hf_clk <= ~hf_clk;
    end

    reg reset;

    // The I2C output wires
    wire scl;
    wire sda_in;
    wire sda_out;
    wire sda_oe;

    // Local I2C clock wire (400kHz)
    wire i2c_clock;

    // Address, RX and TX registers
    reg [7:0] address; 
    wire [7:0] received_data;
    reg [7:0] transmit_data; 

    // Setting this high takes controller into write mode. Read if 0
    reg write_mode;

    // Set this to send multiple data bytes
    reg write_pending;

    // Flag to start the transfer
    reg start_transfer;

    // SDA ACK assignment from peripheral
    reg sda_peripheral_ack = 0;
    assign sda_in = sda_peripheral_ack ? 0 : 'bZ;
    
    // Instantiate the 50MHz-400kHz clock divider
    clock_divider clock_divider (
        .sys_clk(hf_clk),
        .slow_clk(i2c_clk)
    );

    // Instantiate the unit under test
    i2c_controller i2c_controller (
        .clk(i2c_clk),
        .reset(reset),
        .busy(busy),
        .address(address),
        .write_mode(write_mode),
        .transmit_data(transmit_data),
        .received_data(received_data),
        .write_pending(write_pending),
        .read_pending(read_pending),
        .start_transfer(start_transfer),
        .sda_in(sda_in),
        .sda_out(sda_out),
        .sda_oe(sda_oe),
        .scl(scl)
    );

    // Where to save the output file for GTKWave
    initial begin
        $dumpfile(".sim/test_i2c_controller.lxt");
        $dumpvars(0, test_i2c_controller);
    end

    // The test routine is here
    initial begin

        // Reset the controller
        reset = 1;
        # (`SCL_PERIOD * 3);
        
        // Test parameters and start of the test
        transmit_data = 8'hAB;
        address = 8'h9B;
        write_mode = 1;
        write_pending = 1;
        start_transfer = 1;
        reset = 0;

        // Issue the first peripheral ack (address) 
        # (`SCL_PERIOD * 10.25);
        sda_peripheral_ack = 1;
        # `SCL_PERIOD
        sda_peripheral_ack = 0;
        
        // Issue the second peripheral ack (data) and set new data
        # (`SCL_PERIOD * 9);
        sda_peripheral_ack = 1;
        transmit_data = 8'h55;
        # `SCL_PERIOD
        sda_peripheral_ack = 0;

        // Issue the third peripheral ack (data), clear write pending and stop transfer
        # (`SCL_PERIOD * 9);
        sda_peripheral_ack = 1;
        write_pending = 0;
        start_transfer = 0;
        # `SCL_PERIOD
        sda_peripheral_ack = 0;

        // Done!
        # (`SCL_PERIOD * 3);

        $finish;
    end

endmodule