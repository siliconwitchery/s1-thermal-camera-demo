// This gives us a 50MHz clock
`timescale 10 ns / 1 ps

// Include the I2C controller, and clock divider
`include "i2c_controller.v"
`include "clock_divider.v"

module test_i2c_controller;

    // Global system clock and reset
    reg clk;
    reg reset;

    // The I2C output wires
    wire scl;
    wire sda;

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

    // SDA is bidirectional so we use this
    reg sda_ack_en = 0;
    reg sda_ack_val = 0;
    assign sda = (sda_ack_en == 1) ? sda_ack_val : 'bz;

    // Instantiate the 50MHz-400kHz clock divider
    clock_divider clock_divider (
        .sys_clk(clk),
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
        .sda(sda),
        .scl(scl)
    );

    // Where to save the output file for GTKWave
    initial begin
        $dumpfile(".sim/test_i2c_controller.lxt");
        $dumpvars(0, test_i2c_controller);
    end
    
    // Always toggle clock every 10ns, for a 50MHz period
    initial begin
        clk = 0;
        forever begin
            clk = #1 ~clk;
        end
    end

    // The test routine is here
    initial begin

        // Reset the controller
        reset = 1;
        #1000;
        
        // Test parameters and start of the test
        transmit_data = 8'hAA;
        address = 8'h9B;
        write_mode = 1;
        write_pending = 1;
        start_transfer = 1;
        reset = 0;

        // Issue the first peripheral ack (address) 
        #5299
        sda_ack_en = 1;
        #252
        sda_ack_en = 0;
        
        // Issue the second peripheral ack (data) and set new data
        #4788
        sda_ack_en = 1;
        transmit_data = 8'h55;
        #252
        sda_ack_en = 0;

        // Issue the third peripheral ack (data), clear write pending and stop transfer
        #4788
        sda_ack_en = 1;
        write_pending = 0;
        start_transfer = 0;
        #252
        sda_ack_en = 0;

        // Done!
        #10000;

        $finish;
    end

endmodule