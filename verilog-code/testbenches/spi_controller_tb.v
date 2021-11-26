// This gives us a 50MHz tik-tok
`timescale 10ns / 10ns

// 1us is the period of the 1MHz SPI clock
`define SCK_PERIOD 100

// Include the SPI module
`include "spi_controller.v"

module spi_controller_tb;

    // Generate high speed system clock
    reg hf_clk = 0;
    initial begin : clk_10MHz
        forever #5 hf_clk <= ~hf_clk;
    end

    // Generate the 1MHz SPI clock
    reg spi_clk = 0;
    initial begin : clk_1MHz
        forever #50 spi_clk <= ~spi_clk;
    end

    // SPI chip select that we toggle in the test routine
    reg cs = 0;

    wire cipo;

    // Dummy memory
    reg [7:0] memory [15:0];

    // Address selector
    wire [13:0] data_address;

    // SPI clock is enabled when cs is high
    assign sck = cs ? spi_clk : 0;

    // Instantiate the SPI controller
    spi_controller spi_controller (
        .clk(hf_clk),
        .sck(sck),
        .cs(cs),
        .cipo(cipo),
        .data(memory[data_address]),
        .data_address(data_address)
    );

    // This is where the simulation output file is saved
    initial begin
        $dumpfile(".sim/spi_controller_tb.lxt");
        $dumpvars(0, spi_controller_tb);
    end

    // Pregen some data
    initial begin
        memory[0] = 'hFF;
        memory[1] = 'hD4;
        memory[2] = 'h01;
        memory[3] = 'h23;
        memory[4] = 'hFF;
        memory[5] = 'hC9;
        memory[6] = 'h01;
        memory[7] = 'h23;
        memory[8] = 'hFF;
        memory[9] = 'hD5;
        memory[10] = 'h01;
        memory[11] = 'h24;
        memory[12] = 'hFF;
        memory[13] = 'hC7;
        memory[14] = 'h01;
        memory[15] = 'h23;
    end

    // The test routine is here
    initial begin

        // Wait a bit and send chip select high
        # (`SCK_PERIOD * 3);
        cs <= 1;

        // Pull 16 bytes of data
        # (`SCK_PERIOD * 16 * 8);

        // Send CS low again
        cs <= 0;

        // Done!
        # (`SCK_PERIOD * 3);
        $finish;
        
    end

endmodule