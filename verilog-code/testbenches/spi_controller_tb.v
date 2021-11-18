// This gives us a 50MHz tik-tok
`timescale 10ns / 10ns

// 1us is the period of the 1MHz SPI clock
`define SCK_PERIOD 100

// Include the SPI module
`include "spi_controller.v"

module spi_controller_tb;

    // Generate the 1MHz SPI clock
    reg hf_clk = 0;
    initial begin : clk_1MHz
        forever #50 hf_clk <= ~hf_clk;
    end

    // SPI chip select that we toggle in the test routine
    reg cs = 0;

    // Dummy memory
    reg [7:0] memory [9:0];

    // Address selector
    wire [13:0] data_address;

    // SPI clock is enabled when cs is high
    assign sck = cs ? hf_clk : 0;

    // Instantiate the SPI controller
    spi_controller spi_controller (
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
        for (integer i = 0; i < 10; i++) memory[i] = 1+i;
    end

    // The test routine is here
    initial begin

        // Wait a bit and send chip select high
        # (`SCK_PERIOD * 3);
        cs <= 1;

        // Pull 5 bytes of data
        # (`SCK_PERIOD * 5 * 8);

        // Send CS low again
        cs <= 0;

        // Done!
        # (`SCK_PERIOD * 3);
        $finish;
        
    end

endmodule