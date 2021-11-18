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

    // Instantiate the SPI controller
    spi_controller spi_controller (
        .sck(sck),
        .cs(cs),
        .cipo(cipo),
        .data(data),
        .data_address(data_address)
    );

    // This is where the simulation output file is saved
    initial begin
        $dumpfile(".sim/spi_controller_tb.lxt");
        $dumpvars(0, spi_controller_tb);
    end

    // The test routine is here
    initial begin

        
        // Done!
        # (`SCK_PERIOD * 50);
        $finish;
    end

endmodule