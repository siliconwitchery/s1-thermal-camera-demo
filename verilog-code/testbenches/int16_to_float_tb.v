// This gives us a 50MHz tik-tok
`timescale 10ns / 10ns

// Include the int to float module
`include "int16_to_float.v"

module int16_to_float_tb;

    // Generate a 25MHz clock (Note that the real chip will be 24MHz)
    reg clk = 0;
    initial begin : clk_25MHz
        forever #2 clk <= ~clk;
    end

    reg [15:0] my_int = 0;
    wire [31:0] my_float;

    int16_to_float int16_to_float (
        .clk(clk),
        .int_in(my_int),
        .float_out(my_float)
    );

    // This is where the simulation output file is saved
    initial begin
        $dumpfile(".sim/int16_to_float_tb.lxt");
        $dumpvars(0, int16_to_float_tb);
    end

    // The test routine is here
    initial begin
        # 100;
        my_int <= 1253;
        # 100;
        my_int <= -673;
        # 100;
        my_int <= 47;
        # 100;
        my_int <= -5839;
        # 100;
        $finish;
    end

endmodule