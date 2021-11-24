// This gives us a 50MHz tik-tok
`timescale 10ns / 10ns

// Include the int to float module
`include "uint16_to_float.v"

module uint16_to_float_tb;

    reg [15:0] my_int = 0;
    wire [31:0] my_float;

    uint16_to_float uint16_to_float (
        .uint(my_int),
        .float(my_float)
    );

    // This is where the simulation output file is saved
    initial begin
        $dumpfile(".sim/uint16_to_float_tb.lxt");
        $dumpvars(0, uint16_to_float_tb);
    end

    // The test routine is here
    initial begin
        # 100;
        my_int <= 1253;
        # 100;
        my_int <= 673;
        # 100;
        my_int <= 47;
        # 100;
        $finish;
    end

endmodule