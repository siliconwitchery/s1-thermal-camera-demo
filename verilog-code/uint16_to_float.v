module uint16_to_float(
        // Input should be unsigned 16 bt integer
        input wire [15:0] uint,

        // Output is a 32 bit float number
        output wire [31:0] float
    );
        // Wires for the exponent and mantissa values
        wire [7:0] exponent;
        wire [15:0] mantissa;

        // Exponent power
        reg [7:0] scale;

        // Figure out scale value
        always @* begin

                 if (uint & ('b1000_0000_0000_0000)) scale <= 15;
            else if (uint & ('b0100_0000_0000_0000)) scale <= 14;
            else if (uint & ('b0010_0000_0000_0000)) scale <= 13;
            else if (uint & ('b0001_0000_0000_0000)) scale <= 12;
            else if (uint & ('b1000_1000_0000_0000)) scale <= 11;
            else if (uint & ('b1000_0100_0000_0000)) scale <= 10;
            else if (uint & ('b1000_0010_0000_0000)) scale <=  9;
            else if (uint & ('b1000_0001_0000_0000)) scale <=  8;
            else if (uint & ('b1000_0000_1000_0000)) scale <=  7;
            else if (uint & ('b1000_0000_0100_0000)) scale <=  6;
            else if (uint & ('b1000_0000_0010_0000)) scale <=  5;
            else if (uint & ('b1000_0000_0001_0000)) scale <=  4;
            else if (uint & ('b1000_0000_0000_1000)) scale <=  3;
            else if (uint & ('b1000_0000_0000_0100)) scale <=  2;
            else if (uint & ('b1000_0000_0000_0010)) scale <=  1;
            else if (uint & ('b1000_0000_0000_0001)) scale <=  0;

        end

        assign exponent = 127 + scale;
        assign mantissa = uint << (16 - scale);

        // Assign the final value here
        assign float = uint == 0 ? 0 : {1'b0, exponent[7:0], mantissa[15:0], 7'b0};

endmodule