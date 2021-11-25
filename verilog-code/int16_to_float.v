module int16_to_float(
    input wire clk,
        // Input should be a signed 16 bit integer
        input wire [15:0] int_in,

        // Output is a 32 bit float_out number
        output reg [31:0] float_out
    );

    // This signal holds the sign of the number
    wire sign;

    // This signal represents the the absolute value of the input int_in
    wire [15:0] unsigned_int;

    // This value represents the power of the exponent value
    reg [7:0] power;

    // Exponent and mantissa are used to build the final float_out value
    wire [7:0] exponent;
    wire [15:0] mantissa;

    // Assign the MSB to the sign wire
    assign sign = int_in[15];

    // This assigns a two's compliment of int_in to unsigned_int
    assign unsigned_int = sign == 1 ? ~int_in + 1 : int_in ;

    // We assign the exponent and mantissa according to the power
    assign exponent = 127 + power;
    assign mantissa = unsigned_int << (16 - power);

    // Here we figure out the power of the exponent
    always @(posedge clk) begin
            
        if      (unsigned_int & ('b1000_0000_0000_0000)) power <= 15;
        else if (unsigned_int & ('b0100_0000_0000_0000)) power <= 14;
        else if (unsigned_int & ('b0010_0000_0000_0000)) power <= 13;
        else if (unsigned_int & ('b0001_0000_0000_0000)) power <= 12;
        else if (unsigned_int & ('b0000_1000_0000_0000)) power <= 11;
        else if (unsigned_int & ('b0000_0100_0000_0000)) power <= 10;
        else if (unsigned_int & ('b0000_0010_0000_0000)) power <=  9;
        else if (unsigned_int & ('b0000_0001_0000_0000)) power <=  8;
        else if (unsigned_int & ('b0000_0000_1000_0000)) power <=  7;
        else if (unsigned_int & ('b0000_0000_0100_0000)) power <=  6;
        else if (unsigned_int & ('b0000_0000_0010_0000)) power <=  5;
        else if (unsigned_int & ('b0000_0000_0001_0000)) power <=  4;
        else if (unsigned_int & ('b0000_0000_0000_1000)) power <=  3;
        else if (unsigned_int & ('b0000_0000_0000_0100)) power <=  2;
        else if (unsigned_int & ('b0000_0000_0000_0010)) power <=  1;
        else if (unsigned_int & ('b0000_0000_0000_0001)) power <=  0;

        // And finally save the value into the output reg. Note we have a
        // special rule for if the input is zero
        float_out <= unsigned_int == 0 ? 
            0 : {sign, exponent[7:0], mantissa[15:0], 7'b0};

    end

endmodule