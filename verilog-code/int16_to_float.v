module int16_to_float(
        // Convertor needs a clock and takes two periods for valid data
        input wire clk,

        // Input should be a signed 16 bit integer
        input wire [15:0] int_in,

        // Output is a 32 bit float_out number
        output reg [31:0] float_out
    );

    // This signal holds the sign of the input number. Just the MSB
    wire sign;
    assign sign = int_in[15];

    // This is the absolute value of the input. Uses 2's comp
    wire [15:0] unsigned_int;
    assign unsigned_int = int_in[15] == 1 ? ~int_in + 1 : int_in ;

    // The power of the exponent value will be stored here
    reg [7:0] power;

    // Exponent is 127 + the shifted power
    wire [7:0] exponent;
    assign exponent = 127 + power;

    // Mantissa is the remaining bits left justified
    wire [15:0] mantissa;
    assign mantissa = unsigned_int << (16 - power);

    // Loop variable for the for loop below
    integer i;

    // Here we figure out the power of the exponent
    always @(posedge clk) begin
            
        for (i = 0; i < 16; i = i + 1) begin

            if (unsigned_int & (1'b1 << i)) begin
                power <= i;
            end

        end

        // Note we have a special rule if the input is zero, then output is too
        if (unsigned_int == 0) float_out <= 0;

        // The final output value will be valid on the second clock cycle
        else float_out <= {sign, exponent[7:0], mantissa[15:0], 7'b0};

    end

endmodule