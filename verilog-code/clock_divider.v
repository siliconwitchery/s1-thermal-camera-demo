module clock_divider (
    input wire sys_clk,
    output reg slow_clk = 0
);
    // 8 bit counter
	reg [7:0] counter = 0;

    always @(posedge sys_clk) begin

        // Generate 800kHz I2C module clock from 24MHz system clock
        if (counter == 'd14) begin

            slow_clk = ~slow_clk;
            counter = 0;
        
        end else begin
        
            counter = counter + 1;
        
        end 

    end

endmodule