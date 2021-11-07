module clock_divider (
    input wire sys_clk,
    output reg slow_clk = 0
);
    // 8 bit counter
	reg [7:0] counter = 0;

    always @(sys_clk) begin

        // Toggle the slow_clk every 50 50MHz ticks to give us 400kHz
        if (counter == 125) begin

            slow_clk = ~slow_clk;
            counter = 0;
        
        end else begin
        
            counter = counter + 1;
        
        end 

    end

endmodule