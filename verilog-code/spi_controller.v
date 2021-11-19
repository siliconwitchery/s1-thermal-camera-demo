module spi_controller (
    // Main speed system clock
    input wire clk,

    // SPI SCK pin input.
    input wire sck,

    // SPI chip select input.
    input wire cs,

    // Data output pin to the nRF
    output wire cipo,

    // Input from local memory that we will send
    input wire [7:0] data,

    // Selects which byte to read from
    output wire [13:0] data_address
);

    // Counters for counting bits and bytes pushed out on SPI data line
    reg [3:0] bit_counter = 0;
    reg [13:0] byte_counter = 0;

    // Assign cipo to the current bit selected in the data (MSB first)
    assign cipo = data[7 - bit_counter];

    // Assign data_address to the current byte value
    assign data_address = byte_counter;

    // Registers to keep track of SCK and CS edges
    reg [1:0] cs_edge_monitor = 0;
    reg [1:0] sck_edge_monitor = 0;

    // Clock should be active high (CPOL = 0) and data is valid on leading clock
    // edges (CPHA = 0)
    always @(posedge clk) begin

        // Update the edge monitors with the latest cs and sck signal values
        cs_edge_monitor <= {cs_edge_monitor[0], cs};
        sck_edge_monitor <= {sck_edge_monitor[0], sck};

        // If CS is rising edge, we reset the counters
        if (cs_edge_monitor == 'b01) begin

            bit_counter <= 0;
            byte_counter <= 0;
        
        end

        // We only change data counters on the falling edge of SCK
        else if (sck_edge_monitor == 'b10) begin

            // Increment the bit counter
            bit_counter <= bit_counter + 1;

            // Increment the byte every 8 bits
            if (bit_counter == 7) begin

                byte_counter <= byte_counter + 1;
                bit_counter <= 0;

            end

        end

    end
    
endmodule