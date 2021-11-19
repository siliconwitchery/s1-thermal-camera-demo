module spi_controller (
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

    // Assign cipo to the current bit in data with MSB first
    assign cipo = data[7 - bit_counter];

    // Assign data_address to the current byte
    assign data_address = byte_counter;

    // Clock is active high (CPOL = 0) and data on leading edge (CPHA = 0)
    // We therefore only change data on the negative edge
    always @(negedge sck) begin

        // We use active high CS so this functions as a reset line
        if (cs == 0) begin

            // Reset the two counters
            bit_counter <= 0;
            byte_counter <= 0;

        end

        // Otherwise clock out bits and bytes accordingly
        else begin

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