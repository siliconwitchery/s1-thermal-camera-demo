module spi_controller (
    // SPI SCK pin input.
    input wire sck,

    // SPI chip select input.
    input wire cs,

    // Data output pin to the nRF
    output reg cipo,

    // Input from local memory that we will send
    input wire [7:0] data,

    // Selects which byte to read from
    output reg [15:0] data_address
);

// Local states for the SPI transfer operations
localparam STATE_STOPPED = 0;

// State machine variable
reg [4:0] state = STATE_STOPPED;

// Counter for counting bits pushed out on SPI data line
reg [32:0] counter = 0;

always @(posedge sck) begin

    // We use active high CS so this functions as a reset line
    if (cs == 0) begin

        state <= STATE_STOPPED;

    end

    else begin

        case (state)

            STATE_STOPPED: begin

            end

        endcase

    end

end

    
endmodule