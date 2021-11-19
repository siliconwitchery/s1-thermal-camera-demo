module i2c_controller (
    // Module clock should be double the desired I2C Clock frequency.
    input wire clk,

    // Resets everything.
    input wire reset,

    // This line is high while the controller is idling
    output reg idle,

    // Goes high after a successful data transfer. On this rising edge, data may
    // be written to, or read from on the transmit_data and received_data lines.
    output reg ack,

    // If this goes high, it means that the peripheral did not ack the address
    // or the data.
    output reg nack,

    // The 7 bit peripheral device address.
    input wire [6:0] address,

    // Setting this high will put the controller into read mode, otherwise write.
    input wire read_write,

    // The data in and out interface to the module. Only valid to read or write
    // on the rising edge of ack.
    input wire [7:0] transmit_data,
    output reg [7:0] received_data,

    // Set this line high to start the transfer, and keep it high for continuous
    // reads or writes.
    input wire enable_transfer,

    // Set this line high if you don't want to send a stop bit
    input wire issue_restart,

    // The I2C lines. SDA in, out, and oe should be routed to a tristate block.
    input  wire sda_in,
    output reg sda_out,
    output reg sda_oe,
    output reg scl
);
    // List of the I2C controller states
    localparam STATE_IDLE           = 0;  // Idle state
    localparam STATE_START          = 1;  // Sends the start sequence
    localparam STATE_ADDR           = 2;  // Sends the 7 address bits
    localparam STATE_RW             = 3;  // Sends the read or write bit
    localparam STATE_WAIT_PACK      = 4;  // Checks for the peripheral ACK
    localparam STATE_PREPARE_DATA   = 5;  // Prepares data to send out
    localparam STATE_SEND_DATA      = 6;  // Sends data bits to the peripheral
    localparam STATE_READ_DATA      = 7;  // Reads data bits from the peripheral
    localparam STATE_DECIDE_CACK    = 8;  // Send internal ack and wait for decision
    localparam STATE_SEND_CACK      = 9;  // Prepares the controller ACK response
    localparam STATE_RELEASE_CACK   = 10; // Decides on repeated reads
    localparam STATE_STOP           = 11; // Sends the stop sequence

    // State machine variable
    reg [4:0] state = STATE_IDLE;

    // A separate state variable is needed because SDA and SCL are driven from
    // separate always blocks. We need to preserve the current state until the
    // state machine makes the transition into the new state.
    reg [4:0] next_state = STATE_IDLE;

    // This flash is used to only report data acks, and not address acks.
    reg data_sent = 0;

    // Three bit counter for counting bits pushed or pulled on SDA
    reg [3:0] counter = 0;

    // This block generates the SCL depending on which state is active
    always @(negedge clk) begin

        // When we're in reset, IDLE, Start or Stop, we should keep SCL high
        if (reset == 1 || 
            state == STATE_IDLE || 
            state == STATE_START || 
            state == STATE_STOP) begin

            scl <= 1;

        end

        // Keep low while we're waiting for new data, and CACK is being issued
        else if (state == STATE_PREPARE_DATA ||
                 state == STATE_DECIDE_CACK ||
                 state == STATE_SEND_CACK) begin

            scl <= 0;

        end

        // Otherwise toggle the SCL line
        else begin

            scl <= ~scl;

        end

    end

    // This block handles the different states of the I2C controller logic
    always @(posedge clk) begin
        
        // If in reset, keep outputs high and reset everything
        if(reset == 1) begin

            // In reset, we keep the state variables in IDLE
            state <= STATE_IDLE;
            next_state <= STATE_IDLE;

            // ACK and NACK is cleared
            ack <= 0;
            nack <= 0;

            // Clear idle
            idle <= 0;

            // SDA output is disabled
            sda_oe <= 0;

            // And the received_data register is cleared
            received_data <= 0;

            // Clear internals
            data_sent = 0;
            counter <= 0;

        end

        // Otherwise run the I2C controller logic if not in reset
        else begin

            // This is where we set the new state
            state <= next_state;

            case (state)

                // Idle state
                STATE_IDLE: begin

                    // If ACK remains high, it means that the controller is idle
                    idle <= 1;

                    // Disable output in IDLE
                    sda_oe <= 0;

                    // Start transfer when enable goes high
                    if (enable_transfer == 1) next_state <= STATE_START;
                
                end

                // Sends the start sequence
                STATE_START: begin

                    // Clear the ACK
                    idle <= 0;

                    // Engage SDA to output and set it as low
                    sda_oe <= 1;
                    sda_out <= 0;

                    // Set counter to push out the 7 address bits
                    counter <= 6;
                    next_state <= STATE_ADDR;

                end

                // Sends the 7 address bits
                STATE_ADDR: begin

                    // Only change data on SDA when SCL is low
                    if (scl == 0) begin

                        // Push out the address bits on SDA
                        sda_out <= address[counter];

                        // On the final bit, we go to the RW state
                        if (counter == 0) next_state <= STATE_RW;

                        // Otherwise decrement the counter until we reach 0
                        else counter <= counter - 1;
                    
                    end
                end

                // Sends the read or write bit
                STATE_RW: begin

                    // Only change data on SDA when SCL is low
                    if (scl == 0) begin

                        // Read issues a high bit, and write is low
                        sda_out <= read_write ? 1 : 0;

                        // Then wait for the peripheral acknowledgement
                        next_state <= STATE_WAIT_PACK;

                    end
                end

                // Checks for the peripheral ACK
                STATE_WAIT_PACK: begin

                    // Disable drive to SDA to receive the ACK
                    sda_oe <= 0;

                    // The data is only valid when SCL is high
                    if (scl == 1) begin

                        // Reading low on SDA means that the peripheral 
                        // acknowledged us
                        if (sda_in == 0) begin

                            // If the data was ACK'd then trigger this flag. The
                            // user can prepare the new data on this rising edge
                            if (data_sent == 1) ack <= 1;

                            // Clear the data sent flag once we issue the ack
                            data_sent <= 0;

                            // Set counter to read/write 8 bits
                            counter <= 7;

                            // Go to read or write accordingly
                            next_state <= read_write ? STATE_READ_DATA 
                                                     : STATE_PREPARE_DATA;

                        end

                        // If we don't get an acknowledgement, then stop
                        else begin
                         
                            nack <= 1;
                            next_state <= STATE_STOP;
                            
                        end

                    end
                end

                // We need to send out the first bit here otherwise there'll be
                // a delay of one SCL cycle. We can't set the output at exactly
                // the same time as the state machine enters the next state
                STATE_PREPARE_DATA: begin
                
                    // Clear the ack
                    ack <= 0;
                    
                    // If enabled to do so, we prepare to send data
                    if (enable_transfer == 1) begin

                        // Enable the output and push out the MSB
                        sda_oe <= 1;
                        sda_out <= transmit_data[7];

                        // Adjust the counter and send out the remaining 7 bits
                        counter <= 6;
                        next_state <= STATE_SEND_DATA;

                    end

                    // Otherwise stop
                    else begin

                        // If we're not issuing a restart, this is how we set the
                        // stop sequence.
                        if (issue_restart == 0) begin
                            sda_oe <= 1;
                            sda_out <= 0;
                        end

                        next_state <= STATE_STOP;

                    end

                end

                // Sends the remaining 7 data bits to the peripheral
                STATE_SEND_DATA: begin

                    // We can only change the data when SCL is low
                    if (scl == 0) begin

                        // Send 7 remaining data bits
                        sda_out <= transmit_data[counter];

                        // On the last bit we return to wait for an ack
                        if (counter == 0) begin
                            data_sent <= 1;
                            next_state <= STATE_WAIT_PACK;
                        end

                        // Otherwise decrement the counter
                        else counter <= counter - 1;

                    end
                end

                // Reads 7 data bits from the peripheral
                STATE_READ_DATA: begin
                    
                    // Clear the ACK
                    ack <= 0;

                    // If enabled to do so, we read data
                    if (enable_transfer == 1) begin
                   
                        // Enable SDA to be input
                        sda_oe <= 0;

                        // Data is only valid when SCL is high
                        if (scl == 1) begin

                            // Pull in 8 data bits
                            received_data[counter] <= sda_in;

                            // Once we hit 0, the controller sends an ACK or NACK
                            if (counter == 0) next_state <= STATE_DECIDE_CACK;

                            // Otherwise decrement the counter
                            else counter <= counter - 1;

                        end

                    end

                    // Otherwise stop
                    else begin
                        next_state <= STATE_STOP;
                    end

                end

                // Decide if controller should send an ACK or NACK
                STATE_DECIDE_CACK: begin

                    // Flag the ACK so the user can read the data
                    ack <= 1;

                    next_state <= STATE_SEND_CACK;

                end

                // Prepares the controller ACK response
                STATE_SEND_CACK: begin

                    // Flag the ACK so the user can read the data
                    ack <= 0;
                    sda_oe <= 1;

                    // Send an ACK
                    if (enable_transfer == 1) begin
                        sda_out <= 0;
                    end

                    // Otherwise a NACK
                    else begin
                        sda_out <= 1;
                    end

                    next_state <= STATE_RELEASE_CACK;

                end

                // Similar to STATE_PREPARE_DATA, we need an extra state to
                // release CACK at the right time. Also decide if we want to
                // read again from the device
                STATE_RELEASE_CACK: begin

                    // If still enabled, we can read again
                    if (enable_transfer == 1) begin
                        
                        // Release SDA for another read if needed
                        if (scl == 0) begin
                            sda_oe <= 0;
                        end

                        // Set counter and go read 8 bits
                        counter <= 7;
                        next_state <= STATE_READ_DATA;

                    end

                    // Otherwise stop
                    else begin
                        
                        // Issue stop sequence only when SCL is low
                        if (scl == 0) begin

                            sda_oe <= 1;
                            sda_out <= 0;
                            next_state <= STATE_STOP;

                        end

                    end

                end

                // Sends the stop sequence
                STATE_STOP: begin

                    // Issue stop sequence where SDA goes high after SCL
                    sda_oe <= 1;
                    sda_out <= 1;

                    // Clear ACK and NACK
                    ack <= 0;
                    nack <= 0;

                    // Return to IDLE
                    next_state <= STATE_IDLE;
                    
                end

            endcase
            
        end
    end

endmodule