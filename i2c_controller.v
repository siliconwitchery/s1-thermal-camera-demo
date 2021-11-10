module i2c_controller (
    // Main clock should be 400kHz
    input wire clk,

    // Resets everything
    input wire reset,

    // When this value is low, the data can be read or written
    output reg busy,

    // 8bit device address. LSB is ignored
    input wire [7:0] address,

    // High for write, low for read
    input wire write_mode,

    // The data in and out
    input wire [7:0] transmit_data,
    output reg [7:0] received_data,

    // If this line is high, the controller will send more data after the first one when the peripheral acks
    input wire write_pending,

    // This line goes high when the peripheral sends an ack in read mode, signifying that there is more data to read
    output reg read_pending,

    // Set this line high to start the transfer
    input wire start_transfer,

    // The real I2C lines
    input  wire sda_in,
    output reg sda_out,
    output reg sda_oe,
    output reg scl
);
    // List of possible I2C states
    localparam STATE_IDLE = 0;          // Idle
    localparam STATE_START = 1;         // Send start bit
    localparam STATE_ADDR = 2;          // Send address
    localparam STATE_RW = 3;            // Send read/write bit
    localparam STATE_WAIT_PACK = 4;     // Waiting for peripheral ack
    localparam STATE_POST_ACK_HOLD = 5; // Required to prevent an extra clock. Might remove later
    localparam STATE_SEND_CACK = 6;     // Controller responds with ack
    localparam STATE_READ_DATA = 7;     // Peripheral sending data
    localparam STATE_SEND_DATA = 8;     // Controller sending data
    localparam STATE_STOP = 9;          // Stop bit

    // State machine variable
    reg [7:0] state = STATE_IDLE;

    // We can only change state on entry, hence we need an extra variable. Else the SCL timing will be wrong
    // TODO see if we can get rid of this
    reg [7:0] next_state = STATE_IDLE;

    // Local counter for shifting data to and from the SDA line
    // TODO make this shift
    reg [7:0] counter = 0;

    // Internal address reg
    reg [7:0] i_address = 0;
    reg [7:0] i_transmit_data = 0;

    // Internal TX/RX buffers that sit just before the SDA line
    // TODO fix this comment
    reg sda_received = 0;

    // Set SDA either as in or out depending on OE
    // TODO fix this for yosys
    // assign sda = sda_oe ? sda_out : 1'b0;
    // assign sda = sda_oe ? sda_out : 1'bZ;

    // Inbound SDA is only valid on a positive SCL edge
    always @(posedge scl) begin
        sda_received = sda_in;
    end

    // This block generates SCL under the different states
    // TODO FIX THIS
    always @(negedge clk || reset) begin

        // When we're in reset, IDLE, Start or Stop, we keep scl high
        if ((reset == 1)
         || (state == STATE_IDLE) 
         || (state == STATE_START)
         || (state == STATE_STOP)) begin
            scl <= 1;

        // Keep clock low while we switch the pin direction
        end else if ((state == STATE_POST_ACK_HOLD)) begin
            scl <= 0;

        // Otherwise toggle the clock
        end else begin
            scl <= ~scl;
        end
    end

    // This block handles the different states of the I2C logic 
    always @(posedge clk or posedge reset) begin
        
        // If in reset, keep outputs high and reset everything
        if(reset == 1) begin

            // In reset, we keep state as IDLE
            state <= STATE_IDLE;
            next_state <= STATE_IDLE;

            // SDA is high impedance
            sda_oe <= 0;

            // Read register and pending is cleared
            received_data <= 8'h00;
            read_pending <= 0;

            // Busy is high
            busy <= 1;

        // This is the state transition logic
        end else begin

            // This is where we set the new state. Not at the exit, but entry
            state <= next_state;

            case (state)

                // Idle state
                STATE_IDLE: begin

                    // In idle, we keep SDA high impedance
                    sda_oe <= 0;

                    // Signify not busy
                    busy <= 0;

                    // Start transfer when this flag goes high
                    if (start_transfer == 1) next_state <= STATE_START;
                
                end

                // Issues the start sequence
                STATE_START: begin

                    // Engage drive to SDA and send low for start sequence
                    sda_oe = 1;
                    sda_out <= 0;

                    // Set counter to push out 7 addr bits and 1 RW bit
                    counter <= 6;
                    next_state <= STATE_ADDR;
                    i_address <= address;

                    // Flag that the controller is busy
                    busy <= 1;

                end

                // Sends the 7 address bits
                STATE_ADDR: begin

                    // Only change data on SDA when SCL is low
                    if (scl == 0) begin

                        // Push out address bits and decrement counter.

                        // sda_out <= address[counter];
                        // counter <= counter - 1;

                        //////
                        sda_out <= i_address[7];
                        i_address <= {i_address[6:0], 1'b0};
                        //////

                        // The last bit is R/W so we skip it and go to the RW state
                        if (counter == 0) next_state <= STATE_RW;
                        else counter <= counter - 'b1;
                    
                    end
                end

                // Sends the read/write bit
                STATE_RW: begin

                    // Only change data on SDA when SCL is low
                    if (scl == 0) begin

                        // If write mode, we send a 0, otherwise send a 1
                        sda_out <= write_mode ? 0 : 1;

                        // Then wait for the peripheral acknowledgement
                        next_state <= STATE_WAIT_PACK;

                    end
                end

                // Waiting for the peripheral to acknowledge the controller
                STATE_WAIT_PACK: begin

                    // Disable drive to SDA in order for the peripheral to pull the line
                    sda_oe <= 0;

                    // At this point, the user may update the data register
                    busy <= 0;

                    // The data is only valid when SCL is high
                    if (scl == 1) begin

                        // Low on SDA means that the peripheral acknowledged us
                        // and write pending, we go send data
                        if (sda_received == 0 && write_pending == 1) begin

                            // A single cycle delay to allow the 
                            next_state <= STATE_POST_ACK_HOLD;

                        end

                        // If we don't see an acknowledgement, then stop
                        else begin
                         
                            next_state <= STATE_STOP;
                            
                        end

                    end
                end

                // Pause required for SDA to switch back to transmit mode and peripheral
                // to prepare for sending data. This is shown in the I2C spec
                STATE_POST_ACK_HOLD: begin

                    // Now we set busy again because we're about to push out data
                    busy <= 1;
                    
                    // TODO there can be an interrupt here where the peripheral says it's ready
                    // If write mode we go to the send state
                    if (write_mode == 1) begin

                        // If outbound data is pending, we send it
                        if (write_pending == 1) begin

                            // We need to set SDA to transmit and issue the first bit here
                            sda_oe <= 1;
                            i_transmit_data <= transmit_data;
                            sda_out <= i_transmit_data[7];
                            next_state <= STATE_SEND_DATA;

                        end

                        // Otherwise we are done
                        else next_state <= STATE_STOP;

                    end

                    // Otherwise we go to read mode
                    else begin
                        next_state <= STATE_READ_DATA;
                    end

                    // Set counter to read/write 8 bits
                    counter <= 6;

                end

                // Sending 8 bits of data
                STATE_SEND_DATA: begin
                    
                    // Now we only need to change the data on SDA when SCL is low
                    if (scl == 0) begin

                        // Pre decrement the counter and then send data. Ensure SDA is driven
                        // sda_out <= transmit_data[--counter];
                        // sda_out <= transmit_data[counter];
                        // counter <= counter - 1;
                        // TODO fix this

                    //////
                        sda_out <= i_transmit_data[6];
                        i_transmit_data <= {i_transmit_data[6:0], 1'b0};
                        //////

                        // On the last bit we wait for another ack from the peripheral
                        if (counter == 0) next_state <= STATE_WAIT_PACK;
                        else counter <= counter - 'b1;

                    end
                end

                // TODO finish this
                STATE_READ_DATA: begin
                    sda_oe <= 0;
                    received_data[counter] <= sda_received;
                    if (counter == 0) begin
                        next_state <= STATE_SEND_CACK;
                        // TODO flag that the data is ready and can be read
                    end else begin
                            counter = counter - 1;
                    end
                end

                // TODO finish this
                STATE_SEND_CACK: begin
                    sda_oe <= 1;
                    sda_out <= 1;
                    // if () begin
                        // TODO if data pending, loop back to read
                    // end else begin
                        // Otherwise stop
                        next_state <= STATE_STOP;
                    // end
                end

                STATE_STOP: begin
                    sda_oe <= 1;
                    sda_out <= 1;
                    next_state <= STATE_IDLE;
                end

            endcase
        end
    end

endmodule