// This gives us a 50MHz tik-tok
`timescale 10ns / 10ns

// Include the int to float module
`include "int16_to_float.v"

module int16_to_float_tb;

    // This is where the simulation output file is saved
    integer i;
    initial begin
        $dumpfile(".sim/int16_to_float_tb.lxt");
        $dumpvars(0, int16_to_float_tb);
        
        // Dump partial input array
        for (i = 0; i < 8; i = i + 1)
            $dumpvars(1, raw_pixel_buffer[i]);
        
        // Dump partial output array
        for (i = 0; i < 16; i = i + 1)
            $dumpvars(1, output_pixel_buffer[i]);
    end

    // Generate a 50MHz clock (Note that the real chip will be 24MHz)
    reg clk = 0;
    initial begin : clk_50MHz
        forever #1 clk <= ~clk;
    end

    // Goes high once data is read and ready
    reg raw_bytes_ready_to_process = 0;

    // Run the simulation for 50 cycles
    initial begin
        #9
        raw_bytes_ready_to_process = 1;
        #2
        raw_bytes_ready_to_process = 0;
        #100
        $finish;
    end

    // Dummy interrupt line
    wire INT;

    // Dummy reset line
    reg reset = 0;

    // Input 6x int16 buffer
    reg [7:0] raw_pixel_buffer [1535:0];   
    
    // Create some dummy data inside the buffer
    initial begin

        raw_pixel_buffer[0] = 'hFF;
        raw_pixel_buffer[1] = 'hAC;
        raw_pixel_buffer[2] = 'hFF;
        raw_pixel_buffer[3] = 'hB4;
        raw_pixel_buffer[4] = 'hFF;
        raw_pixel_buffer[5] = 'hAA;
        raw_pixel_buffer[6] = 'hFF;
        raw_pixel_buffer[7] = 'hB3;

    end

    ////////////////////////////////////////////////////////////////
    ////////////////////////////////////////////////////////////////
    ////////////////////////////////////////////////////////////////

    // Variables for the data processing and final camera data
    integer bytes_loaded;               // Counter for bytes loaded as int16
    integer bytes_saved;                // Counter for bytes saved as float
    reg data_processing_completed;      // Goes high once data is ready

    // Connect the data process complete flag to the interrupt pin
    assign INT = data_processing_completed;

    // Intermediate registers for the int16 to float conversion module
    reg [15:0] int_data_in;
    wire [31:0] float_data_out;

    // Instantiation and wiring up the module
    int16_to_float int16_to_float (
        .clk(clk),
        .int_in(int_data_in),
        .float_out(float_data_out)
    );

    // Final output buffer
    reg [7:0] output_pixel_buffer[3071:0];  // Final output buffer of floats

    // State machine variable for the processor
    reg [7:0] data_processor_state = STATE_DATA_WAIT_FOR_START;

    // List of data processor states
    localparam STATE_DATA_WAIT_FOR_START    =  0;
    localparam STATE_DATA_READ_MSB          =  1;
    localparam STATE_DATA_READ_LSB          =  2;
    localparam STATE_WAIT_FOR_CONVERSION_1  =  3;
    localparam STATE_WAIT_FOR_CONVERSION_2  =  4;
    localparam STATE_WAIT_FOR_CONVERSION_3  =  5;
    localparam STATE_DATA_WRITE_1           =  6;
    localparam STATE_DATA_WRITE_2           =  7;
    localparam STATE_DATA_WRITE_3           =  8;
    localparam STATE_DATA_WRITE_4           =  9;
    localparam STATE_INCREMENT_COUNTERS     = 10;
    localparam STATE_DATA_PROCESSED         = 11;

    // The state machine itself
    always @(posedge clk) begin

        // If reset, we start again
        if (reset == 1) begin

            // Bring state back to the start
            data_processor_state <= STATE_DATA_WAIT_FOR_START;

            data_processing_completed <= 1;

            bytes_loaded <= 0;
            bytes_saved <= 0;

        end

        else begin

            case (data_processor_state)

                STATE_DATA_WAIT_FOR_START: begin
                
                    // Keep done flag high
                    data_processing_completed <= 1;

                    // Reset counters
                    bytes_loaded <= 0;
                    bytes_saved <= 0;

                    // Once the data is ready, we can proceed with processing
                    if (raw_bytes_ready_to_process == 1) 
                        data_processor_state <= STATE_DATA_READ_MSB;

                end

                STATE_DATA_READ_MSB: begin

                    // Clear the flag to indicate that we are busy
                    data_processing_completed <= 0;

                    int_data_in[15:8] 
                        <= raw_pixel_buffer[bytes_loaded];

                    data_processor_state <= STATE_DATA_READ_LSB;

                end

                STATE_DATA_READ_LSB: begin
                    
                    // Then load the LSB
                    int_data_in[7:0] 
                        <= raw_pixel_buffer[bytes_loaded + 1];

                    data_processor_state <= STATE_WAIT_FOR_CONVERSION_1;

                end

                STATE_WAIT_FOR_CONVERSION_1: begin

                    // Here we wait two cycles for the conversion to complete
                    data_processor_state <= STATE_WAIT_FOR_CONVERSION_2;

                end

                STATE_WAIT_FOR_CONVERSION_2: begin

                    data_processor_state <= STATE_WAIT_FOR_CONVERSION_3;

                end

                STATE_WAIT_FOR_CONVERSION_3: begin

                    data_processor_state <= STATE_DATA_WRITE_1;

                end

                STATE_DATA_WRITE_1: begin

                    output_pixel_buffer[bytes_saved] 
                        <= float_data_out[31:24];

                    data_processor_state <= STATE_DATA_WRITE_2;

                end

                STATE_DATA_WRITE_2: begin

                    output_pixel_buffer[bytes_saved + 1] 
                        <= float_data_out[23:16];

                    data_processor_state <= STATE_DATA_WRITE_3;

                end

                STATE_DATA_WRITE_3: begin

                    output_pixel_buffer[bytes_saved + 2] 
                        <= float_data_out[15:8];

                    data_processor_state <= STATE_DATA_WRITE_4;

                end

                STATE_DATA_WRITE_4: begin

                    output_pixel_buffer[bytes_saved + 3] 
                        <= float_data_out[7:0];

                    data_processor_state <= STATE_INCREMENT_COUNTERS;

                end

                STATE_INCREMENT_COUNTERS: begin

                    bytes_loaded <= bytes_loaded + 2;
                    bytes_saved <= bytes_saved + 4;

                    data_processor_state <= STATE_DATA_PROCESSED;

                end

                STATE_DATA_PROCESSED: begin

                    // Process 1536 bytes for the full buffer
                    if (bytes_loaded < 8) 
                        data_processor_state <= STATE_DATA_READ_MSB;

                    else 
                        data_processor_state <= STATE_DATA_WAIT_FOR_START;

                end
                
            endcase

        end

    end

    ////////////////////////////////////////////////////////////////
    ////////////////////////////////////////////////////////////////
    ////////////////////////////////////////////////////////////////

endmodule