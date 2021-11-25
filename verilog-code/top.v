// Include the other modules
`include "spi_controller.v"
`include "i2c_controller.v"
`include "clock_divider.v"
`include "int16_to_float.v"

// Multiply this by posedge ticks to get MS delays. Based on 24MHz clock
`define US_TICKS 24

module top (
    input  wire SCK,    // SPI clock from nRF
    input  wire CS,     // SPI chip select from nRF
    output wire CIPO,   // SPI data out to nRF
    input  wire COPI,   // SPI data in from nRF
    output wire INT,    // Interrupt trigger to nRF
    output wire D3,     // LED on S1 Popout. Used for status
    input  wire D4,     // Button on S1 Popout. Used for reset
    output wire D5,     // SDA pin on S1 Popout
    output wire D6      // SCL pin on S1 Popout
);
    // -------------------------------------------------------------------------
    //
    // Main reset and clock controls.
    //
    // -------------------------------------------------------------------------

    // The button signal is inverted so here we wire up the NOT of the pin
    assign reset = ~inv_reset_pin;

    // Configure the button to use a 10k pullup, and wire it to reset
    SB_IO #(
        .PIN_TYPE('b0000_01),
        .PULLUP(1)
    ) button_pin (
        .PACKAGE_PIN(D4),
        .D_IN_0(inv_reset_pin)
    ) /* synthesis PULLUP_RESISTOR = "10K" */ ;

    // Configure internal HS oscillator to 24MHz and wire it to the clk signal
	SB_HFOSC #(
        .CLKHF_DIV("0b01")
    ) hf_osc (
        .CLKHFEN(1'b1),             // Enable
        .CLKHFPU(1'b1),             // Power up
        .CLKHF(clk)
    ) /* synthesis ROUTE_THROUGH_FABRIC=0 */;

    // The LED is mostly for debugging. Assigned on D3
    reg led = 0;
    assign D3 = led;

    // -------------------------------------------------------------------------
    //
    // I2C controller state machine for handling all communication between the
    // FPGA and camera sensor.
    //
    // -------------------------------------------------------------------------

    // Wire up the the SCL line to D6
    assign D6 = scl;

    // Configure D5 as a tristate and connect it to the sda_in, _out, _oe lines
    SB_IO #(
        .PIN_TYPE('b1010_11),
        .PULLUP(1)
    ) sda_pin (
        .PACKAGE_PIN(D5),
        .OUTPUT_ENABLE(sda_oe),
        .D_OUT_0(sda_out),
        .D_IN_0(sda_in)
    ) /* synthesis PULLUP_RESISTOR = "10K" */ ;

    // Instantiate the 24MHz-800kHz clock divider and output it to i2c_clk
    clock_divider clock_divider (
        .sys_clk(clk),
        .slow_clk(i2c_clk)
    );

    // Instantiate the I2C controller and wire up the signals
    i2c_controller i2c_controller (
        .clk(i2c_clk),
        .reset(reset),
        .idle(idle),
        .ack(ack),
        .nack(nack),
        .address(address),
        .read_write(read_write),
        .transmit_data(transmit_data),
        .received_data(received_data),
        .enable_transfer(enable_transfer),
        .issue_restart(issue_restart),
        .sda_in(sda_in),
        .sda_out(sda_out),
        .sda_oe(sda_oe),
        .scl(scl)
    );

    // I2C registers which we will use to control the camera module
    reg [6:0] address;                      // This is the register address
    reg read_write;                         // Flag for read or write modes
    reg [7:0] transmit_data;                // Data buffer for sending data
    wire [7:0] received_data;               // Data buffer for receiving data
    reg enable_transfer = 0;                // Flag which starts the transfer
    reg issue_restart = 0;                  // Issues restart between transfers

    // The camera EEPROM dump data with calibrations and offsets
    reg [7:0] camera_rom [1663:0];    

    // Raw camera buffer containing uint16 data for each pixel
    reg [7:0] raw_pixel_buffer [1535:0];   

    // Counts how many bytes have been read so far
    integer camera_bytes_read;

    // This flag goes high after every read to trigger the data processing
    reg raw_bytes_ready_to_process = 0;
    
    // I2C controller state variable
    reg [7:0] camera_state = STATE_CAMERA_START;

    // List of states for running the I2C controller logic
    localparam STATE_CAMERA_START                   = 0;

    localparam STATE_SET_CONTROL_REG_ADR_1          = 1;
    localparam STATE_SET_CONTROL_REG_ADR_2          = 2;
    localparam STATE_SET_CONTROL_REG_DATA_1         = 3;
    localparam STATE_SET_CONTROL_REG_DATA_2         = 4;
    localparam STATE_SET_CONTROL_REG_DONE           = 5;

    localparam STATE_SET_CONF_REG_ADR_1             = 10;
    localparam STATE_SET_CONF_REG_ADR_2             = 11;
    localparam STATE_SET_CONF_REG_DATA_1            = 12;
    localparam STATE_SET_CONF_REG_DATA_2            = 13;
    localparam STATE_SET_CONF_REG_DONE              = 14;

    localparam STATE_CAM_READ_ROM_ADR_1             = 20;
    localparam STATE_CAM_READ_ROM_ADR_2             = 21;
    localparam STATE_CAM_READ_ROM_SWITCH_MODE       = 22;
    localparam STATE_CAM_READ_ROM_BYTE_N            = 23;
    localparam STATE_CAM_READ_ROM_INC_N             = 24;
    localparam STATE_CAM_READ_ROM_DONE              = 25;

    localparam STATE_CAM_RESET_STATUS_ADR_1         = 30;
    localparam STATE_CAM_RESET_STATUS_ADR_2         = 31;
    localparam STATE_CAM_RESET_STATUS_DATA_1        = 32;
    localparam STATE_CAM_RESET_STATUS_DATA_2        = 33;
    localparam STATE_CAM_RESET_STATUS_DONE          = 34;

    localparam STATE_CAM_READ_STATUS_ADR_1          = 40;
    localparam STATE_CAM_READ_STATUS_ADR_2          = 41;
    localparam STATE_CAM_READ_STATUS_SWITCH_MODE    = 42;
    localparam STATE_CAM_READ_STATUS_BYTE_1         = 43;
    localparam STATE_CAM_READ_STATUS_BYTE_2         = 44;
    localparam STATE_CAM_CHECK_STATUS               = 45;

    localparam STATE_CAM_READ_PIXELS_ADR_1          = 50;
    localparam STATE_CAM_READ_PIXELS_ADR_2          = 51;
    localparam STATE_CAM_READ_PIXELS_SWITCH_MODE    = 52;
    localparam STATE_CAM_READ_PIXELS_BYTE_N         = 53;
    localparam STATE_CAM_READ_PIXELS_INC_N          = 54;
    localparam STATE_CAM_READ_PIXELS_DONE           = 55;

    localparam STATE_I2C_ERROR                      = 255;

    // Keeps track if I2C ack/nack are rising, falling, high or low
    reg [1:0] i2c_ack_monitor = 0;
    reg [1:0] i2c_nack_monitor = 0;

    // These go high when i2c_ack_monitor or i2c_nack_monitor is a rising edge
    assign i2c_success = i2c_ack_monitor == 'b01 ? 1 : 0;
    assign i2c_failure = i2c_nack_monitor == 'b01 ? 1 : 0;

    // State machine for running the sensor configuration and downloading data
    always @(posedge clk) begin

        // If reset, we start again
        if (reset == 1) begin

            // Bring state back to the start
            camera_state <= STATE_CAMERA_START;

            // Disable transfers
            enable_transfer <= 0;
            issue_restart <= 0;

            // Clear the edge monitors
            i2c_ack_monitor <= 0;
            i2c_nack_monitor <= 0;

            // Clear the counters
            camera_bytes_read <= 0;

            // Clear the process ready flag
            raw_bytes_ready_to_process <= 0;

        end

        // Otherwise run the state machine logic
        else begin

            // Shift the ack and nack into these registers to find the edge state
            i2c_ack_monitor <= {i2c_ack_monitor[0], ack};
            i2c_nack_monitor <= {i2c_nack_monitor[0], nack};

            case (camera_state)

                // Initial settings for the I2C communication to the camera
                // and sending the first address byte of the ROM address
                STATE_CAMERA_START: begin
                    
                    // Chip address of the MLX90640 thermal camera
                    address <= 'h33;
                    
                    // First state is setting the configuration
                    camera_state <= STATE_SET_CONTROL_REG_ADR_1;

                end

                STATE_SET_CONTROL_REG_ADR_1: begin

                    // Write the control register address
                    read_write <= 0;
                    transmit_data <= 'h80; 
                    enable_transfer <= 1;
                    issue_restart <= 0;

                    if_i2c_success(STATE_SET_CONTROL_REG_ADR_2);

                end

                STATE_SET_CONTROL_REG_ADR_2: begin

                    transmit_data <= 'h0D; 

                    if_i2c_success(STATE_SET_CONTROL_REG_DATA_1);

                end

                STATE_SET_CONTROL_REG_DATA_1: begin

                    transmit_data <= 'h19; 

                    if_i2c_success(STATE_SET_CONTROL_REG_DATA_2);

                end

                STATE_SET_CONTROL_REG_DATA_2: begin

                    transmit_data <= 'h01; 

                    if_i2c_success(STATE_SET_CONTROL_REG_DONE);

                end

                STATE_SET_CONTROL_REG_DONE: begin

                    enable_transfer <= 0;
                    issue_restart <= 0;

                    // Once the I2C is idle, we can read the ROM bytes
                    if (idle == 1) camera_state <= STATE_SET_CONF_REG_ADR_1;

                end

                STATE_SET_CONF_REG_ADR_1: begin
                    
                    enable_transfer <= 1;

                    transmit_data <= 'h80; 

                    if_i2c_success(STATE_SET_CONF_REG_ADR_2);

                end
                
                STATE_SET_CONF_REG_ADR_2: begin
                    
                    transmit_data <= 'h0F; 

                    if_i2c_success(STATE_SET_CONF_REG_DATA_1);

                end
                
                STATE_SET_CONF_REG_DATA_1: begin
                    
                    transmit_data <= 'h00; 

                    if_i2c_success(STATE_SET_CONF_REG_DATA_2);

                end
                
                STATE_SET_CONF_REG_DATA_2: begin
                    
                    transmit_data <= 'h00; 

                    if_i2c_success(STATE_SET_CONF_REG_DONE);

                end

                STATE_SET_CONF_REG_DONE: begin

                    enable_transfer <= 0;
                    issue_restart <= 0;

                    // Once the I2C is idle, we can read the ROM bytes
                    if (idle == 1) camera_state <= STATE_CAM_READ_ROM_ADR_1;

                end
                
                STATE_CAM_READ_ROM_ADR_1: begin
                    
                    // Write the first byte of the ROM address
                    read_write <= 0;
                    transmit_data <= 'h24;
                    enable_transfer <= 1;

                    if_i2c_success(STATE_CAM_READ_ROM_ADR_2);

                end

                // Sending the second byte of the camera ROM start address
                STATE_CAM_READ_ROM_ADR_2: begin

                    // Write second byte of the ROM address
                    transmit_data <= 'h00;

                    if_i2c_success(STATE_CAM_READ_ROM_SWITCH_MODE);

                end

                // Switch over to read the camera ROM data
                STATE_CAM_READ_ROM_SWITCH_MODE: begin
                    
                    // Stop transfer and prepare to read the ROM data
                    enable_transfer <= 0;
                    issue_restart <= 1;
                    read_write <= 1;
                    camera_bytes_read <= 0;

                    // Once the I2C is idle, we can read the ROM bytes
                    if (idle == 1) camera_state <= STATE_CAM_READ_ROM_BYTE_N;

                end

                STATE_CAM_READ_ROM_BYTE_N: begin

                    // Read byte by enabling transfer flag
                    enable_transfer <= 1;

                    if (i2c_success == 1) begin
                        camera_state <= STATE_CAM_READ_ROM_INC_N;
                        camera_bytes_read <= camera_bytes_read + 1;
                    end
                    
                    if (i2c_failure == 1) camera_state <= STATE_I2C_ERROR;

                end
                
                STATE_CAM_READ_ROM_INC_N: begin

                    // Save the received data into local memory
                    // Note we incremented before reading so we -1
                    camera_rom[camera_bytes_read - 1] <= received_data;

                    // Read a total of 1664 bytes
                    camera_state <= camera_bytes_read == 1664 
                        ? STATE_CAM_READ_ROM_DONE
                        : STATE_CAM_READ_ROM_BYTE_N;
                    
                end

                STATE_CAM_READ_ROM_DONE: begin

                    // Stop transfer
                    enable_transfer <= 0;
                    issue_restart <= 0;

                    // Now we can go and clear the camera status register which
                    // tells the camera to start the first conversion
                    if (idle == 1) camera_state <= STATE_CAM_RESET_STATUS_ADR_1;

                end

                STATE_CAM_RESET_STATUS_ADR_1: begin
                    
                    read_write <= 0;
                    enable_transfer <= 1;

                    transmit_data <= 'h80; 

                    // Clear the ready flag because we might just have come from
                    // the STATE_CAM_READ_PIXELS_DONE state
                    raw_bytes_ready_to_process <= 0;

                    if_i2c_success(STATE_CAM_RESET_STATUS_ADR_2);

                end

                STATE_CAM_RESET_STATUS_ADR_2: begin

                    transmit_data <= 'h00; 

                    if_i2c_success(STATE_CAM_RESET_STATUS_DATA_1);

                end

                STATE_CAM_RESET_STATUS_DATA_1: begin
                    
                    transmit_data <= 'h00; 

                    if_i2c_success(STATE_CAM_RESET_STATUS_DATA_2);

                end

                STATE_CAM_RESET_STATUS_DATA_2: begin
                    
                    transmit_data <= 'h30; 

                    if_i2c_success(STATE_CAM_RESET_STATUS_DONE);

                end

                STATE_CAM_RESET_STATUS_DONE: begin

                    enable_transfer <= 0;
                    issue_restart <= 0;

                    // Once the I2C is idle, we can read the ROM bytes
                    if (idle == 1) camera_state <= STATE_CAM_READ_STATUS_ADR_1;

                end

                STATE_CAM_READ_STATUS_ADR_1: begin

                    // Write the first byte of the status register address
                    read_write <= 0; 
                    transmit_data <= 'h80;
                    enable_transfer <= 1;

                    if_i2c_success(STATE_CAM_READ_STATUS_ADR_2);
                    
                end

                STATE_CAM_READ_STATUS_ADR_2: begin

                    // Write second byte of the status register address
                    transmit_data <= 'h00;

                    if_i2c_success(STATE_CAM_READ_STATUS_SWITCH_MODE);

                end

                STATE_CAM_READ_STATUS_SWITCH_MODE: begin

                    // Stop transfer and prepare to read 1 byte
                    enable_transfer <= 0;
                    issue_restart <= 1;
                    read_write <= 1;

                    // Once the I2C is idle, we can start the read transaction
                    if (idle == 1) camera_state <= STATE_CAM_READ_STATUS_BYTE_1;

                end

                STATE_CAM_READ_STATUS_BYTE_1: begin

                    // Read bytes by enabling transfer flag
                    enable_transfer <= 1;

                    if_i2c_success(STATE_CAM_READ_STATUS_BYTE_2);

                end

                STATE_CAM_READ_STATUS_BYTE_2: begin
          
                    if_i2c_success(STATE_CAM_CHECK_STATUS);

                end

                STATE_CAM_CHECK_STATUS: begin

                    // Stop the transfer
                    enable_transfer <= 0;
                    issue_restart <= 0;

                    // Wait for idle and before checking the received_data
                    if (idle == 1) begin

                        // If we get the ready flag
                        if (received_data[3] == 1'b1) begin

                            // Wait for DSP operations to complete
                            if (data_processing_completed == 1) begin

                                // Then start reading pixels
                                camera_state <= STATE_CAM_READ_PIXELS_ADR_1;

                            end

                        end

                        // Otherwise keep checking the status register
                        else camera_state <= STATE_CAM_READ_STATUS_ADR_1;
                    
                    end

                end

                STATE_CAM_READ_PIXELS_ADR_1: begin
                
                    // Write the first byte of the frame address
                    read_write <= 0;
                    transmit_data <= 'h04;
                    enable_transfer <= 1;

                    if_i2c_success(STATE_CAM_READ_PIXELS_ADR_2);

                end

                STATE_CAM_READ_PIXELS_ADR_2: begin
                
                    // Write second byte of the frame address
                    transmit_data <= 'h00;

                    if_i2c_success(STATE_CAM_READ_PIXELS_SWITCH_MODE);
                
                end

                STATE_CAM_READ_PIXELS_SWITCH_MODE: begin
                
                    // Stop transfer and prepare to read the frame data
                    enable_transfer <= 0;
                    issue_restart <= 1;
                    read_write <= 1;
                    camera_bytes_read <= 0;

                    // Once the I2C is idle, we can read the ROM bytes
                    if (idle == 1) camera_state <= STATE_CAM_READ_PIXELS_BYTE_N;

                end

                STATE_CAM_READ_PIXELS_BYTE_N: begin
                
                    // Read byte by enabling transfer flag
                    enable_transfer <= 1;

                    if (i2c_success == 1) begin
                        camera_state <= STATE_CAM_READ_PIXELS_INC_N;
                        camera_bytes_read <= camera_bytes_read + 1;
                    end
                    
                    if (i2c_failure == 1) camera_state <= STATE_I2C_ERROR;

                end

                STATE_CAM_READ_PIXELS_INC_N: begin
                
                    // Save the received data into local memory
                    // Note we incremented before reading so we -1
                    raw_pixel_buffer[camera_bytes_read - 1] <= received_data;

                    // Read a total of 1536 bytes
                    camera_state <= camera_bytes_read == 1536
                        ? STATE_CAM_READ_PIXELS_DONE
                        : STATE_CAM_READ_PIXELS_BYTE_N;
                
                end

                STATE_CAM_READ_PIXELS_DONE: begin
                
                    // Stop transfer
                    enable_transfer <= 0;
                    issue_restart <= 0;

                    // Flag that we are ready to start data processing
                    raw_bytes_ready_to_process <= 1;

                    // Once idle, we need to go reset the status register again
                    if (idle == 1) camera_state <= STATE_CAM_RESET_STATUS_ADR_1;

                end

                STATE_I2C_ERROR: begin

                    // Stop the transfer
                    enable_transfer <= 0;
                    issue_restart <= 0;

                end

            endcase

        end

    end

    // Task for catching I2C errors and putting us into the error state
    task if_i2c_success(
        input wire [7:0] next_state
    );

        if (i2c_success == 1) camera_state <= next_state;
        if (i2c_failure == 1) camera_state <= STATE_I2C_ERROR;

    endtask


    // -------------------------------------------------------------------------
    //
    // State machine for processing camera sensor data into the final float 
    // values.
    //
    // -------------------------------------------------------------------------

    // Variables for the data processing and final camera data
    integer bytes_loaded;               // Counter for bytes loaded as int16
    integer bytes_saved;                // Counter for bytes saved as float
    reg data_processing_completed;      // Goes high once data is ready

    // Connect the data process complete flag to the interrupt pin
    assign INT = data_processing_completed;

    // Intermediate registers for the int16 to float conversion module
    reg [15:0] int_data_in;
    reg [31:0] float_data_out;

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
    localparam STATE_DATA_WAIT_FOR_START    = 0;
    localparam STATE_DATA_READ_MSB          = 1;
    localparam STATE_DATA_READ_LSB          = 2;
    localparam STATE_DATA_WRITE_1           = 3;
    localparam STATE_DATA_WRITE_2           = 4;
    localparam STATE_DATA_WRITE_3           = 5;
    localparam STATE_DATA_WRITE_4           = 6;
    localparam STATE_DATA_PROCESSED         = 7;

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

                    // This increment happens first
                    bytes_loaded <= bytes_loaded + 2;

                    // Read -2 to account for that when loading MSB
                    int_data_in[15:8] 
                        <= raw_pixel_buffer[bytes_loaded - 2];

                    data_processor_state <= STATE_DATA_READ_LSB;

                end

                STATE_DATA_READ_LSB: begin
                    
                    // Then load the LSB
                    int_data_in[7:0] 
                        <= raw_pixel_buffer[bytes_loaded - 1];

                    data_processor_state <= STATE_DATA_WRITE_1;

                end

                STATE_DATA_WRITE_1: begin

                    // Increment 4 bytes at a time for floats
                    bytes_saved <= bytes_saved + 4;

                    output_pixel_buffer[bytes_saved - 4] 
                        <= float_data_out[31:24];

                    data_processor_state <= STATE_DATA_WRITE_2;

                end

                STATE_DATA_WRITE_2: begin

                    output_pixel_buffer[bytes_saved - 3] 
                        <= float_data_out[23:16];

                    data_processor_state <= STATE_DATA_WRITE_3;

                end

                STATE_DATA_WRITE_3: begin

                    output_pixel_buffer[bytes_saved - 2] 
                        <= float_data_out[15:8];

                    data_processor_state <= STATE_DATA_WRITE_4;

                end

                STATE_DATA_WRITE_4: begin

                    output_pixel_buffer[bytes_saved - 1] 
                        <= float_data_out[7:0];

                    data_processor_state <= STATE_DATA_PROCESSED;

                end

                STATE_DATA_PROCESSED: begin

                    // Process 1536 bytes for the full buffer
                    if (bytes_loaded < 1536) 
                        data_processor_state <= STATE_DATA_READ_MSB;
                    
                    else
                        data_processor_state <= STATE_DATA_WAIT_FOR_START;

                end
                
            endcase

        end

    end


    // -------------------------------------------------------------------------
    //
    // Output logic for sending the output pixel data to the nRF over SPI.
    //
    // -------------------------------------------------------------------------

    // Variables for connecting the SPI controller to the camera frame buffer
    wire [13:0] spi_data_out_address;
    reg [7:0] spi_data_out;
    
    // Instantiation and connting up the SPI interface to frame buffer memory
    spi_controller spi_controller (
        .clk(clk),
        .sck(SCK),
        .cs(CS),
        .cipo(CIPO),
        .data(spi_data_out),
        .data_address(spi_data_out_address)
    );

    // Always provide the latest data to the SPI controller
    always @(posedge clk) begin
        spi_data_out <= output_pixel_buffer[spi_data_out_address];
    end

endmodule