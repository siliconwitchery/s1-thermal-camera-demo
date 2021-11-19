// Include the other modules
`include "spi_controller.v"
`include "i2c_controller.v"
`include "clock_divider.v"

// Multiply this by posedge ticks to get MS delays
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
    output wire D6,     // SCL pin on S1 Popout
    output wire D1,     // Test vector
    output wire D2,
    output wire D7,
    output wire D8
);

    // Test signals
    reg[4:0] test_vector;
    assign D1 = test_vector[0];
    assign D2 = test_vector[1];
    assign D3 = test_vector[2];
    assign D7 = test_vector[3]; 
    assign D8 = test_vector[4];

    // Assign the SCL line to D6
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

    // Connect reset to the inverse of the push button signal. Pull-up required
    assign reset = ~not_reset;

    SB_IO #(
        .PIN_TYPE('b0000_01),
        .PULLUP(1)
    ) button_pin (
        .PACKAGE_PIN(D4),
        .D_IN_0(not_reset)
    ) /* synthesis PULLUP_RESISTOR = "10K" */ ;

    // Configure internal HS oscillator to 24MHz and wire it to the clk signal
	SB_HFOSC #(
        .CLKHF_DIV("0b01")
    ) hf_osc (
        .CLKHFEN(1'b1),             // Enable
        .CLKHFPU(1'b1),             // Power up
        .CLKHF(clk)
    ) /* synthesis ROUTE_THROUGH_FABRIC=0 */;

    // Instantiate the 24MHz-800kHz clock divider and output it to i2c_clk
    clock_divider clock_divider (
        .sys_clk(clk),
        .slow_clk(i2c_clk)
    );

    // Instantiate the I2C controller
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

    // I2C controller registers which we will use to control the camera module
    reg [6:0] address; 
    reg read_write;
    reg [7:0] transmit_data; 
    wire [7:0] received_data; 
    reg enable_transfer;

    // Variables related to camera data
    reg [7:0] camera_rom [1664:0];          // Camera EEPROM data file
    reg [7:0] pixel_buffer [1536:0];        // Complete camera pixel buffer
    integer camera_bytes_read = 0;          // How many bytes read so far
    reg camera_current_page = 0;            // Page 1 or 0 that is being read
    
    // Variables for connecting the SPI controller to the camera frame buffer
    wire [13:0] spi_data_out_address;
    reg [7:0] spi_data_out;

    // Connect the SPI interface to the frame buffer memory
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
        spi_data_out <= camera_rom[spi_data_out_address];
    end

    // General use delay counter
    reg [31:0] delay_ticker = 0;
    
    // List of states for running the system logic
    localparam STATE_START = 0;

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
    localparam STATE_CAM_WAIT_FOR_PAGE              = 46;

    localparam STATE_CAM_READ_PIXELS_ADR_1          = 50;
    localparam STATE_CAM_READ_PIXELS_ADR_2          = 51;
    localparam STATE_CAM_READ_PIXELS_SWITCH_MODE    = 52;
    localparam STATE_CAM_READ_PIXELS_BYTE_N         = 53;
    localparam STATE_CAM_READ_PIXELS_INC_N          = 54;
    localparam STATE_CAM_READ_PIXELS_DONE           = 55;

    localparam STATE_I2C_ERROR                      = 255;

    // Local state variable
    reg [7:0] state = STATE_START;

    // Keeps track if I2C ack/nack are rising, falling, high or low
    reg [1:0] i2c_ack_monitor = 0;
    reg [1:0] i2c_nack_monitor = 0;

    // These go high when i2c_ack_monitor or i2c_nack_monitor is a rising edge
    assign i2c_success = i2c_ack_monitor == 'b01 ? 1 : 0;
    assign i2c_failure = i2c_nack_monitor == 'b01 ? 1 : 0;

    // State machine for running the sensor configuration and read logic
    always @(posedge clk) begin

        // If reset, we start again
        if (reset == 1) begin

            // Bring state back to the start
            state <= STATE_START;

            // Disable transfers
            enable_transfer <= 0;   // Don't transfer yet
            issue_restart <= 0;

            // Clear the edge monitors
            i2c_ack_monitor <= 0;
            i2c_nack_monitor <= 0;

        end

        // Otherwise run the state machine logic
        else begin

            // Shift the ack and nack into these registers to find the edge state
            i2c_ack_monitor <= {i2c_ack_monitor[0], ack};
            i2c_nack_monitor <= {i2c_nack_monitor[0], nack};

            case (state)

                // Initial settings for the I2C communication to the camera
                // and sending the first address byte of the ROM address
                STATE_START: begin
                    
                    // Chip address of the MLX90640 thermal camera
                    address <= 'h33;
                    
                    // First state is setting the configuration
                    state <= STATE_SET_CONTROL_REG_ADR_1;

                end

                STATE_SET_CONTROL_REG_ADR_1: begin

                    // Write the control register address
                    read_write <= 0;
                    transmit_data <= 'h80; 
                    enable_transfer <= 1;

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
                    if (idle == 1) state <= STATE_SET_CONF_REG_ADR_1;

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
                    if (idle == 1) state <= STATE_CAM_READ_ROM_ADR_1;

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
                    if (idle == 1) state <= STATE_CAM_READ_ROM_BYTE_N;

                end

                STATE_CAM_READ_ROM_BYTE_N: begin

                    // Read byte by enabling transfer flag
                    enable_transfer <= 1;

                    if (i2c_success == 1) begin
                        state <= STATE_CAM_READ_ROM_INC_N;
                        camera_bytes_read <= camera_bytes_read + 1;
                    end
                    
                    if (i2c_failure == 1) state <= STATE_I2C_ERROR;

                end
                
                STATE_CAM_READ_ROM_INC_N: begin

                    // Save the received data into local memory
                    camera_rom[camera_bytes_read] <= received_data;

                    // Read a total of 1664 bytes
                    state <= camera_bytes_read == 1664 
                        ? STATE_CAM_READ_ROM_DONE
                        : STATE_CAM_READ_ROM_BYTE_N;
                    
                end

                STATE_CAM_READ_ROM_DONE: begin

                    // Stop transfer
                    enable_transfer <= 0;
                    issue_restart <= 0;

                    // Once I2C is idle again, we can read the status register
                    if (idle == 1) state <= STATE_CAM_RESET_STATUS_ADR_1;

                end

                STATE_CAM_RESET_STATUS_ADR_1: begin
                    
                    read_write <= 0;
                    enable_transfer <= 1;

                    transmit_data <= 'h80; 

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
                    if (idle == 1) state <= STATE_CAM_READ_STATUS_ADR_1;

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
                    if (idle == 1) state <= STATE_CAM_READ_STATUS_BYTE_1;

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
                    
                    // Bit 0 tells us which page was read
                    camera_current_page <= received_data[0];

                    // Reset the delay ticker
                    delay_ticker <= 0;

                    // Wait to be done reading
                    if (idle == 1) begin

                        // Check if read is complete by checking bit 3
                        // only change state once I2C is idle
                        state <= received_data[3] == 1
                            ? STATE_CAM_READ_PIXELS_ADR_1
                            : STATE_CAM_WAIT_FOR_PAGE;

                    end

                end

                STATE_CAM_WAIT_FOR_PAGE: begin

                    // Increment the delay ticker
                    delay_ticker <= delay_ticker + 1;

                    // Wait 1000ms us to check again
                    if (delay_ticker == 1000000 * `US_TICKS) 
                        state <= STATE_CAM_READ_STATUS_ADR_1;
                
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
                    if (idle == 1) state <= STATE_CAM_READ_PIXELS_BYTE_N;

                end

                STATE_CAM_READ_PIXELS_BYTE_N: begin
                
                    // Read byte by enabling transfer flag
                    enable_transfer <= 1;

                    if (i2c_success == 1) begin
                        state <= STATE_CAM_READ_PIXELS_INC_N;
                        camera_bytes_read <= camera_bytes_read + 1;
                    end
                    
                    if (i2c_failure == 1) state <= STATE_I2C_ERROR;

                end

                STATE_CAM_READ_PIXELS_INC_N: begin
                
                    // Save the received data into local memory
                    pixel_buffer[camera_bytes_read] <= received_data;

                    // Read a total of 1536 bytes
                    state <= camera_bytes_read == 1536 
                        ? STATE_CAM_READ_PIXELS_DONE
                        : STATE_CAM_READ_PIXELS_BYTE_N;
                
                end

                STATE_CAM_READ_PIXELS_DONE: begin
                
                    // Stop transfer
                    enable_transfer <= 0;
                    issue_restart <= 0;

                    // Once I2C is idle again, we can read the status register
                    if (idle == 1) state <= STATE_CAM_READ_STATUS_ADR_1;
                
                end

                STATE_I2C_ERROR: begin

                    // Stop the transfer
                    enable_transfer <= 0;
                    issue_restart <= 0;

                end

            endcase

        end

    end

    task if_i2c_success(
        input wire [7:0] next_state
    );

        if (i2c_success == 1) state <= next_state;
        if (i2c_failure == 1) state <= STATE_I2C_ERROR;

    endtask

endmodule