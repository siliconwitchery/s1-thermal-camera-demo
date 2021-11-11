// Include the I2C controller, and clock divider modules
`include "i2c_controller.v"
`include "clock_divider.v"

module top (
    output wire D3, // LED on S1 Popout. Used for status
    input  wire D4, // Button on S1 Popout. Used for reset
    output  wire D5, // SDA pin on S1 Popout
    output wire D6  // SCL pin on S1 Popout
);

    // Assign the SCL line to D6
    assign D6 = scl;

    // Assign the enable flag to the status led
    assign D3 = enable_transfer;

    // Configure SDA as a tristate and connect it to the sda_in, sda_out, and 
    // sda_oe lines. Pull-up is not needed for QWICC/Stemma QT demo boards
    SB_IO #(
        .PIN_TYPE('b1010_11),
        .PULLUP(1)
    ) sda_pin (
        .PACKAGE_PIN(D5),
        .OUTPUT_ENABLE(sda_oe),
        .D_OUT_0(sda_out),
        .D_IN_0(sda_in)
    ) /* synthesis PULLUP_RESISTOR = "10K" */ ;

    // Connect D4 to the invert line and invert it. A pull-up is required here
    SB_IO #(
        .PIN_TYPE('b0000_01),
        .PULLUP(1)
    ) button_pin (
        .PACKAGE_PIN(D4),
        .D_IN_0(not_reset)
    ) /* synthesis PULLUP_RESISTOR = "10K" */ ;

    assign reset = ~not_reset;

    // Configure internal HS oscillator as 24MHz
	SB_HFOSC #(.CLKHF_DIV("0b01"))  osc(.CLKHFEN(1'b1), // enable
  										.CLKHFPU(1'b1), // power up
  										.CLKHF(clk)     // output to clk line
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
        .sda_in(sda_in),
        .sda_out(sda_out),
        .sda_oe(sda_oe),
        .scl(scl)
    );

    // I2C controller registers which we will use to control the module
    reg [6:0] address; 
    reg read_write;
    reg [7:0] transmit_data; 
    wire [7:0] received_data; 
    reg enable_transfer;
    
    // List of states for running the sensor logic
    localparam STATE_0 = 1; //
    localparam STATE_1 = 2;   //
    localparam STATE_2 = 3;   //
    localparam STATE_3 = 4;   //
    localparam STATE_4 = 5;   //
    localparam STATE_5 = 6;   //
    localparam STATE_ERROR = 100; //

    // Local state variable
    reg [8:0] state = STATE_0;

    // Keeps track if ack are nack are rising edge, falling edge, high or low
    reg [1:0] ack_monitor = 0;
    reg [1:0] nack_monitor = 0;

    // These go high when ack_monitor or nack_monitor is a rising edge
    assign success = ack_monitor == 'b01 ? 1 : 0;
    assign failure = nack_monitor == 'b01 ? 1 : 0;

    // State machine for running the sensor configuration and read logic
    always @(posedge clk) begin

        // If reset, we start again
        if (reset == 1) begin

            // Bring state back to the start
            state <= STATE_0;

            // Disable transfers
            enable_transfer <= 0;   // Don't transfer yet

            // Clear the edge monitors
            ack_monitor <= 0;
            nack_monitor <= 0;

        end

        // Otherwise run the state machine logic
        else begin

            // Shift the ack and nack into these registers to find the edge state
            ack_monitor <= {ack_monitor[0], ack};
            nack_monitor <= {nack_monitor[0], nack};

            case (state)

                STATE_0: begin

                    // address <= 'h29;        // Address for the VL6180X sensor
                    address <= 'h33;        // Address for the MLX90640 sensor
                    read_write = 0;         // Write mode for sending address
                    transmit_data <= 'h24;  // Device ID register first half
                    enable_transfer <= 1;   // Start transfer

                    if (success == 1) state <= STATE_1;
                    if (failure == 1) state <= STATE_ERROR;

                end

                STATE_1: begin

                    transmit_data <= 'h07;  // Device ID register second half

                    if (success == 1) state <= STATE_2;
                    if (failure == 1) state <= STATE_ERROR;

                end

                STATE_2: begin
                    
                    enable_transfer <= 0;   // Stop transfer
                    
                    if (idle == 1) state <= STATE_3;

                end

                STATE_3: begin

                    read_write = 1;         // Read mode to get device ID
                    enable_transfer <= 1;   // Start transfer

                    if (success == 1) state <= STATE_4;
                    if (failure == 1) state <= STATE_ERROR;

                end

                STATE_4: begin

                    // Read again
                
                    if (success == 1) state <= STATE_5;
                    if (failure == 1) state <= STATE_ERROR;
                
                end

                STATE_5: begin

                    enable_transfer <= 0;   // Stop transfer

                end

                STATE_ERROR: begin

                    enable_transfer <= 0;   // Stop transfer

                end

            endcase

        end

    end

endmodule