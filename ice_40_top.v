// Include the I2C controller, and clock divider modules
`include "i2c_controller.v"
`include "clock_divider.v"

module top (
    output wire D3, // LED on S1 Popout. Used for status
    input  wire D4, // Button on S1 Popout. Used for reset
    output wire D5, // SDA pin on S1 Popout
    output wire D6  // SCL pin on S1 Popout
);

    // Assign the SCL line to D6
    assign D6 = scl;

    // Assign the enable flag to the status led
    assign D3 = enable_transfer;

    // Configure SDA as a tristate and connect it to the sda_in, sda_out, and 
    // sda_oe lines. Pull-up is not needed for QWICC/Stemma QT demo boards
    SB_IO #(
        .PIN_TYPE('b1010_01),
    ) sda_pin (
        .PACKAGE_PIN(D5),
        .OUTPUT_ENABLE(sda_oe),
        .D_OUT_0(sda_out),
        .D_IN_0(sda_in)
    );

    // Connect D4 to the invert line and invert it. A pull-up is required here
    SB_IO #(
        .PIN_TYPE('b0000_01),
        .PULLUP(1)
    ) button_pin (
        .PACKAGE_PIN(D4),
        .D_IN_0(!reset)
    ) /* synthesis PULLUP_RESISTOR = "10K" */ ;

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
    reg enable_transfer;
    
    // List of states for running the sensor logic
    localparam STATE_IDLE = 0; //
    localparam STATE_1 = 1;   //
    localparam STATE_2 = 2;   //

    // Local state variable
    reg [4:0] state = STATE_IDLE;

    // State machine for running the sensor configuration and read logic
    always @(posedge clk || reset) begin
        
        // If reset, we start again
        if (reset == 1) begin
            state <= STATE_IDLE;
        end

        // Otherwise run the state machine logic
        else begin

            case (state)

                STATE_IDLE: begin
                    
                    state <= STATE_1;
                end

                STATE_1: begin
                end

                STATE_2: begin
                end

            endcase
        end

    end

endmodule