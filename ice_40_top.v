// Include the I2C controller, and clock divider
`include "i2c_controller.v"
`include "clock_divider.v"

module top (
    output wire D3,     // LED on S1 Popout. Used for status
    input wire D4,      // Button on S1 Popout. Used for reset
    output wire D5,      // SDA pin on S1 Popout
    output wire D6     // SCL pin on S1 Popout
);

    // Configure SDA as tristate. No pullup
    SB_IO #(
        .PIN_TYPE(6'b1010_01),
        .PULLUP(1'b0)
    ) tristate_SDA (
        .PACKAGE_PIN(D5),
        .OUTPUT_ENABLE(sda_oe),
        .D_OUT_0(sda_out),
        .D_IN_0(sda_in)
    );

    // Global system clock and reset
    reg clk;
    reg reset;

    // Assign reset to the button
    assign D4 = reset;

    // Local I2C clock wire (400kHz)
    wire i2c_clock;

    // Address, RX and TX  registers
    reg [7:0] address = 8'h52; 
    wire [7:0] received_data;
    reg [7:0] transmit_data = 8'h00; 

    // Flag indicating that the I2C module is busy
    reg busy;

    // Assign busy flag to the status led
    assign D3 = busy;

    // Assign SDA and SCL lines
    assign D6 = scl;

    // TODO configure the pins as pull up and tristate

    // Configure internal HS oscillator as 24MHz
	SB_HFOSC #(.CLKHF_DIV("0b01"))  osc(.CLKHFEN(1'b1), // enable
  										.CLKHFPU(1'b1), // power up
  										.CLKHF(clk)     // output to clk line
                                        ) /* synthesis ROUTE_THROUGH_FABRIC=0 */;

    // Instantiate the 24MHz-800kHz clock divider
    clock_divider clock_divider (
        .sys_clk(clk),
        .slow_clk(i2c_clk)
    );

    // Instantiate the i2c module
    i2c_controller i2c_controller (
        .clk(i2c_clk), // This should be twice the I2C SCL frequency
        .reset(reset),
        .busy(busy),
        .address(address),
        .write_mode(1'b1),
        .transmit_data(transmit_data),
        .received_data(received_data),
        .write_pending(1'b1),
        .read_pending(read_pending),
        .start_transfer(1'b1),
        .sda_in(sda_in),
        .sda_out(sda_out),
        .sda_oe(sda_oe),
        .scl(scl)
    );

endmodule