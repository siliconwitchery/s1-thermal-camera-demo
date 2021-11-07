// Include the I2C controller, and clock divider
`include "i2c_controller.v"
`include "clock_divider.v"

module top (
    output wire D3,     // LED on S1 Popout. Used for status
    input wire D4,      // Button on S1 Popout. Used for reset
    inout wire D5;      // SDA pin on S1 Popout
    output wire D6;     // SCL pin on S1 Popout
)

    // Global system clock and reset
    reg clk;
    reg reset;

    // Assign reset to the button
    assign D4 = reset;

    // Local I2C clock wire (400kHz)
    wire i2c_clock;

    // Address, RX and TX  registers
    reg [7:0] address = 8'h9B; 
    wire [7:0] received_data;
    reg [7:0] transmit_data = 8'hAA; 

    // Flag indicating that the I2C module is busy
    reg busy;

    // Assign busy flag to the status led
    assign D3 = busy;

    // TODO configure the pins as pull up and tristate

    // Configure internal HS oscillator as 6MHz
	SB_HFOSC #(.CLKHF_DIV("0b11"))  osc(.CLKHFEN(1'b1), // enable
  										.CLKHFPU(1'b1), // power up
  										.CLKHF(clk)     // output to clk line
                                        ) /* synthesis ROUTE_THROUGH_FABRIC=0 */;

    // Instantiate the 50MHz-400kHz clock divider
    clock_divider clock_divider (
        .sys_clk(clk),
        .slow_clk(i2c_clk)
    );

    // Instantiate the unit under test
    i2c_controller i2c_controller (
        .clk(i2c_clk),
        .reset(reset),
        .busy(busy),
        .address(address),
        .write_mode(write_mode),
        .transmit_data(transmit_data),
        .received_data(received_data),
        .scl(scl),
        .sda(sda)
    );

endmodule