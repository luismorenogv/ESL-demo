// PWM.v
module PWM #(
  parameter CLK_FREQ    = 50_000_000,
  parameter PWM_FREQ    = 20_000,
  parameter COUNTER_W   = 12  // Adjust resolution as needed
) (
  input  wire                  clk,
  input  wire                  reset,       // active-high
  input  wire                  enable,
  input  wire [COUNTER_W-1:0]  duty_cycle,  // 0 to (2^COUNTER_W–1)
  input  wire                  direction,   // 0=CW, 1=CCW
  output reg                   ina,
  output reg                   inb,
  output reg                   pwm_out
);

  localparam integer PERIOD = CLK_FREQ / PWM_FREQ;
  reg [31:0] counter;

  always @(posedge clk or posedge reset) begin
    if (reset) begin
      counter  <= 0;
      pwm_out  <= 0;
      ina      <= 0;
      inb      <= 0;
    end else if (enable) begin
      // PWM generator
      if (counter < PERIOD-1) counter <= counter + 1;
      else                   counter <= 0;

      pwm_out <= (counter < (duty_cycle * PERIOD) >> COUNTER_W) ? 1 : 0;

      // Direction lines (constant during PWM period)
      ina <= direction;
      inb <= ~direction;
    end else begin
      // disable outputs
      counter <= 0;
      pwm_out <= 0;
      // Brake
      ina     <= 0;
      inb     <= 0;
    end
  end

endmodule