// PWM_tb.v
`timescale 1ns / 1ps

module PWM_tb;

  // Parameters
  localparam CLK_FREQ = 50_000_000;
  localparam PWM_FREQ = 20_000;
  localparam COUNTER_W = 12;
  
  // Calculate simulation-specific constants
  localparam CLK_PERIOD = 1_000_000_000 / CLK_FREQ; // in ns
  localparam PWM_PERIOD_CYCLES = CLK_FREQ / PWM_FREQ; // 2500 cycles

  // Testbench signals
  reg clk;
  reg reset;
  reg enable;
  reg [COUNTER_W-1:0] duty_cycle;
  reg direction;

  wire ina;
  wire inb;
  wire pwm_out;

  // Instantiate DUT
  PWM #(
    .CLK_FREQ(CLK_FREQ),
    .PWM_FREQ(PWM_FREQ),
    .COUNTER_W(COUNTER_W)
  ) dut (
    .clk(clk),
    .reset(reset),
    .enable(enable),
    .duty_cycle(duty_cycle),
    .direction(direction),
    .ina(ina),
    .inb(inb),
    .pwm_out(pwm_out)
  );

  // Clock Generator
  initial begin
    clk = 0;
    forever #(CLK_PERIOD / 2) clk = ~clk;
  end

  // Test Sequence
  initial begin
    $display("Starting Simple Testbench...");
    // Dump waves for visual inspection
    $dumpfile("pwm_signals.vcd");
    $dumpvars(0, PWM_tb);

    // reset the DUT
    reset = 1;
    enable = 0;
    duty_cycle = 0;
    direction = 0;
    #(CLK_PERIOD * 5); // Wait 5 clock cycles
    reset = 0;
    #(CLK_PERIOD * 5);

    // 1) Disable module
    $display("Test: Module is disabled");
    enable = 0;
    duty_cycle = {COUNTER_W{1'b1}}; // Set to 100% to prove it's off
    #(PWM_PERIOD_CYCLES * CLK_PERIOD * 2); // Wait 2 PWM periods

    // 2) Enable module with 25% duty cycle, CW
    $display("Test: Enable, 25%% duty cycle, CW");
    enable = 1;
    direction = 0; // CW
    duty_cycle = (1 << (COUNTER_W - 2)); // 25% = 1/4 * 2^10 = 1024
    #(PWM_PERIOD_CYCLES * CLK_PERIOD * 3);

    // 3) Enable module with 75% duty cycle, CCW
    $display("Test: 75%% duty cycle, CCW");
    direction = 1; // CCW
    duty_cycle = 3 * (1 << (COUNTER_W - 2)); // 75% = 3/4 * 2^10 = 3072
    #(PWM_PERIOD_CYCLES * CLK_PERIOD * 3);

    // 3) Enable module with 0% duty cycle
    $display("Test: 0%% duty cycle");
    duty_cycle = 0;
    #(PWM_PERIOD_CYCLES * CLK_PERIOD * 2);

    // 4) Enable module with 100% duty cycle
    $display("Test: ~100%% duty cycle");
    duty_cycle = {COUNTER_W{1'b1}}; // 4095
    # (PWM_PERIOD_CYCLES * CLK_PERIOD * 2);

    // 5) Disable module again
    $display("Test: Disable module");
    enable = 0;
    # (PWM_PERIOD_CYCLES * CLK_PERIOD * 2);

    // End of test
    $display("Testbench finished.");
    $finish;
  end

endmodule