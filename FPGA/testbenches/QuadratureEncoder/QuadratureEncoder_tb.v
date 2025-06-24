// QuadratureEncoder_tb.v
`timescale 1ns / 1ps

module QuadratureEncoder_tb;

  // Parameters
  localparam CLK_FREQ = 50_000_000;
  localparam CLK_PERIOD = 20; // 1_000_000_000 / CLK_FREQ in ns
  localparam IDLE_TIMEOUT_CYCLES = CLK_FREQ / 10; // Cycles for 100ms idle timeout

  // Testbench signals
  reg clk;
  reg reset;
  reg ENCA_raw;
  reg ENCB_raw;

  wire [1:0] DIR;
  wire signed [31:0] position;

  // Instantiate the DUT
  QuadratureEncoder #(
    .CLK_FREQ(CLK_FREQ)
  ) dut (
    .clk(clk),
    .reset(reset),
    .ENCA_raw(ENCA_raw),
    .ENCB_raw(ENCB_raw),
    .DIR(DIR),
    .position(position)
  );

  // Clock generator
  initial begin
    clk = 0;
    forever #(CLK_PERIOD / 2) clk = ~clk;
  end

  // Task to simulate one step of a CW rotation
  task rotate_cw_step;
    begin
      case ({ENCA_raw, ENCB_raw})
        2'b00: {ENCA_raw, ENCB_raw} = 2'b01;
        2'b01: {ENCA_raw, ENCB_raw} = 2'b11;
        2'b11: {ENCA_raw, ENCB_raw} = 2'b10;
        2'b10: {ENCA_raw, ENCB_raw} = 2'b00;
        default: {ENCA_raw, ENCB_raw} = 2'b00;
      endcase
      #(CLK_PERIOD * 5000); // wait between steps
    end
  endtask

  // Task to simulate one step of a CCW rotation
  task rotate_ccw_step;
    begin
      case ({ENCA_raw, ENCB_raw})
        2'b00: {ENCA_raw, ENCB_raw} = 2'b10;
        2'b10: {ENCA_raw, ENCB_raw} = 2'b11;
        2'b11: {ENCA_raw, ENCB_raw} = 2'b01;
        2'b01: {ENCA_raw, ENCB_raw} = 2'b00;
        default: {ENCA_raw, ENCB_raw} = 2'b00;
      endcase
      #(CLK_PERIOD * 5000); // wait between steps
    end
  endtask

  // Test Sequence
  initial begin
    $display("Starting Quadrature Encoder Testbench...");
    // Dump waves for visual inspection
    $dumpfile("quad_signals.vcd");
    $dumpvars(0, QuadratureEncoder_tb);

    // Initial state and reset
    reset = 1;
    ENCA_raw = 0;
    ENCB_raw = 0;
    #(CLK_PERIOD * 10);
    reset = 0;
    #(CLK_PERIOD * 10);

    // 1) Test Clockwise (CW) rotation
    $display("Test: Rotating CW for 10 steps...");
    repeat (10) begin
      rotate_cw_step();
    end
    #(CLK_PERIOD * 10000);

    // 2) Test Counter-Clockwise (CCW) rotation
    $display("Test: Rotating CCW for 15 steps...");
    repeat (15) begin
      rotate_ccw_step();
    end
    #(CLK_PERIOD * 10000);

    // 4) Resume CW rotation from idle
    $display("Test: Resuming CW rotation...");
    repeat (5) begin
      rotate_cw_step();
    end
    #(CLK_PERIOD * 10000);

    // 3) Test idle timeout
    $display("Test: Waiting for idle timeout...");
    #(CLK_PERIOD * 1000000);




    // End of test
    $display("Testbench finished.");
    $finish;
  end

endmodule