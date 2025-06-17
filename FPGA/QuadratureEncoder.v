// QuadratureEncoder.v
module QuadratureEncoder(
  input  wire        clk,
  input  wire        reset,            // asynchronous active-high reset
  input  wire        ENCA_raw,         // raw encoder channel A
  input  wire        ENCB_raw,         // raw encoder channel B
  output reg [1:0]   DIR,              // direction: 01=CW, 11=CCW, 00=idle, 10=hold
  output reg signed [31:0] position    // signed position count (increment/decrement)
);

  //—————————— PARAMETERS ——————————
  parameter integer CLK_FREQ              = 50_000_000;         // 50 MHz board clock
  parameter integer NO_MOVEMENT_THRESHOLD = CLK_FREQ / 10;      // ≈100 ms

  //————————— INPUT SYNCHRONIZATION —————————
  // Two-stage synchronizers for each external line
  reg [1:0] syncA = 2'b00, syncB = 2'b00;
  always @(posedge clk) begin
    syncA <= {syncA[0], ENCA_raw};
    syncB <= {syncB[0], ENCB_raw};
  end
  wire encA = syncA[1];
  wire encB = syncB[1];

  //————————— EDGE DETECTION STATE —————————
  reg [1:0] old_enc = 2'b00;
  wire [1:0] enc = {encA, encB};
  wire change = (enc != old_enc);

  //————————— DIRECTION LOOKUP —————————
  // A 2-bit code: 01 = CW, 11 = CCW, 00 = no move
function [1:0] code_lut(input [1:0] new_enc, input [1:0] old_enc, input [1:0] old_dir);
  reg [1:0] lookup;
  begin
    case ({new_enc, old_enc})
      4'b0001,
      4'b0111,
      4'b1000,
      4'b1110: lookup = 2'b01;  // CW
      4'b0010,
      4'b0100,
      4'b1011,
      4'b1101: lookup = 2'b11;  // CCW
      4'b0000,
      4'b0101,
      4'b1010,
      4'b1111: lookup = 2'b00;  // no movement
      default: lookup = old_dir; // “maintain” for all illegal/hold combos
    endcase
    code_lut = lookup;
  end
endfunction

  //————————— IDLE COUNTER —————————
  reg [31:0] idle_cnt = 0;
  reg [1:0] next_dir;

  //————————— MAIN SEQUENTIAL LOGIC —————————
  always @(posedge clk or posedge reset) begin
    if (reset) begin
      old_enc   <= 2'b00;
      DIR       <= 2'b00;
      position  <= 32'sd0;
      idle_cnt  <= 0;
    end else begin
      if (change) begin
        idle_cnt <= 0;
        // Compute next_dir only once
        next_dir = code_lut(enc, old_enc, DIR);
        DIR      <= next_dir;

        // Update position based on next_dir
        if      (next_dir == 2'b01) position <= position + 1;
        else if (next_dir == 2'b11) position <= position - 1;

        old_enc <= enc;
      end else if (idle_cnt < NO_MOVEMENT_THRESHOLD) begin
        idle_cnt <= idle_cnt + 1;
        if (idle_cnt + 1 >= NO_MOVEMENT_THRESHOLD)
          DIR <= 2'b00;  // idle
      end
    end
  end

endmodule