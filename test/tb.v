`default_nettype none
`timescale 1ns / 1ps

/* This testbench instantiates the top module (8x13-bit ECC-Protected Scrubbing Memory)
   and exposes the standard Tiny Tapeout ports to Cocotb (test_scrub_mem.py).
*/
module tb ();

  // Dump signals to an FST file for viewing in GTKWave or Surfer.
  initial begin
    $dumpfile("tb.fst");
    $dumpvars(0, tb);
    #1;
  end

  // Wire up the inputs and outputs:
  reg clk;
  reg rst_n;
  reg ena;
  reg [7:0] ui_in;
  reg [7:0] uio_in;
  wire [7:0] uo_out;
  wire [7:0] uio_out;
  wire [7:0] uio_oe;
`ifdef GL_TEST
  wire VPWR = 1'b1;
  wire VGND = 1'b0;
`endif

  tt_um_ecc_prot_scrub_mem user_project (

      // Include power ports for the Gate Level test:
`ifdef GL_TEST
      .VPWR(VPWR),
      .VGND(VGND),
`endif

      .ui_in  (ui_in),    // Write data[7:0] / Inject Fault {3'b0, err_double, err_pos[3:0]} / CSR scrub_en (bit 0)
      .uo_out (uo_out),   // Healed read data[7:0] or Telemetry byte[7:0]
      .uio_in (uio_in),   // [2:0] addr / [3:0] telem_ctrl, [5:4] cmd
      .uio_out(uio_out),  // [6] stall, [7] err_alert (sticky)
      .uio_oe (uio_oe),   // Enable path (8'b1100_0000)
      .ena    (ena),      // Enable - goes high when design is selected
      .clk    (clk),      // Clock
      .rst_n  (rst_n)     // Active-low synchronous reset
  );

endmodule
