/*
 * Bezier Curve Rasterizer — no framebuffer edition
 *
 * Architecture:
 *   - Stores 3 bezier control points (P0, P1, P2).
 *   - Every H-blank period (160 cycles) the 129-step forward-difference
 *     stepper runs and writes any point that lands on the target logical
 *     row into a 160-bit row buffer.
 *   - During active video the row buffer is streamed to the VGA output
 *     (each logical pixel is displayed as a 4x4 VGA block).
 *   - No framebuffer: the curve is re-rendered from scratch every frame.
 *     Only one curve is visible at a time; submitting new points replaces it.
 *
 * Control pins (uio_in used as inputs, uio_oe = 0):
 *   uio_in[0] rising -> latch ui_in[7:0] as staging X
 *   uio_in[1] rising -> latch ui_in[6:0] as staging Y
 *   uio_in[2] rising -> commit staged (X,Y) as next control point (P0->P1->P2->P0)
 *   uio_in[3] rising -> activate curve (start displaying)
 *   uio_in[4] rising -> deactivate curve (blank the screen)
 *
 * Coordinate system: X 0-159, Y 0-119  (160x120 logical canvas, 4x4 per VGA pixel)
 *
 * SPDX-License-Identifier: Apache-2.0
 */

`default_nettype none

module tt_um_bezier_vga (
    input  wire [7:0] ui_in,
    output wire [7:0] uo_out,
    input  wire [7:0] uio_in,
    output wire [7:0] uio_out,
    output wire [7:0] uio_oe,
    input  wire       ena,
    input  wire       clk,
    input  wire       rst_n
);

  // ---------------------------------------------------------------------------
  // VGA sync generator (25 MHz -> 640x480)
  // ---------------------------------------------------------------------------
  wire        hsync, vsync, video_active;
  wire [9:0]  pix_x, pix_y;

  hvsync_generator hvsync_gen (
      .clk        (clk),
      .reset      (~rst_n),
      .hsync      (hsync),
      .vsync      (vsync),
      .display_on (video_active),
      .hpos       (pix_x),
      .vpos       (pix_y)
  );

  reg [1:0] R, G, B;
  assign uo_out  = {hsync, B[0], G[0], R[0], vsync, B[1], G[1], R[1]};
  assign uio_out = 8'b0;
  assign uio_oe  = 8'b0;

  // ---------------------------------------------------------------------------
  // Control pin aliases and edge-detect
  // ---------------------------------------------------------------------------
  wire ctrl_ldx = uio_in[0];
  wire ctrl_ldy = uio_in[1];
  wire ctrl_apt = uio_in[2];
  wire ctrl_sub = uio_in[3];
  wire ctrl_clr = uio_in[4];
  reg  [4:0] prev_ctrl;

  // ---------------------------------------------------------------------------
  // Bezier control points and staging registers
  // ---------------------------------------------------------------------------
  reg [7:0] px [0:2];
  reg [6:0] py [0:2];
  reg [7:0] stg_x;
  reg [6:0] stg_y;
  reg [1:0] pt_idx;
  reg       curve_active;

  // ---------------------------------------------------------------------------
  // Row buffer: 160 bits, one logical row
  // Written during H-blank, read during active video.
  // ---------------------------------------------------------------------------
  reg [159:0] row_buf;

  // ---------------------------------------------------------------------------
  // Forward-difference registers (signed)
  // ---------------------------------------------------------------------------
  reg signed [22:0] x_d0;
  reg signed [17:0] x_d1;
  reg signed [11:0] x_d2;
  reg signed [22:0] y_d0;
  reg signed [17:0] y_d1;
  reg signed [11:0] y_d2;

  // Scan state
  reg       scanning;
  reg [7:0] scan_t;
  reg [6:0] scan_y;

  // ---------------------------------------------------------------------------
  // Combinatorial forward-difference init values
  // ---------------------------------------------------------------------------
  wire signed [17:0] px0e = {10'b0, px[0]};
  wire signed [17:0] px1e = {10'b0, px[1]};
  wire signed [17:0] px2e = {10'b0, px[2]};
  wire signed [17:0] py0e = {11'b0, py[0]};
  wire signed [17:0] py1e = {11'b0, py[1]};
  wire signed [17:0] py2e = {11'b0, py[2]};

  wire signed [17:0] x_d1_init = -(px0e<<8) + px0e + (px1e<<8) - (px1e<<1) + px2e;
  wire signed [17:0] y_d1_init = -(py0e<<8) + py0e + (py1e<<8) - (py1e<<1) + py2e;
  wire signed [11:0] x_d2_init = (px0e[11:0]<<1) - (px1e[11:0]<<2) + (px2e[11:0]<<1);
  wire signed [11:0] y_d2_init = (py0e[11:0]<<1) - (py1e[11:0]<<2) + (py2e[11:0]<<1);

  // ---------------------------------------------------------------------------
  // Current scan pixel position
  // ---------------------------------------------------------------------------
  wire [7:0] x_pix   = x_d0[21:14];
  wire [6:0] y_pix   = y_d0[20:14];
  wire       x_valid = !x_d0[22] && (x_pix <= 8'd159);
  wire       y_match = (y_pix == scan_y);

  // ---------------------------------------------------------------------------
  // Scan trigger
  //
  // Fires at pix_x == 640 (first cycle of H-blank) on:
  //   (a) last VGA row of each logical row: pix_y[1:0] == 3
  //       -> pre-fills row_buf for logical row (pix_y/4)+1
  //   (b) last VGA line of the frame: pix_y == 524
  //       -> pre-fills row_buf for logical row 0 of the next frame
  //
  // This gives 160 H-blank cycles to complete 130 cycles of work (1 init + 129
  // bezier steps), with 30 cycles of margin.
  // ---------------------------------------------------------------------------
  wire trigger = (pix_x == 10'd640) && curve_active &&
                 ((pix_y[1:0] == 2'b11) || (pix_y == 10'd524));

  wire [6:0] trigger_row = (pix_y == 10'd524) ? 7'd0 : pix_y[8:2] + 7'd1;

  // ---------------------------------------------------------------------------
  // VGA display
  // ---------------------------------------------------------------------------
  wire [7:0] lx       = pix_x[9:2];
  wire       pixel_on = video_active && curve_active && row_buf[lx];

  // ---------------------------------------------------------------------------
  // Main clocked process
  // ---------------------------------------------------------------------------
  always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      R <= 2'b00; G <= 2'b00; B <= 2'b00;
      pt_idx       <= 2'd0;
      stg_x        <= 8'd0;  stg_y <= 7'd0;
      px[0] <= 8'd0; px[1] <= 8'd0; px[2] <= 8'd0;
      py[0] <= 7'd0; py[1] <= 7'd0; py[2] <= 7'd0;
      curve_active <= 1'b0;
      row_buf      <= 160'b0;
      scanning     <= 1'b0;
      scan_t       <= 8'd0;
      scan_y       <= 7'd0;
      x_d0 <= 23'sd0; x_d1 <= 18'sd0; x_d2 <= 12'sd0;
      y_d0 <= 23'sd0; y_d1 <= 18'sd0; y_d2 <= 12'sd0;
      prev_ctrl    <= 5'd0;
    end else begin

      prev_ctrl <= {ctrl_clr, ctrl_sub, ctrl_apt, ctrl_ldy, ctrl_ldx};

      // VGA output
      R <= pixel_on ? 2'b11 : 2'b00;
      G <= pixel_on ? 2'b11 : 2'b00;
      B <= pixel_on ? 2'b11 : 2'b00;

      // Control inputs
      if (ctrl_ldx && !prev_ctrl[0]) stg_x <= ui_in;
      if (ctrl_ldy && !prev_ctrl[1]) stg_y <= ui_in[6:0];
      if (ctrl_apt && !prev_ctrl[2]) begin
        px[pt_idx] <= stg_x;
        py[pt_idx] <= stg_y;
        pt_idx <= (pt_idx == 2'd2) ? 2'd0 : pt_idx + 2'd1;
      end
      if (ctrl_sub && !prev_ctrl[3]) curve_active <= 1'b1;
      if (ctrl_clr && !prev_ctrl[4]) curve_active <= 1'b0;

      // Scan trigger: initialise for next logical row
      if (trigger) begin
        row_buf  <= 160'b0;
        x_d0     <= {1'b0,  px[0], 14'b0};
        x_d1     <= x_d1_init;
        x_d2     <= x_d2_init;
        y_d0     <= {2'b0,  py[0], 14'b0};
        y_d1     <= y_d1_init;
        y_d2     <= y_d2_init;
        scan_t   <= 8'd0;
        scan_y   <= trigger_row;
        scanning <= 1'b1;

      // Scan running: one bezier step per cycle
      end else if (scanning) begin
        if (y_match && x_valid)
          row_buf[x_pix] <= 1'b1;

        x_d0 <= x_d0 + {{5{x_d1[17]}}, x_d1};
        x_d1 <= x_d1 + {{6{x_d2[11]}}, x_d2};
        y_d0 <= y_d0 + {{5{y_d1[17]}}, y_d1};
        y_d1 <= y_d1 + {{6{y_d2[11]}}, y_d2};

        if (scan_t == 8'd128) scanning <= 1'b0;
        scan_t <= scan_t + 8'd1;
      end

    end
  end

  wire _unused = &{ena, uio_in[7:5], pix_x[1:0]};

endmodule
