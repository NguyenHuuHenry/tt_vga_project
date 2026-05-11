/*
 * Bezier Curve Rasterizer for Tiny Tapeout VGA
 * Inspired by Stampfl 2025 "High-performance 2D graphics rendering on the CPU
 * using sparse strips" — adapted for hardware scanline rasterization.
 *
 * Pipeline:
 *   1. INPUT   – Accept quadratic bezier control points via serial interface
 *   2. FLATTEN – Evaluate bezier at N=128 parametric steps using forward
 *                differences (only additions in inner loop, no multipliers)
 *   3. RASTER  – Plot evaluated pixel positions into 160×120 framebuffer
 *   4. DISPLAY – Stream framebuffer to 640×480 VGA (each logical pixel = 4×4)
 *
 * Control pins (uio_in used as inputs, uio_oe = 0):
 *   uio_in[0] rising edge → latch ui_in[7:0] as X coordinate of staging point
 *   uio_in[1] rising edge → latch ui_in[6:0] as Y coordinate of staging point
 *   uio_in[2] rising edge → commit staging (X,Y) as bezier control point,
 *                           advance point index 0→1→2→0
 *   uio_in[3] rising edge → submit: rasterize current 3 bezier control points
 *   uio_in[4] rising edge → clear framebuffer (takes 2400 cycles)
 *
 * Coordinate system:
 *   X: 0-159 (fits in ui_in[7:0], values 160-255 are out-of-range and clamped)
 *   Y: 0-119 (fits in ui_in[6:0], values 120-127 are out-of-range and clamped)
 *
 * SPDX-License-Identifier: Apache-2.0
 */

`default_nettype none

module tt_um_bezier_vga (
    input  wire [7:0] ui_in,    // data: 8-bit coordinate value
    output wire [7:0] uo_out,   // VGA PMOD output
    input  wire [7:0] uio_in,   // control inputs (see above)
    output wire [7:0] uio_out,  // unused (driven 0)
    output wire [7:0] uio_oe,   // all bidirectional pins as inputs
    input  wire       ena,      // always 1, ignore
    input  wire       clk,      // 25 MHz clock for VGA
    input  wire       rst_n     // active-low reset
);

  // ---------------------------------------------------------------------------
  // VGA sync generator
  // ---------------------------------------------------------------------------
  wire       hsync, vsync, video_active;
  wire [9:0] pix_x, pix_y;

  hvsync_generator hvsync_gen (
      .clk     (clk),
      .reset   (~rst_n),
      .hsync   (hsync),
      .vsync   (vsync),
      .display_on (video_active),
      .hpos    (pix_x),
      .vpos    (pix_y)
  );

  // TinyVGA PMOD: {hsync, B[0], G[0], R[0], vsync, B[1], G[1], R[1]}
  reg [1:0] R, G, B;
  assign uo_out  = {hsync, B[0], G[0], R[0], vsync, B[1], G[1], R[1]};
  assign uio_out = 8'b0;
  assign uio_oe  = 8'b0;

  // ---------------------------------------------------------------------------
  // Control pin aliases
  // ---------------------------------------------------------------------------
  wire ctrl_ldx = uio_in[0];  // load X
  wire ctrl_ldy = uio_in[1];  // load Y
  wire ctrl_apt = uio_in[2];  // add point
  wire ctrl_sub = uio_in[3];  // submit bezier
  wire ctrl_clr = uio_in[4];  // clear framebuffer

  // ---------------------------------------------------------------------------
  // Framebuffer: 160×120 pixels, 1 bit per pixel, packed 8 pixels per byte
  // Layout: row-major, 20 bytes per row × 120 rows = 2400 bytes
  // Byte address = row*20 + col/8,  bit = col%8
  // ---------------------------------------------------------------------------
  reg [7:0] fb [0:2399];

  // ---------------------------------------------------------------------------
  // Bezier control points (P0, P1, P2)
  // ---------------------------------------------------------------------------
  reg [7:0] px [0:2];   // X coords, range 0..159
  reg [6:0] py [0:2];   // Y coords, range 0..119
  reg [7:0] stg_x;      // staging X before commit
  reg [6:0] stg_y;      // staging Y
  reg [1:0] pt_idx;     // which point we are loading (0, 1, 2)

  // ---------------------------------------------------------------------------
  // State machine
  // ---------------------------------------------------------------------------
  localparam ST_CLEAR = 2'd0;  // sequential framebuffer clear
  localparam ST_IDLE  = 2'd1;  // waiting for user input
  localparam ST_INIT  = 2'd2;  // one-cycle forward-difference setup
  localparam ST_RAST  = 2'd3;  // iterate 129 bezier steps (t = 0..128)

  reg [1:0]  state;
  reg [11:0] clear_cnt;  // 0..2399
  reg [7:0]  t;          // bezier parameter step, 0..128

  // ---------------------------------------------------------------------------
  // Forward difference registers (signed)
  //
  // Parameterisation: the bezier is evaluated at t/128 for t = 0, 1, ..., 128.
  // All values are scaled by N² = 128² = 16384 so arithmetic stays integer.
  //
  //   Q(t) * 16384 = s²·P0 + 2st·P1 + t²·P2   where s = 128 − t
  //
  // Forward differences (Δ = difference operator at step h = 1):
  //   d0  = Q(0) * 16384 = P0 * 16384
  //   d1  = ΔQ(0) * 16384 = −255·P0 + 254·P1 + P2     (first diff)
  //   d2  = Δ²Q    * 16384 = 2·(P0 − 2·P1 + P2)        (constant)
  //
  // Iteration: d0 += d1;  d1 += d2;   pixel = d0 >> 14
  //
  // Bit widths:
  //   d0_x: 23-bit signed (max |value| = 159 * 16384 = 2 605 056 < 2²²)
  //   d1_x: 18-bit signed (max |value| = 255*159 ≈ 40 545 < 2¹⁶)
  //   d2_x: 12-bit signed (max |value| = 2*4*159 = 1 272 < 2¹¹)
  //   d0_y: 23-bit signed (max = 119*16384 = 1 949 696 < 2²¹)
  //   d1_y: 17-bit signed (max = 255*119 ≈ 30 345 < 2¹⁵)
  //   d2_y: 11-bit signed (max = 2*4*119 = 952 < 2¹⁰)
  // ---------------------------------------------------------------------------
  reg signed [22:0] x_d0;
  reg signed [17:0] x_d1;
  reg signed [11:0] x_d2;
  reg signed [22:0] y_d0;
  reg signed [17:0] y_d1;
  reg signed [11:0] y_d2;

  // ---------------------------------------------------------------------------
  // Combinatorial: initial forward-difference values from stored control points
  // Computed using only shifts and additions (no multipliers).
  // ---------------------------------------------------------------------------
  wire signed [17:0] px0e = {10'b0, px[0]};  // 18-bit zero-extended, always ≥ 0
  wire signed [17:0] px1e = {10'b0, px[1]};
  wire signed [17:0] px2e = {10'b0, px[2]};
  wire signed [17:0] py0e = {11'b0, py[0]};
  wire signed [17:0] py1e = {11'b0, py[1]};
  wire signed [17:0] py2e = {11'b0, py[2]};

  // d1 = −255·P0 + 254·P1 + P2
  //    = −(P0<<8) + P0 + (P1<<8) − (P1<<1) + P2
  // max |d1_x| = 255*159 ≈ 40545 < 2^17  →  18-bit signed is sufficient
  wire signed [17:0] x_d1_full = -(px0e << 8) + px0e + (px1e << 8) - (px1e << 1) + px2e;
  wire signed [17:0] y_d1_full = -(py0e << 8) + py0e + (py1e << 8) - (py1e << 1) + py2e;

  // d2 = 2·(P0 − 2·P1 + P2) = (P0<<1) − (P1<<2) + (P2<<1)
  // max |d2_x| = 4*2*159 = 1272 < 2^11  →  12-bit signed is sufficient
  wire signed [11:0] x_d2_full = (px0e[11:0] << 1) - (px1e[11:0] << 2) + (px2e[11:0] << 1);
  wire signed [11:0] y_d2_full = (py0e[11:0] << 1) - (py1e[11:0] << 2) + (py2e[11:0] << 1);

  // ---------------------------------------------------------------------------
  // Current bezier pixel position (combinatorial from d0 registers)
  // pixel = d0 >> 14
  // ---------------------------------------------------------------------------
  wire [7:0] x_pix = x_d0[21:14];  // bits 21:14 of 23-bit signed = value/16384
  wire [6:0] y_pix = y_d0[20:14];  // bits 20:14

  // Validity: d0 must be non-negative (sign bit clear) and within display bounds
  wire x_in_range = !x_d0[22] && (x_pix <= 8'd159);
  wire y_in_range = !y_d0[22] && (y_pix <= 7'd119);

  // Framebuffer write address (row*20 + col/8)
  // row*20 = row*16 + row*4 = (row<<4) + (row<<2)
  wire [11:0] fb_waddr = ({5'b0, y_pix} << 4) + ({5'b0, y_pix} << 2)
                         + {7'b0, x_pix[7:3]};
  wire [2:0]  fb_wbit  = x_pix[2:0];

  // ---------------------------------------------------------------------------
  // VGA read path: map 640×480 VGA coordinates to 160×120 framebuffer
  // Each logical pixel is displayed as a 4×4 block (pix_x/4, pix_y/4)
  // ---------------------------------------------------------------------------
  wire [7:0] vga_x = pix_x[9:2];   // pix_x / 4,  range 0..159
  wire [7:0] vga_y = pix_y[9:2];   // pix_y / 4,  range 0..119
  wire [11:0] vga_addr = ({4'b0, vga_y} << 4) + ({4'b0, vga_y} << 2)
                         + {7'b0, vga_x[7:3]};
  wire [2:0]  vga_bit  = vga_x[2:0];

  // ---------------------------------------------------------------------------
  // Edge detection for control inputs (detect rising edges)
  // ---------------------------------------------------------------------------
  reg [4:0] prev_ctrl;

  // ---------------------------------------------------------------------------
  // Main clocked process
  // ---------------------------------------------------------------------------
  always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      state     <= ST_CLEAR;
      clear_cnt <= 12'd0;
      pt_idx    <= 2'd0;
      stg_x     <= 8'd0;
      stg_y     <= 7'd0;
      px[0] <= 8'd0; px[1] <= 8'd0; px[2] <= 8'd0;
      py[0] <= 7'd0; py[1] <= 7'd0; py[2] <= 7'd0;
      x_d0  <= 23'sd0; x_d1 <= 18'sd0; x_d2 <= 12'sd0;
      y_d0  <= 23'sd0; y_d1 <= 18'sd0; y_d2 <= 12'sd0;
      t         <= 8'd0;
      prev_ctrl <= 5'd0;
      R <= 2'b00; G <= 2'b00; B <= 2'b00;
    end else begin

      // -- Edge-detect control signals --
      prev_ctrl <= {ctrl_clr, ctrl_sub, ctrl_apt, ctrl_ldy, ctrl_ldx};

      // -- VGA output (registered, 1-cycle delay is acceptable) --
      if (video_active && fb[vga_addr][vga_bit]) begin
        R <= 2'b11; G <= 2'b11; B <= 2'b11;  // white pixel
      end else begin
        R <= 2'b00; G <= 2'b00; B <= 2'b00;  // black
      end

      // -- State machine --
      case (state)

        // -------- Sequential framebuffer clear --------
        ST_CLEAR: begin
          fb[clear_cnt] <= 8'h00;
          if (clear_cnt == 12'd2399) begin
            clear_cnt <= 12'd0;
            state     <= ST_IDLE;
          end else begin
            clear_cnt <= clear_cnt + 12'd1;
          end
        end

        // -------- Idle: accept control inputs --------
        ST_IDLE: begin
          // Load X coordinate into staging register
          if (ctrl_ldx && !prev_ctrl[0])
            stg_x <= ui_in;

          // Load Y coordinate into staging register
          if (ctrl_ldy && !prev_ctrl[1])
            stg_y <= ui_in[6:0];

          // Commit staged (X, Y) as the next bezier control point
          if (ctrl_apt && !prev_ctrl[2]) begin
            px[pt_idx] <= stg_x;
            py[pt_idx] <= stg_y;
            pt_idx     <= (pt_idx == 2'd2) ? 2'd0 : pt_idx + 2'd1;
          end

          // Submit: begin rasterizing the current bezier
          if (ctrl_sub && !prev_ctrl[3])
            state <= ST_INIT;

          // Clear: wipe framebuffer
          if (ctrl_clr && !prev_ctrl[4]) begin
            clear_cnt <= 12'd0;
            state     <= ST_CLEAR;
          end
        end

        // -------- One-cycle forward-difference initialisation --------
        // d0_x = P0.x << 14     (= P0.x * 16384)
        // d0_y = P0.y << 14
        // d1, d2 from combinatorial wires (all shifts+adds, no multipliers)
        ST_INIT: begin
          x_d0  <= {1'b0, px[0], 14'b0};   // 1+8+14 = 23 bits
          x_d1  <= x_d1_full;
          x_d2  <= x_d2_full;
          y_d0  <= {2'b0, py[0], 14'b0};   // 2+7+14 = 23 bits
          y_d1  <= y_d1_full;
          y_d2  <= y_d2_full;
          t     <= 8'd0;
          state <= ST_RAST;
        end

        // -------- Rasterisation: 129 steps (t = 0 .. 128) --------
        ST_RAST: begin
          // Plot current bezier position into framebuffer
          if (x_in_range && y_in_range)
            fb[fb_waddr] <= fb[fb_waddr] | (8'h01 << fb_wbit);

          // Advance forward differences (explicit sign-extension to silence width warnings)
          x_d0 <= x_d0 + {{5{x_d1[17]}}, x_d1};   // sign-extend 18→23
          x_d1 <= x_d1 + {{6{x_d2[11]}}, x_d2};   // sign-extend 12→18
          y_d0 <= y_d0 + {{5{y_d1[17]}}, y_d1};
          y_d1 <= y_d1 + {{6{y_d2[11]}}, y_d2};
          t    <= t + 8'd1;

          if (t == 8'd128)
            state <= ST_IDLE;
        end

        default: state <= ST_IDLE;
      endcase
    end
  end

  // Suppress unused-signal warnings
  wire _unused = &{ena, uio_in[7:5], pix_x[1:0], pix_y[1:0]};

endmodule
