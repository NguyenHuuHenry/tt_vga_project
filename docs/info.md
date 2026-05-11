## How it works

This design is a quadratic bezier curve rasterizer for VGA output.

Control points are loaded via the bidirectional pins (`uio_in`) and `ui_in`, using a simple rising-edge
protocol:

1. Put the X coordinate (0–159) on `ui_in[7:0]` and pulse `uio_in[0]` (Load X).
2. Put the Y coordinate (0–119) on `ui_in[6:0]` and pulse `uio_in[1]` (Load Y).
3. Pulse `uio_in[2]` (Add Point) to commit the staged (X, Y) as the next bezier control point.
   Repeat steps 1–3 for all three control points (P0, P1, P2).
4. Pulse `uio_in[3]` (Submit) to rasterize the quadratic bezier defined by the three control points.
5. Pulse `uio_in[4]` (Clear) at any time to wipe the framebuffer (takes ~2400 cycles).

Multiple bezier curves can be accumulated onto the same frame before clearing.

### Internal pipeline

| Stage | Detail |
|-------|--------|
| **INPUT** | Accept control points via serial pulse protocol on `uio_in` |
| **INIT** | Compute forward-difference seed values from the three control points (shifts + adds only, no multipliers) |
| **RASTER** | Iterate 129 parametric steps using forward differences (`d0 += d1; d1 += d2`) — inner loop is 4 additions per pixel |
| **DISPLAY** | Stream the 160×120 packed framebuffer to 640×480 VGA (each logical pixel displayed as a 4×4 block) |

The framebuffer is 160×120 pixels, 1 bit per pixel, packed 8 pixels per byte = 2400 bytes total.
The 640×480 VGA output uses a 25 MHz pixel clock, standard HSYNC/VSYNC timing, and the TinyVGA PMOD
pin mapping.

## How to test

Connect a TinyVGA PMOD to the output pins. Drive control points using the DIP switches on `ui_in` and
toggle switches on `uio_in[4:0]`.

For simulation, run the cocotb test suite from the `test/` directory:

```sh
make -B
```

The tests cover: reset/clear behaviour, VGA sync timing, straight and curved bezier rasterization
accuracy, curve accumulation, and out-of-range coordinate clamping.

## External hardware

- **TinyVGA PMOD** connected to `uo_out` — required for VGA output
- VGA monitor or capture device accepting 640×480 @ 60 Hz
