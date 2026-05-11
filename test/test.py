import cocotb
from cocotb.clock import Clock
from cocotb.triggers import ClockCycles

# VGA timing (640x480 @ 25 MHz, standard porches)
H_DISPLAY = 640
H_FRONT   =  16
H_SYNC    =  96
H_BACK    =  48
V_DISPLAY = 480
V_FRONT   =  10
V_SYNC    =   2
V_BACK    =  33
H_TOTAL   = H_DISPLAY + H_FRONT + H_SYNC + H_BACK   # 800
V_TOTAL   = V_DISPLAY + V_FRONT + V_SYNC + V_BACK    # 525

# Control bit positions in uio_in
BIT_LDX = 0
BIT_LDY = 1
BIT_APT = 2
BIT_SUB = 3
BIT_CLR = 4


async def reset(dut):
    dut.ena.value    = 1
    dut.ui_in.value  = 0
    dut.uio_in.value = 0
    dut.rst_n.value  = 0
    await ClockCycles(dut.clk, 10)
    dut.rst_n.value = 1
    # After reset the design enters ST_CLEAR which takes 2400 cycles
    await ClockCycles(dut.clk, 2410)


async def pulse(dut, bit):
    """Assert a single control bit for one clock cycle then deassert."""
    dut.uio_in.value = (1 << bit)
    await ClockCycles(dut.clk, 1)
    dut.uio_in.value = 0
    await ClockCycles(dut.clk, 1)


async def add_point(dut, x, y):
    """Load X, load Y, then commit the staged point."""
    dut.ui_in.value  = x & 0xFF
    await ClockCycles(dut.clk, 1)
    await pulse(dut, BIT_LDX)

    dut.ui_in.value  = y & 0x7F
    await ClockCycles(dut.clk, 1)
    await pulse(dut, BIT_LDY)

    await pulse(dut, BIT_APT)


async def submit_bezier(dut):
    """Submit the current three control points and wait for rasterization."""
    await pulse(dut, BIT_SUB)
    # ST_INIT: 1 cycle; ST_RAST: 129 steps (t=0..128) + margin
    await ClockCycles(dut.clk, 135)


async def clear_fb(dut):
    await pulse(dut, BIT_CLR)
    await ClockCycles(dut.clk, 2410)


def decode_pixel(uo_out_val):
    """Return True if any colour channel is set (white pixel)."""
    # uo_out = {hsync, B[0], G[0], R[0], vsync, B[1], G[1], R[1]}
    color_bits = uo_out_val & 0b01110111   # mask out hsync/vsync
    return color_bits != 0


async def capture_framebuffer(dut):
    """
    Synchronise to the start of a display frame and read back the 160x120
    logical framebuffer by sampling uo_out on every active pixel.
    Returns fb[y][x] = True/False.
    """
    # Wait for vsync to go low (sync pulse) then come back high (back porch)
    while int(dut.uo_out.value) & 0x08:
        await ClockCycles(dut.clk, 1)
    while not (int(dut.uo_out.value) & 0x08):
        await ClockCycles(dut.clk, 1)
    # Now at start of back porch; skip through it
    await ClockCycles(dut.clk, H_TOTAL * V_BACK)

    fb = [[False] * 160 for _ in range(120)]
    for row in range(V_DISPLAY):
        for col in range(H_TOTAL):
            if col < H_DISPLAY:
                val = int(dut.uo_out.value)
                lx = col >> 2   # col // 4  (4x scale factor)
                ly = row >> 2
                if lx < 160 and ly < 120 and decode_pixel(val):
                    fb[ly][lx] = True
            await ClockCycles(dut.clk, 1)
    return fb


def bezier_reference(p0, p1, p2, steps=129):
    """Software reference: returns set of (x,y) logical pixels for the bezier."""
    pixels = set()
    for i in range(steps):
        t = i / 128.0
        s = 1.0 - t
        x = s*s*p0[0] + 2*s*t*p1[0] + t*t*p2[0]
        y = s*s*p0[1] + 2*s*t*p1[1] + t*t*p2[1]
        xi, yi = int(x), int(y)  # truncation matches hardware's d0>>14 shift
        if 0 <= xi < 160 and 0 <= yi < 120:
            pixels.add((xi, yi))
    return pixels


# ---------------------------------------------------------------------------

@cocotb.test()
async def test_reset_blank(dut):
    """After reset+clear the framebuffer should be all black."""
    clock = Clock(dut.clk, 40, unit="ns")
    cocotb.start_soon(clock.start())
    await reset(dut)

    fb = await capture_framebuffer(dut)
    lit = [(x, y) for y in range(120) for x in range(160) if fb[y][x]]
    assert len(lit) == 0, f"Expected blank framebuffer after reset, found {len(lit)} lit pixels"


@cocotb.test()
async def test_vga_sync_timing(dut):
    """Verify hsync and vsync pulse widths are within VGA spec."""
    clock = Clock(dut.clk, 40, unit="ns")
    cocotb.start_soon(clock.start())
    await reset(dut)

    # Wait for vsync rising edge (end of sync pulse = start of back porch)
    while not (int(dut.uo_out.value) & 0x08):
        await ClockCycles(dut.clk, 1)
    # From start of back porch, skip back porch + display + front porch to reach next sync pulse
    await ClockCycles(dut.clk, H_TOTAL * (V_BACK + V_DISPLAY + V_FRONT))

    vsync = (int(dut.uo_out.value) >> 3) & 1
    assert vsync == 0, "Expected vsync low (active) at start of sync pulse"
    await ClockCycles(dut.clk, H_TOTAL * V_SYNC)
    vsync = (int(dut.uo_out.value) >> 3) & 1
    assert vsync == 1, "Expected vsync high after sync pulse"

    # Spot-check hsync pulse on the next line
    await ClockCycles(dut.clk, H_DISPLAY + H_FRONT)
    hsync = (int(dut.uo_out.value) >> 7) & 1
    assert hsync == 0, "Expected hsync low during sync"
    await ClockCycles(dut.clk, H_SYNC)
    hsync = (int(dut.uo_out.value) >> 7) & 1
    assert hsync == 1, "Expected hsync high after sync"


@cocotb.test()
async def test_straight_line_bezier(dut):
    """Collinear control points should rasterize as a straight horizontal line."""
    clock = Clock(dut.clk, 40, unit="ns")
    cocotb.start_soon(clock.start())
    await reset(dut)

    await add_point(dut, 10, 60)
    await add_point(dut, 80, 60)
    await add_point(dut, 150, 60)
    await submit_bezier(dut)

    fb = await capture_framebuffer(dut)
    ref = bezier_reference((10, 60), (80, 60), (150, 60))
    missing = [p for p in ref if not fb[p[1]][p[0]]]
    assert len(missing) == 0, f"Missing {len(missing)} pixels from straight bezier: {missing[:10]}"


@cocotb.test()
async def test_curved_bezier(dut):
    """A curved bezier should produce pixels within ±1 of the software reference."""
    clock = Clock(dut.clk, 40, unit="ns")
    cocotb.start_soon(clock.start())
    await reset(dut)

    P0, P1, P2 = (5, 5), (80, 110), (155, 5)
    await add_point(dut, *P0)
    await add_point(dut, *P1)
    await add_point(dut, *P2)
    await submit_bezier(dut)

    fb = await capture_framebuffer(dut)
    lit = {(x, y) for y in range(120) for x in range(160) if fb[y][x]}
    ref = bezier_reference(P0, P1, P2)

    def near(px, candidates):
        x, y = px
        return any((x+dx, y+dy) in candidates for dx in range(-1, 2) for dy in range(-1, 2))

    missing = [p for p in ref if not near(p, lit)]
    assert len(missing) < 5, f"{len(missing)} ref pixels have no lit neighbour: {missing[:5]}"


@cocotb.test()
async def test_accumulation(dut):
    """Two submitted bezier curves must both remain visible in the framebuffer."""
    clock = Clock(dut.clk, 40, unit="ns")
    cocotb.start_soon(clock.start())
    await reset(dut)

    await add_point(dut, 5, 5)
    await add_point(dut, 40, 20)
    await add_point(dut, 75, 5)
    await submit_bezier(dut)

    await add_point(dut, 85, 90)
    await add_point(dut, 120, 60)
    await add_point(dut, 155, 90)
    await submit_bezier(dut)

    fb = await capture_framebuffer(dut)
    lit = {(x, y) for y in range(120) for x in range(160) if fb[y][x]}
    ref1 = bezier_reference((5, 5), (40, 20), (75, 5))
    ref2 = bezier_reference((85, 90), (120, 60), (155, 90))

    assert len(lit & ref1) > 0, "No pixels from first curve"
    assert len(lit & ref2) > 0, "No pixels from second curve"


@cocotb.test()
async def test_clear(dut):
    """After drawing a curve and clearing, the framebuffer should be blank."""
    clock = Clock(dut.clk, 40, unit="ns")
    cocotb.start_soon(clock.start())
    await reset(dut)

    await add_point(dut, 10, 10)
    await add_point(dut, 80, 60)
    await add_point(dut, 150, 10)
    await submit_bezier(dut)
    await clear_fb(dut)

    fb = await capture_framebuffer(dut)
    lit = [(x, y) for y in range(120) for x in range(160) if fb[y][x]]
    assert len(lit) == 0, f"Expected blank FB after clear, found {len(lit)} lit pixels"


@cocotb.test()
async def test_out_of_range_safe(dut):
    """Out-of-range control points must not produce any lit pixels."""
    clock = Clock(dut.clk, 40, unit="ns")
    cocotb.start_soon(clock.start())
    await reset(dut)

    await add_point(dut, 200, 127)
    await add_point(dut, 210, 127)
    await add_point(dut, 220, 127)
    await submit_bezier(dut)

    fb = await capture_framebuffer(dut)
    lit = [(x, y) for y in range(120) for x in range(160) if fb[y][x]]
    assert len(lit) == 0, f"Out-of-range bezier produced {len(lit)} lit pixels"
