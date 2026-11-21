# SPDX-FileCopyrightText: © 2026 Tejas Patel, Pandit Deendayal Energy University
# SPDX-License-Identifier: Apache-2.0

import cocotb
from cocotb.clock import Clock
from cocotb.triggers import ClockCycles, FallingEdge, RisingEdge


EXPECTED_UIO_OE = 0b1110_0000


def decode_uio_out(uio_out_val: int):
    """Extract flags and verify unused lower 5 output bits remain 0."""
    unused_low = uio_out_val & 0x1F
    sec = (uio_out_val >> 5) & 1
    ded = (uio_out_val >> 6) & 1
    ok  = (uio_out_val >> 7) & 1
    return unused_low, sec, ded, ok


def secded_golden_model(data_in: int, err_pos: int, err_double: int):
    """Independent golden reference model for Hamming(13,8) SECDED + fault injector."""
    d = [(data_in >> i) & 1 for i in range(8)]
    p1 = d[0] ^ d[1] ^ d[3] ^ d[4] ^ d[6]
    p2 = d[0] ^ d[2] ^ d[3] ^ d[5] ^ d[6]
    p4 = d[1] ^ d[2] ^ d[3] ^ d[7]
    p8 = d[4] ^ d[5] ^ d[6] ^ d[7]

    bits = [0, p1, p2, d[0], p4, d[1], d[2], d[3], p8, d[4], d[5], d[6], d[7]]
    p0 = 0
    for b in bits[1:]:
        p0 ^= b
    bits[0] = p0

    code = 0
    for i, b in enumerate(bits):
        code |= (b << i)

    # Fault mask matching specification
    if 1 <= err_pos <= 12:
        mask = (1 << (err_pos - 1)) | ((1 << err_pos) if err_double else 0)
    elif err_pos == 13:
        mask = (1 << 12) | (1 if err_double else 0)
    else:  # err_pos in {0, 14, 15}
        mask = 0x0003 if err_double else 0

    corrupted = code ^ mask

    # Independent syndrome and overall parity calculation
    c = [(corrupted >> i) & 1 for i in range(13)]
    s0 = c[1] ^ c[3] ^ c[5] ^ c[7] ^ c[9]  ^ c[11]
    s1 = c[2] ^ c[3] ^ c[6] ^ c[7] ^ c[10] ^ c[11]
    s2 = c[4] ^ c[5] ^ c[6] ^ c[7] ^ c[12]
    s3 = c[8] ^ c[9] ^ c[10] ^ c[11] ^ c[12]
    syndrome = (s3 << 3) | (s2 << 2) | (s1 << 1) | s0

    p_tot = 0
    for b in c:
        p_tot ^= b

    sec = p_tot
    ded = 1 if (p_tot == 0 and syndrome != 0) else 0
    ok  = 1 if (p_tot == 0 and syndrome == 0) else 0

    # Apply correction ONLY on single-bit error (never miscorrect on DED)
    fixed = corrupted ^ ((1 << syndrome) if (sec and syndrome <= 12) else 0)
    data_out = (
        (((fixed >> 12) & 1) << 7) |
        (((fixed >> 11) & 1) << 6) |
        (((fixed >> 10) & 1) << 5) |
        (((fixed >> 9)  & 1) << 4) |
        (((fixed >> 7)  & 1) << 3) |
        (((fixed >> 6)  & 1) << 2) |
        (((fixed >> 5)  & 1) << 1) |
        (((fixed >> 3)  & 1) << 0)
    )
    return data_out, sec, ded, ok


async def init_dut(dut):
    """Start 25 MHz clock and apply clean synchronous reset."""
    clock = Clock(dut.clk, 40, units="ns")
    cocotb.start_soon(clock.start())

    dut.ena.value = 1
    dut.ui_in.value = 0
    dut.uio_in.value = 0
    dut.rst_n.value = 0
    await ClockCycles(dut.clk, 3)


@cocotb.test()
async def test_reset_and_pin_directions(dut):
    """Verify uio_oe direction mask and synchronous reset state."""
    await init_dut(dut)
    await FallingEdge(dut.clk)

    assert int(dut.uio_oe.value) == EXPECTED_UIO_OE, (
        f"uio_oe mismatch: got {int(dut.uio_oe.value):#010b}, exp {EXPECTED_UIO_OE:#010b}"
    )
    assert int(dut.uo_out.value) == 0, f"uo_out not 0 during reset: {int(dut.uo_out.value):#x}"
    assert int(dut.uio_out.value) == 0, f"uio_out not 0 during reset: {int(dut.uio_out.value):#x}"

    # Drive non-zero inputs while still in reset; outputs must stay 0
    dut.ui_in.value = 0xA5
    dut.uio_in.value = 0x05
    await RisingEdge(dut.clk)
    await FallingEdge(dut.clk)
    assert int(dut.uo_out.value) == 0, "uo_out changed while rst_n=0"
    assert int(dut.uio_out.value) == 0, "uio_out changed while rst_n=0"


@cocotb.test()
async def test_exhaustive_truth_table_streaming(dut):
    """
    Exhaustively verify ALL 8,192 combinations (256 data bytes x 32 fault states)
    streamed back-to-back at 1 vector per clock cycle to verify exact 1-cycle latency.
    """
    await init_dut(dut)
    await FallingEdge(dut.clk)
    dut.rst_n.value = 1

    # Build full 8,192-vector list: (data_in, err_pos, err_double)
    vectors = [
        (data, err_pos, err_double)
        for err_double in (0, 1)
        for err_pos in range(16)
        for data in range(256)
    ]

    # Prime first vector on FallingEdge
    d0, p0, dbl0 = vectors[0]
    dut.ui_in.value = d0
    dut.uio_in.value = (dbl0 << 4) | p0

    for idx in range(len(vectors)):
        # Wait for DUT to register vectors[idx], then sample at FallingEdge
        await RisingEdge(dut.clk)
        await FallingEdge(dut.clk)

        data_in, err_pos, err_double = vectors[idx]
        exp_data, exp_sec, exp_ded, exp_ok = secded_golden_model(data_in, err_pos, err_double)

        got_data = int(dut.uo_out.value)
        got_uio  = int(dut.uio_out.value)
        got_oe   = int(dut.uio_oe.value)
        unused_low, got_sec, got_ded, got_ok = decode_uio_out(got_uio)

        # 1. Pin direction & unused output checks
        assert got_oe == EXPECTED_UIO_OE, f"uio_oe drifted: {got_oe:#010b}"
        assert unused_low == 0, f"uio_out[4:0] non-zero: {unused_low:#07b}"

        # 2. Strict one-hot flag mutual exclusivity
        assert (got_sec + got_ded + got_ok) == 1, (
            f"Flags not one-hot for data={data_in:#x} pos={err_pos} dbl={err_double}: "
            f"sec={got_sec} ded={got_ded} ok={got_ok}"
        )

        # 3. Specification-level behavioral assertions
        if err_double == 0 and err_pos in (0, 14, 15):
            assert got_data == data_in and (got_sec, got_ded, got_ok) == (0, 0, 1), (
                f"Zero-error edge case failed: data={data_in:#x} pos={err_pos}"
            )
        elif err_double == 0 and 1 <= err_pos <= 13:
            assert got_data == data_in and (got_sec, got_ded, got_ok) == (1, 0, 0), (
                f"SEC failed: data={data_in:#x} pos={err_pos} got={got_data:#x}"
            )
        else:
            assert (got_sec, got_ded, got_ok) == (0, 1, 0), (
                f"DED flag failed: data={data_in:#x} pos={err_pos}"
            )

        # 4. Bit-exact golden model check (verifies DED does not miscorrect data_out)
        assert (got_data, got_sec, got_ded, got_ok) == (exp_data, exp_sec, exp_ded, exp_ok), (
            f"Golden mismatch at data={data_in:#x} pos={err_pos} dbl={err_double}: "
            f"got=({got_data:#x},{got_sec},{got_ded},{got_ok}) "
            f"exp=({exp_data:#x},{exp_sec},{exp_ded},{exp_ok})"
        )

        # Drive next vector immediately for back-to-back 1-cycle latency verification
        if idx + 1 < len(vectors):
            dn, pn, dbln = vectors[idx + 1]
            dut.ui_in.value = dn
            dut.uio_in.value = (dbln << 4) | pn

    dut._log.info(f"Exhaustive streaming test passed all {len(vectors)} combinations!")


@cocotb.test()
async def test_mid_operation_reset_and_recovery(dut):
    """Assert rst_n=0 mid-stream during active SEC/DED states and confirm clean recovery."""
    await init_dut(dut)
    await FallingEdge(dut.clk)
    dut.rst_n.value = 1

    # Drive an active SEC state
    dut.ui_in.value = 0x5A
    dut.uio_in.value = 0x07  # err_pos = 7, err_double = 0
    await RisingEdge(dut.clk)
    await FallingEdge(dut.clk)
    assert int(dut.uo_out.value) == 0x5A
    assert decode_uio_out(int(dut.uio_out.value)) == (0, 1, 0, 0)

    # Assert synchronous reset mid-operation
    dut.rst_n.value = 0
    await RisingEdge(dut.clk)
    await FallingEdge(dut.clk)
    assert int(dut.uo_out.value) == 0x00, "Mid-op reset failed to clear uo_out"
    assert int(dut.uio_out.value) == 0x00, "Mid-op reset failed to clear uio_out"

    # Release reset and immediately feed a DED vector -> verify 1-cycle recovery
    dut.rst_n.value = 1
    dut.ui_in.value = 0xC3
    dut.uio_in.value = 0x13  # err_pos = 3, err_double = 1
    await RisingEdge(dut.clk)
    await FallingEdge(dut.clk)
    exp_data, exp_sec, exp_ded, exp_ok = secded_golden_model(0xC3, 3, 1)
    assert int(dut.uo_out.value) == exp_data
    assert decode_uio_out(int(dut.uio_out.value)) == (0, exp_sec, exp_ded, exp_ok)


@cocotb.test()
async def test_unused_uio_inputs_immunity(dut):
    """Ensure toggling upper uio_in[7:5] (configured as outputs) has zero effect."""
    await init_dut(dut)
    await FallingEdge(dut.clk)
    dut.rst_n.value = 1

    for upper in range(8):
        for data_in, err_pos, err_double in [(0x3C, 0, 0), (0xA5, 9, 0), (0xFF, 13, 1)]:
            dut.ui_in.value = data_in
            dut.uio_in.value = (upper << 5) | (err_double << 4) | err_pos
            await RisingEdge(dut.clk)
            await FallingEdge(dut.clk)

            exp_data, exp_sec, exp_ded, exp_ok = secded_golden_model(data_in, err_pos, err_double)
            assert int(dut.uo_out.value) == exp_data
            assert decode_uio_out(int(dut.uio_out.value)) == (0, exp_sec, exp_ded, exp_ok)
