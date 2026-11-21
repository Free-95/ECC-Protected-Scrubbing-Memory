# SPDX-FileCopyrightText: © 2026 Tejas Patel, Pandit Deendayal Energy University
# SPDX-License-Identifier: Apache-2.0

import random
import cocotb
from cocotb.clock import Clock
from cocotb.triggers import ClockCycles, FallingEdge, RisingEdge


EXPECTED_UIO_OE = 0b1000_0000

CMD_READ   = 0b00
CMD_WRITE  = 0b01
CMD_INJECT = 0b10
CMD_TELEM  = 0b11


# -----------------------------------------------------------------------------
# Independent Cycle-Accurate Golden Reference Model
# -----------------------------------------------------------------------------
def encode_hamming13(data_in: int) -> int:
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
    return code


def inject_fault(code_in: int, err_pos: int, err_double: int) -> int:
    if 1 <= err_pos <= 12:
        mask = (1 << (err_pos - 1)) | ((1 << err_pos) if err_double else 0)
    elif err_pos == 13:
        mask = (1 << 12) | (1 if err_double else 0)
    else:  # 0, 14, 15
        mask = 0x0003 if err_double else 0
    return code_in ^ mask


def decode_secded(code_in: int):
    c = [(code_in >> i) & 1 for i in range(13)]
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

    corrected = code_in ^ ((1 << syndrome) if (sec and syndrome <= 12) else 0)
    data_out = (
        (((corrected >> 12) & 1) << 7) |
        (((corrected >> 11) & 1) << 6) |
        (((corrected >> 10) & 1) << 5) |
        (((corrected >> 9)  & 1) << 4) |
        (((corrected >> 7)  & 1) << 3) |
        (((corrected >> 6)  & 1) << 2) |
        (((corrected >> 5)  & 1) << 1) |
        (((corrected >> 3)  & 1) << 0)
    )
    return syndrome, p_tot, data_out, corrected, sec, ded, ok


class EccMemoryGoldenModel:
    """Bit-accurate, cycle-accurate mirror of top design module"""

    def __init__(self):
        self.reset()

    def reset(self):
        self.mem = [0] * 16
        self.scrub_ptr = 0
        self.sec_cnt = 0
        self.ded_cnt = 0
        self.last_err_addr = 0
        self.last_syndrome = 0
        self.last_sec = 0
        self.last_ded = 0
        self.err_alert = 0
        self.uo_out = 0

    def step(self, ui_in: int, addr: int, cmd: int, scrub_en: int):
        ui_in &= 0xFF
        addr &= 0xF
        cmd &= 0x3
        scrub_en &= 1

        telem_clr = (cmd == CMD_TELEM) and bool((addr >> 3) & 1)
        scrub_active = bool(scrub_en) and (
            (cmd == CMD_READ) or ((cmd == CMD_TELEM) and not bool((addr >> 3) & 1))
        )

        # Combinational paths evaluated on current state
        wr_encoded = encode_hamming13(ui_in)
        host_raw = self.mem[addr]
        inj_corrupted = inject_fault(host_raw, ui_in & 0xF, (ui_in >> 4) & 1)
        read_syn, _, read_data, _, read_sec, read_ded, _ = decode_secded(host_raw)

        scrub_raw = self.mem[self.scrub_ptr]
        scrub_syn, _, _, scrub_corrected, scrub_sec, scrub_ded, _ = decode_secded(scrub_raw)

        sel = addr & 0x3
        if sel == 0:
            telem_byte = (self.ded_cnt << 4) | self.sec_cnt
        elif sel == 1:
            telem_byte = (self.scrub_ptr << 4) | self.last_err_addr
        elif sel == 2:
            telem_byte = (self.last_ded << 7) | (self.last_sec << 6) | self.last_syndrome
        else:
            telem_byte = self.err_alert

        # Next-state registers (mirrors Verilog non-blocking `<=` semantics)
        n_mem = list(self.mem)
        n_scrub_ptr = self.scrub_ptr
        n_sec_cnt = self.sec_cnt
        n_ded_cnt = self.ded_cnt
        n_last_err_addr = self.last_err_addr
        n_last_syndrome = self.last_syndrome
        n_last_sec = self.last_sec
        n_last_ded = self.last_ded
        n_err_alert = self.err_alert
        n_uo_out = self.uo_out

        # 1. Host command
        if cmd == CMD_READ:
            n_uo_out = read_data
            if read_sec or read_ded:
                n_err_alert = 1
                if not (scrub_active and (scrub_sec or scrub_ded)):
                    n_last_err_addr = addr
                    n_last_syndrome = read_syn
                    n_last_sec = read_sec
                    n_last_ded = read_ded
        elif cmd == CMD_WRITE:
            n_mem[addr] = wr_encoded
            n_uo_out = ui_in
        elif cmd == CMD_INJECT:
            n_mem[addr] = inj_corrupted
            n_uo_out = read_data
        elif cmd == CMD_TELEM:
            n_uo_out = telem_byte
            if telem_clr:
                n_sec_cnt = 0
                n_ded_cnt = 0
                n_last_err_addr = 0
                n_last_syndrome = 0
                n_last_sec = 0
                n_last_ded = 0
                n_err_alert = 0

        # 2. Autonomous background scrubber
        if scrub_active:
            n_scrub_ptr = (self.scrub_ptr + 1) & 0xF
            if scrub_sec:
                n_mem[self.scrub_ptr] = scrub_corrected
                if self.sec_cnt != 0xF:
                    n_sec_cnt = self.sec_cnt + 1
                n_last_err_addr = self.scrub_ptr
                n_last_syndrome = scrub_syn
                n_last_sec = 1
                n_last_ded = 0
                n_err_alert = 1
            elif scrub_ded:
                if self.ded_cnt != 0xF:
                    n_ded_cnt = self.ded_cnt + 1
                n_last_err_addr = self.scrub_ptr
                n_last_syndrome = scrub_syn
                n_last_sec = 0
                n_last_ded = 1
                n_err_alert = 1

        # Commit
        self.mem = n_mem
        self.scrub_ptr = n_scrub_ptr
        self.sec_cnt = n_sec_cnt
        self.ded_cnt = n_ded_cnt
        self.last_err_addr = n_last_err_addr
        self.last_syndrome = n_last_syndrome
        self.last_sec = n_last_sec
        self.last_ded = n_last_ded
        self.err_alert = n_err_alert
        self.uo_out = n_uo_out

        return self.uo_out, self.err_alert


# -----------------------------------------------------------------------------
# Pin Drivers & Scoreboard Helper
# -----------------------------------------------------------------------------
def pack_uio(addr: int, cmd: int, scrub_en: int = 0, unused_msb: int = 0) -> int:
    return ((unused_msb & 1) << 7) | ((scrub_en & 1) << 6) | ((cmd & 0x3) << 4) | (addr & 0xF)


def unpack_alert(uio_out_val: int):
    return (uio_out_val & 0x7F), ((uio_out_val >> 7) & 1)


async def init_dut(dut):
    """Start 25 MHz clock and apply clean synchronous reset."""
    clock = Clock(dut.clk, 40, unit="ns")
    cocotb.start_soon(clock.start())

    dut.ena.value = 1
    dut.ui_in.value = 0
    dut.uio_in.value = 0
    dut.rst_n.value = 0
    await ClockCycles(dut.clk, 3)


async def step_and_check(
    dut,
    model: EccMemoryGoldenModel,
    ui_val: int,
    addr: int,
    cmd: int,
    scrub_en: int = 0,
    unused_msb: int = 0,
    ctx: str = "",
):
    """Drive inputs on FallingEdge, clock once, and verify DUT against Golden Model."""
    dut.ui_in.value = ui_val & 0xFF
    dut.uio_in.value = pack_uio(addr, cmd, scrub_en, unused_msb)
    await RisingEdge(dut.clk)
    await FallingEdge(dut.clk)

    exp_uo, exp_alert = model.step(ui_val, addr, cmd, scrub_en)

    got_oe = int(dut.uio_oe.value)
    got_uo = int(dut.uo_out.value)
    unused_low, got_alert = unpack_alert(int(dut.uio_out.value))

    assert got_oe == EXPECTED_UIO_OE, f"[{ctx}] uio_oe mismatch: got {got_oe:#010b}"
    assert unused_low == 0, f"[{ctx}] uio_out[6:0] non-zero: {unused_low:#09b}"
    assert (got_uo, got_alert) == (exp_uo, exp_alert), (
        f"[{ctx}] Mismatch (ui={ui_val:#x}, addr={addr:#x}, cmd={cmd}, scrub={scrub_en}): "
        f"got=({got_uo:#x}, alert={got_alert}) exp=({exp_uo:#x}, alert={exp_alert})"
    )
    return got_uo, got_alert


# -----------------------------------------------------------------------------
# Directed & Exhaustive Edge-Case Verification Tests
# -----------------------------------------------------------------------------
@cocotb.test()
async def test_reset_override_and_pin_directions(dut):
    """Verify uio_oe and prove active writes/injects are ignored while rst_n=0."""
    model = EccMemoryGoldenModel()
    await init_dut(dut)
    await FallingEdge(dut.clk)

    # Attempt writes & fault injects while still in reset
    for addr in range(4):
        dut.ui_in.value = 0xA5
        dut.uio_in.value = pack_uio(addr, CMD_WRITE, scrub_en=1, unused_msb=1)
        await RisingEdge(dut.clk)
        await FallingEdge(dut.clk)
        assert int(dut.uio_oe.value) == EXPECTED_UIO_OE
        assert int(dut.uo_out.value) == 0
        assert int(dut.uio_out.value) == 0

    # Release reset: all 16 RAM addresses and all 4 telemetry regs must be 0
    dut.rst_n.value = 1
    for addr in range(16):
        data, alert = await step_and_check(dut, model, 0, addr, CMD_READ, 0, ctx="post_rst_ram")
        assert data == 0x00 and alert == 0
    for reg in range(4):
        val, alert = await step_and_check(dut, model, 0, reg, CMD_TELEM, 0, ctx="post_rst_telem")
        assert val == 0x00 and alert == 0


@cocotb.test()
async def test_all_fault_positions_sec_ded_and_out_of_range(dut):
    """
    Test all 16 err_pos values (0..15) for both err_double=0 and err_double=1,
    including out-of-range (0, 14, 15) and err_pos=13 wrap-around, plus Host Read on DED.
    """
    model = EccMemoryGoldenModel()
    await init_dut(dut)
    await FallingEdge(dut.clk)
    dut.rst_n.value = 1

    # 1. Single-bit & out-of-range (err_double=0) across all 16 err_pos values
    for pos in range(16):
        data_byte = (0x37 * (pos + 1)) & 0xFF
        await step_and_check(dut, model, data_byte, pos, CMD_WRITE, 0, ctx=f"sec_wr_{pos}")
        await step_and_check(dut, model, pos, pos, CMD_INJECT, 0, ctx=f"sec_inj_{pos}")
        healed, alert = await step_and_check(dut, model, 0, pos, CMD_READ, 0, ctx=f"sec_rd_{pos}")

        assert healed == data_byte
        if 1 <= pos <= 13:
            assert alert == 1, f"Expected SEC alert at pos={pos}"
        else:
            assert alert == 0, f"Unexpected alert for no-fault pos={pos}"

        # Inspect telemetry regs 0..3 and clear
        for r in range(4):
            await step_and_check(dut, model, 0, r, CMD_TELEM, 0, ctx=f"sec_t{r}_{pos}")
        await step_and_check(dut, model, 0, 0x8, CMD_TELEM, 0, ctx=f"sec_clr_{pos}")

    # 2. Double-bit (err_double=1) across all 16 err_pos values (including 0, 13 wrap, 14, 15)
    for pos in range(16):
        data_byte = (0xC5 ^ (pos * 17)) & 0xFF
        await step_and_check(dut, model, data_byte, pos, CMD_WRITE, 0, ctx=f"ded_wr_{pos}")
        await step_and_check(dut, model, (1 << 4) | pos, pos, CMD_INJECT, 0, ctx=f"ded_inj_{pos}")
        # Host read with scrubber paused: must assert DED, alert=1, and not miscorrect
        _, alert = await step_and_check(dut, model, 0, pos, CMD_READ, 0, ctx=f"ded_rd_{pos}")
        assert alert == 1

        diag, _ = await step_and_check(dut, model, 0, 2, CMD_TELEM, 0, ctx=f"ded_diag_{pos}")
        assert ((diag >> 7) & 1) == 1 and ((diag >> 6) & 1) == 0
        await step_and_check(dut, model, 0, 0x8, CMD_TELEM, 0, ctx=f"ded_clr_{pos}")


@cocotb.test()
async def test_cumulative_fault_injection_and_cancellation(dut):
    """
    Verify two sequential 1-bit faults on distinct bits become a DED,
    and two sequential 1-bit faults on the same bit cancel out cleanly.
    """
    model = EccMemoryGoldenModel()
    await init_dut(dut)
    await FallingEdge(dut.clk)
    dut.rst_n.value = 1

    # Case A: Same bit flipped twice -> cancels out
    await step_and_check(dut, model, 0xA5, 2, CMD_WRITE, 0, ctx="cancel_wr")
    await step_and_check(dut, model, 5, 2, CMD_INJECT, 0, ctx="cancel_inj1")
    await step_and_check(dut, model, 5, 2, CMD_INJECT, 0, ctx="cancel_inj2")
    val, alert = await step_and_check(dut, model, 0, 2, CMD_READ, 0, ctx="cancel_rd")
    assert val == 0xA5 and alert == 0, "Double flip of same bit failed to cancel"

    # Case B: Two distinct 1-bit flips -> escalates to DED
    await step_and_check(dut, model, 0x5A, 3, CMD_WRITE, 0, ctx="accum_wr")
    await step_and_check(dut, model, 2, 3, CMD_INJECT, 0, ctx="accum_inj1")
    await step_and_check(dut, model, 9, 3, CMD_INJECT, 0, ctx="accum_inj2")
    _, alert = await step_and_check(dut, model, 0, 3, CMD_READ, 0, ctx="accum_rd")
    assert alert == 1
    diag, _ = await step_and_check(dut, model, 0, 2, CMD_TELEM, 0, ctx="accum_diag")
    assert ((diag >> 7) & 1) == 1 and ((diag >> 6) & 1) == 0, "Accumulated 2-bit fault not flagged as DED"


@cocotb.test()
async def test_scrubber_pointer_pause_advance_and_telem_scrub(dut):
    """
    Verify scrub_ptr advances during CMD_READ and CMD_TELEM(addr[3]=0) when scrub_en=1,
    and strictly pauses during CMD_WRITE, CMD_INJECT, and CMD_TELEM(addr[3]=1).
    """
    model = EccMemoryGoldenModel()
    await init_dut(dut)
    await FallingEdge(dut.clk)
    dut.rst_n.value = 1

    # Inject 1-bit fault at addr 0 and addr 1
    await step_and_check(dut, model, 0x11, 0, CMD_WRITE, scrub_en=1, ctx="ptr_wr0")
    await step_and_check(dut, model, 0x22, 1, CMD_WRITE, scrub_en=1, ctx="ptr_wr1")
    await step_and_check(dut, model, 4, 0, CMD_INJECT, scrub_en=1, ctx="ptr_inj0")
    await step_and_check(dut, model, 7, 1, CMD_INJECT, scrub_en=1, ctx="ptr_inj1")

    # scrub_ptr must still be 0 because WRITE and INJECT pause the scrubber
    ptrs, _ = await step_and_check(dut, model, 0, 1, CMD_TELEM, scrub_en=0, ctx="ptr_chk0")
    assert (ptrs >> 4) == 0, f"scrub_ptr moved during WRITE/INJECT: {ptrs >> 4}"

    # Read telemetry reg 1 with scrub_en=1 (addr[3]=0): scrubber must advance and heal addr 0
    ptrs, alert = await step_and_check(dut, model, 0, 1, CMD_TELEM, scrub_en=1, ctx="ptr_telem_scrub")
    assert (ptrs >> 4) == 0  # pre-step snapshot was 0
    assert alert == 1        # addr 0 fault caught & healed during CMD_TELEM

    # Next read confirms scrub_ptr advanced to 1 and sec_cnt == 1
    ptrs, _ = await step_and_check(dut, model, 0, 1, CMD_TELEM, scrub_en=0, ctx="ptr_chk1")
    assert (ptrs >> 4) == 1
    cnts, _ = await step_and_check(dut, model, 0, 0, CMD_TELEM, scrub_en=0, ctx="ptr_cnt1")
    assert (cnts & 0xF) == 1

    # Clear telemetry with scrub_en=1 (addr[3]=1): scrubber must pause (ptr stays 1)
    await step_and_check(dut, model, 0, 0x8 | 1, CMD_TELEM, scrub_en=1, ctx="ptr_clr_pause")
    ptrs, alert = await step_and_check(dut, model, 0, 1, CMD_TELEM, scrub_en=0, ctx="ptr_chk_still1")
    assert (ptrs >> 4) == 1 and alert == 0


@cocotb.test()
async def test_simultaneous_host_read_and_scrubber_collisions(dut):
    """
    Test both collision scenarios on the exact same clock edge:
    1) Host reads faulted addr A while scrubber heals faulted addr B (scrubber priority on last_*).
    2) Host reads addr C on the exact cycle scrubber heals addr C (same-address collision).
    """
    model = EccMemoryGoldenModel()
    await init_dut(dut)
    await FallingEdge(dut.clk)
    dut.rst_n.value = 1

    # Prep addr 0 (scrubber target) with SEC, addr 5 (host target) with DED, addr 1 with SEC
    await step_and_check(dut, model, 0xAA, 0, CMD_WRITE, 0, ctx="col_wr0")
    await step_and_check(dut, model, 0xBB, 1, CMD_WRITE, 0, ctx="col_wr1")
    await step_and_check(dut, model, 0x55, 5, CMD_WRITE, 0, ctx="col_wr5")

    await step_and_check(dut, model, 3, 0, CMD_INJECT, 0, ctx="col_inj0")          # SEC at 0
    await step_and_check(dut, model, 6, 1, CMD_INJECT, 0, ctx="col_inj1")          # SEC at 1
    await step_and_check(dut, model, (1 << 4) | 2, 5, CMD_INJECT, 0, ctx="col_inj5")  # DED at 5

    # Collision 1: scrub_ptr == 0 (SEC), host reads addr 5 (DED) with scrub_en=1
    await step_and_check(dut, model, 0, 5, CMD_READ, scrub_en=1, ctx="col_diff_addr")
    # Scrubber must win last_err_addr (=0) and last_sec (=1, last_ded=0)
    ptrs, _ = await step_and_check(dut, model, 0, 1, CMD_TELEM, 0, ctx="col1_ptrs")
    diag, _ = await step_and_check(dut, model, 0, 2, CMD_TELEM, 0, ctx="col1_diag")
    assert (ptrs & 0xF) == 0 and ((diag >> 6) & 1) == 1 and ((diag >> 7) & 1) == 0

    # Collision 2: scrub_ptr is now 1 (SEC); host simultaneously reads addr 1 with scrub_en=1
    healed, alert = await step_and_check(dut, model, 0, 1, CMD_READ, scrub_en=1, ctx="col_same_addr")
    assert healed == 0xBB and alert == 1

    # Confirm both addr 0 and addr 1 are now permanently healed in RAM
    await step_and_check(dut, model, 0, 0x8, CMD_TELEM, 0, ctx="col_clr")
    d0, a0 = await step_and_check(dut, model, 0, 0, CMD_READ, 0, ctx="col_verify0")
    d1, a1 = await step_and_check(dut, model, 0, 1, CMD_READ, 0, ctx="col_verify1")
    assert (d0, a0) == (0xAA, 0) and (d1, a1) == (0xBB, 0)


@cocotb.test()
async def test_sec_and_ded_counter_saturation(dut):
    """Verify both sec_cnt and ded_cnt saturate cleanly at 15 (4'hF) without wrap-around."""
    model = EccMemoryGoldenModel()
    await init_dut(dut)
    await FallingEdge(dut.clk)
    dut.rst_n.value = 1

    # 1. Saturate sec_cnt (17 single-bit faults healed)
    await step_and_check(dut, model, 0x77, 4, CMD_WRITE, 0, ctx="sat_wr4")
    for r in range(17):
        pos = (r % 13) + 1
        await step_and_check(dut, model, pos, 4, CMD_INJECT, 1, ctx=f"sat_sec_inj_{r}")
        for _ in range(16):
            await step_and_check(dut, model, 0, 4, CMD_READ, 1, ctx=f"sat_sec_swp_{r}")

    cnts, _ = await step_and_check(dut, model, 0, 0, CMD_TELEM, 0, ctx="sat_sec_chk")
    assert (cnts & 0xF) == 15

    # 2. Saturate ded_cnt (17 scrubber sweeps over a persistent 2-bit fault)
    await step_and_check(dut, model, 0x99, 7, CMD_WRITE, 0, ctx="sat_wr7")
    await step_and_check(dut, model, (1 << 4) | 11, 7, CMD_INJECT, 0, ctx="sat_ded_inj")
    for s in range(17):
        for _ in range(16):
            await step_and_check(dut, model, 0, 0, CMD_READ, 1, ctx=f"sat_ded_swp_{s}")

    cnts, alert = await step_and_check(dut, model, 0, 0, CMD_TELEM, 0, ctx="sat_both_chk")
    assert cnts == 0xFF and alert == 1, f"Expected both counters saturated (0xFF), got {cnts:#x}"

    # Verify telemetry reg 3 (err_alert readback) and atomic clear
    al_reg, _ = await step_and_check(dut, model, 0, 3, CMD_TELEM, 0, ctx="sat_al_reg")
    assert al_reg == 1
    await step_and_check(dut, model, 0, 0x8 | 3, CMD_TELEM, 0, ctx="sat_clr")
    al_reg, alert = await step_and_check(dut, model, 0, 3, CMD_TELEM, 0, ctx="sat_al_post_clr")
    assert al_reg == 0 and alert == 0


@cocotb.test()
async def test_mid_op_reset_and_unused_pin_immunity(dut):
    """Verify mid-operation reset clears all state and toggling uio_in[7] / addr[2] has no side effects."""
    model = EccMemoryGoldenModel()
    await init_dut(dut)
    await FallingEdge(dut.clk)
    dut.rst_n.value = 1

    # Populate state while toggling unused uio_in[7] and addr[2] during CMD_TELEM
    await step_and_check(dut, model, 0xDE, 9, CMD_WRITE, 0, unused_msb=1, ctx="rst_wr")
    await step_and_check(dut, model, 3, 9, CMD_INJECT, 0, unused_msb=1, ctx="rst_inj")
    await step_and_check(dut, model, 0, 9, CMD_READ, 1, unused_msb=1, ctx="rst_rd")
    await step_and_check(dut, model, 0, 0b0100, CMD_TELEM, 0, unused_msb=1, ctx="rst_telem_addr2")

    # Synchronous reset mid-operation
    dut.rst_n.value = 0
    await RisingEdge(dut.clk)
    await FallingEdge(dut.clk)
    model.reset()
    assert int(dut.uo_out.value) == 0 and int(dut.uio_out.value) == 0

    dut.rst_n.value = 1
    for addr in range(16):
        await step_and_check(dut, model, 0, addr, CMD_READ, 0, ctx="mid_rst_verify")


@cocotb.test()
async def test_randomized_golden_scoreboard_stress(dut):
    """
    2,000-cycle deterministic pseudo-random stress test exercising all commands,
    fault modes, scrubber toggles, and periodic full-RAM + full-telemetry audits.
    """
    rng = random.Random(20261121)
    model = EccMemoryGoldenModel()
    await init_dut(dut)
    await FallingEdge(dut.clk)
    dut.rst_n.value = 1

    # Seed RAM with all 16 slots populated
    for addr in range(16):
        await step_and_check(dut, model, rng.randrange(256), addr, CMD_WRITE, 0, ctx="rand_init")

    for cycle in range(2000):
        cmd = rng.choices([CMD_READ, CMD_WRITE, CMD_INJECT, CMD_TELEM], weights=[40, 25, 20, 15])[0]
        addr = rng.randrange(16)
        ui_val = rng.randrange(256)
        scrub_en = rng.randrange(2)
        unused_msb = rng.randrange(2)

        await step_and_check(
            dut, model, ui_val, addr, cmd, scrub_en, unused_msb, ctx=f"rand_c{cycle}"
        )

        # Every 250 cycles, audit all 4 telemetry regs and all 16 RAM addresses
        if (cycle + 1) % 250 == 0:
            for r in range(4):
                await step_and_check(dut, model, 0, r, CMD_TELEM, 0, ctx=f"audit_t{r}_c{cycle}")
            for a in range(16):
                await step_and_check(dut, model, 0, a, CMD_READ, 0, ctx=f"audit_ram{a}_c{cycle}")

    dut._log.info("2,000-cycle randomized golden scoreboard stress test passed with 0 mismatches!")
