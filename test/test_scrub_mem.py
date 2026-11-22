# SPDX-FileCopyrightText: © 2026 Tejas Patel, Pandit Deendayal Energy University
# SPDX-License-Identifier: Apache-2.0

import random
import cocotb
from cocotb.clock import Clock
from cocotb.triggers import ClockCycles, FallingEdge, RisingEdge


EXPECTED_UIO_OE = 0b1100_0000

CMD_READ   = 0b00
CMD_WRITE  = 0b01
CMD_INJECT = 0b10
CMD_TELEM  = 0b11


# --- Independent Cycle-Accurate Golden Reference Model ---
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

    fixed = code_in ^ ((1 << syndrome) if (sec and syndrome <= 12) else 0)
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
    return syndrome, p_tot, data_out, fixed, sec, ded, ok


class EccSinglePortArbiterModel:
    """Bit-accurate, cycle-accurate mirror of arbitered single-port ECC-protected scrubbing memory"""

    def __init__(self):
        self.reset()

    def reset(self):
        self.mem = [0] * 16
        self.scrub_en = 1       # Enabled by default on reset
        self.starve_cnt = 0
        self.scrub_ptr = 0
        self.sec_cnt = 0
        self.ded_cnt = 0
        self.last_err_addr = 0
        self.last_syndrome = 0
        self.last_sec = 0
        self.last_ded = 0
        self.err_alert = 0
        self.stall = 0
        self.uo_out = 0

    def step(self, ui_in: int, addr: int, cmd: int):
        ui_in &= 0xFF
        addr &= 0xF
        cmd &= 0x3

        host_mem_op = (cmd != CMD_TELEM)
        csr_we      = (cmd == CMD_TELEM) and bool((addr >> 2) & 1)
        telem_clr   = (cmd == CMD_TELEM) and bool((addr >> 3) & 1)

        # 3-tier arbiter (evaluated on pre-edge state)
        scrub_forced = bool(self.scrub_en) and (self.starve_cnt == 0xF) and host_mem_op
        scrub_idle   = bool(self.scrub_en) and (cmd == CMD_TELEM) and (not telem_clr)
        scrub_grant  = scrub_forced or scrub_idle
        host_grant   = host_mem_op and (not scrub_forced)

        # Shared single-port read & SECDED decode
        port_addr = self.scrub_ptr if scrub_grant else addr
        port_raw  = self.mem[port_addr]

        wr_encoded    = encode_hamming13(ui_in)
        inj_corrupted = inject_fault(port_raw, ui_in & 0xF, (ui_in >> 4) & 1)
        dec_syn, _, dec_data, dec_fixed, dec_sec, dec_ded, _ = decode_secded(port_raw)

        # Shared single-port write mux
        mem_we = False
        mem_wdata = 0
        if scrub_grant and dec_sec:
            mem_we = True
            mem_wdata = dec_fixed
        elif host_grant and (cmd == CMD_WRITE):
            mem_we = True
            mem_wdata = wr_encoded
        elif host_grant and (cmd == CMD_INJECT):
            mem_we = True
            mem_wdata = inj_corrupted

        # Telemetry readback mux (pre-edge snapshot)
        sel = addr & 0x3
        if sel == 0:
            telem_byte = (self.ded_cnt << 4) | self.sec_cnt
        elif sel == 1:
            telem_byte = (self.scrub_ptr << 4) | self.last_err_addr
        elif sel == 2:
            telem_byte = (self.last_ded << 7) | (self.last_sec << 6) | self.last_syndrome
        else:
            telem_byte = (self.starve_cnt << 4) | (self.scrub_en << 1) | self.err_alert

        # Next-state registers (mirrors Verilog non-blocking `<=` semantics)
        n_mem           = list(self.mem)
        n_scrub_en      = self.scrub_en
        n_starve_cnt    = self.starve_cnt
        n_scrub_ptr     = self.scrub_ptr
        n_sec_cnt       = self.sec_cnt
        n_ded_cnt       = self.ded_cnt
        n_last_err_addr = self.last_err_addr
        n_last_syndrome = self.last_syndrome
        n_last_sec      = self.last_sec
        n_last_ded      = self.last_ded
        n_err_alert     = self.err_alert
        n_stall         = 1 if scrub_forced else 0
        n_uo_out        = self.uo_out

        # 1. Single-port RAM write
        if mem_we:
            n_mem[port_addr] = mem_wdata

        # 2. Watchdog starvation counter
        if (not self.scrub_en) or scrub_grant:
            n_starve_cnt = 0
        elif host_grant and (self.starve_cnt != 0xF):
            n_starve_cnt = self.starve_cnt + 1

        # 3. Host command execution
        if cmd == CMD_TELEM:
            n_uo_out = telem_byte
            if csr_we:
                n_scrub_en = ui_in & 1
            if telem_clr:
                n_sec_cnt       = 0
                n_ded_cnt       = 0
                n_last_err_addr = 0
                n_last_syndrome = 0
                n_last_sec      = 0
                n_last_ded      = 0
                n_err_alert     = 0
        elif host_grant:
            if cmd == CMD_READ:
                n_uo_out = dec_data
                if dec_sec or dec_ded:
                    n_err_alert     = 1
                    n_last_err_addr = addr
                    n_last_syndrome = dec_syn
                    n_last_sec      = dec_sec
                    n_last_ded      = dec_ded
            elif cmd == CMD_WRITE:
                n_uo_out = ui_in
            elif cmd == CMD_INJECT:
                n_uo_out = dec_data

        # 4. Scrubber execution (mandatory cycle-steal or opportunistic idle)
        if scrub_grant:
            n_scrub_ptr = (self.scrub_ptr + 1) & 0xF
            if dec_sec:
                if self.sec_cnt != 0xF:
                    n_sec_cnt = self.sec_cnt + 1
                n_last_err_addr = self.scrub_ptr
                n_last_syndrome = dec_syn
                n_last_sec      = 1
                n_last_ded      = 0
                n_err_alert     = 1
            elif dec_ded:
                if self.ded_cnt != 0xF:
                    n_ded_cnt = self.ded_cnt + 1
                n_last_err_addr = self.scrub_ptr
                n_last_syndrome = dec_syn
                n_last_sec      = 0
                n_last_ded      = 1
                n_err_alert     = 1

        # Commit next state
        self.mem           = n_mem
        self.scrub_en      = n_scrub_en
        self.starve_cnt    = n_starve_cnt
        self.scrub_ptr     = n_scrub_ptr
        self.sec_cnt       = n_sec_cnt
        self.ded_cnt       = n_ded_cnt
        self.last_err_addr = n_last_err_addr
        self.last_syndrome = n_last_syndrome
        self.last_sec      = n_last_sec
        self.last_ded      = n_last_ded
        self.err_alert     = n_err_alert
        self.stall         = n_stall
        self.uo_out        = n_uo_out

        return self.uo_out, self.stall, self.err_alert


# --- Pin Drivers & Scoreboard Helpers ---
def pack_uio(addr: int, cmd: int, unused_hi: int = 0) -> int:
    return ((unused_hi & 0x3) << 6) | ((cmd & 0x3) << 4) | (addr & 0xF)


def unpack_uio_out(uio_out_val: int):
    unused_low = uio_out_val & 0x3F
    stall      = (uio_out_val >> 6) & 1
    alert      = (uio_out_val >> 7) & 1
    return unused_low, stall, alert


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
    model: EccSinglePortArbiterModel,
    ui_val: int,
    addr: int,
    cmd: int,
    unused_hi: int = 0,
    ctx: str = "",
):
    """Drive inputs on FallingEdge, clock once, and verify DUT against Golden Model."""
    dut.ui_in.value = ui_val & 0xFF
    dut.uio_in.value = pack_uio(addr, cmd, unused_hi)
    await RisingEdge(dut.clk)
    await FallingEdge(dut.clk)

    exp_uo, exp_stall, exp_alert = model.step(ui_val, addr, cmd)

    got_oe = int(dut.uio_oe.value)
    got_uo = int(dut.uo_out.value)
    unused_low, got_stall, got_alert = unpack_uio_out(int(dut.uio_out.value))

    assert got_oe == EXPECTED_UIO_OE, f"[{ctx}] uio_oe mismatch: got {got_oe:#010b}"
    assert unused_low == 0, f"[{ctx}] uio_out[5:0] non-zero: {unused_low:#08b}"
    assert (got_uo, got_stall, got_alert) == (exp_uo, exp_stall, exp_alert), (
        f"[{ctx}] Mismatch (ui={ui_val:#x}, addr={addr:#x}, cmd={cmd}): "
        f"got=(uo={got_uo:#x}, stall={got_stall}, alert={got_alert}) "
        f"exp=(uo={exp_uo:#x}, stall={exp_stall}, alert={exp_alert})"
    )
    return got_uo, got_stall, got_alert


async def set_scrub_en(dut, model, enable: int, clear: int = 0, reg_sel: int = 3, ctx: str = "csr"):
    """Write internal scrub_en CSR via CMD_TELEM (addr[2]=1)."""
    addr = ((clear & 1) << 3) | (1 << 2) | (reg_sel & 0x3)
    return await step_and_check(dut, model, enable & 1, addr, CMD_TELEM, ctx=ctx)


# --- Directed & Edge-Case Verification Tests ---
@cocotb.test()
async def test_reset_defaults_and_override(dut):
    """Verify uio_oe, reset override under active writes, and default scrub_en=1."""
    model = EccSinglePortArbiterModel()
    await init_dut(dut)
    await FallingEdge(dut.clk)

    # Attempt active writes/injects while rst_n=0 -> must be completely ignored
    for addr in range(4):
        dut.ui_in.value = 0xA5
        dut.uio_in.value = pack_uio(addr, CMD_WRITE, unused_hi=0b11)
        await RisingEdge(dut.clk)
        await FallingEdge(dut.clk)
        assert int(dut.uio_oe.value) == EXPECTED_UIO_OE
        assert int(dut.uo_out.value) == 0
        assert int(dut.uio_out.value) == 0

    # Release reset: check reg 3 confirms scrub_en=1, starve_cnt=0, alert=0
    dut.rst_n.value = 1
    reg3, stall, alert = await step_and_check(dut, model, 0, 3, CMD_TELEM, ctx="post_rst_reg3")
    assert reg3 == 0x02, f"Expected default scrub_en=1 (0x02), got {reg3:#x}"
    assert stall == 0 and alert == 0

    # Disable scrubber so we can cleanly read all 16 RAM slots without triggering a stall
    await set_scrub_en(dut, model, 0, ctx="rst_dis_scrub")
    for addr in range(16):
        data, stall, alert = await step_and_check(dut, model, 0, addr, CMD_READ, ctx=f"post_rst_ram{addr}")
        assert (data, stall, alert) == (0x00, 0, 0)


@cocotb.test()
async def test_csr_scrub_enable_disable_and_combined_clear(dut):
    """
    Verify exact CSR transition boundary semantics:
    1) Disabling scrub_en (1->0) with telem_clr=0 still runs 1 final idle scrub on that cycle.
    2) Enabling scrub_en (0->1) takes effect on the following cycle.
    3) Combined csr_we + telem_clr (addr[3:2]=2'b11) updates scrub_en AND clears telemetry while pausing scrub.
    4) telem_clr=1 with scrub_en=1 holds starve_cnt steady (neither increments nor resets).
    """
    model = EccSinglePortArbiterModel()
    await init_dut(dut)
    await FallingEdge(dut.clk)
    dut.rst_n.value = 1

    # scrub_en starts at 1, scrub_ptr=0. Turn scrub_en OFF with telem_clr=0 (addr = 0b0101 -> reg 1)
    # Pre-edge scrub_en was 1, so scrub_ptr must advance 0->1 on this transition cycle
    snap, stall, _ = await set_scrub_en(dut, model, enable=0, clear=0, reg_sel=1, ctx="csr_off_edge")
    assert (snap >> 4) == 0 and stall == 0
    reg1, _, _ = await step_and_check(dut, model, 0, 1, CMD_TELEM, ctx="csr_off_verify_ptr")
    assert (reg1 >> 4) == 1, f"Final idle scrub on 1->0 edge did not advance ptr: {reg1 >> 4}"

    # Another CMD_TELEM now that scrub_en=0 must not advance scrub_ptr
    reg1, _, _ = await step_and_check(dut, model, 0, 1, CMD_TELEM, ctx="csr_off_hold_ptr")
    assert (reg1 >> 4) == 1

    # Turn scrub_en ON (0->1) -> pre-edge scrub_en was 0, so ptr stays 1 on this cycle
    await set_scrub_en(dut, model, enable=1, clear=0, reg_sel=3, ctx="csr_on_edge")
    reg1, _, _ = await step_and_check(dut, model, 0, 1, CMD_TELEM, ctx="csr_on_first_scrub")
    assert (reg1 >> 4) == 1  # pre-edge snapshot was 1; this cycle advances it to 2
    reg1, _, _ = await step_and_check(dut, model, 0, 1, CMD_TELEM, ctx="csr_on_verify_ptr2")
    assert (reg1 >> 4) == 2

    # Build starve_cnt to 5 using host reads, then issue telem_clr=1 (scrub_en stays 1)
    # starve_cnt must hold at 5 during telem_clr
    for i in range(5):
        await step_and_check(dut, model, 0, i, CMD_READ, ctx=f"starve_build_{i}")
    reg3_pre, _, _ = await step_and_check(dut, model, 0, 0x8 | 3, CMD_TELEM, ctx="telem_clr_hold_starve")
    assert (reg3_pre >> 4) == 5
    # Next cycle: read reg 3 with telem_clr=0 -> snapshot must still show starve_cnt == 5 (then idle scrub resets to 0)
    reg3_post, _, _ = await step_and_check(dut, model, 0, 3, CMD_TELEM, ctx="verify_starve_held")
    assert (reg3_post >> 4) == 5, f"starve_cnt did not hold during telem_clr: got {reg3_post >> 4}"

    # Combined csr_we + telem_clr (addr[3:2]=2'b11): simultaneously disable scrub_en and clear alert
    await step_and_check(dut, model, 0xA5, 0, CMD_WRITE, ctx="comb_wr")
    await step_and_check(dut, model, 2, 0, CMD_INJECT, ctx="comb_inj")
    await step_and_check(dut, model, 0, 0, CMD_READ, ctx="comb_rd")  # raises err_alert=1
    _, _, alert = await set_scrub_en(dut, model, enable=0, clear=1, reg_sel=3, ctx="comb_csr_clr")
    assert alert == 0
    reg3, _, _ = await step_and_check(dut, model, 0, 3, CMD_TELEM, ctx="comb_verify")
    assert (reg3 & 0x3) == 0, f"Combined CSR+CLR failed: reg3={reg3:#x}"


@cocotb.test()
async def test_all_fault_positions_sec_ded_and_out_of_range(dut):
    """
    Test all 16 err_pos values (0..15) for err_double=0 and err_double=1,
    including out-of-range (0, 14, 15), err_pos=13 wrap-around, and host read on DED.
    """
    model = EccSinglePortArbiterModel()
    await init_dut(dut)
    await FallingEdge(dut.clk)
    dut.rst_n.value = 1
    await set_scrub_en(dut, model, 0, ctx="flt_dis_scrub")

    # 1. Single-bit & out-of-range (err_double=0)
    for pos in range(16):
        data_byte = (0x37 * (pos + 1)) & 0xFF
        await step_and_check(dut, model, data_byte, pos, CMD_WRITE, ctx=f"sec_wr_{pos}")
        await step_and_check(dut, model, pos, pos, CMD_INJECT, ctx=f"sec_inj_{pos}")
        healed, stall, alert = await step_and_check(dut, model, 0, pos, CMD_READ, ctx=f"sec_rd_{pos}")

        assert stall == 0 and healed == data_byte
        if 1 <= pos <= 13:
            assert alert == 1, f"Expected SEC alert at pos={pos}"
        else:
            assert alert == 0, f"Unexpected alert for no-fault pos={pos}"

        for r in range(4):
            await step_and_check(dut, model, 0, r, CMD_TELEM, ctx=f"sec_t{r}_{pos}")
        await step_and_check(dut, model, 0, 0x8, CMD_TELEM, ctx=f"sec_clr_{pos}")

    # 2. Double-bit (err_double=1) across all 16 err_pos values
    for pos in range(16):
        data_byte = (0xC5 ^ (pos * 17)) & 0xFF
        await step_and_check(dut, model, data_byte, pos, CMD_WRITE, ctx=f"ded_wr_{pos}")
        await step_and_check(dut, model, (1 << 4) | pos, pos, CMD_INJECT, ctx=f"ded_inj_{pos}")
        _, stall, alert = await step_and_check(dut, model, 0, pos, CMD_READ, ctx=f"ded_rd_{pos}")
        assert stall == 0 and alert == 1

        diag, _, _ = await step_and_check(dut, model, 0, 2, CMD_TELEM, ctx=f"ded_diag_{pos}")
        assert ((diag >> 7) & 1) == 1 and ((diag >> 6) & 1) == 0
        await step_and_check(dut, model, 0, 0x8, CMD_TELEM, ctx=f"ded_clr_{pos}")


@cocotb.test()
async def test_cumulative_fault_injection_and_cancellation(dut):
    """Two flips on the same bit cancel out; two flips on distinct bits escalate to DED."""
    model = EccSinglePortArbiterModel()
    await init_dut(dut)
    await FallingEdge(dut.clk)
    dut.rst_n.value = 1
    await set_scrub_en(dut, model, 0, ctx="cum_dis_scrub")

    # Same bit flipped twice -> cancels out
    await step_and_check(dut, model, 0xA5, 2, CMD_WRITE, ctx="cancel_wr")
    await step_and_check(dut, model, 5, 2, CMD_INJECT, ctx="cancel_inj1")
    await step_and_check(dut, model, 5, 2, CMD_INJECT, ctx="cancel_inj2")
    val, _, alert = await step_and_check(dut, model, 0, 2, CMD_READ, ctx="cancel_rd")
    assert val == 0xA5 and alert == 0

    # Two distinct bits flipped -> DED
    await step_and_check(dut, model, 0x5A, 3, CMD_WRITE, ctx="accum_wr")
    await step_and_check(dut, model, 2, 3, CMD_INJECT, ctx="accum_inj1")
    await step_and_check(dut, model, 9, 3, CMD_INJECT, ctx="accum_inj2")
    _, _, alert = await step_and_check(dut, model, 0, 3, CMD_READ, ctx="accum_rd")
    assert alert == 1
    diag, _, _ = await step_and_check(dut, model, 0, 2, CMD_TELEM, ctx="accum_diag")
    assert ((diag >> 7) & 1) == 1 and ((diag >> 6) & 1) == 0


@cocotb.test()
async def test_opportunistic_idle_scrubbing_and_permanent_healing(dut):
    """
    Inject 5 single-bit faults with scrub_en=0, re-enable scrub_en=1, run 16
    opportunistic CMD_TELEM idle cycles (zero stalls), and prove all 5 are healed in RAM.
    """
    model = EccSinglePortArbiterModel()
    await init_dut(dut)
    await FallingEdge(dut.clk)
    dut.rst_n.value = 1
    await set_scrub_en(dut, model, 0, ctx="idle_dis_scrub")

    mem_expected = [(0x11 * i) & 0xFF for i in range(16)]
    for addr, val in enumerate(mem_expected):
        await step_and_check(dut, model, val, addr, CMD_WRITE, ctx=f"idle_wr_{addr}")

    faults = [(0, 1), (4, 4), (7, 8), (11, 12), (15, 13)]
    for addr, pos in faults:
        await step_and_check(dut, model, pos, addr, CMD_INJECT, ctx=f"idle_inj_{addr}")

    # Re-enable scrubber and sweep all 16 slots via idle CMD_TELEM cycles
    await set_scrub_en(dut, model, 1, ctx="idle_en_scrub")
    for c in range(16):
        _, stall, _ = await step_and_check(dut, model, 0, 0, CMD_TELEM, ctx=f"idle_swp_{c}")
        assert stall == 0, f"Unexpected stall during opportunistic idle scrub cycle {c}"

    # Verify sec_cnt == 5, ded_cnt == 0, alert == 1
    cnts, stall, alert = await step_and_check(dut, model, 0, 0, CMD_TELEM, ctx="idle_cnts")
    assert (cnts & 0xF) == len(faults) and ((cnts >> 4) & 0xF) == 0
    assert stall == 0 and alert == 1

    # Disable scrubber + clear telemetry, then verify every RAM slot is permanently healed
    await set_scrub_en(dut, model, 0, clear=1, ctx="idle_clr_dis")
    for addr, exp_val in enumerate(mem_expected):
        got_val, stall, alert = await step_and_check(dut, model, 0, addr, CMD_READ, ctx=f"idle_chk_{addr}")
        assert (got_val, stall, alert) == (exp_val, 0, 0), f"Addr {addr} not permanently healed!"


@cocotb.test()
async def test_watchdog_cycle_steal_and_stall_handshake(dut):
    """
    Prove starvation is impossible when scrub_en=1:
    After 15 consecutive host memory ops, the 16th op triggers a mandatory 1-cycle
    scrub with stall=1, holds uo_out, preserves RAM against the stalled op, heals
    the scrubbed slot, and accepts the retried host op on cycle 17.
    Tests cycle-stealing preempting CMD_WRITE, CMD_INJECT, and CMD_READ.
    """
    model = EccSinglePortArbiterModel()
    await init_dut(dut)
    await FallingEdge(dut.clk)
    dut.rst_n.value = 1

    # Prepare 3 faulted slots at scrub_ptr = 0, 1, 2 (note: initial reset puts scrub_ptr=0)
    await set_scrub_en(dut, model, 0, clear=1, ctx="wd_setup_dis")  # clear=1 prevents ptr advance
    for addr, val in [(0, 0xA1), (1, 0xB2), (2, 0xC3), (5, 0x55)]:
        await step_and_check(dut, model, val, addr, CMD_WRITE, ctx=f"wd_wr_{addr}")
    await step_and_check(dut, model, 3, 0, CMD_INJECT, ctx="wd_inj0")
    await step_and_check(dut, model, 6, 1, CMD_INJECT, ctx="wd_inj1")
    await step_and_check(dut, model, 9, 2, CMD_INJECT, ctx="wd_inj2")

    # Enable scrubber (pre-edge was 0, so scrub_ptr stays 0, starve_cnt=0)
    await set_scrub_en(dut, model, 1, ctx="wd_en_scrub")

    # --- Case 1: Preempt a host CMD_WRITE on cycle 16 (heals mem[0]) ---
    for i in range(15):
        uo, stall, _ = await step_and_check(dut, model, 0, 5, CMD_READ, ctx=f"wd1_rd_{i}")
        assert uo == 0x55 and stall == 0

    # 16th consecutive host op: attempt CMD_WRITE(0xEE -> addr 6) -> must stall
    uo, stall, alert = await step_and_check(dut, model, 0xEE, 6, CMD_WRITE, ctx="wd1_stall_wr")
    assert stall == 1, "Watchdog failed to assert stall on 16th consecutive host op"
    assert uo == 0x55, f"uo_out did not hold previous value during stall: got {uo:#x}"
    assert alert == 1, "Mandatory cycle-steal scrub failed to catch fault at mem[0]"

    # 17th cycle: retry CMD_WRITE(0xEE -> addr 6) -> must succeed with stall=0
    uo, stall, _ = await step_and_check(dut, model, 0xEE, 6, CMD_WRITE, ctx="wd1_retry_wr")
    assert stall == 0 and uo == 0xEE

    # --- Case 2: Preempt a host CMD_INJECT on cycle 16 (heals mem[1]) ---
    # starve_cnt is currently 1 (from the retried write), so 14 more ops reach starve_cnt=15
    for i in range(14):
        _, stall, _ = await step_and_check(dut, model, 0, 6, CMD_READ, ctx=f"wd2_rd_{i}")
        assert stall == 0

    # Attempt CMD_INJECT on addr 6 -> must stall and not corrupt addr 6 yet
    uo, stall, _ = await step_and_check(dut, model, 4, 6, CMD_INJECT, ctx="wd2_stall_inj")
    assert stall == 1 and uo == 0xEE

    # Retry CMD_INJECT on addr 6 -> succeeds
    _, stall, _ = await step_and_check(dut, model, 4, 6, CMD_INJECT, ctx="wd2_retry_inj")
    assert stall == 0

    # --- Case 3: Preempt a host CMD_READ on cycle 16 (heals mem[2]) ---
    for i in range(14):
        _, stall, _ = await step_and_check(dut, model, 0, 5, CMD_READ, ctx=f"wd3_rd_{i}")
        assert stall == 0

    # Attempt CMD_READ on addr 6 -> must stall (holds 0x55, heals mem[2])
    uo, stall, _ = await step_and_check(dut, model, 0, 6, CMD_READ, ctx="wd3_stall_rd")
    assert stall == 1 and uo == 0x55

    # Retry CMD_READ on addr 6 -> returns healed 0xEE
    uo, stall, _ = await step_and_check(dut, model, 0, 6, CMD_READ, ctx="wd3_retry_rd")
    assert stall == 0 and uo == 0xEE

    # Verify sec_cnt == 3 and mem[0..2] are all permanently healed
    cnts, _, _ = await step_and_check(dut, model, 0, 0, CMD_TELEM, ctx="wd_chk_cnts")
    assert (cnts & 0xF) == 3, f"Expected 3 watchdog-scrubbed faults, got {cnts & 0xF}"

    await set_scrub_en(dut, model, 0, clear=1, ctx="wd_clr_dis")
    for addr, exp_val in [(0, 0xA1), (1, 0xB2), (2, 0xC3)]:
        got_val, stall, alert = await step_and_check(dut, model, 0, addr, CMD_READ, ctx=f"wd_heal_{addr}")
        assert (got_val, stall, alert) == (exp_val, 0, 0)


@cocotb.test()
async def test_no_stall_when_scrub_disabled(dut):
    """When scrub_en=0, 32 consecutive host memory ops must never stall or move scrub_ptr."""
    model = EccSinglePortArbiterModel()
    await init_dut(dut)
    await FallingEdge(dut.clk)
    dut.rst_n.value = 1

    await set_scrub_en(dut, model, 0, clear=1, ctx="nostall_dis")
    for i in range(32):
        _, stall, _ = await step_and_check(dut, model, i & 0xFF, i & 0xF, CMD_WRITE, ctx=f"nostall_wr_{i}")
        assert stall == 0, f"Stall asserted while scrub_en=0 at cycle {i}"

    reg1, _, _ = await step_and_check(dut, model, 0, 1, CMD_TELEM, ctx="nostall_ptr")
    reg3, _, _ = await step_and_check(dut, model, 0, 3, CMD_TELEM, ctx="nostall_starve")
    assert (reg1 >> 4) == 0, f"scrub_ptr moved while disabled: {reg1 >> 4}"
    assert (reg3 >> 4) == 0, f"starve_cnt incremented while disabled: {reg3 >> 4}"


@cocotb.test()
async def test_sec_and_ded_counter_saturation(dut):
    """Verify both 4-bit sec_cnt and ded_cnt saturate at 15 (4'hF) without wrap-around."""
    model = EccSinglePortArbiterModel()
    await init_dut(dut)
    await FallingEdge(dut.clk)
    dut.rst_n.value = 1

    # 1. Saturate sec_cnt (17 single-bit faults healed via opportunistic idle sweeps)
    await set_scrub_en(dut, model, 0, clear=1, ctx="sat_dis")
    await step_and_check(dut, model, 0x77, 4, CMD_WRITE, ctx="sat_wr4")
    await set_scrub_en(dut, model, 1, ctx="sat_en")

    for r in range(17):
        pos = (r % 13) + 1
        await step_and_check(dut, model, pos, 4, CMD_INJECT, ctx=f"sat_sec_inj_{r}")
        for s in range(16):
            await step_and_check(dut, model, 0, 0, CMD_TELEM, ctx=f"sat_sec_swp_{r}_{s}")

    cnts, _, _ = await step_and_check(dut, model, 0, 0, CMD_TELEM, ctx="sat_sec_chk")
    assert (cnts & 0xF) == 15

    # 2. Saturate ded_cnt (17 full sweeps over a persistent 2-bit fault; verify no miscorrection)
    await step_and_check(dut, model, 0x99, 7, CMD_WRITE, ctx="sat_wr7")
    await step_and_check(dut, model, (1 << 4) | 11, 7, CMD_INJECT, ctx="sat_ded_inj")
    for r in range(17):
        for s in range(16):
            await step_and_check(dut, model, 0, 0, CMD_TELEM, ctx=f"sat_ded_swp_{r}_{s}")

    cnts, _, alert = await step_and_check(dut, model, 0, 0, CMD_TELEM, ctx="sat_both_chk")
    assert cnts == 0xFF and alert == 1, f"Expected 0xFF saturated counters, got {cnts:#x}"

    # Atomic read-and-clear snapshot
    snap, _, alert = await step_and_check(dut, model, 0, 0x8 | 0, CMD_TELEM, ctx="sat_clr")
    assert snap == 0xFF and alert == 0
    post, _, alert = await step_and_check(dut, model, 0, 0, CMD_TELEM, ctx="sat_post_clr")
    assert post == 0x00 and alert == 0


@cocotb.test()
async def test_mid_op_reset_and_unused_pin_immunity(dut):
    """Mid-operation reset clears RAM/counters/stall and restores scrub_en=1; uio_in[7:6] ignored."""
    model = EccSinglePortArbiterModel()
    await init_dut(dut)
    await FallingEdge(dut.clk)
    dut.rst_n.value = 1

    # Disable scrubber, write & corrupt addr 9 while toggling unused uio_in[7:6]
    await set_scrub_en(dut, model, 0, clear=1, ctx="mid_dis")
    await step_and_check(dut, model, 0xDE, 9, CMD_WRITE, unused_hi=0b11, ctx="mid_wr")
    await step_and_check(dut, model, 3, 9, CMD_INJECT, unused_hi=0b10, ctx="mid_inj")
    val, stall, alert = await step_and_check(dut, model, 0, 9, CMD_READ, unused_hi=0b01, ctx="mid_rd")
    assert (val, stall, alert) == (0xDE, 0, 1)

    # Assert synchronous reset mid-operation
    dut.rst_n.value = 0
    await RisingEdge(dut.clk)
    await FallingEdge(dut.clk)
    model.reset()
    assert int(dut.uo_out.value) == 0 and int(dut.uio_out.value) == 0

    # Release reset: confirm scrub_en restored to 1 and addr 9 cleared to 0x00
    dut.rst_n.value = 1
    reg3, _, alert = await step_and_check(dut, model, 0, 3, CMD_TELEM, ctx="mid_post_reg3")
    assert reg3 == 0x02 and alert == 0
    val, _, alert = await step_and_check(dut, model, 0, 9, CMD_READ, ctx="mid_post_rd9")
    assert val == 0x00 and alert == 0


@cocotb.test()
async def test_randomized_golden_scoreboard_stress(dut):
    """
    3,000-cycle deterministic pseudo-random stress test exercising all commands,
    CSR scrub_en toggles, watchdog stalls with automatic host retries, fault injections,
    and periodic full-RAM + full-telemetry audits.
    """
    rng = random.Random(20261122)
    model = EccSinglePortArbiterModel()
    await init_dut(dut)
    await FallingEdge(dut.clk)
    dut.rst_n.value = 1

    # Seed all 16 RAM slots with scrubber disabled
    await set_scrub_en(dut, model, 0, clear=1, ctx="rand_init_dis")
    for addr in range(16):
        await step_and_check(dut, model, rng.randrange(256), addr, CMD_WRITE, ctx="rand_init")
    await set_scrub_en(dut, model, 1, ctx="rand_init_en")

    pending_retry = None
    stall_count = 0

    for cycle in range(3000):
        if pending_retry is not None:
            # Re-issue the exact stalled host command on the next cycle
            ui_val, addr, cmd = pending_retry
            unused_hi = rng.randrange(4)
            pending_retry = None
        else:
            cmd = rng.choices([CMD_READ, CMD_WRITE, CMD_INJECT, CMD_TELEM], weights=[42, 23, 18, 17])[0]
            addr = rng.randrange(16)
            ui_val = rng.randrange(256)
            unused_hi = rng.randrange(4)

        _, stall, _ = await step_and_check(
            dut, model, ui_val, addr, cmd, unused_hi, ctx=f"rand_c{cycle}"
        )
        if stall:
            stall_count += 1
            pending_retry = (ui_val, addr, cmd)

        # Every 300 cycles, pause scrubber briefly to audit all 4 telemetry regs and all 16 RAM slots
        if (cycle + 1) % 300 == 0 and pending_retry is None:
            saved_scrub_en = model.scrub_en
            await set_scrub_en(dut, model, 0, ctx=f"audit_dis_{cycle}")
            for r in range(4):
                await step_and_check(dut, model, 0, r, CMD_TELEM, ctx=f"audit_t{r}_{cycle}")
            for a in range(16):
                await step_and_check(dut, model, 0, a, CMD_READ, ctx=f"audit_ram{a}_{cycle}")
            await set_scrub_en(dut, model, saved_scrub_en, ctx=f"audit_res_{cycle}")

    assert stall_count > 0, "Randomized stress test never exercised a watchdog stall!"
    dut._log.info(
        f"3,000-cycle randomized scoreboard passed with 0 mismatches ({stall_count} watchdog stalls verified)!"
    )
