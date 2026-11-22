# Testbench for 8x13-bit ECC-Protected Scrubbing Memory

This directory contains the [Cocotb](https://docs.cocotb.org/en/stable/) verification suite for `tt_um_ecc_prot_scrub_mem`.

## Files

- `test_scrub_mem.py` — Sign-off Cocotb verification testbench with an independent cycle-accurate Python golden reference model
- `tb.v` — Verilog testbench wrapper (handles `tb.fst` waveform dumping and `GL_TEST` power pins)
- `Makefile` — Runs both RTL and Gate-Level Simulations via Icarus Verilog
- `tb.gtkw` — Preconfigured GTKWave signal layout for `tb.user_project`
- `requirements.txt` — Python test dependencies (`cocotb==2.1.0`, `pytest==8.4.2`)

## Verification Coverage

1. **`test_reset_defaults_and_override`** — Checks `uio_oe == 8'b1100_0000`, verifies active writes/injections are ignored while `rst_n = 0`, confirms default `scrub_en = 1` after reset, and checks all 8 RAM slots reset cleanly to `0`.

2. **`test_csr_scrub_enable_disable_and_combined_clear`** — Verifies internal CSR transition semantics (`scrub_en` `1→0` final idle scrub, `0→1` next-cycle activation), combined `csr_we + telem_clr` (`uio_in[3:2] = 2'b11`), and `starve_cnt` hold behavior during `telem_clr`.

3. **`test_all_fault_positions_sec_ded_and_out_of_range`** — Tests all 16 `err_pos` values (`0`–`15`) for both `err_double = 0` and `err_double = 1` (including out-of-range `0`, `14`, `15` and `err_pos = 13` wrap-around), plus host reads on DED words without miscorrection.

4. **`test_cumulative_fault_injection_and_cancellation`** — Confirms two sequential 1-bit flips on the same bit cancel out cleanly, while two 1-bit flips on distinct bits escalate into an uncorrectable DED.

5. **`test_opportunistic_idle_scrubbing_and_permanent_healing`** — Injects 5 single-bit faults with `scrub_en = 0`, re-enables `scrub_en = 1`, sweeps all 8 slots via opportunistic `CMD_TELEM` idle cycles with zero stalls, and verifies all 5 words are permanently healed in RAM.

6. **`test_watchdog_cycle_steal_and_stall_handshake`** — Proves starvation prevention when `scrub_en = 1`: after 15 consecutive host memory ops, the 16th triggers a mandatory 1-cycle scrub with `stall = 1`, holds `uo_out`, protects RAM against the preempted op, and accepts the host retry on cycle 17 (tested for `CMD_WRITE`, `CMD_INJECT`, and `CMD_READ`).

7. **`test_no_stall_when_scrub_disabled`** — Confirms 32 consecutive host memory ops never assert `stall`, advance `scrub_ptr`, or increment `starve_cnt` while `scrub_en = 0`.

8. **`test_sec_and_ded_counter_saturation`** — Verifies both 4-bit `sec_cnt` and `ded_cnt` saturate at `15` (`4'hF`) without wrapping around, and tests atomic telemetry read-and-clear.

9. **`test_mid_op_reset_and_unused_pin_immunity`** — Asserts `rst_n = 0` mid-operation to verify full state recovery (clearing RAM/counters/stall and restoring `scrub_en = 1`), and confirms toggling unused `uio_in[7:6]` has zero side effects.

10. **`test_randomized_golden_scoreboard_stress`** — Runs a 3,000-cycle deterministic pseudo-random stress test exercising all commands, CSR toggles, watchdog stalls with automatic host retries, and periodic full-RAM (8 slots) and full-telemetry audits every 300 cycles against `EccSinglePortArbiterModel` (Python golden reference model).

## Prerequisites

Install Icarus Verilog (`iverilog`) and the required Python packages:

```sh
pip install -r requirements.txt
```

## How to Run

### 1. RTL Simulation

From inside the `test/` directory:

```sh
make -B
```

### 2. Gate-Level Simulation (GLS)

First harden the design (or download the powered netlist present in `tt_submission` artifact from the GitHub Actions `gds` workflow) and copy the netlist into this folder:

```sh
make -B GATES=yes
```

*(Requires `PDK_ROOT` to point to your local `sky130A` PDK installation; this runs automatically in GitHub Actions.)*

### 3. Optional VCD Waveform Output

By default, simulations generate a compact `tb.fst` file. To generate a `.vcd` file instead, change `$dumpfile("tb.fst");` to `$dumpfile("tb.vcd");` in `tb.v` and run:

```sh
make -B FST=
```

## Viewing Waveforms

Using GTKWave (loads the preconfigured pin groups automatically):

```sh
gtkwave tb.fst tb.gtkw
```

Using Surfer:

```sh
surfer tb.fst
```
