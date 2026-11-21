# Testbench for 16x13-bit ECC-Protected Scrubbing Memory

This directory contains the [Cocotb](https://docs.cocotb.org/en/stable/) verification suite for `tt_um_ecc_prot_scrub_mem` (`../src/design_top.v`).

## Files

- `test_scrub_mem.py` — Sign-off Cocotb verification suite with an independent cycle-accurate Python golden scoreboard
- `tb.v` — Verilog testbench wrapper (handles `tb.fst` waveform dumping and `GL_TEST` power pins)
- `Makefile` — Runs both RTL and Gate-Level Simulations via Icarus Verilog
- `tb.gtkw` — Preconfigured GTKWave signal layout for `tb.user_project`
- `requirements.txt` — Python test dependencies (`cocotb==2.1.0`, `pytest==8.4.2`)

## Verification Coverage (`test_scrub_mem.py`)

1. **`test_reset_override_and_pin_directions`** — Checks `uio_oe == 8'b1000_0000`, verifies active writes/fault injections are ignored while `rst_n = 0`, and confirms all 16 RAM slots and 4 telemetry registers reset cleanly to `0`.
2. **`test_all_fault_positions_sec_ded_and_out_of_range`** — Tests all 16 `err_pos` values (`0`–`15`) for both `err_double = 0` and `err_double = 1` (including out-of-range `0`, `14`, `15` and `err_pos = 13` wrap-around), plus host reads on DED words without miscorrection.
3. **`test_cumulative_fault_injection_and_cancellation`** — Confirms two sequential 1-bit flips on the same bit cancel out cleanly, while two 1-bit flips on distinct bits escalate into an uncorrectable DED.
4. **`test_scrubber_pointer_pause_advance_and_telem_scrub`** — Verifies `scrub_ptr` advances and heals faults during `CMD_READ` and `CMD_TELEM` (`addr[3] = 0`) when `scrub_en = 1`, and strictly pauses during `CMD_WRITE`, `CMD_INJECT`, and telemetry clear (`addr[3] = 1`).
5. **`test_simultaneous_host_read_and_scrubber_collisions`** — Exercises same-cycle collisions where the host reads a faulted word while the scrubber heals either a different address (verifying scrubber priority on `last_*` telemetry) or the exact same address.
6. **`test_sec_and_ded_counter_saturation`** — Verifies both 4-bit `sec_cnt` and `ded_cnt` saturate at `15` (`4'hF`) without wrapping around, and tests atomic telemetry read-and-clear.
7. **`test_mid_op_reset_and_unused_pin_immunity`** — Asserts `rst_n = 0` mid-operation to verify full state recovery, and confirms toggling unused `uio_in[7]` and `addr[2]` (during telemetry reads) has zero side effects.
8. **`test_randomized_golden_scoreboard_stress`** — Runs a 2,000-cycle deterministic pseudo-random stress test checked cycle-by-cycle against Python golden model, with periodic full-RAM and full-telemetry audits every 250 cycles.

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
