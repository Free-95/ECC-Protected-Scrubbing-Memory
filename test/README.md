# Testbench for Hamming(13,8) SECDED Error Corrector & Fault Injector

This directory contains the [Cocotb](https://docs.cocotb.org/en/stable/) verification suite for `tt_um_secded_13_8` (`../src/secded_13_8.v`).

## Files

- `test_secded.py` — Exhaustive Cocotb testbench with an independent Python golden model
- `tb.v` — Verilog testbench wrapper (handles `tb.fst` waveform dumping and `GL_TEST` power pins)
- `Makefile` — Runs both RTL and Gate-Level Simulations via Icarus Verilog
- `tb.gtkw` — Preconfigured GTKWave signal layout for `tb.user_project`
- `requirements.txt` — Python test dependencies (`cocotb==2.1.0`, `pytest==8.4.2`)

## Verification Coverage

1. **`test_reset_and_pin_directions`** — Checks `uio_oe == 8'b1110_0000` and verifies synchronous reset holds outputs at `0`.
2. **`test_exhaustive_truth_table_streaming`** — Streams all 8,192 input combinations (`256` data bytes × `32` fault states) back-to-back at 1 vector/cycle, verifying 1-cycle latency, one-hot status flags, out-of-range `err_pos` handling (`0`, `14`, `15`), and no miscorrection on double-bit errors.
3. **`test_mid_operation_reset_and_recovery`** — Asserts `rst_n = 0` mid-stream during active SEC/DED states and confirms clean 1-cycle recovery.
4. **`test_unused_uio_inputs_immunity`** — Verifies that toggling unused upper `uio_in[7:5]` bits has zero effect on outputs.

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

First harden the design (or download the `gate_level_netlist.v` artifact from the GitHub Actions `gds` workflow) and copy the post-layout netlist into this folder:

```sh
cp ../runs/wokwi/results/final/verilog/gl/tt_um_secded_13_8.v gate_level_netlist.v
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

