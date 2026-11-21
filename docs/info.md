## How it works

This project implements an inline **Hamming(13,8) Single-Error-Correcting, Double-Error-Detecting (SECDED) hardware codec** with a built-in physical fault injector. It demonstrates the same Error-Correcting Code (ECC) mechanism used in server DDR memory and spacecraft telemetry without requiring an external encoder.

```text
  ui_in[7:0] (data_in)
           │
                  ▼
┌─────────────────────┐
│  hamming13_encoder  │  Computes P1, P2, P4, P8 + overall even parity P0 (13-bit codeword)
└──────────┬──────────┘
                  ▼
┌─────────────────────┐
│   fault_injector    │◄── uio_in[3:0] (err_pos), uio_in[4] (err_double)
└──────────┬──────────┘
                  ▼
┌─────────────────────┐
│    syndrome_calc    │  Computes 4-bit syndrome S[3:0] and overall parity p_tot
└──────────┬──────────┘
                  ▼
┌─────────────────────┐
│  secded_corrector   │  Flips single-bit errors; flags uncorrectable double-bit errors
└──────────┬──────────┘
                  ▼
┌─────────────────────┐
│   Output Register   │  Registered on posedge clk (1-cycle latency, synchronous rst_n)
└──────────┬──────────┘
           ├──► uo_out[7:0]  (corrected data_out)
           └──► uio_out[7:5] (parity_ok, ded_flag, sec_flag)

```

### Codeword Bit Mapping (`code[12:0]`)

The 13-bit extended Hamming codeword maps parity bits to power-of-two positions plus bit `0` for overall even parity (`P0`), allowing the 4-bit syndrome to index the corrupted bit directly:

| **Bit Index** | 12 | 11 | 10 | 9 | 8 | 7 | 6 | 5 | 4 | 3 | 2 | 1 | 0 |
| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |
| **Field** | `D7` | `D6` | `D5` | `D4` | `P8` | `D3` | `D2` | `D1` | `P4` | `D0` | `P2` | `P1` | `P0` |

### Decoding & Status Flags (`uio_out[7:5]`)

| **Condition** | `p_tot` | `syndrome` | `parity_ok` (`uio[7]`) | `ded_flag` (`uio[6]`) | `sec_flag` (`uio[5]`) | **Output Behavior** |
| --- | --- | --- | --- | --- | --- | --- |
| *No Error* | `0` | `0000` | 1 | 0 | 0 | `data_out == data_in` |
| *Single-Bit Error* | `1` | `0000`–`1100` | 0 | 0 | 1 | Corrupted bit flipped back; `data_out == data_in` |
| *Double-Bit Error* | `0` | Non-zero | 0 | 1 | 0 | Uncorrectable error flagged; no miscorrection applied |

---

## How to test

1. **Clock and Reset:** Apply a system clock (tested for 25–50 MHz) and pulse `rst_n` low to clear the output registers (`uo_out` and `uio_out[7:5]` reset to `0`).
2. **Zero-Error Pass-Through:** Set any 8-bit byte on `ui_in[7:0]` (for example, `0xA5`) and set `uio_in[4:0] = 5'b00000` (`err_pos = 0`, `err_double = 0`). After one clock cycle, `uo_out[7:0]` matches `0xA5` and `parity_ok` (`uio_out[7]`) goes high.
3. **Single-Bit Error Correction (SEC):** Keep `err_double` (`uio_in[4]`) at `0` and set `err_pos` (`uio_in[3:0]`) to any value from `1` to `13` to flip a bit inside the 13-bit codeword (`code[err_pos - 1]`). Verify that `uo_out[7:0]` remains completely unchanged (`0xA5`) while `sec_flag` (`uio_out[5]`) goes high. *(Values `0`, `14`, and `15` with `err_double = 0` inject no fault.)*
4. **Double-Bit Error Detection (DED):** Set `err_double` (`uio_in[4]`) to `1` with any `err_pos` value (`0`–`15`) to corrupt two distinct bits simultaneously. Verify that `ded_flag` (`uio_out[6]`) goes high while `sec_flag` and `parity_ok` stay low.

---

## External hardware

No external hardware is required—the design can be exercised directly using the Tiny Tapeout carrier board's onboard switches, RP2040 controller, and output display/LEDs. For convenient live bench demonstrations, an optional 5-position DIP switch (or jumper bank) on `uio[4:0]` and three status LEDs on `uio[7:5]` can be attached to the bidirectional Pmod header.

