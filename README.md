# YOLO11n INT8 Accelerator

Custom INT8 convolution accelerator for YOLO11n, written in SystemVerilog.
Modules communicate over unidirectional valid/ack handshakes (groundwork for
the LAGS async-style architecture).

## Architecture
| Module | Role |
|---|---|
| `reshape_im2col_engine_async` | Multiplier-free address generator; 9 spatial taps x 2 channel streams = 18 reads/cycle, halo zero-padding via masks |
| `sram_scratchpad` | 18-bank ping-pong BRAM (288 KB), 1-cycle read, sideband-aligned, elastic skid buffer |
| `matrix_engine_mxu_path_b` | 8 columns x 18 taps = 144 MACs, per-column weight ring buffer |
| `pe_column_18tap` | Pipelined dot-product column with row_last accumulator reset |
| VPU + SiLU ROM | Per-channel requantization and SiLU to INT8 |
| `dma_controller` | AXI4-HP read/write between DDR and scratchpad |
| `instruction_dispatcher`, `sync_barrier` | AXI4-Lite control and sequencing |

## Verification status
- [x] Single layer: 8x8x64 input, 3x3, stride 2, pad 1, 8 output channels
- [x] Raw INT32 accumulators: 128/128 match against Python golden
- [x] INT8 outputs after VPU: 128/128 match, MSE 0.0
- [x] AXI-Lite register readback and DMA read/write complete without deadlock
- [ ] Multiple shift values (including 0) and rounding corner cases
- [ ] Stride 1, 1x1 conv, odd Cin (RGB layer), larger Cin and spatial size
- [ ] Random `ack` / `vpu_ready` stalls
- [ ] Post-implementation (timing) simulation
- [ ] Multi-layer chaining and scratchpad layer merging
- [ ] Hardware bring-up on board

## Implementation snapshot (Vivado, post-implementation)
- Timing: all constraints met (WNS +0.652 ns, WHS +0.041 ns, 0 failing endpoints)
- Resources: 18,632 LUTs, 5,622 registers, 72 BRAM tiles, 18 DSPs
- Power: 0.58 W total (vectorless estimate, medium confidence)
- Target clock: 125 MHz (confirm against the constraint file)

## Known issues / notes
- Golden model must be generated from the original weights and image, never from `hw_output.hex`
- Output backpressure (`vpu_ready_in`) can drop in-flight results; needs an output skid
- Rounding offset is currently fixed (+32); should be `1 << (shift-1)`

## Repo layout
rtl/ · tb/ · scripts/ (stimulus + golden) · docs/
