# ELFCORE - Architecture, SPI Map, and Testbench Flow

## Layout
- `ELFCORE_core.scala`, `ELFCORE.scala`: Chisel sources (core and top with PSP BlackBox).
- `ELFCORE_tb.scala`: ChiselTest harness (dataset-style training and inference).
- `async_serdes/PSP.v` (+ submodules): production SERDES used in synthesis.
  - Top-level selects between Scala PSP model and Verilog PSP BlackBox via `USE_VERILOG_PSP`.

## Async SerDes (PSP)
- Consists of `PtoS` (parallel→serial) and `StoP` (serial→parallel) with two‑phase handshakes:
  - `PtoS`: `ReqIn/AckIn` on the parallel side, `ReqOut/AckOut` on the serial side, `DataIn[36:0] → DataOut`.
  - `StoP`: `ReqIn/AckIn` on the serial side, `ReqOut/AckOut` on the parallel side, `DataIn → DataOut[36:0]`.
- Configurable delay controls: `d_m_setup`, `d_m_hold`, `d_ps_hold` (2‑bit each) select programmable path delays in submodules.
- Default bandwidth is 37 bits; timing uses `timescale 1ps/1ps` with nominal `period=5000` ps and `matched_delay=1000` ps in the standalone TB.

### Signals and handshake
- Two‑phase (toggle) protocol on each interface:
  - Sender toggles `Req` to signal a new transfer; receiver toggles `Ack` to accept.
  - Back‑pressure: before the next toggle, wait until `Req == Ack` on that interface.
- `rst` is active‑high and asynchronously initializes internal state to empty/idle.

### Parameters and delays
- `BW` in `PSP.v` sets width (default 37). Both `PtoS` and `StoP` use `OCCUPANCY=BW` internally.
- Delay knobs (2‑bit each, 0..3):
  - `d_m_setup`: extra delay on mouse‑trap request path (setup of data/control).
  - `d_m_hold`: extra delay on mouse‑trap acknowledge path (hold margin).
  - `d_ps_hold`: extra delay on P→S acknowledge aggregation (write trigger hold).
- Use plusargs in the TB to sweep these: `+D_M_SETUP`, `+D_M_HOLD`, `+D_PS_HOLD`.

### Data framing and bit order
- `PtoS` emits 1 bit per handshake on `ReqOut_PS`; `StoP` assembles one word per handshake on `ReqOut_SP`.
- Bit order depends on internal pointer wiring; verify with a known pattern (e.g., `100..0`, `010..0`) and adjust post‑processing if needed. The TB captures 37 serial bits into a 37‑bit word in order of arrival.

### Standalone SerDes testbench
`async_serdes/PSP_tb.sv` can run PS and/or SP flows with plusargs and user‑defined stimulus files.

Run examples:
```bash
# PS-only: feed 37-bit words; capture serialized output bits
vsim work.PSP_tb +DISABLE_SP +IN_PS=stim/ps_in.txt +OUT_PS=out/ps_out.txt \
  +D_M_SETUP=0 +D_M_HOLD=0 +D_PS_HOLD=0 +DRAIN_PS=50000 +TIMEOUT=200000

# SP-only: feed serial bits; capture 37-bit words
vsim work.PSP_tb +DISABLE_PS +IN_SP=stim/sp_in.txt +OUT_SP=out/sp_out.txt \
  +D_M_SETUP=0 +D_M_HOLD=0 +DRAIN_SP=10000

# Both paths (default files if not provided): test_PS.txt, sim_output_PS.txt, test_SP.txt, sim_output_SP.txt
vsim work.PSP_tb
```

Plusargs supported:
- `+DISABLE_PS`, `+DISABLE_SP`: disable a path.
- `+IN_PS=<file>`, `+OUT_PS=<file>`: PS input/output files.
- `+IN_SP=<file>`, `+OUT_SP=<file>`: SP input/output files.
- `+D_M_SETUP=<0..3>`, `+D_M_HOLD=<0..3>`, `+D_PS_HOLD=<0..3>`: programmable delays.
- `+DRAIN_PS=<ps>`, `+DRAIN_SP=<ps>`: post‑EOF drain before finish.
- `+TIMEOUT=<ps>`: optional simulation timeout.

Input file formats:
- PS input (`+IN_PS`): one 37‑bit binary word per line (e.g., `101...`), consumed per request toggle.
- SP input (`+IN_SP`): binary tokens (`0`/`1`) separated by whitespace; each token is one serial bit.

Example PS input (37‑bit words):
```text
0000000000000000000000000000000000001
1000000000000000000000000000000000000
1111111111111111111111111111111111111
```

Example SP input (serial bits):
```text
1 0 1 1 0 0 1
```

### Compile and run with Questa/ModelSim
```bash
cd ElfCore/async_serdes
vlib work
vlog +acc +define+SINTESI_RTL PSP_submodels.v PtoS.v StoP.v PSP.v PSP_tb.sv
vsim -c work.PSP_tb +DISABLE_SP +IN_PS=../stim/ps_in.txt +OUT_PS=../out/ps_out.txt \
  +D_M_SETUP=0 +D_M_HOLD=0 +D_PS_HOLD=0 +DRAIN_PS=50000 +TIMEOUT=200000 -do "run -all; quit"
```

### Waveform/debug tips
- Probe `ReqIn_*`, `AckIn_*`, `ReqOut_*`, `AckOut_*`, and `Data*` signals to verify two‑phase toggles line up.
- Check that each `ReqOut_PS` toggle increments the collected bit count and produces one `DataOut_PS` bit.
- For `StoP`, ensure a full word is produced only when `ReqOut_SP` toggles.

### Common pitfalls
- Driving both PS and SP paths without disabling one can cause overlapping resets or shared control in a single TB; use `+DISABLE_PS`/`+DISABLE_SP` to isolate paths when debugging.
- Mismatched bit order when comparing against golden files; validate with one‑hot patterns and adjust comparison.
- Forgotten file closure or no finish condition; the TB now closes files, drains, and finishes automatically, with optional `+TIMEOUT`.

## Architecture and Working Flow (detailed)
- Global FSM (one tick per phase transition unless pipelined):
  - IDLE: waits for `SAMPLE` and input.
  - PROP1: VI events enqueued (AER). VI→H1 sparse bursts issue READ→RMW to H1 `u`.
  - STEP1: H1 leak/decay, spike detect; update `u`, `thr`, `T_vi`, `T_h1`; write 4‑tap `T_hist`/`T_prev` rings (see below); accumulate layer-wise similarities.
  - PROP2: drain H1 spikes; H1→H2 sparse bursts READ→RMW H2 `u`.
  - STEP2: H2 leak/decay, spike detect; update `u`, `T_h1`; write `T_hist`/`T_prev` rings; accumulate similarities; update output activity (`outTrace` or `oneur_u`).
  - LEARN: hidden learners (stage 1) or output learner (stage 2); DST may run after hidden learners idle and cadence met.
  - SEND: serialize output (label-only or stream).
  - CONFIG: while `EN_CONF=1`, SPI owns memories.

- Neuron row (128 bits per row) layout (H1/H2):
  - [127:112] `alpha` (Q1.15 leak/decay)
  - [111:96]  `thr` (s16)
  - [95:84]   `T_vi`   (12-bit)
  - [83:72]   `T_h1`   (12-bit)
  - `T_hist`/`T_prev` four-tap rings (12-bit each) packed at offsets:
    - Base slots at [71:60] (`T_hist`) and [59:48] (`T_prev`)
    - Additional taps at +16b, +32b, +48b (i.e., [87:76], [103:92], [119:108] and [75:64], [91:80], [107:96])
  - Rings advance modulo‑4; `T_prev` pointer is snapshotted on `SAMPLE` falling edge for a full sweep window.

- Sparse synapse word (4096×136b memories):
  - 8 slots × 17b each: `{idx[8:0], w[7:0]}`
  - Slot→group: `group = slot % 4`; presyn bank must equal slot group (asserted during learning updates).

- Similarity computation (per hidden layer, per STEP sweep):
  - Accumulators `sim_pred_X += T_hist · spike`, `sim_contr_X += T_prev · spike` (saturating 32‑bit add).
  - Post partial gradient (for DST scoring) uses `|(T_hist−T_prev)·STE(u)|` (24‑bit magnitude).
  - Gating derives `do_pred/do_contr` from scaled similarities vs input activity norm and per-layer thresholds.

- Hidden learners (unsupervised stage):
  - Two micro‑FSMs (L1 for VI→H1 dense weights; L2 for H1→H2 dense weights) run in parallel.
  - Process sparse word in halves (4 slots per half), coalesce updates into single 128‑bit masked write per half.
  - Three‑factor per‑slot update: `Δw ∝ (T_hist−T_prev)·STE(u_post)·trace_pre` → LR shift → clamp → stoch bump → saturating int8 write.
  - Zero‑skipping: all factors must be non‑zero; also requires valid slot, slot‑group match, similarity gate and global input gate.

- DST (per layer):
  - During LEARN, after hidden learners idle and cadence counters `cnt_steps_dst_* ≥ cfg_dyn_x_*`, run prune/regrow over a scan window; H1/H2 can run concurrently.
  - Prune: streaming top‑K of smallest |w| (K configurable).
  - Regrow: per‑group heaps fed by post partial gradient scores during STEP; total regrow capacity matches prune K across groups; basic collision handling.

- Output layer (supervised stage):
  - Output activity from `outTrace` (Q1.15) or `oneur_u` s16 integrator (KAPPA leak with stochastic rounding).
  - Three‑factor supervised update of `wout` using `error · STE(y_post) · preOutLane` (12‑bit presyn per lane), with stochastic ±1 byte updates.

- Input-activity normalization and gating:
  - `i_norm` uses aer-count per tick scaled by `cfg_i_scale/cfg_i_shift` and optionally normalized by number of outputs using a reciprocal-LUT (supports up to 256 outputs).
  - A global gate `cfg_thr_i` must be met for hidden updates.

- Stage gating (no e‑prop dependency):
  - Stage 1 (unsupervised): hidden learners + DST enabled; output learner disabled.
  - Stage 2 (supervised): hidden learners + DST disabled; output learner enabled via `cfg_sup_out_en`.

## SPI Configuration Map (subset)
Config code: `0000b`. Extended subpages enabled by `addr16[13]=1`. Readbacks via cfg-read or diagnostics (`code=1111b`).

- Base flags (write header addr16=0):
  - `EN_CONF`(bit0), `AER_DB`(bit1), `LABEL_SMP`(bit2), `SEND_LABEL_ONLY`(bit3), `SEND_PER_TIMESTEP`(bit4), sizes: `NUM_OUT/REC/INP`.

- Subpage 1 (readback):
  - flags: `cfg_timing_mode`, `cfg_regression`, `cfg_out_ste_en`, `cfg_stream_act`, `cfg_pipelined`, `cfg_sup_out_en`, `cfg_use_oneur`.

- Output/activation options:
  - Subpage 39 (w): `cfg_use_oneur`.
  - Subpage 40 (r/w): `cfg_sv_act_map`.
  - Subpages 52..67 (r/w): `cfg_out_bias8_vec[0..15]`; Subpage 33 (w): global `cfg_out_bias8`.

- Similarity, input gating, normalization:
  - Subpage 13 (w/r): `cfg_sim_scale` (Q16.0)
  - Subpage 14 (w/r): `cfg_sim_shift`
  - Subpage 15 (w/r): `cfg_i_scale` (Q16.0)
  - Subpage 16 (w/r): `cfg_i_shift`
  - Subpages 17/18 (w/r): `cfg_h1_pred_gate`/`cfg_h1_contr_gate`
  - Subpages 19/20 (w/r): `cfg_h2_pred_gate`/`cfg_h2_contr_gate`
  - Subpage 21 (w/r): `cfg_sim_norm_mode`
  - Subpage 55 (w): `cfg_thr_i` (global input activity gate)

- Hidden gradient clamps:
  - Subpages 18/19 (w/r): `cfg_h1_grad_max`/`cfg_h2_grad_max`

- Fixed-point locations (read/write):
  - Subpages 49..54: `cfg_fp_loc_tinp/trec/tout`, `cfg_fp_loc_winp/wrec/wout`

- Timing and seeds:
  - Subpage 41 (w/r): `cfg_cycles_per_tick`
  - Subpage 42 (w/r): `cfg_local_tick`, `cfg_error_halt`
  - Subpages 97/98 (r): `cnt_steps_dst_h1/h2` (hidden commit counters)
  - Subpages 98/99/100 (w/r): `cfg_seed_noise/strnd` and related seeds

- Stage mode (write/read):
  - Subpage 104: `cfg_stage_mode` (0=unsup, 1=sup, 2=auto by `cfg_sup_out_en`)
  - Subpage 34: `cfg_sup_out_en` (enable supervised output learner)

- DST (remapped to avoid conflicts):
  - Subpages 90/91 (w/r): `cfg_dyn_en_h1/h2`
  - Subpages 92/93 (w/r): `cfg_dyn_x_h1/h2` (cadence thresholds)
  - Subpages 94/95 (w/r): `cfg_prune_k_h1/h2`
  - Subpages 37/38 (w/r): `cfg_dst_scan_h1/h2`
  - Subpage 96 (r/w): `cfg_seed_dst`

Diagnostics (code `1111b`):
  - Subpage 6: `aer_cnt_tick`
  - Subpages 7..10: latched similarities
  - Subpages 11/12: H1/H2 spike counts per tick
  - Subpages 15/16: prune counters H1/H2
  - bit 14 of addr16 selects winner readback: `{idx, value}`

## Testbench Working Flow (`ELFCORE_tb.scala`)
1) Parameter initialization stage (CONFIG): programs timing, similarity/i-gating, DST cadence/K/scan/seeds, stage mode, and `cfg_sup_out_en`.
2) Unsupervised training stage (more epochs): sets stage mode 0; runs multiple samples per epoch with AER only; optionally enables DST; verifies hidden commit counters increased.
3) Supervised training stage (fewer epochs): sets stage mode 1; provides labels + AER; consumes SEND; checks hidden commit counters remain frozen.
4) Inference stage: labels hidden from core; evaluates accuracy by reading winner via diagnostics; drains SEND handshakes.

### What the test asserts/observes
- CONFIG state reflects `SPI_RDY` high; clears on exit.
- PROP/STEP activity via diagnostics: `aer_cnt_tick`, spike counts, latched similarities.
- DST activity via prune counters and hidden commit counters before/after stages.
- SEND handshakes occur in supervised stage and during inference.

### Helper formulas used in the bench
- `subpageAddr(N) = (1<<13) | ((N & 0x1F) << 8)` for 0..31 window pages; higher pages use same pattern (bench emits proper headers).

## Run ChiselTest
```bash
cd ElfCore
sbt test
```

## Synthesis vs Simulation
- In simulation, the top uses a Scala PSP model; synthesis uses the real `PSP.v` BlackBox (HasBlackBoxPath) and async SERDES submodules.
- SRAM wrappers implement 2-cycle read-modify-write to honor bit-level masks for sub-byte fields.
- Add CDC timing exceptions for PSP domains (StoP/PtoS) in STA; resets/delays are explicitly wired in the top.
 - Testbench now uses a deterministic RNG seed for reproducibility.
