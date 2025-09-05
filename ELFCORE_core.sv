// SPDX-License-Identifier: Apache-2.0 WITH SHL-2.1

// SFF.sv - ELFCORE Feed-Forward SNN Core
// Two hidden spiking layers (no recurrence).
// - Layer stack: Virtual Input (AER) -> Hidden1 (LIF) -> Hidden2 (LIF) -> Output (LI)
// - Four groups per hidden layer, each with dedicated 128x128 128-bit SRAM bank
// - Structured N:M sparsity for VI->H1 and H1->H2 via 4096x136-bit sparse synapse memories
// - Parallel synapse processing across the 4 groups using interleaved 8-synapse words

`timescale 1ns/1ps

module ELFCORE_core #(
    // Network sizes
    parameter int unsigned N_INP   = 256,     // Virtual input width (<= 256)
    parameter int unsigned N_H1    = 512,     // Hidden layer 1 width (<= 512)
    parameter int unsigned N_H2    = 512,     // Hidden layer 2 width (<= 512)
    parameter int unsigned N_OUT   = 16,      // Output width (<= 16)

    // Grouping (always 4 groups)
    parameter int unsigned NG      = 4,       // Number of groups (fixed to 4)
    parameter int unsigned G_H1    = 128,     // Group size for H1 (N_H1 must be NG*G_H1)
    parameter int unsigned G_H2    = 128,     // Group size for H2 (N_H2 must be NG*G_H2)

    // Sparsity N:M (applies to VI->H1 and H1->H2)
    parameter int unsigned N_PER_G = 4,       // N connections per group from one pre-neuron
    // Memory and numeric formats
    parameter int unsigned FP_LOC_W12   = 0,  // Left-shift for hidden weights accumulation
    parameter int unsigned FP_LOC_WOUT  = 0,  // Left-shift for output weights accumulation
    parameter int unsigned FP_LOC_TINP  = 0,  // Left-shift (radix) for input traces
    parameter int unsigned FP_LOC_TOUT  = 0,  // Left-shift (radix) for output traces
    parameter logic  [7:0] KAPPA        = 8'h7A, // Output trace leak factor (Q1.7)
    parameter bit          H2_REPACK    = 1'b1,  // If 1, H2 T_vi field is unused (kept reserved)
    // Local tick option
    parameter bit          LOCAL_TICK_EN     = 1'b0,
    parameter int unsigned CYCLES_PER_TICK   = 1000
) (
    input  logic                 clk,
    input  logic                 rst,

    // SPI slave interface
    input  logic                 SCK,
    input  logic                 MOSI,
    output logic                 MISO,

    // AER virtual input (events into H1)
    input  logic [$clog2(N_INP)-1:0] AERIN_ADDR,
    input  logic                      AERIN_REQ,
    output logic                      AERIN_ACK,

    // Timestep control
    input  logic                      SAMPLE,      // high during sample window
    input  logic                      TIME_TICK,   // advance one timestep on rising edge

    // Inference modes
    input  logic                      INFER_ACC,

    // Output bus (compatible with ReckOn simplified label streaming)
    output logic                      OUT_REQ,
    input  logic                      OUT_ACK,
    output logic [7:0]                OUT_DATA,

    // Supervised label input (classification index) for output-layer learning
    input  logic                      LABEL_VALID,
    input  logic [M_OUT-1:0]          LABEL_INDEX,

    // Diagnostics
    output logic                      SPI_RDY,
    output logic                      TIMING_ERROR_RDY
);

    // ------------------------------------------------------------------------
    // Derived and local parameters
    // ------------------------------------------------------------------------
    localparam int unsigned M_INP  = (N_INP  <= 1)   ? 1 : $clog2(N_INP);
    localparam int unsigned M_H1   = (N_H1   <= 1)   ? 1 : $clog2(N_H1);
    localparam int unsigned M_H2   = (N_H2   <= 1)   ? 1 : $clog2(N_H2);
    localparam int unsigned M_OUT  = (N_OUT  <= 1)   ? 1 : $clog2(N_OUT);
    localparam int unsigned M_GH1  = (G_H1   <= 1)   ? 1 : $clog2(G_H1);  // local index within group
    localparam int unsigned M_GH2  = (G_H2   <= 1)   ? 1 : $clog2(G_H2);

    // Sanity: NG fixed to 4, banks per layer = 4, group select = top 2 bits of 9-bit index
    // Virtual Input grouping for VI traces stored in H1 banks
    localparam int unsigned G_INP  = (N_INP/NG);
    localparam int unsigned M_GINP = (G_INP <= 1) ? 1 : $clog2(G_INP);
    // H1 bank address offset where VI traces are stored; default maps VI into top rows
    localparam int unsigned VI_BASE_DFLT = 128 - G_INP;
    logic [M_GH1-1:0] SPI_VI_BASE = VI_BASE_DFLT; // SPI-configurable for flexibility
    // Combined neuron SRAM layout (per row, per group):
    // [127:0]   neuron core
    // [223:128] history ring (8 taps x 12b at 16b stride)
    // [319:224] prev ring (8 taps x 12b at 16b stride)
    localparam int unsigned HIST_BASE = 128;
    localparam int unsigned PREV_BASE = 224;
    // Postsynaptic index within sparse synapses is 9-bit (0..511), bank = [8:7], local = [6:0]

    // ------------------------------------------------------------------------
    // State and memory structures
    // ------------------------------------------------------------------------

    // Hidden neuron banks: 4 banks per hidden layer, one neuron per 128b word
    // Word format (per neuron), H1 has T_vi and T_h1; both H1/H2 add T_hist and T_prev snapshots:
    // [127:112] alpha (Q1.15, 16b)
    // [111:96]  thr   (signed 16b)
    // [95:84]   T_vi  (12b, only used in H1)
    // [83:72]   T_h1  (12b)
    // [71:60]   T_hist (12b, last timestep's T_h1)
    // [59:48]   T_prev (12b, snapshot of previous sample's tail)
    // [79:64]   u     (membrane, signed 16b)
    // [47:0]    reserved

    // Bank interfaces for H1
    logic                h1_cs   [NG-1:0];
    logic                h1_we   [NG-1:0];
    logic [M_GH1-1:0]    h1_addr [NG-1:0];
    logic [319:0]        h1_di   [NG-1:0];
    logic [319:0]        h1_mask [NG-1:0];
    logic [319:0]        h1_do   [NG-1:0];
    // Overlay ports for SPI programming/readback
    logic                h1_cs_i [NG-1:0];
    logic                h1_we_i [NG-1:0];
    logic [M_GH1-1:0]    h1_addr_i [NG-1:0];
    logic [319:0]        h1_di_i   [NG-1:0];
    logic [319:0]        h1_mask_i [NG-1:0];

    // Bank interfaces for H2
    logic                h2_cs   [NG-1:0];
    logic                h2_we   [NG-1:0];
    logic [M_GH2-1:0]    h2_addr [NG-1:0];
    logic [319:0]        h2_di   [NG-1:0];
    logic [319:0]        h2_mask [NG-1:0];
    logic [319:0]        h2_do   [NG-1:0];
    logic                h2_cs_i [NG-1:0];
    logic                h2_we_i [NG-1:0];
    logic [M_GH2-1:0]    h2_addr_i [NG-1:0];
    logic [319:0]        h2_di_i   [NG-1:0];
    logic [319:0]        h2_mask_i [NG-1:0];

    // Sparse synapse memories (behavioral): VI->H1 and H1->H2
    // 4096 x 136-bit. Eight synapses per word: each synapse = {idx[8:0], w[7:0]} (17b)
    // Interleaved: slots 0..3 map to groups 0..3, slots 4..7 map again to groups 0..3.
    logic         sp_vi_h1_cs,  sp_vi_h1_we;
    logic [11:0]  sp_vi_h1_addr;
    logic [135:0] sp_vi_h1_di;
    logic         sp_vi_h1_do_valid;
    logic [135:0] sp_vi_h1_do;
    // Learning control for sparse words per presyn (configurable)
    // N per group for N:M sparsity; words per presyn derived as ceil((4N)/8) = ceil(N/2)
    logic [7:0]   SPI_N_PER_G             = 8'd4;

    logic         sp_h1_h2_cs,  sp_h1_h2_we;
    logic [11:0]  sp_h1_h2_addr;
    logic [135:0] sp_h1_h2_di;
    logic         sp_h1_h2_do_valid;
    logic [135:0] sp_h1_h2_do;
    // Derived words per presyn per layer
    wire  [3:0]   WORDS_PER_PRE_H1 = (SPI_N_PER_G[0]) ? ((SPI_N_PER_G + 8'd1) >> 1) : (SPI_N_PER_G >> 1);
    wire  [3:0]   WORDS_PER_PRE_H2 = WORDS_PER_PRE_H1;

    // Output weights SRAM: 512 x 128 (first N_H2 words used), one row per presyn Hidden2 neuron
    logic         wout_cs, wout_we;
    logic [8:0]   wout_addr;       // up to 512 rows
    logic [127:0] wout_di, wout_do, wout_mask;
    logic         wout_cs_i, wout_we_i;
    logic [8:0]   wout_addr_i;
    logic [127:0] wout_di_i, wout_mask_i;

    // Output accumulators (LI); output traces removed per spec
    logic signed [15:0] out_u    [N_OUT-1:0];
    // Output-layer supervised learning controls and state
    logic        SPI_SUP_OUT_EN     = 1'b0;    // enable output-layer supervised updates
    logic [7:0]  SPI_LR_OUT         = 8'd16;   // Q0.8 learning rate for output weights
    logic        SPI_OUT_STOCH_EN   = 1'b1;    // enable stochastic update at output
    logic [3:0]  SPI_OUT_ERR_SCALE  = 4'd0;    // optional right shift on error
    logic [15:0] SPI_OUT_ERR_MAX    = 16'd32767; // clamp error magnitude
    logic        SPI_OUT_STE_EN     = 1'b1;    // apply STE on output path
    logic [7:0]  SPI_OUT_PRE_SCALE  = 8'd255;  // Q0.8 scaling for presyn factor (default 1.0)
    logic [7:0]  SPI_OUT_WDEC       = 8'd0;    // Q0.8 weight decay lambda
    logic        SPI_OUT_LEARN_USE_ACT = 1'b1; // use activation for learning (hard-sigmoid), else raw out_u
    logic [15:0] SPI_OUT_GRAD_MAX   = 16'd32767; // gradient clamp before LR
    logic        SPI_LABEL_USE_SAMPLE = 1'b1;  // if 0, label_active follows LABEL_VALID
    // Latched label over sample window
    logic        label_active;
    logic [M_OUT-1:0] label_idx_reg;
    // Optional bias per output (8-bit signed)
    logic signed [7:0] out_bias8 [N_OUT-1:0];
    // Output error snapshot for SPI readback
    logic signed [15:0] out_err16 [N_OUT-1:0];
    // Hidden learner gradient clip
    logic [15:0] SPI_H1_GRAD_MAX = 16'd32767;
    logic [15:0] SPI_H2_GRAD_MAX = 16'd32767;
    // Refractory/Homeostasis knobs (soft-threshold adaptation)
    logic [7:0]  SPI_REF_ADD_H1 = 8'd0, SPI_REF_DEC_H1 = 8'd0;
    logic [7:0]  SPI_REF_ADD_H2 = 8'd0, SPI_REF_DEC_H2 = 8'd0;
    // Output learner wiring
    logic        ol_start, ol_busy, ol_done;
    logic [8:0]  ol_pre_idx;
    logic [127:0] ol_row_in, ol_row_out;

    // Similarity accumulators per layer per tick
    // Predictive: dot(spike, T_hist tap); Contrastive: dot(spike, T_prev tap)
    logic [31:0] sim_pred_h1, sim_contr_h1;
    logic [31:0] sim_pred_h2, sim_contr_h2;
    // Per-sweep spike counters
    logic [15:0] h1_spk_cnt_tick, h2_spk_cnt_tick;
    // Latched end-of-sweep values for SPI
    logic [31:0] sim_pred_h1_latched,  sim_contr_h1_latched;
    logic [31:0] sim_pred_h2_latched,  sim_contr_h2_latched;
    logic [15:0] h1_spk_cnt_latched,   h2_spk_cnt_latched;
    logic [31:0] sim_pred_h1_scaled,   sim_contr_h1_scaled;
    logic [31:0] sim_pred_h2_scaled,   sim_contr_h2_scaled;
    // Scaling config (common for both layers)
    logic [15:0] SPI_SIM_SCALE = 16'd1; // Q16.0 multiplier
    logic [4:0]  SPI_SIM_SHIFT = 5'd0;  // right shift after multiply
    // Normalization mode: 0=none, 1=divide by spike count, 2=divide by neuron count
    logic [1:0]  SPI_SIM_NORM_MODE = 2'd0;
    // Fixed-point normalization for input activity i
    logic [15:0] SPI_I_SCALE = 16'd1; // Q16.0 multiplier for i
    logic [4:0]  SPI_I_SHIFT = 5'd0;  // right shift for i
    // Learning rates (Q0.8)
    logic [7:0]  SPI_LR_H1 = 8'd0;
    logic [7:0]  SPI_LR_H2 = 8'd0;
    // STE LUT programming strobes
    logic        SPI_STE_WE;
    logic [7:0]  SPI_STE_IDX, SPI_STE_DATA;
    // STE LUT (Q0.8)
    logic [7:0]  ste_lut [0:255];
    always_ff @(posedge clk or posedge rst) begin
        if (rst) begin
            for (int i=0; i<256; i++) ste_lut[i] <= 8'd0;
        end else if (SPI_STE_WE) begin
            ste_lut[SPI_STE_IDX] <= SPI_STE_DATA;
        end
    end
    // Threshold configuration
    logic [15:0] SPI_THR_I     = 16'd0;  // global input threshold
    // Per-layer scaling
    logic [15:0] SPI_H1_PRED   = 16'd0;
    logic [15:0] SPI_H1_CONTR  = 16'd0;
    logic [15:0] SPI_H2_PRED   = 16'd0;
    logic [15:0] SPI_H2_CONTR  = 16'd0;
    // Layer gating outputs (no update flags)
    logic        H1_NO_UPDATE, H2_NO_UPDATE;
    // Tap selects (0..7): override SPI_THIST_DEPTH/Tprev? Use same regs for now
    wire [2:0] tap_hist_sel = SPI_THIST_DEPTH; // 0..7 -> 1..8 steps back
    wire [2:0] tap_prev_sel = SPI_TPREV_DEPTH; // 0..7 -> 1..8 steps back

    // Hidden spikes FIFOs (to drive next layer sparse propagation)
    // Carry 9-bit absolute index (0..511) of spiking neuron
    logic        h1_spike_push, h1_spike_pop, h1_spike_empty;
    logic [8:0]  h1_spike_in,   h1_spike_out;
    logic        h2_spike_push, h2_spike_pop, h2_spike_empty;
    logic [8:0]  h2_spike_in,   h2_spike_out;

    // Event queueing for AER input (1-hot per address is not needed; just process events when they arrive)
    logic aer_req_q, aer_req_qq;
    logic time_tick_q, time_tick_qq;

    // FSM
    typedef enum logic [2:0] {S_IDLE, S_PROP1, S_STEP1, S_PROP2, S_STEP2, S_LEARN, S_SEND} state_t;
    state_t state, next_state;

    // Per-layer scan counters for leak/step
    logic [M_GH1-1:0] step_h1_idx;
    logic [M_GH2-1:0] step_h2_idx;
    logic [1:0]       step_grp; // 0..3

    // Winner selection
    logic [M_OUT-1:0] winner_idx;
    logic [15:0]      cnt_winner [N_OUT-1:0];
    logic signed [15:0] act_y [N_OUT-1:0];

    // ------------------------------------------------------------------------
    // Configuration registers (via SPI)
    // ------------------------------------------------------------------------
    // Defaults emulate current parameterization; SPI can override at runtime
    logic        SPI_EN_CONF           = 1'b0;
    logic        SPI_LOCAL_TICK        = 1'b0;
    logic        SPI_ERROR_HALT        = 1'b0;
    logic        SPI_TIMING_MODE       = 1'b0;
    logic        SPI_REGRESSION        = 1'b0;
    logic        SPI_SEND_PER_TIMESTEP = 1'b0;
    logic        SPI_SEND_LABEL_ONLY   = 1'b1;
    logic        SPI_PIPELINED         = 1'b0; // 0: fall-through, 1: pipelined
    // Configurable delay depths (0..7) => select 1..8 steps
    logic [2:0]  SPI_THIST_DEPTH       = 3'd0;
    logic [2:0]  SPI_TPREV_DEPTH       = 3'd0;
    // SPI-selected bank indices for neuron SRAM monitor/config
    logic [1:0]  SPI_BANK_SEL_H1       = 2'd0;
    logic [1:0]  SPI_BANK_SEL_H2       = 2'd0;
    logic        SPI_STREAM_ACT        = 1'b0; // stream activated y[k] instead of raw u
    logic        SPI_NO_OUT_ACT        = 1'b0;
    logic        SPI_EN_STOCH_ROUND    = 1'b0;
    logic        SPI_NOISE_EN          = 1'b0;
    logic [3:0]  SPI_NOISE_STR         = 4'd0;
    logic [31:0] SPI_CYCLES_PER_TICK   = 32'd1000;
    logic [7:0]  SPI_SRAM_SPEEDMODE_H1 [NG-1:0];
    logic [7:0]  SPI_SRAM_SPEEDMODE_H2 [NG-1:0];
    logic [2:0]  EFF_FP_LOC_TINP       = FP_LOC_TINP[2:0];
    logic [2:0]  EFF_FP_LOC_TOUT       = FP_LOC_TOUT[2:0];
    logic [2:0]  EFF_FP_LOC_WOUT       = FP_LOC_WOUT[2:0];
    logic [M_INP-1:0] SPI_NUM_INP_NEUR = N_INP-1;
    logic [M_H1 -1:0] SPI_NUM_H1_NEUR  = N_H1-1;
    logic [3:0]       SPI_NUM_OUT_NEUR = N_OUT-1;
    logic [14:0]      SPI_SEED_STRND   = 15'h1234;
    logic [16:0]      SPI_SEED_NOISE   = 17'h1F0F1;
    // DST controls
    logic             SPI_DYN_EN_H1    = 1'b0;
    logic             SPI_DYN_EN_H2    = 1'b0;
    logic [15:0]      SPI_DYN_X_H1     = 16'd0;
    logic [15:0]      SPI_DYN_X_H2     = 16'd0;
    logic [7:0]       SPI_PRUNE_K_H1   = 8'd8;
    logic [7:0]       SPI_PRUNE_K_H2   = 8'd8;
    logic             SPI_COLLISION_EN = 1'b1;
    logic [16:0]      SPI_SEED_DST     = 17'h35A5;

    // Simple perf/diag counters
    logic [31:0] CNT_TICKS;
    logic [15:0] CNT_SPIKES_H1, CNT_SPIKES_H2;
    logic [7:0]  AER_FIFO_HWM;
    // DST step counters
    logic [15:0] steps_since_dst_h1, steps_since_dst_h2;
    // DST streaming heaps and handshakes
    typedef struct packed {logic [11:0] addr; logic [2:0] slot; logic [1:0] group;} syn_meta_t;
    // PRUNE heaps per layer (K-min by |w|)
    logic              pr_in_valid1, pr_in_last1, pr_done1, pr_pop_req1, pr_pop_valid1, pr_empty1;
    logic [7:0]        pr_in_val1, pr_pop_val1;
    syn_meta_t         pr_in_meta1, pr_pop_meta1;
    logic [7:0]        pr_k1, rewire_k1;
    logic [11:0]       prune_addr1; logic prune_pending1; logic [135:0] prune_word1;
    assign pr_k1 = SPI_PRUNE_K_H1;
    logic              pr_in_valid2, pr_in_last2, pr_done2, pr_pop_req2, pr_pop_valid2, pr_empty2;
    logic [7:0]        pr_in_val2, pr_pop_val2;
    syn_meta_t         pr_in_meta2, pr_pop_meta2;
    logic [7:0]        pr_k2, rewire_k2;
    logic [11:0]       prune_addr2; logic prune_pending2; logic [135:0] prune_word2;
    assign pr_k2 = SPI_PRUNE_K_H2;
    // REGROW grouped heaps (per-group top-N by |(T_hist-T_prev)*STE|)
    logic              rg_in_valid1, rg_in_last1, rg_done1, rg_pop_req1, rg_pop_valid1, rg_empty1;
    logic [23:0]       rg_in_score1; logic [8:0] rg_in_index1, rg_pop_index1; logic [1:0] rg_pop_group1;
    logic [8:0]        regrow_idx1; logic regrow_pending1;
    logic              rg_in_valid2, rg_in_last2, rg_done2, rg_pop_req2, rg_pop_valid2, rg_empty2;
    logic [23:0]       rg_in_score2; logic [8:0] rg_in_index2, rg_pop_index2; logic [1:0] rg_pop_group2;
    logic [8:0]        regrow_idx2; logic regrow_pending2;
    // Collision micro-FSMs for rewiring (per layer)
    typedef enum logic [2:0] {C_IDLE, C_REQ, C_WAIT, C_REQ_ORIG, C_WAIT_ORIG, C_NEW} cstate_t;
    cstate_t           coll1_state, coll2_state;
    // H1
    logic [M_INP-1:0]  coll1_pre;
    logic [1:0]        coll1_group;
    logic [8:0]        coll1_new_j;
    logic [$clog2(WORDS_PER_PRE_H1):0] coll1_word;
    logic [2:0]        coll1_slot;
    logic [7:0]        coll1_try;
    logic              coll1_collide;
    logic              coll1_ready;
    syn_meta_t         coll1_sd;
    logic              rewire_wait1;
    logic              coll1_no_cand;
    logic [135:0]      coll1_origW;
    // H2
    logic [8:0]        coll2_pre;
    logic [1:0]        coll2_group;
    logic [8:0]        coll2_new_j;
    logic [$clog2(WORDS_PER_PRE_H2):0] coll2_word;
    logic [2:0]        coll2_slot;
    logic [7:0]        coll2_try;
    logic              coll2_collide;
    logic              coll2_ready;
    syn_meta_t         coll2_sd;
    logic              rewire_wait2;
    logic              coll2_no_cand;
    logic [135:0]      coll2_origW;
    // instantiate heaps
    heap_topk_stream #(.K(256), .W(8), .MW($bits(syn_meta_t)), .CMP_MAX(1'b0)) u_prune1 (
        .clk(clk), .rst(rst),
        .in_valid(pr_in_valid1), .in_value(pr_in_val1), .in_meta(pr_in_meta1), .in_last(pr_in_last1), .in_ready(),
        .done(pr_done1), .pop_req(pr_pop_req1), .pop_valid(pr_pop_valid1), .pop_value(pr_pop_val1), .pop_meta(pr_pop_meta1), .empty(pr_empty1)
    );
    heap_topk_stream #(.K(256), .W(8), .MW($bits(syn_meta_t)), .CMP_MAX(1'b0)) u_prune2 (
        .clk(clk), .rst(rst),
        .in_valid(pr_in_valid2), .in_value(pr_in_val2), .in_meta(pr_in_meta2), .in_last(pr_in_last2), .in_ready(),
        .done(pr_done2), .pop_req(pr_pop_req2), .pop_valid(pr_pop_valid2), .pop_value(pr_pop_val2), .pop_meta(pr_pop_meta2), .empty(pr_empty2)
    );
    heap_group_topk_stream #(.K(64), .W(24), .IW(9)) u_regrow1 (
        .clk(clk), .rst(rst),
        .in_valid(rg_in_valid1), .in_score(rg_in_score1), .in_index(rg_in_index1), .in_last(rg_in_last1), .in_ready(),
        .done(rg_done1), .pop_group(rg_pop_group1), .pop_req(rg_pop_req1), .pop_valid(rg_pop_valid1), .pop_index(rg_pop_index1), .empty(rg_empty1)
    );
    heap_group_topk_stream #(.K(64), .W(24), .IW(9)) u_regrow2 (
        .clk(clk), .rst(rst),
        .in_valid(rg_in_valid2), .in_score(rg_in_score2), .in_index(rg_in_index2), .in_last(rg_in_last2), .in_ready(),
        .done(rg_done2), .pop_group(rg_pop_group2), .pop_req(rg_pop_req2), .pop_valid(rg_pop_valid2), .pop_index(rg_pop_index2), .empty(rg_empty2)
    );
    // PRUNE streaming slot/staging registers
    logic [2:0] pr_slot1, pr_slot2;
    logic [135:0] pr_staging1, pr_staging2;
    // RNG for regrowth selection
    logic [16:0] rnd_dst;
    lfsr17 lfsr_dst (.clk(clk), .rst(rst), .en(1'b1), .seed(SPI_SEED_DST), .rnd(rnd_dst));
    // Simple DO-valid wrappers for sparse SRAMs
    sram_do_valid u_sram_vih1_v (.clk(clk), .rst(rst), .cs(sp_vi_h1_cs), .we(sp_vi_h1_we), .do_valid(sp_vi_h1_do_valid));
    sram_do_valid u_sram_h1h2_v (.clk(clk), .rst(rst), .cs(sp_h1_h2_cs), .we(sp_h1_h2_we), .do_valid(sp_h1_h2_do_valid));
    logic [7:0]  aer_fifo_level;
    // Optional debounce control for AER; default on
    logic        SPI_AER_DEBOUNCE;

    // ------------------------------------------------------------------------
    // Synchronizers and AER handshake
    // ------------------------------------------------------------------------
    // Simple AER debounce: require two consecutive cycles high
    logic aer_req_sync1, aer_req_sync2;
    always_ff @(posedge clk or posedge rst) begin
        if (rst) begin
            aer_req_q   <= 1'b0;
            aer_req_qq  <= 1'b0;
            aer_req_sync1 <= 1'b0;
            aer_req_sync2 <= 1'b0;
            time_tick_q <= 1'b0;
            time_tick_qq<= 1'b0;
        end else begin
            if (SPI_AER_DEBOUNCE) begin
                aer_req_sync1 <= AERIN_REQ;
                aer_req_sync2 <= aer_req_sync1;
            end else begin
                aer_req_sync1 <= AERIN_REQ;
                aer_req_sync2 <= AERIN_REQ;
            end
            aer_req_q     <= aer_req_sync2;
            aer_req_qq    <= aer_req_q;
            time_tick_q <= TIME_TICK;
            time_tick_qq<= time_tick_q;
        end
    end

    wire aer_rise      = aer_req_q & ~aer_req_qq;
    // Average input spikes at current timestep i: count of AER events during PROP1 burst window
    logic [7:0] aer_cnt_tick;
    always_ff @(posedge clk or posedge rst) begin
        if (rst) aer_cnt_tick <= 8'd0;
        else if (state == S_PROP1 && !vih1_active && aer_fifo_empty) aer_cnt_tick <= 8'd0; // clear at start of tick
        else if (state == S_PROP1 && aer_fifo_push) aer_cnt_tick <= aer_cnt_tick + 8'd1;
    end

    // ------------------------------------------------------------------------
    // LEARN: Online self-supervised update after STEP2
    // ------------------------------------------------------------------------
    // Common gates
    wire layer1_block = H1_NO_UPDATE;
    wire layer2_block = H2_NO_UPDATE;
    // normalized i for consistent gating
    wire [31:0] i_norm = (({24'd0,aer_cnt_tick} * SPI_I_SCALE) >> SPI_I_SHIFT);
    wire gate_pred1 = (sim_pred_h1_scaled > (i_norm * SPI_H1_PRED));
    wire gate_contr1= (sim_contr_h1_scaled < (i_norm * SPI_H1_CONTR));
    wire gate_pred2 = (sim_pred_h2_scaled > (i_norm * SPI_H2_PRED));
    wire gate_contr2= (sim_contr_h2_scaled < (i_norm * SPI_H2_CONTR));

    // VI->H1 learner
    typedef enum logic [1:0] {L_IDLE, L_READ_SP, L_RMW, L_NEXT} lstate_t;
    lstate_t l1_state;
    logic [M_INP-1:0]  l1_pre_cnt;
    logic [3:0]        l1_word;
    logic [135:0]      l1_word_do, l1_word_di;
    logic [2:0]        l1_syn_idx;
    logic              l1_syn_valid;
    logic [11:0]       l1_pre_val; // T_vi_i
    // staging for compute of previous scheduled synapse
    logic [8:0]        l1_post_idx_stg;
    logic signed [7:0] l1_w_stg;
    // 4-PE half-step control
    logic              l1_half;   // 0: syn0-3, 1: syn4-7
    logic              l1_phase;  // 0: schedule, 1: apply
    logic [8:0]        l1_post_idx_half [3:0];
    logic signed [7:0] l1_w_half       [3:0];

    // H1->H2 learner
    lstate_t l2_state;
    logic [8:0]        l2_pre_cnt; // 9-bit presyn index (0..SPI_NUM_H1_NEUR)
    logic [3:0]        l2_word;
    logic [135:0]      l2_word_do, l2_word_di;
    logic [2:0]        l2_syn_idx;
    logic              l2_syn_valid;
    logic [11:0]       l2_pre_val; // T_h1_i
    logic [8:0]        l2_post_idx_stg;
    logic signed [7:0] l2_w_stg;
    // 4-PE half-step control
    logic              l2_half;
    logic              l2_phase;
    logic [8:0]        l2_post_idx_half [3:0];
    logic signed [7:0] l2_w_half       [3:0];

    // Arbitration requests for H1 neuron SRAM (shared by learners)
    logic              h1_req_l1   [NG-1:0];
    logic              h1_req_l2   [NG-1:0];
    logic [M_GH1-1:0]  h1_addr_l1  [NG-1:0];
    logic [M_GH1-1:0]  h1_addr_l2  [NG-1:0];
    logic [NG-1:0]     h1_rr;
    // Per-bank valid strobes for each client (delayed grant)
    logic              h1_valid_l1 [NG-1:0];
    logic              h1_valid_l2 [NG-1:0];
    // Track which client was last granted for each bank and generate DO-valid per bank
    logic              h1_last_grant_is_l1 [NG-1:0];
    logic              h1_read_q           [NG-1:0];
    logic              h1_do_valid         [NG-1:0];

    // Start learners each tick after STEP2
    always_ff @(posedge clk or posedge rst) begin
        if (rst) begin
            l1_state <= L_IDLE; l2_state <= L_IDLE; l1_word <= '0; l2_word <= '0; l1_pre_cnt <= '0; l2_pre_cnt <= '0;
            l1_syn_idx <= '0; l1_syn_valid <= 1'b0; l2_syn_idx <= '0; l2_syn_valid <= 1'b0;
        end else if (state == S_LEARN) begin
            // Default deassert streaming valids/reqs each cycle
            pr_in_valid1 <= 1'b0; pr_in_last1 <= 1'b0; pr_pop_req1 <= 1'b0; rg_in_valid1 <= 1'b0; rg_in_last1 <= 1'b0; rg_pop_req1 <= 1'b0;
            pr_in_valid2 <= 1'b0; pr_in_last2 <= 1'b0; pr_pop_req2 <= 1'b0; rg_in_valid2 <= 1'b0; rg_in_last2 <= 1'b0; rg_pop_req2 <= 1'b0;
            // Pause learners if DST for that layer is active, or if output supervised learning is active
            if ((dst1_state != D_IDLE && dst1_state != D_DONE) || (SPI_SUP_OUT_EN && (label_active || ol_busy))) begin
                l1_state <= L_IDLE; l1_word <= '0; l1_pre_cnt <= '0; l1_half <= 1'b0; l1_phase <= 1'b0;
            end
            if ((dst2_state != D_IDLE && dst2_state != D_DONE) || (SPI_SUP_OUT_EN && (label_active || ol_busy))) begin
                l2_state <= L_IDLE; l2_word <= '0; l2_pre_cnt <= '0; l2_half <= 1'b0; l2_phase <= 1'b0;
            end
            // VI->H1
            case (l1_state)
                L_IDLE: begin
                    if (!layer1_block) begin
                        l1_pre_cnt <= '0;
                        l1_word  <= 4'd0;
                        l1_state <= L_READ_SP;
                    end
                end
                L_READ_SP: begin
                    // issue sparse read: presyn base address + word idx (extra cycle to fetch word)
                    sp_vi_h1_cs   <= 1'b1; sp_vi_h1_we <= 1'b0;
                    sp_vi_h1_addr <= {l1_pre_cnt, l1_word[$bits(sp_vi_h1_addr)-M_INP-1:0]};
                    l1_state      <= L_RMW; // next cycle latch DO
                    // request pre trace read (T_vi_i) once
                    for (int g=0; g<NG; g++) begin h1_req_l1[g] <= 1'b0; end
                    h1_req_l1[l1_pre_cnt[M_INP-1:M_INP-2]]  <= 1'b1;
                    h1_addr_l1[l1_pre_cnt[M_INP-1:M_INP-2]] <= (SPI_VI_BASE + l1_pre_cnt[M_INP-3:0]);
                end
                L_RMW: begin
                    // latch sparse word and pre-trace
                    l1_word_do   <= sp_vi_h1_do;
                    l1_word_di   <= sp_vi_h1_do;
                    l1_pre_val   <= h1_do[l1_pre_cnt[M_INP-1:M_INP-2]][95:84];
                    // initialize half-step
                    l1_half      <= 1'b0;
                    l1_phase     <= 1'b0; // schedule first half
                    l1_state     <= L_NEXT;
                end
                L_NEXT: begin
                    if (!l1_phase) begin
                        // schedule 4 postsyn reads for current half (groups 0..3)
                        for (int s=0; s<4; s++) begin
                            automatic int slot = (l1_half ? (s+4) : s);
                            automatic int base = slot*17;
                            l1_post_idx_half[s] = l1_word_do[base +: 9];
                            l1_w_half[s]        = l1_word_do[base+9 +: 8];
                            h1_req_l1[l1_post_idx_half[s][8:7]] <= 1'b1;
                            h1_addr_l1[l1_post_idx_half[s][8:7]] <= l1_post_idx_half[s][6:0];
                        end
                        l1_phase <= 1'b1; // next: apply
                    end else begin
                        // apply 4 updates when valid; keep previous values if not yet valid
                        for (int s=0; s<4; s++) begin
                            automatic int slot = (l1_half ? (s+4) : s);
                            automatic int base = slot*17;
                            if (h1_valid_l1[l1_post_idx_half[s][8:7]]) begin
                                automatic logic [11:0] T_hist = h1_do[l1_post_idx_half[s][8:7]][71:60];
                                automatic logic [11:0] T_prev = h1_do[l1_post_idx_half[s][8:7]][59:48];
                                automatic logic [7:0]  ste    = ste_lut[h1_do[l1_post_idx_half[s][8:7]][79:64][15:8] ^ 8'h80];
                                automatic logic do_pred = !gate_pred1;
                                automatic logic do_con  = !gate_contr1;
                                automatic logic signed [31:0] p_pred = do_pred ? (unsigned'(T_hist) * unsigned'(ste)) : 32'sd0; // Q0.8
                                automatic logic signed [31:0] p_con  = do_con  ? (unsigned'(T_prev) * unsigned'(ste)) : 32'sd0;
                                automatic logic signed [31:0] t_pred = (p_pred * unsigned'(l1_pre_val)) >>> 8;
                                automatic logic signed [31:0] t_con  = (p_con  * unsigned'(l1_pre_val)) >>> 8;
                                automatic logic signed [31:0] delta  = t_pred - t_con;
                                automatic logic signed [31:0] prod   = (delta * unsigned'(SPI_LR_H1)) >>> 8; // apply LR Q0.8
                                // gradient clip (pre-add)
                                automatic logic signed [15:0] g16   = prod[15:0];
                                automatic logic signed [15:0] gabs  = g16[15] ? -g16 : g16;
                                automatic logic signed [15:0] gcap  = (gabs > SPI_H1_GRAD_MAX) ? SPI_H1_GRAD_MAX : gabs;
                                automatic logic signed [15:0] gsel  = g16[15] ? -gcap : gcap;
                                automatic logic signed [8:0]  w_new  = $signed(l1_w_half[s]) + $signed(gsel[15:8] + (SPI_EN_STOCH_ROUND ? {{7{1'b0}}, rnd_strnd[0]} : 8'sd0));
                                automatic logic signed [7:0]  w_sat = (w_new > 9'sd127) ? 8'sd127 : (w_new < -9'sd128) ? -8'sd128 : w_new[7:0];
                                l1_word_di[base+9 +: 8] <= w_sat;
                            end
                        end
                        // move to next half or writeback
                        if (!l1_half) begin
                            l1_half  <= 1'b1;
                            l1_phase <= 1'b0;
                        end else begin
                            sp_vi_h1_cs <= 1'b1; sp_vi_h1_we <= 1'b1; sp_vi_h1_di <= l1_word_di;
                            if (l1_word + 1 < WORDS_PER_PRE_H1) begin
                                l1_word  <= l1_word + 1'b1; l1_state <= L_READ_SP; l1_syn_valid <= 1'b0;
                            end else begin
                                l1_syn_valid <= 1'b0;
                                if (l1_pre_cnt < {1'b0, SPI_NUM_INP_NEUR}) begin
                                    l1_pre_cnt <= l1_pre_cnt + 1'b1;
                                    l1_word    <= 4'd0;
                                    l1_state   <= L_READ_SP;
                                end else begin
                                    l1_state   <= L_IDLE;
                                end
                            end
                        end
                    end
                end
            endcase

            // H1->H2
            case (l2_state)
                L_IDLE: begin
                    if (!layer2_block) begin
                        l2_pre_cnt <= '0; l2_word <= 4'd0; l2_state <= L_READ_SP;
                    end
                end
                L_READ_SP: begin
                    sp_h1_h2_cs   <= 1'b1; sp_h1_h2_we <= 1'b0;
                    sp_h1_h2_addr <= {l2_pre_cnt, l2_word[$bits(sp_h1_h2_addr)-9-1:0]};
                    l2_state      <= L_RMW; // extra read cycle
                    // request pre trace read once (T_h1_i)
                    h1_req_l2[l2_pre_cnt[8:7]] <= 1'b1; h1_addr_l2[l2_pre_cnt[8:7]] <= l2_pre_cnt[6:0];
                end
                L_RMW: begin
                    l2_word_do   <= sp_h1_h2_do;
                    l2_word_di   <= sp_h1_h2_do;
                    l2_pre_val   <= h1_do[l2_pre_cnt[8:7]][83:72];
                    l2_half      <= 1'b0;
                    l2_phase     <= 1'b0; // schedule first half
                    l2_state     <= L_NEXT;
                end
                L_NEXT: begin
                    if (!l2_phase) begin
                        for (int s=0; s<4; s++) begin
                            automatic int slot = (l2_half ? (s+4) : s);
                            automatic int base = slot*17;
                            l2_post_idx_half[s] = l2_word_do[base +: 9];
                            l2_w_half[s]        = l2_word_do[base+9 +: 8];
                            h2_cs[l2_post_idx_half[s][8:7]]   <= 1'b1;
                            h2_we[l2_post_idx_half[s][8:7]]   <= 1'b0;
                            h2_addr[l2_post_idx_half[s][8:7]] <= l2_post_idx_half[s][6:0];
                        end
                        l2_phase <= 1'b1;
                    end else begin
                        if (h1_valid_l2[l2_pre_cnt[8:7]]) begin
                            for (int s=0; s<4; s++) begin
                                automatic int slot = (l2_half ? (s+4) : s);
                                automatic int base = slot*17;
                                automatic logic [11:0] T_hist2 = h2_do[l2_post_idx_half[s][8:7]][71:60];
                                automatic logic [11:0] T_prev2 = h2_do[l2_post_idx_half[s][8:7]][59:48];
                                automatic logic [7:0]  ste2    = ste_lut[h2_do[l2_post_idx_half[s][8:7]][79:64][15:8] ^ 8'h80];
                                automatic logic do_pred2 = !gate_pred2;
                                automatic logic do_con2  = !gate_contr2;
                                automatic logic signed [31:0] p_pred2 = do_pred2 ? (unsigned'(T_hist2) * unsigned'(ste2)) : 32'sd0; // Q0.8
                                automatic logic signed [31:0] p_con2  = do_con2  ? (unsigned'(T_prev2) * unsigned'(ste2)) : 32'sd0;
                                automatic logic signed [31:0] t_pred2 = (p_pred2 * unsigned'(l2_pre_val)) >>> 8;
                                automatic logic signed [31:0] t_con2  = (p_con2  * unsigned'(l2_pre_val)) >>> 8;
                                automatic logic signed [31:0] delta2  = t_pred2 - t_con2;
                                automatic logic signed [31:0] prod2   = (delta2 * unsigned'(SPI_LR_H2)) >>> 8;
                                automatic logic signed [15:0] g216   = prod2[15:0];
                                automatic logic signed [15:0] g2abs  = g216[15] ? -g216 : g216;
                                automatic logic signed [15:0] g2cap  = (g2abs > SPI_H2_GRAD_MAX) ? SPI_H2_GRAD_MAX : g2abs;
                                automatic logic signed [15:0] g2sel  = g216[15] ? -g2cap : g2cap;
                                automatic logic signed [8:0]  w2_new  = $signed(l2_w_half[s]) + $signed(g2sel[15:8] + (SPI_EN_STOCH_ROUND ? {{7{1'b0}}, rnd_strnd[1]} : 8'sd0));
                                automatic logic signed [7:0]  w2_sat = (w2_new > 9'sd127) ? 8'sd127 : (w2_new < -9'sd128) ? -8'sd128 : w2_new[7:0];
                                l2_word_di[base+9 +: 8] <= w2_sat;
                            end
                        end
                        if (!l2_half) begin
                            l2_half  <= 1'b1;
                            l2_phase <= 1'b0;
                        end else begin
                            sp_h1_h2_cs <= 1'b1; sp_h1_h2_we <= 1'b1; sp_h1_h2_di <= l2_word_di;
                            if (l2_word + 1 < WORDS_PER_PRE_H2) begin
                                l2_word  <= l2_word + 1'b1; l2_state <= L_READ_SP; l2_syn_valid <= 1'b0;
                            end else begin
                                l2_syn_valid <= 1'b0;
                                if (l2_pre_cnt < {1'b0, SPI_NUM_H1_NEUR}) begin
                                    l2_pre_cnt <= l2_pre_cnt + 1'b1;
                                    l2_word    <= 4'd0;
                                    l2_state   <= L_READ_SP;
                                end else begin
                                    l2_state   <= L_IDLE;
                                end
                            end
                        end
                    end
                end
            endcase
            // Use external arbiter module (combinational within clocked shell)
            h1_bank_arbiter #(.NG(NG), .AW(M_GH1)) u_h1arb (
                .clk(clk), .rst(rst),
                .req_l1(h1_req_l1), .addr_l1(h1_addr_l1),
                .req_l2(h1_req_l2), .addr_l2(h1_addr_l2),
                .cs(h1_cs), .addr(h1_addr),
                .valid_l1(h1_valid_l1), .valid_l2(h1_valid_l2)
            );
            // DST counters and triggers (skeleton)
            // Freeze counting when supervised output learning is active
            if (SPI_DYN_EN_H1 && !H1_NO_UPDATE && !(SPI_SUP_OUT_EN && (label_active || ol_busy))) begin
                steps_since_dst_h1 <= (steps_since_dst_h1 == 16'hFFFF) ? steps_since_dst_h1 : (steps_since_dst_h1 + 16'd1);
            end
            if (SPI_DYN_EN_H2 && !H2_NO_UPDATE && !(SPI_SUP_OUT_EN && (label_active || ol_busy))) begin
                steps_since_dst_h2 <= (steps_since_dst_h2 == 16'hFFFF) ? steps_since_dst_h2 : (steps_since_dst_h2 + 16'd1);
            end
            // H1 DST FSM
            unique case (dst1_state)
                D_IDLE: begin
                    if (SPI_DYN_EN_H1 && (steps_since_dst_h1 >= SPI_DYN_X_H1)) begin
                        prune_addr1 <= 12'd0; prune_pending1 <= 1'b0; pr_in_last1 <= 1'b0; pr_in_valid1 <= 1'b0;
                        regrow_idx1 <= 9'd0; regrow_pending1 <= 1'b0; rg_in_last1 <= 1'b0; rg_in_valid1 <= 1'b0;
                        dst1_state <= D_PRUNE;
                    end
                end
                D_PRUNE: begin
                    if (prune_addr1 < 12'd4096) begin
                        if (!prune_pending1) begin
                            sp_vi_h1_cs <= 1'b1; sp_vi_h1_we <= 1'b0; sp_vi_h1_addr <= prune_addr1; prune_pending1 <= 1'b1;
                            pr_slot1    <= 3'd0; pr_staging1 <= 136'd0;
                        end else if (sp_vi_h1_do_valid) begin
                            pr_staging1 <= sp_vi_h1_do;
                            prune_pending1 <= 1'b0;
                            prune_addr1 <= prune_addr1 + 12'd1;
                            pr_slot1    <= 3'd0;
                        end
                        if (pr_staging1 != 136'd0) begin
                            automatic int base = pr_slot1*17;
                            automatic logic [7:0] w8 = pr_staging1[base+9 +: 8];
                            pr_in_valid1 <= 1'b1; pr_in_val1 <= w8[7] ? (~w8 + 8'd1) : w8;
                            pr_in_meta1  <= '{addr:(prune_addr1-12'd1), slot:pr_slot1, group:pr_slot1[1:0]};
                            pr_in_last1  <= 1'b0;
                            pr_slot1     <= pr_slot1 + 3'd1;
                            if (pr_slot1 == 3'd7) begin
                                pr_staging1 <= 136'd0; // clear to stop emission until next word
                            end
                        end
                        if (prune_addr1 == 12'd4096 && pr_slot1 == 3'd7) pr_in_last1 <= 1'b1;
                    end
                    // Parallel REGROW feed while PRUNE is running
                    if (regrow_idx1 < N_H1[8:0]) begin
                        if (!regrow_pending1) begin
                            h1_req_l1[regrow_idx1[8:7]] <= 1'b1; h1_addr_l1[regrow_idx1[8:7]] <= regrow_idx1[6:0]; regrow_pending1 <= 1'b1;
                        end else if (h1_valid_l1[regrow_idx1[8:7]]) begin
                            automatic logic [11:0] Th = h1_do[regrow_idx1[8:7]][71:60];
                            automatic logic [11:0] Tp = h1_do[regrow_idx1[8:7]][59:48];
                            automatic logic [7:0]  ste= ste_lut[h1_do[regrow_idx1[8:7]][79:64][15:8] ^ 8'h80];
                            automatic logic signed [13:0] diff = $signed({2'b0,Th}) - $signed({2'b0,Tp});
                            automatic logic [23:0] mag = (diff[13] ? (~diff + 14'd1) : diff) * ste;
                            rg_in_valid1 <= 1'b1; rg_in_score1 <= mag; rg_in_index1 <= regrow_idx1; rg_in_last1 <= 1'b0;
                            if (regrow_idx1 == N_H1[8:0]-1) rg_in_last1 <= 1'b1;
                            regrow_idx1 <= regrow_idx1 + 9'd1; regrow_pending1 <= 1'b0;
                        end
                    end
                    if (pr_done1 && rg_done1) begin
                        dst1_state <= D_REWIRE; rewire_k1 <= 8'd0; rg_pop_group1 <= 2'd0;
                    end
                end
                D_REGROW: begin
                    if (regrow_idx1 < N_H1[8:0]) begin
                        if (!regrow_pending1) begin
                            h1_req_l1[regrow_idx1[8:7]] <= 1'b1; h1_addr_l1[regrow_idx1[8:7]] <= regrow_idx1[6:0]; regrow_pending1 <= 1'b1;
                        end else if (h1_valid_l1[regrow_idx1[8:7]]) begin
                            automatic logic [11:0] Th = h1_do[regrow_idx1[8:7]][71:60];
                            automatic logic [11:0] Tp = h1_do[regrow_idx1[8:7]][59:48];
                            automatic logic [7:0]  ste= ste_lut[h1_do[regrow_idx1[8:7]][79:64][15:8] ^ 8'h80];
                            automatic logic signed [13:0] diff = $signed({2'b0,Th}) - $signed({2'b0,Tp});
                            automatic logic [23:0] mag = (diff[13] ? (~diff + 14'd1) : diff) * ste;
                            rg_in_valid1 <= 1'b1; rg_in_score1 <= mag; rg_in_index1 <= regrow_idx1; rg_in_last1 <= 1'b0;
                            if (regrow_idx1 == N_H1[8:0]-1) rg_in_last1 <= 1'b1;
                            regrow_idx1 <= regrow_idx1 + 9'd1; regrow_pending1 <= 1'b0;
                        end
                    end else if (rg_done1) begin
                        dst1_state <= D_REWIRE; rewire_k1 <= 8'd0; rg_pop_group1 <= 2'd0;
                    end
                end
                D_REWIRE: begin
                    // Collision micro-FSM for H1 rewiring
                    case (coll1_state)
                        C_IDLE: begin
                            if (rewire_k1 < pr_k1) begin
                                pr_pop_req1 <= !pr_empty1;
                                if (pr_pop_valid1) begin
                                    coll1_sd    <= pr_pop_meta1;
                                    rg_pop_group1 <= pr_pop_meta1.group; rg_pop_req1 <= !rg_empty1;
                                    if (rg_pop_valid1) begin
                                        coll1_new_j <= rg_pop_index1;
                                        coll1_pre   <= pr_pop_meta1.addr[11 -: M_INP];
                                        coll1_group <= pr_pop_meta1.group;
                                        coll1_word  <= '0; coll1_try <= 8'd0; coll1_collide <= 1'b0;
                                        coll1_state <= SPI_COLLISION_EN ? C_REQ : C_NEW;
                                    end else if (rg_empty1) begin
                                        coll1_no_cand <= 1'b1; coll1_state <= C_NEW; // fallback: no candidate, skip change
                                    end
                                end
                            end else begin
                                steps_since_dst_h1 <= 16'd0; dst1_state <= D_DONE;
                            end
                        end
                        C_REQ: begin
                            sp_vi_h1_cs   <= 1'b1; sp_vi_h1_we <= 1'b0;
                            sp_vi_h1_addr <= {coll1_pre, coll1_word[$bits(sp_vi_h1_addr)-M_INP-1:0]};
                            coll1_state   <= C_WAIT;
                        end
                        C_WAIT: begin
                            if (sp_vi_h1_do_valid) begin
                                coll1_collide <= 1'b0;
                                for (int s=0; s<8; s++) begin
                                    if ((s[1:0]==coll1_group) && (sp_vi_h1_do[s*17 +: 9] == coll1_new_j)) coll1_collide <= 1'b1;
                                end
                                if (coll1_collide && (coll1_word + 1 < WORDS_PER_PRE_H1)) begin
                                    coll1_word <= coll1_word + 1'b1; coll1_state <= C_REQ;
                                end else if (coll1_collide && (coll1_try + 1 < SPI_N_PER_G)) begin
                                    rg_pop_group1 <= coll1_group; rg_pop_req1 <= !rg_empty1; coll1_try <= coll1_try + 1'b1;
                                    if (rg_pop_valid1) coll1_new_j <= rg_pop_index1;
                                    coll1_word <= '0; coll1_state <= C_REQ;
                                end else begin
                                    coll1_state <= C_NEW;
                                end
                            end
                        end
                        C_REQ_ORIG: begin
                            // fetch original word to modify
                            sp_vi_h1_cs   <= 1'b1; sp_vi_h1_we <= 1'b0; sp_vi_h1_addr <= coll1_sd.addr;
                            coll1_state   <= C_WAIT_ORIG;
                        end
                        C_WAIT_ORIG: begin
                            if (sp_vi_h1_do_valid) begin
                                coll1_origW <= sp_vi_h1_do;
                                coll1_state <= C_NEW;
                            end
                        end
                        C_NEW: begin
                            automatic logic [135:0] W = coll1_origW;
                            automatic int base = coll1_sd.slot*17;
                            if (!coll1_no_cand) begin
                                W[base +: 9]   = coll1_new_j;
                                W[base+9 +: 8] = 8'sd0;
                            end
                            sp_vi_h1_cs <= 1'b1; sp_vi_h1_we <= 1'b1; sp_vi_h1_addr <= coll1_sd.addr; sp_vi_h1_di <= W;
                            rewire_k1   <= rewire_k1 + 1'b1;
                            coll1_state <= C_IDLE;
                        end
                        default: coll1_state <= C_IDLE;
                    endcase
                end
                default: dst1_state <= D_IDLE;
            endcase
            // H2 DST FSM
            unique case (dst2_state)
                D_IDLE: begin
                    if (SPI_DYN_EN_H2 && (steps_since_dst_h2 >= SPI_DYN_X_H2)) begin
                        prune_addr2 <= 12'd0; prune_pending2 <= 1'b0; pr_in_last2 <= 1'b0; pr_in_valid2 <= 1'b0;
                        regrow_idx2 <= 9'd0; regrow_pending2 <= 1'b0; rg_in_last2 <= 1'b0; rg_in_valid2 <= 1'b0;
                        dst2_state <= D_PRUNE;
                    end
                end
                D_PRUNE: begin
                    if (prune_addr2 < 12'd4096) begin
                        if (!prune_pending2) begin
                            sp_h1_h2_cs <= 1'b1; sp_h1_h2_we <= 1'b0; sp_h1_h2_addr <= prune_addr2; prune_pending2 <= 1'b1;
                            pr_slot2    <= 3'd0; pr_staging2 <= 136'd0;
                        end else if (sp_h1_h2_do_valid) begin
                            pr_staging2 <= sp_h1_h2_do;
                            prune_pending2 <= 1'b0;
                            prune_addr2 <= prune_addr2 + 12'd1;
                            pr_slot2    <= 3'd0;
                        end
                        if (pr_staging2 != 136'd0) begin
                            automatic int base = pr_slot2*17;
                            automatic logic [7:0] w8 = pr_staging2[base+9 +: 8];
                            pr_in_valid2 <= 1'b1; pr_in_val2 <= w8[7] ? (~w8 + 8'd1) : w8;
                            pr_in_meta2  <= '{addr:(prune_addr2-12'd1), slot:pr_slot2, group:pr_slot2[1:0]};
                            pr_in_last2  <= 1'b0;
                            pr_slot2     <= pr_slot2 + 3'd1;
                            if (pr_slot2 == 3'd7) begin
                                pr_staging2 <= 136'd0;
                            end
                        end
                        if (prune_addr2 == 12'd4096 && pr_slot2 == 3'd7) pr_in_last2 <= 1'b1;
                    end
                    // Parallel REGROW feed while PRUNE is running
                    if (regrow_idx2 < N_H2[8:0]) begin
                        if (!regrow_pending2) begin
                            // request H2 read via future arbiter (direct for now)
                            h2_cs[regrow_idx2[8:7]] <= 1'b1; h2_we[regrow_idx2[8:7]] <= 1'b0; h2_addr[regrow_idx2[8:7]] <= regrow_idx2[6:0]; regrow_pending2 <= 1'b1;
                        end else begin
                            automatic logic [11:0] Th = h2_do[regrow_idx2[8:7]][71:60];
                            automatic logic [11:0] Tp = h2_do[regrow_idx2[8:7]][59:48];
                            automatic logic [7:0]  ste= ste_lut[h2_do[regrow_idx2[8:7]][79:64][15:8] ^ 8'h80];
                            automatic logic signed [13:0] diff = $signed({2'b0,Th}) - $signed({2'b0,Tp});
                            automatic logic [23:0] mag = (diff[13] ? (~diff + 14'd1) : diff) * ste;
                            rg_in_valid2 <= 1'b1; rg_in_score2 <= mag; rg_in_index2 <= regrow_idx2; rg_in_last2 <= 1'b0;
                            if (regrow_idx2 == N_H2[8:0]-1) rg_in_last2 <= 1'b1;
                            regrow_idx2 <= regrow_idx2 + 9'd1; regrow_pending2 <= 1'b0;
                        end
                    end
                    if (pr_done2 && rg_done2) begin
                        dst2_state <= D_REWIRE; rewire_k2 <= 8'd0; rg_pop_group2 <= 2'd0;
                    end
                end
                D_REGROW: begin
                    // Unused when parallel mode is active; kept for clarity
                    if (rg_done2) begin end
                end
                D_REWIRE: begin
                    // Collision micro-FSM for H2 rewiring
                    case (coll2_state)
                        C_IDLE: begin
                            if (rewire_k2 < pr_k2) begin
                                pr_pop_req2 <= !pr_empty2;
                                if (pr_pop_valid2) begin
                                    coll2_sd    <= pr_pop_meta2;
                                    rg_pop_group2 <= pr_pop_meta2.group; rg_pop_req2 <= !rg_empty2;
                                    if (rg_pop_valid2) begin
                                        coll2_new_j <= rg_pop_index2;
                                        coll2_pre   <= pr_pop_meta2.addr[11 -: 9];
                                        coll2_group <= pr_pop_meta2.group;
                                        coll2_word  <= '0; coll2_try <= 8'd0; coll2_collide <= 1'b0;
                                        coll2_state <= SPI_COLLISION_EN ? C_REQ : C_NEW;
                                    end else if (rg_empty2) begin
                                        coll2_no_cand <= 1'b1; coll2_state <= C_NEW;
                                    end
                                end
                            end else begin
                                steps_since_dst_h2 <= 16'd0; dst2_state <= D_DONE;
                            end
                        end
                        C_REQ: begin
                            sp_h1_h2_cs   <= 1'b1; sp_h1_h2_we <= 1'b0;
                            sp_h1_h2_addr <= {coll2_pre, coll2_word[2:0]};
                            coll2_state   <= C_WAIT;
                        end
                        C_WAIT: begin
                            if (sp_h1_h2_do_valid) begin
                                coll2_collide <= 1'b0;
                                for (int s=0; s<8; s++) begin
                                    if ((s[1:0]==coll2_group) && (sp_h1_h2_do[s*17 +: 9] == coll2_new_j)) coll2_collide <= 1'b1;
                                end
                                if (coll2_collide && (coll2_word + 1 < WORDS_PER_PRE_H2)) begin
                                    coll2_word <= coll2_word + 1'b1; coll2_state <= C_REQ;
                                end else if (coll2_collide && (coll2_try + 1 < SPI_N_PER_G)) begin
                                    rg_pop_group2 <= coll2_group; rg_pop_req2 <= !rg_empty2; coll2_try <= coll2_try + 1'b1;
                                    if (rg_pop_valid2) coll2_new_j <= rg_pop_index2;
                                    coll2_word <= '0; coll2_state <= C_REQ;
                                end else begin
                                    coll2_state <= C_NEW;
                                end
                            end
                        end
                        C_REQ_ORIG: begin
                            sp_h1_h2_cs   <= 1'b1; sp_h1_h2_we <= 1'b0; sp_h1_h2_addr <= coll2_sd.addr;
                            coll2_state   <= C_WAIT_ORIG;
                        end
                        C_WAIT_ORIG: begin
                            if (sp_h1_h2_do_valid) begin
                                coll2_origW <= sp_h1_h2_do;
                                coll2_state <= C_NEW;
                            end
                        end
                        C_NEW: begin
                            automatic logic [135:0] W2 = coll2_origW;
                            automatic int base = coll2_sd.slot*17;
                            if (!coll2_no_cand) begin
                                W2[base +: 9]   = coll2_new_j;
                                W2[base+9 +: 8] = 8'sd0;
                            end
                            sp_h1_h2_cs <= 1'b1; sp_h1_h2_we <= 1'b1; sp_h1_h2_addr <= coll2_sd.addr; sp_h1_h2_di <= W2;
                            rewire_k2   <= rewire_k2 + 1'b1;
                            coll2_state <= C_IDLE;
                        end
                        default: coll2_state <= C_IDLE;
                    endcase
                end
                default: dst2_state <= D_IDLE;
            endcase
        end else begin
            l1_state <= L_IDLE; l2_state <= L_IDLE;
        end
    end
    // Sample edge tracking for delay-line control
    logic sample_q;
    always_ff @(posedge clk or posedge rst) begin
        if (rst) sample_q <= 1'b0; else sample_q <= SAMPLE;
    end
    wire sample_fall = sample_q & ~SAMPLE;
    // Push into AER FIFO when debounced edge detected
    assign aer_fifo_push = aer_rise;
    assign aer_fifo_din  = AERIN_ADDR;
    wire tick_rise_ext = time_tick_q & ~time_tick_qq;

    // Local/external tick selection and timing-error policy
    logic [31:0] tick_cnt;
    wire         tick_rise_local = (tick_cnt == SPI_CYCLES_PER_TICK);
    wire         time_tick       = SPI_LOCAL_TICK ? tick_rise_local : tick_rise_ext;
    logic        TIMING_ERROR;
    always_ff @(posedge clk or posedge rst) begin
        if (rst) begin
            tick_cnt <= 32'd1;
        end else if (SPI_LOCAL_TICK) begin
            if ((state != S_IDLE) && (tick_cnt != SPI_CYCLES_PER_TICK)) tick_cnt <= tick_cnt + 32'd1;
            else                                                        tick_cnt <= 32'd1;
        end else begin
            tick_cnt <= 32'd1;
        end
    end
    // Ready-for-next-tick policy
    // Fall-through: only when IDLE. Pipelined: allow next tick when PROP1 is idle AND STEP2 has completed its writeback for current tick.
    logic step2_done_flag;
    always_ff @(posedge clk or posedge rst) begin
        if (rst) step2_done_flag <= 1'b1;
        else if (state == S_STEP2 && h2_step_rw && (h2_step_idx == G_H2-1)) step2_done_flag <= 1'b1; // end of writeback for last index
        else if (time_tick && (state == S_PROP1)) step2_done_flag <= 1'b0; // starting new tick clears until STEP2 finishes
    end
    wire ready_for_tick = SPI_PIPELINED ? ((state == S_PROP1) ? (!vih1_active && step2_done_flag) : 1'b1)
                                        : (state == S_IDLE);
    always_ff @(posedge clk or posedge rst) begin
        if (rst) TIMING_ERROR <= 1'b0;
        else if (time_tick && !ready_for_tick) TIMING_ERROR <= 1'b1;
        else if (SPI_ERROR_HALT && TIMING_ERROR) TIMING_ERROR <= TIMING_ERROR; // sticky
        else TIMING_ERROR <= TIMING_ERROR;
    end
    // Gate next tick if DST is running
    wire dst_busy = (dst1_state != D_IDLE && dst1_state != D_DONE) || (dst2_state != D_IDLE && dst2_state != D_DONE);
    assign TIMING_ERROR_RDY = SPI_TIMING_MODE ? TIMING_ERROR : (ready_for_tick && !dst_busy);
    assign SPI_RDY          = SPI_EN_CONF;

    // Simple one-event-at-a-time handshake
    always_ff @(posedge clk or posedge rst) begin
        if (rst) AERIN_ACK <= 1'b0;
        else     AERIN_ACK <= aer_rise;
    end

    // ------------------------------------------------------------------------
    // Sparse synapse memory addressing helpers
    // ------------------------------------------------------------------------
    // For each presynaptic index, we expect WORDS_PER_PRE = (NG*N_PER_G)/8 words
    localparam int unsigned WORDS_PER_PRE = (NG*N_PER_G + 7) / 8; // ceil

    // VI->H1 propagation control
    logic [$clog2(WORDS_PER_PRE)-1:0] vih1_word_cnt;
    logic                              vih1_active;
    logic [M_INP-1:0]                  vih1_pre_idx;

    // H1->H2 propagation control
    logic [$clog2(WORDS_PER_PRE)-1:0] h1h2_word_cnt;
    logic                              h1h2_active;
    logic [8:0]                        h1h2_pre_abs; // 9-bit absolute index of H1 spiker

    // ------------------------------------------------------------------------
    // FSM
    // ------------------------------------------------------------------------
    always_ff @(posedge clk or posedge rst) begin
        if (rst) state <= S_IDLE;
        else if (SPI_EN_CONF) state <= S_IDLE; // freeze into IDLE for safe programming
        else     state <= next_state;
    end

    always_comb begin
        next_state = state;
        unique case (state)
            S_IDLE:  if (SAMPLE && time_tick)                   next_state = S_PROP1;
                     else                                       next_state = S_IDLE;
            S_PROP1: if (!vih1_active && !(bursts_left != 4'd0 && !aer_fifo_empty)) next_state = S_STEP1;
                     else                                       next_state = S_PROP1;
            S_STEP1: if (time_tick)                             next_state = S_PROP2;
                     else                                       next_state = S_STEP1;
            S_PROP2: if (!h1h2_active && h1_spike_empty)        next_state = S_STEP2;
                     else                                       next_state = S_PROP2;
            S_STEP2: if (time_tick)                              next_state = S_LEARN;
                     else                                       next_state = S_STEP2;
            S_LEARN: if (time_tick && (SPI_SEND_PER_TIMESTEP || !SAMPLE)) next_state = S_SEND;
                     else if (time_tick)                        next_state = S_PROP1;
                     else                                       next_state = S_LEARN;
            S_SEND:  if (OUT_ACK)                               next_state = S_IDLE;
                     else                                       next_state = S_SEND;
            default:                                            next_state = S_IDLE;
        endcase
    end

    // ------------------------------------------------------------------------
    // VI->H1 propagation (sparse)
    // ------------------------------------------------------------------------
    // Start on input AER event during PROP1 (multi-burst coalescing)
    // Track a conflict-free VI trace increment micro-op per event (separate READ->WRITE on H1 bank)
    logic        vi_inc_pending;
    logic        vi_inc_doing;
    logic        vi_inc_rw;      // 0: READ, 1: WRITE
    logic [1:0]  vi_inc_bank;
    logic [M_GH1-1:0] vi_inc_addr;
    logic [11:0] vi_t_cur;
    always_ff @(posedge clk or posedge rst) begin
        if (rst) begin
            vih1_active  <= 1'b0;
            vih1_word_cnt<= '0;
            vih1_pre_idx <= '0;
            bursts_left  <= 4'd4;
        end else begin
            if ((state == S_PROP1) && !vih1_active && !aer_fifo_empty) begin
                vih1_active   <= 1'b1;
                vih1_word_cnt <= '0;
                vih1_pre_idx  <= aer_fifo_dout;
                aer_fifo_pop  <= 1'b1;
                bursts_left   <= (bursts_left != 4'd0) ? (bursts_left - 4'd1) : 4'd0;
                // schedule VI trace increment micro-op for this AER event
                vi_inc_pending<= 1'b1;
                vi_inc_doing  <= 1'b0;
                vi_inc_rw     <= 1'b0;
                vi_inc_bank   <= aer_fifo_dout[M_INP-1:M_INP-2];
                vi_inc_addr   <= SPI_VI_BASE + aer_fifo_dout[M_INP-3:0];
            end else if (state == S_PROP1 && vih1_active) begin
                // issue memory read each cycle
                if (vih1_word_cnt == WORDS_PER_PRE-1) begin
                    vih1_active   <= 1'b0;
                    vih1_word_cnt <= '0;
                    aer_fifo_pop  <= 1'b0;
                end else begin
                    // hold word counter during VI-trace micro-op to avoid skipping sparse words
                    if (!vi_inc_doing) vih1_word_cnt <= vih1_word_cnt + 1'b1;
                end
            end else if ((state == S_PROP1) && !vih1_active && (bursts_left != 4'd0) && !aer_fifo_empty) begin
                // Chain next presyn within the same tick to amortize control overhead
                vih1_active   <= 1'b1;
                vih1_word_cnt <= '0;
                vih1_pre_idx  <= aer_fifo_dout;
                aer_fifo_pop  <= 1'b1;
                bursts_left   <= bursts_left - 4'd1;
                // schedule VI trace increment micro-op for this AER event
                vi_inc_pending<= 1'b1;
                vi_inc_doing  <= 1'b0;
                vi_inc_rw     <= 1'b0;
                vi_inc_bank   <= aer_fifo_dout[M_INP-1:M_INP-2];
                vi_inc_addr   <= SPI_VI_BASE + aer_fifo_dout[M_INP-3:0];
            end else if (state != S_PROP1) begin
                vih1_active <= 1'b0;
                aer_fifo_pop<= 1'b0;
                bursts_left <= 4'd4;
                vi_inc_pending<= 1'b0;
                vi_inc_doing  <= 1'b0;
                vi_inc_rw     <= 1'b0;
            end
        end
    end

    // Sparse mem read for VI->H1
    assign sp_vi_h1_cs   = (state == S_PROP1) && vih1_active;
    assign sp_vi_h1_we   = 1'b0;
    assign sp_vi_h1_addr = {vih1_pre_idx, vih1_word_cnt[$bits(sp_vi_h1_addr)-M_INP-1:0]};
    assign sp_vi_h1_di   = '0;

    // Parallel 4-PE processing per sparse word: syn1..syn4 then syn5..syn8
    // Control: wait one cycle after issuing sp_vi_h1_addr for DO to be valid, then READ -> WRITE per half
    logic        vih1_wait_do;   // waiting for DO valid after address issue
    logic        vih1_half;      // 0: slots 0..3, 1: slots 4..7
    logic        vih1_rw;        // 0: READ (assert CS, addr), 1: WRITE (WE with DI built from last DO)
    logic [6:0]  vih1_lix[NG-1:0];
    logic signed [15:0] vih1_w16[NG-1:0];

    // Decode helper for current half
    function automatic [6:0] get_local_ix(input logic [135:0] word, input int slot);
        get_local_ix = word[17*slot + 16 -: 9][6:0];
    endfunction
    function automatic [1:0] get_group(input logic [135:0] word, input int slot);
        get_group = word[17*slot + 16 -: 9][8:7];
    endfunction
    function automatic logic signed [15:0] get_w16(input logic [135:0] word, input int slot);
        logic signed [7:0] w8;
        begin
            w8   = word[17*slot +: 8];
            get_w16 = {{8{w8[7]}}, w8} <<< FP_LOC_W12;
        end
    endfunction

    // Sequencer for VI->H1 half processing
    always_ff @(posedge clk or posedge rst) begin
        if (rst) begin
            vih1_wait_do <= 1'b0;
            vih1_half    <= 1'b0;
            vih1_rw      <= 1'b0;
        end else if (state == S_PROP1 && vih1_active) begin
            if (!vih1_wait_do && !vih1_rw) begin
                // Issue READ for current half using freshly decoded indices
                vih1_lix[0] <= get_local_ix(sp_vi_h1_do, (vih1_half ? 4 : 0) + 0);
                vih1_lix[1] <= get_local_ix(sp_vi_h1_do, (vih1_half ? 4 : 0) + 1);
                vih1_lix[2] <= get_local_ix(sp_vi_h1_do, (vih1_half ? 4 : 0) + 2);
                vih1_lix[3] <= get_local_ix(sp_vi_h1_do, (vih1_half ? 4 : 0) + 3);
                vih1_w16[0] <= get_w16    (sp_vi_h1_do, (vih1_half ? 4 : 0) + 0);
                vih1_w16[1] <= get_w16    (sp_vi_h1_do, (vih1_half ? 4 : 0) + 1);
                vih1_w16[2] <= get_w16    (sp_vi_h1_do, (vih1_half ? 4 : 0) + 2);
                vih1_w16[3] <= get_w16    (sp_vi_h1_do, (vih1_half ? 4 : 0) + 3);
                vih1_rw     <= 1'b1; // next cycle write using captured DO
            end else if (vih1_rw) begin
                // Completed WRITE, move to next half or next word
                vih1_rw      <= 1'b0;
                if (!vih1_half) begin
                    vih1_half <= 1'b1; // process syn5..syn8 next
                end else begin
                    vih1_half    <= 1'b0;
                    // advance to next word (sp DO will update next cycle due to CS held)
                    if (vih1_word_cnt == WORDS_PER_PRE-1) begin
                        // end handled by outer controller
                    end else begin
                        // word_cnt increments in outer controller; DO will be valid next cycle
                    end
                end
            end
        end else begin
            vih1_wait_do <= 1'b0;
            vih1_half    <= 1'b0;
            vih1_rw      <= 1'b0;
        end
    end

    // Drive H1 banks during PROP1 based on vih1_rw (READ) and then WRITE next cycle
    always_comb begin
        if (state == S_PROP1 && vih1_active) begin
            // Default no access
            for (int g = 0; g < NG; g++) begin
                h1_cs[g]   = 1'b0;
                h1_we[g]   = 1'b0;
                h1_addr[g] = '0;
                h1_di[g]   = '0;
                h1_mask[g] = '0;
            end
            // Priority: VI trace micro-op to avoid single-port conflict
            if (vi_inc_doing) begin
                h1_cs[vi_inc_bank]   = 1'b1;
                h1_we[vi_inc_bank]   = vi_inc_rw;
                h1_addr[vi_inc_bank] = vi_inc_addr;
                if (vi_inc_rw) begin
                    // write incremented VI trace (saturating add by fixed step)
                    logic [11:0] t_cur = vi_t_cur;
                    logic [11:0] t_inc = t_cur + (12'd8 << EFF_FP_LOC_TINP);
                    logic [11:0] t_next= (t_inc < t_cur) ? 12'hFFF : t_inc;
                    h1_di[vi_inc_bank][95:84]   = t_next;
                    h1_mask[vi_inc_bank][95:84] = 12'hFFF;
                end
            end else if (!vih1_rw) begin
                // READ stage: present addresses to all 4 banks for this half
                for (int g = 0; g < NG; g++) begin
                    h1_cs[g]   = 1'b1;
                    h1_we[g]   = 1'b0;
                    h1_addr[g] = vih1_lix[g];
                end
            end else begin
                // WRITE stage: add w16 to current DO and write back membrane; also event-driven VI trace increment at VI slot
                for (int g = 0; g < NG; g++) begin
                    logic signed [15:0] u_cur;
                    logic signed [15:0] u_sum;
                    logic signed [15:0] u_sat;
                    u_cur = h1_do[g][79:64];
                    // Optional membrane noise during accumulation
                    logic signed [17:0] noise_raw = $signed({1'b0, rnd_noise, 1'b0}) - 18'sd131072; // center around 0, align for shift
                    logic signed [15:0] noise = SPI_NOISE_EN ? $signed(noise_raw[17:2]) >>> SPI_NOISE_STR : 16'sd0;
                    u_sum = u_cur + vih1_w16[g] + noise;
                    u_sat = ( u_cur[15] &  vih1_w16[g][15] & ~u_sum[15]) ? 16'sh8000 :
                            (~u_cur[15] & ~vih1_w16[g][15] &  u_sum[15]) ? 16'sh7FFF : u_sum;
                    h1_cs[g]                 = 1'b1;
                    h1_we[g]                 = (u_sat != u_cur);
                    h1_addr[g]               = vih1_lix[g];
                    h1_di[g][79:64]          = u_sat;
                    h1_mask[g][79:64]        = {16{(u_sat != u_cur)}};
                end
            end
        end
    end

    // VI trace micro-op sequencer
    always_ff @(posedge clk or posedge rst) begin
        if (rst) begin
            vi_inc_doing <= 1'b0;
            vi_inc_rw    <= 1'b0;
            vi_t_cur     <= 12'd0;
        end else if (state == S_PROP1) begin
            if (vi_inc_pending && !vi_inc_doing && !vih1_rw) begin
                // start micro-op: READ current T_vi
                vi_inc_doing <= 1'b1;
                vi_inc_rw    <= 1'b0;
            end else if (vi_inc_doing && !vi_inc_rw) begin
                // latch read data and switch to WRITE
                vi_t_cur  <= h1_do[vi_inc_bank][95:84];
                vi_inc_rw <= 1'b1;
            end else if (vi_inc_doing && vi_inc_rw) begin
                // complete, clear flags
                vi_inc_doing  <= 1'b0;
                vi_inc_rw     <= 1'b0;
                vi_inc_pending<= 1'b0;
            end
        end else begin
            vi_inc_doing <= 1'b0;
            vi_inc_rw    <= 1'b0;
            vi_inc_pending<= 1'b0;
        end
    end

    // ------------------------------------------------------------------------
    // H1 STEP: leak + threshold + spike to FIFO
    // ------------------------------------------------------------------------
    // STEP1: parallel 4-bank read then write per index; stage spikes and push sequentially
    logic [M_GH1-1:0] h1_step_idx;
    logic             h1_step_rw; // 0: READ, 1: WRITE
    logic [3:0]       h1_spk_pending;
    logic [8:0]       h1_spk_idx   [NG-1:0];
    // H1 delay buffers implemented as SRAM-based circular buffers (one 128b word per neuron)
    // Layout per word: 8 taps x 12b = 96b placed at [12*k +: 12], k in [0..7]
    logic [127:0]      h1_hist_q   [NG-1:0];
    logic              h1_hist_cs  [NG-1:0];
    logic              h1_hist_we  [NG-1:0];
    logic [M_GH1-1:0]  h1_hist_addr[NG-1:0];
    logic [127:0]      h1_hist_di  [NG-1:0];
    logic [127:0]      h1_hist_mask[NG-1:0];
    logic [2:0]        h1_hist_wptr[NG-1:0];
    logic [127:0]      h1_prev_q   [NG-1:0];
    logic              h1_prev_cs  [NG-1:0];
    logic              h1_prev_we  [NG-1:0];
    logic [M_GH1-1:0]  h1_prev_addr[NG-1:0];
    logic [127:0]      h1_prev_di  [NG-1:0];
    logic [127:0]      h1_prev_mask[NG-1:0];
    logic [2:0]        h1_prev_wptr[NG-1:0];
    logic [2:0]        h1_prev_wptr_snap[NG-1:0];

    always_ff @(posedge clk or posedge rst) begin
        if (rst) begin
            h1_step_idx   <= '0;
            h1_step_rw    <= 1'b0;
            h1_spk_pending<= 4'b0000;
            // Clear H1 circular buffer pointers
            for (int g=0; g<NG; g++) begin
                h1_hist_wptr[g] <= '0;
                h1_prev_wptr[g] <= '0;
                h1_prev_wptr_snap[g] <= '0;
            end
            sim_pred_h1   <= 32'd0;
            sim_contr_h1  <= 32'd0;
        end else if (state == S_STEP1) begin
            // push pending spikes one per cycle if any
            if (|h1_spk_pending) begin
                for (int g=0; g<NG; g++) begin
                    if (h1_spk_pending[g]) begin
                        h1_spike_push <= 1'b1;
                        h1_spike_in   <= h1_spk_idx[g];
                        h1_spk_pending[g] <= 1'b0;
                        break;
                    end
                end
            end else begin
                h1_spike_push <= 1'b0;
            end

            if (!h1_step_rw) begin
                // READ stage -> next cycle WRITE
                h1_step_rw <= 1'b1;
                if (h1_step_idx == '0) begin
                    // start of sweep: clear accumulators
                    sim_pred_h1      <= 32'd0;
                    sim_contr_h1     <= 32'd0;
                    h1_spk_cnt_tick  <= 16'd0;
                end
            end else begin
                // WRITE stage -> advance index and go back to READ
                // Collect spikes flags and indices for this index
                for (int g=0; g<NG; g++) begin
                    logic signed [15:0] u   = h1_do[g][79:64];
                    logic signed [15:0] thr = h1_do[g][111:96];
                    logic               spk = (u >= thr);
                    if (spk) begin
                        h1_spk_pending[g] <= 1'b1;
                        h1_spk_idx[g]     <= {g[1:0], h1_step_idx};
                        h1_spk_cnt_tick   <= h1_spk_cnt_tick + 16'd1;
                        // Accumulate similarity using selected taps
                        logic [2:0] sel_hist = (h1_hist_wptr[g] + 3'd7 - tap_hist_sel);
                        logic [2:0] sel_prev = (h1_prev_wptr_snap[g] + 3'd7 - tap_prev_sel);
                        logic [11:0] t_hist  = (sel_hist==3'd0)? h1_do[g][HIST_BASE+ 0 +:12] :
                                               (sel_hist==3'd1)? h1_do[g][HIST_BASE+16 +:12] :
                                               (sel_hist==3'd2)? h1_do[g][HIST_BASE+32 +:12] :
                                               (sel_hist==3'd3)? h1_do[g][HIST_BASE+48 +:12] :
                                               (sel_hist==3'd4)? h1_do[g][HIST_BASE+64 +:12] :
                                               (sel_hist==3'd5)? h1_do[g][HIST_BASE+80 +:12] :
                                               (sel_hist==3'd6)? h1_do[g][HIST_BASE+96 +:12] :
                                                                 h1_do[g][HIST_BASE+112+:12];
                        logic [11:0] t_prev  = (sel_prev==3'd0)? h1_do[g][PREV_BASE+ 0 +:12] :
                                               (sel_prev==3'd1)? h1_do[g][PREV_BASE+16 +:12] :
                                               (sel_prev==3'd2)? h1_do[g][PREV_BASE+32 +:12] :
                                               (sel_prev==3'd3)? h1_do[g][PREV_BASE+48 +:12] :
                                               (sel_prev==3'd4)? h1_do[g][PREV_BASE+64 +:12] :
                                               (sel_prev==3'd5)? h1_do[g][PREV_BASE+80 +:12] :
                                               (sel_prev==3'd6)? h1_do[g][PREV_BASE+96 +:12] :
                                                                 h1_do[g][PREV_BASE+112+:12];
                        sim_pred_h1  <= sim_pred_h1  + t_hist;
                        sim_contr_h1 <= sim_contr_h1 + t_prev;
                    end
                    // Advance circular buffers (write current T_h1 into slot indicated by wptr)
                    // write into combined SRAM upper regions (history ring)
                    h1_cs[g]    <= 1'b1;
                    h1_we[g]    <= 1'b1;
                    h1_addr[g]  <= h1_step_idx;
                    h1_di[g]    <= '0;
                    h1_mask[g]  <= '0;
                    h1_di[g][HIST_BASE + {h1_hist_wptr[g], 4'b0000} +: 12]   <= h1_do[g][83:72];
                    h1_mask[g][HIST_BASE + {h1_hist_wptr[g], 4'b0000} +: 12] <= 12'hFFF;
                    h1_hist_wptr[g]  <= (h1_hist_wptr[g] == 3'd7) ? 3'd0 : (h1_hist_wptr[g] + 3'd1);
                    if (SAMPLE) begin
                        h1_di[g][PREV_BASE + {h1_prev_wptr[g], 4'b0000} +: 12]   <= h1_do[g][83:72];
                        h1_mask[g][PREV_BASE + {h1_prev_wptr[g], 4'b0000} +: 12] <= 12'hFFF;
                        h1_prev_wptr[g] <= (h1_prev_wptr[g] == 3'd7) ? 3'd0 : (h1_prev_wptr[g] + 3'd1);
                    end
                end
                h1_step_rw <= 1'b0;
                h1_step_idx<= (h1_step_idx == G_H1-1) ? '0 : (h1_step_idx + 1'b1);
                if (h1_step_idx == G_H1-1) begin
                    // end of sweep: latch raw and scaled
                    sim_pred_h1_latched  <= sim_pred_h1;
                    sim_contr_h1_latched <= sim_contr_h1;
                    h1_spk_cnt_latched   <= h1_spk_cnt_tick;
                    // Normalization: choose denominator per mode
                    automatic logic [31:0] denom_h1 = 32'd1;
                    case (SPI_SIM_NORM_MODE)
                        2'd1: denom_h1 = (h1_spk_cnt_tick == 16'd0) ? 32'd1 : {16'd0,h1_spk_cnt_tick};
                        2'd2: denom_h1 = {23'd0, SPI_NUM_H1_NEUR} + 32'd1; // count = value+1
                        default: denom_h1 = 32'd1;
                    endcase
                    sim_pred_h1_scaled   <= ((sim_pred_h1  / denom_h1) * SPI_SIM_SCALE) >> SPI_SIM_SHIFT;
                    sim_contr_h1_scaled  <= ((sim_contr_h1 / denom_h1) * SPI_SIM_SCALE) >> SPI_SIM_SHIFT;
                end
            end
        end else begin
            h1_spike_push <= 1'b0;
            h1_step_rw    <= 1'b0;
        end
    end

    always_comb begin
        if (state == S_STEP1) begin
            // default zeros
            for (int g = 0; g < NG; g++) begin
                h1_cs[g]   = 1'b0;
                h1_we[g]   = 1'b0;
                h1_addr[g] = '0;
                h1_di[g]   = '0;
                h1_mask[g] = '0;
            end
            if (!h1_step_rw) begin
                // READ all 4 banks at current index
                for (int g=0; g<NG; g++) begin
                    h1_cs[g]   = 1'b1;
                    h1_we[g]   = 1'b0;
                    h1_addr[g] = h1_step_idx;
                end
            end else begin
                // WRITE back leak/threshold results for all 4 banks; include decay of both VI and H1 traces during STEP1
                for (int g=0; g<NG; g++) begin
                    logic signed [15:0] u      = h1_do[g][79:64];
                    logic signed [15:0] thr    = h1_do[g][111:96];
                    logic [15:0]        alpha  = h1_do[g][127:112];
                    logic signed [31:0] u_mul  = $signed({1'b0,u}) * $signed({1'b0,alpha});
                    // Stochastic rounding on leak: increment by 1 LSB if cropped part >= rnd and MSBs non-zero
                    logic [14:0] frac = u_mul[14:0];
                    logic signed [15:0] u_leak = (SPI_EN_STOCH_ROUND && (alpha != 16'h8000) && (frac >= rnd_strnd) && |u_mul[30:15])
                                                 ? ($signed(u_mul[30:15]) + 16'sd1) : $signed(u_mul[30:15]);
                    logic               spk    = (u >= thr);
                    logic signed [15:0] u_next = spk ? (u_leak - thr) : u_leak;
                    // VI trace (T_vi): decay only (increments happen on VI events)
                    logic [11:0] tvi_cur = h1_do[g][95:84];
                    logic [27:0] tvi_mul = {12'b0,tvi_cur} * {12'b0,alpha};
                    logic [14:0] tvi_frac= tvi_mul[14:0];
                    logic [11:0] tvi_dec = (SPI_EN_STOCH_ROUND && (alpha != 16'h8000) && (tvi_frac >= rnd_strnd[11:0]) && |tvi_mul[26:15])
                                            ? (tvi_mul[26:15] + 12'd1) : tvi_mul[26:15];
                    // H1 neuron trace (T_h1): increment on spike, then decay
                    logic [11:0] th1_cur = h1_do[g][83:72];
                    logic [11:0] th1_inc = spk ? (th1_cur + (12'd2 << EFF_FP_LOC_TOUT)) : th1_cur;
                    logic [27:0] th1_mul = {12'b0,th1_inc} * {12'b0,alpha};
                    logic [14:0] th1_frac= th1_mul[14:0];
                    logic [11:0] th1_dec = (SPI_EN_STOCH_ROUND && (alpha != 16'h8000) && (th1_frac >= rnd_strnd[11:0]) && |th1_mul[26:15])
                                            ? (th1_mul[26:15] + 12'd1) : th1_mul[26:15];
                    // Refractory/homeostasis: adjust threshold softly
                    logic signed [15:0] thr_upd = spk ? (thr + {{8{SPI_REF_ADD_H1[7]}}, SPI_REF_ADD_H1}) : (thr - {{8{SPI_REF_DEC_H1[7]}}, SPI_REF_DEC_H1});
                    // Delay lines: shift T_h1 for T_hist (depth 1..8)
                    // Implement by writing the selected delay tap directly as T_hist each visit
                    // Effective depth = SPI_THIST_DEPTH + 1
                    logic [11:0] thist_next = th1_cur; // depth=1 default
                    // Read thist tap from combined SRAM: use Q (read in READ phase) and current wptr
                    logic [2:0] thist_sel = (h1_hist_wptr[g] + 3'd7 - SPI_THIST_DEPTH);
                    unique case (thist_sel)
                        3'd0: thist_next = h1_do[g][HIST_BASE +  0 +: 12];
                        3'd1: thist_next = h1_do[g][HIST_BASE + 16 +: 12];
                        3'd2: thist_next = h1_do[g][HIST_BASE + 32 +: 12];
                        3'd3: thist_next = h1_do[g][HIST_BASE + 48 +: 12];
                        3'd4: thist_next = h1_do[g][HIST_BASE + 64 +: 12];
                        3'd5: thist_next = h1_do[g][HIST_BASE + 80 +: 12];
                        3'd6: thist_next = h1_do[g][HIST_BASE + 96 +: 12];
                        default: thist_next = h1_do[g][HIST_BASE + 112 +: 12];
                    endcase
                    logic [11:0] tprev_cur  = h1_do[g][59:48];
                    logic        snap_prev  = h1_prev_snap; // snapshot window across one full sweep
                    logic [11:0] tprev_sel;
                    // Read tprev tap from previous-sample circular buffer (snapshot of its wptr)
                    logic [2:0] prev_base = h1_prev_wptr_snap[g];
                    unique case (SPI_TPREV_DEPTH)
                        3'd0: tprev_sel = h1_do[g][PREV_BASE + {prev_base,4'b0000} +: 12];
                        default: begin
                            logic [2:0] prev_sel = (prev_base + 3'd7 - SPI_TPREV_DEPTH);
                            tprev_sel = h1_do[g][PREV_BASE + {prev_sel,4'b0000} +: 12];
                        end
                    endcase
                    logic [11:0] tprev_next = snap_prev ? tprev_sel : tprev_cur;
                    h1_cs[g]                 = 1'b1;
                    h1_we[g]                 = (u_next != u) || (tvi_dec != tvi_cur) || (th1_dec != th1_cur) || (thist_next != h1_do[g][71:60]) || (tprev_next != tprev_cur) || 1'b0;
                    h1_addr[g]               = h1_step_idx;
                    h1_di[g][79:64]          = u_next;
                    h1_mask[g][79:64]        = {16{(u_next != u)}};
                    h1_di[g][95:84]          = tvi_dec;
                    h1_mask[g][95:84]        = {12{(tvi_dec != tvi_cur)}};
                    h1_di[g][83:72]          = th1_dec;
                    h1_mask[g][83:72]        = {12{(th1_dec != th1_cur)}};
                    h1_di[g][71:60]          = thist_next;
                    h1_mask[g][71:60]        = {12{(thist_next != h1_do[g][71:60])}};
                    h1_di[g][59:48]          = tprev_next;
                    h1_mask[g][59:48]        = {12{(tprev_next != tprev_cur)}};
                    h1_di[g][111:96]         = thr_upd;
                    h1_mask[g][111:96]       = {16{(thr_upd != thr)}};
                end
            end
        end
    end

    // ------------------------------------------------------------------------
    // H1->H2 propagation (sparse)
    // ------------------------------------------------------------------------
    // Start a burst for each H1 spike from FIFO
    always_ff @(posedge clk or posedge rst) begin
        if (rst) begin
            h1h2_active   <= 1'b0;
            h1h2_pre_abs  <= '0;
            h1h2_word_cnt <= '0;
            h1_spike_pop  <= 1'b0;
        end else if (state == S_PROP2) begin
            h1_spike_pop <= 1'b0;
            if (!h1h2_active && !h1_spike_empty) begin
                h1_spike_pop  <= 1'b1; // pop at next cycle
                h1h2_pre_abs  <= h1_spike_out;
                h1h2_word_cnt <= '0;
                h1h2_active   <= 1'b1;
            end else if (h1h2_active) begin
                if (h1h2_word_cnt == WORDS_PER_PRE-1) begin
                    h1h2_active   <= 1'b0;
                    h1h2_word_cnt <= '0;
                end else begin
                    h1h2_word_cnt <= h1h2_word_cnt + 1'b1;
                end
            end
        end else begin
            h1h2_active  <= 1'b0;
            h1_spike_pop <= 1'b0;
        end
    end

    // Prefetch: overlap READ of next word while writing current half (address logic updated above)

    // Sparse mem read for H1->H2
    assign sp_h1_h2_cs   = (state == S_PROP2) && h1h2_active;
    assign sp_h1_h2_we   = 1'b0;
    // Prefetch next word during WRITE phase by pointing address ahead; capture indices from DO beforehand
    wire [$clog2(WORDS_PER_PRE)-1:0] h1h2_word_next = (h1h2_word_cnt == WORDS_PER_PRE-1) ? h1h2_word_cnt : (h1h2_word_cnt + 1'b1);
    assign sp_h1_h2_addr = {h1h2_pre_abs[7:0], (h1h2_rw ? h1h2_word_next : h1h2_word_cnt)[$bits(sp_h1_h2_addr)-8-1:0]};
    assign sp_h1_h2_di   = '0;
    // SPI programming of sparse memories (8-synapse packing over 4-word bursts)
    // Protocol: for a given word address, send 4 consecutive 32b data words with SPI_ADDR[1:0]=00,01,10,11
    // Layout: syn[0] in chunk0 bits[16:0], syn[1] in chunk0 bits[49:33], syn[2] in chunk1 bits[16:0], syn[3] in chunk1 bits[49:33],
    // syn[4] in chunk2 bits[16:0], syn[5] in chunk2 bits[49:33], syn[6] in chunk3 bits[16:0], syn[7] in chunk3 bits[49:33].
    // Use SPI_CODE 3'b011 for VI->H1 and 3'b100 for H1->H2
    logic [135:0] sp_pack;
    always_comb begin
        sp_vi_h1_cs = 1'b0; sp_vi_h1_we = 1'b0; sp_vi_h1_di = '0; sp_vi_h1_addr = '0;
        sp_h1_h2_cs = 1'b0; sp_h1_h2_we = 1'b0; sp_h1_h2_di = '0; sp_h1_h2_addr = '0;
    end
    always_ff @(negedge SCK, posedge rst) begin
        if (rst) begin
            sp_pack <= '0;
        end else if (SPI_EN_CONF && ~SPI_READ && ((SPI_CODE==3'b011) || (SPI_CODE==3'b100))) begin
            case (SPI_ADDR[1:0])
                2'b00: begin
                    sp_pack[16:0]   <= SPI_DATA[16:0];
                    sp_pack[49:33]  <= SPI_DATA[16:0]; // next syn in next chunk
                end
                2'b01: begin
                    sp_pack[33+16:33] <= SPI_DATA[16:0];
                    sp_pack[2*33+16:2*33] <= SPI_DATA[16:0];
                end
                2'b10: begin
                    sp_pack[3*33+16:3*33] <= SPI_DATA[16:0];
                    sp_pack[4*33+16:4*33] <= SPI_DATA[16:0];
                end
                2'b11: begin
                    sp_pack[5*33+16:5*33] <= SPI_DATA[16:0];
                    sp_pack[6*33+16:6*33] <= SPI_DATA[16:0];
                    // write full pack now
                    if (SPI_CODE==3'b011) begin
                        sp_vi_h1_cs   <= 1'b1;
                        sp_vi_h1_we   <= 1'b1;
                        sp_vi_h1_addr <= SPI_ADDR[11:0];
                        sp_vi_h1_di   <= sp_pack;
                    end else begin
                        sp_h1_h2_cs   <= 1'b1;
                        sp_h1_h2_we   <= 1'b1;
                        sp_h1_h2_addr <= SPI_ADDR[11:0];
                        sp_h1_h2_di   <= sp_pack;
                    end
                end
            endcase
        end
    end
    // Readback low 32b of sparse word on SPI read; default SR_READBACK is muxed out to SPI module
    always_comb begin
        if (SPI_EN_CONF && SPI_READ) begin
            if (SPI_CODE==3'b011) begin
                sp_vi_h1_cs   = 1'b1; sp_vi_h1_we = 1'b0; sp_vi_h1_addr = SPI_ADDR[11:0];
                SR_READBACK   = sp_vi_h1_do[31:0];
            end else if (SPI_CODE==3'b100) begin
                sp_h1_h2_cs   = 1'b1; sp_h1_h2_we = 1'b0; sp_h1_h2_addr = SPI_ADDR[11:0];
                SR_READBACK   = sp_h1_h2_do[31:0];
            end else begin
                // pass-through of previously prepared SR_READBACK
                SR_READBACK = SR_READBACK;
            end
        end
    end

    // Parallel 4-PE processing per sparse word for H1->H2: syn1..syn4 then syn5..syn8
    logic        h1h2_half;   // 0: 0..3, 1: 4..7
    logic        h1h2_rw;     // 0: READ, 1: WRITE
    logic [6:0]  h1h2_lix[NG-1:0];
    logic signed [15:0] h1h2_w16[NG-1:0];

    function automatic [6:0] get_local_ix2(input logic [135:0] word, input int slot);
        get_local_ix2 = word[17*slot + 16 -: 9][6:0];
    endfunction
    function automatic logic signed [15:0] get_w16_2(input logic [135:0] word, input int slot);
        logic signed [7:0] w8;
        begin
            w8 = word[17*slot +: 8];
            get_w16_2 = {{8{w8[7]}}, w8} <<< FP_LOC_W12;
        end
    endfunction

    always_ff @(posedge clk or posedge rst) begin
        if (rst) begin
            h1h2_half <= 1'b0;
            h1h2_rw   <= 1'b0;
        end else if (state == S_PROP2 && h1h2_active) begin
            if (!h1h2_rw) begin
                // READ: capture indices and weights for this half
                h1h2_lix[0] <= get_local_ix2(sp_h1_h2_do, (h1h2_half ? 4 : 0) + 0);
                h1h2_lix[1] <= get_local_ix2(sp_h1_h2_do, (h1h2_half ? 4 : 0) + 1);
                h1h2_lix[2] <= get_local_ix2(sp_h1_h2_do, (h1h2_half ? 4 : 0) + 2);
                h1h2_lix[3] <= get_local_ix2(sp_h1_h2_do, (h1h2_half ? 4 : 0) + 3);
                h1h2_w16[0] <= get_w16_2   (sp_h1_h2_do, (h1h2_half ? 4 : 0) + 0);
                h1h2_w16[1] <= get_w16_2   (sp_h1_h2_do, (h1h2_half ? 4 : 0) + 1);
                h1h2_w16[2] <= get_w16_2   (sp_h1_h2_do, (h1h2_half ? 4 : 0) + 2);
                h1h2_w16[3] <= get_w16_2   (sp_h1_h2_do, (h1h2_half ? 4 : 0) + 3);
                h1h2_rw     <= 1'b1;
            end else begin
                // WRITE: update all 4 banks and advance
                h1h2_rw   <= 1'b0;
                if (!h1h2_half) begin
                    h1h2_half <= 1'b1;
                end else begin
                    h1h2_half <= 1'b0;
                end
            end
        end else begin
            h1h2_rw   <= 1'b0;
            h1h2_half <= 1'b0;
        end
    end

    always_comb begin
        if (state == S_PROP2 && h1h2_active) begin
            for (int g=0; g<NG; g++) begin
                h2_cs[g]   = 1'b0;
                h2_we[g]   = 1'b0;
                h2_addr[g] = '0;
                h2_di[g]   = '0;
                h2_mask[g] = '0;
            end
            if (!h1h2_rw) begin
                // READ addresses for all 4 banks (combinational index)
                for (int g=0; g<NG; g++) begin
                    h2_cs[g]   = 1'b1;
                    h2_we[g]   = 1'b0;
                    h2_addr[g] = h1h2_lix[g];
                end
            end else begin
                // WRITE back u + w16 into all 4 banks; no H2 trace increment on H1 events (traces are output-only)
                for (int g=0; g<NG; g++) begin
                    logic signed [15:0] u_cur;
                    logic signed [15:0] u_sum;
                    logic signed [15:0] u_sat;
                    u_cur = h2_do[g][79:64];
                    u_sum = u_cur + h1h2_w16[g];
                    u_sat = ( u_cur[15] &  h1h2_w16[g][15] & ~u_sum[15]) ? 16'sh8000 :
                            (~u_cur[15] & ~h1h2_w16[g][15] &  u_sum[15]) ? 16'sh7FFF : u_sum;
                    h2_cs[g]                 = 1'b1;
                    h2_we[g]                 = 1'b1;
                    h2_addr[g]               = h1h2_lix[g];
                    h2_di[g][79:64]          = u_sat;
                    h2_mask[g][79:64]        = {16{1'b1}};
                end
            end
        end
    end

    // ------------------------------------------------------------------------
    // H2 STEP: leak + threshold + spike to FIFO for output accumulation
    // ------------------------------------------------------------------------
    logic        step2_do;
    always_ff @(posedge clk or posedge rst) begin
        if (rst) begin
            step2_do    <= 1'b0;
            step_h2_idx <= '0;
            // Clear H2 delay lines
            for (int g=0; g<NG; g++) begin
                for (int i=0; i<G_H2; i++) begin
                    h2_hist1[g][i] <= '0; h2_hist2[g][i] <= '0; h2_hist3[g][i] <= '0; h2_hist4[g][i] <= '0;
                    h2_hist5[g][i] <= '0; h2_hist6[g][i] <= '0; h2_hist7[g][i] <= '0;
                    h2_prev1[g][i] <= '0; h2_prev2[g][i] <= '0; h2_prev3[g][i] <= '0; h2_prev4[g][i] <= '0;
                    h2_prev5[g][i] <= '0; h2_prev6[g][i] <= '0; h2_prev7[g][i] <= '0;
                end
            end
        end else if (state == S_STEP2 && tick_rise) begin
            step2_do <= 1'b1;
        end else begin
            step2_do <= 1'b0;
        end
    end

    // Threshold gating computed once per tick after STEP sweeps latch similarity
    // i = average input spike at current timestep -> approximate by aer_cnt_tick (events count)
    // Thr_pred = i * h_pred, Thr_contr = i * h_contr; Gate no-update if (i < Thr_i) or (pred > Thr_pred and contr < Thr_contr)
    always_ff @(posedge clk or posedge rst) begin
        if (rst) begin
            H1_NO_UPDATE <= 1'b0;
            H2_NO_UPDATE <= 1'b0;
        end else begin
            // Evaluate after both sweeps completed (end of STEP2). Here we recompute every cycle, but values change at sweep end.
            // i: averaged/normalized input activity
            automatic logic [31:0] i_raw = {24'd0, aer_cnt_tick};
            automatic logic [31:0] i_val = (i_raw * SPI_I_SCALE) >> SPI_I_SHIFT;
            // Layer-specific thresholds
            automatic logic [31:0] thr_pred1 = i_val * SPI_H1_PRED;
            automatic logic [31:0] thr_contr1= i_val * SPI_H1_CONTR;
            automatic logic [31:0] thr_pred2 = i_val * SPI_H2_PRED;
            automatic logic [31:0] thr_contr2= i_val * SPI_H2_CONTR;
            automatic logic        cond_low_i = (i_val[15:0] < SPI_THR_I);
            automatic logic        cond_dir1  = (sim_pred_h1_scaled > thr_pred1) && (sim_contr_h1_scaled < thr_contr1);
            automatic logic        cond_dir2  = (sim_pred_h2_scaled > thr_pred2) && (sim_contr_h2_scaled < thr_contr2);
            H1_NO_UPDATE <= cond_low_i || cond_dir1;
            H2_NO_UPDATE <= cond_low_i || cond_dir2;
        end
    end

    integer g2;
    always_comb begin
        for (g2 = 0; g2 < NG; g2++) begin
            h2_cs[g2]   = 1'b0;
            h2_we[g2]   = 1'b0;
            h2_addr[g2] = step_h2_idx;
            h2_di[g2]   = '0;
            h2_mask[g2] = '0;
        end
    end

    // STEP2: parallel 4-bank read/write per index; stage spikes and push sequentially
    logic [M_GH2-1:0] h2_step_idx;
    logic             h2_step_rw; // 0: READ, 1: WRITE
    logic [3:0]       h2_spk_pending;
    logic [8:0]       h2_spk_idx   [NG-1:0];
    // H2 delay buffers implemented as SRAM-based circular buffers
    logic [127:0]      h2_hist_q   [NG-1:0];
    logic              h2_hist_cs  [NG-1:0];
    logic              h2_hist_we  [NG-1:0];
    logic [M_GH2-1:0]  h2_hist_addr[NG-1:0];
    logic [127:0]      h2_hist_di  [NG-1:0];
    logic [127:0]      h2_hist_mask[NG-1:0];
    logic [2:0]        h2_hist_wptr[NG-1:0];
    logic [127:0]      h2_prev_q   [NG-1:0];
    logic              h2_prev_cs  [NG-1:0];
    logic              h2_prev_we  [NG-1:0];
    logic [M_GH2-1:0]  h2_prev_addr[NG-1:0];
    logic [127:0]      h2_prev_di  [NG-1:0];
    logic [127:0]      h2_prev_mask[NG-1:0];
    logic [2:0]        h2_prev_wptr[NG-1:0];
    logic [2:0]        h2_prev_wptr_snap[NG-1:0];

    // H1/H2 prev snapshot window: asserted for one full sweep after sample_fall
    logic h1_prev_snap, h2_prev_snap;
    always_ff @(posedge clk or posedge rst) begin
        if (rst) begin
            h1_prev_snap <= 1'b0;
            h2_prev_snap <= 1'b0;
        end else begin
            // Launch windows on falling edge of SAMPLE
            if (sample_fall) begin
                h1_prev_snap <= 1'b1;
                h2_prev_snap <= 1'b1;
                // snapshot write pointers for prev circular buffers
                for (int g=0; g<NG; g++) begin
                    h1_prev_wptr_snap[g] <= h1_prev_wptr[g];
                    h2_prev_wptr_snap[g] <= h2_prev_wptr[g];
                end
            end
            // End windows when sweep returns to zero in respective STEP states
            if (state == S_STEP1 && h1_step_rw == 1'b0 && h1_step_idx == '0) begin
                // Next READ cycle indicates new sweep start; if window was active and we've just wrapped, clear it
                if (h1_prev_snap && h1_step_idx == '0) h1_prev_snap <= 1'b0;
            end
            if (state == S_STEP2 && h2_step_rw == 1'b0 && h2_step_idx == '0) begin
                if (h2_prev_snap && h2_step_idx == '0) h2_prev_snap <= 1'b0;
            end
        end
    end

    always_ff @(posedge clk or posedge rst) begin
        if (rst) begin
            h2_step_idx    <= '0;
            h2_step_rw     <= 1'b0;
            h2_spk_pending <= 4'b0000;
            sim_pred_h2    <= 32'd0;
            sim_contr_h2   <= 32'd0;
        end else if (state == S_STEP2) begin
            // push pending spikes one per cycle if any
            if (|h2_spk_pending) begin
                for (int g=0; g<NG; g++) begin
                    if (h2_spk_pending[g]) begin
                        h2_spike_push <= 1'b1;
                        h2_spike_in   <= h2_spk_idx[g];
                        h2_spk_pending[g] <= 1'b0;
                        break;
                    end
                end
            end else begin
                h2_spike_push <= 1'b0;
            end

            if (!h2_step_rw) begin
                h2_step_rw <= 1'b1;
                if (h2_step_idx == '0) begin
                    sim_pred_h2      <= 32'd0;
                    sim_contr_h2     <= 32'd0;
                    h2_spk_cnt_tick  <= 16'd0;
                end
            end else begin
                // collect spikes and advance
                for (int g=0; g<NG; g++) begin
                    logic signed [15:0] u   = h2_do[g][79:64];
                    logic signed [15:0] thr = h2_do[g][111:96];
                    logic               spk = (u >= thr);
                    if (spk) begin
                        h2_spk_pending[g] <= 1'b1;
                        h2_spk_idx[g]     <= {g[1:0], h2_step_idx};
                        h2_spk_cnt_tick   <= h2_spk_cnt_tick + 16'd1;
                        // Accumulate similarity using selected taps
                        logic [2:0] sel_hist2 = (h2_hist_wptr[g] + 3'd7 - tap_hist_sel);
                        logic [2:0] sel_prev2 = (h2_prev_wptr_snap[g] + 3'd7 - tap_prev_sel);
                        logic [11:0] t_hist2  = (sel_hist2==3'd0)? h2_do[g][HIST_BASE+ 0 +:12] :
                                                (sel_hist2==3'd1)? h2_do[g][HIST_BASE+16 +:12] :
                                                (sel_hist2==3'd2)? h2_do[g][HIST_BASE+32 +:12] :
                                                (sel_hist2==3'd3)? h2_do[g][HIST_BASE+48 +:12] :
                                                (sel_hist2==3'd4)? h2_do[g][HIST_BASE+64 +:12] :
                                                (sel_hist2==3'd5)? h2_do[g][HIST_BASE+80 +:12] :
                                                (sel_hist2==3'd6)? h2_do[g][HIST_BASE+96 +:12] :
                                                                   h2_do[g][HIST_BASE+112+:12];
                        logic [11:0] t_prev2  = (sel_prev2==3'd0)? h2_do[g][PREV_BASE+ 0 +:12] :
                                                (sel_prev2==3'd1)? h2_do[g][PREV_BASE+16 +:12] :
                                                (sel_prev2==3'd2)? h2_do[g][PREV_BASE+32 +:12] :
                                                (sel_prev2==3'd3)? h2_do[g][PREV_BASE+48 +:12] :
                                                (sel_prev2==3'd4)? h2_do[g][PREV_BASE+64 +:12] :
                                                (sel_prev2==3'd5)? h2_do[g][PREV_BASE+80 +:12] :
                                                (sel_prev2==3'd6)? h2_do[g][PREV_BASE+96 +:12] :
                                                                   h2_do[g][PREV_BASE+112+:12];
                        sim_pred_h2  <= sim_pred_h2  + t_hist2;
                        sim_contr_h2 <= sim_contr_h2 + t_prev2;
                    end
                end
                h2_step_rw  <= 1'b0;
                h2_step_idx <= (h2_step_idx == G_H2-1) ? '0 : (h2_step_idx + 1'b1);
                if (h2_step_idx == G_H2-1) begin
                    sim_pred_h2_latched  <= sim_pred_h2;
                    sim_contr_h2_latched <= sim_contr_h2;
                    h2_spk_cnt_latched   <= h2_spk_cnt_tick;
                    automatic logic [31:0] denom_h2 = 32'd1;
                    case (SPI_SIM_NORM_MODE)
                        2'd1: denom_h2 = (h2_spk_cnt_tick == 16'd0) ? 32'd1 : {16'd0,h2_spk_cnt_tick};
                        2'd2: denom_h2 = {23'd0, SPI_NUM_H1_NEUR} + 32'd1; // H2 neuron count not explicit; approximate with H1 count+1
                        default: denom_h2 = 32'd1;
                    endcase
                    sim_pred_h2_scaled   <= ((sim_pred_h2  / denom_h2) * SPI_SIM_SCALE) >> SPI_SIM_SHIFT;
                    sim_contr_h2_scaled  <= ((sim_contr_h2 / denom_h2) * SPI_SIM_SCALE) >> SPI_SIM_SHIFT;
                end
            end
        end else begin
            h2_spike_push <= 1'b0;
            h2_step_rw    <= 1'b0;
        end
    end

    always_comb begin
        if (state == S_STEP2) begin
            for (int g=0; g<NG; g++) begin
                h2_cs[g]   = 1'b0;
                h2_we[g]   = 1'b0;
                h2_addr[g] = '0;
                h2_di[g]   = '0;
                h2_mask[g] = '0;
            end
            if (!h2_step_rw) begin
                for (int g=0; g<NG; g++) begin
                    h2_cs[g]   = 1'b1;
                    h2_we[g]   = 1'b0;
                    h2_addr[g] = h2_step_idx;
                end
            end else begin
                for (int g=0; g<NG; g++) begin
                    logic signed [15:0] u      = h2_do[g][79:64];
                    logic signed [15:0] thr    = h2_do[g][111:96];
                    logic [15:0]        alpha  = h2_do[g][127:112];
                    logic signed [31:0] u_mul  = $signed({1'b0,u}) * $signed({1'b0,alpha});
                    // stochastic rounding on leak
                    logic [14:0] frac = u_mul[14:0];
                    logic signed [15:0] u_leak = (SPI_EN_STOCH_ROUND && (alpha != 16'h8000) && (frac >= rnd_strnd) && |u_mul[30:15])
                                                 ? ($signed(u_mul[30:15]) + 16'sd1) : $signed(u_mul[30:15]);
                    logic               spk    = (u >= thr);
                    logic signed [15:0] u_next = spk ? (u_leak - thr) : u_leak;
                    // H2 trace: increment on spike then decay by alpha
                    logic [11:0] th1_cur = h2_do[g][83:72];
                    logic [11:0] th1_inc = spk ? (th1_cur + (12'd2 << EFF_FP_LOC_TOUT)) : th1_cur;
                    logic [27:0] th1_mul = {12'b0,th1_inc} * {12'b0,alpha};
                    logic [14:0] th1_frac= th1_mul[14:0];
                    logic [11:0] th1_dec = (SPI_EN_STOCH_ROUND && (alpha != 16'h8000) && (th1_frac >= rnd_strnd[11:0]) && |th1_mul[26:15])
                                            ? (th1_mul[26:15] + 12'd1) : th1_mul[26:15];
                    // Refractory/homeostasis: adjust threshold softly
                    logic signed [15:0] thr_upd = spk ? (thr + {{8{SPI_REF_ADD_H2[7]}}, SPI_REF_ADD_H2}) : (thr - {{8{SPI_REF_DEC_H2[7]}}, SPI_REF_DEC_H2});
                    // Delay lines for H2 (shift on each writeback)
                    logic [11:0] thist_next;
                    unique case (h2_hist_wptr[g])
                        3'd0: thist_next = h2_do[g][HIST_BASE +  0 +: 12];
                        3'd1: thist_next = h2_do[g][HIST_BASE + 16 +: 12];
                        3'd2: thist_next = h2_do[g][HIST_BASE + 32 +: 12];
                        3'd3: thist_next = h2_do[g][HIST_BASE + 48 +: 12];
                        3'd4: thist_next = h2_do[g][HIST_BASE + 64 +: 12];
                        3'd5: thist_next = h2_do[g][HIST_BASE + 80 +: 12];
                        3'd6: thist_next = h2_do[g][HIST_BASE + 96 +: 12];
                        default: thist_next = h2_do[g][HIST_BASE + 112 +: 12];
                    endcase
                    logic [11:0] tprev_cur  = h2_do[g][59:48];
                    logic [11:0] tprev_next;
                    unique case (h2_prev_wptr[g])
                        3'd0: tprev_next = h2_do[g][PREV_BASE +  0 +: 12];
                        3'd1: tprev_next = h2_do[g][PREV_BASE + 16 +: 12];
                        3'd2: tprev_next = h2_do[g][PREV_BASE + 32 +: 12];
                        3'd3: tprev_next = h2_do[g][PREV_BASE + 48 +: 12];
                        3'd4: tprev_next = h2_do[g][PREV_BASE + 64 +: 12];
                        3'd5: tprev_next = h2_do[g][PREV_BASE + 80 +: 12];
                        3'd6: tprev_next = h2_do[g][PREV_BASE + 96 +: 12];
                        default: tprev_next = h2_do[g][PREV_BASE + 112 +: 12];
                    endcase
                    // Advance circular buffers for H2
                    // combined SRAM writes for H2
                    h2_cs[g]    <= 1'b1;
                    h2_we[g]    <= 1'b1;
                    h2_addr[g]  <= h2_step_idx;
                    h2_di[g]    <= '0;
                    h2_mask[g]  <= '0;
                    h2_di[g][HIST_BASE + {h2_hist_wptr[g], 4'b0000} +: 12]   <= th1_cur;
                    h2_mask[g][HIST_BASE + {h2_hist_wptr[g], 4'b0000} +: 12] <= 12'hFFF;
                    h2_hist_wptr[g]  <= (h2_hist_wptr[g] == 3'd7) ? 3'd0 : (h2_hist_wptr[g] + 3'd1);
                    if (SAMPLE) begin
                        h2_di[g][PREV_BASE + {h2_prev_wptr[g], 4'b0000} +: 12]   <= th1_cur;
                        h2_mask[g][PREV_BASE + {h2_prev_wptr[g], 4'b0000} +: 12] <= 12'hFFF;
                        h2_prev_wptr[g] <= (h2_prev_wptr[g] == 3'd7) ? 3'd0 : (h2_prev_wptr[g] + 3'd1);
                    end
                    h2_di[g][79:64]          = u_next;
                    h2_mask[g][79:64]        = {16{(u_next != u)}};
                    h2_di[g][83:72]          = th1_dec;
                    h2_mask[g][83:72]        = {12{(th1_dec != th1_cur)}};
                    h2_di[g][71:60]          = thist_next;
                    h2_mask[g][71:60]        = {12{(thist_next != h2_do[g][71:60])}};
                    h2_di[g][59:48]          = tprev_next;
                    h2_mask[g][59:48]        = {12{(tprev_next != tprev_cur)}};
                    h2_di[g][111:96]         = thr_upd;
                    h2_mask[g][111:96]       = {16{(thr_upd != thr)}};
                end
            end
        end
    end

    // ------------------------------------------------------------------------
    // Output accumulation on H2 spikes using Wout row (128b => 16x8b weights) + activation
    // ------------------------------------------------------------------------
    // Accumulate immediately when a spike pops
    logic        out_pop_pending;
    always_ff @(posedge clk or posedge rst) begin
        if (rst) begin
            out_pop_pending <= 1'b0;
            wout_cs         <= 1'b0;
            wout_we         <= 1'b0;
            wout_addr       <= '0;
            wout_di         <= '0;
            wout_mask       <= '0;
            out_learn_active<= 1'b0; out_lrn_pre <= '0; out_lrn_k <= '0; wout_row_q <= '0;
        end else begin
            wout_cs   <= 1'b0;
            wout_we   <= 1'b0;
            wout_di   <= '0;
            wout_mask <= '0;
            // Latch label at sample start; clear at sample end
            if (SPI_LABEL_USE_SAMPLE) begin
                if (sample_q & ~SAMPLE) begin
                    label_active  <= 1'b0;
                end else if (~sample_q & SAMPLE) begin
                    label_active  <= LABEL_VALID;
                    label_idx_reg <= LABEL_INDEX;
                end
            end else begin
                label_active  <= LABEL_VALID;
                if (LABEL_VALID) label_idx_reg <= LABEL_INDEX;
            end
            if ((state == S_STEP2) && !h2_spike_empty && !out_pop_pending && !ol_busy) begin
                out_pop_pending <= 1'b1;
                // Initiate read of Wout row for presyn index
                wout_cs   <= 1'b1;
                wout_addr <= {1'b0, h2_spike_out};
            end else if (out_pop_pending) begin
                // One cycle later, wout_do valid: accumulate into outputs
                out_pop_pending <= 1'b0;
                for (int k = 0; k < N_OUT; k++) begin
                    automatic logic signed [7:0] w8  = wout_do[8*k +: 8];
                    automatic logic signed [15:0] w16= {{8{w8[7]}}, w8} <<< EFF_FP_LOC_WOUT;
                    // Optional output noise
                    automatic logic signed [17:0] noise_raw = $signed({1'b0, rnd_noise, 1'b0}) - 18'sd131072; // center
                    automatic logic signed [15:0] noise = SPI_NOISE_EN ? $signed(noise_raw[17:2]) >>> SPI_NOISE_STR : 16'sd0;
                    automatic logic signed [15:0] bias16 = {{8{out_bias8[k][7]}}, out_bias8[k]} <<< EFF_FP_LOC_WOUT;
                    automatic logic signed [15:0] s  = out_u[k] + w16 + bias16 + noise;
                    out_u[k] <= ( out_u[k][15] &  w16[15] & ~s[15]) ? 16'sh8000 :
                                (~out_u[k][15] & ~w16[15] &  s[15]) ? 16'sh7FFF : s;
                end
                // Pop spike now
                h2_spike_pop <= 1'b1;

                // If supervised label present and enabled, start output-layer learning for this pre
                ol_start  <= SPI_SUP_OUT_EN && label_active;
                ol_pre_idx<= h2_spike_out;
                ol_row_in <= wout_do;
            end else begin
                h2_spike_pop <= 1'b0;
            end
        end
    end

    // Instantiate output learner
    output_learner #(.N_OUT(N_OUT), .M_OUT(M_OUT)) u_outlearn (
        .clk(clk), .rst(rst),
        .sup_en(SPI_SUP_OUT_EN), .learn_use_act(SPI_OUT_LEARN_USE_ACT), .lr_q8(SPI_LR_OUT), .err_shift(SPI_OUT_ERR_SCALE), .grad_max(SPI_OUT_GRAD_MAX),
        .err_max(SPI_OUT_ERR_MAX), .ste_en(SPI_OUT_STE_EN), .pre_scale_q8(SPI_OUT_PRE_SCALE), .wdec_q8(SPI_OUT_WDEC), .stoch_en(SPI_OUT_STOCH_EN),
        .label_active(label_active), .label_idx(label_idx_reg), .out_u(out_u), .act_y(act_y),
        .start(ol_start), .pre_idx(ol_pre_idx), .row_w8(ol_row_in), .rnd_bit(rnd_noise[0]),
        .busy(ol_busy), .done(ol_done), .pre_idx_o(), .row_w8_o(ol_row_out), .err_vec(out_err16)
    );

    // Writeback when learner completes
    always_ff @(posedge clk or posedge rst) begin
        if (rst) begin
            // no-op
        end else if (ol_done) begin
            wout_cs   <= 1'b1; wout_we <= 1'b1; wout_addr <= {1'b0, ol_pre_idx}; wout_di <= ol_row_out; wout_mask <= '0;
        end
    end

    // Optional stochastic rounding and noise: LFSRs (placeholders)
    logic [14:0] rnd_strnd;
    logic [16:0] rnd_noise;
    lfsr_param #(.WIDTH(15), .TAPS(15'b110000000000000)) lfsr_strnd (
        .clk(clk), .rst(rst), .en(SPI_EN_STOCH_ROUND), .seed(SPI_SEED_STRND), .rnd(rnd_strnd)
    );
    lfsr_param #(.WIDTH(17), .TAPS(17'b10010000000000000)) lfsr_noise (
        .clk(clk), .rst(rst), .en(SPI_NOISE_EN), .seed(SPI_SEED_NOISE), .rnd(rnd_noise)
    );
    // Output leak (optional): for simplicity, keep outputs as pure integrators here.

    // Winner selection and output bus (label at end-of-sample)
    always_ff @(posedge clk or posedge rst) begin
        if (rst) begin
            for (int k = 0; k < N_OUT; k++) out_u[k] <= '0;
            OUT_REQ   <= 1'b0;
            OUT_DATA  <= 8'h00;
            winner_idx<= '0;
        end else begin
            if (state == S_STEP2 && time_tick) begin
                // Hard-sigmoid activation if enabled
                if (!SPI_NO_OUT_ACT) begin
                    for (int k=0; k<N_OUT; k++) begin
                        // map [-2048, 2048] to [0, 2048] and clamp
                        automatic logic signed [15:0] vo = out_u[k];
                        if      (vo >  $signed(16'd2047)) act_y[k] <= 16'sd2048;
                        else if (vo <  $signed(-16'd2048)) act_y[k] <= 16'sd0;
                        else                                act_y[k] <= (vo + 16'sd2048) >>> 1;
                    end
                end else begin
                    for (int k=0; k<N_OUT; k++) act_y[k] <= out_u[k];
                end
            end
            if (state == S_STEP2 && time_tick && INFER_ACC && !SPI_REGRESSION) begin
                // Per-timestep winner accumulation
                automatic logic signed [15:0] best_val;
                automatic logic [M_OUT-1:0]  best_idx;
                best_val = act_y[0];
                best_idx = '0;
                for (int k = 1; k < N_OUT; k++) begin
                    if (k <= SPI_NUM_OUT_NEUR && act_y[k] > best_val) begin
                        best_val = act_y[k];
                        best_idx = k[M_OUT-1:0];
                    end
                end
                cnt_winner[best_idx] <= cnt_winner[best_idx] + 16'd1;
            end else if (state == S_SEND) begin
                // Emit result (label or streaming values)
                if (SPI_SEND_LABEL_ONLY && !SPI_REGRESSION) begin
                    // Compute winner over counts
                    automatic logic [15:0] aer_best;
                    automatic logic [M_OUT-1:0] send_best;
                    aer_best = cnt_winner[0];
                    send_best= '0;
                    for (int k = 1; k < N_OUT; k++) begin
                        if (k <= SPI_NUM_OUT_NEUR && cnt_winner[k] > aer_best) begin
                            aer_best = cnt_winner[k];
                            send_best= k[M_OUT-1:0];
                        end
                    end
                    winner_idx <= send_best;
                    OUT_REQ    <= 1'b1;
                    OUT_DATA   <= {4'h0, send_best[3:0]};
                    if (OUT_ACK) begin
                        OUT_REQ <= 1'b0;
                        // Clear for next sample
                        for (int k2 = 0; k2 < N_OUT; k2++) begin
                            out_u[k2]      <= '0;
                            cnt_winner[k2] <= '0;
                        end
                    end
                end else begin
                    // Streaming outputs: send 16-bit per enabled neuron (raw or activated per SPI_STREAM_ACT)
                    static logic [4:0] send_cnt;
                    automatic logic [4:0] send_num = {SPI_NUM_OUT_NEUR,1'b1};
                    if (state == S_STEP2) send_cnt <= '0;
                    if (!OUT_ACK && !OUT_REQ && (send_cnt <= send_num)) begin
                        OUT_REQ  <= 1'b1;
                        OUT_DATA <= send_cnt[0] ? (SPI_STREAM_ACT ? act_y[send_cnt[4:1]][15:8] : out_u[send_cnt[4:1]][15:8])
                                                : (SPI_STREAM_ACT ? act_y[send_cnt[4:1]][7:0]  : out_u[send_cnt[4:1]][7:0]);
                    end else if (OUT_ACK && OUT_REQ) begin
                        OUT_REQ  <= 1'b0;
                        send_cnt <= send_cnt + 1'b1;
                    end
                end
            end else begin
                OUT_REQ <= 1'b0;
            end
        end
    end

    // SPI slave for configuration (minimal)
    spi_elfcore spi0 (
        .SCK(SCK), .MOSI(MOSI), .CS_N(1'b0), .MISO(MISO), .RST(rst), .clk(clk),
        // Config outputs
        .SPI_EN_CONF(SPI_EN_CONF), .SPI_LOCAL_TICK(SPI_LOCAL_TICK), .SPI_ERROR_HALT(SPI_ERROR_HALT),
        .SPI_TIMING_MODE(SPI_TIMING_MODE), .SPI_REGRESSION(SPI_REGRESSION), .SPI_SEND_PER_TIMESTEP(SPI_SEND_PER_TIMESTEP),
        .SPI_SEND_LABEL_ONLY(SPI_SEND_LABEL_ONLY), .SPI_NO_OUT_ACT(SPI_NO_OUT_ACT), .SPI_EN_STOCH_ROUND(SPI_EN_STOCH_ROUND),
        .SPI_NOISE_EN(SPI_NOISE_EN), .SPI_NOISE_STR(SPI_NOISE_STR), .SPI_CYCLES_PER_TICK(SPI_CYCLES_PER_TICK),
        .EFF_FP_LOC_TINP(EFF_FP_LOC_TINP), .EFF_FP_LOC_TOUT(EFF_FP_LOC_TOUT), .EFF_FP_LOC_WOUT(EFF_FP_LOC_WOUT),
        .SPI_NUM_INP_NEUR(SPI_NUM_INP_NEUR), .SPI_NUM_H1_NEUR(SPI_NUM_H1_NEUR), .SPI_NUM_OUT_NEUR(SPI_NUM_OUT_NEUR),
        .SPI_SEED_STRND(SPI_SEED_STRND), .SPI_SEED_NOISE(SPI_SEED_NOISE), .SPI_STREAM_ACT(SPI_STREAM_ACT),
        // Output supervised learning and VI base
        .SPI_SUP_OUT_EN(SPI_SUP_OUT_EN), .SPI_LR_OUT(SPI_LR_OUT), .SPI_OUT_STOCH_EN(SPI_OUT_STOCH_EN), .SPI_OUT_ERR_SCALE(SPI_OUT_ERR_SCALE),
        .SPI_OUT_ERR_MAX(SPI_OUT_ERR_MAX), .SPI_OUT_STE_EN(SPI_OUT_STE_EN), .SPI_OUT_PRE_SCALE(SPI_OUT_PRE_SCALE), .SPI_OUT_WDEC(SPI_OUT_WDEC), .SPI_OUT_LEARN_USE_ACT(SPI_OUT_LEARN_USE_ACT),
        .SPI_VI_BASE(SPI_VI_BASE),
        .SPI_ADDR(SPI_ADDR), .SPI_DATA(SPI_DATA), .SPI_CODE(SPI_CODE), .SPI_READ(SPI_READ),
        .PROG_NEUR_H1(PROG_NEUR_H1), .PROG_NEUR_H2(PROG_NEUR_H2), .PROG_WOUT(PROG_WOUT),
        .READ_NEUR_H1(READ_NEUR_H1), .READ_NEUR_H2(READ_NEUR_H2), .READ_WOUT(READ_WOUT),
        .SR_READBACK(SR_READBACK),
        // Per-bank speed modes
        .SPI_SRAM_SPEEDMODE_H1_0(SPI_SRAM_SPEEDMODE_H1[0]),
        .SPI_SRAM_SPEEDMODE_H1_1(SPI_SRAM_SPEEDMODE_H1[1]),
        .SPI_SRAM_SPEEDMODE_H1_2(SPI_SRAM_SPEEDMODE_H1[2]),
        .SPI_SRAM_SPEEDMODE_H1_3(SPI_SRAM_SPEEDMODE_H1[3]),
        .SPI_SRAM_SPEEDMODE_H2_0(SPI_SRAM_SPEEDMODE_H2[0]),
        .SPI_SRAM_SPEEDMODE_H2_1(SPI_SRAM_SPEEDMODE_H2[1]),
        .SPI_SRAM_SPEEDMODE_H2_2(SPI_SRAM_SPEEDMODE_H2[2]),
        .SPI_SRAM_SPEEDMODE_H2_3(SPI_SRAM_SPEEDMODE_H2[3]),
        // Perf counters
        .CNT_TICKS(CNT_TICKS), .CNT_SPIKES_H1(CNT_SPIKES_H1), .CNT_SPIKES_H2(CNT_SPIKES_H2), .AER_FIFO_HWM(AER_FIFO_HWM),
        .SPI_THIST_DEPTH(SPI_THIST_DEPTH), .SPI_TPREV_DEPTH(SPI_TPREV_DEPTH),
        .SIM_PRED_H1(sim_pred_h1_scaled), .SIM_CONTR_H1(sim_contr_h1_scaled),
        .SIM_PRED_H2(sim_pred_h2_scaled), .SIM_CONTR_H2(sim_contr_h2_scaled),
        .H1_SPK_CNT(h1_spk_cnt_latched), .H2_SPK_CNT(h2_spk_cnt_latched),
        .SPI_SIM_SCALE(SPI_SIM_SCALE), .SPI_SIM_SHIFT(SPI_SIM_SHIFT), .SPI_SIM_NORM_MODE(SPI_SIM_NORM_MODE),
        .SPI_THR_I(SPI_THR_I), .SPI_H1_PRED(SPI_H1_PRED), .SPI_H1_CONTR(SPI_H1_CONTR), .SPI_H2_PRED(SPI_H2_PRED), .SPI_H2_CONTR(SPI_H2_CONTR),
        .SPI_I_SCALE(SPI_I_SCALE), .SPI_I_SHIFT(SPI_I_SHIFT),
        .LEARN_L1_PRE({24'd0, l1_pre_cnt}), .LEARN_L1_WORD({28'd0, l1_word}),
        .LEARN_L2_PRE({23'd0, l2_pre_cnt}), .LEARN_L2_WORD({28'd0, l2_word}),
        .H1_NO_UPDATE(H1_NO_UPDATE), .H2_NO_UPDATE(H2_NO_UPDATE),
        // DST controls
        .SPI_DYN_EN_H1(SPI_DYN_EN_H1), .SPI_DYN_EN_H2(SPI_DYN_EN_H2), .SPI_DYN_X_H1(SPI_DYN_X_H1), .SPI_DYN_X_H2(SPI_DYN_X_H2),
        .SPI_PRUNE_K_H1(SPI_PRUNE_K_H1), .SPI_PRUNE_K_H2(SPI_PRUNE_K_H2), .SPI_COLLISION_EN(SPI_COLLISION_EN), .SPI_SEED_DST(SPI_SEED_DST), .SPI_CFG_COMMIT()
    );

    // Diagnostics
    assign SPI_RDY = SPI_EN_CONF;
    always_ff @(posedge clk or posedge rst) begin
        if (rst) begin
            CNT_TICKS      <= 32'd0;
            CNT_SPIKES_H1  <= 16'd0;
            CNT_SPIKES_H2  <= 16'd0;
            AER_FIFO_HWM   <= 8'd0;
            steps_since_dst_h1 <= 16'd0;
            steps_since_dst_h2 <= 16'd0;
            dst1_state <= D_IDLE; dst2_state <= D_IDLE;
        end else begin
            if (time_tick) CNT_TICKS <= CNT_TICKS + 32'd1;
            if (h1_spike_push) CNT_SPIKES_H1 <= CNT_SPIKES_H1 + 16'd1;
            if (h2_spike_push) CNT_SPIKES_H2 <= CNT_SPIKES_H2 + 16'd1;
            if (aer_fifo_level > AER_FIFO_HWM) AER_FIFO_HWM <= aer_fifo_level[7:0];
        end
    end

    // ------------------------------------------------------------------------
    // Memory instances
    // ------------------------------------------------------------------------
    // Hidden neuron banks (H1 and H2) with SPI programming/readback overlay
    genvar b;
    generate
        for (b = 0; b < NG; b++) begin : g_h1_banks
            SRAM_128x320_wrapper h1_bank (
                .CK (clk),
                .SM (SPI_SRAM_SPEEDMODE_H1[b][1:0]),
                .CS (h1_cs_i[b]),
                .WE (h1_we_i[b]),
                .A  (h1_addr_i[b]),
                .D  (h1_di_i[b]),
                .M  (h1_mask_i[b]),
                .Q  (h1_do[b])
            );
        end
        for (b = 0; b < NG; b++) begin : g_h2_banks
            SRAM_128x320_wrapper h2_bank (
                .CK (clk),
                .SM (SPI_SRAM_SPEEDMODE_H2[b][1:0]),
                .CS (h2_cs_i[b]),
                .WE (h2_we_i[b]),
                .A  (h2_addr_i[b]),
                .D  (h2_di_i[b]),
                .M  (h2_mask_i[b]),
                .Q  (h2_do[b])
            );
        end
    endgenerate

    // Programming/readback overlay muxing for H1/H2/Wout
    // Default to runtime control signals
    always_comb begin
        for (int g=0; g<NG; g++) begin
            h1_cs_i[g]   = h1_cs[g];
            h1_we_i[g]   = h1_we[g];
            h1_addr_i[g] = h1_addr[g];
            h1_di_i[g]   = h1_di[g];
            h1_mask_i[g] = h1_mask[g];
            h2_cs_i[g]   = h2_cs[g];
            h2_we_i[g]   = h2_we[g];
            h2_addr_i[g] = h2_addr[g];
            h2_di_i[g]   = h2_di[g];
            h2_mask_i[g] = h2_mask[g];
        end
        wout_cs_i   = wout_cs;
        wout_we_i   = wout_we;
        wout_addr_i = wout_addr;
        wout_di_i   = wout_di;
        wout_mask_i = wout_mask;
        // SPI programming overrides when freeze is enabled
        if (SPI_EN_CONF) begin
            if (PROG_NEUR_H1 || READ_NEUR_H1) begin
                // SPI_ADDR: [8:2] word, [1:0] 32b chunk
                for (int g=0; g<NG; g++) begin
                    h1_cs_i[g]   = (g==SPI_BANK_SEL_H1);
                    h1_we_i[g]   = PROG_NEUR_H1;
                    h1_addr_i[g] = SPI_ADDR[8:2];
                    // Allow reading both T_hist and T_prev via high pages when READ_NEUR_H1 is asserted
                    // page select: SPI_ADDR[10:9] -> 00: core [127:0], 01: hist [223:128], 10: prev [319:224]
                    unique case (SPI_ADDR[10:9])
                        2'b00: begin
                            h1_di_i[g]   = {192'd0, SPI_DATA} << ({6'd0, SPI_ADDR[1:0]}<<5);
                            h1_mask_i[g] = {192'd0, 32'hFFFF_FFFF} << ({6'd0, SPI_ADDR[1:0]}<<5);
                        end
                        2'b01: begin
                            h1_di_i[g]   = SPI_TRACE_WR_EN ? ({96'd0, SPI_DATA, 96'd0} << ({6'd0, SPI_ADDR[1:0]}<<5)) : '0;
                            h1_mask_i[g] = SPI_TRACE_WR_EN ? ({96'd0, 32'hFFFF_FFFF, 96'd0} << ({6'd0, SPI_ADDR[1:0]}<<5)) : '0;
                        end
                        default: begin
                            h1_di_i[g]   = SPI_TRACE_WR_EN ? ({SPI_DATA, 192'd0} << ({6'd0, SPI_ADDR[1:0]}<<5)) : '0;
                            h1_mask_i[g] = SPI_TRACE_WR_EN ? ({32'hFFFF_FFFF, 192'd0} << ({6'd0, SPI_ADDR[1:0]}<<5)) : '0;
                        end
                    endcase
                end
                unique case (SPI_ADDR[10:9])
                    2'b00: SR_READBACK = h1_do[SPI_BANK_SEL_H1][127:0]   >> ({6'd0, SPI_ADDR[1:0]}<<5);
                    2'b01: SR_READBACK = h1_do[SPI_BANK_SEL_H1][223:128] >> ({6'd0, SPI_ADDR[1:0]}<<5);
                    default: SR_READBACK = h1_do[SPI_BANK_SEL_H1][319:224] >> ({6'd0, SPI_ADDR[1:0]}<<5);
                endcase
            end else if (PROG_NEUR_H2 || READ_NEUR_H2) begin
                for (int g=0; g<NG; g++) begin
                    h2_cs_i[g]   = (g==SPI_BANK_SEL_H2);
                    h2_we_i[g]   = PROG_NEUR_H2;
                    h2_addr_i[g] = SPI_ADDR[8:2];
                    unique case (SPI_ADDR[10:9])
                        2'b00: begin
                            h2_di_i[g]   = {192'd0, SPI_DATA} << ({6'd0, SPI_ADDR[1:0]}<<5);
                            h2_mask_i[g] = {192'd0, 32'hFFFF_FFFF} << ({6'd0, SPI_ADDR[1:0]}<<5);
                        end
                        2'b01: begin
                            h2_di_i[g]   = SPI_TRACE_WR_EN ? ({96'd0, SPI_DATA, 96'd0} << ({6'd0, SPI_ADDR[1:0]}<<5)) : '0;
                            h2_mask_i[g] = SPI_TRACE_WR_EN ? ({96'd0, 32'hFFFF_FFFF, 96'd0} << ({6'd0, SPI_ADDR[1:0]}<<5)) : '0;
                        end
                        default: begin
                            h2_di_i[g]   = SPI_TRACE_WR_EN ? ({SPI_DATA, 192'd0} << ({6'd0, SPI_ADDR[1:0]}<<5)) : '0;
                            h2_mask_i[g] = SPI_TRACE_WR_EN ? ({32'hFFFF_FFFF, 192'd0} << ({6'd0, SPI_ADDR[1:0]}<<5)) : '0;
                        end
                    endcase
                end
                unique case (SPI_ADDR[10:9])
                    2'b00: SR_READBACK = h2_do[SPI_BANK_SEL_H2][127:0]   >> ({6'd0, SPI_ADDR[1:0]}<<5);
                    2'b01: SR_READBACK = h2_do[SPI_BANK_SEL_H2][223:128] >> ({6'd0, SPI_ADDR[1:0]}<<5);
                    default: SR_READBACK = h2_do[SPI_BANK_SEL_H2][319:224] >> ({6'd0, SPI_ADDR[1:0]}<<5);
                endcase
            end else if (PROG_WOUT || READ_WOUT) begin
                wout_cs_i   = 1'b1;
                wout_we_i   = PROG_WOUT;
                wout_addr_i = SPI_ADDR[10:2];
                wout_di_i   = {96'd0, SPI_DATA} << ({5'd0, SPI_ADDR[1:0]}<<5);
                wout_mask_i = {96'd0, 32'hFFFF_FFFF} << ({5'd0, SPI_ADDR[1:0]}<<5);
                SR_READBACK = wout_do >> ({5'd0, SPI_ADDR[1:0]}<<5);
            end else if (SPI_CODE==3'b101) begin
                // Output bias programming/readback: SPI_ADDR[3:0] selects output index (0..N_OUT-1)
                automatic int k = SPI_ADDR[3:0];
                if (~SPI_READ) out_bias8[k] <= SPI_DATA[7:0];
                SR_READBACK = {24'd0, {16{out_bias8[k][7]}}, out_bias8[k]};
            end else if (SPI_CODE==3'b110) begin
                // Output error readback: SPI_ADDR[3:0] selects output index
                automatic int k = SPI_ADDR[3:0];
                SR_READBACK = out_err16[k][15] ? {16'hFFFF, out_err16[k]} : {16'h0000, out_err16[k]};
            end else begin
                SR_READBACK = 32'd0;
            end
        end else begin
            SR_READBACK = 32'd0;
        end
    end

    // Sparse memories
    SRAM_4096x136_wrapper sp_vi_h1 (
        .CK (clk),
        .CS (sp_vi_h1_cs),
        .WE (sp_vi_h1_we),
        .A  (sp_vi_h1_addr),
        .D  (sp_vi_h1_di),
        .Q  (sp_vi_h1_do)
    );

    SRAM_4096x136_wrapper sp_h1_h2 (
        .CK (clk),
        .CS (sp_h1_h2_cs),
        .WE (sp_h1_h2_we),
        .A  (sp_h1_h2_addr),
        .D  (sp_h1_h2_di),
        .Q  (sp_h1_h2_do)
    );

    // Output weights
    SRAM_512x128_wrapper weights_out (
        .CK (clk),
        .SM (2'b00),
        .CS (wout_cs_i),
        .WE (wout_we_i),
        .A  (wout_addr_i),
        .D  (wout_di_i),
        .M  (wout_mask_i),
        .Q  (wout_do)
    );

    // ------------------------------------------------------------------------
    // FIFOs for spikes (simple, tiny depth)
    // ------------------------------------------------------------------------
    fifo_simpl #(.WIDTH(9), .DEPTH(8)) fifo_h1 (
        .clk (clk), .rst (rst),
        .push (h1_spike_push), .din (h1_spike_in),
        .pop  (h1_spike_pop),  .dout(h1_spike_out),
        .empty(h1_spike_empty)
    );

    fifo_simpl #(.WIDTH(9), .DEPTH(8)) fifo_h2 (
        .clk (clk), .rst (rst),
        .push (h2_spike_push), .din (h2_spike_in),
        .pop  (h2_spike_pop),  .dout(h2_spike_out),
        .empty(h2_spike_empty)
    );

    // AER input FIFO (for robustness/backpressure)
    fifo_simpl #(.WIDTH(M_INP), .DEPTH(16)) fifo_aer (
        .clk (clk), .rst (rst),
        .push (aer_fifo_push), .din (aer_fifo_din),
        .pop  (aer_fifo_pop),  .dout(aer_fifo_dout),
        .empty(aer_fifo_empty), .level(aer_fifo_level)
    );

endmodule


// -----------------------------------------------------------------------------
// Behavioral memory wrappers
// -----------------------------------------------------------------------------

// wrapper moved to elfcore_utils.sv

// SPI module moved to elfcore_utils.sv
