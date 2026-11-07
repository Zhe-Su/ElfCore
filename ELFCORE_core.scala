package elfcore

import chisel3._
import chisel3.util._

// Minimal Chisel version of ELFCORE_core focusing on IO parity and basic handshakes
class ELFCORECoreChisel(
  val N_INP: Int = 256,
  val N_H1:  Int = 512,
  val N_H2:  Int = 512,
  val N_OUT: Int = 16
) extends Module {
  private val M_INP = Math.max(1, log2Ceil(N_INP))
  private val M_OUT = Math.max(1, log2Ceil(N_OUT))

  val io = IO(new Bundle {
    // SPI slave interface
    val SCK   = Input(Bool())
    val MOSI  = Input(Bool())
    val MISO  = Output(Bool())

    // AER virtual input
    val AERIN_ADDR = Input(UInt(M_INP.W))
    val AERIN_REQ  = Input(Bool())
    val AERIN_ACK  = Output(Bool())

    // Timestep control
    val SAMPLE     = Input(Bool())
    val TIME_TICK  = Input(Bool())
    val INFER_ACC  = Input(Bool())

    // Output handshake
    val OUT_REQ    = Output(Bool())
    val OUT_ACK    = Input(Bool())
    val OUT_DATA   = Output(UInt(8.W))

    // Supervised label input
    val LABEL_VALID= Input(Bool())
    val LABEL_INDEX= Input(UInt(M_OUT.W))

    // Diagnostics
    val SPI_RDY          = Output(Bool())
    val TIMING_ERROR_RDY = Output(Bool())
  })

  // ------------------------------------------------------------------
  // Minimal SPI config + write bank (32-bit write-only on SCK rising edges)
  // Format (bit positions):
  //   [31:28] code: 0000=cfg, 0001=neur, 0011=winp, 0100=wrec, 0101=wout
  //   [27:16] numWrites (for multi-write, optional)
  //   [15:0]  address
  //   If code==cfg: bits [2:0] as control flags: EN_CONF, AER_DB, LABEL_SMP
  //   Data phase: 32-bit payload for memory writes
  // ------------------------------------------------------------------
  // SPI output shift for reads
  val spi_read_mode  = RegInit(false.B)
  val spi_read_shift = RegInit(0.U(32.W))
  val spi_read_cnt   = RegInit(0.U(6.W))
  val miso_wire      = WireDefault(false.B)
  when (spi_read_mode) { miso_wire := spi_read_shift(31) }
  io.MISO := miso_wire
  val sck_q         = RegNext(io.SCK, init=false.B)
  val sck_rise      = !sck_q && io.SCK
  val spi_cnt       = RegInit(0.U(6.W)) // up to 32 for one word
  val spi_shift_in  = RegInit(0.U(32.W))
  val spi_header    = RegInit(0.U(32.W))
  val spi_en_conf   = RegInit(false.B)
  val cfg_aer_db    = RegInit(true.B)
  val cfg_label_smp = RegInit(true.B)
  val cfg_send_label_only = RegInit(true.B)
  val cfg_num_out_neur    = RegInit(1.U(4.W))
  val cfg_num_inp_neur    = RegInit(39.U(8.W))
  val cfg_num_rec_neur    = RegInit(99.U(8.W))
  val cfg_send_per_timestep = RegInit(false.B)
  val cfg_timing_mode     = RegInit(false.B)
  val cfg_regression      = RegInit(false.B)
  val cfg_single_label    = RegInit(true.B)
  val cfg_no_out_act      = RegInit(false.B)
  val cfg_stream_act      = RegInit(false.B)
  val cfg_out_ste_en      = RegInit(false.B)
  val cfg_pipelined       = RegInit(false.B)
  val cfg_sup_out_en      = RegInit(true.B) // supervised output learner enable
  // VI base offset and T_prev depth
  val cfg_vi_base         = RegInit(0.U(M_INP.W))
  val cfg_tprev_depth     = RegInit(0.U(3.W))
  // Output learner knobs (subset)
  val cfg_out_err_scale   = RegInit(0.U(4.W))  // right shift on error
  val cfg_out_err_max     = RegInit(32767.U(16.W)) // clamp error magnitude
  val cfg_out_pre_scale   = RegInit(255.U(8.W)) // Q0.8 presyn factor
  val cfg_out_wdec        = RegInit(0.U(8.W))   // Q0.8 weight decay
  val cfg_clip_wout       = RegInit(255.U(8.W)) // clamp per-step delta magnitude for wout
  val cfg_clip_hidden     = RegInit(255.U(8.W)) // clamp per-step delta magnitude for hidden
  // Global input activity gating threshold (like SPI_THR_I in SV)
  val cfg_thr_i           = RegInit(0.U(16.W))
  val cfg_en_stoch_round  = RegInit(false.B)
  val cfg_noise_en        = RegInit(false.B)
  val cfg_kappa           = RegInit("h79".U(8.W))
  val cfg_alpha           = RegInit("h8000".U(16.W))
  val cfg_thr_out         = RegInit(0.U(16.W))
  val cfg_out_mask        = RegInit("hFFFF".U(16.W))
  val cfg_out_bias8       = RegInit(0.S(8.W))   // signed 8-bit bias, applied as <<8 in activation path
  // Optional per-output biases (override global if programmed)
  val cfg_out_bias8_vec   = RegInit(VecInit(Seq.fill(16)(0.S(8.W))))
  val cfg_out_learn_use_act = RegInit(false.B)
  val cfg_out_stoch_en      = RegInit(false.B)
  val cfg_sv_act_map        = RegInit(false.B)
  // When true, use output integrator (oneur_u) for winner/SEND paths instead of outTrace
  // SPI mapping:
  //   - Write: subpage 39 (addr16[13]=1, [12:8]=39) bit[0]
  //   - Read:  subpage 1  (addr16[13]=1, [12:8]=1)  bit[5]
  val cfg_use_oneur         = RegInit(false.B)
  // Regularization knobs (subset parity with srnn)
  val cfg_regul_mode        = RegInit(0.U(3.W))  // {modes}
  val cfg_regul_w           = RegInit(0.U(2.W))  // selects input/recurrent reg application
  val cfg_regul_f0          = RegInit(0.U(12.W))
  val cfg_regul_k_inp_r     = RegInit(0.U(5.W))
  val cfg_regul_k_inp_p     = RegInit(0.U(5.W))
  val cfg_regul_k_rec_r     = RegInit(0.U(5.W))
  val cfg_regul_k_rec_p     = RegInit(0.U(5.W))
  val cfg_regul_k_mul       = RegInit(0.U(5.W))
  // Fixed-point locations for traces/weights
  val cfg_fp_loc_tinp       = RegInit(0.U(3.W))
  val cfg_fp_loc_trec       = RegInit(0.U(3.W))
  val cfg_fp_loc_tout       = RegInit(0.U(3.W))
  val cfg_fp_loc_winp       = RegInit(0.U(3.W))
  val cfg_fp_loc_wrec       = RegInit(0.U(3.W))
  val cfg_fp_loc_wout       = RegInit(0.U(3.W))
  // Timing controls: local tick and timing error policy
  val cfg_local_tick      = RegInit(false.B)
  val cfg_cycles_per_tick = RegInit(1000.U(32.W))
  val cfg_error_halt      = RegInit(false.B)
  // Seeds for stochastic rounding and noise
  val cfg_seed_strnd      = RegInit("h1234".U(15.W))
  val cfg_seed_noise      = RegInit("h1F0F1".U(17.W))
  // DST scaffolding
  val cfg_dyn_en_h1       = RegInit(false.B)
  val cfg_dyn_en_h2       = RegInit(false.B)
  val cfg_dyn_x_h1        = RegInit(0.U(16.W))
  val cfg_dyn_x_h2        = RegInit(0.U(16.W))
  val cfg_prune_k_h1      = RegInit(8.U(8.W))
  val cfg_prune_k_h2      = RegInit(8.U(8.W))
  // Effective per-group regrow heap active size = ceil(K/4); enforce when pushing scores/popping
  def effHeapPerGroup(k: UInt): UInt = {
    // ceil(k/4): (k+3)>>2
    ((k + 3.U(8.W)) >> 2).asUInt()
  }
  val cfg_collision_en    = RegInit(true.B)
  val cfg_seed_dst        = RegInit("h35A5".U(17.W))
  // REGROW: per-group top-K heaps scaffolding (score width 24b, index 9b)
  // Total regrow capacity per layer is matched to pruning K via per-group active entries.
  private val RG_HEAP_K = 16 // max entries per group (matches prune-heap max per layer)
  val rg_k_h1             = RG_HEAP_K.U(8.W)
  val rg_k_h2             = RG_HEAP_K.U(8.W)
  val rg_heap_score1      = RegInit(VecInit(Seq.fill(4)(VecInit(Seq.fill(RG_HEAP_K)(0.U(24.W))))))
  val rg_heap_index1      = RegInit(VecInit(Seq.fill(4)(VecInit(Seq.fill(RG_HEAP_K)(0.U(9.W))))))
  val rg_heap_score2      = RegInit(VecInit(Seq.fill(4)(VecInit(Seq.fill(RG_HEAP_K)(0.U(24.W))))))
  val rg_heap_index2      = RegInit(VecInit(Seq.fill(4)(VecInit(Seq.fill(RG_HEAP_K)(0.U(9.W))))))
  val rg_scan_idx1        = RegInit(0.U(9.W))
  val rg_scan_idx2        = RegInit(0.U(9.W))
  val rg_fill_done1       = RegInit(false.B)
  val rg_fill_done2       = RegInit(false.B)
  val cnt_steps_dst_h1    = RegInit(0.U(16.W))
  val cnt_steps_dst_h2    = RegInit(0.U(16.W))
  val cfg_dst_scan_h1     = RegInit(64.U(12.W))
  val cfg_dst_scan_h2     = RegInit(64.U(12.W))
  // DST diagnostics
  val cnt_dst_cycles_h1   = RegInit(0.U(16.W))
  val cnt_dst_cycles_h2   = RegInit(0.U(16.W))
  val cnt_dst_prunes_h1   = RegInit(0.U(16.W))
  val cnt_dst_prunes_h2   = RegInit(0.U(16.W))
  val last_dst_meta_h1    = RegInit(0.U(32.W)) // {grp[1:0], j[9:0], w[7:0], addr[11:0]}
  val last_dst_meta_h2    = RegInit(0.U(32.W))
  // Simple DST RNG (17-bit LFSR)
  val dst_lfsr = RegInit("h1A35B".U(17.W))
  val dst_bit  = dst_lfsr(0) ^ dst_lfsr(3) ^ dst_lfsr(5) ^ dst_lfsr(8)
  val dst_next = Cat(dst_bit, dst_lfsr(16,1))
  when (tick_rise) { dst_lfsr := (dst_next ^ cfg_seed_dst) } .otherwise { dst_lfsr := dst_next }
  // EPROP enables and learning rates (placeholder formats)
  val cfg_do_eprop        = RegInit(0.U(3.W)) // [0]=winp, [1]=wrec, [2]=wout
  val lr_r_winp           = RegInit(0.U(5.W))
  val lr_p_winp           = RegInit(0.U(5.W))
  val lr_r_wrec           = RegInit(0.U(5.W))
  val lr_p_wrec           = RegInit(0.U(5.W))
  val lr_r_wout           = RegInit(0.U(5.W))
  val lr_p_wout           = RegInit(0.U(5.W))
  // Explicit training stage selector: 0=unsup, 1=sup, 2=auto(derive from OL enable)
  val cfg_stage_mode      = RegInit(2.U(2.W))
  // Training stage gating: supervised stage determined by cfg_stage_mode and cfg_sup_out_en (no e-prop)
  val stage_sup  = Wire(Bool())
  stage_sup := MuxLookup(cfg_stage_mode, cfg_sup_out_en, Seq(
    0.U -> false.B,
    1.U -> true.B,
    2.U -> cfg_sup_out_en
  ))
  val stage_unsup = !stage_sup
  // Sparse words-per-presyn configuration (default 4)
  val cfg_words_per_pre_h1= RegInit(4.U(4.W))
  val cfg_words_per_pre_h2= RegInit(4.U(4.W))
  // STE LUT for output activation/learning (placeholder usage)
  val ste_lut = RegInit(VecInit(Seq.fill(256)(0.U(8.W))))
  // Similarity/gating knobs
  val cfg_sim_scale       = RegInit(1.U(16.W))  // Q16.0
  val cfg_sim_shift       = RegInit(0.U(5.W))   // >> after multiply
  val cfg_i_scale         = RegInit(1.U(16.W))  // Q16.0 i normalization
  val cfg_i_shift         = RegInit(0.U(5.W))
  val cfg_h1_pred_gate    = RegInit(0.U(16.W))  // threshold multiplier for pred
  val cfg_h1_contr_gate   = RegInit(0.U(16.W))  // threshold for contr
  val cfg_h2_pred_gate    = RegInit(0.U(16.W))
  val cfg_h2_contr_gate   = RegInit(0.U(16.W))
  val cfg_sim_norm_mode   = RegInit(0.U(2.W)) // 0=none, 1=by spike count, 2=by neuron count
  val cfg_h1_grad_max     = RegInit(32767.U(16.W))
  val cfg_h2_grad_max     = RegInit(32767.U(16.W))
  // Optional SRAM speed and noise strength knobs (not functionally used here)
  val cfg_sram_speed_h1   = RegInit(VecInit(Seq.fill(4)(0.U(8.W))))
  val cfg_sram_speed_h2   = RegInit(VecInit(Seq.fill(4)(0.U(8.W))))
  val cfg_noise_str       = RegInit(0.U(4.W))
  // Refractory/Homeostasis knobs (present in SV; placeholders here)
  val cfg_ref_add_h1      = RegInit(0.U(8.W))
  val cfg_ref_dec_h1      = RegInit(0.U(8.W))
  val cfg_ref_add_h2      = RegInit(0.U(8.W))
  val cfg_ref_dec_h2      = RegInit(0.U(8.W))
  def hardSigmoid8(raw16: UInt): UInt = {
    val u = raw16.asSInt
    val shifted = (u + 32768.S(16.W)).asUInt // map -32768..32767 -> 0..65535
    (shifted >> 8).asUInt()(7,0)             // 0..255
  }
  // Saturation helpers
  def clampS16(x: SInt): SInt = {
    Mux(x > 32767.S, 32767.S, Mux(x < (-32768).S, (-32768).S, x))
  }
  def satAdd32(a: UInt, b: UInt): UInt = {
    val sum = a.zext() +& b.zext() // 33-bit
    Mux(sum(32), "hFFFF_FFFF".U(32.W), sum(31,0))
  }
  def svActY(u: SInt): UInt = {
    val y = Wire(UInt(16.W))
    when (u > 2047.S)      { y := 2048.U }
    .elsewhen (u < -2048.S){ y := 0.U }
    .otherwise             { y := ((u + 2048.S(16.W)) >> 1).asUInt }
    y
  }
  // Minimal stochastic update for 8-bit weights (probabilistic ±1 with LR_r/LR_p)
  // probMag: magnitude 0..255, dirPos: true => +1, false => -1
  // rnd: 8-bit random threshold
  def stochUpdateByte(probMag: UInt, dirPos: Bool, wIn: UInt, lr_r: UInt, lr_p: UInt, rnd: UInt): (UInt, UInt) = {
    require(wIn.getWidth == 8)
    val prob0    = probMag === 0.U
    val probLrp  = (probMag.zext() << lr_p).asUInt
    val probTot  = (probLrp >> lr_r)
    val update   = !prob0 && (probTot(7,0) >= rnd)
    val incVal   = Mux(dirPos, 1.U(8.W), (~0.U(8.W))) // +1 or -1 (two's comp)
    val wUpd     = (wIn + incVal)(7,0)
    val overflow = (dirPos && (wIn === 255.U)) || (!dirPos && (wIn === 0.U))
    val noChange = !update || overflow
    val wOut     = Mux(noChange, wIn, wUpd)
    val mask     = Mux(noChange, 0.U(8.W), "hFF".U(8.W))
    (wOut, mask)
  }
  val spi_in_data   = Wire(Bool())
  spi_in_data := io.MOSI
  when (sck_rise) {
    spi_shift_in := Cat(spi_shift_in(30,0), spi_in_data.asUInt)
    spi_cnt      := spi_cnt + 1.U
    when (spi_cnt === 31.U) {
      // First word captured is header (code, numWrites, address)
      when (spi_en_conf) { spi_header := spi_shift_in } .otherwise { spi_header := 0.U }
      spi_cnt := 0.U
    }
  }
  io.SPI_RDY := spi_en_conf

  // Header decode and modes
  val word_done  = (spi_cnt === 31.U) && sck_rise
  val code_full  = spi_header(31,28)
  val read_flag  = code_full(3)
  val code       = code_full(2,0)
  val addr16    = spi_header(15,0)
  val data_word = spi_shift_in
  val spi_expect_data = RegInit(false.B)
  // SPI burst tracking for auto-increment writes
  val spi_burst_active = RegInit(false.B)
  val spi_burst_code   = RegInit(0.U(3.W))
  val spi_burst_addr   = RegInit(0.U(16.W))
  val spi_burst_rem    = RegInit(0.U(12.W))
  val spi_burst_wsel   = RegInit(0.U(2.W))

  // Launch read mode on header
  when (word_done && !spi_expect_data && spi_en_conf && read_flag) {
    val read_data = WireDefault(0.U(32.W))
    switch (code) {
      is ("b000".U) { // cfg readback: pack flags and sizes
        // Base flags [7:0]: [7]=NOISE_EN [6]=EN_STOCH_ROUND [5]=NO_OUT_ACT [4]=SEND_PER_TIMESTEP [3]=SEND_LABEL_ONLY [2]=LABEL_SMP [1]=AER_DB [0]=EN_CONF
        val flags = Cat(cfg_noise_en.asUInt, cfg_en_stoch_round.asUInt, cfg_no_out_act.asUInt, cfg_send_per_timestep.asUInt, cfg_send_label_only.asUInt, cfg_label_smp.asUInt, cfg_aer_db.asUInt, spi_en_conf.asUInt)
        val base = Cat(cfg_kappa.asUInt, cfg_num_out_neur.asUInt, cfg_num_rec_neur.asUInt, cfg_num_inp_neur.asUInt, flags)
        when (addr16(13)) {
          // Extended subpage readback
          switch (addr16(14,8)) {
            is (0.U)  { read_data := base }
            is (1.U)  { read_data := Cat(0.U(19.W), cfg_fp_loc_wout, cfg_fp_loc_wrec, cfg_fp_loc_winp, cfg_fp_loc_tout, cfg_fp_loc_trec, cfg_fp_loc_tinp,
                                         cfg_regul_k_mul, cfg_regul_k_rec_p, cfg_regul_k_rec_r, cfg_regul_k_inp_p, cfg_regul_k_inp_r,
                                         cfg_regul_f0(3,0),
                                         cfg_use_oneur, cfg_sup_out_en, cfg_pipelined, cfg_stream_act, cfg_out_ste_en, cfg_regression, cfg_timing_mode) }
          is (40.U) { read_data := Cat(0.U(31.W), cfg_sv_act_map) }
            is (2.U)  { read_data := Cat(0.U(28.W), cfg_words_per_pre_h1(3,0)) }
            is (3.U)  { read_data := Cat(0.U(28.W), cfg_words_per_pre_h2(3,0)) }
            is (4.U)  { read_data := Cat(0.U(16.W), cfg_out_err_max) }
            is (5.U)  { read_data := Cat(0.U(24.W), cfg_out_err_scale) }
            is (6.U)  { read_data := Cat(0.U(24.W), cfg_out_pre_scale) }
            is (7.U)  { read_data := Cat(0.U(24.W), cfg_out_wdec) }
            is (8.U)  { read_data := Cat(0.U(16.W), cfg_sim_scale) }
            is (9.U)  { read_data := Cat(0.U(27.W), cfg_sim_shift) }
            is (10.U) { read_data := Cat(0.U(16.W), cfg_i_scale) }
            is (11.U) { read_data := Cat(0.U(27.W), cfg_i_shift) }
            is (12.U) { read_data := Cat(0.U(16.W), cfg_h1_pred_gate) }
            is (13.U) { read_data := Cat(0.U(16.W), cfg_h1_contr_gate) }
            is (14.U) { read_data := Cat(0.U(16.W), cfg_h2_pred_gate) }
            is (15.U) { read_data := Cat(0.U(16.W), cfg_h2_contr_gate) }
            is (16.U) { read_data := Cat(0.U(30.W), cfg_sim_norm_mode) }
            // DST readback remapped to 90..101
            is (90.U) { read_data := Cat(0.U(31.W), cfg_dyn_en_h1) }
            is (91.U) { read_data := Cat(0.U(31.W), cfg_dyn_en_h2) }
            is (92.U) { read_data := Cat(0.U(16.W), cfg_dyn_x_h1) }
            is (93.U) { read_data := Cat(0.U(16.W), cfg_dyn_x_h2) }
            is (94.U) { read_data := Cat(0.U(24.W), cfg_prune_k_h1) }
            is (95.U) { read_data := Cat(0.U(24.W), cfg_prune_k_h2) }
            is (96.U) { read_data := Cat(0.U(15.W), cfg_seed_dst) }
            is (97.U) { read_data := Cat(0.U(16.W), cnt_steps_dst_h1) }
            is (98.U) { read_data := Cat(0.U(16.W), cnt_steps_dst_h2) }
            is (99.U) { read_data := Cat(0.U(17.W), cfg_seed_noise) }
            is (100.U){ read_data := Cat(0.U(17.W), cfg_seed_strnd) }
            is (27.U) { read_data := Cat(0.U(16.W), cfg_vi_base) }
            is (28.U) { read_data := Cat(0.U(29.W), cfg_tprev_depth) }
            is (29.U) { read_data := Cat(0.U(24.W), cfg_out_bias8.asUInt) }
            is (30.U) { read_data := Cat(0.U(24.W), cfg_clip_wout) }
            is (31.U) { read_data := Cat(0.U(24.W), cfg_clip_hidden) }
            is (32.U) { read_data := Cat(0.U(20.W), cfg_dst_scan_h1) }
            is (33.U) { read_data := Cat(0.U(20.W), cfg_dst_scan_h2) }
            // Stage mode readback (remapped to 104)
            is (104.U) { read_data := Cat(0.U(30.W), cfg_stage_mode) }
            is (41.U) { read_data := cfg_cycles_per_tick }
            is (42.U) { read_data := Cat(0.U(30.W), cfg_error_halt, cfg_local_tick) }
            is (43.U) { read_data := Cat(0.U(28.W), cfg_noise_str) }
            is (44.U) { read_data := Cat(0.U(24.W), cfg_sram_speed_h1(0)) }
            is (45.U) { read_data := Cat(0.U(24.W), cfg_sram_speed_h1(1)) }
            is (46.U) { read_data := Cat(0.U(24.W), cfg_sram_speed_h1(2)) }
            is (47.U) { read_data := Cat(0.U(24.W), cfg_sram_speed_h1(3)) }
            is (48.U) { read_data := Cat(0.U(24.W), cfg_sram_speed_h2(0)) }
            is (49.U) { read_data := Cat(0.U(24.W), cfg_sram_speed_h2(1)) }
            is (50.U) { read_data := Cat(0.U(24.W), cfg_sram_speed_h2(2)) }
            is (51.U) { read_data := Cat(0.U(24.W), cfg_sram_speed_h2(3)) }
            is (68.U) { read_data := Cat(0.U(24.W), cfg_ref_add_h1) }
            is (69.U) { read_data := Cat(0.U(24.W), cfg_ref_dec_h1) }
            is (70.U) { read_data := Cat(0.U(24.W), cfg_ref_add_h2) }
            is (71.U) { read_data := Cat(0.U(24.W), cfg_ref_dec_h2) }
            // Per-output bias readback 52..67 map to out[0..15]
            is (52.U) { read_data := Cat(0.U(24.W), cfg_out_bias8_vec(0).asUInt) }
            is (53.U) { read_data := Cat(0.U(24.W), cfg_out_bias8_vec(1).asUInt) }
            is (54.U) { read_data := Cat(0.U(24.W), cfg_out_bias8_vec(2).asUInt) }
            is (55.U) { read_data := Cat(0.U(24.W), cfg_out_bias8_vec(3).asUInt) }
            is (56.U) { read_data := Cat(0.U(24.W), cfg_out_bias8_vec(4).asUInt) }
            is (57.U) { read_data := Cat(0.U(24.W), cfg_out_bias8_vec(5).asUInt) }
            is (58.U) { read_data := Cat(0.U(24.W), cfg_out_bias8_vec(6).asUInt) }
            is (59.U) { read_data := Cat(0.U(24.W), cfg_out_bias8_vec(7).asUInt) }
            is (60.U) { read_data := Cat(0.U(24.W), cfg_out_bias8_vec(8).asUInt) }
            is (61.U) { read_data := Cat(0.U(24.W), cfg_out_bias8_vec(9).asUInt) }
            is (62.U) { read_data := Cat(0.U(24.W), cfg_out_bias8_vec(10).asUInt) }
            is (63.U) { read_data := Cat(0.U(24.W), cfg_out_bias8_vec(11).asUInt) }
            is (64.U) { read_data := Cat(0.U(24.W), cfg_out_bias8_vec(12).asUInt) }
            is (65.U) { read_data := Cat(0.U(24.W), cfg_out_bias8_vec(13).asUInt) }
            is (66.U) { read_data := Cat(0.U(24.W), cfg_out_bias8_vec(14).asUInt) }
            is (67.U) { read_data := Cat(0.U(24.W), cfg_out_bias8_vec(15).asUInt) }
            // Remapped readbacks (output-learning, regularization, FP-locations, thr_i)
            is (108.U) { read_data := Cat(0.U(31.W), cfg_out_learn_use_act) }
            is (109.U) { read_data := Cat(0.U(31.W), cfg_out_stoch_en) }
            is (112.U) { read_data := Cat(0.U(29.W), cfg_regul_mode) }
            is (113.U) { read_data := Cat(0.U(30.W), cfg_regul_w) }
            is (114.U) { read_data := Cat(0.U(20.W), cfg_regul_f0) }
            is (115.U) { read_data := Cat(0.U(27.W), cfg_regul_k_inp_r) }
            is (116.U) { read_data := Cat(0.U(27.W), cfg_regul_k_inp_p) }
            is (117.U) { read_data := Cat(0.U(27.W), cfg_regul_k_rec_r) }
            is (118.U) { read_data := Cat(0.U(27.W), cfg_regul_k_rec_p) }
            is (119.U) { read_data := Cat(0.U(27.W), cfg_regul_k_mul) }
            is (120.U) { read_data := Cat(0.U(29.W), cfg_fp_loc_tinp) }
            is (121.U) { read_data := Cat(0.U(29.W), cfg_fp_loc_trec) }
            is (122.U) { read_data := Cat(0.U(29.W), cfg_fp_loc_tout) }
            is (123.U) { read_data := Cat(0.U(29.W), cfg_fp_loc_winp) }
            is (124.U) { read_data := Cat(0.U(29.W), cfg_fp_loc_wrec) }
            is (125.U) { read_data := Cat(0.U(29.W), cfg_fp_loc_wout) }
            is (127.U) { read_data := Cat(0.U(16.W), cfg_thr_i) }
          }
        } .otherwise {
          read_data := base
        }
      }
      is ("b110".U) { // alpha readback (low 16 bits)
        read_data := Cat(0.U(16.W), cfg_alpha)
      }
      is ("b010".U) { // output threshold readback
        read_data := Cat(0.U(16.W), cfg_thr_out)
      }
      is ("b111".U) { // diagnostics / mask / winner
        when (addr16(15)) {
          // mask
          read_data := Cat(0.U(16.W), cfg_out_mask)
        }.elsewhen (addr16(14)) {
          // winner: [15:0]=winnerVal, [19:16]=winnerIdx
          read_data := Cat(0.U(12.W), winnerIdx_w(3,0), winnerVal_w)
        }.otherwise {
          // Diagnostics subfields when addr[13]=1: counters, FIFO level/HWM
          when (addr16(13)) {
            // Derived diagnostics
            val h1_heap_count = dst1_heap_mags.map(m => (m =/= 255.U).asUInt).reduce(_+&_)
            val h2_heap_count = dst2_heap_mags.map(m => (m =/= 255.U).asUInt).reduce(_+&_)
            switch (addr16(14,8)) {
              is (0.U) { read_data := cnt_ticks } // 32-bit
              is (1.U) { read_data := Cat(0.U(16.W), cnt_spikes_h1) }
              is (2.U) { read_data := Cat(0.U(16.W), cnt_spikes_h2) }
              is (3.U) { read_data := Cat(0.U(24.W), aer_fifo_level) }
              is (4.U) { read_data := Cat(0.U(24.W), aer_fifo_hwm) }
              is (5.U) { read_data := Cat(0.U(30.W), h2_sp_active, vi_sp_active) } // burst-active flags
              is (6.U) { read_data := Cat(0.U(24.W), aer_cnt_tick) } // AER events in current PROP1 tick
              is (7.U) { read_data := sim_pred_h1_l }
              is (8.U) { read_data := sim_contr_h1_l }
              is (9.U) { read_data := sim_pred_h2_l }
              is (10.U){ read_data := sim_contr_h2_l }
              is (11.U){ read_data := Cat(0.U(16.W), h1_spk_cnt_tick) }
              is (12.U){ read_data := Cat(0.U(16.W), h2_spk_cnt_tick) }
              is (13.U){ read_data := Cat(0.U(16.W), cnt_dst_cycles_h1) }
              is (14.U){ read_data := Cat(0.U(16.W), cnt_dst_cycles_h2) }
              is (15.U){ read_data := Cat(0.U(16.W), cnt_dst_prunes_h1) }
              is (16.U){ read_data := Cat(0.U(16.W), cnt_dst_prunes_h2) }
              is (17.U){ read_data := last_dst_meta_h1 }
              is (18.U){ read_data := last_dst_meta_h2 }
              // Regrow heap diagnostics (H1 groups 0..3, entries 0..15)
              is (19.U){ read_data := Cat(0.U(8.W), rg_heap_index1(0)(0), rg_heap_score1(0)(0)) }
              is (20.U){ read_data := Cat(0.U(8.W), rg_heap_index1(0)(1), rg_heap_score1(0)(1)) }
              is (21.U){ read_data := Cat(0.U(8.W), rg_heap_index1(0)(2), rg_heap_score1(0)(2)) }
              is (22.U){ read_data := Cat(0.U(8.W), rg_heap_index1(0)(3), rg_heap_score1(0)(3)) }
              is (23.U){ read_data := Cat(0.U(8.W), rg_heap_index1(1)(0), rg_heap_score1(1)(0)) }
              is (24.U){ read_data := Cat(0.U(8.W), rg_heap_index1(1)(1), rg_heap_score1(1)(1)) }
              is (25.U){ read_data := Cat(0.U(8.W), rg_heap_index1(2)(0), rg_heap_score1(2)(0)) }
              is (26.U){ read_data := Cat(0.U(8.W), rg_heap_index1(3)(0), rg_heap_score1(3)(0)) }
              // H2 regrow heaps subset
              is (27.U){ read_data := Cat(0.U(8.W), rg_heap_index2(0)(0), rg_heap_score2(0)(0)) }
              is (28.U){ read_data := Cat(0.U(8.W), rg_heap_index2(1)(0), rg_heap_score2(1)(0)) }
              is (29.U){ read_data := Cat(0.U(8.W), rg_heap_index2(2)(0), rg_heap_score2(2)(0)) }
              is (30.U){ read_data := Cat(0.U(8.W), rg_heap_index2(3)(0), rg_heap_score2(3)(0)) }
              // Heap counts and dst_busy
              is (51.U){ read_data := Cat(0.U(31.W), dst_busy) }
              is (52.U){ read_data := Cat(0.U(23.W), h1_heap_count(8,0)) }
              is (53.U){ read_data := Cat(0.U(23.W), h2_heap_count(8,0)) }
              is (54.U){ read_data := Cat(0.U(24.W), DST_HEAP_K_MAX.U(8.W)) }
              // Error snapshot by index 60..75
              is (60.U){ read_data := Cat(0.U(16.W), err_snapshot(0)) }
              is (61.U){ read_data := Cat(0.U(16.W), err_snapshot(1)) }
              is (62.U){ read_data := Cat(0.U(16.W), err_snapshot(2)) }
              is (63.U){ read_data := Cat(0.U(16.W), err_snapshot(3)) }
              is (64.U){ read_data := Cat(0.U(16.W), err_snapshot(4)) }
              is (65.U){ read_data := Cat(0.U(16.W), err_snapshot(5)) }
              is (66.U){ read_data := Cat(0.U(16.W), err_snapshot(6)) }
              is (67.U){ read_data := Cat(0.U(16.W), err_snapshot(7)) }
              is (68.U){ read_data := Cat(0.U(16.W), err_snapshot(8)) }
              is (69.U){ read_data := Cat(0.U(16.W), err_snapshot(9)) }
              is (70.U){ read_data := Cat(0.U(16.W), err_snapshot(10)) }
              is (71.U){ read_data := Cat(0.U(16.W), err_snapshot(11)) }
              is (72.U){ read_data := Cat(0.U(16.W), err_snapshot(12)) }
              is (73.U){ read_data := Cat(0.U(16.W), err_snapshot(13)) }
              is (74.U){ read_data := Cat(0.U(16.W), err_snapshot(14)) }
              is (75.U){ read_data := Cat(0.U(16.W), err_snapshot(15)) }
              is (76.U) { read_data := Cat(0.U(20.W), dst1_heap_addrs(0),  dst1_heap_mags(0)) }
              is (77.U) { read_data := Cat(0.U(20.W), dst1_heap_addrs(1),  dst1_heap_mags(1)) }
              is (78.U) { read_data := Cat(0.U(20.W), dst1_heap_addrs(2),  dst1_heap_mags(2)) }
              is (79.U) { read_data := Cat(0.U(20.W), dst1_heap_addrs(3),  dst1_heap_mags(3)) }
              is (80.U) { read_data := Cat(0.U(20.W), dst1_heap_addrs(4),  dst1_heap_mags(4)) }
              is (81.U) { read_data := Cat(0.U(20.W), dst1_heap_addrs(5),  dst1_heap_mags(5)) }
              is (82.U) { read_data := Cat(0.U(20.W), dst1_heap_addrs(6),  dst1_heap_mags(6)) }
              is (83.U) { read_data := Cat(0.U(20.W), dst1_heap_addrs(7),  dst1_heap_mags(7)) }
              is (84.U) { read_data := Cat(0.U(20.W), dst1_heap_addrs(8),  dst1_heap_mags(8)) }
              is (85.U) { read_data := Cat(0.U(20.W), dst1_heap_addrs(9),  dst1_heap_mags(9)) }
              is (86.U) { read_data := Cat(0.U(20.W), dst1_heap_addrs(10), dst1_heap_mags(10)) }
              is (87.U) { read_data := Cat(0.U(20.W), dst1_heap_addrs(11), dst1_heap_mags(11)) }
              is (88.U) { read_data := Cat(0.U(20.W), dst1_heap_addrs(12), dst1_heap_mags(12)) }
              is (89.U) { read_data := Cat(0.U(20.W), dst1_heap_addrs(13), dst1_heap_mags(13)) }
              is (90.U) { read_data := Cat(0.U(20.W), dst1_heap_addrs(14), dst1_heap_mags(14)) }
              is (91.U) { read_data := Cat(0.U(20.W), dst1_heap_addrs(15), dst1_heap_mags(15)) }
              is (92.U) { read_data := Cat(0.U(20.W), dst2_heap_addrs(0),  dst2_heap_mags(0)) }
              is (93.U) { read_data := Cat(0.U(20.W), dst2_heap_addrs(1),  dst2_heap_mags(1)) }
              is (94.U) { read_data := Cat(0.U(20.W), dst2_heap_addrs(2),  dst2_heap_mags(2)) }
              is (95.U) { read_data := Cat(0.U(20.W), dst2_heap_addrs(3),  dst2_heap_mags(3)) }
              is (96.U) { read_data := Cat(0.U(20.W), dst2_heap_addrs(4),  dst2_heap_mags(4)) }
              is (97.U) { read_data := Cat(0.U(20.W), dst2_heap_addrs(5),  dst2_heap_mags(5)) }
              is (98.U) { read_data := Cat(0.U(20.W), dst2_heap_addrs(6),  dst2_heap_mags(6)) }
              is (99.U) { read_data := Cat(0.U(20.W), dst2_heap_addrs(7),  dst2_heap_mags(7)) }
              is (100.U){ read_data := Cat(0.U(20.W), dst2_heap_addrs(8),  dst2_heap_mags(8)) }
              is (101.U){ read_data := Cat(0.U(20.W), dst2_heap_addrs(9),  dst2_heap_mags(9)) }
              is (102.U){ read_data := Cat(0.U(20.W), dst2_heap_addrs(10), dst2_heap_mags(10)) }
              is (103.U){ read_data := Cat(0.U(20.W), dst2_heap_addrs(11), dst2_heap_mags(11)) }
              is (104.U){ read_data := Cat(0.U(20.W), dst2_heap_addrs(12), dst2_heap_mags(12)) }
              is (105.U){ read_data := Cat(0.U(20.W), dst2_heap_addrs(13), dst2_heap_mags(13)) }
              is (106.U){ read_data := Cat(0.U(20.W), dst2_heap_addrs(14), dst2_heap_mags(14)) }
              is (107.U){ read_data := Cat(0.U(20.W), dst2_heap_addrs(15), dst2_heap_mags(15)) }
            }
          }.otherwise {
            val selCnt = addr16(4) // 0=trace, 1=count
            val idx    = addr16(3,0)
            val val16  = Mux(selCnt, outCnt(idx), outTrace(idx))
            read_data  := Cat(0.U(16.W), val16)
          }
        }
      }
      is ("b001".U) { // neur
        val a = addr16(8,2)
        val wsel = addr16(1,0)
        spi_neur_cs := true.B; spi_neur_we := false.B; spi_neur_a := a
        val row = neurons.io.q
        read_data := (row >> (wsel<<5)).asUInt()(31,0)
      }
      is ("b011".U) { // winp
        val a = addr16(13,2)
        val wsel = addr16(1,0)
        spi_winp_cs := true.B; spi_winp_we := false.B; spi_winp_a := a
        val row = weightsIn.io.q
        read_data := (row >> (wsel<<5)).asUInt()(31,0)
      }
      is ("b100".U) { // wrec
        val a = addr16(13,2)
        val wsel = addr16(1,0)
        spi_wrec_cs := true.B; spi_wrec_we := false.B; spi_wrec_a := a
        val row = weightsRec.io.q
        read_data := (row >> (wsel<<5)).asUInt()(31,0)
      }
      is ("b101".U) { // wout
        val a = addr16(10,2)
        val wsel = addr16(1,0)
        spi_wout_cs := true.B; spi_wout_we := false.B; spi_wout_a := a
        val row = weightsOut.io.q
        read_data := (row >> (wsel<<5)).asUInt()(31,0)
      }
      is ("b010".U) { // h1 banks: addr[8:7]=bank, [6:0]=row
        val bank = addr16(8,7)
        val a    = addr16(6,0)
        val wsel = addr16(1,0)
        for (b <- 0 until 4) {
          when (bank === b.U) {
            h1(b).io.cs := true.B; h1(b).io.we := false.B; h1(b).io.a := a
            val row = h1(b).io.q
            read_data := (row >> (wsel<<5)).asUInt()(31,0)
          }
        }
      }
      is ("b110".U) { // h2 banks: addr[8:7]=bank, [6:0]=row
        val bank = addr16(8,7)
        val a    = addr16(6,0)
        val wsel = addr16(1,0)
        for (b <- 0 until 4) {
          when (bank === b.U) {
            h2(b).io.cs := true.B; h2(b).io.we := false.B; h2(b).io.a := a
            val row = h2(b).io.q
            read_data := (row >> (wsel<<5)).asUInt()(31,0)
          }
        }
      }
    }
    spi_read_shift := read_data
    spi_read_cnt   := 0.U
    spi_read_mode  := true.B
  }

  // Shift out read data on SCK rising edges
  when (sck_rise && spi_read_mode) {
    spi_read_shift := Cat(spi_read_shift(30,0), 0.U(1.W))
    spi_read_cnt   := spi_read_cnt + 1.U
    when (spi_read_cnt === 31.U) { spi_read_mode := false.B }
  }

  // Second 32-bit word after header is treated as data for memory writes or cfg update
  when (word_done && !spi_expect_data && spi_en_conf && !read_flag) {
    spi_expect_data := true.B
    spi_burst_active := true.B
    spi_burst_code   := code
    spi_burst_addr   := addr16
    spi_burst_rem    := spi_header(27,16)
    spi_burst_wsel   := addr16(1,0)
  }
  when (word_done && spi_expect_data && spi_en_conf) {
    val cur_code = Mux(spi_burst_active, spi_burst_code, code)
    val cur_addr = Mux(spi_burst_active, spi_burst_addr, addr16)
    val cur_wsel = Mux(spi_burst_active, spi_burst_wsel, addr16(1,0))
    switch (cur_code) {
      is ("b000".U) { // cfg
        spi_en_conf          := data_word(0)
        cfg_aer_db           := data_word(1)
        cfg_label_smp        := data_word(2)
        cfg_send_label_only  := data_word(3)
        cfg_send_per_timestep:= data_word(4)
        cfg_num_out_neur     := data_word(7,4)
        cfg_num_inp_neur     := data_word(15,8)
        cfg_num_rec_neur     := data_word(23,16)
        cfg_timing_mode      := data_word(24)
        cfg_regression       := data_word(25)
        cfg_single_label     := data_word(26)
        cfg_no_out_act       := data_word(27)
        cfg_en_stoch_round   := data_word(28)
        cfg_noise_en         := data_word(29)
        cfg_kappa            := data_word(31,24)
        // Extended cfg subpage via addr16(13)
        when (addr16(13)) {
          switch (addr16(14,8)) {
            // E-prop and learning rate knobs (0..12)
            is (0.U)  { cfg_do_eprop := data_word(2,0) }
            is (1.U)  { lr_r_winp := data_word(4,0); lr_p_winp := data_word(9,5) }
            is (2.U)  { lr_r_wrec := data_word(4,0); lr_p_wrec := data_word(9,5) }
            is (3.U)  { lr_r_wout := data_word(4,0); lr_p_wout := data_word(9,5) }
            is (4.U)  { cfg_words_per_pre_h1 := data_word(3,0) }
            is (5.U)  { cfg_words_per_pre_h2 := data_word(3,0) }
            is (6.U)  { ste_lut(data_word(15,8)) := data_word(7,0) }
            is (7.U)  { cfg_stream_act := data_word(0) }
            is (8.U)  { cfg_out_err_scale := data_word(3,0) }
            is (9.U)  { cfg_out_err_max   := data_word(15,0) }
            is (10.U) { cfg_out_pre_scale := data_word(7,0) }
            is (11.U) { cfg_out_wdec      := data_word(7,0) }
            is (12.U) { cfg_out_ste_en    := data_word(0) }
            // Similarity, input gating, normalization (13..21)
            is (13.U) { cfg_sim_scale     := data_word(15,0) }
            is (14.U) { cfg_sim_shift     := data_word(4,0) }
            is (15.U) { cfg_i_scale       := data_word(15,0) }
            is (16.U) { cfg_i_shift       := data_word(4,0) }
            is (17.U) { cfg_h1_pred_gate  := data_word(15,0) }
            is (18.U) { cfg_h1_contr_gate := data_word(15,0) }
            is (19.U) { cfg_h2_pred_gate  := data_word(15,0) }
            is (20.U) { cfg_h2_contr_gate := data_word(15,0) }
            is (21.U) { cfg_sim_norm_mode := data_word(1,0) }
            // Core control knobs (30..40)
            is (30.U) { cfg_pipelined     := data_word(0) }
            is (31.U) { cfg_vi_base       := data_word(M_INP-1,0) }
            is (32.U) { cfg_tprev_depth   := data_word(2,0) }
            is (33.U) { cfg_out_bias8     := data_word(7,0).asSInt }
            is (34.U) { cfg_sup_out_en    := data_word(0) }
            is (35.U) { cfg_clip_wout     := data_word(7,0) }
            is (36.U) { cfg_clip_hidden   := data_word(7,0) }
            is (37.U) { cfg_dst_scan_h1   := data_word(11,0) }
            is (38.U) { cfg_dst_scan_h2   := data_word(11,0) }
            is (39.U) { cfg_use_oneur     := data_word(0) }
            is (40.U) { cfg_sv_act_map    := data_word(0) }
            // Timing and noise (41..43)
            is (41.U) { cfg_cycles_per_tick := data_word }
            is (42.U) { cfg_local_tick := data_word(0); cfg_error_halt := data_word(1) }
            is (43.U) { cfg_noise_str := data_word(3,0) }
            // SRAM speed knobs (44..51)
            is (44.U) { cfg_sram_speed_h1(0) := data_word(7,0) }
            is (45.U) { cfg_sram_speed_h1(1) := data_word(7,0) }
            is (46.U) { cfg_sram_speed_h1(2) := data_word(7,0) }
            is (47.U) { cfg_sram_speed_h1(3) := data_word(7,0) }
            is (48.U) { cfg_sram_speed_h2(0) := data_word(7,0) }
            is (49.U) { cfg_sram_speed_h2(1) := data_word(7,0) }
            is (50.U) { cfg_sram_speed_h2(2) := data_word(7,0) }
            is (51.U) { cfg_sram_speed_h2(3) := data_word(7,0) }
            // Per-output bias writes (52..67)
            is (52.U) { cfg_out_bias8_vec(0)  := data_word(7,0).asSInt }
            is (53.U) { cfg_out_bias8_vec(1)  := data_word(7,0).asSInt }
            is (54.U) { cfg_out_bias8_vec(2)  := data_word(7,0).asSInt }
            is (55.U) { cfg_out_bias8_vec(3)  := data_word(7,0).asSInt }
            is (56.U) { cfg_out_bias8_vec(4)  := data_word(7,0).asSInt }
            is (57.U) { cfg_out_bias8_vec(5)  := data_word(7,0).asSInt }
            is (58.U) { cfg_out_bias8_vec(6)  := data_word(7,0).asSInt }
            is (59.U) { cfg_out_bias8_vec(7)  := data_word(7,0).asSInt }
            is (60.U) { cfg_out_bias8_vec(8)  := data_word(7,0).asSInt }
            is (61.U) { cfg_out_bias8_vec(9)  := data_word(7,0).asSInt }
            is (62.U) { cfg_out_bias8_vec(10) := data_word(7,0).asSInt }
            is (63.U) { cfg_out_bias8_vec(11) := data_word(7,0).asSInt }
            is (64.U) { cfg_out_bias8_vec(12) := data_word(7,0).asSInt }
            is (65.U) { cfg_out_bias8_vec(13) := data_word(7,0).asSInt }
            is (66.U) { cfg_out_bias8_vec(14) := data_word(7,0).asSInt }
            is (67.U) { cfg_out_bias8_vec(15) := data_word(7,0).asSInt }
            // Refractory/homeostasis (68..71)
            is (68.U) { cfg_ref_add_h1 := data_word(7,0) }
            is (69.U) { cfg_ref_dec_h1 := data_word(7,0) }
            is (70.U) { cfg_ref_add_h2 := data_word(7,0) }
            is (71.U) { cfg_ref_dec_h2 := data_word(7,0) }
            // DST (90..99)
            is (90.U) { cfg_dyn_en_h1    := data_word(0) }
            is (91.U) { cfg_dyn_en_h2    := data_word(0) }
            is (92.U) { cfg_dyn_x_h1     := data_word(15,0) }
            is (93.U) { cfg_dyn_x_h2     := data_word(15,0) }
            is (94.U) { cfg_prune_k_h1   := data_word(7,0) }
            is (95.U) { cfg_prune_k_h2   := data_word(7,0) }
            is (96.U) { cfg_collision_en := data_word(0) }
            is (97.U) { cfg_seed_dst     := data_word(16,0) }
            is (98.U) { cfg_seed_noise   := data_word(16,0) }
            is (99.U) { cfg_seed_strnd   := data_word(14,0) }
            // Stage mode (104)
            is (104.U){ cfg_stage_mode    := data_word(1,0) }
            // Output learning knobs (remapped: 108..111)
            is (108.U){ cfg_out_learn_use_act := data_word(0) }
            is (109.U){ cfg_out_stoch_en      := data_word(0) }
            is (110.U){ cfg_h1_grad_max       := data_word(15,0) }
            is (111.U){ cfg_h2_grad_max       := data_word(15,0) }
            // Regularization (remapped: 112..119)
            is (112.U){ cfg_regul_mode := data_word(2,0) }
            is (113.U){ cfg_regul_w    := data_word(1,0) }
            is (114.U){ cfg_regul_f0   := data_word(11,0) }
            is (115.U){ cfg_regul_k_inp_r := data_word(4,0) }
            is (116.U){ cfg_regul_k_inp_p := data_word(4,0) }
            is (117.U){ cfg_regul_k_rec_r := data_word(4,0) }
            is (118.U){ cfg_regul_k_rec_p := data_word(4,0) }
            is (119.U){ cfg_regul_k_mul   := data_word(4,0) }
            // Fixed-point locations (remapped: 120..125)
            is (120.U){ cfg_fp_loc_tinp := data_word(2,0) }
            is (121.U){ cfg_fp_loc_trec := data_word(2,0) }
            is (122.U){ cfg_fp_loc_tout := data_word(2,0) }
            is (123.U){ cfg_fp_loc_winp := data_word(2,0) }
            is (124.U){ cfg_fp_loc_wrec := data_word(2,0) }
            is (125.U){ cfg_fp_loc_wout := data_word(2,0) }
            // Global input activity gate threshold (remapped: 127)
            is (127.U){ cfg_thr_i         := data_word(15,0) }
          }
        }
        spi_burst_active     := false.B
      }
      is ("b110".U) { // alpha write (low 16 bits)
        cfg_alpha := data_word(15,0)
        spi_burst_active := false.B
      }
      is ("b111".U) { // diagnostics/mask write: when addr[15]=1 update out mask
        when (addr16(15)) { cfg_out_mask := data_word(15,0) }
        spi_burst_active := false.B
      }
      is ("b010".U) { // output threshold write (low 16 bits)
        cfg_thr_out := data_word(15,0)
        spi_burst_active := false.B
      }
      is ("b001".U) { // neur
        val a = cur_addr(8,2) // 7 bits
        val wsel = cur_wsel // 2 bits (which 32-bit chunk)
        spi_neur_cs := true.B; spi_neur_we := true.B; spi_neur_a := a
        spi_neur_m  := ("hFFFFFFFF".U(128.W) << (wsel<<5))
        spi_neur_d  := (data_word.zext().asUInt << (wsel<<5))
      }
      is ("b011".U) { // winp
        val a = cur_addr(13,2) // 12 bits
        val wsel = cur_wsel
        spi_winp_cs := true.B; spi_winp_we := true.B; spi_winp_a := a
        spi_winp_m  := ("hFFFFFFFF".U(128.W) << (wsel<<5))
        spi_winp_d  := (data_word.zext().asUInt << (wsel<<5))
      }
      is ("b100".U) { // wrec
        val a = cur_addr(13,2)
        val wsel = cur_wsel
        spi_wrec_cs := true.B; spi_wrec_we := true.B; spi_wrec_a := a
        spi_wrec_m  := ("hFFFFFFFF".U(128.W) << (wsel<<5))
        spi_wrec_d  := (data_word.zext().asUInt << (wsel<<5))
      }
      is ("b101".U) { // wout (assume 128b row, write 32b chunk selected by addr[1:0])
        val a = cur_addr(10,2) // 9 bits
        val wsel = cur_wsel
        spi_wout_cs := true.B; spi_wout_we := true.B; spi_wout_a := a
        spi_wout_m  := ("hFFFFFFFF".U(128.W) << (wsel<<5))
        spi_wout_d  := (data_word.zext().asUInt << (wsel<<5))
      }
      is ("b010".U) { // h1 banks overlay write: addr[8:7]=bank, [6:0]=row
        val bank = cur_addr(8,7)
        val a    = cur_addr(6,0)
        val wsel = cur_wsel
        val m32  = ("hFFFFFFFF".U(128.W) << (wsel<<5))
        val d32  = (data_word.zext().asUInt << (wsel<<5))
        for (b <- 0 until 4) {
          when (bank === b.U) { h1(b).io.cs := true.B; h1(b).io.we := true.B; h1(b).io.a := a; h1(b).io.d := d32; h1(b).io.m := m32 }
        }
      }
      is ("b110".U) { // h2 banks overlay write
        val bank = cur_addr(8,7)
        val a    = cur_addr(6,0)
        val wsel = cur_wsel
        val m32  = ("hFFFFFFFF".U(128.W) << (wsel<<5))
        val d32  = (data_word.zext().asUInt << (wsel<<5))
        for (b <- 0 until 4) {
          when (bank === b.U) { h2(b).io.cs := true.B; h2(b).io.we := true.B; h2(b).io.a := a; h2(b).io.d := d32; h2(b).io.m := m32 }
        }
      }
    }
    // Advance burst indexing (auto-increment 32-bit chunk, wrap every 4 chunks)
    when (spi_burst_active) {
      when (spi_burst_rem === 0.U) {
        spi_burst_active := false.B
        spi_expect_data  := false.B
      }.otherwise {
        spi_burst_rem := spi_burst_rem - 1.U
        when (spi_burst_wsel === 3.U) {
          spi_burst_wsel := 0.U
          // increment row address field per space
          switch (spi_burst_code) {
            is ("b001".U) { spi_burst_addr := Cat(spi_burst_addr(15,9) + 1.U,  spi_burst_addr(8,0)) }
            is ("b011".U) { spi_burst_addr := Cat(spi_burst_addr(15,14) + 1.U, spi_burst_addr(13,0)) }
            is ("b100".U) { spi_burst_addr := Cat(spi_burst_addr(15,14) + 1.U, spi_burst_addr(13,0)) }
            is ("b101".U) { spi_burst_addr := Cat(spi_burst_addr(15,11) + 1.U, spi_burst_addr(10,0)) }
          }
        }.otherwise {
          spi_burst_wsel := spi_burst_wsel + 1.U
        }
        spi_expect_data := true.B
      }
    }.otherwise {
      spi_expect_data := false.B
    }
  }

  // ------------------------------------------------------------------
  // AER debounce: double-sample and edge detect
  // ------------------------------------------------------------------
  val aer_req_sync1 = RegInit(false.B)
  val aer_req_sync2 = RegInit(false.B)
  when (cfg_aer_db) {
    aer_req_sync1 := io.AERIN_REQ
    aer_req_sync2 := aer_req_sync1
  } .otherwise {
    aer_req_sync1 := io.AERIN_REQ
    aer_req_sync2 := io.AERIN_REQ
  }
  val aer_req_q  = RegNext(aer_req_sync2, init=false.B)
  val aer_req_qq = RegNext(aer_req_q,     init=false.B)
  val aer_rise   = aer_req_q && !aer_req_qq

  // One-cycle ACK on aer_rise
  io.AERIN_ACK := aer_rise

  // ------------------------------------------------------------------
  // Simple AER FIFO to buffer input events (depth 16)
  // ------------------------------------------------------------------
  val aer_fifo = Module(new Queue(UInt(M_INP.W), 16))
  aer_fifo.io.enq.valid := aer_rise
  aer_fifo.io.enq.bits  := io.AERIN_ADDR
  val aer_has_evt = aer_fifo.io.deq.valid
  val aer_evt     = aer_fifo.io.deq.bits
  aer_fifo.io.deq.ready := false.B
  // Track FIFO level and high-water mark (diagnostic)
  val aer_fifo_level = RegInit(0.U(8.W))
  val aer_fifo_hwm   = RegInit(0.U(8.W))
  val aer_enq_fire   = aer_fifo.io.enq.valid && aer_fifo.io.enq.ready
  val aer_deq_fire   = aer_fifo.io.deq.valid && aer_fifo.io.deq.ready
  when (aer_enq_fire && !aer_deq_fire) {
    aer_fifo_level := Mux(aer_fifo_level === 255.U, 255.U, aer_fifo_level + 1.U)
  }.elsewhen (!aer_enq_fire && aer_deq_fire) {
    aer_fifo_level := aer_fifo_level - 1.U
  }
  when (aer_fifo_level > aer_fifo_hwm) { aer_fifo_hwm := aer_fifo_level }
  // Count AER events during PROP1 window per tick
  val aer_cnt_tick = RegInit(0.U(8.W))
  when ((state === State.PROP1) && aer_enq_fire) {
    aer_cnt_tick := Mux(aer_cnt_tick === 255.U, 255.U, aer_cnt_tick + 1.U)
  }
  when (tick_rise && (state === State.PROP1)) { aer_cnt_tick := 0.U }
  when (sample_rise) { aer_cnt_tick := 0.U }

  // ------------------------------------------------------------------
  // Label latch: emulate sample-latched behavior as default
  //   - On SAMPLE rising edge, capture LABEL_VALID/LABEL_INDEX
  //   - Clear on SAMPLE falling edge
  // ------------------------------------------------------------------
  val sample_q  = RegNext(io.SAMPLE, init=false.B)
  val sample_rise = !sample_q && io.SAMPLE
  val sample_fall = sample_q && !io.SAMPLE

  val label_active  = RegInit(false.B)
  val label_idx_reg = RegInit(0.U(M_OUT.W))
  when (cfg_label_smp) {
    when (sample_fall) {
      label_active := false.B
    } .elsewhen (sample_rise) {
      label_active := io.LABEL_VALID
      when (io.LABEL_VALID) { label_idx_reg := io.LABEL_INDEX }
    }
  } .otherwise {
    label_active := io.LABEL_VALID
    when (io.LABEL_VALID) { label_idx_reg := io.LABEL_INDEX }
  }

  // ------------------------------------------------------------------
  // Simple event-to-output accumulation (placeholder):
  // - Maintain 16 counters and 16 traces, clear on SAMPLE rise
  // - Increment bucket indexed by (AER address % (NUM_OUT_NEUR+1))
  // - traces decay by alpha; winner based on traces
  // ------------------------------------------------------------------
  val outCnt   = RegInit(VecInit(Seq.fill(16)(0.U(16.W))))
  val outTrace = RegInit(VecInit(Seq.fill(16)(0.U(16.W))))
  // Presynetic lane accumulators for output three-factor learning (sum H2 traces per lane within a sample)
  val preOutLane = RegInit(VecInit(Seq.fill(16)(0.U(12.W))))
  // Optional output neuron integrator (oneur) with leak by KAPPA and stochastic rounding
  val oneur_u   = RegInit(VecInit(Seq.fill(16)(0.S(16.W))))
  val oneur_lfsr= RegInit("h7ABA".U(15.W))
  def lfsrNext15(x: UInt): UInt = { val b = x(0)^x(1)^x(2)^x(14); Cat(b, x(14,1)) }
  // Noise LFSR for PROP phases
  val noise_lfsr = RegInit("h1F0F".U(16.W))
  noise_lfsr := lfsrNext(noise_lfsr)
  // Per-output error vector (e-prop placeholder), signed 16-bit
  val errVec   = RegInit(VecInit(Seq.fill(16)(0.S(16.W))))
  val err_snapshot = RegInit(VecInit(Seq.fill(16)(0.U(16.W))))
  // Per-output delta buffer for wout updates (placeholder), signed 16-bit
  val woutDelta= RegInit(VecInit(Seq.fill(16)(0.S(16.W))))
  // Per-output delta buffers for winp/wrec (placeholders)
  val winpDelta= RegInit(VecInit(Seq.fill(16)(0.S(16.W))))
  val wrecDelta= RegInit(VecInit(Seq.fill(16)(0.S(16.W))))
  when (sample_rise) {
    for (i <- 0 until 16) { outCnt(i) := 0.U; outTrace(i) := 0.U; preOutLane(i) := 0.U }
  }
  val numOutEff = (cfg_num_out_neur + 1.U)(4,0) // up to 16
  when ((state === State.PROP1) && aer_has_evt && aer_fifo.io.deq.ready && !vi_sp_active && !vi_inc_pending && !vi_inc_doing) {
    val idx = (aer_evt(M_OUT-1,0) % numOutEff)(M_OUT-1,0)
    val curr = outCnt(idx)
    outCnt(idx) := curr + 1.U
    val trc = outTrace(idx)
    // increment by 8 (placeholder fixed-point unit)
    val inc = 8.U(16.W)
    outTrace(idx) := Mux(trc > ("hFFFF".U(16.W) - inc), "hFFFF".U, trc + inc)
    // Initiate a read of input weights row for this AER address
    // Map: addr = {pre=aer_evt, postIdx chunk via sendCnt}
    ctrl_winp_cs := true.B
    ctrl_winp_we := false.B
    ctrl_winp_a  := Cat(aer_evt(7,0), 0.U(4.W)) // coarse: postsyn chunk 0
    ctrl_winp_pending := true.B
    // Schedule VI trace increment micro-op for H1 bank derived from AER address
    val vi_addr = (aer_evt + cfg_vi_base)(M_INP-1,0)
    vi_inc_bank_sel := vi_addr(8,7)
    vi_inc_row_idx  := vi_addr(6,0)
    vi_inc_pending  := true.B
    // Recurrent weights row (same presyn index placeholder)
    when (aer_evt(7,0) <= cfg_num_rec_neur) {
      ctrl_wrec_cs := true.B
      ctrl_wrec_we := false.B
      ctrl_wrec_a  := Cat(aer_evt(7,0), 0.U(4.W))
      ctrl_wrec_pending := true.B
    }
    // Track last presyn
    last_presyn     := aer_evt(7,0)
    last_presyn_vld := true.B
    // Neuron SRAM read (placeholder on lower 7 bits)
    ctrl_neur_cs := true.B
    ctrl_neur_we := false.B
    ctrl_neur_a  := aer_evt(6,0)
    ctrl_neur_pending := true.B
  }
  // Winner computation (combinational) with optional thresholding and activation mapping
  val winnerIdx_w = Wire(UInt(M_OUT.W))
  val winnerVal_w = Wire(UInt(16.W))
  var winIdx = 0.U(M_OUT.W)
  var winVal = 0.U(16.W)
  for (i <- 0 until 16) {
    val valid = (i.U <= cfg_num_out_neur) && cfg_out_mask(i)
    val raw_sel = Mux(cfg_use_oneur, oneur_u(i).asUInt, outTrace(i))
    val raw_u = raw_sel
    val actIdx = raw_u(15,8)
    val lutVal = ste_lut(actIdx ^ 128.U(8.W))
    val hsVal  = hardSigmoid8(raw_u)
    val act16  = Cat(Mux(cfg_out_ste_en, lutVal, hsVal), Mux(cfg_out_ste_en, lutVal, hsVal))
    val sv16   = svActY(raw_u.asSInt)
    val cand16 = Mux(cfg_sv_act_map && !cfg_no_out_act, sv16, Mux(cfg_stream_act && !cfg_no_out_act, act16, raw_u))
    val pass  = cand16 >= cfg_thr_out
    when (valid && pass && (cand16 > winVal)) { winVal = cand16; winIdx = i.U }
  }
  winnerIdx_w := winIdx
  winnerVal_w := winVal
  // Similarity accumulators and gating (basic): use H1/H2 T_hist/T_prev deltas accumulated when spikes occur
  val sim_pred_h1 = RegInit(0.U(32.W))
  val sim_contr_h1= RegInit(0.U(32.W))
  val sim_pred_h2 = RegInit(0.U(32.W))
  val sim_contr_h2= RegInit(0.U(32.W))
  val sim_pred_h1_l = RegInit(0.U(32.W))
  val sim_contr_h1_l= RegInit(0.U(32.W))
  val sim_pred_h2_l = RegInit(0.U(32.W))
  val sim_contr_h2_l= RegInit(0.U(32.W))
  // REGROW score feed during STEP sweeps: (T_hist - T_prev) * STE(u)
  // Simple per-group top-K heaps (K=16) by replacing group-local minimum when new score is larger
  def updateRgHeap(score: UInt, index: UInt, group: UInt,
                   heapScore: Vec[Vec[UInt]], heapIndex: Vec[Vec[UInt]], kActive: UInt): Unit = {
    val g = group(1,0)
    val s = score(23,0)
    val i = index(8,0)
    val hs = heapScore(g)
    val hi = heapIndex(g)
    // find current minimum among 16
    def min2(a: UInt, b: UInt) = Mux(a <= b, a, b)
    def min2i(a: UInt, b: UInt, ia: UInt, ib: UInt) = Mux(a <= b, ia, ib)
    val p0 = min2(hs(0), hs(1));  val i0 = min2i(hs(0), hs(1), 0.U, 1.U)
    val p1 = min2(hs(2), hs(3));  val i1 = min2i(hs(2), hs(3), 2.U, 3.U)
    val p2 = min2(hs(4), hs(5));  val i2 = min2i(hs(4), hs(5), 4.U, 5.U)
    val p3 = min2(hs(6), hs(7));  val i3 = min2i(hs(6), hs(7), 6.U, 7.U)
    val p4 = min2(hs(8), hs(9));  val i4 = min2i(hs(8), hs(9), 8.U, 9.U)
    val p5 = min2(hs(10),hs(11)); val i5 = min2i(hs(10),hs(11),10.U,11.U)
    val p6 = min2(hs(12),hs(13)); val i6 = min2i(hs(12),hs(13),12.U,13.U)
    val p7 = min2(hs(14),hs(15)); val i7 = min2i(hs(14),hs(15),14.U,15.U)
    val q0 = min2(p0,p1); val j0 = Mux(p0 <= p1, i0, i1)
    val q1 = min2(p2,p3); val j1 = Mux(p2 <= p3, i2, i3)
    val q2 = min2(p4,p5); val j2 = Mux(p4 <= p5, i4, i5)
    val q3 = min2(p6,p7); val j3 = Mux(p6 <= p7, i6, i7)
    val r0 = min2(q0,q1); val k0 = Mux(q0 <= q1, j0, j1)
    val r1 = min2(q2,q3); val k1 = Mux(q2 <= q3, j2, j3)
    val m  = min2(r0,r1); val im = Mux(r0 <= r1, k0, k1)
    // Only consider first kActive entries per group as active; ignore writes if s==0
    val imActive = im < kActive
    when ((s =/= 0.U) && (s > m) && imActive) { hs(im) := s; hi(im) := i }
  }
  val h1_spk_cnt_tick = RegInit(0.U(16.W))
  val h2_spk_cnt_tick = RegInit(0.U(16.W))
  // Clear per-tick at tick rise in STEP1/STEP2
  when (tick_rise && (state === State.STEP1)) {
    sim_pred_h1 := 0.U; sim_contr_h1 := 0.U; h1_spk_cnt_tick := 0.U
  }
  when (tick_rise && (state === State.STEP2)) {
    sim_pred_h2 := 0.U; sim_contr_h2 := 0.U; h2_spk_cnt_tick := 0.U
  }
  // Latch similarities at end of sweep (on tick rise entering next phase)
  when (tick_rise && (state === State.PROP2)) {
    sim_pred_h1_l  := sim_pred_h1_scaled
    sim_contr_h1_l := sim_contr_h1_scaled
  }
  when (tick_rise && (state === State.LEARN)) {
    sim_pred_h2_l  := sim_pred_h2_scaled
    sim_contr_h2_l := sim_contr_h2_scaled
    // snapshot errors for diagnostics
    for (i <- 0 until 16) { err_snapshot(i) := errVec(i).asUInt }
  }
  // Normalization and gates (combinational)
  // i_norm based on normalization mode
  val i_raw = Cat(0.U(24.W), aer_cnt_tick)
  val i_div = Wire(UInt(32.W))
  val neuron_count9 = (cfg_num_out_neur + 1.U)(8,0) // support up to 256
  val recipQ12 = VecInit((0 to 256).map { n =>
    val v = if (n == 0) 0 else ((1 << 12) + (n/2)) / n // rounded 1/n in Q12
    v.U(16.W)
  })
  val recip = recipQ12(Mux(neuron_count9 > 256.U, 256.U, neuron_count9)) // Q12
  val div_lut_wide = (i_raw * recip)(47,0) // 32+16 bits
  val i_div_lut = (div_lut_wide >> 12).asUInt()(31,0)
  i_div := MuxLookup(cfg_sim_norm_mode, i_raw, Seq(
    0.U -> i_raw, // none
    1.U -> i_raw, // already normalized by aer_cnt_tick semantics
    2.U -> Mux(neuron_count9 === 0.U, i_raw, i_div_lut)
  ))
  val i_norm = ((i_div * cfg_i_scale) >> cfg_i_shift)(31,0)
  val i_gate = i_norm >= cfg_thr_i
  val sim_pred_h1_scaled = ((sim_pred_h1 * cfg_sim_scale) >> cfg_sim_shift)(31,0)
  val sim_contr_h1_scaled= ((sim_contr_h1 * cfg_sim_scale) >> cfg_sim_shift)(31,0)
  val sim_pred_h2_scaled = ((sim_pred_h2 * cfg_sim_scale) >> cfg_sim_shift)(31,0)
  val sim_contr_h2_scaled= ((sim_contr_h2 * cfg_sim_scale) >> cfg_sim_shift)(31,0)
  val gate_pred1  = sim_pred_h1_scaled > (i_norm * cfg_h1_pred_gate)
  val gate_contr1 = sim_contr_h1_scaled < (i_norm * cfg_h1_contr_gate)
  val gate_pred2  = sim_pred_h2_scaled > (i_norm * cfg_h2_pred_gate)
  val gate_contr2 = sim_contr_h2_scaled < (i_norm * cfg_h2_contr_gate)
  val do_pred1    = !gate_pred1
  val do_contr1   = !gate_contr1
  val do_pred2    = !gate_pred2
  val do_contr2   = !gate_contr2
  // Latch winner near SEND if no label was provided (after STEP2)
  val winner_latched = RegInit(false.B)
  when (state === State.STEP2 && !io.SAMPLE && io.TIME_TICK) {
    when (!label_active) { label_idx_reg := winnerIdx_w }
    winner_latched := true.B
  }.elsewhen (state === State.IDLE) {
    winner_latched := false.B
  }

  // INFER_ACC: accumulate per-tick winners across a sample window (classification mode)
  val winAcc = RegInit(VecInit(Seq.fill(16)(0.U(16.W))))
  when (sample_rise) {
    for (i <- 0 until 16) { winAcc(i) := 0.U }
  }
  when ((state === State.STEP2) && tick_rise && io.INFER_ACC && !cfg_regression) {
    val idx = winnerIdx_w
    when (idx <= cfg_num_out_neur) {
      val v = winAcc(idx)
      winAcc(idx) := Mux(v === 65535.U, v, v + 1.U)
    }
  }

  // ------------------------------------------------------------------
  // Minimal per-output trace decay (uses alpha and kappa placeholders):
  // traces decay by alpha (Q1.15): tr' = tr * alpha >> 15
  // counts decay by kappa (Q0.7): cnt' = cnt * kappa >> 7
  // ------------------------------------------------------------------
  // Simple LFSR for stochastic rounding / noise
  val lfsr = RegInit("hACE1".U(16.W))
  def lfsrNext(x: UInt): UInt = {
    val bit = x(0) ^ x(2) ^ x(3) ^ x(5)
    Cat(bit, x(15,1))
  }
  when ((state === State.STEP1) || (state === State.STEP2)) {
    // Reseed low bits occasionally from cfg_seed_strnd and cfg_seed_noise XOR to allow deterministic runs
    when (tick_rise) { lfsr := (lfsrNext(lfsr) ^ Cat(0.U(1.W), cfg_seed_strnd, 0.U(0.W)) ^ cfg_seed_noise(15,0)) }
    lfsr := lfsrNext(lfsr)
    for (i <- 0 until 16) {
      val multCnt = (outCnt(i) * cfg_kappa)
      outCnt(i) := multCnt(14,7)
      val multTr  = (outTrace(i) * cfg_alpha)
      val baseTr  = multTr(30,15)
      val fracTr  = multTr(14,7) // 8-bit fraction proxy
      val stochInc= (cfg_en_stoch_round && (fracTr > lfsr(7,0))).asUInt
      val noisy   = Mux(cfg_noise_en, (lfsr(3,0)), 0.U)
      val upd     = baseTr + stochInc + noisy
      outTrace(i) := Mux(upd > "hFFFF".U, "hFFFF".U, upd)
      // oneur leak update when enabled: oneur_u = leak(oneur_u) with KAPPA and stochastic rounding
      when (cfg_use_oneur) {
        val lk = (oneur_u(i) * cfg_kappa.zext().asSInt)
        val int = lk(22,7).asSInt
        val frac= lk(6,0)
        oneur_lfsr := lfsrNext15(oneur_lfsr)
        val inc1 = (cfg_en_stoch_round && (cfg_kappa =/= "h80".U) && (frac >= oneur_lfsr) && lk(22,7).orR).asBool
        val lkst = int + Mux(inc1, 1.S, 0.S)
        oneur_u(i) := clampS16(lkst)
      }
    }
    // If a weight row is available, accumulate into traces (placeholder)
    when (ctrl_winp_pending) {
      val row = weightsIn.io.q
      // Accumulate per output index using corresponding 8-bit weight in the 128-bit row
      for (i <- 0 until 16) {
        when (i.U <= cfg_num_out_neur) {
          val w8  = row(8*i+7, 8*i)
          val acc = outTrace(i) + w8
          outTrace(i) := Mux(acc > "hFFFF".U, "hFFFF".U, acc)
        }
      }
      ctrl_winp_pending := false.B
    }
    when (ctrl_wrec_pending) {
      val rowR = weightsRec.io.q
      for (i <- 0 until 16) {
        when (i.U <= cfg_num_out_neur) {
          val w8r = rowR(8*i+7, 8*i)
          val acc = outTrace(i) + w8r
          outTrace(i) := Mux(acc > "hFFFF".U, "hFFFF".U, acc)
        }
      }
      ctrl_wrec_pending := false.B
    }
    when (ctrl_neur_pending) {
      val neurRow = neurons.io.q
      // Placeholder: if any bit set in the row, nudge traces by 1
      val bump = Mux(neurRow.orR, 1.U(16.W), 0.U)
      for (i <- 0 until 16) {
        val acc = outTrace(i) + bump
        outTrace(i) := Mux(acc > "hFFFF".U, "hFFFF".U, acc)
      }
      ctrl_neur_pending := false.B
    }
  }

  // ------------------------------------------------------------------
  // Minimal e-prop stub: compute output errors once when entering EPROP
  // e[k] = (label==k ? +2048 : 0) - (activation)
  // Activation placeholder uses outTrace (optionally thresholded/masked)
  // ------------------------------------------------------------------
  // Learn phase entry detection (parity with Verilog S_LEARN)
  val learn_prev_in = RegInit(false.B)
  val inLearn = (state === State.LEARN)
  learn_prev_in := inLearn
  val learn_enter = inLearn && !learn_prev_in
  when (inLearn && learn_enter) {
    for (i <- 0 until 16) {
      val valid = (i.U <= cfg_num_out_neur) && cfg_out_mask(i)
      val raw_sel = Mux(cfg_use_oneur, oneur_u(i).asUInt, outTrace(i))
      val raw_u = Mux(valid, raw_sel, 0.U)
      val actIdx= raw_u(15,8) ^ 255.U(8.W)
      val lutVal= ste_lut(actIdx ^ 128.U(8.W))
      val hsVal = hardSigmoid8(raw_u)
      val actB  = Mux(cfg_out_ste_en, lutVal, hsVal)
      // Apply per-output bias if programmed (non-zero), else global bias
      val bias8 = Mux(cfg_out_bias8_vec(i) =/= 0.S, cfg_out_bias8_vec(i), cfg_out_bias8)
      val biased16s = clampS16((raw_u.asSInt + (bias8 << 8)).asSInt)
      val biased16 = biased16s.asUInt
      val y_act16 = Cat(actB, actB)
      val y_u   = Mux(cfg_stream_act && !cfg_no_out_act, y_act16, biased16)
      val y_learn16 = Mux(cfg_out_learn_use_act, y_act16.asSInt, biased16s)
      val y     = y_learn16
      val yt_cls= Mux(label_active && (label_idx_reg === i.U), 2048.S(16.W), 0.S(16.W))
      val yt_reg= (io.LABEL_INDEX.zext() << 8).asSInt
      val yt    = Mux(cfg_regression, yt_reg, yt_cls)
      val e_raw = (yt - y)
      val e_shf = (e_raw >> cfg_out_err_scale).asSInt
      val e_mag = Mux(e_shf.abs().asUInt > cfg_out_err_max, cfg_out_err_max.asSInt, e_shf.abs())
      val e     = Mux(e_shf < 0.S, -e_mag, e_mag)
      errVec(i) := e
      when (cfg_do_eprop(2)) {
        val lrw  = lr_p_wout
        val pre    = cfg_out_pre_scale
        val e_pre  = ((e.asSInt * pre.zext().asSInt) >> 8).asSInt
        val stoch  = Mux(cfg_out_stoch_en, (lfsr(7,0).zext()).asSInt, 0.S)
        val scaled = ((e_pre + stoch) >> lrw).asSInt
        woutDelta(i) := scaled
      }.otherwise {
        woutDelta(i) := 0.S
      }
      when (cfg_do_eprop(0)) {
        winpDelta(i) := (e.asSInt >> lr_p_winp).asSInt
      }.otherwise { winpDelta(i) := 0.S }
      when (cfg_do_eprop(1)) {
        wrecDelta(i) := (e.asSInt >> lr_p_wrec).asSInt
      }.otherwise { wrecDelta(i) := 0.S }
    }
  }
  // Three-factor supervised output learner
  when (inLearn && !spi_en_conf && cfg_sup_out_en) {
    val out_k  = Mux(label_active, label_idx_reg, winnerIdx_w)
    val a      = Cat(0.U(5.W), out_k)
    // postsyn surrogate from selected output neuron
    val y_sel  = Mux(cfg_use_oneur, oneur_u(out_k).asUInt, outTrace(out_k))
    val steOut = ste_lut(y_sel(15,8) ^ 128.U(8.W))
    val e_k    = errVec(out_k)
    val oldRow = weightsOut.io.q
    ctrl_wout_cs := true.B; ctrl_wout_we := true.B; ctrl_wout_a := a
    var accRow = 0.U(128.W)
    var accM   = 0.U(128.W)
    for (i <- 0 until 16) {
      val old8   = oldRow(8*i+7, 8*i)
      val pre12  = preOutLane(i)
      // ((e*ste)>>8)*pre >> 8 => s16
      val prod1  = (e_k.asSInt * steOut.zext().asSInt).asSInt
      val red1   = (prod1 >> 8).asSInt
      val prod2  = (red1 * pre12.zext().asSInt).asSInt
      val delta16= (prod2 >> 8).asSInt
      val g16    = (delta16 >> lr_p_wout).asSInt
      val gabs   = Mux(g16 < 0.S, (-g16).asSInt, g16)
      val gcap   = Mux(gabs.asUInt > cfg_h2_grad_max, cfg_h2_grad_max.asSInt, gabs)
      val gsel   = Mux(g16 < 0.S, (-gcap).asSInt, gcap)
      val mag8   = gsel.asUInt(15,8)
      val rnd8   = lfsr(7,0)
      val upd    = stochUpdateByte(mag8, (gsel >= 0.S), old8, lr_r_wout, lr_p_wout, rnd8)
      accRow     = accRow | (upd._1 << (8*i).U)
      accM       = accM   | ("hFF".U(128.W) << (8*i).U)
    }
    ctrl_wout_d := accRow
    ctrl_wout_m := accM
  }
  when (inLearn && !spi_en_conf && last_presyn_vld) {
    when (cfg_dyn_en_h1 && !ol_busy) {
      // Hidden learner update for winp using similarity diff when enabled and gated
      val aI = Cat(last_presyn, 0.U(4.W))
      ctrl_winp_cs := true.B; ctrl_winp_we := true.B; ctrl_winp_a := aI
      val oldRowI = weightsIn.io.q
      var accRowI = 0.U(128.W)
      var accMI   = 0.U(128.W)
      val simDiffH1 = (sim_pred_h1.asSInt - sim_contr_h1.asSInt)
      val signPosH1 = simDiffH1 >= 0.S
      val magH1Full = (simDiffH1.abs().asUInt >> 8) // coarse scale
      val magH1     = (magH1Full >> lr_p_winp).asUInt()(7,0)
      val doUpdH1   = do_pred1 || do_contr1
      for (i <- 0 until 16) {
        val old8 = oldRowI(8*i+7, 8*i)
        val upd  = Mux(signPosH1, Mux(old8 + magH1 < old8, 255.U(8.W), old8 + magH1),
                                 Mux(old8 > magH1, old8 - magH1, 0.U(8.W)))
        val sat  = Mux(doUpdH1, upd, old8)
        accRowI  = accRowI | (sat << (8*i).U)
        accMI    = accMI   | ("hFF".U(128.W) << (8*i).U)
      }
      ctrl_winp_d := accRowI
      ctrl_winp_m := accMI
    }
    }.elsewhen (cfg_dyn_en_h2 && !ol_busy) {
      // Hidden learner update for wrec using similarity diff when enabled and gated
      val aR = Cat(last_presyn, 0.U(4.W))
      ctrl_wrec_cs := true.B; ctrl_wrec_we := true.B; ctrl_wrec_a := aR
      val oldRowR = weightsRec.io.q
      var accRowR = 0.U(128.W)
      var accMR   = 0.U(128.W)
      val simDiffH2 = (sim_pred_h2.asSInt - sim_contr_h2.asSInt)
      val signPosH2 = simDiffH2 >= 0.S
      val magH2Full = (simDiffH2.abs().asUInt >> 8)
      val magH2     = (magH2Full >> lr_p_wrec).asUInt()(7,0)
      val doUpdH2   = do_pred2 || do_contr2
      for (i <- 0 until 16) {
        val old8 = oldRowR(8*i+7, 8*i)
        val upd  = Mux(signPosH2, Mux(old8 + magH2 < old8, 255.U(8.W), old8 + magH2),
                                 Mux(old8 > magH2, old8 - magH2, 0.U(8.W)))
        val sat  = Mux(doUpdH2, upd, old8)
        accRowR  = accRowR | (sat << (8*i).U)
        accMR    = accMR   | ("hFF".U(128.W) << (8*i).U)
      }
      ctrl_wrec_d := accRowR
      ctrl_wrec_m := accMR
    }
  }

  // ------------------------------------------------------------------
  // Hidden learner micro-FSMs (minimal): iterate sparse words for last presyn
  // and apply per-synaptic updates to dense weights using lane masks
  // ------------------------------------------------------------------
  object LState extends ChiselEnum { val IDLE, SP_READ, SP_WAIT, POST, RMW, NEXT = Value }
  // L1 (VI->H1)
  val l1_state      = RegInit(LState.IDLE)
  val l1_word_idx   = RegInit(0.U(3.W))
  val l1_presyn_idx = RegInit(0.U(9.W))
  val l1_bank       = Wire(UInt(2.W))
  val l1_row        = Wire(UInt(7.W))
  l1_bank := l1_presyn_idx(8,7); l1_row := l1_presyn_idx(6,0)
  val l1_sp_q       = Wire(UInt(136.W))
  l1_sp_q := sp_vi_h1.io.q
  // SV-aligned parallel half-word processing (4 slots per half)
  val l1_sp_word    = RegInit(0.U(136.W))
  val l1_slot_half  = RegInit(0.U(1.W)) // 0 -> slots 0..3, 1 -> 4..7
  val l1_slot_idx   = RegInit(0.U(2.W)) // 0..3 within half
  val l1_acc_mask   = RegInit(0.U(128.W))
  val l1_acc_data   = RegInit(0.U(128.W))
  // L2 (H1->H2)
  val l2_state      = RegInit(LState.IDLE)
  val l2_word_idx   = RegInit(0.U(3.W))
  val l2_presyn_idx = RegInit(0.U(9.W))
  val l2_bank       = Wire(UInt(2.W))
  val l2_row        = Wire(UInt(7.W))
  l2_bank := l2_presyn_idx(8,7); l2_row := l2_presyn_idx(6,0)
  val l2_sp_q       = Wire(UInt(136.W))
  l2_sp_q := sp_h1_h2.io.q
  // SV-aligned parallel half-word processing (4 slots per half)
  val l2_sp_word    = RegInit(0.U(136.W))
  val l2_slot_half  = RegInit(0.U(1.W))
  val l2_slot_idx   = RegInit(0.U(2.W))
  val l2_acc_mask   = RegInit(0.U(128.W))
  val l2_acc_data   = RegInit(0.U(128.W))

  // Kick learners at LEARN if DST not busy and supervised OL not busy
  val learners_en = inLearn && !ol_busy && stage_unsup
  when (!learners_en) {
    l1_state := LState.IDLE; l2_state := LState.IDLE
    l1_word_idx := 0.U; l2_word_idx := 0.U
  }
  // Learners considered idle when both micro-FSMs are in IDLE within LEARN
  val learners_idle = (state === State.LEARN) && (l1_state === LState.IDLE) && (l2_state === LState.IDLE)
  // L1 sequence
  when (learners_en && cfg_dyn_en_h1) {
    switch (l1_state) {
      is (LState.IDLE) {
        when (last_presyn_vld) {
          l1_presyn_idx := Cat(0.U(1.W), last_presyn) // 9-bit
          l1_word_idx   := 0.U
          l1_state      := LState.SP_READ
        }
      }
      is (LState.SP_READ) {
        val sp_addr = Cat(l1_bank, l1_row, l1_word_idx)
        sp_vi_h1.io.cs := true.B; sp_vi_h1.io.we := false.B; sp_vi_h1.io.a := sp_addr
        l1_state := LState.SP_WAIT
      }
      is (LState.SP_WAIT) {
        when (sp_vi_h1_do_valid) {
          l1_sp_word   := l1_sp_q
          l1_slot_half := 0.U
          l1_slot_idx  := 0.U
          l1_acc_mask  := 0.U
          l1_acc_data  := 0.U
          l1_state     := LState.POST
        }
      }
      is (LState.POST) {
        // Prepare slot and read postsynaptic neuron row (H1) for STE and traces
        val slot   = Cat(l1_slot_half, l1_slot_idx).asUInt // 0..7
        val idxVec = spIdxVec(l1_sp_word)
        val j_idx  = idxVec(slot)
        val bank2  = j_idx(8,7)
        val row2   = j_idx(6,0)
        for (b <- 0 until 4) { when (bank2 === b.U) { h1(b).io.cs := true.B; h1(b).io.we := false.B; h1(b).io.a := row2 } }
        // Also read dense weights row once per half (to access old bytes)
        when (l1_slot_idx === 0.U) {
          val aI = Cat(last_presyn, 0.U(4.W))
          ctrl_winp_cs := true.B; ctrl_winp_we := false.B; ctrl_winp_a := aI
        }
        l1_state := LState.RMW
      }
      is (LState.RMW) {
        // Strict three-factor update for current slot; accumulate into per-half masked write
        val slot   = Cat(l1_slot_half, l1_slot_idx).asUInt
        val idxVec = spIdxVec(l1_sp_word)
        val wVec   = spW8Vec(l1_sp_word)
        val j_idx  = idxVec(slot)
        val w8     = wVec(slot)
        val lane   = j_idx(3,0)
        val byteOff= (lane << 3).asUInt
        val aI     = Cat(last_presyn, 0.U(4.W))
        val rowOld = weightsIn.io.q
        val old8u  = rowOld((8.U*lane) + 7.U, 8.U*lane)
        val old8s  = old8u.asSInt
        // Capture postsyn row
        val bank2 = j_idx(8,7)
        val q_post = WireDefault(0.U(128.W))
        for (b <- 0 until 4) { when (bank2 === b.U) { q_post := h1(b).io.q } }
        val u_post   = q_post(79,64)
        val t_hist   = q_post(71,60)
        val t_prev   = q_post(59,48)
        val t_vi     = q_post(95,84)
        val ste8     = ste_lut(u_post(15,8) ^ 128.U(8.W))
        val diff_tp  = (t_hist.zext() - t_prev.zext()).asSInt
        val postMul  = (diff_tp * ste8.zext().asSInt).asSInt
        val postRed  = (postMul >> 8).asSInt
        val delta32  = (postRed * t_vi.zext().asSInt).asSInt
        val delta16  = (delta32 >> 8).asSInt
        val g16_raw  = (delta16 >> lr_p_winp).asSInt
        val gabs     = Mux(g16_raw < 0.S, (-g16_raw).asSInt, g16_raw)
        val gcap     = Mux(gabs.asUInt > cfg_h1_grad_max, cfg_h1_grad_max.asSInt, gabs)
        val gsel     = Mux(g16_raw < 0.S, (-gcap).asSInt, gcap)
        val mag8     = gsel.asUInt(15,8)
        val rnd1     = (lfsr(0)).asUInt
        val mag8s    = (mag8 + Mux(cfg_en_stoch_round, rnd1, 0.U))(7,0)
        val pos      = gsel >= 0.S
        val step8s   = Mux(pos, mag8s.zext().asSInt, -(mag8s.zext().asSInt))
        val w9       = (old8s.zext() + step8s.zext()).asSInt
        val w_sat    = Mux(w9 > 127.S, 127.S, Mux(w9 < (-128).S, (-128).S, w9))
        // Slot validity: treat {idx,w}==0 as empty; group match: slot%4 == presyn bank
        val slotGroup = (slot(1,0))
        val syn_valid = (j_idx.orR || w8.orR)
        val group_ok  = (slotGroup === l1_bank)
        // Zero-skipping: require all three-factor components non-zero
        val pre_nz    = t_vi =/= 0.U
        val post_nz   = (diff_tp =/= 0.S) && (ste8 =/= 0.U)
        val doUpd     = syn_valid && group_ok && (do_pred1 || do_contr1) && i_gate && pre_nz && post_nz
        when (doUpd) {
          l1_acc_mask := l1_acc_mask | ("hFF".U(128.W) << byteOff)
          l1_acc_data := l1_acc_data | (w_sat.asUInt << byteOff)
        }
        // Assertions: group vs bank, and write mask non-empty when committing
        when (syn_valid) {
          assert(slotGroup === l1_bank, "L1 slot group must equal presyn bank")
        }
        // Advance within half or commit
        when (l1_slot_idx =/= 3.U) {
          l1_slot_idx := l1_slot_idx + 1.U
          l1_state    := LState.POST
        }.otherwise {
          // Commit one coalesced write for this half if any lane updated
          when (l1_acc_mask.orR) {
          ctrl_winp_cs := true.B; ctrl_winp_we := true.B; ctrl_winp_a := aI
            ctrl_winp_m  := l1_acc_mask
            ctrl_winp_d  := l1_acc_data
            assert(ctrl_winp_m.orR, "L1 write mask must be non-empty when writing")
            // Count a hidden-learning commit toward DST cadence
            when (cfg_dyn_en_h1) { cnt_steps_dst_h1 := Mux(cnt_steps_dst_h1 === 65535.U, cnt_steps_dst_h1, cnt_steps_dst_h1 + 1.U) }
          }
          // Prepare next half or next word
          when (l1_slot_half === 0.U) {
            l1_slot_half := 1.U; l1_slot_idx := 0.U; l1_acc_mask := 0.U; l1_acc_data := 0.U
            l1_state := LState.POST
          }.otherwise {
        l1_state := LState.NEXT
          }
        }
      }
      is (LState.NEXT) {
        val last = (l1_word_idx === (cfg_words_per_pre_h1 - 1.U))
        when (last) { l1_state := LState.IDLE }
          .otherwise { l1_word_idx := l1_word_idx + 1.U; l1_state := LState.SP_READ }
      }
    }
  }
  // L2 sequence
  when (learners_en && cfg_dyn_en_h2) {
    switch (l2_state) {
      is (LState.IDLE) {
        when (last_presyn_vld) {
          l2_presyn_idx := Cat(0.U(1.W), last_presyn)
          l2_word_idx   := 0.U
          l2_state      := LState.SP_READ
        }
      }
      is (LState.SP_READ) {
        val sp_addr = Cat(l2_bank, l2_row, l2_word_idx)
        sp_h1_h2.io.cs := true.B; sp_h1_h2.io.we := false.B; sp_h1_h2.io.a := sp_addr
        l2_state := LState.SP_WAIT
      }
      is (LState.SP_WAIT) {
        when (sp_h1_h2_do_valid) {
          l2_sp_word   := l2_sp_q
          l2_slot_half := 0.U
          l2_slot_idx  := 0.U
          l2_acc_mask  := 0.U
          l2_acc_data  := 0.U
          l2_state     := LState.POST
        }
      }
      is (LState.POST) {
        // Prepare slot and read postsyn row
        val slot   = Cat(l2_slot_half, l2_slot_idx).asUInt
        val idxVec = spIdxVec(l2_sp_word)
        val j_idx  = idxVec(slot)
        val bank2  = j_idx(8,7)
        val row2   = j_idx(6,0)
        for (b <- 0 until 4) { when (bank2 === b.U) { h2(b).io.cs := true.B; h2(b).io.we := false.B; h2(b).io.a := row2 } }
        when (l2_slot_idx === 0.U) {
          val aR = Cat(last_presyn, 0.U(4.W))
          ctrl_wrec_cs := true.B; ctrl_wrec_we := false.B; ctrl_wrec_a := aR
        }
        l2_state := LState.RMW
      }
      is (LState.RMW) {
        val slot   = Cat(l2_slot_half, l2_slot_idx).asUInt
        val idxVec = spIdxVec(l2_sp_word)
        val wVec   = spW8Vec(l2_sp_word)
        val j_idx  = idxVec(slot)
        val w8     = wVec(slot)
        val lane   = j_idx(3,0)
        val byteOff= (lane << 3).asUInt
        val aR     = Cat(last_presyn, 0.U(4.W))
        val rowOld = weightsRec.io.q
        val old8u  = rowOld((8.U*lane) + 7.U, 8.U*lane)
        val old8s  = old8u.asSInt
        val bank2 = j_idx(8,7)
        val q_post = WireDefault(0.U(128.W))
        for (b <- 0 until 4) { when (bank2 === b.U) { q_post := h2(b).io.q } }
        val u_post   = q_post(79,64)
        val t_hist   = q_post(71,60)
        val t_prev   = q_post(59,48)
        val t_h1     = q_post(83,72)
        val ste8     = ste_lut(u_post(15,8) ^ 128.U(8.W))
        val diff_tp  = (t_hist.zext() - t_prev.zext()).asSInt
        val postMul  = (diff_tp * ste8.zext().asSInt).asSInt
        val postRed  = (postMul >> 8).asSInt
        val delta32  = (postRed * t_h1.zext().asSInt).asSInt
        val delta16  = (delta32 >> 8).asSInt
        val g16_raw  = (delta16 >> lr_p_wrec).asSInt
        val gabs     = Mux(g16_raw < 0.S, (-g16_raw).asSInt, g16_raw)
        val gcap     = Mux(gabs.asUInt > cfg_h2_grad_max, cfg_h2_grad_max.asSInt, gabs)
        val gsel     = Mux(g16_raw < 0.S, (-gcap).asSInt, gcap)
        val mag8     = gsel.asUInt(15,8)
        val rnd1     = (lfsr(1)).asUInt
        val mag8s    = (mag8 + Mux(cfg_en_stoch_round, rnd1, 0.U))(7,0)
        val pos      = gsel >= 0.S
        val step8s   = Mux(pos, mag8s.zext().asSInt, -(mag8s.zext().asSInt))
        val w9       = (old8s.zext() + step8s.zext()).asSInt
        val w_sat    = Mux(w9 > 127.S, 127.S, Mux(w9 < (-128).S, (-128).S, w9))
        val slotGroup = slot(1,0)
        val syn_valid = (j_idx.orR || w8.orR)
        val group_ok  = (slotGroup === l2_bank)
        // Zero-skipping: require all three-factor components non-zero
        val pre_nz    = t_h1 =/= 0.U
        val post_nz   = (diff_tp =/= 0.S) && (ste8 =/= 0.U)
        val doUpd     = syn_valid && group_ok && (do_pred2 || do_contr2) && i_gate && pre_nz && post_nz
        when (doUpd) {
          l2_acc_mask := l2_acc_mask | ("hFF".U(128.W) << byteOff)
          l2_acc_data := l2_acc_data | (w_sat.asUInt << byteOff)
        }
        when (syn_valid) {
          assert(slotGroup === l2_bank, "L2 slot group must equal presyn bank")
        }
        when (l2_slot_idx =/= 3.U) {
          l2_slot_idx := l2_slot_idx + 1.U
          l2_state    := LState.POST
        }.otherwise {
          when (l2_acc_mask.orR) {
          ctrl_wrec_cs := true.B; ctrl_wrec_we := true.B; ctrl_wrec_a := aR
            ctrl_wrec_m  := l2_acc_mask
            ctrl_wrec_d  := l2_acc_data
            assert(ctrl_wrec_m.orR, "L2 write mask must be non-empty when writing")
            when (cfg_dyn_en_h2) { cnt_steps_dst_h2 := Mux(cnt_steps_dst_h2 === 65535.U, cnt_steps_dst_h2, cnt_steps_dst_h2 + 1.U) }
          }
          when (l2_slot_half === 0.U) {
            l2_slot_half := 1.U; l2_slot_idx := 0.U; l2_acc_mask := 0.U; l2_acc_data := 0.U
            l2_state := LState.POST
          }.otherwise {
        l2_state := LState.NEXT
          }
        }
      }
      is (LState.NEXT) {
        val last = (l2_word_idx === (cfg_words_per_pre_h2 - 1.U))
        when (last) { l2_state := LState.IDLE }
          .otherwise { l2_word_idx := l2_word_idx + 1.U; l2_state := LState.SP_READ }
      }
    }
  }

  // ------------------------------------------------------------------
  // Event controller FSM (skeleton): WAIT -> CNT -> EVT -> CNT/WAIT
  // Steps through input neuron indices during PROP, reacts to aer_has_evt
  // ------------------------------------------------------------------
  object EState extends ChiselEnum { val WAIT, CNT, EVT, SUPP = Value }
  val eState     = RegInit(EState.WAIT)
  val eNextState = WireDefault(eState)
  eState := eNextState
  // Compact trace gating signals inspired by srnn
  val inp_evt_trace  = Wire(Bool())
  val rec_evt_trace  = Wire(Bool())
  val out_evt_trace  = Wire(Bool())
  val neur256_cnt = RegInit(0.U(8.W))

  when ((state =/= State.PROP1) && (state =/= State.PROP2)) {
    eState := EState.WAIT
    neur256_cnt := 0.U
  }

  when ((state === State.PROP1) || (state === State.PROP2)) {
    switch (eState) {
      is (EState.WAIT) {
        eNextState := EState.CNT
      }
      is (EState.CNT) {
        // advance scan; loop back to WAIT at end
        when (neur256_cnt === cfg_num_inp_neur) {
          eNextState := EState.WAIT
          neur256_cnt := 0.U
        }.otherwise {
          // if any event pending, process one
          when (aer_has_evt) { eNextState := EState.EVT }
          .otherwise { neur256_cnt := neur256_cnt + 1.U }
        }
      }
      is (EState.EVT) {
      // one compact SUPP cycle if configured and scan crossed rec region
      eNextState := Mux(cfg_send_per_timestep, EState.SUPP, EState.CNT)
      }
      is (EState.SUPP) {
        eNextState := EState.CNT
      }
    }
  // Gating: drive trace updates during EVT/SUPP over the scanned address
  inp_evt_trace := ((eState === EState.EVT) || (eState === EState.SUPP)) && aer_has_evt
  // Recurrent trace gate during PROP2 when a sparse burst is active
  rec_evt_trace :=  ((state === State.PROP2) && (((eState === EState.EVT) || (eState === EState.SUPP)) && h2_sp_active))
  // Output trace gate during EVT/SUPP (aligned with scan events)
  out_evt_trace :=  ((eState === EState.EVT) || (eState === EState.SUPP))
  }

  // ------------------------------------------------------------------
  // Output SEND behavior (placeholder): stream either 1 label byte or
  // 2*NUM_OUT_NEUR bytes (dummy) based on cfg_send_label_only
  // ------------------------------------------------------------------
  val out_req    = RegInit(false.B)
  val out_data_r = RegInit(0.U(8.W))
  val sendCnt    = RegInit(0.U(6.W))
  val sendNum    = Wire(UInt(6.W))
  val numOutPlusOne = (cfg_num_out_neur + 1.U)(5,0)
  // Parity with Verilog: label-only applies only when not in regression mode
  sendNum := Mux(cfg_send_label_only && !cfg_regression, 1.U, Mux(cfg_single_label, 2.U, (numOutPlusOne << 1).asUInt))

  // Default outputs
  io.OUT_REQ  := out_req
  io.OUT_DATA := out_data_r

  // SEND state driving
  when (state =/= State.SEND) {
    out_req := false.B
    sendCnt := 0.U
  }.otherwise {
    // Issue request when not active and more to send
    when (!out_req && (sendCnt < sendNum)) {
      when (cfg_send_label_only && !cfg_regression) {
        // Emit predicted class index. If INFER_ACC enabled, use argmax of winAcc; else use current winnerIdx
        val accMaxIdx = Wire(UInt(M_OUT.W))
        val accMaxVal = Wire(UInt(16.W))
        var mIdx = 0.U(M_OUT.W)
        var mVal = 0.U(16.W)
        for (i <- 0 until 16) {
          when ((i.U <= cfg_num_out_neur) && (winAcc(i) > mVal)) { mVal = winAcc(i); mIdx = i.U }
        }
        accMaxIdx := mIdx; accMaxVal := mVal
        val sendIdx = Mux(io.INFER_ACC, accMaxIdx, winnerIdx_w)
        out_data_r := Cat(0.U(4.W), sendIdx(3,0))
      }.otherwise {
        when (cfg_single_label) {
          val hi = sendCnt(0)
          val raw_u16 = winnerVal_w
          val sv16    = svActY(raw_u16.asSInt)
          val lsbW = Mux(cfg_sv_act_map && !cfg_no_out_act, sv16(7,0), raw_u16(7,0))
          val msbW = Mux(cfg_sv_act_map && !cfg_no_out_act, sv16(15,8), raw_u16(15,8))
          out_data_r := Mux(hi, msbW, lsbW)
        }.otherwise {
          val idx = (sendCnt >> 1).asUInt // 0..NUM_OUT_NEUR
          val hi  = sendCnt(0)
          val valRaw0 = Mux((idx <= cfg_num_out_neur) && cfg_out_mask(idx), outTrace(idx), 0.U)
          val biaseds = clampS16((valRaw0.asSInt + (cfg_out_bias8.asSInt << 8)).asSInt)
          val biased  = biaseds.asUInt
          val actIdx  = biased(15,8)
          val lutVal  = ste_lut(actIdx ^ 128.U(8.W))
          val hsVal   = hardSigmoid8(biased)
          val act16   = Cat(Mux(cfg_out_ste_en, lutVal, hsVal), Mux(cfg_out_ste_en, lutVal, hsVal))
          val sv16    = svActY(biaseds)
          val stream16= Mux(cfg_regression || (cfg_sv_act_map && !cfg_no_out_act), sv16,
                            Mux(cfg_stream_act && !cfg_no_out_act, act16, biased))
          val lsb = stream16(7,0)
          val msb = stream16(15,8)
          out_data_r := Mux(hi, msb, lsb)
        }
      }
      out_req    := true.B
    }
    // On ACK, consume one item
    when (out_req && io.OUT_ACK) {
      out_req := false.B
      sendCnt := sendCnt + 1.U
    }
  }

  // ------------------------------------------------------------------
  // TIMING_ERROR_RDY policy: when timing mode enabled, require state in [IDLE, PROP]
  // otherwise default to a simple barrier on tick fall
  // ------------------------------------------------------------------
  // Local/external tick selection
  val tick_cnt  = RegInit(0.U(32.W))
  val tick_int  = RegInit(false.B)
  when (cfg_local_tick) {
    when (tick_cnt === cfg_cycles_per_tick) {
      tick_int := true.B; tick_cnt := 0.U
    } .otherwise {
      tick_int := false.B; tick_cnt := tick_cnt + 1.U
    }
  } .otherwise {
    tick_int := io.TIME_TICK
    tick_cnt := 0.U
  }
  val tick_q = RegNext(tick_int, init=false.B)
  val tick_rise = !tick_q && tick_int
  val tick_fall = tick_q && !tick_int
  val bursts_idle = !((state === State.PROP1 && vi_sp_active) || (state === State.PROP2 && h2_sp_active))
  // STEP2 done flag: set when finishing last writeback in STEP2, clear on new tick start in PROP1
  val step2_done_flag = RegInit(true.B)
  when (reset.asBool) { step2_done_flag := true.B }
  .elsewhen (state === State.STEP2 && h2_step_rw && (h2_step_idx === (127).U)) { step2_done_flag := true.B }
  .elsewhen (io.TIME_TICK && (state === State.PROP1)) { step2_done_flag := false.B }
  // Burst window accounting: SV uses 4 bursts per tick by default
  val bursts_left = RegInit(0.U(4.W))
  when (state === State.IDLE && io.SAMPLE) {
    bursts_left := 4.U
  }.elsewhen ((state === State.PROP1) && vi_sp_active && aer_has_evt && aer_fifo.io.deq.ready) {
    // decrement when we actually chain a new presyn event in PROP1
    when (bursts_left =/= 0.U) { bursts_left := bursts_left - 1.U }
  }.elsewhen (state =/= State.PROP1) {
    // reset when leaving PROP1
    bursts_left := 0.U
  }
  // DST busy gate: block readiness when DST FSMs are active
  val dst_busy = (dst1_state =/= DState.D_IDLE && dst1_state =/= DState.D_DONE) || (dst2_state =/= DState.D_IDLE && dst2_state =/= DState.D_DONE)
  val ready_for_tick = Mux(cfg_pipelined,
    Mux(state === State.PROP1, (bursts_idle && step2_done_flag && (bursts_left === 0.U) && !h2_spike_fifo.io.deq.valid && !dst_busy).B, true.B),
    (state === State.IDLE) && !dst_busy)
  // Timing error/sticky policy
  val timing_error = RegInit(false.B)
  when (tick_int && !ready_for_tick) { timing_error := true.B }
  .elsewhen (!cfg_error_halt && timing_error) { timing_error := false.B }
  val timing_ok = ready_for_tick
  io.TIMING_ERROR_RDY := Mux(cfg_timing_mode, !timing_error, ready_for_tick && !dst_busy)

  // Diagnostics: tick and spike counters
  val cnt_ticks     = RegInit(0.U(32.W))
  val cnt_spikes_h1 = RegInit(0.U(16.W))
  val cnt_spikes_h2 = RegInit(0.U(16.W))
  when (tick_rise) { cnt_ticks := cnt_ticks + 1.U }
  // DST step counters (increment when enabled and in STEP phases, paused by output learning)
  val ol_busy = Wire(Bool())
  ol_busy := (cfg_sup_out_en && cfg_do_eprop(2) && ((state === State.LEARN) || (state === State.SEND)) && label_active)
  // Hidden-learning driven DST cadence: increment counters on hidden learner commit (see learner RMW commits)

  // ------------------------------------------------------------------
  // FSM (parity with Verilog): IDLE, PROP1, STEP1, PROP2, STEP2, LEARN, SEND, CONFIG
  // ------------------------------------------------------------------
  object State extends ChiselEnum {
    val IDLE, PROP1, STEP1, PROP2, STEP2, LEARN, SEND, CONFIG = Value
  }
  val state     = RegInit(State.IDLE)
  val nextState = WireDefault(state)
  state := nextState

  val stepCnt   = RegInit(0.U(16.W)) // used for placeholder delays
  val tickSeen  = RegInit(false.B)

  switch (state) {
    is (State.IDLE)  {
      when (spi_en_conf)               { nextState := State.CONFIG }
      .elsewhen (io.SAMPLE && aer_has_evt) { nextState := State.PROP1 }
    }
    is (State.PROP1)  {
      // Chain presyn bursts while bursts_left!=0 and FIFO not empty; else proceed to STEP1 when idle
      when (!vi_sp_active && aer_has_evt && (bursts_left =/= 0.U)) {
        aer_fifo.io.deq.ready := true.B
      }
      .elsewhen (!vi_sp_active && ((bursts_left === 0.U) || !aer_has_evt)) {
        nextState := State.STEP1
      }
    }
    is (State.STEP1)  {
      when (io.TIME_TICK) { nextState := State.PROP2 }
    }
    is (State.PROP2)  {
      // Drain H1 spikes; move to STEP2 when burst engine is idle and FIFO empty
      when (!h2_sp_active && !h1_spike_fifo.io.deq.valid) { nextState := State.STEP2 }
    }
    is (State.STEP2)  {
      when (io.TIME_TICK) { nextState := State.LEARN }
    }
    is (State.LEARN) {
      when (io.TIME_TICK && (cfg_send_per_timestep || !io.SAMPLE)) { nextState := State.SEND }
      .elsewhen (io.TIME_TICK)                                      { nextState := State.PROP1 }
    }
    is (State.CONFIG) {
      when (!spi_en_conf)              { nextState := State.IDLE }
    }
    is (State.SEND)  {
      when ((sendCnt === sendNum) && !out_req) { nextState := State.IDLE }
    }
  }

  // ------------------------------------------------------------------
  // Assertions
  // ------------------------------------------------------------------
  // OUT bus invariants
  assert(!(state =/= State.SEND && out_req), "OUT_REQ must be low outside SEND state")
  assert(!(out_req && (sendCnt >= sendNum)), "OUT_REQ asserted when no data to send")
  // If SEND has items remaining and no request in-flight, request should be issued
  when (state === State.SEND && (sendCnt < sendNum) && !spi_en_conf) {
    assert(out_req || io.OUT_ACK, "SEND should assert OUT_REQ when items remain or be acknowledging")
  }
  // ACK must drop request by next cycle
  val out_req_q = RegNext(out_req, init=false.B)
  val out_ack_q = RegNext(io.OUT_ACK, init=false.B)
  when (out_req_q && out_ack_q) {
    assert(!out_req, "OUT_REQ must deassert in the cycle after OUT_ACK")
  }
  // Mode-derived send count expectations
  when (cfg_send_label_only && !cfg_regression) {
    assert(sendNum === 1.U, "sendNum must be 1 in label-only classification mode")
  }
  // CONFIG state requires SPI enable
  when (state === State.CONFIG) {
    assert(spi_en_conf, "CONFIG state requires spi_en_conf to be set")
  }

  // Basic coverage (event counters) for FSM and OUT handshake
  val cov_idle_to_prop1   = RegInit(0.U(16.W))
  val cov_prop1_to_step1  = RegInit(0.U(16.W))
  val cov_step1_to_prop2  = RegInit(0.U(16.W))
  val cov_prop2_to_step2  = RegInit(0.U(16.W))
  val cov_step2_to_learn  = RegInit(0.U(16.W))
  val cov_learn_to_send   = RegInit(0.U(16.W))
  val cov_send_to_idle    = RegInit(0.U(16.W))
  val cov_out_req_acks    = RegInit(0.U(16.W))
  val state_q             = RegNext(state, init=State.IDLE)
  when (state_q === State.IDLE  && state === State.PROP1) { cov_idle_to_prop1  := Mux(cov_idle_to_prop1 === 65535.U, cov_idle_to_prop1, cov_idle_to_prop1 + 1.U) }
  when (state_q === State.PROP1 && state === State.STEP1) { cov_prop1_to_step1 := Mux(cov_prop1_to_step1=== 65535.U, cov_prop1_to_step1,cov_prop1_to_step1+1.U) }
  when (state_q === State.STEP1 && state === State.PROP2) { cov_step1_to_prop2 := Mux(cov_step1_to_prop2=== 65535.U, cov_step1_to_prop2,cov_step1_to_prop2+1.U) }
  when (state_q === State.PROP2 && state === State.STEP2) { cov_prop2_to_step2 := Mux(cov_prop2_to_step2=== 65535.U, cov_prop2_to_step2,cov_prop2_to_step2+1.U) }
  when (state_q === State.STEP2 && state === State.LEARN) { cov_step2_to_learn := Mux(cov_step2_to_learn=== 65535.U, cov_step2_to_learn,cov_step2_to_learn+1.U) }
  when (state_q === State.LEARN && state === State.SEND)  { cov_learn_to_send  := Mux(cov_learn_to_send === 65535.U, cov_learn_to_send, cov_learn_to_send + 1.U) }
  when (state_q === State.SEND  && state === State.IDLE)  { cov_send_to_idle   := Mux(cov_send_to_idle  === 65535.U, cov_send_to_idle,  cov_send_to_idle  + 1.U) }
  val out_req_ack_fire = out_req && io.OUT_ACK
  when (out_req_ack_fire) { cov_out_req_acks := Mux(cov_out_req_acks === 65535.U, cov_out_req_acks, cov_out_req_acks + 1.U) }

  // ------------------------------------------------------------------
  // SRAM wrappers (replace placeholders)
  // ------------------------------------------------------------------

  // Thin SRAM wrapper modules (not yet fully integrated)
  class Sram128x128 extends Module {
    val io = IO(new Bundle {
      val cs = Input(Bool())
      val we = Input(Bool())
      val a  = Input(UInt(7.W))
      val d  = Input(UInt(128.W))
      val m  = Input(UInt(128.W))
      val q  = Output(UInt(128.W))
    })
    // 2-cycle RMW to honor bit-level mask m: next = (d & m) | (old & ~m)
    val mem = SyncReadMem(128, Vec(16, UInt(8.W)))
    val qReg = RegInit(0.U(128.W))
    val wrPending = RegInit(false.B)
    val wrAddr    = RegInit(0.U(7.W))
    val wrD       = RegInit(0.U(128.W))
    val wrM       = RegInit(0.U(128.W))
    val rdVec     = Wire(Vec(16, UInt(8.W)))
    val rdHold    = Reg(Vec(16, UInt(8.W)))
    val rdEn      = WireDefault(false.B)
    val rdValid   = RegInit(false.B)
    rdVec := mem.read(Mux(wrPending, wrAddr, io.a), rdEn)
    rdValid := rdEn

    when (io.cs && !io.we && !wrPending) {
      rdEn := true.B
    }.elsewhen (io.cs && io.we && !wrPending) {
      // Launch read for RMW
      rdEn := true.B
      wrPending := true.B
      wrAddr    := io.a
      wrD       := io.d
      wrM       := io.m
    }.elsewhen (wrPending) {
      // Second cycle: have rdVec valid, perform write of full 128b
      rdEn := false.B
      for (i <- 0 until 16) { rdHold(i) := rdVec(i) }
      val oldWord = Cat(rdHold.reverse)
      val nextWord= (wrD & wrM) | (oldWord & (~wrM).asUInt)
      val nextBytes = Wire(Vec(16, UInt(8.W)))
      for (i <- 0 until 16) { nextBytes(i) := nextWord(8*i+7, 8*i) }
      // Write all bytes
      mem.write(wrAddr, nextBytes, VecInit(Seq.fill(16)(true.B)))
      wrPending := false.B
    }
    // Register read data one cycle after rdEn for clean timing
    when (rdValid && !wrPending) { qReg := Cat(rdVec.reverse) }
    io.q := qReg
  }
  class Sram4096x128 extends Module {
    val io = IO(new Bundle {
      val cs = Input(Bool())
      val we = Input(Bool())
      val a  = Input(UInt(12.W))
      val d  = Input(UInt(128.W))
      val m  = Input(UInt(128.W))
      val q  = Output(UInt(128.W))
    })
    val mem = SyncReadMem(4096, Vec(16, UInt(8.W)))
    val qReg = RegInit(0.U(128.W))
    val wrPending = RegInit(false.B)
    val wrAddr    = RegInit(0.U(12.W))
    val wrD       = RegInit(0.U(128.W))
    val wrM       = RegInit(0.U(128.W))
    val rdVec     = Wire(Vec(16, UInt(8.W)))
    val rdHold    = Reg(Vec(16, UInt(8.W)))
    val rdEn      = WireDefault(false.B)
    val rdValid   = RegInit(false.B)
    rdVec := mem.read(Mux(wrPending, wrAddr, io.a), rdEn)
    rdValid := rdEn

    when (io.cs && !io.we && !wrPending) {
      rdEn := true.B
    }.elsewhen (io.cs && io.we && !wrPending) {
      rdEn := true.B
      wrPending := true.B
      wrAddr    := io.a
      wrD       := io.d
      wrM       := io.m
    }.elsewhen (wrPending) {
      rdEn := false.B
      for (i <- 0 until 16) { rdHold(i) := rdVec(i) }
      val oldWord = Cat(rdHold.reverse)
      val nextWord= (wrD & wrM) | (oldWord & (~wrM).asUInt)
      val nextBytes = Wire(Vec(16, UInt(8.W)))
      for (i <- 0 until 16) { nextBytes(i) := nextWord(8*i+7, 8*i) }
      mem.write(wrAddr, nextBytes, VecInit(Seq.fill(16)(true.B)))
      wrPending := false.B
    }
    when (rdValid && !wrPending) { qReg := Cat(rdVec.reverse) }
    io.q := qReg
  }
  class Sram512x128 extends Module {
    val io = IO(new Bundle {
      val cs = Input(Bool())
      val we = Input(Bool())
      val a  = Input(UInt(9.W))
      val d  = Input(UInt(128.W))
      val m  = Input(UInt(128.W))
      val q  = Output(UInt(128.W))
    })
    val mem = SyncReadMem(512, Vec(16, UInt(8.W)))
    val qReg = RegInit(0.U(128.W))
    val wrPending = RegInit(false.B)
    val wrAddr    = RegInit(0.U(9.W))
    val wrD       = RegInit(0.U(128.W))
    val wrM       = RegInit(0.U(128.W))
    val rdVec     = Wire(Vec(16, UInt(8.W)))
    val rdHold    = Reg(Vec(16, UInt(8.W)))
    val rdEn      = WireDefault(false.B)
    val rdValid   = RegInit(false.B)
    rdVec := mem.read(Mux(wrPending, wrAddr, io.a), rdEn)
    rdValid := rdEn

    when (io.cs && !io.we && !wrPending) {
      rdEn := true.B
    }.elsewhen (io.cs && io.we && !wrPending) {
      rdEn := true.B
      wrPending := true.B
      wrAddr    := io.a
      wrD       := io.d
      wrM       := io.m
    }.elsewhen (wrPending) {
      rdEn := false.B
      for (i <- 0 until 16) { rdHold(i) := rdVec(i) }
      val oldWord = Cat(rdHold.reverse)
      val nextWord= (wrD & wrM) | (oldWord & (~wrM).asUInt)
      val nextBytes = Wire(Vec(16, UInt(8.W)))
      for (i <- 0 until 16) { nextBytes(i) := nextWord(8*i+7, 8*i) }
      mem.write(wrAddr, nextBytes, VecInit(Seq.fill(16)(true.B)))
      wrPending := false.B
    }
    when (rdValid && !wrPending) { qReg := Cat(rdVec.reverse) }
    io.q := qReg
  }

  // Simple sparse memory wrapper: 4096 x 136b
  // SV-aligned packing: 8 synapses per 136-bit word, each synapse = {idx[8:0], w[7:0]} (17b)
  // Slot mapping: slots 0..3 map to groups 0..3; slots 4..7 map again to groups 0..3
  // We derive group = (slot % 4) and bank from idx[8:7].
  // Helper extractors (compile-time and dynamic selection):
  def spSynIdx(q: UInt, slot: Int): UInt = { require(slot >=0 && slot < 8); q(17*slot+16, 17*slot+8) }
  def spSynW8(q: UInt, slot: Int): UInt  = { require(slot >=0 && slot < 8); q(17*slot+7,  17*slot) }
  def spIdxVec(q: UInt): Vec[UInt] = { VecInit((0 until 8).map(s => q(17*s+16, 17*s+8))) }
  def spW8Vec(q: UInt): Vec[UInt]  = { VecInit((0 until 8).map(s => q(17*s+7,  17*s))) }
  // DST compact-format bit-field positions (from Cat(0, w, j, group, slot, flag) at prune/regrow)
  private val SP_W_L = 15; private val SP_W_H = 22
  private val SP_J_L =  6; private val SP_J_H = 14
  private val SP_G_L =  4; private val SP_G_H =  5
  private val DST_HEAP_K_MAX = 16
  class Sparse4096x136 extends Module {
    val io = IO(new Bundle {
      val cs = Input(Bool())
      val we = Input(Bool())
      val a  = Input(UInt(12.W))
      val d  = Input(UInt(136.W))
      val q  = Output(UInt(136.W))
    })
    val mem = SyncReadMem(4096, UInt(136.W))
    val qReg = RegInit(0.U(136.W))
    when (io.cs) {
      when (io.we) { mem.write(io.a, io.d) } .otherwise { qReg := mem.read(io.a, true.B) }
    }
    io.q := qReg
  }

  // Instances (currently idle)
  val neurons  = Module(new Sram128x128)
  neurons.io.cs := false.B; neurons.io.we := false.B
  neurons.io.a  := 0.U; neurons.io.d := 0.U; neurons.io.m := 0.U
  val weightsIn = Module(new Sram4096x128)
  weightsIn.io.cs := false.B; weightsIn.io.we := false.B
  weightsIn.io.a  := 0.U; weightsIn.io.d := 0.U; weightsIn.io.m := 0.U
  val weightsRec = Module(new Sram4096x128)
  weightsRec.io.cs := false.B; weightsRec.io.we := false.B
  weightsRec.io.a  := 0.U; weightsRec.io.d := 0.U; weightsRec.io.m := 0.U
  val weightsOut = Module(new Sram512x128)
  weightsOut.io.cs := false.B; weightsOut.io.we := false.B
  weightsOut.io.a  := 0.U; weightsOut.io.d := 0.U; weightsOut.io.m := 0.U

  // Sparse synapse memories (starter): VI->H1 and H1->H2
  val sp_vi_h1 = Module(new Sparse4096x136)
  sp_vi_h1.io.cs := false.B; sp_vi_h1.io.we := false.B; sp_vi_h1.io.a := 0.U; sp_vi_h1.io.d := 0.U
  val sp_h1_h2 = Module(new Sparse4096x136)
  sp_h1_h2.io.cs := false.B; sp_h1_h2.io.we := false.B; sp_h1_h2.io.a := 0.U; sp_h1_h2.io.d := 0.U
  // DO-valid wrappers for sparse SRAMs (1-cycle read latency)
  val sp_vi_h1_do_valid = RegInit(false.B)
  val sp_h1_h2_do_valid = RegInit(false.B)
  sp_vi_h1_do_valid := RegNext(sp_vi_h1.io.cs && !sp_vi_h1.io.we, init=false.B)
  sp_h1_h2_do_valid := RegNext(sp_h1_h2.io.cs && !sp_h1_h2.io.we, init=false.B)

  // ------------------------------------------------------------------
  // Minimal DST prune/regrow: for each layer, scan a window of words for
  // the smallest |weight| and replace it with a random new synapse meta.
  // This is a drastic simplification of SV's heaps and collision logic.
  // ------------------------------------------------------------------
  object DState extends ChiselEnum { val D_IDLE, D_SCAN, D_WRITE, D_DONE = Value }
  object CState extends ChiselEnum { val C_IDLE, C_NEW = Value }
  // H1 DST
  val dst1_state   = RegInit(DState.D_IDLE)
  val dst1_idx     = RegInit(0.U(12.W))
  val dst1_limit   = RegInit(64.U(12.W)) // scan window per trigger
  val dst1_minMag  = RegInit(255.U(8.W))
  val dst1_minAddr = RegInit(0.U(12.W))
  val dst1_k_left  = RegInit(0.U(8.W))
  // Collision staging for H1 DST write
  val coll1_state  = RegInit(CState.C_IDLE)
  val coll1_addr   = RegInit(0.U(12.W))
  val coll1_word   = RegInit(0.U(136.W))
  val coll2_state  = RegInit(CState.C_IDLE)
  val coll2_addr   = RegInit(0.U(12.W))
  val coll2_word   = RegInit(0.U(136.W))
  // Top-K heap for smallest |w| (K<=16)
  val dst1_heap_mags  = RegInit(VecInit(Seq.fill(16)(255.U(8.W))))
  val dst1_heap_addrs = RegInit(VecInit(Seq.fill(16)(0.U(12.W))))
  val dst1_heap_wi    = RegInit(0.U(5.W))
  // Run H1 DST during LEARN, when hidden learners are active (avoid SRAM conflicts): parallel with H2
  when (cfg_dyn_en_h1 && stage_unsup && (state === State.LEARN) && learners_idle && (cnt_steps_dst_h1 >= cfg_dyn_x_h1) && (cfg_prune_k_h1 =/= 0.U)) {
    switch (dst1_state) {
      is (DState.D_IDLE) {
        dst1_idx := 0.U; dst1_minMag := 255.U; dst1_minAddr := 0.U; dst1_k_left := cfg_prune_k_h1; dst1_limit := cfg_dst_scan_h1;
        for (i <- 0 until 16) { dst1_heap_mags(i) := 255.U; dst1_heap_addrs(i) := 0.U }
        dst1_heap_wi := 0.U
        dst1_state := DState.D_SCAN
      }
      is (DState.D_SCAN) {
        cnt_dst_cycles_h1 := Mux(cnt_dst_cycles_h1 === 65535.U, cnt_dst_cycles_h1, cnt_dst_cycles_h1 + 1.U)
        // Avoid conflict with learners: only access sparse when L1 not committing
        sp_vi_h1.io.cs := true.B; sp_vi_h1.io.we := false.B; sp_vi_h1.io.a := dst1_idx
        when (sp_vi_h1_do_valid) {
          val q = sp_vi_h1.io.q
          val w8 = q(SP_W_H,SP_W_L)
          val mag = Mux(w8(7), (~w8 + 1.U)(7,0), w8)
          // Update top-K heap: replace current worst if new mag smaller
          // 16-way max to identify current worst entry
          val m0 = dst1_heap_mags(0);  val m1 = dst1_heap_mags(1);  val m2 = dst1_heap_mags(2);  val m3 = dst1_heap_mags(3)
          val m4 = dst1_heap_mags(4);  val m5 = dst1_heap_mags(5);  val m6 = dst1_heap_mags(6);  val m7 = dst1_heap_mags(7)
          val m8 = dst1_heap_mags(8);  val m9 = dst1_heap_mags(9);  val mA = dst1_heap_mags(10); val mB = dst1_heap_mags(11)
          val mC = dst1_heap_mags(12); val mD = dst1_heap_mags(13); val mE = dst1_heap_mags(14); val mF = dst1_heap_mags(15)
          def max2(a: UInt, b: UInt, ia: UInt, ib: UInt):(UInt,UInt) = { (Mux(b >= a, b, a), Mux(b >= a, ib, ia)) }
          val (v01,i01) = max2(m0,m1,0.U,1.U); val (v23,i23) = max2(m2,m3,2.U,3.U)
          val (v45,i45) = max2(m4,m5,4.U,5.U); val (v67,i67) = max2(m6,m7,6.U,7.U)
          val (v89,i89) = max2(m8,m9,8.U,9.U); val (vAB,iAB) = max2(mA,mB,10.U,11.U)
          val (vCD,iCD) = max2(mC,mD,12.U,13.U);val (vEF,iEF) = max2(mE,mF,14.U,15.U)
          val (v0F,i0F) = max2(v01,v23,i01,i23); val (v4F,i4F) = max2(v45,v67,i45,i67)
          val (v8F,i8F) = max2(v89,vAB,i89,iAB); val (vCF,iCF) = max2(vCD,vEF,iCD,iEF)
          val (vL, iL)  = max2(v0F,v4F,i0F,i4F)
          val (vR, iR)  = max2(v8F,vCF,i8F,iCF)
          val vmax = Mux(vR >= vL, vR, vL); val imax = Mux(vR >= vL, iR, iL)
          when (mag < vmax) { dst1_heap_mags(imax) := mag; dst1_heap_addrs(imax) := dst1_idx }
          // Keep fallback min for compatibility
          when (mag < dst1_minMag) { dst1_minMag := mag; dst1_minAddr := dst1_idx }
          val last = (dst1_idx === (dst1_limit - 1.U))
          when (last) {
            when (dst1_k_left > 0.U && (dst1_limit < 4096.U)) {
              // continue scan to cover full space in multiple windows
              dst1_idx := 0.U; dst1_limit := Mux((dst1_limit + cfg_dst_scan_h1) > 4096.U, 4096.U, dst1_limit + cfg_dst_scan_h1)
            }.otherwise {
              dst1_heap_wi := 0.U; dst1_state := DState.D_WRITE
            }
          } .otherwise { dst1_idx := dst1_idx + 1.U }
        }
      }
      is (DState.D_WRITE) {
        // Iterate heap entries; prune up to K without rescanning
        val addrSel = dst1_heap_addrs(dst1_heap_wi)
        val magSel  = dst1_heap_mags(dst1_heap_wi)
        sp_vi_h1.io.cs := true.B; sp_vi_h1.io.we := false.B; sp_vi_h1.io.a := addrSel
        when (sp_vi_h1_do_valid && (magSel =/= 255.U) && (dst1_k_left =/= 0.U)) {
          val cur = sp_vi_h1.io.q
          val cur_j   = cur(SP_J_H, SP_J_L)
          val cur_grp = cur(SP_G_H, SP_G_L)
          // Choose regrow candidate per-group: select max score within pruned group's heap
          val zero24 = 0.U(24.W)
          var bestScore = zero24
          var bestIdx9  = 0.U(9.W)
          var bestEnt   = 0.U(4.W)
            for (e <- 0 until 16) {
            val sc = rg_heap_score1(cur_grp)(e)
            when (sc > bestScore) { bestScore = sc; bestIdx9 = rg_heap_index1(cur_grp)(e); bestEnt = e.U }
          }
          val useHeap = (bestScore =/= 0.U)
          val new_group = cur_grp
          val new_j_raw = Mux(useHeap, bestIdx9,  dst_lfsr(10,2))
          val new_j     = Mux(cfg_collision_en && (new_group === cur_grp) && (new_j_raw === cur_j), new_j_raw + 1.U, new_j_raw)
          // Clear consumed regrow heap entry in pruned group
          when (useHeap) { rg_heap_score1(cur_grp)(bestEnt) := 0.U; rg_heap_index1(cur_grp)(bestEnt) := 0.U }
          val new_w     = dst_lfsr(14,7)
          val new_slot  = dst_lfsr(6,4)
          val word = Cat(0.U(113.W), new_w, new_j, new_group, new_slot, 1.U(1.W))
          coll1_addr := addrSel; coll1_word := word; coll1_state := CState.C_NEW
        }.otherwise {
          when (dst1_heap_wi === 15.U || dst1_k_left === 0.U) {
            cnt_steps_dst_h1 := 0.U
            dst1_state := DState.D_DONE
          }.otherwise {
            dst1_heap_wi := dst1_heap_wi + 1.U
          }
        }
      }
      is (DState.D_DONE) {
        dst1_state := DState.D_IDLE
      }
    }
  }.otherwise { dst1_state := DState.D_IDLE }
  // H1 collision staged write
  when (coll1_state === CState.C_NEW) {
    sp_vi_h1.io.cs := true.B; sp_vi_h1.io.we := true.B; sp_vi_h1.io.a := coll1_addr; sp_vi_h1.io.d := coll1_word
    val new_w = coll1_word(SP_W_H,SP_W_L)
    val new_j = coll1_word(SP_J_H,SP_J_L)
    val new_g = coll1_word(SP_G_H,SP_G_L)
    cnt_dst_prunes_h1 := Mux(cnt_dst_prunes_h1 === 65535.U, cnt_dst_prunes_h1, cnt_dst_prunes_h1 + 1.U)
    last_dst_meta_h1 := Cat(coll1_addr, new_w, new_j(1,0), new_g)
    dst1_k_left := dst1_k_left - 1.U
    dst1_heap_wi := dst1_heap_wi + 1.U
    coll1_state := CState.C_IDLE
  }

  // H2 DST
  val dst2_state   = RegInit(DState.D_IDLE)
  val dst2_idx     = RegInit(0.U(12.W))
  val dst2_limit   = RegInit(64.U(12.W))
  val dst2_minMag  = RegInit(255.U(8.W))
  val dst2_minAddr = RegInit(0.U(12.W))
  val dst2_k_left  = RegInit(0.U(8.W))
  // Top-K heap for H2 (K<=16)
  val dst2_heap_mags  = RegInit(VecInit(Seq.fill(16)(255.U(8.W))))
  val dst2_heap_addrs = RegInit(VecInit(Seq.fill(16)(0.U(12.W))))
  val dst2_heap_wi    = RegInit(0.U(5.W))
  // Run H2 DST during LEARN as well
  when (cfg_dyn_en_h2 && stage_unsup && (state === State.LEARN) && learners_idle && (cnt_steps_dst_h2 >= cfg_dyn_x_h2) && (cfg_prune_k_h2 =/= 0.U)) {
    switch (dst2_state) {
      is (DState.D_IDLE) {
        dst2_idx := 0.U; dst2_minMag := 255.U; dst2_minAddr := 0.U; dst2_k_left := cfg_prune_k_h2; dst2_limit := cfg_dst_scan_h2;
        for (i <- 0 until 16) { dst2_heap_mags(i) := 255.U; dst2_heap_addrs(i) := 0.U }
        dst2_heap_wi := 0.U
        dst2_state := DState.D_SCAN
      }
      is (DState.D_SCAN) {
        cnt_dst_cycles_h2 := Mux(cnt_dst_cycles_h2 === 65535.U, cnt_dst_cycles_h2, cnt_dst_cycles_h2 + 1.U)
        // Avoid conflict with learners: only access sparse when L2 not committing
        sp_h1_h2.io.cs := true.B; sp_h1_h2.io.we := false.B; sp_h1_h2.io.a := dst2_idx
        when (sp_h1_h2_do_valid) {
          val q = sp_h1_h2.io.q
          val w8 = q(SP_W_H,SP_W_L)
          val mag = Mux(w8(7), (~w8 + 1.U)(7,0), w8)
          // 16-way max
          val m0 = dst2_heap_mags(0);  val m1 = dst2_heap_mags(1);  val m2 = dst2_heap_mags(2);  val m3 = dst2_heap_mags(3)
          val m4 = dst2_heap_mags(4);  val m5 = dst2_heap_mags(5);  val m6 = dst2_heap_mags(6);  val m7 = dst2_heap_mags(7)
          val m8 = dst2_heap_mags(8);  val m9 = dst2_heap_mags(9);  val mA = dst2_heap_mags(10); val mB = dst2_heap_mags(11)
          val mC = dst2_heap_mags(12); val mD = dst2_heap_mags(13); val mE = dst2_heap_mags(14); val mF = dst2_heap_mags(15)
          def max2(a: UInt, b: UInt, ia: UInt, ib: UInt):(UInt,UInt) = { (Mux(b >= a, b, a), Mux(b >= a, ib, ia)) }
          val (v01,i01) = max2(m0,m1,0.U,1.U); val (v23,i23) = max2(m2,m3,2.U,3.U)
          val (v45,i45) = max2(m4,m5,4.U,5.U); val (v67,i67) = max2(m6,m7,6.U,7.U)
          val (v89,i89) = max2(m8,m9,8.U,9.U); val (vAB,iAB) = max2(mA,mB,10.U,11.U)
          val (vCD,iCD) = max2(mC,mD,12.U,13.U);val (vEF,iEF) = max2(mE,mF,14.U,15.U)
          val (v0F,i0F) = max2(v01,v23,i01,i23); val (v4F,i4F) = max2(v45,v67,i45,i67)
          val (v8F,i8F) = max2(v89,vAB,i89,iAB); val (vCF,iCF) = max2(vCD,vEF,iCD,iEF)
          val (vL, iL)  = max2(v0F,v4F,i0F,i4F)
          val (vR, iR)  = max2(v8F,vCF,i8F,iCF)
          val vmax = Mux(vR >= vL, vR, vL); val imax = Mux(vR >= vL, iR, iL)
          when (mag < vmax) { dst2_heap_mags(imax) := mag; dst2_heap_addrs(imax) := dst2_idx }
          when (mag < dst2_minMag) { dst2_minMag := mag; dst2_minAddr := dst2_idx }
          val last = (dst2_idx === (dst2_limit - 1.U))
          when (last) {
            when (dst2_k_left > 0.U && (dst2_limit < 4096.U)) {
              dst2_idx := 0.U; dst2_limit := Mux((dst2_limit + cfg_dst_scan_h2) > 4096.U, 4096.U, dst2_limit + cfg_dst_scan_h2)
            }.otherwise {
              dst2_heap_wi := 0.U; dst2_state := DState.D_WRITE
            }
          } .otherwise { dst2_idx := dst2_idx + 1.U }
        }
      }
      is (DState.D_WRITE) {
        val addrSel = dst2_heap_addrs(dst2_heap_wi)
        val magSel  = dst2_heap_mags(dst2_heap_wi)
        sp_h1_h2.io.cs := true.B; sp_h1_h2.io.we := false.B; sp_h1_h2.io.a := addrSel
        when (sp_h1_h2_do_valid && (magSel =/= 255.U) && (dst2_k_left =/= 0.U)) {
          val cur = sp_h1_h2.io.q
          val cur_j   = cur(SP_J_H, SP_J_L)
          val cur_grp = cur(SP_G_H, SP_G_L)
          // Pick best regrow candidate within pruned group's heap (per-group regrow)
          val zero24 = 0.U(24.W)
          var bestScore = zero24
          var bestIdx9  = 0.U(9.W)
          var bestEnt   = 0.U(4.W)
            for (e <- 0 until 16) {
            val sc = rg_heap_score2(cur_grp)(e)
            when (sc > bestScore) { bestScore = sc; bestIdx9 = rg_heap_index2(cur_grp)(e); bestEnt = e.U }
          }
          val useHeap = (bestScore =/= 0.U)
          val new_group = cur_grp
          val new_j_raw = Mux(useHeap, bestIdx9,  dst_lfsr(10,2))
          val new_j     = Mux(cfg_collision_en && (new_group === cur_grp) && (new_j_raw === cur_j), new_j_raw + 1.U, new_j_raw)
          when (useHeap) { rg_heap_score2(cur_grp)(bestEnt) := 0.U; rg_heap_index2(cur_grp)(bestEnt) := 0.U }
          val new_w     = dst_lfsr(14,7)
          val new_slot  = dst_lfsr(6,4)
          val word = Cat(0.U(113.W), new_w, new_j, new_group, new_slot, 1.U(1.W))
          coll2_addr := addrSel; coll2_word := word; coll2_state := CState.C_NEW
        }.otherwise {
          when (dst2_heap_wi === 15.U || dst2_k_left === 0.U) {
            cnt_steps_dst_h2 := 0.U
            dst2_state := DState.D_DONE
          }.otherwise {
            dst2_heap_wi := dst2_heap_wi + 1.U
          }
        }
      }
      is (DState.D_DONE) {
        dst2_state := DState.D_IDLE
      }
    }
  }.otherwise { dst2_state := DState.D_IDLE }
  // H2 collision staged write
  when (coll2_state === CState.C_NEW) {
    sp_h1_h2.io.cs := true.B; sp_h1_h2.io.we := true.B; sp_h1_h2.io.a := coll2_addr; sp_h1_h2.io.d := coll2_word
    val new_w = coll2_word(SP_W_H,SP_W_L)
    val new_j = coll2_word(SP_J_H,SP_J_L)
    val new_g = coll2_word(SP_G_H,SP_G_L)
    cnt_dst_prunes_h2 := Mux(cnt_dst_prunes_h2 === 65535.U, cnt_dst_prunes_h2, cnt_dst_prunes_h2 + 1.U)
    last_dst_meta_h2 := Cat(coll2_addr, new_w, new_j(1,0), new_g)
    dst2_k_left := dst2_k_left - 1.U
    dst2_heap_wi := dst2_heap_wi + 1.U
    coll2_state := CState.C_IDLE
  }

  // ------------------------------------------------------------------
  // H1 neuron banks (4x 128x128) with simple STEP1 leak/threshold update
  // Layout per 128b word (simplified): [127:112]=alpha(Q1.15), [111:96]=thr(s16),
  // [95:84]=T_vi(12b), [83:72]=T_h1(12b), [71:60]=T_hist(12b), [59:48]=T_prev(12b)
  // ------------------------------------------------------------------
  val h1 = Seq.fill(4)(Module(new Sram128x128))
  for (b <- 0 until 4) {
    h1(b).io.cs := false.B; h1(b).io.we := false.B
    h1(b).io.a  := 0.U;     h1(b).io.d  := 0.U;     h1(b).io.m := 0.U
  }

  // Simple STEP1 sweep over 128 indices across 4 banks in parallel
  val h1_step_idx   = RegInit(0.U(7.W))
  val h1_step_rw    = RegInit(false.B) // 0=READ, 1=WRITE
  val h1_spk_pending= RegInit(VecInit(Seq.fill(4)(false.B)))
  // Circular buffer write pointers for hist/prev per bank (4-tap rings)
  val h1_hist_wptr  = RegInit(VecInit(Seq.fill(4)(0.U(2.W))))
  val h1_prev_wptr  = RegInit(VecInit(Seq.fill(4)(0.U(2.W))))
  // Snapshot of prev write pointers captured on SAMPLE falling edge for stable T_prev reads across one sweep
  val h1_prev_wptr_snap = RegInit(VecInit(Seq.fill(4)(0.U(2.W))))
  // SAMPLE-fall snapshot window flag for H1
  val h1_prev_snap = RegInit(false.B)
  when (state === State.STEP1) {
    when (!h1_step_rw) {
      // READ phase: issue read to all banks
      for (b <- 0 until 4) { h1(b).io.cs := true.B; h1(b).io.we := false.B; h1(b).io.a := h1_step_idx }
      h1_step_rw := true.B
    }.otherwise {
      // WRITE phase: compute leak/threshold/spike and write back with stochastic rounding
      for (b <- 0 until 4) {
        val q      = h1(b).io.q
        val u      = q(79,64).asSInt
        val thr    = q(111,96).asSInt
        val alpha  = q(127,112).asUInt
        val spk    = u >= thr
        // refractory/homeostasis: adjust thr
        val thr_inc = (thr.asSInt + cfg_ref_add_h1.zext().asSInt).asSInt
        val thr_dec = (thr.asSInt - cfg_ref_dec_h1.zext().asSInt).asSInt
        val thr_next= Mux(spk, thr_inc, thr_dec)
        val thr_clp = Mux(thr_next > 32767.S, 32767.S, Mux(thr_next < (-32768).S, (-32768).S, thr_next))
        // u leak: 16x16 -> 32, integer 30:15, frac 14:0
        val u_mul  = (u.asSInt * alpha.asSInt)
        val u_int  = u_mul(30,15).asSInt
        val u_frac = u_mul(14,0).asUInt
        val rnd15  = lfsr(14,0)
        val u_leak = Mux(cfg_en_stoch_round && (alpha =/= "h8000".U) && (u_frac >= rnd15) && u_mul(30,15).orR,
                         (u_int + 1.S(16.W)).asSInt, u_int)
        val noise_s = Mux(cfg_noise_en, (lfsr(11,0).zext().asSInt >> cfg_noise_str).asSInt, 0.S)
        val u_next_raw = Mux(spk, (u_leak - thr).asSInt, u_leak) + noise_s
        val u_next     = clampS16(u_next_raw)
        // T_vi decay: 12x16 -> 28, integer 26:15, frac 14:0
        val tvi    = q(95,84)
        val tvi_mul= Cat(0.U(12.W), tvi) * Cat(0.U(0.W), alpha)
        val tvi_int= tvi_mul(26,15)
        val tvi_fr = tvi_mul(14,0)
        val rnd12  = lfsr(11,0)
        val tvi_dec= Mux(cfg_en_stoch_round && (alpha =/= "h8000".U) && (tvi_fr >= rnd12) && tvi_mul(26,15).orR,
                         tvi_int + 1.U, tvi_int)
        val tvi_dec_c = Mux(tvi_dec > "hFFF".U, "hFFF".U, tvi_dec)
        // T_h1: increment on spike then decay
        val th1    = q(83,72)
        val th1_inc= Mux(spk, th1 + 2.U, th1)
        val th1_mul= Cat(0.U(12.W), th1_inc) * Cat(0.U(0.W), alpha)
        val th1_int= th1_mul(26,15)
        val th1_fr = th1_mul(14,0)
        val th1_dec= Mux(cfg_en_stoch_round && (alpha =/= "h8000".U) && (th1_fr >= rnd12) && th1_mul(26,15).orR,
                         th1_int + 1.U, th1_int)
        val th1_dec_c = Mux(th1_dec > "hFFF".U, "hFFF".U, th1_dec)
        // Similarity accumulation for H1 using T_hist/T_prev and STE(u) with 4-tap ring selection
        val depth2   = cfg_tprev_depth(1,0) // reuse low 2 bits for 4-tap selection
        val sel_hist = (h1_hist_wptr(b) + 3.U(2.W) - depth2)(1,0)
        val t_hist   = MuxLookup(sel_hist, q(71,60), Seq(
          1.U -> q(87,76),
          2.U -> q(103,92),
          3.U -> q(119,108)
        ))
        val prev_base= Mux(h1_prev_snap, h1_prev_wptr_snap(b), h1_prev_wptr(b))
        val sel_prev = (prev_base + 3.U(2.W) - depth2)(1,0)
        val t_prev   = MuxLookup(sel_prev, q(59,48), Seq(
          1.U -> q(75,64),
          2.U -> q(91,80),
          3.U -> q(107,96)
        ))
        val ste    = ste_lut(q(79,64)(15,8) ^ 128.U(8.W))
        // Layer-wise similarity: dot-product between trace and spike activity
        val p_pred = (t_hist * spk.asUInt)
        val p_con  = (t_prev * spk.asUInt)
when (do_pred1) { sim_pred_h1 := satAdd32(sim_pred_h1, p_pred) }
when (do_contr1){ sim_contr_h1:= satAdd32(sim_contr_h1, p_con) }
        // REGROW feed: score = |(t_hist - t_prev) * ste| (Q8.8 -> take upper 24b)
        val diff = (t_hist.zext() - t_prev.zext()).asSInt
        val score24 = (diff.asSInt * ste.zext().asSInt).abs().asUInt()(23,0)
        val index9  = Cat(b.U(2.W), h1_step_idx)
      updateRgHeap(score24, index9, b.U, rg_heap_score1, rg_heap_index1, effHeapPerGroup(cfg_prune_k_h1))
        h1_spk_cnt_tick := Mux(h1_spk_cnt_tick === 65535.U, h1_spk_cnt_tick, h1_spk_cnt_tick + spk.asUInt)
        // Snapshots (circular): write hist at current hist wptr; write prev at current prev wptr (on SAMPLE advance)
        val thist_w = th1_dec
        val tprev_w = th1
        val di     = WireDefault(0.U(128.W))
        val m      = WireDefault(0.U(128.W))
        // u
        di = di | (u_next.asUInt << 64)
        m  = m  | ("hFFFF".U(128.W) << 64)
        // thr
        di = di | (thr_clp.asUInt << 96)
        m  = m  | ("hFFFF".U(128.W) << 96)
        // tvi
        di = di | (tvi_dec_c << 84)
        m  = m  | ("hFFF".U(128.W) << 84)
        // th1
        di = di | (th1_dec_c << 72)
        m  = m  | ("hFFF".U(128.W) << 72)
        // thist at slot indicated by hist wptr (12b wide at [60+:12] rotated by 16*sel, 4 taps)
        val hist_off = Cat((h1_hist_wptr(b) + depth2)(1,0), 4.U(4.W))
        di = di | (thist_w << (60.U + hist_off))
        m  = m  | ("hFFF".U(128.W) << (60.U + hist_off))
        // tprev at slot indicated by prev wptr (only when SAMPLE), 4 taps
        when (io.SAMPLE) {
          val prev_off = Cat((h1_prev_wptr(b) + depth2)(1,0), 4.U(4.W))
          di = di | (tprev_w << (48.U + prev_off))
          m  = m  | ("hFFF".U(128.W) << (48.U + prev_off))
        }
        h1(b).io.cs := true.B; h1(b).io.we := true.B; h1(b).io.a := h1_step_idx; h1(b).io.d := di; h1(b).io.m := m
        h1_spk_pending(b) := spk
      }
      // advance index
      when (h1_step_idx === 127.U) {
        h1_step_idx := 0.U
      }.otherwise {
        h1_step_idx := h1_step_idx + 1.U
      }
      // advance circular buffer pointers (mod 4)
      for (b <- 0 until 4) {
        // hist wptr increments every visit
        h1_hist_wptr(b) := Mux(h1_hist_wptr(b) === 3.U, 0.U, (h1_hist_wptr(b) + 1.U))
        // prev wptr increments only when SAMPLE asserted
        when (io.SAMPLE) { h1_prev_wptr(b) := Mux(h1_prev_wptr(b) === 3.U, 0.U, (h1_prev_wptr(b) + 1.U)) }
      }
      h1_step_rw := false.B
    }
  }.otherwise {
    h1_step_rw := false.B
  }

  // H1 prev snapshot window control: launch on SAMPLE falling edge, clear at next STEP1 sweep start
  when (sample_fall) {
    h1_prev_snap := true.B
    for (b <- 0 until 4) { h1_prev_wptr_snap(b) := h1_prev_wptr(b) }
  }
  when (state === State.STEP1 && !h1_step_rw && (h1_step_idx === 0.U)) {
    when (h1_prev_snap) { h1_prev_snap := false.B }
  }

  // H1 spike FIFO (carry 9-bit absolute index: {bank[1:0], row[6:0]})
  val h1_spike_fifo = Module(new Queue(UInt(9.W), 16))
  h1_spike_fifo.io.enq.valid := false.B
  h1_spike_fifo.io.enq.bits  := 0.U
  // Enqueue at most one spike per STEP1 write cycle (lowest bank index priority)
  when (state === State.STEP1 && h1_step_rw) {
    val anySpk = h1_spk_pending.asUInt.orR
    when (anySpk && !h1_spike_fifo.io.enq.valid) {
      val sel = PriorityEncoder(h1_spk_pending.asUInt)
      val row = h1_step_idx
      h1_spike_fifo.io.enq.valid := true.B
      h1_spike_fifo.io.enq.bits  := Cat(sel, row)
      for (b <- 0 until 4) {
        when (sel === b.U) { h1_spk_pending(b) := false.B }
      }
    }
    // Count H1 spikes enqueued
    when (h1_spike_fifo.io.enq.valid && h1_spike_fifo.io.enq.ready) { cnt_spikes_h1 := cnt_spikes_h1 + 1.U }
  }

  // ------------------------------------------------------------------
  // H2 neuron banks (4x 128x128) and simple PROP2/STEP2 handling
  // Optional H2 spike FIFO mirroring SV (not strictly required for current flow)
  val h2_spike_fifo = Module(new Queue(UInt(9.W), 16))
  h2_spike_fifo.io.enq.valid := false.B
  h2_spike_fifo.io.enq.bits  := 0.U
  h2_spike_fifo.io.deq.ready := false.B
  // - PROP2: consume one H1 spike -> READ H2 row at that index
  // - STEP2: WRITE updated u/trace and increment OUT traces on spike
  // ------------------------------------------------------------------
  val h2 = Seq.fill(4)(Module(new Sram128x128))
  for (b <- 0 until 4) {
    h2(b).io.cs := false.B; h2(b).io.we := false.B
    h2(b).io.a  := 0.U;     h2(b).io.d  := 0.U;     h2(b).io.m := 0.U
  }
  // Circular buffer pointers and snapshots for H2 (4 taps) and snapshot window flag
  val h2_hist_wptr  = RegInit(VecInit(Seq.fill(4)(0.U(2.W))))
  val h2_prev_wptr  = RegInit(VecInit(Seq.fill(4)(0.U(2.W))))
  val h2_prev_wptr_snap = RegInit(VecInit(Seq.fill(4)(0.U(2.W))))
  val h2_prev_snap = RegInit(false.B)
  val h2_step_idx = RegInit(0.U(7.W))
  val h2_step_rw  = RegInit(false.B)
  val h2_active   = RegInit(false.B)
  val h2_rw       = RegInit(false.B) // 0=READ, 1=WRITE
  val h2_bank_sel = RegInit(0.U(2.W))
  val h2_row_idx  = RegInit(0.U(7.W))
  when (state === State.PROP2) {
    when (!h2_active && h1_spike_fifo.io.deq.valid && !h2_sp_active) {
      val idx9 = h1_spike_fifo.io.deq.bits
      // Seed H1->H2 sparse burst for this presyn spike
      h2_sp_presyn_bank := idx9(8,7)
      h2_sp_presyn_row  := idx9(6,0)
      h2_sp_active      := true.B
      h2_sp_phase       := 0.U
      h2_sp_word_idx    := 0.U
      h1_spike_fifo.io.deq.ready := true.B
    }.elsewhen (h2_active && h2_rw) {
      // WRITE back update based on read result with decay and stochastic rounding
      for (b <- 0 until 4) {
        when (h2_bank_sel === b.U) {
          val q      = h2(b).io.q
          val u      = q(79,64).asSInt
          val thr    = q(111,96).asSInt
          val alpha  = q(127,112).asUInt
          val spk    = u >= thr
          val u_mul  = (u.asSInt * alpha.asSInt)
          val u_int  = u_mul(30,15).asSInt
          val u_frac = u_mul(14,0).asUInt
          val rnd15  = lfsr(14,0)
          val u_leak = Mux(cfg_en_stoch_round && (alpha =/= "h8000".U) && (u_frac >= rnd15) && u_mul(30,15).orR,
                           (u_int + 1.S(16.W)).asSInt, u_int)
          val u_next_raw = Mux(spk, (u_leak - thr).asSInt, u_leak)
          val u_next     = clampS16(u_next_raw)
          val th1    = q(83,72)
          val th1_inc= Mux(spk, th1 + 2.U, th1)
          val th1_mul= Cat(0.U(12.W), th1_inc) * Cat(0.U(0.W), alpha)
          val th1_int= th1_mul(26,15)
          val th1_fr = th1_mul(14,0)
          val rnd12  = lfsr(11,0)
          val th1_dec= Mux(cfg_en_stoch_round && (alpha =/= "h8000".U) && (th1_fr >= rnd12) && th1_mul(26,15).orR,
                           th1_int + 1.U, th1_int)
          val th1_dec_c = Mux(th1_dec > "hFFF".U, "hFFF".U, th1_dec)
          val di     = WireDefault(0.U(128.W))
          val m      = WireDefault(0.U(128.W))
          di = di | (u_next.asUInt << 64); m = m | ("hFFFF".U(128.W) << 64)
          di = di | (th1_dec_c << 72);      m = m | ("hFFF".U(128.W) << 72)
          // Similarity accumulation for H2
          val t_hist2 = q(71,60)
          val t_prev2 = q(59,48)
          val ste2    = ste_lut(q(79,64)(15,8) ^ 128.U(8.W))
          // Layer-wise similarity: dot-product between trace and spike activity
          val p_pred2 = (t_hist2 * spk.asUInt)
          val p_con2  = (t_prev2 * spk.asUInt)
when (do_pred2) { sim_pred_h2 := satAdd32(sim_pred_h2, p_pred2) }
when (do_contr2){ sim_contr_h2:= satAdd32(sim_contr_h2, p_con2) }
          h2_spk_cnt_tick := Mux(h2_spk_cnt_tick === 65535.U, h2_spk_cnt_tick, h2_spk_cnt_tick + spk.asUInt)
          // thist ring (4 taps)
          val depth2_2 = cfg_tprev_depth(1,0)
          val hist_off = Cat((h2_hist_wptr(b) + depth2_2)(1,0), 4.U(4.W))
          di = di | (th1_dec_c << (60.U + hist_off))
          m  = m  | ("hFFF".U(128.W) << (60.U + hist_off))
          // tprev ring on SAMPLE
          when (io.SAMPLE) {
            val prev_off = Cat((h2_prev_wptr(b) + depth2_2)(1,0), 4.U(4.W))
            di = di | (q(83,72) << (48.U + prev_off))
            m  = m  | ("hFFF".U(128.W) << (48.U + prev_off))
          }
          h2(b).io.cs := true.B; h2(b).io.we := true.B; h2(b).io.a := h2_row_idx; h2(b).io.d := di; h2(b).io.m := m
          when (spk) {
            // Increment output accumulation bucket or oneur integrator
            val idx = (h2_row_idx(M_OUT-1,0) % (cfg_num_out_neur + 1.U))(M_OUT-1,0)
            when (!cfg_use_oneur) {
            val curr = outCnt(idx)
            outCnt(idx) := curr + 1.U
            val trc = outTrace(idx)
            val inc = 8.U(16.W)
            outTrace(idx) := Mux(trc > ("hFFFF".U(16.W) - inc), "hFFFF".U, trc + inc)
            }.otherwise {
              // Read output weights row and add selected byte to oneur_u
              ctrl_wout_cs := true.B; ctrl_wout_we := false.B; ctrl_wout_a := Cat(0.U(5.W), h2_row_idx)
              val rowW = weightsOut.io.q
              val w8  = rowW(8*idx+7, 8*idx).asSInt
              val sum = oneur_u(idx) + w8
              val sat = Mux(sum > 32767.S, 32767.S, Mux(sum < (-32768).S, (-32768).S, sum))
              oneur_u(idx) := sat
            }
            // Accumulate presyn trace into lane bucket for three-factor OL
            val lane = h2_row_idx(3,0)
            val preOld = preOutLane(lane)
            val preAdd = th1_dec_c // 12-bit decayed trace for H2 neuron
            val preSum = preOld + preAdd
            preOutLane(lane) := Mux(preSum > 4095.U, 4095.U, preSum)
            // Enqueue H2 spike absolute index
            h2_spike_fifo.io.enq.valid := true.B
            h2_spike_fifo.io.enq.bits  := Cat(h2_bank_sel, h2_row_idx)
          }
        }
      }
      // Count H2 spikes processed
      when (h2_active && h2_rw) { cnt_spikes_h2 := cnt_spikes_h2 + 1.U }
      h2_active := false.B
      h2_rw     := false.B
    }.otherwise {
      h1_spike_fifo.io.deq.ready := false.B
      for (b <- 0 until 4) { h2(b).io.cs := false.B }
    }
  }.otherwise {
    h1_spike_fifo.io.deq.ready := false.B
    h2_active := false.B
    h2_rw     := false.B
  }

  // H1->H2 sparse burst micro-FSM (PROP2): read sp, then READ H2, then RMW H2 u per word
  val h2_sp_active      = RegInit(false.B)
  val h2_sp_phase       = RegInit(0.U(2.W)) // 0=sp read, 1=H2 read, 2=RMW H2
  val h2_sp_word_idx    = RegInit(0.U(3.W))
  val h2_sp_presyn_bank = RegInit(0.U(2.W))
  val h2_sp_presyn_row  = RegInit(0.U(7.W))
  val h2_sp_weight8     = RegInit(0.U(8.W))
  val h2_sp_j_idx       = RegInit(0.U(9.W))
  val h2_sp_group       = RegInit(0.U(2.W))
  val h2_sp_valid       = RegInit(false.B)
  when (state === State.PROP2 && h2_sp_active) {
    switch (h2_sp_phase) {
      is (0.U) { // sp read
        val sp_addr2 = Cat(h2_sp_presyn_bank, h2_sp_presyn_row, h2_sp_word_idx)
        sp_h1_h2.io.cs := true.B; sp_h1_h2.io.we := false.B; sp_h1_h2.io.a := sp_addr2
        h2_sp_phase := 1.U
      }
      is (1.U) { // capture sp and issue H2 read
        when (sp_h1_h2_do_valid) {
          val spq = sp_h1_h2.io.q
          val weight8 = spq(7,0)
          val j_idx   = spq(16,8)
          val group   = spq(18,17)
          val valid   = spq(22)
          h2_sp_weight8 := weight8
          h2_sp_j_idx   := j_idx
          h2_sp_group   := group
          h2_sp_valid   := valid
          val bank2 = j_idx(8,7)
          val row2  = j_idx(6,0)
          for (b <- 0 until 4) {
            when (bank2 === b.U) { h2(b).io.cs := true.B; h2(b).io.we := false.B; h2(b).io.a := row2 }
          }
          h2_sp_phase := 2.U
        }
      }
      is (2.U) { // RMW H2 u += weight and write back (if valid and group matches)
        val bank2 = h2_sp_j_idx(8,7)
        val row2  = h2_sp_j_idx(6,0)
        when (h2_sp_valid && (bank2 === h2_sp_group)) {
          for (b <- 0 until 4) {
            when (bank2 === b.U) {
              val q   = h2(b).io.q
              val u16 = q(79,64)
              val sum = u16 + h2_sp_weight8
              val sat = Mux(sum < u16, "hFFFF".U(16.W), sum)
              val di  = (sat << 64)
              val m   = ("hFFFF".U(128.W) << 64)
              h2(b).io.cs := true.B; h2(b).io.we := true.B; h2(b).io.a := row2; h2(b).io.d := di; h2(b).io.m := m
            }
          }
        }
        val last2 = (h2_sp_word_idx === (cfg_words_per_pre_h2 - 1.U))
        when (last2) { h2_sp_active := false.B; h2_sp_phase := 0.U }
          .otherwise { h2_sp_word_idx := h2_sp_word_idx + 1.U; h2_sp_phase := 0.U }
      }
    }
  }.otherwise {
    h2_sp_active := false.B
  }

  // ------------------------------------------------------------------
  // VI trace increment micro-op controller (PROP1): READ then RMW on H1
  // Avoid conflict with STEP1 by operating only in PROP1; uses T_vi field [95:84]
  // ------------------------------------------------------------------
  val vi_inc_pending   = RegInit(false.B)
  val vi_inc_doing     = RegInit(false.B)
  val vi_inc_bank_sel  = RegInit(0.U(2.W))
  val vi_inc_row_idx   = RegInit(0.U(7.W))
  // VI->H1 sparse burst (multi-word) state
  val vi_sp_active     = RegInit(false.B)
  val vi_sp_phase      = RegInit(0.U(2.W)) // 0=sp read, 1=H1 read, 2=RMW H1
  val vi_sp_word_idx   = RegInit(0.U(3.W))
  val vi_sp_weight8    = RegInit(0.U(8.W))
  val vi_sp_j_idx      = RegInit(0.U(9.W))
  val vi_sp_group      = RegInit(0.U(2.W))
  val vi_sp_valid      = RegInit(false.B)
  when (state === State.PROP1) {
    when (vi_inc_pending && !vi_inc_doing) {
      // issue READ
      for (b <- 0 until 4) {
        when (vi_inc_bank_sel === b.U) { h1(b).io.cs := true.B; h1(b).io.we := false.B; h1(b).io.a := vi_inc_row_idx }
      }
      vi_inc_doing := true.B
    }.elsewhen (vi_inc_pending && vi_inc_doing) {
      // perform RMW: increment T_vi by 1 (limited to 12b), write back masked into [95:84]
      // Gate on H1 DO-valid (emulate one-cycle read latency)
      val h1_do_valid = true.B // SRAMS already 1-cyc pipelined; treat as valid this cycle
      when (h1_do_valid) {
        for (b <- 0 until 4) {
          when (vi_inc_bank_sel === b.U) {
            val q   = h1(b).io.q
            val tvi = q(95,84)
            val tvi_inc = Mux(tvi === "hFFF".U, tvi, tvi + 1.U)
            val di  = (tvi_inc << 84)
            val m   = ("hFFF".U(128.W) << 84)
            h1(b).io.cs := true.B; h1(b).io.we := true.B; h1(b).io.a := vi_inc_row_idx; h1(b).io.d := di; h1(b).io.m := m
          }
        }
        vi_inc_pending := false.B
        vi_inc_doing   := false.B
      }
      // Start VI->H1 sparse burst for this presyn
      vi_sp_active   := true.B
      vi_sp_phase    := 0.U
      vi_sp_word_idx := 0.U
    }
    // VI->H1 sparse burst FSM
    when (vi_sp_active) {
      switch (vi_sp_phase) {
        is (0.U) { // issue sp read
          val sp_addr = Cat(vi_inc_bank_sel, vi_inc_row_idx, vi_sp_word_idx)
          sp_vi_h1.io.cs := true.B; sp_vi_h1.io.we := false.B; sp_vi_h1.io.a := sp_addr
          vi_sp_phase := 1.U
        }
        is (1.U) { // capture sp and issue H1 read
          when (sp_vi_h1_do_valid) {
            val spq = sp_vi_h1.io.q
            val weight8 = spq(7,0)
            val j_idx   = spq(16,8)
            val group   = spq(18,17)
            val valid   = spq(22)
            vi_sp_weight8 := weight8
            vi_sp_j_idx   := j_idx
            vi_sp_group   := group
            vi_sp_valid   := valid
            val bank2 = j_idx(8,7)
            val row2  = j_idx(6,0)
            for (b <- 0 until 4) {
              when (bank2 === b.U) { h1(b).io.cs := true.B; h1(b).io.we := false.B; h1(b).io.a := row2 }
            }
            vi_sp_phase := 2.U
          }
        }
        is (2.U) { // RMW H1 u += weight when valid and group matches
          val bank2 = vi_sp_j_idx(8,7)
          val row2  = vi_sp_j_idx(6,0)
          when (vi_sp_valid && (bank2 === vi_sp_group)) {
            for (b <- 0 until 4) {
              when (bank2 === b.U) {
                val q   = h1(b).io.q
                val u16 = q(79,64)
                val sum = u16 + vi_sp_weight8
                val sat = Mux(sum < u16, "hFFFF".U(16.W), sum)
                val di  = (sat << 64)
                val m   = ("hFFFF".U(128.W) << 64)
                h1(b).io.cs := true.B; h1(b).io.we := true.B; h1(b).io.a := row2; h1(b).io.d := di; h1(b).io.m := m
              }
            }
          }
          val last = (vi_sp_word_idx === (cfg_words_per_pre_h1 - 1.U))
          when (last) { vi_sp_active := false.B; vi_sp_phase := 0.U }
            .otherwise { vi_sp_word_idx := vi_sp_word_idx + 1.U; vi_sp_phase := 0.U }
        }
      }
    }
  }.otherwise {
    vi_inc_pending := false.B
    vi_inc_doing   := false.B
    vi_sp_active   := false.B
  }

  // Advance H2 circular pointers after STEP2 write (mod 4)
  when (state === State.STEP2 && h2_active === false.B && h2_rw === false.B) {
    for (b <- 0 until 4) {
      h2_hist_wptr(b) := Mux(h2_hist_wptr(b) === 3.U, 0.U, (h2_hist_wptr(b) + 1.U))
      when (io.SAMPLE) { h2_prev_wptr(b) := Mux(h2_prev_wptr(b) === 3.U, 0.U, (h2_prev_wptr(b) + 1.U)) }
    }
  }
  // H2 prev snapshot window: launch on SAMPLE falling edge; clear at next STEP2 sweep start
  when (sample_fall) {
    h2_prev_snap := true.B
    for (b <- 0 until 4) { h2_prev_wptr_snap(b) := h2_prev_wptr(b) }
  }
  when (state === State.STEP2 && !h2_rw && (h2_step_idx === 0.U)) {
    when (h2_prev_snap) { h2_prev_snap := false.B }
  }

  // ------------------------------------------------------------------
  // SRAM arbitration: SPI vs controller (SPI has priority when active)
  // ------------------------------------------------------------------
  // SPI-side intents
  val spi_neur_cs = WireDefault(false.B)
  val spi_neur_we = WireDefault(false.B)
  val spi_neur_a  = WireDefault(0.U(7.W))
  val spi_neur_d  = WireDefault(0.U(128.W))
  val spi_neur_m  = WireDefault(0.U(128.W))

  val spi_winp_cs = WireDefault(false.B)
  val spi_winp_we = WireDefault(false.B)
  val spi_winp_a  = WireDefault(0.U(12.W))
  val spi_winp_d  = WireDefault(0.U(128.W))
  val spi_winp_m  = WireDefault(0.U(128.W))

  val spi_wrec_cs = WireDefault(false.B)
  val spi_wrec_we = WireDefault(false.B)
  val spi_wrec_a  = WireDefault(0.U(12.W))
  val spi_wrec_d  = WireDefault(0.U(128.W))
  val spi_wrec_m  = WireDefault(0.U(128.W))

  val spi_wout_cs = WireDefault(false.B)
  val spi_wout_we = WireDefault(false.B)
  val spi_wout_a  = WireDefault(0.U(9.W))
  val spi_wout_d  = WireDefault(0.U(128.W))
  val spi_wout_m  = WireDefault(0.U(128.W))

  // Controller-side intents (placeholders for now)
  val ctrl_neur_cs = WireDefault(false.B)
  val ctrl_neur_we = WireDefault(false.B)
  val ctrl_neur_a  = WireDefault(0.U(7.W))
  val ctrl_neur_d  = WireDefault(0.U(128.W))
  val ctrl_neur_m  = WireDefault(0.U(128.W))

  val ctrl_winp_cs = WireDefault(false.B)
  val ctrl_winp_we = WireDefault(false.B)
  val ctrl_winp_a  = WireDefault(0.U(12.W))
  val ctrl_winp_d  = WireDefault(0.U(128.W))
  val ctrl_winp_m  = WireDefault(0.U(128.W))

  val ctrl_wrec_cs = WireDefault(false.B)
  val ctrl_wrec_we = WireDefault(false.B)
  val ctrl_wrec_a  = WireDefault(0.U(12.W))
  val ctrl_wrec_d  = WireDefault(0.U(128.W))
  val ctrl_wrec_m  = WireDefault(0.U(128.W))

  val ctrl_wout_cs = WireDefault(false.B)
  val ctrl_wout_we = WireDefault(false.B)
  val ctrl_wout_a  = WireDefault(0.U(9.W))
  val ctrl_wout_d  = WireDefault(0.U(128.W))
  val ctrl_wout_m  = WireDefault(0.U(128.W))

  // Controller-side state for weight readback during PROP/STEP
  val ctrl_winp_pending = RegInit(false.B)
  val ctrl_wrec_pending = RegInit(false.B)
  val ctrl_neur_pending = RegInit(false.B)
  val last_presyn       = RegInit(0.U(8.W))
  val last_presyn_vld   = RegInit(false.B)
  val ctrl_last_idx     = RegInit(0.U(M_OUT.W))

  // Drive wrappers with SPI intents if asserted, otherwise controller intents
  neurons.io.cs := Mux(spi_neur_cs, true.B, ctrl_neur_cs)
  neurons.io.we := Mux(spi_neur_cs, spi_neur_we, ctrl_neur_we)
  neurons.io.a  := Mux(spi_neur_cs, spi_neur_a,  ctrl_neur_a)
  neurons.io.d  := Mux(spi_neur_cs, spi_neur_d,  ctrl_neur_d)
  neurons.io.m  := Mux(spi_neur_cs, spi_neur_m,  ctrl_neur_m)

  weightsIn.io.cs := Mux(spi_winp_cs, true.B, ctrl_winp_cs)
  weightsIn.io.we := Mux(spi_winp_cs, spi_winp_we, ctrl_winp_we)
  weightsIn.io.a  := Mux(spi_winp_cs, spi_winp_a,  ctrl_winp_a)
  weightsIn.io.d  := Mux(spi_winp_cs, spi_winp_d,  ctrl_winp_d)
  weightsIn.io.m  := Mux(spi_winp_cs, spi_winp_m,  ctrl_winp_m)

  weightsRec.io.cs := Mux(spi_wrec_cs, true.B, ctrl_wrec_cs)
  weightsRec.io.we := Mux(spi_wrec_cs, spi_wrec_we, ctrl_wrec_we)
  weightsRec.io.a  := Mux(spi_wrec_cs, spi_wrec_a,  ctrl_wrec_a)
  weightsRec.io.d  := Mux(spi_wrec_cs, spi_wrec_d,  ctrl_wrec_d)
  weightsRec.io.m  := Mux(spi_wrec_cs, spi_wrec_m,  ctrl_wrec_m)

  weightsOut.io.cs := Mux(spi_wout_cs, true.B, ctrl_wout_cs)
  weightsOut.io.we := Mux(spi_wout_cs, spi_wout_we, ctrl_wout_we)
  weightsOut.io.a  := Mux(spi_wout_cs, spi_wout_a,  ctrl_wout_a)
  weightsOut.io.d  := Mux(spi_wout_cs, spi_wout_d,  ctrl_wout_d)
  weightsOut.io.m  := Mux(spi_wout_cs, spi_wout_m,  ctrl_wout_m)

  when (state === State.STEP2) {
    for (b <- 0 until 4) {
      val q      = h2(b).io.q
      val u      = q(79,64)
      val ste    = ste_lut(u(15,8) ^ 128.U(8.W))
      val t_hist = q(71,60)
      val t_prev = q(59,48)
      val diff   = (t_hist.zext() - t_prev.zext()).asSInt
      val score24= (diff.asSInt * ste.zext().asSInt).abs().asUInt()(23,0)
      val index9 = Cat(b.U(2.W), h2_step_idx)
      updateRgHeap(score24, index9, b.U, rg_heap_score2, rg_heap_index2, effHeapPerGroup(cfg_prune_k_h2))
      // refractory/homeostasis for H2 thr
      val thr2   = q(111,96).asSInt
      val spk2   = u.asSInt >= thr2
      val thr2_inc = (thr2 + cfg_ref_add_h2.zext().asSInt).asSInt
      val thr2_dec = (thr2 - cfg_ref_dec_h2.zext().asSInt).asSInt
      val thr2_next= Mux(spk2, thr2_inc, thr2_dec)
      val thr2_clp = Mux(thr2_next > 32767.S, 32767.S, Mux(thr2_next < (-32768).S, (-32768).S, thr2_next))
      val di2_thr  = (thr2_clp.asUInt << 96)
      val m2_thr   = ("hFFFF".U(128.W) << 96)
      h2(b).io.cs := true.B; h2(b).io.we := true.B; h2(b).io.a := h2_step_idx; h2(b).io.d := di2_thr; h2(b).io.m := m2_thr
    }
  }
  // Clear last_presyn_vld at LEARN exit to avoid stale presyn driving next tick
  when (state === State.SEND && nextState === State.IDLE) {
    last_presyn_vld := false.B
  }
  // Also clear on new SAMPLE rise to avoid carrying presyn across samples
  when (sample_rise) { last_presyn_vld := false.B }
}


