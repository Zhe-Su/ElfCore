package elfcore

import chisel3._
import chisel3.util._

// Shared PSP IO bundle
class PSPIO(val BW: Int) extends Bundle {
  // Parallel->Serial path (PS)
  val ReqIn_PS   = Input(Bool())
  val AckIn_PS   = Output(Bool())
  val DataIn_PS  = Input(UInt(BW.W))
  val d_m_setup  = Input(UInt(2.W))
  val d_m_hold   = Input(UInt(2.W))
  val d_ps_hold  = Input(UInt(2.W))
  val ReqOut_PS  = Output(Bool())
  val AckOut_PS  = Input(Bool())
  val DataOut_PS = Output(Bool())

  // Serial->Parallel path (SP)
  val ReqIn_SP   = Input(Bool())
  val AckIn_SP   = Output(Bool())
  val DataIn_SP  = Input(Bool())
  val ReqOut_SP  = Output(Bool())
  val AckOut_SP  = Input(Bool())
  val DataOut_SP = Output(UInt(BW.W))

  // Reset
  val rst        = Input(Bool())
}

// BlackBox for PSP (Parallel<->Serial bridge)
class PSPBlackBox(val BW: Int) extends BlackBox(Map("BW" -> IntParam(BW))) with HasBlackBoxPath {
  val io = IO(new PSPIO(BW))
  // Point to real RTL files
  addPath("/home/zhesu/Open_Source_Digital/ElfCore/async_serdes/specs.v")
  addPath("/home/zhesu/Open_Source_Digital/ElfCore/async_serdes/PtoS.v")
  addPath("/home/zhesu/Open_Source_Digital/ElfCore/async_serdes/StoP.v")
  addPath("/home/zhesu/Open_Source_Digital/ElfCore/async_serdes/PSP_submodels.v")
  addPath("/home/zhesu/Open_Source_Digital/ElfCore/async_serdes/PSP.v")
}

// Simple Scala PSP model for simulation only (functional, single-clock)
class PSPModel(val BW: Int) extends Module {
  val io = IO(new PSPIO(BW))

  // Defaults
  io.AckIn_PS   := io.AckOut_PS
  io.ReqOut_PS  := io.ReqIn_PS
  io.DataOut_PS := io.DataIn_PS(0)

  io.AckIn_SP   := io.AckOut_SP
  io.ReqOut_SP  := io.ReqIn_SP
  // Place serial input into LSB and zero-extend (functional placeholder)
  io.DataOut_SP := Cat(0.U((BW-1).W), io.DataIn_SP.asUInt)
}

// BlackBox for ELFCORE_core (functional core)
class ELFCoreBlackBox(val M_INP: Int, val M_OUT: Int) extends BlackBox {
  val io = IO(new Bundle {
    val clk  = Input(Clock())
    val rst  = Input(Bool())

    // SPI slave interface
    val SCK  = Input(Bool())
    val MOSI = Input(Bool())
    val MISO = Output(Bool())

    // AER virtual input
    val AERIN_ADDR = Input(UInt(M_INP.W))
    val AERIN_REQ  = Input(Bool())
    val AERIN_ACK  = Output(Bool())

    // Timestep control
    val SAMPLE     = Input(Bool())
    val TIME_TICK  = Input(Bool())
    val INFER_ACC  = Input(Bool())

    // Serialized output bus as AER source
    val OUT_REQ    = Output(Bool())
    val OUT_ACK    = Input(Bool())
    val OUT_DATA   = Output(UInt(8.W))

    // Label inputs
    val LABEL_VALID= Input(Bool())
    val LABEL_INDEX= Input(UInt(M_OUT.W))

    // Diagnostics
    val SPI_RDY          = Output(Bool())
    val TIMING_ERROR_RDY = Output(Bool())
  })
}

// Chisel top-level equivalent of Verilog ELFCORE
class ELFCOREChisel(
  val N_INP: Int = 256,
  val N_H1:  Int = 512,
  val N_H2:  Int = 512,
  val N_OUT: Int = 16,
  val LOCAL_TICK_EN: Boolean = false,
  val CYCLES_PER_TICK: Int = 1000,
  val USE_OUT_BUS_AS_AER: Boolean = true,
  val STOP_DELAY_CYCLES: Int = 0,
  val USE_VERILOG_PSP: Boolean = true
) extends Module {
  // Derived widths
  private val M_INP = Math.max(1, log2Ceil(N_INP))
  private val M_OUT = Math.max(1, log2Ceil(N_OUT))
  private val PSP_BW = Math.max(M_INP, M_OUT)

  val io = IO(new Bundle {
    // Global
    val clk   = Input(Clock())
    val rst   = Input(Bool())

    // SPI
    val SCK   = Input(Bool())
    val MOSI  = Input(Bool())
    val MISO  = Output(Bool())

    // Timestep control
    val SAMPLE    = Input(Bool())
    val TIME_TICK = Input(Bool())
    val INFER_ACC = Input(Bool())

    // Serial AER input (StoP)
    val AER_SER_IN_REQ  = Input(Bool())
    val AER_SER_IN_ACK  = Output(Bool())
    val AER_SER_IN_DATA = Input(Bool())

    // Serial AER output (PtoS)
    val AER_SER_OUT_REQ  = Output(Bool())
    val AER_SER_OUT_ACK  = Input(Bool())
    val AER_SER_OUT_DATA = Output(Bool())

    // Supervised training inputs
    val LABEL_VALID = Input(Bool())
    val LABEL_INDEX = Input(UInt(M_OUT.W))

    // Diagnostics
    val SPI_RDY          = Output(Bool())
    val TIMING_ERROR_RDY = Output(Bool())
  })

  // Default clock/reset domain
  withClockAndReset(io.clk, io.rst) {
    // ------------------------------------------------------------------
    // Local/external tick mux
    // ------------------------------------------------------------------
    val tick_cnt = RegInit(0.U(32.W))
    val local_tick = WireDefault(false.B)
    when (LOCAL_TICK_EN.B) {
      tick_cnt := Mux(tick_cnt === (CYCLES_PER_TICK-1).U, 0.U, tick_cnt + 1.U)
      local_tick := tick_cnt === (CYCLES_PER_TICK-1).U
    }
    val eff_time_tick = Mux(LOCAL_TICK_EN.B, local_tick, io.TIME_TICK)

    // ------------------------------------------------------------------
    // Core instance (Chisel implementation)
    // ------------------------------------------------------------------
    val core = Module(new ELFCORECoreChisel(N_INP, N_H1, N_H2, N_OUT))
    core.io.SCK := io.SCK
    core.io.MOSI:= io.MOSI
    io.MISO     := core.io.MISO

    core.io.SAMPLE    := io.SAMPLE
    core.io.TIME_TICK := eff_time_tick
    core.io.INFER_ACC := io.INFER_ACC

    core.io.LABEL_VALID := io.LABEL_VALID
    core.io.LABEL_INDEX := io.LABEL_INDEX

    io.SPI_RDY          := core.io.SPI_RDY
    io.TIMING_ERROR_RDY := core.io.TIMING_ERROR_RDY

    // ------------------------------------------------------------------
    // PSP instance
    // ------------------------------------------------------------------
    val psp = if (USE_VERILOG_PSP) Module(new PSPBlackBox(PSP_BW)) else Module(new PSPModel(PSP_BW))
    // Delay knobs = 0 (fastest)
    psp.io.d_m_setup := 0.U
    psp.io.d_m_hold  := 0.U
    psp.io.d_ps_hold := 0.U
    psp.io.rst       := io.rst

    // Serial pads connection
    psp.io.ReqIn_SP  := io.AER_SER_IN_REQ
    io.AER_SER_IN_ACK:= psp.io.AckIn_SP
    psp.io.DataIn_SP := io.AER_SER_IN_DATA

    io.AER_SER_OUT_REQ  := psp.io.ReqOut_PS
    psp.io.AckOut_PS    := io.AER_SER_OUT_ACK
    io.AER_SER_OUT_DATA := psp.io.DataOut_PS

    // ------------------------------------------------------------------
    // CDC: StoP (SP) into core with 3-FF sync and optional delay
    // ------------------------------------------------------------------
    val sp_req_meta1 = RegInit(false.B)
    val sp_req_meta2 = RegInit(false.B)
    val sp_req_meta3 = RegInit(false.B)
    sp_req_meta1 := psp.io.ReqOut_SP
    sp_req_meta2 := sp_req_meta1
    sp_req_meta3 := sp_req_meta2
    val sp_req_edge = sp_req_meta2 ^ sp_req_meta3

    val sp_pipe = RegInit(0.U(4.W))
    sp_pipe := Cat(sp_pipe(2,0), sp_req_edge.asUInt)
    val sp_req_edge_delayed = sp_pipe(STOP_DELAY_CYCLES)

    // 4-slot FIFO between PSP StoP and core
    val sp_fifo_mem = Reg(Vec(4, UInt(M_INP.W)))
    val sp_fifo_wr_ptr = RegInit(0.U(2.W))
    val sp_fifo_rd_ptr = RegInit(0.U(2.W))
    val sp_fifo_count  = RegInit(0.U(3.W)) // 0..4
    val sp_fifo_full   = sp_fifo_count === 4.U
    val sp_fifo_empty  = sp_fifo_count === 0.U
    val sp_req_busy    = RegInit(false.B)

    val sff_AERIN_ADDR = RegInit(0.U(M_INP.W))
    val sff_AERIN_REQ  = RegInit(false.B)
    val sff_AERIN_ACK  = Wire(Bool())
    sff_AERIN_ACK := core.io.AERIN_ACK

    // Ack toggle back to PSP when enqueue accepted
    val psp_AckOut_SP_tgl = RegInit(false.B)
    psp.io.AckOut_SP := psp_AckOut_SP_tgl

    // Connect to core AER inputs
    core.io.AERIN_ADDR := sff_AERIN_ADDR
    core.io.AERIN_REQ  := sff_AERIN_REQ

    // Enqueue from PSP edge if space
    when (sp_req_edge_delayed && !sp_fifo_full) {
      sp_fifo_mem(sp_fifo_wr_ptr) := psp.io.DataOut_SP(M_INP-1, 0)
      sp_fifo_wr_ptr := sp_fifo_wr_ptr + 1.U
      sp_fifo_count  := sp_fifo_count + 1.U
      psp_AckOut_SP_tgl := ~psp_AckOut_SP_tgl
    }

    // Default deassert Req
    sff_AERIN_REQ := false.B

    // Dequeue: present one item per cycle; pop on ACK
    when (!sp_fifo_empty && !sp_req_busy) {
      sff_AERIN_ADDR := sp_fifo_mem(sp_fifo_rd_ptr)
      sff_AERIN_REQ  := true.B
      sp_req_busy    := true.B
    }
    when (sp_req_busy && sff_AERIN_ACK) {
      sp_fifo_rd_ptr := sp_fifo_rd_ptr + 1.U
      sp_fifo_count  := sp_fifo_count - 1.U
      sp_req_busy    := false.B
    }

    // ------------------------------------------------------------------
    // CDC: core to PSP (PtoS)
    // ------------------------------------------------------------------
    val sff_OUT_REQ  = core.io.OUT_REQ
    val sff_OUT_ACK  = RegInit(false.B)
    val sff_OUT_DATA = core.io.OUT_DATA
    core.io.OUT_ACK  := sff_OUT_ACK

    val psp_DataIn_PS    = RegInit(0.U(PSP_BW.W))
    val psp_ReqIn_PS_tgl = RegInit(false.B)
    psp.io.DataIn_PS := psp_DataIn_PS
    psp.io.ReqIn_PS  := psp_ReqIn_PS_tgl

    val psp_AckIn_PS = Wire(Bool())
    psp_AckIn_PS := psp.io.AckIn_PS

    val ack_ps_meta1 = RegInit(false.B)
    val ack_ps_meta2 = RegInit(false.B)
    val ack_ps_meta3 = RegInit(false.B)
    ack_ps_meta1 := psp_AckIn_PS
    ack_ps_meta2 := ack_ps_meta1
    ack_ps_meta3 := ack_ps_meta2

    val ptos_pending = RegInit(false.B)
    sff_OUT_ACK := false.B

    when (USE_OUT_BUS_AS_AER.B && sff_OUT_REQ && !ptos_pending) {
      val nextPS = WireDefault(0.U(PSP_BW.W))
      val lo = M_OUT min 8
      nextPS := 0.U
      // Place OUT_DATA's lower M_OUT bits into PS payload LSBs
      nextPS := Cat(0.U((PSP_BW - lo).W), sff_OUT_DATA(lo-1, 0))
      psp_DataIn_PS := nextPS
      psp_ReqIn_PS_tgl := ~psp_ReqIn_PS_tgl
      ptos_pending := true.B
    }

    when (ptos_pending && (ack_ps_meta2 ^ ack_ps_meta3)) {
      sff_OUT_ACK := true.B
      ptos_pending := false.B
    }
  }
}


