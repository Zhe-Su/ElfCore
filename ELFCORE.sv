`timescale 1ns/1ps
`include "async_serdes/specs.v"

// Bridge ELFCORE AER to async SERDES via a single PSP instance (StoP+PtoS)
// Carefully handles CDC between async PSP and sync ELFCORE using edge-sync toggles.

module ELFCORE #(
    parameter int unsigned N_INP   = 256,
    parameter int unsigned N_H1    = 512,
    parameter int unsigned N_H2    = 512,
    parameter int unsigned N_OUT   = 16,
    parameter bit          LOCAL_TICK_EN     = 1'b0,
    parameter int unsigned CYCLES_PER_TICK   = 1000,
    // Map ELFCORE OUT_* as AER source for PtoS
    parameter bit          USE_OUT_BUS_AS_AER = 1'b1,
    // Configurable CDC pulse delay for StoP->SFF (0..3 cycles)
    parameter logic [1:0]  STOP_DELAY_CYCLES  = 2'd0
) (
    input  logic                 clk,
    input  logic                 rst,

    // SPI slave interface (passed to ELFCORE)
    input  logic                 SCK,
    input  logic                 MOSI,
    output logic                 MISO,

    // Timestep control
    input  logic                 SAMPLE,
    input  logic                 TIME_TICK,
    input  logic                 INFER_ACC,

    // Serial AER input pads (into ELFCORE via PSP StoP)
    input  logic                 AER_SER_IN_REQ,
    output logic                 AER_SER_IN_ACK,
    input  logic                 AER_SER_IN_DATA,

    // Serial AER output pads (from ELFCORE via PSP PtoS)
    output logic                 AER_SER_OUT_REQ,
    input  logic                 AER_SER_OUT_ACK,
    output logic                 AER_SER_OUT_DATA
);

    localparam int unsigned M_INP  = (N_INP  <= 1) ? 1 : $clog2(N_INP);
    localparam int unsigned M_OUT  = (N_OUT  <= 1) ? 1 : $clog2(N_OUT);
    localparam int unsigned PSP_BW = (M_INP > M_OUT) ? M_INP : M_OUT;

    // ------------------------------------------------------------------
    // ELFCORE instance
    // ------------------------------------------------------------------
    logic [M_INP-1:0] sff_AERIN_ADDR;
    logic             sff_AERIN_REQ;
    logic             sff_AERIN_ACK;

    logic             sff_OUT_REQ;
    logic             sff_OUT_ACK;
    logic [7:0]       sff_OUT_DATA;

    logic             SPI_RDY, TIMING_ERROR_RDY;

    ELFCORE_core #(
        .N_INP(N_INP), .N_H1(N_H1), .N_H2(N_H2), .N_OUT(N_OUT),
        .LOCAL_TICK_EN(LOCAL_TICK_EN), .CYCLES_PER_TICK(CYCLES_PER_TICK)
    ) u_sff (
        .clk(clk), .rst(rst),
        .SCK(SCK), .MOSI(MOSI), .MISO(MISO),
        .AERIN_ADDR(sff_AERIN_ADDR),
        .AERIN_REQ (sff_AERIN_REQ),
        .AERIN_ACK (sff_AERIN_ACK),
        .SAMPLE(SAMPLE), .TIME_TICK(TIME_TICK),
        .INFER_ACC(INFER_ACC),
        .OUT_REQ(sff_OUT_REQ), .OUT_ACK(sff_OUT_ACK), .OUT_DATA(sff_OUT_DATA),
        .LABEL_VALID(1'b0), .LABEL_INDEX('0),
        .SPI_RDY(SPI_RDY), .TIMING_ERROR_RDY(TIMING_ERROR_RDY)
    );

    // ------------------------------------------------------------------
    // PSP instance (both StoP and PtoS)
    // ------------------------------------------------------------------
    logic [PSP_BW-1:0] psp_DataOut_SP;
    logic              psp_ReqOut_SP;
    logic              psp_AckIn_SP;   // to pad (ack back to serial source)

    logic              psp_ReqOut_PS;  // to pad (req to serial sink)
    logic              psp_DataOut_PS; // to pad (serial data)
    logic              psp_AckIn_PS;   // from PSP back to our logic

    // Delay knobs tied to 0 (fastest)
    logic [1:0] d_m_setup = 2'b00;
    logic [1:0] d_m_hold  = 2'b00;
    logic [1:0] d_ps_hold = 2'b00;

    // Handshake toggles generated in clk domain
    logic psp_AckOut_SP_tgl;
    logic psp_ReqIn_PS_tgl;
    logic [PSP_BW-1:0] psp_DataIn_PS;

    PSP #(.BW(PSP_BW)) u_psp (
        // Parallel->Serial path (chip output)
        .ReqIn_PS (psp_ReqIn_PS_tgl),
        .AckIn_PS (psp_AckIn_PS),
        .DataIn_PS(psp_DataIn_PS),
        .d_m_setup(d_m_setup),
        .d_m_hold (d_m_hold),
        .d_ps_hold(d_ps_hold),
        .ReqOut_PS(AER_SER_OUT_REQ),
        .AckOut_PS(AER_SER_OUT_ACK),
        .DataOut_PS(psp_DataOut_PS),
        // Serial->Parallel path (chip input)
        .ReqIn_SP (AER_SER_IN_REQ),
        .AckIn_SP (AER_SER_IN_ACK),
        .DataIn_SP(AER_SER_IN_DATA),
        .ReqOut_SP(psp_ReqOut_SP),
        .AckOut_SP(psp_AckOut_SP_tgl),
        .DataOut_SP(psp_DataOut_SP),
        // Reset
        .rst(rst)
    );

    assign AER_SER_OUT_DATA = psp_DataOut_PS;

    // -------------------- CDC: StoP (SP) into ELFCORE -----------------
    // Sync the async toggle into clk domain
    logic sp_req_meta1, sp_req_meta2, sp_req_meta3;
    always_ff @(posedge clk or posedge rst) begin
        if (rst) begin
            sp_req_meta1 <= 1'b0;
            sp_req_meta2 <= 1'b0;
            sp_req_meta3 <= 1'b0;
        end else begin
            sp_req_meta1 <= psp_ReqOut_SP;
            sp_req_meta2 <= sp_req_meta1;
            sp_req_meta3 <= sp_req_meta2;
        end
    end
    wire sp_req_edge = sp_req_meta2 ^ sp_req_meta3;

    // Optional 4-slot cycle delay line for the synchronized pulse
    logic [3:0] sp_pipe;
    always_ff @(posedge clk or posedge rst) begin
        if (rst) begin
            sp_pipe <= 4'b0000;
        end else begin
            sp_pipe <= {sp_pipe[2:0], sp_req_edge};
        end
    end
    wire sp_req_edge_delayed = sp_pipe[STOP_DELAY_CYCLES];

    // On edge, capture lower M_INP bits and toggle AckOut to PSP
    always_ff @(posedge clk or posedge rst) begin
        if (rst) begin
            sff_AERIN_ADDR   <= '0;
            sff_AERIN_REQ    <= 1'b0;
            psp_AckOut_SP_tgl<= 1'b0;
        end else begin
            sff_AERIN_REQ <= 1'b0;
            if (sp_req_edge_delayed) begin
                sff_AERIN_ADDR    <= psp_DataOut_SP[M_INP-1:0];
                sff_AERIN_REQ     <= 1'b1;
                psp_AckOut_SP_tgl <= ~psp_AckOut_SP_tgl;
            end
        end
    end

    // -------------------- CDC: ELFCORE to PtoS (PS) -------------------
    // Launch serialization on OUT_REQ and ACK back only when PSP accepts (AckIn_PS toggle)
    logic ack_ps_meta1, ack_ps_meta2, ack_ps_meta3;
    logic ptos_pending;
    always_ff @(posedge clk or posedge rst) begin
        if (rst) begin
            psp_DataIn_PS     <= '0;
            psp_ReqIn_PS_tgl  <= 1'b0;
            sff_OUT_ACK       <= 1'b0;
            ack_ps_meta1      <= 1'b0;
            ack_ps_meta2      <= 1'b0;
            ack_ps_meta3      <= 1'b0;
            ptos_pending      <= 1'b0;
        end else begin
            // Synchronize PSP AckIn_PS toggle
            ack_ps_meta1 <= psp_AckIn_PS;
            ack_ps_meta2 <= ack_ps_meta1;
            ack_ps_meta3 <= ack_ps_meta2;
            // Default deassert
            sff_OUT_ACK  <= 1'b0;
            // Launch when request arrives and not already pending
            if (USE_OUT_BUS_AS_AER && sff_OUT_REQ && !ptos_pending) begin
                psp_DataIn_PS                  <= '0;
                psp_DataIn_PS[M_OUT-1:0]      <= sff_OUT_DATA[M_OUT-1:0];
                psp_ReqIn_PS_tgl               <= ~psp_ReqIn_PS_tgl;
                ptos_pending                   <= 1'b1;
            end
            // Complete when ack toggle observed
            if (ptos_pending && (ack_ps_meta2 ^ ack_ps_meta3)) begin
                sff_OUT_ACK <= 1'b1;   // one-cycle acknowledge back to ELFCORE
                ptos_pending<= 1'b0;
            end
        end
    end

endmodule


