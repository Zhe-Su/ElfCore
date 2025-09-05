// SPDX-License-Identifier: Apache-2.0 WITH SHL-2.1

`timescale 1ns/1ps

// LFSR (parameterized)
module lfsr_param #(
    parameter int unsigned WIDTH = 17,
    parameter logic [WIDTH-1:0] TAPS = (WIDTH==17) ? 17'b10010000000000000 : 17'b00000000000000001
) (
    input  logic clk,
    input  logic rst,
    input  logic en,
    input  logic [WIDTH-1:0] seed,
    output logic [WIDTH-1:0] rnd
);
    function logic feedback_bit (input logic [WIDTH-1:0] state);
        feedback_bit = ^(state & TAPS);
    endfunction
    always_ff @(posedge clk, posedge rst) begin
        if (rst) rnd <= (seed == '0) ? {{(WIDTH-1){1'b0}},1'b1} : seed;
        else if (en) rnd <= {rnd[WIDTH-2:0], feedback_bit(rnd)};
    end
`ifdef ASSERT_ON
    // Seed should be non-zero to avoid lock-up (we force to 1 if zero)
    assert property (@(posedge clk) rst |-> (seed != '0));
`endif
endmodule

// SRAM wrappers
module SRAM_512x128_wrapper (
    input  logic        CK,
    input  logic [1:0]  SM,
    input  logic        CS,
    input  logic        WE,
    input  logic [8:0]  A,
    input  logic [127:0]D,
    input  logic [127:0]M,
    output logic [127:0]Q
);
    logic [127:0] mem [0:511];
    logic [127:0] Qr;
    always_ff @(posedge CK) begin
        if (CS) Qr <= mem[A];
        if (CS && WE) mem[A] <= (D & M) | (mem[A] & ~M);
    end
    assign Q = Qr;
`ifdef ASSERT_ON
    // Address must be within range when selected
    assert property (@(posedge CK) CS |-> (A < 9'd512));
`endif
endmodule

module SRAM_128x320_wrapper (
    input  logic        CK,
    input  logic [1:0]  SM,
    input  logic        CS,
    input  logic        WE,
    input  logic [$clog2(128)-1:0] A,
    input  logic [319:0]D,
    input  logic [319:0]M,
    output logic [319:0]Q
);
    logic [319:0] mem [0:127];
    logic [319:0] Qr;
    always_ff @(posedge CK) begin
        if (CS) Qr <= mem[A];
        if (CS && WE) mem[A] <= (D & M) | (mem[A] & ~M);
    end
    assign Q = Qr;
`ifdef ASSERT_ON
    assert property (@(posedge CK) CS |-> (A < 8'd128));
`endif
endmodule

module SRAM_4096x136_wrapper (
    input  logic        CK,
    input  logic        CS,
    input  logic        WE,
    input  logic [11:0] A,
    input  logic [135:0]D,
    output logic [135:0]Q
);
    logic [135:0] mem [0:4095];
    logic [135:0] Qr;
    always_ff @(posedge CK) begin
        if (CS) Qr <= mem[A];
        if (CS && WE) mem[A] <= D;
    end
    assign Q = Qr;
`ifdef ASSERT_ON
    assert property (@(posedge CK) CS |-> (A < 12'd4096));
`endif
endmodule

// FIFO
module fifo_simpl #(
    parameter int unsigned WIDTH = 9,
    parameter int unsigned DEPTH = 8
) (
    input  logic             clk,
    input  logic             rst,
    input  logic             push,
    input  logic [WIDTH-1:0] din,
    input  logic             pop,
    output logic [WIDTH-1:0] dout,
    output logic             empty,
    output logic [$clog2(DEPTH):0] level
);
    localparam int unsigned AW = (DEPTH <= 1) ? 1 : $clog2(DEPTH);
    logic [WIDTH-1:0] mem [0:DEPTH-1];
    logic [AW:0]      wr_ptr, rd_ptr;
    assign empty = (wr_ptr == rd_ptr);
    assign level = wr_ptr - rd_ptr;
    assign dout  = mem[rd_ptr[AW-1:0]];
    always_ff @(posedge clk or posedge rst) begin
        if (rst) begin
            wr_ptr <= '0; rd_ptr <= '0;
        end else begin
            if (push) begin mem[wr_ptr[AW-1:0]] <= din; wr_ptr <= wr_ptr + 1'b1; end
            if (pop && !empty) rd_ptr <= rd_ptr + 1'b1;
        end
    end
endmodule

// SPI slave (optional consolidation target)
module spi_elfcore (
    input  logic SCK,
    input  logic MOSI,
    input  logic CS_N,
    output logic MISO,
    input  logic RST,
    input  logic clk,
    output logic        SPI_EN_CONF,
    output logic        SPI_LOCAL_TICK,
    output logic        SPI_ERROR_HALT,
    output logic        SPI_TIMING_MODE,
    output logic        SPI_REGRESSION,
    output logic        SPI_SEND_PER_TIMESTEP,
    output logic        SPI_SEND_LABEL_ONLY,
    output logic        SPI_NO_OUT_ACT,
    output logic        SPI_EN_STOCH_ROUND,
    output logic        SPI_NOISE_EN,
    output logic [3:0]  SPI_NOISE_STR,
    output logic [31:0] SPI_CYCLES_PER_TICK,
    output logic        SPI_STREAM_ACT,
    output logic        SPI_PIPELINED,
    output logic        SPI_AER_DEBOUNCE,
    output logic [2:0]  EFF_FP_LOC_TINP,
    output logic [2:0]  EFF_FP_LOC_TOUT,
    output logic [2:0]  EFF_FP_LOC_WOUT,
    output logic [7:0]  SPI_NUM_INP_NEUR,
    output logic [8:0]  SPI_NUM_H1_NEUR,
    output logic [3:0]  SPI_NUM_OUT_NEUR,
    output logic [14:0] SPI_SEED_STRND,
    output logic [16:0] SPI_SEED_NOISE,
    output logic [7:0]  SPI_SRAM_SPEEDMODE_H1_0,
    output logic [7:0]  SPI_SRAM_SPEEDMODE_H1_1,
    output logic [7:0]  SPI_SRAM_SPEEDMODE_H1_2,
    output logic [7:0]  SPI_SRAM_SPEEDMODE_H1_3,
    output logic [7:0]  SPI_SRAM_SPEEDMODE_H2_0,
    output logic [7:0]  SPI_SRAM_SPEEDMODE_H2_1,
    output logic [7:0]  SPI_SRAM_SPEEDMODE_H2_2,
    output logic [7:0]  SPI_SRAM_SPEEDMODE_H2_3,
    input  logic [31:0] CNT_TICKS,
    input  logic [15:0] CNT_SPIKES_H1,
    input  logic [15:0] CNT_SPIKES_H2,
    input  logic [7:0]  AER_FIFO_HWM,
    output logic [2:0]  SPI_THIST_DEPTH,
    output logic [2:0]  SPI_TPREV_DEPTH,
    input  logic [31:0] SIM_PRED_H1,
    input  logic [31:0] SIM_CONTR_H1,
    input  logic [31:0] SIM_PRED_H2,
    input  logic [31:0] SIM_CONTR_H2,
    input  logic [15:0] H1_SPK_CNT,
    input  logic [15:0] H2_SPK_CNT,
    output logic [15:0] SPI_SIM_SCALE,
    output logic [4:0]  SPI_SIM_SHIFT,
    output logic [1:0]  SPI_SIM_NORM_MODE,
    output logic [15:0] SPI_THR_I,
    output logic [15:0] SPI_H1_PRED,
    output logic [15:0] SPI_H1_CONTR,
    output logic [15:0] SPI_H2_PRED,
    output logic [15:0] SPI_H2_CONTR,
    output logic [15:0] SPI_I_SCALE,
    output logic [4:0]  SPI_I_SHIFT,
    output logic [7:0]  SPI_N_PER_G,
    input  logic [31:0] LEARN_L1_PRE,
    input  logic [31:0] LEARN_L1_WORD,
    input  logic [31:0] LEARN_L2_PRE,
    input  logic [31:0] LEARN_L2_WORD,
    input  logic        H1_NO_UPDATE,
    input  logic        H2_NO_UPDATE,
    output logic        SPI_TRACE_WR_EN,
    output logic [7:0]  SPI_LR_H1,
    output logic [7:0]  SPI_LR_H2,
    output logic        SPI_STE_WE,
    output logic [7:0]  SPI_STE_IDX,
    output logic [7:0]  SPI_STE_DATA,
    output logic        SPI_DYN_EN_H1,
    output logic        SPI_DYN_EN_H2,
    output logic [15:0] SPI_DYN_X_H1,
    output logic [15:0] SPI_DYN_X_H2,
    output logic [7:0]  SPI_PRUNE_K_H1,
    output logic [7:0]  SPI_PRUNE_K_H2,
    output logic        SPI_COLLISION_EN,
    output logic [16:0] SPI_SEED_DST,
    output logic        SPI_CFG_COMMIT
);
    logic [31:0] sh_in, sh_out;
    logic [31:0] addr;
    logic [5:0]  bitcnt;
    logic        addr_latched;
    always_ff @(negedge SCK, posedge RST) begin
        if (RST) begin
            bitcnt <= 6'd0; addr_latched <= 1'b0; sh_out <= 32'd0;
            SPI_EN_CONF <= 1'b0; SPI_LOCAL_TICK <= 1'b0; SPI_ERROR_HALT <= 1'b0;
            SPI_TIMING_MODE <= 1'b0; SPI_REGRESSION <= 1'b0; SPI_SEND_PER_TIMESTEP <= 1'b0; SPI_SEND_LABEL_ONLY <= 1'b1;
            SPI_NO_OUT_ACT <= 1'b0; SPI_EN_STOCH_ROUND <= 1'b0; SPI_NOISE_EN <= 1'b0; SPI_NOISE_STR <= 4'd0;
            SPI_CYCLES_PER_TICK <= 32'd1000; SPI_STREAM_ACT <= 1'b0; SPI_PIPELINED <= 1'b0; SPI_AER_DEBOUNCE <= 1'b1;
            SPI_THIST_DEPTH <= 3'd0; SPI_TPREV_DEPTH <= 3'd0;
            SPI_SRAM_SPEEDMODE_H1_0 <= 8'h00; SPI_SRAM_SPEEDMODE_H1_1 <= 8'h00; SPI_SRAM_SPEEDMODE_H1_2 <= 8'h00; SPI_SRAM_SPEEDMODE_H1_3 <= 8'h00;
            SPI_SRAM_SPEEDMODE_H2_0 <= 8'h00; SPI_SRAM_SPEEDMODE_H2_1 <= 8'h00; SPI_SRAM_SPEEDMODE_H2_2 <= 8'h00; SPI_SRAM_SPEEDMODE_H2_3 <= 8'h00;
            EFF_FP_LOC_TINP <= 3'd0; EFF_FP_LOC_TOUT <= 3'd0; EFF_FP_LOC_WOUT <= 3'd0;
            SPI_NUM_INP_NEUR <= 8'hFF; SPI_NUM_H1_NEUR <= 9'h1FF; SPI_NUM_OUT_NEUR <= 4'hF;
            SPI_SEED_STRND <= 15'h1234; SPI_SEED_NOISE <= 17'h1F0F1;
            SPI_SIM_SCALE <= 16'd1; SPI_SIM_SHIFT <= 5'd0; SPI_SIM_NORM_MODE <= 2'd0;
            SPI_THR_I <= 16'd0; SPI_H1_PRED <= 16'd0; SPI_H1_CONTR <= 16'd0; SPI_H2_PRED <= 16'd0; SPI_H2_CONTR <= 16'd0;
            SPI_I_SCALE <= 16'd1; SPI_I_SHIFT <= 5'd0; SPI_N_PER_G <= 8'd4; SPI_LR_H1 <= 8'd0; SPI_LR_H2 <= 8'd0;
            SPI_STE_WE <= 1'b0; SPI_STE_IDX <= 8'd0; SPI_STE_DATA <= 8'd0; SPI_TRACE_WR_EN <= 1'b0;
            SPI_DYN_EN_H1 <= 1'b0; SPI_DYN_EN_H2 <= 1'b0; SPI_DYN_X_H1 <= 16'd0; SPI_DYN_X_H2 <= 16'd0; SPI_PRUNE_K_H1 <= 8'd8; SPI_PRUNE_K_H2 <= 8'd8;
            SPI_COLLISION_EN <= 1'b1; SPI_SEED_DST <= 17'h35A5;
        end else if (!CS_N) begin
            bitcnt <= bitcnt + 6'd1; sh_in <= {sh_in[30:0], MOSI};
            if (bitcnt == 6'd31) begin
                if (!addr_latched) begin addr <= {sh_in[30:0], MOSI}; addr_latched <= 1'b1; end
                else begin
                    unique case (addr[15:0])
                        16'd0  : SPI_EN_CONF           <= MOSI;
                        16'd10 : SPI_LOCAL_TICK        <= MOSI;
                        16'd11 : SPI_ERROR_HALT        <= MOSI;
                        16'd23 : SPI_TIMING_MODE       <= MOSI;
                        16'd25 : SPI_REGRESSION        <= MOSI;
                        16'd27 : SPI_NO_OUT_ACT        <= MOSI;
                        16'd30 : SPI_SEND_PER_TIMESTEP <= MOSI;
                        16'd31 : SPI_SEND_LABEL_ONLY   <= MOSI;
                        16'd28 : SPI_STREAM_ACT        <= MOSI;
                        16'd29 : SPI_PIPELINED         <= MOSI;
                        16'd33 : SPI_AER_DEBOUNCE      <= MOSI;
                        16'd34 : SPI_THIST_DEPTH       <= {sh_in[1:0], MOSI};
                        16'd35 : SPI_TPREV_DEPTH       <= {sh_in[1:0], MOSI};
                        16'd200: SPI_SRAM_SPEEDMODE_H1_0 <= {sh_in[6:0], MOSI};
                        16'd201: SPI_SRAM_SPEEDMODE_H1_1 <= {sh_in[6:0], MOSI};
                        16'd202: SPI_SRAM_SPEEDMODE_H1_2 <= {sh_in[6:0], MOSI};
                        16'd203: SPI_SRAM_SPEEDMODE_H1_3 <= {sh_in[6:0], MOSI};
                        16'd204: SPI_SRAM_SPEEDMODE_H2_0 <= {sh_in[6:0], MOSI};
                        16'd205: SPI_SRAM_SPEEDMODE_H2_1 <= {sh_in[6:0], MOSI};
                        16'd206: SPI_SRAM_SPEEDMODE_H2_2 <= {sh_in[6:0], MOSI};
                        16'd207: SPI_SRAM_SPEEDMODE_H2_3 <= {sh_in[6:0], MOSI};
                        16'd12 : EFF_FP_LOC_TINP       <= {sh_in[1:0], MOSI};
                        16'd17 : EFF_FP_LOC_TOUT       <= {sh_in[1:0], MOSI};
                        16'd14 : EFF_FP_LOC_WOUT       <= {sh_in[1:0], MOSI};
                        16'd64 : SPI_CYCLES_PER_TICK   <= {sh_in[30:0], MOSI};
                        16'd94 : SPI_NUM_INP_NEUR      <= {sh_in[6:0], MOSI};
                        16'd95 : SPI_NUM_H1_NEUR       <= {sh_in[7:0], MOSI};
                        16'd96 : SPI_NUM_OUT_NEUR      <= {sh_in[2:0], MOSI};
                        16'd21 : SPI_EN_STOCH_ROUND    <= MOSI;
                        16'd32 : SPI_NOISE_EN          <= MOSI;
                        16'd104: SPI_NOISE_STR         <= {sh_in[2:0], MOSI};
                        16'd88 : SPI_SEED_STRND        <= {sh_in[13:0], MOSI};
                        16'd93 : SPI_SEED_NOISE        <= {sh_in[15:0], MOSI};
                        16'd240: SPI_SIM_SCALE         <= {sh_in[14:0], MOSI};
                        16'd241: SPI_SIM_SHIFT         <= {sh_in[3:0], MOSI};
                        16'd242: SPI_SIM_NORM_MODE     <= {sh_in[0], MOSI};
                        16'd243: SPI_THR_I             <= {sh_in[14:0], MOSI};
                        16'd244: SPI_H1_PRED           <= {sh_in[14:0], MOSI};
                        16'd245: SPI_H1_CONTR          <= {sh_in[14:0], MOSI};
                        16'd246: SPI_H2_PRED           <= {sh_in[14:0], MOSI};
                        16'd247: SPI_H2_CONTR          <= {sh_in[14:0], MOSI};
                        16'd248: SPI_I_SCALE           <= {sh_in[14:0], MOSI};
                        16'd249: SPI_I_SHIFT           <= {sh_in[3:0], MOSI};
                        16'd250: SPI_N_PER_G             <= {sh_in[6:0], MOSI};
                        16'd252: SPI_LR_H1              <= {sh_in[6:0], MOSI};
                        16'd253: SPI_LR_H2              <= {sh_in[6:0], MOSI};
                        16'd254: SPI_STE_IDX            <= {sh_in[6:0], MOSI};
                        16'd255: begin SPI_STE_WE <= MOSI; SPI_STE_DATA <= sh_in[7:0]; end
                        16'd256: SPI_TRACE_WR_EN        <= MOSI;
                        16'd270: SPI_DYN_EN_H1          <= MOSI;
                        16'd271: SPI_DYN_EN_H2          <= MOSI;
                        16'd272: SPI_DYN_X_H1           <= {sh_in[14:0], MOSI};
                        16'd273: SPI_DYN_X_H2           <= {sh_in[14:0], MOSI};
                        16'd274: SPI_PRUNE_K_H1         <= {sh_in[6:0], MOSI};
                        16'd275: SPI_PRUNE_K_H2         <= {sh_in[6:0], MOSI};
                        16'd276: SPI_COLLISION_EN       <= MOSI;
                        16'd277: SPI_SEED_DST           <= {sh_in[15:0], MOSI};
                        default: ;
                    endcase
                    addr_latched <= 1'b0;
                end
            end
        end
    end
    assign MISO = (!CS_N) ? sh_out[31] : 1'b0;
    always_ff @(negedge SCK, posedge RST) begin
        if (RST) sh_out <= 32'd0;
        else if (!CS_N && addr_latched && (addr[31])) begin
            unique case (addr[15:0])
                16'd0  : sh_out <= {31'd0, SPI_EN_CONF};
                16'd10 : sh_out <= {31'd0, SPI_LOCAL_TICK};
                16'd11 : sh_out <= {31'd0, SPI_ERROR_HALT};
                16'd23 : sh_out <= {31'd0, SPI_TIMING_MODE};
                16'd25 : sh_out <= {31'd0, SPI_REGRESSION};
                16'd27 : sh_out <= {31'd0, SPI_NO_OUT_ACT};
                16'd30 : sh_out <= {31'd0, SPI_SEND_PER_TIMESTEP};
                16'd31 : sh_out <= {31'd0, SPI_SEND_LABEL_ONLY};
                16'd28 : sh_out <= {31'd0, SPI_STREAM_ACT};
                16'd29 : sh_out <= {31'd0, SPI_PIPELINED};
                16'd33 : sh_out <= {31'd0, SPI_AER_DEBOUNCE};
                16'd34 : sh_out <= {29'd0, SPI_THIST_DEPTH};
                16'd35 : sh_out <= {29'd0, SPI_TPREV_DEPTH};
                16'd12 : sh_out <= {29'd0, EFF_FP_LOC_TINP};
                16'd210: sh_out <= CNT_TICKS;
                16'd211: sh_out <= {16'd0, CNT_SPIKES_H1};
                16'd212: sh_out <= {16'd0, CNT_SPIKES_H2};
                16'd213: sh_out <= {24'd0, AER_FIFO_HWM};
                16'd17 : sh_out <= {29'd0, EFF_FP_LOC_TOUT};
                16'd14 : sh_out <= {29'd0, EFF_FP_LOC_WOUT};
                16'd64 : sh_out <= SPI_CYCLES_PER_TICK;
                16'd94 : sh_out <= {24'd0, SPI_NUM_INP_NEUR};
                16'd95 : sh_out <= {23'd0, SPI_NUM_H1_NEUR};
                16'd96 : sh_out <= {28'd0, SPI_NUM_OUT_NEUR};
                16'd21 : sh_out <= {31'd0, SPI_EN_STOCH_ROUND};
                16'd32 : sh_out <= {31'd0, SPI_NOISE_EN};
                16'd104: sh_out <= {28'd0, SPI_NOISE_STR};
                16'd88 : sh_out <= {17'd0, SPI_SEED_STRND};
                16'd93 : sh_out <= {15'd0, SPI_SEED_NOISE};
                16'd230: sh_out <= SIM_PRED_H1;
                16'd231: sh_out <= SIM_CONTR_H1;
                16'd232: sh_out <= SIM_PRED_H2;
                16'd233: sh_out <= SIM_CONTR_H2;
                16'd234: sh_out <= {16'd0, H1_SPK_CNT};
                16'd235: sh_out <= {16'd0, H2_SPK_CNT};
                16'd240: sh_out <= {16'd0, SPI_SIM_SCALE};
                16'd241: sh_out <= {27'd0, SPI_SIM_SHIFT};
                16'd242: sh_out <= {30'd0, SPI_SIM_NORM_MODE};
                16'd243: sh_out <= {16'd0, SPI_THR_I};
                16'd244: sh_out <= {16'd0, SPI_H1_PRED};
                16'd245: sh_out <= {16'd0, SPI_H1_CONTR};
                16'd246: sh_out <= {16'd0, SPI_H2_PRED};
                16'd247: sh_out <= {16'd0, SPI_H2_CONTR};
                16'd248: sh_out <= {16'd0, SPI_I_SCALE};
                16'd249: sh_out <= {27'd0, SPI_I_SHIFT};
                16'd250: sh_out <= {24'd0, SPI_N_PER_G};
                16'd260: sh_out <= LEARN_L1_PRE;
                16'd261: sh_out <= LEARN_L1_WORD;
                16'd262: sh_out <= LEARN_L2_PRE;
                16'd263: sh_out <= LEARN_L2_WORD;
                16'd264: sh_out <= {31'd0, H1_NO_UPDATE};
                16'd265: sh_out <= {31'd0, H2_NO_UPDATE};
                16'd270: sh_out <= {31'd0, SPI_DYN_EN_H1};
                16'd271: sh_out <= {31'd0, SPI_DYN_EN_H2};
                16'd272: sh_out <= {16'd0, SPI_DYN_X_H1};
                16'd273: sh_out <= {16'd0, SPI_DYN_X_H2};
                16'd274: sh_out <= {24'd0, SPI_PRUNE_K_H1};
                16'd275: sh_out <= {24'd0, SPI_PRUNE_K_H2};
                16'd276: sh_out <= {31'd0, SPI_COLLISION_EN};
                16'd277: sh_out <= {15'd0, SPI_SEED_DST};
                default: sh_out <= 32'd0;
            endcase
        end else begin
            sh_out <= {sh_out[30:0], 1'b0};
        end
    end
    logic sck_commit;
    always_ff @(negedge SCK or posedge RST) begin
        if (RST) sck_commit <= 1'b0;
        else if (!CS_N && (bitcnt == 6'd31) && addr_latched) sck_commit <= 1'b1;
        else sck_commit <= 1'b0;
    end
    logic sckc_sync1, sckc_sync2;
    always_ff @(posedge clk or posedge RST) begin
        if (RST) begin sckc_sync1 <= 1'b0; sckc_sync2 <= 1'b0; end
        else begin sckc_sync1 <= sck_commit; sckc_sync2 <= sckc_sync1; end
    end
    assign SPI_CFG_COMMIT = (sckc_sync1 & ~sckc_sync2);
`ifdef ASSERT_ON
    assert property (@(negedge SCK) CS_N |-> (MISO==1'b0));
`endif
endmodule

// H1 bank arbiter (two-client RR)
module h1_bank_arbiter #(
    parameter int NG = 4,
    parameter int AW = 7
)(
    input  logic                 clk,
    input  logic                 rst,
    input  logic                 req_l1   [NG],
    input  logic [AW-1:0]        addr_l1  [NG],
    input  logic                 req_l2   [NG],
    input  logic [AW-1:0]        addr_l2  [NG],
    output logic                 cs       [NG],
    output logic [AW-1:0]        addr     [NG],
    output logic                 valid_l1 [NG],
    output logic                 valid_l2 [NG]
);
    logic rr[NG], last_is_l1[NG], rd_q[NG], do_valid[NG];
    integer g;
    always_ff @(posedge clk or posedge rst) begin
        if (rst) begin
            for (g=0; g<NG; g++) begin rr[g]<=0; last_is_l1[g]<=0; cs[g]<=0; addr[g]<='0; rd_q[g]<=0; do_valid[g]<=0; valid_l1[g]<=0; valid_l2[g]<=0; end
        end else begin
            for (g=0; g<NG; g++) begin
                logic both = req_l1[g] & req_l2[g];
                logic grant_l1 = both ? ~rr[g] : req_l1[g];
                rr[g]   <= both ? ~rr[g] : rr[g];
                cs[g]   <= req_l1[g] | req_l2[g];
                addr[g] <= grant_l1 ? addr_l1[g] : addr_l2[g];
                if (req_l1[g] | req_l2[g]) last_is_l1[g] <= grant_l1;
                rd_q[g]       <= cs[g];
                do_valid[g]   <= rd_q[g];
                valid_l1[g]   <= do_valid[g] & last_is_l1[g];
                valid_l2[g]   <= do_valid[g] & ~last_is_l1[g];
            end
        end
    end
endmodule

// H2 bank arbiter (two-client RR)
module h2_bank_arbiter #(
    parameter int NG = 4,
    parameter int AW = 7
)(
    input  logic                 clk,
    input  logic                 rst,
    input  logic                 req_l1   [NG],
    input  logic [AW-1:0]        addr_l1  [NG],
    input  logic                 req_l2   [NG],
    input  logic [AW-1:0]        addr_l2  [NG],
    output logic                 cs       [NG],
    output logic [AW-1:0]        addr     [NG],
    output logic                 valid_l1 [NG],
    output logic                 valid_l2 [NG]
);
    logic rr[NG], last_is_l1[NG], rd_q[NG], do_valid[NG];
    integer g;
    always_ff @(posedge clk or posedge rst) begin
        if (rst) begin
            for (g=0; g<NG; g++) begin rr[g]<=0; last_is_l1[g]<=0; cs[g]<=0; addr[g]<='0; rd_q[g]<=0; do_valid[g]<=0; valid_l1[g]<=0; valid_l2[g]<=0; end
        end else begin
            for (g=0; g<NG; g++) begin
                logic both = req_l1[g] & req_l2[g];
                logic grant_l1 = both ? ~rr[g] : req_l1[g];
                rr[g]   <= both ? ~rr[g] : rr[g];
                cs[g]   <= req_l1[g] | req_l2[g];
                addr[g] <= grant_l1 ? addr_l1[g] : addr_l2[g];
                if (req_l1[g] | req_l2[g]) last_is_l1[g] <= grant_l1;
                rd_q[g]       <= cs[g];
                do_valid[g]   <= rd_q[g];
                valid_l1[g]   <= do_valid[g] & last_is_l1[g];
                valid_l2[g]   <= do_valid[g] & ~last_is_l1[g];
            end
        end
    end
endmodule

// SRAM DO-valid helper
module sram_do_valid (
    input  logic clk,
    input  logic rst,
    input  logic cs,
    input  logic we,
    output logic do_valid
);
    logic read_q;
    always_ff @(posedge clk or posedge rst) begin
        if (rst) begin read_q<=0; do_valid<=0; end
        else begin read_q <= (cs & ~we); do_valid <= read_q; end
    end
`ifdef ASSERT_ON
    // do_valid should indicate a read in previous cycle
    assert property (@(posedge clk) do_valid |-> $past(cs & ~we));
`endif
endmodule

// DST controller (skeleton)
module dst_controller #(
    parameter int NG = 4
)(
    input  logic clk,
    input  logic rst,
    input  logic start,
    output logic busy,
    output logic done
);
    typedef enum logic [1:0] {DIDLE, DPR, DREW, DDONE} st_t;
    st_t st;
    always_ff @(posedge clk or posedge rst) begin
        if (rst) begin st <= DIDLE; busy <= 1'b0; done <= 1'b0; end
        else begin
            done <= 1'b0;
            case (st)
                DIDLE: begin busy <= 1'b0; if (start) begin st <= DPR; busy <= 1'b1; end end
                DPR:   begin st <= DREW; end
                DREW:  begin st <= DDONE; end
                DDONE: begin done <= 1'b1; busy <= 1'b0; st <= DIDLE; end
            endcase
        end
    end
endmodule

// Streaming Top-K heap
module heap_topk_stream #(
    parameter int K  = 256,
    parameter int W  = 8,
    parameter int MW = 16,
    parameter bit CMP_MAX = 1'b0
)(
    input  logic              clk,
    input  logic              rst,
    input  logic              in_valid,
    input  logic [W-1:0]      in_value,
    input  logic [MW-1:0]     in_meta,
    input  logic              in_last,
    output logic              in_ready,
    output logic              done,
    input  logic              pop_req,
    output logic              pop_valid,
    output logic [W-1:0]      pop_value,
    output logic [MW-1:0]     pop_meta,
    output logic              empty
);
    typedef struct packed { logic valid; logic [W-1:0] val; logic [MW-1:0] meta; } entry_t;
    entry_t entries [K];
    logic [$clog2(K+1)-1:0] num_valid; logic accepting;
    assign in_ready = 1'b1;
    function automatic bit is_better(input logic [W-1:0] a, input logic [W-1:0] b);
        if (CMP_MAX) return (a > b); else return (a < b);
    endfunction
    function automatic int find_worst_idx(); int idx=-1; logic [W-1:0] worst_val; for (int i=0;i<K;i++) if (entries[i].valid) begin if (idx==-1) begin idx=i; worst_val=entries[i].val; end else if (CMP_MAX?(entries[i].val<worst_val):(entries[i].val>worst_val)) begin idx=i; worst_val=entries[i].val; end end return (idx==-1)?0:idx; endfunction
    function automatic int find_best_idx(); int idx=-1; logic [W-1:0] best_val; for (int i=0;i<K;i++) if (entries[i].valid) begin if (idx==-1) begin idx=i; best_val=entries[i].val; end else if (is_better(entries[i].val,best_val)) begin idx=i; best_val=entries[i].val; end end return (idx==-1)?0:idx; endfunction
    always_ff @(posedge clk or posedge rst) begin
        if (rst) begin for (int i=0;i<K;i++) entries[i].valid<=1'b0; num_valid<='0; done<=1'b0; pop_valid<=1'b0; empty<=1'b1; accepting<=1'b1; end
        else begin
            pop_valid <= 1'b0;
            if (accepting && in_valid) begin
                if (num_valid < K) begin for (int i=0;i<K;i++) if (!entries[i].valid) begin entries[i].valid<=1'b1; entries[i].val<=in_value; entries[i].meta<=in_meta; break; end num_valid <= num_valid + 1'b1; end
                else begin int iw = find_worst_idx(); if (is_better(in_value, entries[iw].val)) begin entries[iw].val<=in_value; entries[iw].meta<=in_meta; end end
                if (in_last) begin accepting<=1'b0; done<=1'b1; empty<=(num_valid==0); end
            end
            if (!accepting && pop_req) begin
                if (num_valid != 0) begin int ib = find_best_idx(); pop_value<=entries[ib].val; pop_meta<=entries[ib].meta; pop_valid<=1'b1; entries[ib].valid<=1'b0; num_valid<=num_valid-1'b1; empty<=(num_valid==1); end
                else begin pop_valid<=1'b0; empty<=1'b1; end
            end
        end
    end
endmodule

// Grouped Top-K
module heap_group_topk_stream #(
    parameter int K  = 64,
    parameter int W  = 24,
    parameter int IW = 9
)(
    input  logic          clk,
    input  logic          rst,
    input  logic          in_valid,
    input  logic [W-1:0]  in_score,
    input  logic [IW-1:0] in_index,
    input  logic          in_last,
    output logic          in_ready,
    output logic          done,
    input  logic  [1:0]   pop_group,
    input  logic          pop_req,
    output logic          pop_valid,
    output logic [IW-1:0] pop_index,
    output logic          empty
);
    localparam int MW = IW;
    logic              done_g [4];
    logic              empty_g[4];
    logic              pop_req_g[4];
    logic              pop_valid_g[4];
    logic [W-1:0]      pop_value_g[4];
    logic [MW-1:0]     pop_meta_g [4];
    logic [1:0] gsel; assign gsel = in_index[IW-1:IW-2]; assign in_ready = 1'b1; assign done = done_g[0] & done_g[1] & done_g[2] & done_g[3];
    for (genvar g=0; g<4; g++) begin: G
        heap_topk_stream #(.K(K), .W(W), .MW(MW), .CMP_MAX(1'b1)) u_heap (
            .clk(clk), .rst(rst), .in_valid(in_valid && (gsel==g[1:0])), .in_value(in_score), .in_meta({{(MW-IW){1'b0}}, in_index}),
            .in_last(in_last && (g==3)), .in_ready(), .done(done_g[g]), .pop_req(pop_req_g[g]), .pop_valid(pop_valid_g[g]), .pop_value(pop_value_g[g]), .pop_meta(pop_meta_g[g]), .empty(empty_g[g])
        );
    end
    always_comb begin for (int g=0; g<4; g++) pop_req_g[g] = 1'b0; pop_req_g[pop_group] = pop_req; pop_valid = pop_valid_g[pop_group]; pop_index = pop_meta_g[pop_group][IW-1:0]; empty = empty_g[pop_group]; end
endmodule

// Output learner
module output_learner #(
    parameter int N_OUT = 16,
    parameter int M_OUT = (N_OUT<=1)?1:$clog2(N_OUT)
)(
    input  logic                    clk,
    input  logic                    rst,
    input  logic                    sup_en,
    input  logic                    learn_use_act,
    input  logic [7:0]              lr_q8,
    input  logic [3:0]              err_shift,
    input  logic [15:0]             grad_max,
    input  logic [15:0]             err_max,
    input  logic                    ste_en,
    input  logic [7:0]              pre_scale_q8,
    input  logic [7:0]              wdec_q8,
    input  logic                    stoch_en,
    input  logic                    label_active,
    input  logic [M_OUT-1:0]        label_idx,
    input  logic signed [15:0]      out_u    [N_OUT],
    input  logic signed [15:0]      act_y    [N_OUT],
    input  logic                    start,
    input  logic [8:0]              pre_idx,
    input  logic [127:0]            row_w8,
    input  logic                    rnd_bit,
    output logic                    busy,
    output logic                    done,
    output logic [8:0]              pre_idx_o,
    output logic [127:0]            row_w8_o,
    output logic signed [15:0]      err_vec [N_OUT]
);
    logic [3:0] k; logic [127:0] row_q; typedef enum logic [1:0] {OL_IDLE, OL_RUN, OL_DONE} ols_t; ols_t st;
    always_ff @(posedge clk or posedge rst) begin
        if (rst) begin st<=OL_IDLE; k<='0; row_q<='0; done<=1'b0; busy<=1'b0; pre_idx_o<='0; row_w8_o<='0; end
        else begin done<=1'b0; case (st)
            OL_IDLE: begin busy<=1'b0; if (sup_en && label_active && start) begin busy<=1'b1; st<=OL_RUN; k<=4'd0; row_q<=row_w8; pre_idx_o<=pre_idx; end end
            OL_RUN: begin
                automatic logic signed [15:0] yk   = learn_use_act ? act_y[k] : out_u[k];
                automatic logic signed [15:0] err0 = ($unsigned(k)==$unsigned(label_idx)) ? (16'sd32767 - yk) : (16'sd0 - yk);
                automatic logic signed [15:0] err1 = (err0[15] ? -err0 : err0) > err_max ? (err0[15] ? -err_max : err_max) : err0;
                automatic logic signed [15:0] steo = ste_en ? (16'sd32767 - (yk[15] ? -yk : yk)) : 16'sd32767;
                automatic logic signed [31:0] err  = ((err1 * steo) >>> 15) >>> err_shift;
                automatic logic signed [15:0] pre_s= {8'd0, pre_scale_q8};
                automatic logic signed [31:0] derr0= (err * pre_s) >>> 8;
                automatic logic signed [15:0] derr16 = derr0[31] ? -derr0[15:0] : derr0[15:0];
                automatic logic signed [15:0] dclamp = (derr16 > grad_max) ? grad_max : derr16;
                automatic logic signed [31:0] derr = (derr0[31] ? -{{16{1'b0}}, dclamp} : {{16{1'b0}}, dclamp});
                automatic logic signed [7:0]  w8   = row_q[8*k +: 8];
                automatic logic signed [15:0] w16  = {{8{w8[7]}}, w8};
                automatic logic signed [31:0] wdec = (w16 * $signed({8'd0,wdec_q8})) >>> 8;
                automatic logic signed [31:0] prod = ((derr - wdec) * $signed({8'd0,lr_q8})) >>> 8;
                automatic logic signed [8:0]  wnew = w16[15:8] + prod[15:8] + (stoch_en ? {{8{1'b0}}, rnd_bit} : 9'sd0);
                automatic logic signed [7:0]  wsat = (wnew > 9'sd127) ? 8'sd127 : (wnew < -9'sd128) ? -8'sd128 : wnew[7:0];
                row_q[8*k +: 8] <= wsat; err_vec[k] <= err0; k <= k + 4'd1; if (k == (N_OUT-1)) st <= OL_DONE; end
            OL_DONE: begin row_w8_o <= row_q; done <= 1'b1; busy <= 1'b0; st <= OL_IDLE; end
        endcase end
    end
endmodule

// Sparse learner stub
module sparse_learner_vi_h1 #(
    parameter int NG = 4,
    parameter int M_INP = 9,
    parameter int M_GH1 = 7
)(
    input  logic                 clk,
    input  logic                 rst,
    input  logic                 enable,
    input  logic [7:0]           lr_q8,
    input  logic                 stoch_en,
    input  logic                 gate_pred,
    input  logic                 gate_contr,
    output logic                 sp_cs,
    output logic                 sp_we,
    output logic [11:0]          sp_addr,
    output logic [135:0]         sp_di,
    input  logic [135:0]         sp_do,
    input  logic                 sp_do_valid,
    output logic                 h1_req   [NG],
    output logic [M_GH1-1:0]     h1_addr  [NG],
    input  logic                 h1_valid [NG],
    input  logic [319:0]         h1_do    [NG],
    input  logic [7:0]           ste_lut  [256],
    input  logic                 rnd_bit
);
    integer g; always_ff @(posedge clk or posedge rst) begin if (rst) begin sp_cs<=1'b0; sp_we<=1'b0; sp_addr<='0; sp_di<='0; for (g=0; g<NG; g++) begin h1_req[g]<=1'b0; h1_addr[g]<='0; end end else begin sp_cs<=1'b0; sp_we<=1'b0; sp_di<='0; for (g=0; g<NG; g++) h1_req[g]<=1'b0; end end
endmodule


