`include "specs.v"

// Aggregated submodules used by PtoS and StoP

// From GENERIC_LatchReg.v
// latch_sr, latch_srneg, generic_latch_enh, generic_latch_enh_rstn,
// generic_latch_enl_rstn, generic_latch_bar_enl, generic_reg_posedge,
// generic_reg_posedge_b, generic_reg_posedge_setl,
// toggle_ff_posedge, toggle_ff_posedge_hold, toggle_ff_posedge_counter
// ... existing code ...
module latch_sr (s, r, q, qn);
parameter norDelay = `norDelay;
input wire s;
input wire r;
output wire q;
output wire qn;
   assign #(norDelay) q = ~(r | qn);
   assign #(norDelay) qn = ~(s | q);
endmodule
// ... existing code ...
module latch_srneg (sn, rn, q, qn);
parameter nandDelay = `nandDelay;
input wire sn;
input wire rn;
output wire q;
output wire qn;
   assign #(nandDelay) q = ~(sn & qn);
   assign #(nandDelay) qn = ~(rn & q);
endmodule
// ... existing code ...
module generic_latch_enh (en, d, q);
parameter WIDTH = 16;
parameter LATCH_DELAY = `DLatchDelay;
input wire en;
input wire [WIDTH-1:0] d;
output wire  [WIDTH-1:0] q;
 `ifdef SINTESI_RTL
   reg [WIDTH-1:0] x;
     always @ (en or d) begin
       if (en) begin
         x = d;
       end
     end
     assign #(LATCH_DELAY) q = x;
 `else
    genvar i;
    generate
      for (i=0; i< WIDTH; i=i+1) begin: Data_reg
        GTECH_LD1 mygate_latch_enh ( .G(en), .D(d[i]), .Q(q[i]) );
      end
    endgenerate
 `endif
endmodule
// ... existing code ...
module generic_latch_enh_rstn (rst, enh, d, q);
parameter WIDTH = 16;
parameter LATCH_DELAY = `DLatchDelayR;
input wire rst;
input wire enh;
input wire [WIDTH-1:0] d;
output wire  [WIDTH-1:0] q;
wire [WIDTH-1:0] qb;
 `ifdef SINTESI_RTL
   reg [WIDTH-1:0] x;
     always @ (rst or enh or d) begin
       if (~rst) begin
         x = 0;
       end else begin
         if (enh) begin
           x = d;
         end
       end
     end
     assign #(LATCH_DELAY) q = x;
 `else
    genvar i;
    generate
      for (i=0; i< WIDTH; i=i+1) begin: Data_reg
        GTECH_LD3 mygate_latch_enh_rstn ( .G(enh), .D(d[i]), .CD(rst), .Q(q[i]), .QN(qb[i]) );
      end
    endgenerate
 `endif
endmodule
// ... existing code ...
module generic_latch_enl_rstn (rst, enl, d, q);
parameter WIDTH = 16;
parameter LATCH_DELAY = `DLatchDelayR;
input wire rst;
input wire enl;
input wire [WIDTH-1:0] d;
output wire  [WIDTH-1:0] q;
 `ifdef SINTESI_RTL
   reg [WIDTH-1:0] x;
     always @ (rst or enl or d) begin
       if (~rst) begin
         x = 0;
       end else begin
         if (~enl) begin
           x = d;
         end
       end
     end
     assign #(LATCH_DELAY) q = x;
 `else
    genvar i;
    generate
      for (i=0; i< WIDTH; i=i+1) begin: Data_reg
        GTECH_LD4_1 mygate_latch_enh_rstn ( .GN(enl), .D(d[i]), .CD(rst), .Q(q[i]) );
      end
    endgenerate
 `endif
endmodule
// ... existing code ...
module generic_latch_bar_enl (enl, d, q, rst);
parameter WIDTH = 16;
parameter LATCH_DELAY = `DLatchDelayR;
input wire rst;
input wire [WIDTH-1:0] enl;
input wire [WIDTH-1:0] d;
output wire  [WIDTH-1:0] q;
 `ifdef SINTESI_RTL
   reg [WIDTH-1:0] x;
    genvar i;
    generate
      for (i=0; i< WIDTH; i=i+1) begin
        always @ (rst or enl[i] or d[i]) begin
          if (~rst) begin
            x[i] = 0;
          end else begin
            if (~enl[i]) begin
              x[i] = d[i];
            end
          end
        end
      end
    endgenerate
    assign #(LATCH_DELAY) q = x;
 `else
    genvar i;
    generate
      for ( i=0; i < WIDTH; i=i+1 )  begin : Reg_x_latch
        GTECH_LD4_1 mygate_latch_enh_rstn ( .GN(enl[i]), .D(d[i]), .CD(rst), .Q(q[i]) );
      end
    endgenerate
 `endif
endmodule
// ... existing code ...
module generic_reg_posedge (clk, rst, d, q);
parameter WIDTH = 16;
parameter FF_DELAY = `FlipFlopDelayR;
input wire rst;
input wire clk;
input wire [WIDTH-1:0] d;
output wire  [WIDTH-1:0] q;
 `ifdef SINTESI_RTL
   reg [WIDTH-1:0] x;
     always @ (posedge clk or negedge rst) begin
       if (~rst) begin
         x = 0;
       end else begin
         x = d;
       end
     end
     assign #(FF_DELAY) q = x;
 `else
    genvar i;
    generate
      for (i=0; i< WIDTH; i=i+1) begin: Data_reg
        GTECH_FD2 mygate_reg_posedge ( .CP(clk), .D(d[i]), .CD(rst), .Q(q[i]) );
      end
    endgenerate
 `endif
endmodule
// ... existing code ...
module generic_reg_posedge_b (clk, rst, d, q, qn);
parameter WIDTH = 16;
parameter FF_DELAY = `FlipFlopDelayR;
input wire rst;
input wire clk;
input wire [WIDTH-1:0] d;
output wire  [WIDTH-1:0] q;
output wire  [WIDTH-1:0] qn;
 `ifdef SINTESI_RTL
   reg [WIDTH-1:0] x;
     always @ (posedge clk or negedge rst) begin
       if (~rst) begin
         x = 0;
       end else begin
         x = d;
       end
     end
     assign #(FF_DELAY) q = x;
     assign #(FF_DELAY) qn = ~ x;
 `else
    genvar i;
    generate
      for (i=0; i< WIDTH; i=i+1) begin: Data_reg
        GTECH_FD2 mygate_reg_posedge ( .CP(clk), .D(d[i]), .CD(rst), .Q(q[i]), .QN(qn[i]) );
      end
    endgenerate
 `endif
endmodule
// ... existing code ...
module generic_reg_posedge_setl (clk, setl, d, q);
parameter WIDTH = 16;
parameter FF_DELAY = `FlipFlopDelayR;
input wire setl;
input wire clk;
input wire [WIDTH-1:0] d;
output wire  [WIDTH-1:0] q;
 `ifdef SINTESI_RTL
   reg [WIDTH-1:0] x;
     always @ (posedge clk or negedge setl) begin
       if (~setl) begin
         x = 1;
       end else begin
         x = d;
       end
     end
     assign #(FF_DELAY) q = x;
 `else
    genvar i;
    generate
      for (i=0; i< WIDTH; i=i+1) begin: Data_reg
        GTECH_FD4 reg_posedge ( .CP(clk), .D(d[i]), .SD(setl), .Q(q[i]) );
      end
    endgenerate
 `endif
endmodule
// ... existing code ...
module toggle_ff_posedge (clk, rst, t, q);
  parameter xorDelay = `xorDelay;
  input wire clk;
  input wire rst;
  input wire t;
  output wire q;
  wire d;
  generic_reg_posedge #(.WIDTH(1)) ffd (.clk(clk), .rst(rst), .d(d), .q(q));
  `ifdef SINTESI_RTL
    assign #(xorDelay) d = q ^ t;
  `else
    GTECH_XOR2 feedback_xor (.A(q), .B(t), .Z(d));
  `endif
endmodule
// ... existing code ...
module toggle_ff_posedge_hold (clk, rst, t, nrz, q);
  parameter xorDelay = `xorDelay;
  input wire clk;
  input wire rst;
  input wire t;
  input wire nrz;
  output wire q;
  wire d;
  generic_reg_posedge #(.WIDTH(1)) ffd (.clk(clk), .rst(rst), .d(d), .q(q));
  `ifdef SINTESI_RTL
assign #(xorDelay) d = nrz ^ t;
  `else
    GTECH_XOR2 feedback_xor (.A(nrz), .B(t), .Z(d));
  `endif
endmodule
// ... existing code ...
module toggle_ff_posedge_counter (clk, rst, w, t, d, q, qn);
  parameter xorDelay = `xorDelay;
  parameter orDelay = `orDelay;
  input wire clk;
  input wire rst;
  input wire w;
  input wire t;
  input wire d;
  output wire q;
  output wire qn;
  wire d_m, g;
    generic_reg_posedge_b #(.WIDTH(1)) ffd (.clk(clk), .rst(rst), .d(g), .q(q), .qn(qn));
    assign #(xorDelay) d_m = q ^ t;
    assign g = (d_m & (~w)) | (d & w);
endmodule

// From GENERIC_my_components.v (subset used)
// mousetrap_v4, mousetrap_with_and
// ... existing code ...
module mousetrap_v4 (
  rst,
  ReqIn,
  DataIn,
  AckIn,
  ReqOut,
  ReqOut_2,
  DataOut,
  AckOut,
  AckOut_4,
  d_c_setup,
  d_c_hold
) ;
  parameter xnorDelay = `xnorDelay;
  parameter nandDelay = `nandDelay;
  parameter notDelay = `notDelay;
  parameter WIDTH = 37;
  input  wire               rst;
  input  wire               ReqIn;
  input  wire [WIDTH-1:0]   DataIn;
  input  wire               AckOut;
  input  wire               AckOut_4;
  input  wire [2-1:0]      d_c_setup;
  input  wire [2-1:0]      d_c_hold;
  output wire               AckIn; 
  output wire               ReqOut_2;
  output wire               ReqOut; 
  output wire [WIDTH-1:0]   DataOut;
  wire ReqIn_d, AckIn_m, AckOut_d, en, re, xnor_out;
  wire [WIDTH-1:0] de;
generic_latch_enl_rstn #( .WIDTH(1)) 
lreq ( .rst(rst),
       .enl(en),
       .d(ReqIn_d),
       .q(re));
generic_latch_enl_rstn #( .WIDTH(WIDTH)) 
ldata ( .rst(rst),
        .enl(en),
        .d(DataIn),
        .q(de));
assign #(notDelay) ReqOut = ~xnor_out;
assign AckIn_m = re;
assign ReqOut_2 = re;
assign DataOut = de;
 `ifdef SINTESI_RTL
    assign #(xnorDelay) xnor_out = ~(re ^ AckOut_d);
    assign #(nandDelay) en = ~(xnor_out & AckOut_4);
 `else
    GTECH_XNOR2 mtp_xnor (.A(re), .B(AckOut_d), .Z(xnor_out));
    GTECH_NAND2	mtp_nand (.A(xnor_out), .B(AckOut_4), .Z(en));
 `endif
conf_delay_4to1
delay_m4_setup (.xin(ReqIn),
      .q(ReqIn_d),
      .cont(d_c_setup));
conf_delay_4to1
delay_m4_hold_in (.xin(AckOut),
      .q(AckOut_d),
      .cont(d_c_hold));
conf_delay_4to1
delay_m4_hold_out (.xin(AckIn_m),
      .q(AckIn),
      .cont(d_c_hold));
endmodule
// ... existing code ...
module mousetrap_with_and (
  rst,
  ReqIn,
  DataIn,
  AckIn,
  ReqOut,
  DataOut,
  AckOut,
  PRSReady
  );
  parameter xnorDelay = `xnorDelay;
  parameter nandDelay = `nandDelay;
  parameter WIDTH = 16;
  input  wire               rst;
  input  wire               ReqIn;
  input  wire [WIDTH-1:0]   DataIn;
  input  wire               AckOut;
  input  wire               PRSReady;
  output wire               AckIn; 
  output wire               ReqOut; 
  output wire [WIDTH-1:0]   DataOut;
  wire en, re, xnor_out;
  wire [WIDTH-1:0] de;
  generic_latch_enl_rstn #( .WIDTH(1)) 
  lreq ( .rst(rst),
         .enl(en),
         .d(ReqIn),
         .q(re));
  generic_latch_enl_rstn #( .WIDTH(WIDTH)) 
  ldata ( .rst(rst),
          .enl(en),
          .d(DataIn),
          .q(de));
  assign ReqOut = re;
  assign AckIn = re;
  assign DataOut = de;
  `ifdef SINTESI_RTL
    assign #(xnorDelay) xnor_out = ~(re ^ AckOut);
    assign #(nandDelay) en = ~(xnor_out & PRSReady);
  `else
    GTECH_XNOR2 mtp_xnor (.A(re), .B(AckOut), .Z(xnor_out));
    GTECH_NAND2 mtp_nand (.A(xnor_out), .B(PRSReady), .Z(en));
  `endif
endmodule

// From conf_delay.v (subset used)
// conf_delay_4to1
// ... existing code ...
module conf_delay_4to1 (xin, q, cont); 
parameter mux4x1Delay = `mux4x1Delay;
input wire xin;
input wire [1:0] cont;
output wire q;
reg   ge;
wire  out0, out1, out2, out3;
 `ifdef SINTESI_RTL
 assign #20  out0 = xin;
 assign #100 out1 = xin;
 assign #500 out2 = xin;
 assign #1000 out3 = xin;
always @(*)
case (cont) // synopsys full_case
2'b00: ge <= out0;
2'b01: ge <= out1;
2'b10: ge <= out2;
2'b11: ge <= out3;
default: ge <= out0;
endcase
assign #(mux4x1Delay) q = ge; 
 `else
delay_line_4to1 d_l (.xin(xin), .out0(out0), .out1(out1), .out2(out2), .out3(out3));
multiplexer_4to1
mux4 (
.cont0(cont[0]),
.cont1(cont[1]),
.out0(out0),
.out1(out1),
.out2(out2),
.out3(out3),
.q(q) );
 `endif
endmodule

// From Counters.v (subset used)
// counter_write, counter_write_PS, counter_read, chain
// ... existing code ...
module counter_write ( 
                        Req,
                        Ack,
                        write_pointer,
                        rst
                      );
  parameter xnorDelay = `xnorDelay;
  parameter OCCUPANCY = 4;
  input  wire                   rst;
  input  wire                   Req;
  input  wire                   Ack;
  output wire [OCCUPANCY-1:0]  write_pointer;
  wire clk;
  wire neg_rst;
  assign #(xnorDelay) clk = ~(Req ^ Ack);
  chain#( .OCCUPANCY(OCCUPANCY))  wp ( .pointer(write_pointer), .clk(clk), .rst(rst) );
endmodule
// ... existing code ...
module counter_write_PS ( 
                        Req,
                        Ack,
                        write_pointer,
                        write_pointer_o,
                        rst
                      );
  parameter xnorDelay = `xnorDelay;
  parameter OCCUPANCY = 8;
  input  wire                   rst;
  input  wire                   Req;
  input  wire                   Ack;
  output wire [OCCUPANCY-1:0]  write_pointer;
  output wire [1:0]  write_pointer_o;
  wire clk;
  wire neg_rst;
  assign #(xnorDelay) clk = ~(Req ^ Ack);
  assign write_pointer_o[0] = write_pointer[0];
  assign write_pointer_o[1] = write_pointer[OCCUPANCY-1];
  chain #( .OCCUPANCY(OCCUPANCY))  wp ( .pointer(write_pointer), .clk(clk), .rst(rst) );
endmodule
// ... existing code ...
module counter_read ( 
                      Req,
                      Ack,
                      read_pulse,
                      read_pointer,
                      mux_pointer,
                      rst
                    );
  parameter xorDelay = `xorDelay;
  parameter notDelay = `notDelay;
  parameter andDelay = `andDelay;
  parameter OCCUPANCY = 4;
  input  wire                   rst;
  input  wire                   Req;
  input  wire                   Ack;
  output wire                   read_pulse;
  output wire [OCCUPANCY-1:0]  read_pointer;
  output wire [OCCUPANCY-1:0]  mux_pointer;
  wire clk_0;
  wire clk_1;
  wire neg_rst;
  wire [OCCUPANCY-1:0]     int_pointer_0;
  wire [OCCUPANCY-1:0]     int_pointer_1;
  assign #(xorDelay) clk_0 = Req  ^ Ack;    
  assign #(notDelay) clk_1 = ~clk_0;
  assign read_pulse = clk_0;  
  chain #( .OCCUPANCY(OCCUPANCY)) rp_0 ( .pointer(int_pointer_0), .clk(clk_0), .rst(rst) );
  chain #( .OCCUPANCY(OCCUPANCY)) rp_1 ( .pointer(int_pointer_1), .clk(clk_1), .rst(rst) );
  assign mux_pointer  = int_pointer_1;
  assign #(andDelay) read_pointer = int_pointer_0 & int_pointer_1;
endmodule
// ... existing code ...
module chain    (
                  pointer,
                  clk,
                  rst
                );
  parameter OCCUPANCY = 4;
  input wire clk;
  input wire rst;
  output wire [OCCUPANCY-1:0] pointer;
  generic_reg_posedge_setl #( .WIDTH(1)) FF0 ( .clk(clk), .setl(rst), .d(pointer[OCCUPANCY-1]), .q(pointer[0]));
  generic_reg_posedge #( .WIDTH(OCCUPANCY-1)) FFothers ( .clk(clk), .rst(rst), .d(pointer[OCCUPANCY-2:0]), .q(pointer[OCCUPANCY-1:1]));  
endmodule

// From Buffers.v (subset used)
// buffers, buffers_PS
// ... existing code ...
module buffers  (
                  DataIn,
                  DataOut,
                  selector,
                  rst
                );
  parameter OCCUPANCY = 4;
  parameter WIDTH = 38;
  input  wire                           rst;
  input  wire [OCCUPANCY-1:0]           selector;
  input  wire [WIDTH-1:0]             DataIn;
  output wire [(WIDTH*OCCUPANCY)-1:0] DataOut;
  genvar i;
  generate
    for (i=0; i<OCCUPANCY; i=i+1) begin: buffers
    generic_latch_enh_rstn #( .WIDTH(WIDTH)) slot ( .d(DataIn), .q(DataOut[(WIDTH*(i+1))-1:WIDTH*i]), .enh(selector[i]), .rst(rst) );
    end
  endgenerate
endmodule
// ... existing code ...
module buffers_PS  (
                  DataIn,
                  DataOut,
                  selector,
                  rst
                );
  parameter OCCUPANCY = 8;
  input  wire                           rst;
  input  wire [OCCUPANCY-1:0]           selector;
  input  wire [OCCUPANCY-1:0]           DataIn;
  output wire [OCCUPANCY-1:0] 			DataOut;
  genvar i;
  generate
    for (i=0; i<OCCUPANCY; i=i+1) begin: buffers
    generic_latch_enh_rstn #( .WIDTH(1)) slot ( .d(DataIn[i]), .q(DataOut[i]), .enh(selector[i]), .rst(rst) );
    end
  endgenerate
endmodule

// From fifo_components.v (subset used)
// write_control
// ... existing code ...
module write_control (
                        rst,
                        Req_in,
                        Full,
                        Fulli1,
                        Empty,
                        Enable,
                        Write_pointer
                      );
parameter notDelay = `notDelay;
parameter norDelay = `norDelay;
parameter xnorDelay = `xnorDelay;
parameter andDelay = `andDelay;
parameter POSITION = 0;
parameter OCCUPANCY = `OCCUPANCY;
input  wire rst;
input  wire Req_in;
input  wire Fulli1;
input  wire Empty;
input  wire Write_pointer;
output wire Full;
output wire Enable;
wire matched_req;
wire SlotEnabled;
wire Enable_neg;
generate
  if ((OCCUPANCY % 2)==1) begin : match_odd
    if ((POSITION % 2)==0) assign /* synopsys translate_off */ #0 /* synopsys translate_on */ matched_req =  Req_in;
    if ((POSITION % 2)==1) assign /* synopsys translate_off */ #(notDelay) /* synopsys translate_on */ matched_req = ~Req_in;
  end else begin : match_even
    if ((POSITION % 2)==0) assign /* synopsys translate_off */ #(norDelay) /* synopsys translate_on */ matched_req =  Req_in ^ Fulli1;
    if ((POSITION % 2)==1) assign /* synopsys translate_off */ #(xnorDelay) /* synopsys translate_on */ matched_req =  ~(Req_in ^ Fulli1);
  end
endgenerate
generic_latch_enl_rstn  #( .WIDTH(1)) Full_reg ( .rst(rst), .enl(Enable_neg), .d(matched_req), .q(Full));
assign #(xnorDelay) SlotEnabled =  ~(Full ^ Empty);
assign #(andDelay) Enable = Write_pointer & SlotEnabled;
assign #(notDelay) Enable_neg = ~(Enable);
endmodule

// From tree.v (subset used)
// xor_tree
// ... existing code ...
module xor_tree (xin, z);
  parameter xorDelay = `xorDelay;
  parameter xor3Delay = `xor3Delay;
  parameter WIDTH = 4;
  input wire [WIDTH-1:0] xin;
  output wire z;
  generate
  if ( WIDTH==1 ) begin : xor1wh  
    assign z = xin[0];
  end
  if ( WIDTH==2 ) begin : xor2wh
    assign #(xorDelay) z = (xin[0]^xin[1]);
  end
  if ( WIDTH==3 ) begin : xor3wh
      assign #(xor3Delay) z = ^(xin);
  end
  if ( WIDTH==4 ) begin : xor4wh
      assign #(2*xorDelay) z = ^(xin);
  end
  if ( WIDTH==5 ) begin : xor5wh
      assign #(xor3Delay+xorDelay) z = ^(xin);
  end
  if ( WIDTH==6 ) begin : xor6wh
      assign #(xor3Delay+xorDelay) z = ^(xin);
  end
  if ( WIDTH==7 ) begin : xor7wh
      assign #(3*xorDelay) z = ^(xin);
  end
  if ( WIDTH==8 ) begin : xor8wh
      assign #(3*xorDelay) z = ^(xin);
  end
  if ( WIDTH==37 ) begin : xor37wh
      assign #(3*xorDelay+xor3Delay+xorDelay) z = ^(xin);
  end
  endgenerate
endmodule

// From multiplexer.v (subset used by FIFO)
// mux
// ... existing code ...
module mux (
             selector,
             Data_in,
             Data_out
           );
parameter mux2x1Delay = `mux2x1Delay;
parameter mux3x1Delay = `mux3x1Delay;
parameter mux4x1Delay = `mux4x1Delay;
parameter mux5x1Delay = `mux5x1Delay;
parameter mux6x1Delay = `mux6x1Delay;
parameter mux7x1Delay = `mux7x1Delay;
parameter mux8x1Delay = `mux8x1Delay;
  parameter DIMENSION = 4;
  parameter BITWIDTH = 38;
  input wire [DIMENSION-1:0] selector;
  input wire [(BITWIDTH*DIMENSION)-1:0] Data_in;
  output wire [BITWIDTH-1:0] Data_out;
  reg [DIMENSION-1:0] match;
  wire [BITWIDTH-1:0] Data_var[DIMENSION-1:0];
  reg [BITWIDTH-1:0] Data_out_x;
  integer loop;
  genvar i;
  generate 
    for (i=0; i<DIMENSION; i=i+1) begin: initialization
      assign Data_var[i] = Data_in[(BITWIDTH*(i+1))-1:BITWIDTH*i];
    end
  endgenerate
  generate 
    if (DIMENSION==37) begin: mux37
      always @(*)
        case (selector)
        37'b0000000000000000000000000000000000001: Data_out_x <= Data_var[0];
        37'b0000000000000000000000000000000000010: Data_out_x <= Data_var[1];
        37'b0000000000000000000000000000000000100: Data_out_x <= Data_var[2];
        37'b0000000000000000000000000000000001000: Data_out_x <= Data_var[3];
        37'b0000000000000000000000000000000010000: Data_out_x <= Data_var[4];
        37'b0000000000000000000000000000000100000: Data_out_x <= Data_var[5];
        37'b0000000000000000000000000000001000000: Data_out_x <= Data_var[6];
        37'b0000000000000000000000000000010000000: Data_out_x <= Data_var[7];
        37'b0000000000000000000000000000100000000: Data_out_x <= Data_var[8];
        37'b0000000000000000000000000001000000000: Data_out_x <= Data_var[9];
        37'b0000000000000000000000000010000000000: Data_out_x <= Data_var[10];
        37'b0000000000000000000000000100000000000: Data_out_x <= Data_var[11];
        37'b0000000000000000000000001000000000000: Data_out_x <= Data_var[12];
        37'b0000000000000000000000010000000000000: Data_out_x <= Data_var[13];
        37'b0000000000000000000000100000000000000: Data_out_x <= Data_var[14];
        37'b0000000000000000000001000000000000000: Data_out_x <= Data_var[15];
        37'b0000000000000000000010000000000000000: Data_out_x <= Data_var[16];
        37'b0000000000000000000100000000000000000: Data_out_x <= Data_var[17];
        37'b0000000000000000001000000000000000000: Data_out_x <= Data_var[18];
        37'b0000000000000000010000000000000000000: Data_out_x <= Data_var[19];
        37'b0000000000000000100000000000000000000: Data_out_x <= Data_var[20];
        37'b0000000000000001000000000000000000000: Data_out_x <= Data_var[21];
        37'b0000000000000010000000000000000000000: Data_out_x <= Data_var[22];
        37'b0000000000000100000000000000000000000: Data_out_x <= Data_var[23];
        37'b0000000000001000000000000000000000000: Data_out_x <= Data_var[24];
        37'b0000000000010000000000000000000000000: Data_out_x <= Data_var[25];
        37'b0000000000100000000000000000000000000: Data_out_x <= Data_var[26];
        37'b0000000001000000000000000000000000000: Data_out_x <= Data_var[27];
        37'b0000000010000000000000000000000000000: Data_out_x <= Data_var[28];
        37'b0000000100000000000000000000000000000: Data_out_x <= Data_var[29];
        37'b0000001000000000000000000000000000000: Data_out_x <= Data_var[30];
        37'b0000010000000000000000000000000000000: Data_out_x <= Data_var[31];
        37'b0000100000000000000000000000000000000: Data_out_x <= Data_var[32];
        37'b0001000000000000000000000000000000000: Data_out_x <= Data_var[33];
        37'b0010000000000000000000000000000000000: Data_out_x <= Data_var[34];
        37'b0100000000000000000000000000000000000: Data_out_x <= Data_var[35];
        37'b1000000000000000000000000000000000000: Data_out_x <= Data_var[36];
        37'b0000000000000000000000000000000000000: Data_out_x <= 0;
        default: Data_out_x <= 0;
      endcase
      assign Data_out = Data_out_x;
    end
    if (DIMENSION==8) begin: mux8
      always @(*)
        case (selector) // synopsys full_case
        8'b10000000: Data_out_x <= Data_var[7];
        8'b01000000: Data_out_x <= Data_var[6];
        8'b00100000: Data_out_x <= Data_var[5];
        8'b00010000: Data_out_x <= Data_var[4];
        8'b00001000: Data_out_x <= Data_var[3];
        8'b00000100: Data_out_x <= Data_var[2];
        8'b00000010: Data_out_x <= Data_var[1];
        8'b00000001: Data_out_x <= Data_var[0];
        8'b00000000: Data_out_x <= 0;
        default: Data_out_x <= 0;
      endcase
      assign #(mux8x1Delay) Data_out = Data_out_x;
    end
    if (DIMENSION==4) begin: mux4
      always @(*)
        case (selector) // synopsys full_case
        4'b1000: Data_out_x <= Data_var[3];
        4'b0100: Data_out_x <= Data_var[2];
        4'b0010: Data_out_x <= Data_var[1];
        4'b0001: Data_out_x <= Data_var[0];
        4'b0000: Data_out_x <= 0;
        default: Data_out_x <= 0;
      endcase
      assign #(mux4x1Delay) Data_out = Data_out_x;
    end
  endgenerate
endmodule

// From asynchronous_fifo.v (subset used)
// asynchronous_fifo_PS
// ... existing code ...
module asynchronous_fifo_PS (
                            ReqIn,
                            ReqOut,
                            AckOut,
                            AckIn,
                            DataIn,
                            DataOut,
                            FIFOfull,
                            write_pointer_o,
                            full_last,
                            rst
                          );
parameter xnorDelay = `xnorDelay;
parameter notDelay = `notDelay;
parameter OCCUPANCY = 8;
input  wire               rst;
input  wire               ReqIn;
input  wire [OCCUPANCY-1:0]   DataIn;
output wire               AckIn;
output wire               ReqOut;
output wire  DataOut;
input  wire               AckOut;
output wire               FIFOfull;
output wire [1:0]         write_pointer_o;
output wire               full_last;
wire [OCCUPANCY-1:0] Data_slots;
wire [OCCUPANCY:0] Full;
wire [OCCUPANCY:0] Empty;
wire [OCCUPANCY-1:0] Write_pointer;
wire [OCCUPANCY-1:0] Buffer_pointer;
wire [OCCUPANCY-1:0] Read_pointer;
wire [OCCUPANCY-1:0] Mux_pointer;
wire [OCCUPANCY-1:0] Req_int;
wire DataOut_x;
wire [OCCUPANCY-1:0] slot_free;
wire read_pulse, read_pulse_neg, AckDel;
genvar i;
  counter_write_PS #(.OCCUPANCY(OCCUPANCY)) cw ( .Req(ReqIn), .Ack(AckIn), .write_pointer(Write_pointer), .write_pointer_o(write_pointer_o), .rst(rst) );
  counter_read #(.OCCUPANCY(OCCUPANCY)) cr ( .Req(ReqOut), .Ack(AckDel), .read_pointer(Read_pointer), .mux_pointer(Mux_pointer), .read_pulse(read_pulse), .rst (rst));
  buffers_PS #(.OCCUPANCY(OCCUPANCY)) buffi ( .DataIn(DataIn), .DataOut(Data_slots), .selector(Buffer_pointer), .rst(rst) );
  mux #(.DIMENSION(OCCUPANCY), .BITWIDTH(1)) muuuux ( .selector(Mux_pointer), .Data_in(Data_slots), .Data_out(DataOut_x) );
  assign DataOut = DataOut_x;
  assign #(notDelay) Full[OCCUPANCY]  = ~Full[0];
  assign #(notDelay) Empty[OCCUPANCY] = ~Empty[0];
  generate
      for (i=0; i<OCCUPANCY; i=i+1) begin: control_blocks
        write_control #( .POSITION(i), .OCCUPANCY(OCCUPANCY) ) wc ( .rst(rst), .Req_in(ReqIn), .Full(Full[i]), .Fulli1(Full[i+1]), .Empty(Empty[i]), .Enable(Buffer_pointer[i]), .Write_pointer(Write_pointer[i]) );
        read_control #( .POSITION(i), .OCCUPANCY(OCCUPANCY) ) rc ( .rst(rst), .Req_out(Req_int[i]), .Full(Full[i]), .Ack_in(AckOut), .Empty(Empty[i]), .Emptyi1(Empty[i+1]), .Read_pointer(Read_pointer[i]) );
      end
  endgenerate
  xor_tree #(.WIDTH(OCCUPANCY)) rd_xor (.xin(Req_int), .z(ReqOut));
  xor_tree #(.WIDTH(OCCUPANCY)) wr_xor (.xin(Full[OCCUPANCY-1:0]), .z(AckIn));
  assign full_last = Full[OCCUPANCY-1];
  assign #(notDelay) read_pulse_neg = ~read_pulse;
  generic_latch_enl_rstn #( .WIDTH(1)) ackblock ( .rst(rst), .enl(read_pulse_neg), .d(AckOut), .q(AckDel));
  assign #(xnorDelay) slot_free = ~(Full ^ Empty);
  assign #100 FIFOfull = ~|{slot_free[OCCUPANCY-1:0]};
endmodule

// From fifo_components.v (subset used by asynchronous_fifo_PS)
// read_control
// ... existing code ...
module read_control (
                        rst,
                        Req_out,
                        Full,
                        Ack_in,
                        Empty,
                        Emptyi1,
                        Read_pointer
                      );
parameter notDelay = `notDelay;
parameter norDelay = `norDelay;
parameter xorDelay = `xorDelay;
parameter xnorDelay = `xnorDelay;
parameter POSITION = 0;
parameter OCCUPANCY = `OCCUPANCY;
input  wire rst;
input  wire Emptyi1;
input  wire Read_pointer;
input  wire Full;
input  wire Ack_in;
output wire Req_out;
output wire Empty;
wire matched_ack;
wire waiting;
wire RP_neg;
generate
  if ((OCCUPANCY % 2)==1) begin : match_odd
    if ((POSITION % 2)==0) assign matched_ack =  Ack_in;
    if ((POSITION % 2)==1) assign #(notDelay) matched_ack = ~Ack_in;
  end else begin : match_even
    if ((POSITION % 2)==0) assign #(xorDelay) matched_ack =  Ack_in ^ Emptyi1;
    if ((POSITION % 2)==1) assign #(xnorDelay) matched_ack =  ~(Ack_in ^ Emptyi1);
  end
endgenerate
assign #(xnorDelay) waiting = ~(Req_out ^ Empty);
assign #(notDelay) RP_neg = ~Read_pointer;
generic_latch_enl_rstn  #( .WIDTH(1)) ReqOut_reg ( .rst(rst), .enl(RP_neg), .d(Full), .q(Req_out));
generic_latch_enl_rstn  #( .WIDTH(1)) Empty_reg ( .rst(rst), .enl(waiting), .d(matched_ack), .q(Empty) );
endmodule

// From GENERIC_mullerc.v (subset used)
// MullerC2, MullerC2Rn (used in some components referenced above)
// ... existing code ...
module MullerC2 (Q, A, B);
  parameter mullerCDelay = `mullerCDelay;
  input  A, B;
  output Q;
  `ifdef SINTESI_RTL
    reg net_0;
    always @(*)
      case ({A,B}) // synopsys full_case
      2'b00: net_0 <= 1'b0;
      2'b11: net_0 <= 1'b1;
      2'b01: net_0 <= Q;
      2'b10: net_0 <= Q;
    endcase
    assign #(mullerCDelay) Q = net_0;
  `else
    wire net_0;
    GTECH_AOI222 Cel ( .Z(net_0), .A(A), .B(B), .C(A), .D(Q), .E(B), .F(Q) );
    assign Q = ~net_0;
  `endif
endmodule
// ... existing code ...
module MullerC2Rn (Q, A, B, RN);
  parameter mullerCDelay = `mullerCDelay;
  output  Q;
  input  A, B, RN;
  `ifdef SINTESI_RTL
    reg net_0;
    wire net_1;
    always @(*)
      case ({A,B}) // synopsys full_case
      2'b00: net_0 <= 1'b0;
      2'b11: net_0 <= 1'b1;
      2'b01: net_0 <= net_1;
      2'b10: net_0 <= net_1;
    endcase
    assign #(mullerCDelay) net_1 = net_0;
    assign Q = RN & (~net_1);
  `else
    wire net_0, net_1;
    GTECH_AOI222 Cel ( .Z(net_0), .A(A), .B(B), .C(A), .D(net_1), .E(B), .F(net_1) );
    assign Q = net_0 & RN;
    assign net_1 = ~Q;
  `endif
endmodule


