`include "specs.v"

module StoP (
                            ReqIn,
                            ReqOut,
                            AckOut,
                            AckIn,
                            DataIn,
                            DataOut,
				    		d_m_setup,
				    		d_m_hold,
                            rst
                          );

parameter xnorDelay = `xnorDelay;
parameter notDelay = `notDelay;

parameter OCCUPANCY = 8;

input  wire               	rst;

input  wire[2-1:0] 		  	d_m_setup, d_m_hold;


input  wire               	ReqIn;
input  wire               	DataIn;
output wire               	AckIn;

output wire               	ReqOut;
output wire[OCCUPANCY-1:0]	DataOut;
input  wire               	AckOut;

wire [OCCUPANCY-1:0] 		Data_slots;

wire [OCCUPANCY:0] 			Full;
wire [OCCUPANCY:0] 			Empty;
wire [OCCUPANCY-1:0] 		Write_pointer;
wire [OCCUPANCY-1:0] 		Buffer_pointer;

wire AckIn_m, ReqOut_m, AckOut_d;
genvar i;

  counter_write #(.OCCUPANCY(OCCUPANCY)) cw ( .Req(ReqIn), .Ack(AckIn), .write_pointer(Write_pointer), .rst(rst) );

  buffers #(.OCCUPANCY(OCCUPANCY), .WIDTH(1)) buffi ( .DataIn(DataIn), .DataOut(Data_slots), .selector(Buffer_pointer), .rst(rst) );

  assign #(notDelay) Full[OCCUPANCY]  = ~Full[0];
  assign #(notDelay) Empty[OCCUPANCY] = ~Empty[0];

  generate
      for (i=0; i<OCCUPANCY; i=i+1) begin: control_blocks
    assign Empty[i] = AckIn_m;
    write_control #( .POSITION(i), .OCCUPANCY(OCCUPANCY) ) wc ( .rst(rst), .Req_in(ReqIn), .Full(Full[i]), .Fulli1(Full[i+1]), .Empty(Empty[i]), .Enable(Buffer_pointer[i]), .Write_pointer(Write_pointer[i]) );
      end
  endgenerate

  xor_tree #(.WIDTH(OCCUPANCY)) wr_xor (.xin(Full[OCCUPANCY-1:0]), .z(AckIn));

    mousetrap_with_and #(.WIDTH(OCCUPANCY))
    mtp ( 
         .rst(rst),
         .ReqIn(Full[OCCUPANCY-1]),
         .AckIn(AckIn_m),
         .DataIn(Data_slots),
         .ReqOut(ReqOut_m),
         .AckOut(AckOut_d),
         .DataOut(DataOut),
         .PRSReady(1'b1) );

conf_delay_4to1
delay_m_req (.xin(ReqOut_m),
      .q(ReqOut),
      .cont(d_m_setup));

conf_delay_4to1
delay_m_ack (.xin(AckOut),
      .q(AckOut_d),
      .cont(d_m_hold));


endmodule

