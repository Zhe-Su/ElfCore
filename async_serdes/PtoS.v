`include "specs.v"

module PtoS ( 
				    ReqIn,          //
				    AckIn,          //                              
				    DataIn,         //

				    d_ps_hold,
				    d_m_setup,
				    d_m_hold,
				    ReqOut,         //
				    AckOut,         // 
				    DataOut,        //

				    rst             //
				   );


	/// Declarations

	parameter andDelay = `andDelay;
	parameter orDelay = `orDelay;
	parameter notDelay = `notDelay;

	parameter OCCUPANCY = 37;

	// I/O
	input  wire ReqIn;
	input  wire AckOut;
	input  wire[ OCCUPANCY-1:0 ] DataIn; 
	input  wire rst;
	input  wire[2-1:0] d_m_setup, d_m_hold, d_ps_hold;

	output wire ReqOut;
	output wire AckIn;
	output wire DataOut;


	// Internal wires

	wire re, ack_mtp, re_4, ack_trigger, en_latch, en_latch_0, en_latch_1, rst_d;
    wire [1:0] write_pointer_o;
	wire full_last;
	wire Ack_fifo, Ack_fifo_d;
	wire Req_fifo, Req_fifo_d;
	wire[OCCUPANCY-1:0] Data_fifo;

mousetrap_v4 #( .WIDTH(OCCUPANCY) )
mtp_i ( 
     .rst(rst),
     .ReqIn(ReqIn),
     .AckIn(AckIn),
     .DataIn(DataIn),
     .ReqOut(re_4),
     .ReqOut_2(re),
     .AckOut(ack_mtp),
     .AckOut_4(1'b1),
     .DataOut(Data_fifo),
     .d_c_setup(d_m_setup),
     .d_c_hold(d_m_hold));

assign ack_trigger = write_pointer_o[0] & (~write_pointer_o[1]);

toggle_ff_posedge_hold
tog_ps (.clk(ack_trigger),
     .q(ack_mtp),
     .nrz(re),
     .t(1'b0),
     .rst(rst)); 

conf_delay_4to1
delay_ps_ack (.xin(Ack_fifo),
      .q(Ack_fifo_d),
      .cont(d_ps_hold));

assign Req_fifo = re_4 ^ Ack_fifo_d;

assign rst_d = re_4 & rst;

generic_reg_posedge #( .WIDTH(1)) d_reg_0 ( .rst(rst_d), .clk(full_last), .d(1'b1), .q(en_latch_0));
generic_reg_posedge #( .WIDTH(1)) d_reg_1 ( .rst(rst_d), .clk(~full_last), .d(1'b1), .q(en_latch_1));

assign en_latch = en_latch_0 | en_latch_1;

generic_latch_enl_rstn #( .WIDTH(1)) d_latch ( .rst(rst), .enl(en_latch), .d(Req_fifo), .q(Req_fifo_d));

asynchronous_fifo_PS #(.OCCUPANCY(OCCUPANCY))
	fifo ( .ReqIn(Req_fifo_d),
			.ReqOut(ReqOut),
			.AckIn(Ack_fifo),
			.AckOut(AckOut),
			.DataIn(Data_fifo),
			.DataOut(DataOut),
            .write_pointer_o(write_pointer_o),
			.full_last(full_last),
			.rst(rst) );

endmodule

