`include "specs.v"

module PSP ( 
				    ReqIn_PS,
				    AckIn_PS,
				    DataIn_PS,
				    d_m_setup,
				    d_m_hold,
				    d_ps_hold,
				    ReqOut_PS,
				    AckOut_PS,
				    DataOut_PS,

					ReqIn_SP,
					AckIn_SP,
					DataIn_SP,
					ReqOut_SP,
					AckOut_SP,
					DataOut_SP,

				    rst             
				   );

parameter BW = 37;    // Number of bitwidth

input wire rst, ReqIn_PS, AckOut_PS, ReqIn_SP, AckOut_SP;

input wire [1:0] d_m_setup;
input wire [1:0] d_m_hold;
input wire [1:0] d_ps_hold;

input wire [BW-1:0] DataIn_PS;
input wire DataIn_SP;

output wire AckIn_PS, ReqOut_PS, ReqOut_SP, AckIn_SP;

output wire DataOut_PS;
output wire [BW-1:0] DataOut_SP;


PtoS  #( .OCCUPANCY(BW)) ps (
    .ReqIn(ReqIn_PS),
	.AckIn(AckIn_PS),
	.DataIn(DataIn_PS),
    .d_m_setup(d_m_setup),
	.d_m_hold(d_m_hold),
	.d_ps_hold(d_ps_hold),
	.ReqOut(ReqOut_PS),
	.AckOut(AckOut_PS),
	.DataOut(DataOut_PS),
    .rst(rst)
);

StoP  #( .OCCUPANCY(BW)) sp (
    .ReqIn(ReqIn_SP),
	.AckIn(AckIn_SP),      
	.DataIn(DataIn_SP),
	.ReqOut(ReqOut_SP),
	.AckOut(AckOut_SP),
	.DataOut(DataOut_SP),
    .d_m_setup(d_m_setup),
	.d_m_hold(d_m_hold),

    .rst(rst)
);

endmodule
