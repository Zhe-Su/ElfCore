`include "specs.v"

module PSP_tb;

parameter period = 5000;
parameter matched_delay = 1000;

reg rst, ReqIn_PS, AckOut_PS, ReqIn_SP, AckOut_SP;

reg [1:0] d_m_setup;
reg [1:0] d_m_hold;
reg [1:0] d_ps_hold;

reg [36:0] DataIn_PS;
reg DataIn_SP;

wire AckIn_PS, ReqOut_PS, ReqOut_SP, AckIn_SP;

wire DataOut_PS;
wire [36:0] DataOut_SP;

integer f_in_PS, f_out_PS, f_in_SP, f_out_SP;

string in_ps_path, out_ps_path, in_sp_path, out_sp_path;

reg done_ps, done_sp;
integer drain_ps, drain_sp;

reg [36:0] packet_PS;
reg packet_SP;

reg [7:0] cnt;
reg [36:0] out_PS, out_SP;

PSP psp_module (
    .ReqIn_PS(ReqIn_PS),
	.AckIn_PS(AckIn_PS),
	.DataIn_PS(DataIn_PS),

    .d_m_setup(d_m_setup),
	.d_m_hold(d_m_hold),
	.d_ps_hold(d_ps_hold),

	.ReqOut_PS(ReqOut_PS),
	.AckOut_PS(AckOut_PS),
	.DataOut_PS(DataOut_PS),

    .ReqIn_SP(ReqIn_SP),
	.AckIn_SP(AckIn_SP),      
	.DataIn_SP(DataIn_SP),

	.ReqOut_SP(ReqOut_SP),
	.AckOut_SP(AckOut_SP),
	.DataOut_SP(DataOut_SP),

    .rst(rst)
);

// Centralized init/config
initial begin : TB_INIT
	d_m_setup = 2'b00;
	d_m_hold = 2'b00;
	d_ps_hold = 2'b00;
	ReqIn_PS = 0;
	AckOut_PS = 0;
	DataIn_PS = 'b0;
	ReqIn_SP = 0;
	AckOut_SP = 0;
	DataIn_SP = 'b0;
	cnt = 0;
	out_PS = 0;
	out_SP = 0;
	done_ps = 0;
	done_sp = 0;
	void'($value$plusargs("D_M_SETUP=%d", d_m_setup));
	void'($value$plusargs("D_M_HOLD=%d", d_m_hold));
	void'($value$plusargs("D_PS_HOLD=%d", d_ps_hold));
	drain_ps = 10*period;
	drain_sp = 2*period;
	void'($value$plusargs("DRAIN_PS=%d", drain_ps));
	void'($value$plusargs("DRAIN_SP=%d", drain_sp));
end

// Reset generation
initial begin : GEN_RESET
	rst = 0;
	#period;
	rst = 1;
	#period;
end

// Mark disabled tests as done to allow finish
initial begin : DISABLE_DONE
	if ($test$plusargs("DISABLE_PS")) done_ps = 1;
	if ($test$plusargs("DISABLE_SP")) done_sp = 1;
end

// Finish monitor with optional timeout
initial begin : FINISH_MON
	integer local_timeout;
	local_timeout = 0;
	void'($value$plusargs("TIMEOUT=%d", local_timeout));
	if (local_timeout > 0) fork
		#local_timeout $fatal(1, "Simulation TIMEOUT after %0d ps", local_timeout);
	join_none
	wait(done_ps && done_sp);
	$finish;
end


//test PS
initial begin : TEST_PS
\tif ($test$plusargs("DISABLE_PS")) disable TEST_PS;

	in_ps_path = "test_PS.txt";
	out_ps_path = "sim_output_PS.txt";
	void'($value$plusargs("IN_PS=%s", in_ps_path));
	void'($value$plusargs("OUT_PS=%s", out_ps_path));
	f_in_PS = $fopen(in_ps_path,"r");
	if (f_in_PS == 0) $fatal(1, "Failed to open IN_PS=%s", in_ps_path);
	f_out_PS = $fopen(out_ps_path,"w");
	if (f_out_PS == 0) $fatal(1, "Failed to open OUT_PS=%s", out_ps_path);
    rst = 0;
    ReqIn_PS = 0;
    DataIn_PS = 'b0;
    d_m_setup = 2'b00;
    d_m_hold = 2'b00;
    d_ps_hold = 2'b00;

    AckOut_PS = 0;
    ReqIn_SP = 0;
    DataIn_SP = 'b0;
    AckOut_SP = 0;
	cnt = 0;
	out_PS = 0;

    #period;
	rst = 1;
	#period;

    while (!$feof(f_in_PS)) begin
		$fscanf(f_in_PS, "%b", packet_PS);
			 	if (ReqIn_PS != AckIn_PS) begin
			 		@ (AckIn_PS)
			 		#period;
			 	end
			 	DataIn_PS = packet_PS;
			 	#matched_delay;
			 	ReqIn_PS = ~ReqIn_PS;
    end
    #drain_ps;
    $fclose(f_in_PS);
    $fclose(f_out_PS);
    done_ps = 1;
end

    always @ (ReqOut_PS)
if ($time>5000)
    begin
		#matched_delay;
		AckOut_PS = ~AckOut_PS;
        cnt = cnt+1;
        out_PS[cnt-1] = DataOut_PS;
		if (cnt == 'd37)
		begin
            cnt = 'd0;
			$fdisplay(f_out_PS, "packet %b;", out_PS, $time);
		end
    end



//test SP
initial begin : TEST_SP
\tif ($test$plusargs("DISABLE_SP")) disable TEST_SP;

	in_sp_path = "test_SP.txt";
	out_sp_path = "sim_output_SP.txt";
	void'($value$plusargs("IN_SP=%s", in_sp_path));
	void'($value$plusargs("OUT_SP=%s", out_sp_path));
	f_in_SP = $fopen(in_sp_path,"r");
	if (f_in_SP == 0) $fatal(1, "Failed to open IN_SP=%s", in_sp_path);
	f_out_SP = $fopen(out_sp_path,"w");
	if (f_out_SP == 0) $fatal(1, "Failed to open OUT_SP=%s", out_sp_path);
    rst = 0;
    ReqIn_PS = 0;
    DataIn_PS = 'b0;
    d_m_setup = 2'b00;
    d_m_hold = 2'b00;
    d_ps_hold = 2'b00;
    AckOut_PS = 0;
    ReqIn_SP = 0;
    DataIn_SP = 'b0;
    AckOut_SP = 0;
	out_PS = 0;
	out_SP = 0;

    #period;
	rst = 1;
	#period;

    while (!$feof(f_in_SP)) begin
		$fscanf(f_in_SP, "%b", packet_SP);
			 	if (ReqIn_SP != AckIn_SP) begin
			 		@ (AckIn_SP)
			 		#period;
			 	end
			 	DataIn_SP = packet_SP;
			 	#matched_delay;
			 	ReqIn_SP = ~ReqIn_SP;
    end
    #drain_sp;
    $fclose(f_in_SP);
    $fclose(f_out_SP);
    done_sp = 1;
end

    always @ (ReqOut_SP)
if ($time>5000)
    begin
		#matched_delay;
		AckOut_SP = ~AckOut_SP;
        out_SP = DataOut_SP;
		$fdisplay(f_out_SP, "packet %b;", out_SP, $time);
    end


endmodule
