//altshift_taps CBX_SINGLE_OUTPUT_FILE="ON" INTENDED_DEVICE_FAMILY=""MAX 10"" LPM_HINT="RAM_BLOCK_TYPE=M9K" LPM_TYPE="altshift_taps" NUMBER_OF_TAPS=1 TAP_DISTANCE=0 WIDTH=24 clken clock shiftin shiftout taps
//VERSION_BEGIN 16.1 cbx_mgl 2016:10:24:15:05:03:SJ cbx_stratixii 2016:10:24:15:04:16:SJ cbx_util_mgl 2016:10:24:15:04:16:SJ  VERSION_END
// synthesis VERILOG_INPUT_VERSION VERILOG_2001
// altera message_off 10463



// Copyright (C) 2016  Intel Corporation. All rights reserved.
//  Your use of Intel Corporation's design tools, logic functions 
//  and other software and tools, and its AMPP partner logic 
//  functions, and any output files from any of the foregoing 
//  (including device programming or simulation files), and any 
//  associated documentation or information are expressly subject 
//  to the terms and conditions of the Intel Program License 
//  Subscription Agreement, the Intel Quartus Prime License Agreement,
//  the Intel MegaCore Function License Agreement, or other 
//  applicable license agreement, including, without limitation, 
//  that your use is for the sole purpose of programming logic 
//  devices manufactured by Intel and sold by Intel or its 
//  authorized distributors.  Please refer to the applicable 
//  agreement for further details.



//synthesis_resources = altshift_taps 1 
//synopsys translate_off
`timescale 1 ps / 1 ps
//synopsys translate_on
module  mgtvi
	( 
	clken,
	clock,
	shiftin,
	shiftout,
	taps) /* synthesis synthesis_clearbox=1 */;
	input   clken;
	input   clock;
	input   [23:0]  shiftin;
	output   [23:0]  shiftout;
	output   [23:0]  taps;

	wire  [23:0]   wire_mgl_prim1_shiftout;
	wire  [23:0]   wire_mgl_prim1_taps;

	altshift_taps   mgl_prim1
	( 
	.clken(clken),
	.clock(clock),
	.shiftin(shiftin),
	.shiftout(wire_mgl_prim1_shiftout),
	.taps(wire_mgl_prim1_taps)
	// synopsys translate_off
	,
	.sclr(1'b0)
	// synopsys translate_on
	);
	defparam
		mgl_prim1.intended_device_family = ""MAX 10"",
		mgl_prim1.lpm_type = "altshift_taps",
		mgl_prim1.number_of_taps = 1,
		mgl_prim1.tap_distance = 0,
		mgl_prim1.width = 24,
		mgl_prim1.lpm_hint = "RAM_BLOCK_TYPE=M9K";
	assign
		shiftout = wire_mgl_prim1_shiftout,
		taps = wire_mgl_prim1_taps;
endmodule //mgtvi
//VALID FILE
