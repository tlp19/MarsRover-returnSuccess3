   module EEE_IMGPROC(
	// global clock & reset
	clk,
	reset_n,
	
	// mm slave
	s_chipselect,
	s_read,
	s_write,
	s_readdata,
	s_writedata,
	s_address,

	// stream sink
	sink_data,
	sink_valid,
	sink_ready,
	sink_sop,
	sink_eop,
	
	// streaming source
	source_data,
	source_valid,
	source_ready,
	source_sop,
	source_eop,
	
	// conduit
	mode
	
);


// global clock & reset
input	clk;
input	reset_n;

// mm slave
input							s_chipselect;
input							s_read;
input							s_write;
output	reg	[31:0]	s_readdata;
input	[31:0]				s_writedata;
input	[2:0]					s_address;


// streaming sink
input	[23:0]            	sink_data;
input								sink_valid;
output							sink_ready;
input								sink_sop;
input								sink_eop;

// streaming source
output	[23:0]			  	   source_data;
output								source_valid;
input									source_ready;
output								source_sop;
output								source_eop;

// conduit export
input                         mode;

////////////////////////////////////////////////////////////////////////
//
parameter IMAGE_W = 11'd640;
parameter IMAGE_H = 11'd480;
parameter MESSAGE_BUF_MAX = 256;
parameter MSG_INTERVAL = 6;
parameter BB_COL_DEFAULT = 24'h00ff00;


wire [7:0]   red, green, blue, grey;
wire [7:0]   red_out, green_out, blue_out;

wire         sop, eop, in_valid, out_ready;
////////////////////////////////////////////////////////////////////////

// Find boundary of cursor box
wire [23:0] cur_img;
assign cur_img = {red, green, blue};

//prev
wire [23:0] prev_img;
wire [7:0]  prev_red, prev_green, prev_blue;
assign prev_red = prev_img[23:16];
assign prev_green = prev_img[15:8];
assign prev_blue = prev_img[7:0];
//top img
wire [23:0] top_img;
wire [7:0]  top_red, top_green, top_blue;
assign top_red = top_img[23:16];
assign top_green = top_img[15:8];
assign top_blue = top_img[7:0];

wire [7:0]   conv_red, conv_green, conv_blue;
//convoution filter

assign conv_red = red/3 + prev_red/3 + top_red/3;
assign conv_green = green/3 + prev_green/3 + top_green/3;
assign conv_blue = blue/3 + prev_blue/3 + top_blue/3;

wire [23:0] taps_sig;
wire [23:0] taps_sig2;
wire [2:0] taps_sig3;

// Detect pink areas
integer ired;
always @( conv_red )
    ired = conv_red;

wire pink_detect; //Color range for pink ball
assign pink_detect = (conv_red[7]&conv_red[6]) & (conv_green[7]&~conv_green[6]) & ((conv_blue[7] & ~conv_blue[6] &~conv_blue[5])|(~conv_blue[7] & conv_blue[6] & conv_blue[5] & conv_blue[4]));

wire orange_detect; //Color range for orange ball
assign orange_detect = (conv_red[7]&conv_red[6]) & ((conv_green[7]&~conv_green[6]&conv_green[5]&conv_green[4])|(conv_green[7]&~conv_green[6])) & ((~conv_blue[7] & conv_blue[6] & ~conv_blue[5])|(~conv_blue[7] & ~conv_blue[6] & conv_blue[5] & conv_blue[4]));

wire blue_detect; //Color range for blue ball
assign blue_detect = (~conv_red[7]&conv_red[6]) & ((conv_green[7] & ~conv_green[6] & ~conv_green[5])|(conv_green[6] & conv_green[5] & conv_green[4])) & (conv_blue[7]|(conv_blue[6] & conv_blue[5]));

wire green_detect; //Color range for green ball
assign green_detect = (~conv_red[7]&conv_red[6]) & (conv_green[7]) & ((~conv_blue[7] & conv_blue[6] & ~conv_blue[5]));



// Detect red areas
wire red_detect;
//assign red_detect = (red[7]|red[6]) & ~(green[7]|green[6]) & ~(blue[7]|blue[6]);
//assign red_detect = (conv_red[7]) & (conv_blue[7]|(conv_blue[6] & conv_blue[5]));


// Highlight detected areas
reg [23:0] red_high;
assign grey = green[7:1] + red[7:2] + blue[7:2]; //Grey = green/2 + red/4 + blue/4
//assign red_high  =  (red_detect) ? {8'hff, 8'h0, 8'h0} : {grey, grey, grey};
//test conv filter
reg pink_last, pink_last2, orange_last, orange_last2,green_last,green_last2,blue_last,blue_last2;



always@(posedge clk) begin
	if (sop) begin
	end
	else if (in_valid) begin
		pink_last2 <= pink_last;
		pink_last <= pink_detect;
		orange_last2 <= orange_last;
		orange_last <= orange_detect;
		green_last2 <= green_last;
		green_last <= green_detect;
		blue_last2 <= blue_last;
		blue_last <= blue_detect;
		
	end
end

wire [11:0] cur_binary;
assign cur_binary =  {pink_last2, pink_last, pink_detect, orange_last2, orange_last, orange_detect,green_last2,green_last,green_detect, blue_last2,blue_last,blue_detect};
wire [11:0] above_binary;

always@(*) begin
	if(orange_detect |((x == 320) & (y ==240))) begin
		red_high = {8'he6, 8'h7d, 8'h14};
	end
	else if (pink_detect) begin
		red_high = {8'he6, 8'h14, 8'hd9};
	end 
	else if (green_detect) begin
		red_high = {8'h22, 8'ha2, 8'h03};
	end 
	else if (blue_detect) begin
		red_high = {8'h11, 8'h11, 8'ha2};
	end 
	else begin
		red_high = {conv_red, conv_green, conv_blue};
	end
end
	
// Show bounding box
reg [23:0] new_image;
wire pink_active;
assign pink_active = (x == pinkl) | (x == pinkr) | (y == pinkt) | (y == pinkb);

wire orange_active;
assign orange_active = (x == orangel) | (x == oranger) | (y == oranget) | (y == orangeb);

wire green_active;
assign green_active = (x == greenl) | (x == greenr) | (y == greent) | (y == greenb);

wire blue_active;
assign blue_active = (x == bluel) | (x == bluer) | (y == bluet) | (y == blueb);



always@(*) begin
	if(pink_active) begin
		new_image = {8'he6, 8'h14, 8'hd9};
	end
	else if (orange_active) begin
		new_image = {8'he6, 8'h7d, 8'h14};
	end
	else if (green_active) begin
		new_image = {8'h0, 8'h7d, 8'h0};
	end 
	else if (blue_active) begin
		new_image = {8'h0, 8'h00, 8'h7d};
	end 
	else begin
		new_image = red_high;
	end
end

// Switch output pixels depending on mode switch
// Don't modify the start-of-packet word - it's a packet discriptor
// Don't modify data in non-video packets
assign {red_out, green_out, blue_out} = (mode & ~sop & packet_video) ? new_image : {red,green,blue};

//Count valid pixels to tget the image coordinates. Reset and detect packet type on Start of Packet.
reg [10:0] x, y;
reg packet_video;
always@(posedge clk) begin
	if (sop) begin
		x <= 11'h0;
		y <= 11'h0;
		packet_video <= (blue[3:0] == 3'h0);
	end
	else if (in_valid) begin
		if (x == IMAGE_W-1) begin
			x <= 11'h0;
			y <= y + 11'h1;
		end
		else begin
			x <= x + 11'h1;
		end
	end
end

reg [23:0] colorspace;
always@(posedge clk) begin
	if ((x == 320) & (y ==240) & in_valid) begin	//Update bounds when the pixel is red
		colorspace <= cur_img;
	end
	if (sop & in_valid) begin	//Reset bounds on start of packet
		colorspace <= 24'h0;
	end
end


//Find first and last red pixels
reg [10:0] x_min, y_min, x_max, y_max,xpink_min, ypink_min, xpink_max, ypink_max,xorange_min, yorange_min, xorange_max, yorange_max,xgreen_min, ygreen_min, xgreen_max, ygreen_max,xblue_min, yblue_min, xblue_max, yblue_max;
always@(posedge clk) begin
	if (((pink_detect & pink_last & pink_last2) &(above_binary[11] & above_binary[10] & above_binary[9]) ) & in_valid) begin	//Update bounds when the pixel is pink
		if (x < xpink_min) xpink_min <= x;
		if (x > xpink_max) xpink_max <= x;
		if (y < ypink_min) ypink_min <= y;
		ypink_max <= y;
	end
	if ((orange_last & orange_last2 & orange_detect & (above_binary[8] & above_binary[7] & above_binary[6])) & in_valid) begin	//Update bounds when the pixel is orange
		if (x < xorange_min) xorange_min <= x;
		if (x > xorange_max) xorange_max <= x;
		if (y < yorange_min) yorange_min <= y;
		yorange_max <= y;
	end
	
		if ((green_last & green_last2 & green_detect ) & (above_binary[5] & above_binary[4] & above_binary[3]) & in_valid) begin	//Update bounds when the pixel is orange
		if (x < xgreen_min) xgreen_min <= x;
		if (x > xgreen_max) xgreen_max <= x;
		if (y < ygreen_min) ygreen_min <= y;
		ygreen_max <= y;
	end
	if ((blue_last & blue_last2 & blue_detect ) & (above_binary[2] & above_binary[1] & above_binary[0]) & in_valid) begin	//Update bounds when the pixel is orange
		if (x < xblue_min) xblue_min <= x;
		if (x > xblue_max) xblue_max <= x;
		if (y < yblue_min) yblue_min <= y;
		yblue_max <= y;
	end
	
	if (sop & in_valid) begin	//Reset bounds on start of packet
		x_min <= IMAGE_W-11'h1;
		x_max <= 0;
		y_min <= IMAGE_H-11'h1;
		y_max <= 0;
		
		xpink_min <= IMAGE_W-11'h1;
		xpink_max <= 0;
		ypink_min <= IMAGE_H-11'h1;
		ypink_max <= 0;
		
		xorange_min <= IMAGE_W-11'h1;
		xorange_max <= 0;
		yorange_min <= IMAGE_H-11'h1;
		yorange_max <= 0;
		
		xgreen_min <= IMAGE_W-11'h1;
		xgreen_max <= 0;
		ygreen_min <= IMAGE_H-11'h1;
		ygreen_max <= 0;
		
		xblue_min <= IMAGE_W-11'h1;
		xblue_max <= 0;
		yblue_min <= IMAGE_H-11'h1;
		yblue_max <= 0;
		
	end
end


//Process bounding box at the end of the frame.
reg [1:0] msg_state;
reg [10:0] pinkl, pinkr, pinkt, pinkb, orangel,oranger, oranget, orangeb, greenl,greenr, greent, greenb, bluel,bluer, bluet, blueb;
reg [7:0] frame_count;
always@(posedge clk) begin
	if (eop & in_valid & packet_video) begin  //Ignore non-video packets
		
		//Latch edges for display overlay on next frame
		
		pinkl <= xpink_min;
		pinkr <= xpink_max;
		pinkt <= ypink_min;
		pinkb <= ypink_max;
		
		orangel <= xorange_min;
		oranger <= xorange_max;
		oranget <= yorange_min;
		orangeb <= yorange_max;
		
		greenl <= xgreen_min;
		greenr <= xgreen_max;
		greent <= ygreen_min;
		greenb <= ygreen_max;
				
		bluel <= xblue_min;
		bluer <= xblue_max;
		bluet <= yblue_min;
		blueb <= yblue_max;
//
//
//
//
//		left <=  10'b1111111111;
//		right <= 10'b1111111111;
//		top <= 10'b1111111111;
//		bottom <=10'b1111111111;

		
		//Start message writer FSM once every MSG_INTERVAL frames, if there is room in the FIFO
		frame_count <= frame_count - 1;
		
		if (frame_count == 0 && msg_buf_size < MESSAGE_BUF_MAX - 3) begin
			msg_state <= 2'b01;
			frame_count <= MSG_INTERVAL-1;
		end
	end
	
	//Cycle through message writer states once started
	if (msg_state != 2'b00) msg_state <= msg_state + 2'b01;

end
	
//Generate output messages for CPU
reg [31:0] msg_buf_in; 
wire [31:0] msg_buf_out;
reg msg_buf_wr;
wire msg_buf_rd, msg_buf_flush;
wire [7:0] msg_buf_size;
wire msg_buf_empty;

`define RED_BOX_MSG_ID "RBB"

reg [31:0] outmsg; 


always@(*) begin	//Write words to FIFO as state machine advances
	case(msg_state)
		2'b00: begin
			msg_buf_in = 32'b0;
			msg_buf_wr = 1'b0;
		end
		2'b01: begin
		
			msg_buf_in = `RED_BOX_MSG_ID;	//Message ID
			msg_buf_wr = 1'b1;
		end
		2'b10: begin
			msg_buf_in = {5'b0,(pinkr-pinkl),  5'b0,(oranger-orangel)};
			//msg_buf_in =  {5'b0, x_min, 5'b0, y_min};
			msg_buf_wr = 1'b1;
		end
		2'b11: begin
			msg_buf_in = {5'b0,(greenr-greenl),5'b0,(bluer-bluel)}  ; //Bottom right coordinate
			msg_buf_wr = 1'b1;
		end
	endcase
end


//Output message FIFO
MSG_FIFO	MSG_FIFO_inst (
	.clock (clk),
	.data (msg_buf_in),
	.rdreq (msg_buf_rd),
	.sclr (~reset_n | msg_buf_flush),
	.wrreq (msg_buf_wr),
	.q (msg_buf_out),
	.usedw (msg_buf_size),
	.empty (msg_buf_empty)
	);


//Streaming registers to buffer video signal
STREAM_REG #(.DATA_WIDTH(26)) in_reg (
	.clk(clk),
	.rst_n(reset_n),
	.ready_out(sink_ready),
	.valid_out(in_valid),
	.data_out({red,green,blue,sop,eop}),
	.ready_in(out_ready),
	.valid_in(sink_valid),
	.data_in({sink_data,sink_sop,sink_eop})
);

STREAM_REG #(.DATA_WIDTH(26)) out_reg (
	.clk(clk),
	.rst_n(reset_n),
	.ready_out(out_ready),
	.valid_out(source_valid),
	.data_out({source_data,source_sop,source_eop}),
	.ready_in(source_ready),
	.valid_in(in_valid),
	.data_in({red_out, green_out, blue_out, sop, eop})
);



shiftreg1	shiftreg1_inst (
	.clken ( in_valid ),
	.clock ( clk ),
	.shiftin ( cur_img ),
	.shiftout ( prev_img ),
	.taps ( taps_sig )
	);

shiftreg2	shiftreg2_inst (
	.clken ( in_valid ),
	.clock ( clk ),
	.shiftin ( cur_img ),
	.shiftout ( top_img ),
	.taps ( taps_sig2 )
	);
binaryabove	binaryabove_inst (
	.clken ( in_valid ),
	.clock ( clk ),
	.shiftin ( cur_binary ),
	.shiftout ( above_binary ),
	.taps ( taps_sig )
	);


/////////////////////////////////
/// Memory-mapped port		 /////
/////////////////////////////////

// Addresses
`define REG_STATUS    			0
`define READ_MSG    				1
`define READ_ID    				2
`define REG_BBCOL					3

//Status register bits
// 31:16 - unimplemented
// 15:8 - number of words in message buffer (read only)
// 7:5 - unused
// 4 - flush message buffer (write only - read as 0)
// 3:0 - unused


// Process write

reg  [7:0]   reg_status;
reg	[23:0]	bb_col;

always @ (posedge clk)
begin
	if (~reset_n)
	begin
		reg_status <= 8'b0;
		bb_col <= BB_COL_DEFAULT;
	end
	else begin
		if(s_chipselect & s_write) begin
		   if      (s_address == `REG_STATUS)	reg_status <= s_writedata[7:0];
		   if      (s_address == `REG_BBCOL)	bb_col <= s_writedata[23:0];
		end
	end
end


//Flush the message buffer if 1 is written to status register bit 4
assign msg_buf_flush = (s_chipselect & s_write & (s_address == `REG_STATUS) & s_writedata[4]);


// Process reads
reg read_d; //Store the read signal for correct updating of the message buffer

// Copy the requested word to the output port when there is a read.
always @ (posedge clk)
begin
   if (~reset_n) begin
	   s_readdata <= {32'b0};
		read_d <= 1'b0;
	end
	
	else if (s_chipselect & s_read) begin
		if   (s_address == `REG_STATUS) s_readdata <= {16'b0,msg_buf_size,reg_status};
		if   (s_address == `READ_MSG) s_readdata <= {msg_buf_out};
		if   (s_address == `READ_ID) s_readdata <= 32'h1234EEE2;
		if   (s_address == `REG_BBCOL) s_readdata <= {8'h0, bb_col};
	end
	
	read_d <= s_read;
end

//Fetch next word from message buffer after read from READ_MSG
assign msg_buf_rd = s_chipselect & s_read & ~read_d & ~msg_buf_empty & (s_address == `READ_MSG);
						


endmodule

