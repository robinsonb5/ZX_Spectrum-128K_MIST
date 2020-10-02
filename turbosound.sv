module turbosound
(
	input         CLK,		   // Global clock
	input         CE,        // PSG Clock enable
	input         RESET,	   // Chip RESET (set all Registers to '0', active high)
	input         BDIR,	   // Bus Direction (0 - read , 1 - write)
	input         BC,		   // Bus control
	input   [7:0] DI,	      // Data In
	output  [7:0] DO,	      // Data Out
	output [10:0] AUDIO_L,
	output [10:0] AUDIO_R,

	input  [7:0] IOA_in,
	output [7:0] IOA_out,

	input  [7:0] IOB_in,
	output [7:0] IOB_out
);

// AY1 selected by default
reg ay_select = 1'b1;

// Bus control for each AY chips
wire BC_0;
wire BC_1;

// Data outputs for each AY chips
wire [7:0] DO_0;
wire [7:0] DO_1;

// AY0 channel output data
wire [9:0] ay0_left;
wire [9:0] ay0_right;

// AY1 channel output data
wire [9:0] ay1_left;
wire [9:0] ay1_right;

always_ff @(posedge CLK or posedge RESET) begin
	if (RESET == 1'b1) begin
		// Select AY1 after reset
		ay_select <= 1'b1;
	end
	else if (BDIR && BC && DI[7:1] == 7'b1111111) begin
		// Select AY0 or AY1 according to lower bit of data register (1111 111N)
		ay_select <= DI[0];
	end
end

YM2149 ym2149_0
(
	.CLK(CLK),
	.ENA(CE),
	.RESET_L(!RESET),
	.I_BDIR(BDIR),
	.I_BC1(BC_0),
	.I_DA(DI),
	.O_DA(DO_0),
	.I_STEREO(1'b0),
	.O_AUDIO_L(ay0_left),
	.O_AUDIO_R(ay0_right),

	.I_IOA(),
	.O_IOA(),
	.I_IOB(),
	.O_IOB()
);

// AY1 (Default AY)
YM2149 ym2149_1
(
	.CLK(CLK),
	.ENA(CE),
	.RESET_L(!RESET),
	.I_BDIR(BDIR),
	.I_BC1(BC_1),
	.I_DA(DI),
	.O_DA(DO_1),
	.I_STEREO(1'b0),
	.O_AUDIO_L(ay1_left),
	.O_AUDIO_R(ay1_right),

	.I_IOA(IOA_in),
	.O_IOA(IOA_out),
	.I_IOB(IOB_in),
	.O_IOB(IOB_out)
);

assign BC_0 = ~ay_select & BC;
assign BC_1 = ay_select & BC;
assign DO = ay_select ? DO_1 : DO_0;

// Mix channel signals from both AY/YM chips (extending to 10 bits width to prevent clipping)
assign AUDIO_L = { 1'b0, ay0_left  } + { 1'b0, ay1_left  };
assign AUDIO_R = { 1'b0, ay0_right } + { 1'b0, ay1_right };


endmodule
