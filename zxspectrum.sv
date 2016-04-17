//============================================================================
// Sinclair ZX Spectrum host board
// 
//  Port to MIST board. 
//  Copyright (C) 2015 Sorgelig
//
//  Based on sample ZX Spectrum code by Goran Devic
//
//  This program is free software; you can redistribute it and/or modify it
//  under the terms of the GNU General Public License as published by the Free
//  Software Foundation; either version 2 of the License, or (at your option)
//  any later version.
//
//  This program is distributed in the hope that it will be useful, but WITHOUT
//  ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
//  FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
//  more details.
//
//  You should have received a copy of the GNU General Public License along
//  with this program; if not, write to the Free Software Foundation, Inc.,
//  51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
//============================================================================
module zxspectrum
(
   input         CLOCK_27,   // Input clock 27 MHz

   output  [5:0] VGA_R,
   output  [5:0] VGA_G,
   output  [5:0] VGA_B,
   output        VGA_HS,
   output        VGA_VS,

   output        LED,

   output        AUDIO_L,
   output        AUDIO_R,

   input         SPI_SCK,
   output        SPI_DO,
   input         SPI_DI,
   input         SPI_SS2,
   input         SPI_SS3,
   input         SPI_SS4,
   input         CONF_DATA0,

   output [12:0] SDRAM_A,
   inout  [15:0] SDRAM_DQ,
   output        SDRAM_DQML,
   output        SDRAM_DQMH,
   output        SDRAM_nWE,
   output        SDRAM_nCAS,
   output        SDRAM_nRAS,
   output        SDRAM_nCS,
   output  [1:0] SDRAM_BA,
   output        SDRAM_CLK,
   output        SDRAM_CKE
);
`default_nettype none

assign      LED = ~(divmmc_sd_activity | ioctl_erasing | ioctl_download);


//////////////////   MIST ARM I/O   ///////////////////
wire        PS2_CLK;
wire        PS2_DAT;

wire  [7:0] joystick_0;
wire  [7:0] joystick_1;
wire  [1:0] buttons;
wire  [1:0] switches;
wire        scandoubler_disable;
wire  [7:0] status;

wire [31:0] sd_lba;
wire        sd_rd;
wire        sd_wr;
wire        sd_ack;
wire        sd_conf;
wire        sd_sdhc;
wire  [7:0] sd_dout;
wire        sd_dout_strobe;
wire  [7:0] sd_din;
wire        sd_din_strobe;

reg  [10:0] clk14k_div;
wire        clk_ps2 = clk14k_div[10];
always @(posedge clk_sys) clk14k_div <= clk14k_div + 1'b1;

user_io #(.STRLEN(120)) user_io
(
	.*,
	.conf_str
	(
        "SPECTRUM;CSW;O5,Autoload ESXDOS,No,Yes;O2,CPU Speed,3.5MHz,4MHz;O3,Video Type,ZX,Pent;O6,Video Version,48k,128k;T4,Reset"
	),

	// ps2 keyboard emulation
	.ps2_clk(clk_ps2),				// 12-16khz provided by core
	.ps2_kbd_clk(PS2_CLK),
	.ps2_kbd_data(PS2_DAT),

	// unused
	.joystick_analog_0(),
	.joystick_analog_1(),
	.ps2_mouse_clk(),
	.ps2_mouse_data(),
	.serial_data(),
	.serial_strobe()
);

wire        ioctl_wr;
wire [24:0] ioctl_addr;
wire  [7:0] ioctl_dout;
wire        ioctl_download;
wire        ioctl_erasing;
wire  [4:0] ioctl_index;
reg         force_erase = 0;

data_io data_io
(
	.sck(SPI_SCK),
	.ss(SPI_SS2),
	.sdi(SPI_DI),

	.force_erase(force_erase),
	.downloading(ioctl_download),
	.erasing(ioctl_erasing),
	.index(ioctl_index),

	.clk(clk_sys),
	.wr(ioctl_wr),
	.addr(ioctl_addr),
	.dout(ioctl_dout)
);


///////////////////   CPU   ///////////////////
wire [15:0] addr;
wire  [7:0] cpu_din;
wire  [7:0] cpu_dout;
wire        nM1;
wire        nMREQ;
wire        nIORQ;
wire        nRD;
wire        nWR;
wire        nRFSH;
wire        nHALT;
wire        nBUSACK;
wire        nINT;
wire        nWAIT	 = 1;
wire        nNMI   = esxNMI;
wire        nBUSRQ = ~(ioctl_download | ioctl_erasing);
wire        nRESET = locked & ~buttons[1] & ~status[0] & ~status[4] & esxRESET & ~cold_reset & ~warm_reset & ~test_reset;

T80a cpu
(
	.RESET_n(nRESET),
	.CLK_n(clk_cpu),
	.WAIT_n(nWAIT),
	.INT_n(nINT),
	.NMI_n(nNMI),
	.BUSRQ_n(nBUSRQ),
	.M1_n(nM1),
	.MREQ_n(nMREQ),
	.IORQ_n(nIORQ),
	.RD_n(nRD),
	.WR_n(nWR),
	.RFSH_n(nRFSH),
	.HALT_n(nHALT),
	.BUSAK_n(nBUSACK),
	.A(addr),
	.DO(cpu_dout),
	.DI(cpu_din),
	.RestorePC_n(1),
	.RestorePC(0),
	.RestoreINT(0)
);

always_comb begin
	casex({nMREQ, ~nM1 | nIORQ | nRD, divmmc_sel, addr[7:0]==8'h1F})
		'b0XXX: cpu_din = ram_dout;
		'b101X: cpu_din = divmmc_dout;
		'b1001: cpu_din = {2'b00, joystick_0[5:0] | joystick_1[5:0]};
		'b1000: cpu_din = ula_dout;
		'b11XX: cpu_din = 8'hFF;
	endcase
end


//////////////////   MEMORY   //////////////////
wire vram_we = ((addr[15:13] == 3'b010) | ((addr[15:13] == 3'b110) & page_ram[2] & page_ram[0])) & ~nMREQ & ~nWR;
vram vram
(
    .clock(clk_sys),

    .wraddress({addr[15] & page_ram[1], addr[12:0]}),
    .data(cpu_dout),
    .wren(vram_we),

    .rdaddress({page_scr, vram_addr}),
    .q(vram_dout)
);

wire        dma = (~nRESET | ~nBUSACK) & ~nBUSRQ;
reg  [24:0] ram_addr;
reg   [7:0] ram_din;
reg         ram_we;
reg         ram_rd;
always_comb begin
	casex({dma, tape_req, ext_ram, addr[15:14]})
		'b1XX_XX: ram_addr = ioctl_addr;
		'b01X_XX: ram_addr = tape_addr;
		'b001_00: ram_addr = divmmc_addr;
		'b000_00: ram_addr = {5'h17, page_rom, addr[13:0]};
		'b00X_01: ram_addr = {       3'd5,     addr[13:0]};
		'b00X_10: ram_addr = {       3'd2,     addr[13:0]};
		'b00X_11: ram_addr = {       page_ram, addr[13:0]};
	endcase

	casex({dma, tape_req})
		'b1X: ram_din = ioctl_dout;
		'b01: ram_din = 0;
		'b00: ram_din = cpu_dout;
	endcase

	casex({dma, tape_req})
		'b1X: ram_we = ioctl_wr;
		'b01: ram_we = 0;
		'b00: ram_we = (ext_ram_write | addr[15] | addr[14]) & ~nMREQ & ~nWR;
	endcase

	casex({dma, tape_req})
		'b1X: ram_rd = 0;
		'b01: ram_rd = tape_rd;
		'b00: ram_rd = ~nMREQ & ~nRD;
	endcase
end

wire  [7:0] ram_dout;
sram ram
(
	.*,
	.init(~locked),
	.clk_sdram(clk_ram),
	.dout(ram_dout),
	.din (ram_din),
	.addr(ram_addr),
	.we(ram_we),
	.rd(ram_rd)
);

reg         test_rom;
reg   [7:0] page_reg     = 0;
wire        page_disable = page_reg[5];
wire  [1:0] page_rom     = {~test_rom, page_reg[4] & ~test_rom};
wire        page_scr     = page_reg[3];
wire  [2:0] page_ram     = page_reg[2:0];
wire        page_write   = ~nIORQ & ~nWR & nM1 & ~addr[15] & ~addr[1] & ~page_disable;

always @ (negedge clk_cpu) begin
	if(~nRESET) begin
		page_reg <= 0;
		test_rom <= test_reset;
	end else if(page_write) begin
		page_reg <= cpu_dout;
	end
end


///////////////////   ULA   ///////////////////
wire        locked;
wire        clk_cpu;       // CPU clock of 3.5 MHz
wire        clk_ram;       // 84MHz clock for RAM 
wire        clk_sys;       // 28MHz for system synchronization 
wire        clk_ula;       // 14MHz
wire [12:0] vram_addr;
wire  [7:0] vram_dout;
wire  [7:0] ula_dout;
wire        F11;
wire        F1;
wire        warm_reset;
wire        cold_reset;
wire        test_reset;
reg         AUDIO_IN;

ula ula( .*, .din(cpu_dout), .dout(ula_dout), .turbo(status[2]), .mZX(~status[3]), .m128(status[6]));


//////////////////   DIVMMC   //////////////////
reg   [1:0] esxdos_downloaded = 1'b00;
wire        esxdos_ready = esxdos_downloaded[~status[5]];
wire        ext_ram = divmmc_active && esxdos_ready;
wire        ext_ram_write = ext_ram && (addr[15:13] == 3'b001);
wire [24:0] divmmc_addr = {6'b000011, divmmc_mapaddr};

wire        esxRESET = ~(esxRQ & ~esxdos_ready & esxdos_downloaded[0]) & !initRESET;
wire        esxNMI   = ~(esxRQ &  esxdos_ready);
reg         esxRQ    = 0;

always @(posedge clk_ps2) begin
	reg sRST1 = 0, sRST2 = 0;

	sRST1 <= F11 | joystick_0[7] | joystick_1[7];
	sRST2 <= sRST1;

	if(sRST2 && ~sRST1) esxRQ <= 1;
		else esxRQ <= 0;
end

// wait for ESXDOS ROM loading 
integer initRESET = 32000000;
always @(posedge clk_sys) if(initRESET) initRESET <= initRESET - 1;

always @(negedge esxRQ, posedge cold_reset) begin
	if(cold_reset) esxdos_downloaded[1] <= 0;
		else esxdos_downloaded[1] <= esxdos_downloaded[0];
end

wire        divmmc_sd_activity;
wire        divmmc_active;
wire        divmmc_sel;
wire [18:0] divmmc_mapaddr;
wire  [7:0] divmmc_dout;

divmmc divmmc
(
	.*,
	.clk(clk_sys),

	.enabled(esxdos_ready),
	.din(cpu_dout),
	.dout(divmmc_dout),

	.active(divmmc_active),
	.active_io(divmmc_sel),
	.mapped_addr(divmmc_mapaddr),

	.sd_activity(divmmc_sd_activity)
);

always @(posedge ioctl_wr) if(ioctl_addr == 25'h181fff) esxdos_downloaded[0] <= 1;
always @(posedge clk_sys) force_erase <= cold_reset;


///////////////////   TAPE   ///////////////////
wire        tape_req;
wire        tape_rd;
wire [24:0] tape_addr;

tape tape
(
	.clk(clk_sys),
	.reset(~nRESET),

	.audio_out(AUDIO_IN),
	.pause(F1),

	.downloading((ioctl_index == 1) && ioctl_download),
	.addr_in(ioctl_addr),

	.active(tape_req),
	.rd_en(~nRFSH),
	.rd(tape_rd),
	.addr_out(tape_addr),
	.din(ram_dout)
);

endmodule
