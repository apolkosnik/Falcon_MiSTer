//============================================================================
//
//  Atari Falcon030 for MiSTer
//
//  CPU: AP68030 (rtl/AP68030).  Chipset modelled on the Hatari emulator.
//  Platform: the MiSTer core template (sys/).
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
//
//============================================================================

module emu
(
	`include "sys/emu_ports.vh"
);

///////// Default values for ports not used in this core /////////

assign USER_OUT = '1;
assign {UART_RTS, UART_DTR} = 0;
assign {SD_SCK, SD_MOSI, SD_CS} = 'Z;
assign {SDRAM_DQ, SDRAM_A, SDRAM_BA, SDRAM_CLK, SDRAM_CKE, SDRAM_DQML, SDRAM_DQMH, SDRAM_nWE, SDRAM_nCAS, SDRAM_nRAS, SDRAM_nCS} = 'Z;

assign VGA_SL = 0;
assign VGA_F1 = 0;
assign VGA_SCALER  = 0;
assign VGA_DISABLE = 0;
assign HDMI_FREEZE = 0;
assign HDMI_BLACKOUT = 0;
assign HDMI_BOB_DEINT = 0;

assign AUDIO_S = 1;
assign AUDIO_MIX = 0;

assign LED_POWER = 0;
assign BUTTONS = 0;

//////////////////////////////////////////////////////////////////

wire [1:0] ar = status[5:4];
assign VIDEO_ARX = (!ar) ? 12'd4 : (ar - 1'd1);
assign VIDEO_ARY = (!ar) ? 12'd3 : 12'd0;

// OSD
//   status[0]    reset
//   status[2:1]  monitor: VGA, RGB, TV, mono
//   status[3]    ST-RAM: 14 MB, 4 MB
//   status[5:4]  aspect ratio
//   status[6]    MIDI on the user port UART
//   status[7]    cold reset (clears the warm start flag)
`include "build_id.v"
localparam CONF_STR = {
	"Falcon;;",
	"F1,IMGROM,Load TOS;",
	"F2,STCIMGROM,Load Cartridge;",
	"S0,ST ,Floppy A:;",
	"S1,ST ,Floppy B:;",
	"S2,VHDIMGHDF,IDE Master;",
	"S3,VHDIMGHDF,IDE Slave;",
	"S4,VHDIMGHDF,SCSI 0;",
	"S5,VHDIMGHDF,SCSI 1;",
	"S6,ISOCUEBIN,SCSI 2 CD-ROM;",
	"-;",
	"O[2:1],Monitor,VGA,RGB,TV,Mono;",
	"O[3],ST-RAM,14 MB,4 MB;",
	"O[5:4],Aspect ratio,Original,Full Screen,[ARC1],[ARC2];",
	"O[6],UART,Serial,MIDI;",
	"-;",
	"T[0],Reset;",
	"T[7],Cold Reset;",
	"R[0],Reset and close OSD;",
	"v,0;",
	"V,v",`BUILD_DATE
};

wire forced_scandoubler;
wire   [1:0] buttons;
wire [127:0] status;
wire  [10:0] ps2_key;
wire  [24:0] ps2_mouse;
wire  [31:0] joy0, joy1;
wire  [64:0] rtc;

wire        ioctl_download;
wire [15:0] ioctl_index;
wire        ioctl_wr;
wire [26:0] ioctl_addr;
wire  [7:0] ioctl_dout;
wire        ioctl_wait;

// disk slots: 0 floppy A, 1 floppy B, 2 IDE master, 3 IDE slave, 4..6 SCSI ID 0..2
// (seven is the hps_io ceiling: the mount byte's bit 7 is the read-only flag)
wire  [6:0] img_mounted;
wire        img_readonly;
wire [63:0] img_size;
wire [31:0] sd_lba[7];
wire  [6:0] sd_rd, sd_wr, sd_ack;
wire [13:0] sd_buff_addr;
wire  [7:0] sd_buff_dout;
wire  [7:0] sd_buff_din[7];
wire        sd_buff_wr;

hps_io #(.CONF_STR(CONF_STR), .VDNUM(7)) hps_io
(
	.clk_sys(clk_sys),
	.HPS_BUS(HPS_BUS),
	.EXT_BUS(),
	.gamma_bus(),

	.forced_scandoubler(forced_scandoubler),

	.buttons(buttons),
	.status(status),
	.status_menumask(0),

	.ioctl_download(ioctl_download),
	.ioctl_index(ioctl_index),
	.ioctl_wr(ioctl_wr),
	.ioctl_addr(ioctl_addr),
	.ioctl_dout(ioctl_dout),
	.ioctl_wait(ioctl_wait),

	.img_mounted(img_mounted),
	.img_readonly(img_readonly),
	.img_size(img_size),
	.sd_lba(sd_lba),
	.sd_rd(sd_rd),
	.sd_wr(sd_wr),
	.sd_ack(sd_ack),
	.sd_buff_addr(sd_buff_addr),
	.sd_buff_dout(sd_buff_dout),
	.sd_buff_din(sd_buff_din),
	.sd_buff_wr(sd_buff_wr),

	.RTC(rtc),
	.joystick_0(joy0),
	.joystick_1(joy1),
	.ps2_key(ps2_key),
	.ps2_mouse(ps2_mouse)
);

///////////////////////   CLOCKS   ///////////////////////////////

wire clk_sys;   // 32 MHz: CPU, chipset, DSP, DDR3
wire pll_locked;
pll pll
(
	.refclk(CLK_50M),
	.rst(0),
	.outclk_0(clk_sys),
	.outclk_1(),
	.locked(pll_locked)
);

///////////////////////   RESET / ROM LOAD   ////////////////////

// ioctl index 0 is the boot.rom the MiSTer loads at start, 1 the OSD
// "Load TOS"; both go to $E00000.  Index 2 is a cartridge at $FA0000.
wire rom_download  = ioctl_download && (ioctl_index[5:0] <= 6'd1);
wire cart_download = ioctl_download && (ioctl_index[5:0] == 6'd2);
reg  rom_loaded = 0;
always @(posedge clk_sys) if (rom_download) rom_loaded <= 1;

reg [7:0] por_cnt = 0;
wire      por = ~&por_cnt;
always @(posedge clk_sys) if (por && pll_locked) por_cnt <= por_cnt + 1'd1;

wire cold_reset = por | status[7] | rom_download;
wire reset      = cold_reset | RESET | status[0] | buttons[1] | ioctl_download | ~rom_loaded;

wire [23:0] ld_addr = rom_download ? {5'b11100, ioctl_addr[18:0]} : {7'b1111101, ioctl_addr[16:0]};
wire        ld_busy;
assign ioctl_wait = ld_busy;

///////////////////////   SYSTEM    //////////////////////////////

wire [7:0] r, g, b;
wire       hs, vs, hblank, vblank, ce_pix;
wire signed [15:0] audio_l, audio_r;
wire       fdd_led, hdd_led;
wire       midi_tx, ser_tx;

falcon_system #(.CLK_HZ(32000000)) system
(
	.clk(clk_sys),
	.reset(reset),
	.cold_reset(cold_reset),
	.por(por),

	.ram_mb(status[3] ? 4'd4 : 4'd14),
	.monitor(status[2:1] == 2'd0 ? 2'b10 : status[2:1] == 2'd1 ? 2'b01 : status[2:1] == 2'd2 ? 2'b11 : 2'b00),

	.ld_wr(ioctl_wr & (rom_download | cart_download)),
	.ld_addr(ld_addr),
	.ld_data(ioctl_dout),
	.ld_busy(ld_busy),

	.ps2_key(ps2_key),
	.ps2_mouse(ps2_mouse),
	.joy0(joy0),
	.joy1(joy1),
	.rtc(rtc),

	.img_mounted(img_mounted),
	.img_readonly(img_readonly),
	.img_size(img_size),
	.sd_lba(sd_lba),
	.sd_rd(sd_rd),
	.sd_wr(sd_wr),
	.sd_ack(sd_ack),
	.sd_buff_addr(sd_buff_addr[8:0]),
	.sd_buff_dout(sd_buff_dout),
	.sd_buff_din(sd_buff_din),
	.sd_buff_wr(sd_buff_wr),

	.r(r), .g(g), .b(b),
	.hsync(hs), .vsync(vs), .hblank(hblank), .vblank(vblank), .ce_pix(ce_pix),

	.audio_l(audio_l),
	.audio_r(audio_r),

	.midi_rx(status[6] ? UART_RXD : 1'b1),
	.midi_tx(midi_tx),
	.ser_rx(status[6] ? 1'b1 : UART_RXD),
	.ser_tx(ser_tx),

	.fdd_led(fdd_led),
	.hdd_led(hdd_led),

	.DDRAM_BUSY(DDRAM_BUSY),
	.DDRAM_BURSTCNT(DDRAM_BURSTCNT),
	.DDRAM_ADDR(DDRAM_ADDR),
	.DDRAM_DOUT(DDRAM_DOUT),
	.DDRAM_DOUT_READY(DDRAM_DOUT_READY),
	.DDRAM_RD(DDRAM_RD),
	.DDRAM_DIN(DDRAM_DIN),
	.DDRAM_BE(DDRAM_BE),
	.DDRAM_WE(DDRAM_WE)
);

assign DDRAM_CLK = clk_sys;
assign UART_TXD  = status[6] ? midi_tx : ser_tx;

assign AUDIO_L = audio_l;
assign AUDIO_R = audio_r;

assign LED_USER = fdd_led;
assign LED_DISK = {1'b0, hdd_led};

///////////////////////   VIDEO    ///////////////////////////////

assign CLK_VIDEO = clk_sys;
assign CE_PIXEL  = ce_pix;

assign VGA_DE = ~(hblank | vblank);
assign VGA_HS = hs;
assign VGA_VS = vs;
assign VGA_R  = r;
assign VGA_G  = g;
assign VGA_B  = b;

endmodule
