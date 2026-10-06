// tb_scsi_top.sv - Verilator top for the SCSI bench: the real falcon_fdc
// (ST DMA chip, EXT_SCSI = 1) wired to the real falcon_scsi (NCR 5380 and
// targets), exactly as falcon_system is expected to connect them.  The
// floppy image ports of falcon_fdc are tied off (no floppy images).

module tb_scsi_top (
	input             clk,
	input             reset,

	input             bus_cs,
	input             bus_stb,
	input             bus_we,
	input       [3:1] bus_addr,
	input             bus_uds,
	input             bus_lds,
	input      [15:0] bus_din,
	output     [15:0] bus_dout,
	output            bus_ack,
	output            bus_berr,
	output            irq,

	output            dma_req,
	output            dma_we,
	output     [23:1] dma_addr,
	output      [1:0] dma_be,
	output     [15:0] dma_wdata,
	input      [15:0] dma_rdata,
	input             dma_ack,

	input       [2:0] img_mounted,
	input             img_readonly,
	input      [63:0] img_size,
	output     [31:0] sd_lba0,
	output     [31:0] sd_lba1,
	output     [31:0] sd_lba2,
	output      [2:0] sd_rd,
	output      [2:0] sd_wr,
	input       [2:0] sd_ack,
	input      [13:0] sd_buff_addr,
	output      [5:0] sd_blk_cnt,
	input       [7:0] sd_buff_dout,
	output      [7:0] sd_buff_din0,
	output      [7:0] sd_buff_din1,
	output      [7:0] sd_buff_din2,
	input             sd_buff_wr,
	output            scsi_led
);

wire        hdc_acc, hdc_we;
wire  [2:0] hdc_rs;
wire  [7:0] hdc_wdata, hdc_rdata;
wire        hdc_irq_set, hdc_irq_clr;
wire        hdc_push_req, hdc_push_ack, hdc_pull_req, hdc_pull_ack, hdc_flush;
wire  [7:0] hdc_push_byte, hdc_pull_byte;

wire [31:0] f_lba0, f_lba1;
wire  [1:0] f_rd, f_wr;
wire  [7:0] f_din0, f_din1;
wire        f_led;

falcon_fdc #(.CLK_HZ(32000000), .EXT_SCSI(1)) fdc (
	.clk(clk), .reset(reset),
	.bus_cs(bus_cs), .bus_stb(bus_stb), .bus_we(bus_we), .bus_addr(bus_addr),
	.bus_uds(bus_uds), .bus_lds(bus_lds), .bus_din(bus_din), .bus_dout(bus_dout),
	.bus_ack(bus_ack), .bus_berr(bus_berr),
	.irq(irq),
	.dma_req(dma_req), .dma_we(dma_we), .dma_addr(dma_addr), .dma_be(dma_be),
	.dma_wdata(dma_wdata), .dma_rdata(dma_rdata), .dma_ack(dma_ack),
	.drv_sel(2'b11), .side_sel(1'b1),
	.img_mounted(2'b00), .img_readonly(1'b0), .img_size(64'd0),
	.sd_lba0(f_lba0), .sd_lba1(f_lba1), .sd_rd(f_rd), .sd_wr(f_wr), .sd_ack(2'b00),
	.sd_buff_addr(9'd0), .sd_buff_dout(8'd0), .sd_buff_din0(f_din0), .sd_buff_din1(f_din1),
	.sd_buff_wr(1'b0),
	.led(f_led),
	.hdc_acc(hdc_acc), .hdc_we(hdc_we), .hdc_rs(hdc_rs), .hdc_wdata(hdc_wdata),
	.hdc_rdata(hdc_rdata), .hdc_irq_set(hdc_irq_set), .hdc_irq_clr(hdc_irq_clr),
	.hdc_push_req(hdc_push_req), .hdc_push_byte(hdc_push_byte), .hdc_push_ack(hdc_push_ack),
	.hdc_pull_req(hdc_pull_req), .hdc_pull_byte(hdc_pull_byte), .hdc_pull_ack(hdc_pull_ack),
	.hdc_flush(hdc_flush)
);

falcon_scsi #(.CLK_HZ(32000000)) scsi (
	.clk(clk), .reset(reset),
	.acc(hdc_acc), .we(hdc_we), .rs(hdc_rs), .wdata(hdc_wdata), .rdata(hdc_rdata),
	.irq_set(hdc_irq_set), .irq_clr(hdc_irq_clr),
	.dma_push_req(hdc_push_req), .dma_push_byte(hdc_push_byte), .dma_push_ack(hdc_push_ack),
	.dma_pull_req(hdc_pull_req), .dma_pull_byte(hdc_pull_byte), .dma_pull_ack(hdc_pull_ack),
	.dma_flush(hdc_flush),
	.img_mounted(img_mounted), .img_readonly(img_readonly), .img_size(img_size),
	.sd_lba0(sd_lba0), .sd_lba1(sd_lba1), .sd_lba2(sd_lba2),
	.sd_rd(sd_rd), .sd_wr(sd_wr), .sd_ack(sd_ack), .sd_blk_cnt(sd_blk_cnt),
	.sd_buff_addr(sd_buff_addr), .sd_buff_dout(sd_buff_dout),
	.sd_buff_din0(sd_buff_din0), .sd_buff_din1(sd_buff_din1), .sd_buff_din2(sd_buff_din2),
	.sd_buff_wr(sd_buff_wr),
	.led(scsi_led)
);

endmodule
