// Real NVRAM and persistence controller; the C++ bench drives CPU register
// accesses and the ioctl endpoint used by the real Main persistence code.
module tb_nvram_store (
    input clk, reset, defaults, cfg_vga,
    input bus_cs, bus_stb, bus_we, bus_addr,
    input [15:0] bus_din,
    output [15:0] bus_dout,
    input fp_enable, io_enable, io_strobe,
    input [15:0] hps_din,
    output [15:0] hps_dout,
    output save_req, busy, nv_ready
);
wire [45:0] hps_bus;
wire [35:0] ext_bus;
assign hps_bus[35:33] = {fp_enable, io_enable, io_strobe};
assign hps_bus[31:16] = hps_din;
assign hps_bus[45:38] = 0;
assign hps_dout = hps_bus[15:0];
assign ext_bus[32] = 0;
assign ext_bus[15:0] = 0;
wire ioctl_download, ioctl_upload, ioctl_wr;
wire [15:0] ioctl_index;
wire [26:0] ioctl_addr;
wire [7:0] ioctl_dout, ioctl_din;
hps_io #(.CONF_STR("Falcon;;")) hps (
    .clk_sys(clk), .HPS_BUS(hps_bus), .EXT_BUS(ext_bus),
    .ioctl_download(ioctl_download), .ioctl_upload(ioctl_upload),
    .ioctl_index(ioctl_index), .ioctl_addr(ioctl_addr),
    .ioctl_wr(ioctl_wr), .ioctl_dout(ioctl_dout), .ioctl_din(ioctl_din),
    .ioctl_wait(1'b0), .ioctl_upload_req(save_req), .ioctl_upload_index(8'd3),
    .sd_rd(1'b0), .sd_wr(1'b0), .sd_lba('{32'd0}), .sd_blk_cnt('{6'd0}),
    .sd_buff_din('{8'd0}), .new_vmode(1'b0), .video_rotated(1'b0),
    .info_req(1'b0), .info(8'd0), .status_set(1'b0), .status_in(128'd0),
    .status_menumask(32'd0)
);
wire nv_init, nv_wr, nv_changed;
wire [5:0] nv_addr;
wire [7:0] nv_din, nv_dout;
falcon_nvram_store #(.SAVE_CYCLES(256), .RETRY_CYCLES(4096)) store (
    .clk(clk), .defaults(defaults), .ioctl_download(ioctl_download),
    .ioctl_upload(ioctl_upload), .ioctl_index(ioctl_index), .ioctl_addr(ioctl_addr),
    .ioctl_wr(ioctl_wr), .ioctl_dout(ioctl_dout), .ioctl_din(ioctl_din),
    .save_req(save_req), .busy(busy), .nv_init(nv_init), .nv_ready(nv_ready),
    .nv_addr(nv_addr), .nv_dout(nv_dout), .nv_din(nv_din), .nv_wr(nv_wr),
    .nv_changed(nv_changed)
);
falcon_nvram nvram (
    .clk(clk), .reset(reset | busy), .bus_cs(bus_cs), .bus_stb(bus_stb),
    .bus_we(bus_we), .bus_addr(bus_addr), .bus_uds(1'b0), .bus_lds(1'b1),
    .bus_din(bus_din), .bus_dout(bus_dout), .bus_ack(), .rtc(65'd0),
    .cfg_vga(cfg_vga), .cfg_lang(8'd0), .cfg_kbd(8'd0),
    .nv_init(nv_init), .nv_ready(nv_ready), .nv_addr(nv_addr),
    .nv_dout(nv_dout), .nv_din(nv_din), .nv_wr(nv_wr), .nv_changed(nv_changed), .irq()
);
endmodule
