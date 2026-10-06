// MiSTer persistence for the MC146818's 50 NVRAM bytes (Hatari file layout).
// ioctl index 3: download exactly 50 bytes to restore; an empty download
// selects defaults. Upload returns a stable 50-byte snapshot followed by
// its 32-bit generation, least significant byte first, and valid marker $A5.
// ioctl index 4: download that generation only AFTER the file is safely
// saved. An old acknowledgement cannot clear newer guest writes.
// Main_MiSTer support/falcon/falcon_nvram.cpp implements the host protocol.
// Machine resets do not reset this controller or discard pending saves.
module falcon_nvram_store #(
    parameter integer SAVE_CYCLES = 32000000,
    parameter integer RETRY_CYCLES = 160000000
) (
    input             clk,
    input             defaults,
    input             ioctl_download,
    input             ioctl_upload,
    input      [15:0] ioctl_index,
    input      [26:0] ioctl_addr,
    input             ioctl_wr,
    input       [7:0] ioctl_dout,
    output      [7:0] ioctl_din,
    output reg        save_req = 0,
    output            busy,
    output reg        nv_init = 0,
    input             nv_ready,
    output      [5:0] nv_addr,
    input       [7:0] nv_dout,
    output      [7:0] nv_din,
    output            nv_wr,
    input             nv_changed
);
localparam [2:0] IDLE = 0, CAPTURE = 1, WAIT_ACK = 2,
                 RESTORE = 3, DRAIN = 4, INIT_WAIT = 5;
reg [2:0] state = IDLE;
(* ramstyle = "MLAB, no_rw_check" *) reg [7:0] image [0:63];
reg [5:0] ptr = 0;
reg [31:0] generation = 0, saved_generation = 0, ack_generation = 0;
reg [31:0] timer = 0;
reg dirty = 0, downloading = 0, defaults_d = 0;
reg [15:0] download_index = 0;
reg [6:0] received = 0;
reg bad_download = 0;
wire image_download = ioctl_download && ioctl_index == 16'd3;
wire command_download = ioctl_download &&
                        (ioctl_index == 16'd3 || ioctl_index == 16'd4);
wire start_download = command_download && !downloading;
wire end_download = downloading && !command_download;
wire reset_image = (defaults && !defaults_d) ||
                  (end_download && download_index == 16'd3 &&
                   received == 0 && !bad_download);

assign busy = defaults || image_download ||
              (downloading && download_index == 16'd3) || state == RESTORE || state == DRAIN ||
              state == INIT_WAIT || nv_init;
assign nv_addr = ptr;
// One write port and one shared asynchronous read port infer a small MLAB.
wire image_write = (state == CAPTURE) || (command_download && ioctl_wr &&
                   ioctl_index == 16'd3 && ioctl_addr == {20'd0, received} && received < 50);
wire image_loading = command_download && ioctl_wr && ioctl_index == 16'd3;
wire [5:0] image_wa = image_loading ? ioctl_addr[5:0] : ptr;
wire [7:0] image_wd = image_loading ? ioctl_dout : nv_dout;
wire [5:0] image_ra = state == RESTORE ? ptr : ioctl_addr[5:0];
wire [7:0] image_q = image[image_ra];
always @(posedge clk) if (image_write) image[image_wa] <= image_wd;
assign nv_din = image_q;
assign nv_wr = state == RESTORE && nv_ready;
// The snapshot stays unchanged until acknowledgement or restore. Reads of
// unrelated uploads and of addresses outside the packet return zero.
assign ioctl_din = !ioctl_upload || ioctl_index != 16'd3 || state != WAIT_ACK ? 8'd0 :
                   ioctl_addr < 27'd50 ? image_q :
                   ioctl_addr == 27'd50 ? saved_generation[7:0] :
                   ioctl_addr == 27'd51 ? saved_generation[15:8] :
                   ioctl_addr == 27'd52 ? saved_generation[23:16] :
                   ioctl_addr == 27'd53 ? saved_generation[31:24] :
                   ioctl_addr == 27'd54 ? 8'hA5 : 8'd0;

always @(posedge clk) begin
    save_req <= 0;
    nv_init <= 0;
    downloading <= command_download;
    defaults_d <= defaults;

    case (state)
    IDLE: begin
        if (dirty && nv_ready && !ioctl_download && !ioctl_upload) begin
            if (timer == SAVE_CYCLES - 1) begin
                ptr <= 0;
                timer <= 0;
                state <= CAPTURE;
            end else timer <= timer + 1'd1;
        end else timer <= 0;
    end
    CAPTURE: begin
        ptr <= ptr + 1'd1;
        if (ptr == 6'd49) begin
            saved_generation <= generation;
            save_req <= 1;
            timer <= 0;
            state <= WAIT_ACK;
        end
    end
    WAIT_ACK: begin
        // A failed file write gets no acknowledgement. Retry automatically;
        // never replace a snapshot while the HPS is reading it.
        if (!ioctl_upload && !ioctl_download) begin
            if (timer == RETRY_CYCLES - 1) begin
                timer <= 0;
                save_req <= 1;
            end else timer <= timer + 1'd1;
        end
    end
    RESTORE: if (nv_ready) begin
        ptr <= ptr + 1'd1;
        if (ptr == 6'd49) state <= DRAIN;
    end
    DRAIN: state <= IDLE;
    INIT_WAIT: if (nv_ready && !nv_init) state <= IDLE;
    default: state <= IDLE;
    endcase

    if (nv_changed) begin
        dirty <= 1;
        generation <= generation + 1'd1;
        // Do not request a save of an image torn by a concurrent CPU write.
        if (state == CAPTURE) begin
            state <= IDLE;
            save_req <= 0;
        end
        if (state != WAIT_ACK) timer <= 0;
    end

    if (start_download) begin
        download_index <= ioctl_index;
        received <= 0;
        bad_download <= 0;
        ack_generation <= 0;
        if (ioctl_index == 16'd3) begin
            state <= IDLE;
            timer <= 0;
            save_req <= 0;
        end
    end
    if (command_download && ioctl_wr) begin
        if (ioctl_addr != {20'd0, received} || received >=
            (ioctl_index == 16'd3 ? 7'd50 : 7'd4)) begin
            bad_download <= 1;
        end else begin
            received <= received + 1'd1;
            if (ioctl_index == 16'd4) ack_generation <= {ioctl_dout, ack_generation[31:8]};
        end
    end
    if (end_download && !bad_download) begin
        if (download_index == 16'd3 && received == 7'd50) begin
            ptr <= 0;
            state <= RESTORE;
            dirty <= 0;
            generation <= generation + 1'd1;
        end else if (download_index == 16'd4 && received == 7'd4 &&
                     state == WAIT_ACK && ack_generation == saved_generation) begin
            dirty <= nv_changed || (ack_generation != generation);
            state <= IDLE;
            timer <= 0;
        end
    end
    if (reset_image) begin
        nv_init <= 1;
        state <= INIT_WAIT;
        dirty <= 1;
        timer <= 0;
        generation <= generation + 1'd1;
        save_req <= 0;
    end
end
endmodule
