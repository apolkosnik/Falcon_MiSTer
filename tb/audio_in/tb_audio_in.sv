`timescale 1ns/1ps
// Real LTC2308 controller, audio adapter and Falcon crossbar. Only the
// external ADC and DMA memory response are modeled. Serial timing follows
// LTC2308 2308fc: next-conversion configuration, MSB first, falling-edge SDO.
module tb_audio_in;
reg clk = 0, reset = 1;
always #15.625 clk = ~clk;
wire [3:0] adc_bus;
wire signed [15:0] mic_l, mic_r;
wire sample_stb;
falcon_audio_adc dut (
    .clk(clk), .reset(reset), .ADC_BUS(adc_bus),
    .mic_l(mic_l), .mic_r(mic_r), .sample_stb(sample_stb)
);

// Include a conservative 20 ns effective FPGA/board delay in each direction
// and the ADC's 15 ns enable / 12.5 ns data-valid delays. The FPGA budgets
// in Falcon.sdc are 18 ns plus a 2 ns board assumption per direction.
wire sck, convst, sdi;
assign #20 sck = adc_bus[3];
assign #20 convst = adc_bus[0];
assign #20 sdi = adc_bus[1];
reg sdo = 0;
assign #20 adc_bus[2] = sdo;
reg [11:0] level_l = 12'd2048, level_r = 12'd1536;
reg [11:0] shift = 0, converted = 0;
reg [5:0] command = 0;
integer selected = 7, converting = 7, bits = 0, conversions = 0;
integer serial_checks = 0;
realtime start_time = 0, last_fall = 0, last_rise = 0;
always @(posedge reset) begin
    bits = 0;
    conversions = 0;
    start_time = 0;
    last_rise = 0;
end
always @(posedge convst) if (!reset) begin
    if (conversions != 0 && bits != 13) $fatal(1, "ADC conversion overlapped serial read");
    start_time = $realtime;
    converting = selected;
    converted = selected == 0 ? level_l : selected == 1 ? level_r : 12'hbad;
    bits = 0;
end
always @(negedge convst) if (!reset && start_time != 0) begin
    if ($realtime - start_time < 2000) $fatal(1, "ADC conversion wait shorter than 2 us");
    shift = converted;
    #15; sdo = shift[11];
end
always @(negedge sck) if (!reset && start_time != 0) begin
    last_fall = $realtime;
    shift = {shift[10:0], 1'b0};
    #12.5; sdo = shift[11];
end
always @(sdi) if (!reset && last_rise != 0 && $realtime - last_rise < 2.5)
    $fatal(1, "ADC SDI hold time violated");
always @(posedge sck) if (!reset) begin
    last_rise = $realtime;
    if (bits != 0 && $realtime - last_fall < 62.5) $fatal(1, "ADC SCK too fast");
    if (bits < 6) command = {command[4:0], sdi};
    if (bits == 5) begin
        if (command != (conversions % 2 == 0 ? 6'b100010 : 6'b110010))
            $fatal(1, "ADC channel/configuration mismatch: %06b", command);
        selected = command[4] ? 1 : 0;
    end
    bits++;
end
always @(negedge clk) if (!reset && bits == 12) begin
    if (conversions != 0) begin
        if (dut.raw[converting*12 +: 12] !== converted)
            $fatal(1, "ADC channel %0d data expected %03x got %03x", converting,
                   converted, dut.raw[converting*12 +: 12]);
        serial_checks++;
    end
    conversions++;
    bits = 13;
end

reg bus_cs = 0, bus_stb = 0, bus_we = 0, bus_uds = 0, bus_lds = 0;
reg [6:1] bus_addr = 0;
reg [15:0] bus_din = 0;
wire [15:0] bus_dout;
wire bus_ack, bus_berr;
wire dma_req, dma_we;
wire [23:1] dma_addr;
wire [1:0] dma_be;
wire [15:0] dma_wdata;
reg dma_ack = 0;
wire signed [15:0] audio_l, audio_r;
wire audio_stb, ssi_slot_stb, ssi_rx_en, ssi_rx_frame;
wire [15:0] ssi_rx_data;
reg signed [15:0] psg_audio = 16'sd1234;
falcon_crossbar xbar (
    .clk(clk), .reset(reset),
    .bus_cs(bus_cs), .bus_stb(bus_stb), .bus_we(bus_we),
    .bus_addr(bus_addr), .bus_uds(bus_uds), .bus_lds(bus_lds), .bus_din(bus_din),
    .bus_dout(bus_dout), .bus_ack(bus_ack), .bus_berr(bus_berr),
    .dma_req(dma_req), .dma_we(dma_we), .dma_addr(dma_addr), .dma_be(dma_be),
    .dma_wdata(dma_wdata), .dma_rdata(16'd0), .dma_ack(dma_ack),
    .sndint(), .soundint(), .psg_audio(psg_audio), .mic_l(mic_l), .mic_r(mic_r),
    .audio_l(audio_l), .audio_r(audio_r), .audio_stb(audio_stb),
    .ssi_slot_stb(ssi_slot_stb), .ssi_frame(), .ssi_rx_data(ssi_rx_data),
    .ssi_tx_data(16'd0), .ssi_tx_valid(1'b0), .ssi_hs_play_req(1'b0),
    .ssi_tx_en(), .ssi_rx_en(ssi_rx_en), .ssi_rx_frame(ssi_rx_frame), .dbg_underrun()
);

integer checks = 0, samples = 0, last_sample_cycle = 0, cycles = 0;
reg [31:0] prev_pcm = 0;
always @(negedge clk) begin
    cycles++;
    if (reset) begin samples = 0; last_sample_cycle = 0; end
    else begin
        if (sample_stb) begin
            if (last_sample_cycle != 0 && cycles-last_sample_cycle != 333 && cycles-last_sample_cycle != 334)
                $fatal(1, "Stereo rate not 96 kHz: interval %0d", cycles-last_sample_cycle);
            last_sample_cycle = cycles;
            samples++;
        end else if ({mic_l, mic_r} !== prev_pcm)
            $fatal(1, "PCM changed outside sample_stb");
    end
    prev_pcm = {mic_l, mic_r};
end
task check(input bit ok, input string message);
    checks++;
    if (!ok) $fatal(1, "FAIL: %s (PCM L=%0d R=%0d)", message, mic_l, mic_r);
    $display("PASS: %s", message);
endtask
task wait_samples(input integer count);
    integer goal;
    goal = samples + count;
    while (samples < goal) @(negedge clk);
    #1;
endtask
task wr(input [7:0] addr, input [15:0] value, input bit word_access);
    @(negedge clk);
    bus_cs = 1; bus_stb = 1; bus_we = 1; bus_addr = addr[6:1];
    bus_uds = word_access || !addr[0]; bus_lds = word_access || addr[0];
    bus_din = word_access ? value : {value[7:0], value[7:0]};
    @(negedge clk);
    if (!bus_ack || bus_berr) $fatal(1, "Crossbar register write failed at %02x", addr);
    bus_cs = 0; bus_stb = 0; bus_we = 0;
endtask
task frame_reg(input [7:0] addr, input [23:0] value);
    wr(addr, {8'd0, value[23:16]}, 0);
    wr(addr+2, {8'd0, value[15:8]}, 0);
    wr(addr+4, {8'd0, value[7:0]}, 0);
endtask

integer recorded = 0, positive = 0, negative = 0, dsp_left = 0, dsp_right = 0;
reg monitor_routes = 0;
reg [15:0] recording [0:63];
always @(posedge clk) begin
    dma_ack <= 0;
    if (!reset && dma_req && !dma_ack) begin
        if (!dma_we || dma_be != 2'b11 || recorded >= 64 || dma_addr != (23'h10000 + 23'(recorded)))
            $fatal(1, "Unexpected ADC DMA request: address=%06x count=%0d", {dma_addr,1'b0}, recorded);
        recording[recorded] = dma_wdata;
        if ($signed(dma_wdata) > 0) positive++;
        if ($signed(dma_wdata) < 0) negative++;
        recorded++;
        dma_ack <= 1;
    end
    if (!reset && monitor_routes && ssi_slot_stb && ssi_rx_en) begin
        if (ssi_rx_frame) begin
            if ($signed(ssi_rx_data) <= 0) $fatal(1, "ADC left channel lost on DSP route");
            dsp_left++;
        end else begin
            if ($signed(ssi_rx_data) >= 0) $fatal(1, "ADC right channel lost on DSP route");
            dsp_right++;
        end
    end
end

initial begin
    repeat (8) @(negedge clk);
    reset = 0;
    // Neither raw reset zeros nor the initial unknown-channel word may
    // produce a PCM pulse before the valid pair seeds both bias trackers.
    while (samples == 0) begin
        @(negedge clk); #1;
        check(mic_l == 0 && mic_r == 0, "startup PCM is silent");
        // Check the rest of startup without printing every system clock.
        if (samples == 0) begin
            while (!sample_stb) begin
                @(negedge clk); #1;
                if (mic_l != 0 || mic_r != 0) $fatal(1, "Startup bias pulse");
            end
        end
    end
    wait_samples(10);
    check(mic_l == 0 && mic_r == 0, "different left/right DC biases produce silence");
    level_l = 12'd2304; level_r = 12'd1408;
    wait_samples(8);
    check(mic_l > 4000 && mic_l <= 4096, "positive channel 0 amplitude, signed x16 scaling");
    check(mic_r < -2000 && mic_r >= -2064, "negative channel 1 amplitude, independent DC tracking");

    // Guest selects microphone, 16-bit recording, ADC -> DSP and DMA.
    // Register layout/routing: Hatari Crossbar_* and existing crossbar bench.
    wr(8'h35, 16'd1, 0);
    wr(8'h21, 16'h40, 0);
    wr(8'h38, 16'h00, 0);
    wr(8'h37, 16'h01, 0);
    wr(8'h39, 16'h88, 0);
    wr(8'h3a, 16'h0000, 1);
    wr(8'h32, 16'h00e7, 1);
    wr(8'h01, 16'h80, 0);
    frame_reg(8'h03, 24'h020000);
    frame_reg(8'h0f, 24'h020080);
    monitor_routes = 1;
    wr(8'h01, 16'h90, 0);
    while (recorded != 64) @(negedge clk);
    repeat (3000) @(negedge clk);
    monitor_routes = 0;
    check(positive == 32 && negative == 32, "serial ADC reaches both channels of 16-bit DMA recording");
    for (integer i = 1; i < 64; i++)
        if (recording[i][15] == recording[i-1][15]) $fatal(1, "DMA channels not alternating at word %0d", i);
    check(dsp_left > 20 && dsp_right > 20, "serial ADC reaches DSP receive with correct channel frames");
    check(audio_l > 2000 && audio_r < -1000, "microphone passes through codec gain and audio output");
    check(recorded == 64 && !dma_req, "recording stops at programmed frame end");

    // PSG still wins when selected by the guest; microphone wiring does
    // not bypass the Falcon's mixer/source controls.
    wr(8'h38, 16'h03, 0);
    repeat (3000) @(negedge clk);
    check(audio_l == audio_r && audio_l > 0, "PSG selection still overrides external microphone");

    wait_samples(16000);
    check(mic_l >= -16 && mic_l <= 16 && mic_r >= -16 && mic_r <= 16,
          "constant input settles back to silence");
    // Reacquire offset after a reset in the middle of serial activity.
    reset = 1;
    level_l = 12'd256; level_r = 12'd3800;
    repeat (8) @(negedge clk);
    reset = 0;
    wait_samples(6);
    check(mic_l == 0 && mic_r == 0, "reset reacquires new channel biases silently");
    level_l = 12'd4095; level_r = 12'd0;
    wait_samples(5);
    check(mic_l == 32767 && mic_r == -32768, "large excursions saturate without signed wraparound");
    check(serial_checks > 32000, "serial words verified with delayed ADC data and channel configuration");
    $display("SUMMARY: %0d checks, %0d serial words, 0 failures", checks, serial_checks);
    $display("RESULT: PASS");
    $finish;
end
initial begin #200000000; $fatal(1, "Audio input watchdog"); end
endmodule
