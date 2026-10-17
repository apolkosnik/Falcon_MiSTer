// MiSTer LTC2308 channels 0/1 -> Falcon left/right microphone samples.
// Reference: LTC2308 datasheet (2308fc, serial timing and unipolar format),
// MiSTer ADCTest's stereo channel ordering, and Hatari crossbar.c's signed
// microphone samples (Crossbar_GenerateSamples / Process_ADCXmit_Transfer).
// The controller and crossbar share clk; samples are held between updates.
//
// Capture at 96 kHz/channel, remove each channel's DC bias with a 1/1024
// tracking filter (~15 Hz corner), then scale 12-bit ADC units by 16 and
// saturate to signed 16-bit PCM. The first valid pair seeds the bias, keeping
// reset/startup silent. This is a host input adapter: Falcon sample-rate
// selection still belongs to the crossbar. There is no rate-dependent
// anti-alias filter when the guest selects a lower recording rate.
module falcon_audio_adc #(
    parameter integer CLK_HZ = 32000000
) (
    input                    clk,
    input                    reset,
    inout              [3:0] ADC_BUS,
    output signed     [15:0] mic_l,
    output signed     [15:0] mic_r,
    output reg               sample_stb
);

// At 32 MHz: 8 MHz SCK, 2 us CONVST, 62.5 ns from CONVST falling
// to MSB capture. A complete transfer fits within the 5.208 us period.
localparam integer SCK_DIV = (CLK_HZ + 15999999) / 16000000;
wire [23:0] raw;
wire raw_sync;
ltc2308 #(
    .NUM_CH(2), .ADC_RATE(192000), .CLK_RATE(CLK_HZ),
    .SCK_DIV(SCK_DIV), .CONV_WAIT_NS(2000), .LONG_CONVST(1)
) adc (
    .clk(clk), .reset(reset), .ADC_BUS(ADC_BUS),
    .dout(raw), .dout_sync(raw_sync)
);

reg sync_d;
reg [1:0] warmup;
reg have_bias;
wire pair_tick = (raw_sync != sync_d);
always @(posedge clk) begin
    sample_stb <= 1'b0;
    if (reset) begin
        sync_d <= 1'b0;
        warmup <= 2'd0;
        have_bias <= 1'b0;
    end else begin
        sync_d <= raw_sync;
        if (pair_tick) begin
            // dout_sync starts the next round, not the completed read.
            // Discard the initial toggle and the pair containing the ADC's
            // unknown power-up channel. The third toggle has two valid words.
            if (warmup != 2'd2) warmup <= warmup + 1'b1;
            else begin
                have_bias <= 1'b1;
                sample_stb <= have_bias;
            end
        end
    end
end

function [15:0] pcm(input signed [12:0] centered);
    if (centered > 13'sd2047) pcm = 16'h7fff;
    else if (centered < -13'sd2048) pcm = 16'h8000;
    else pcm = {centered[11:0], 4'b0000};
endfunction

wire [31:0] samples;
genvar ch;
generate for (ch = 0; ch < 2; ch = ch + 1) begin : channel
    reg signed [24:0] bias;
    reg signed [15:0] sample;
    wire signed [24:0] value = $signed({1'b0, raw[ch*12 +: 12], 12'd0});
    wire signed [24:0] error = value - bias;
    wire signed [12:0] centered = error[24:12];
    assign samples[ch*16 +: 16] = sample;
    always @(posedge clk) begin
        if (reset) begin
            bias <= 25'sd0;
            sample <= 16'sd0;
        end else if (pair_tick && warmup == 2'd2) begin
            if (!have_bias) bias <= value;
            else begin
                bias <= bias + (error >>> 10);
                sample <= pcm(centered);
            end
        end
    end
end endgenerate
assign mic_l = samples[15:0];
assign mic_r = samples[31:16];

endmodule
