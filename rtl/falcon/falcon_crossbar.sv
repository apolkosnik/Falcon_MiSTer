// falcon_crossbar.sv - Atari Falcon030 DMA sound, sound matrix (crossbar)
//                      and codec (DAC / ADC / 16-bit adder)
//
// Behavioural reference: Hatari src/falcon/crossbar.c
//   Crossbar_Reset, Crossbar_BufferInter_WriteByte ($FF8900),
//   Crossbar_DmaCtrlReg_WriteByte ($FF8901), Crossbar_Frame{Start,Count,End}
//   {High,Med,Low}_{Read,Write}Byte, Crossbar_DmaTrckCtrl_WriteByte ($FF8920),
//   Crossbar_SoundModeCtrl_WriteByte ($FF8921), Crossbar_Microwire_WriteWord
//   + Crossbar_InterruptHandler_Microwire ($FF8924),
//   Crossbar_SrcControler_WriteWord ($FF8930), Crossbar_DstControler_WriteWord
//   ($FF8932), Crossbar_FreqDivInt_WriteByte ($FF8935),
//   Crossbar_TrackRecSelect_WriteByte, Crossbar_CodecInput_WriteByte,
//   Crossbar_AdcInput_WriteByte, Crossbar_InputAmp_WriteByte,
//   Crossbar_OutputReduct_WriteWord, Crossbar_Recalculate_Clocks_Cycles,
//   Crossbar_DetectSampleRate, Crossbar_InterruptHandler_25Mhz/_32Mhz,
//   Crossbar_Process_DSPXmit_Transfer, Crossbar_SendDataToDspReceive,
//   Crossbar_setDmaPlay_Settings, Crossbar_Process_DMAPlay_Transfer,
//   Crossbar_DmaPlayInHandShakeMode, Crossbar_setDmaRecord_Settings,
//   Crossbar_SendDataToDmaRecord, Crossbar_Process_DMARecord_HandshakeMode,
//   Crossbar_Process_ADCXmit_Transfer, Crossbar_SendDataToDAC,
//   Crossbar_GenerateSamples, Crossbar_Update_DMA_Sound_Line and the
//   ioMemTabFalcon.c entries for $FF8900-$FF8943.
//
// ---------------------------------------------------------------------------
// Register bus (docs/ARCHITECTURE.md), bus_addr[6:1] = word offset inside
// $FF8900-$FF8943.  bus_ack one clock after bus_stb, bus_dout registered.
// Byte lanes merge into word registers (Hatari's IoMem behaviour).
//   $FF8900.b  buffer interrupts: bit0 play->GPIP7 (SNDINT), bit1 record->
//              GPIP7, bit2 play->TAI (SOUNDINT), bit3 record->TAI.  Reads
//              back all 8 bits.  Reset $05 (Crossbar_Reset).
//   $FF8901.b  control: bit0 play, bit1 play loop, bit4 record, bit5 record
//              loop, bit7 register select (0 play, 1 record) for $FF8903-13.
//              Reads back all 8 bits written; bit0/bit4 are cleared by the
//              hardware at the end of a non-looped frame.  The loop bits are
//              only latched when play/record starts.
//   $FF8902/04/../12 (even bytes) read $FF, writes ignored.
//   $FF8903/05/07 frame start, $FF8909/0B/0D frame count, $FF890F/11/13
//              frame end (H/M/L).  As in Hatari each group has one shadow
//              byte per address (IoMem); a byte write updates its shadow and
//              sets the selected channel's register to the 24-bit value of
//              the three shadows with bit 0 cleared; a byte read returns the
//              selected channel's byte and copies it into the shadow.
//              Count reads return start-of-current-frame + counter (the
//              current DMA address); count writes have no effect.
//   $FF8914-$FF891F  bus error (bus_berr with bus_ack).
//   $FF8920.b  bits 1:0 number of play tracks - 1, bits 5:4 monitored track
//   $FF8921.b  bit7 mono, bit6 16 bit, bits 1:0 STE frequency (6258, 12517,
//              25033, 50066 Hz) used while $FF8935 = 0
//   $FF8922.w  reads 0 (no Microwire on the Falcon)
//   $FF8924.w  Microwire mask: a write stores the complement of the value
//              and restores the value 8 CPU cycles (16 MHz) later.
//   $FF8926-$FF892F  bus error
//   $FF8930.w  source control, $FF8932.w destination control (full words
//              stored; decoding as Crossbar_Src/DstControler_WriteWord)
//   $FF8934.b  external clock divider (stored only), $FF8935.b bits 3:0
//              internal clock divider, $FF8936.b record tracks (stored only)
//   $FF8937.b  codec adder input: bit0 ADC, bit1 crossbar (DAC multiplexer)
//   $FF8938.b  ADC input: bit1 left, bit0 right, 1 = PSG, 0 = microphone
//   $FF8939.b  ADC gain LLLLRRRR, $FF893A.w DAC attenuation bits 11:8 left,
//              bits 7:4 right
//   $FF893C.w  codec status (stored, reset $2401), $FF893E.w stored,
//   $FF8940.w  GPIO direction, $FF8942.w GPIO data (stored only)
//
// ---------------------------------------------------------------------------
// Clocks / slots.  The crossbar works in "slots": one slot carries one
// 16-bit word of one channel of one track.  A frame is 2 * play_tracks slots
// (L0 R0 L1 R1 ...).  Slot rates (Crossbar_Recalculate_Clocks_Cycles):
//   25 MHz clock : rate25 * 2 * tracks slots/s, rate25 = STE table when
//                  $FF8935 = 0, else 25.175 MHz table [div-1]
//   32 MHz clock : rate32 * 2 * tracks slots/s, rate32 = 32 MHz table [div-1]
//   external clock: never ticks (as in Hatari).
// Hatari's integer frequency tables are used and the slot ticks are made
// from clk with a fractional accumulator (exact average rate).
// On every 25 MHz slot: in STE mode ($FF8935 = 0) DSP transmit, DMA play
// and ADC are processed (in that order); in Falcon mode the ADC, then DSP
// transmit and DMA play when their clock ($FF8930 bits 6:5 / 2:1) is 25 MHz.
// On every 32 MHz slot (Falcon mode only): DSP transmit and DMA play when
// their clock is 32 MHz.
//
// DMA play (16-bit: one word per slot; 8-bit stereo: one byte per slot,
// sent to the DAC as byte*64; 8-bit mono: one byte per slot pair, same byte
// on both channels).  Data come from a FIFO_WORDS word prefetch FIFO filled
// through the DMA port.  In loop mode the prefetcher runs into the next frame
// using the live frame start/end registers; when the frame actually loops,
// the registers are latched again and, if they changed, the FIFO is flushed
// and refilled.  After a start or a flush the play waits (slots are skipped)
// until the first word is in the FIFO.
// DMA record: one DMA write per recorded word (16-bit and 8-bit stereo:
// word write, +2) or byte (8-bit mono: byte write, +1), through an 8 entry
// write queue.  Both share the DMA port; a play fetch has priority when the
// prefetch FIFO is less than half full.
//
// SNDINT (GPIP7) / SOUNDINT (MFP TAI): low while the selected DMA channel is
// active, high when idle, as Crossbar_Update_DMA_Sound_Line: a play or
// record start, stop or frame end sets the line from the current $FF8900
// bits of that channel (bit clear -> line high).  At a looped frame end the
// line goes high and back low LINE_PULSE clocks later (Hatari does both in
// the same instant; the MFP needs a pulse it can see).
//
// Codec / DAC output (Crossbar_GenerateSamples): the DAC takes a left/right
// pair from its source (the monitored track's slots); a pair is committed
// when its right word arrives.  audio_stb is one clock per frame of the
// 25 MHz clock (the codec sample rate = rate25, Hatari's DAC rate).  With
// ADC_l/r = PSG or microphone per $FF8938, gain/attenuation from Hatari's
// tables:
//   direct = (ADC * gain) >> 14 ; dac = committed pair (0 if the DAC got
//   no data during the last sample period)
//   adder  = $FF8937: 0 -> 0, 1 -> direct, 2 -> dac, 3 -> direct + dac
//   audio  = (adder * attenuation) >> 16 ; 0 while the DAC is muted
//            (div 6, 8, 10, >= 12, or div 0 with STE frequency 0).
// audio_l/audio_r change only with audio_stb (fixed 4 clock latency after
// the frame's last slot).
//
// ---------------------------------------------------------------------------
// DSP SSI link (slot level).  All outputs registered; one strobe clock per
// slot event.  For each slot where the crossbar talks to the DSP:
//   ssi_slot_stb   one clock.  The DSP samples ssi_rx_data / ssi_frame /
//                  ssi_rx_frame at this clock edge, and the crossbar samples
//                  ssi_tx_data (and the DSP must present it during this
//                  clock; it may depend combinationally on ssi_frame).
//   ssi_tx_en      this slot clocks the DSP transmitter (Hatari:
//                  DSP_SsiReceive_SC2(frame) + DSP_SsiReceive_SCK +
//                  DSP_SsiReadTxValue).  ssi_tx_data must be the value the
//                  SSI shifts out in this slot (0 when TE is off or the
//                  transmitter waits for a frame, as dsp_core_ssi_Receive_SCK).
//   ssi_frame      TX frame sync (SC2): 1 on the first slot of the DSP
//                  transmit frame (Hatari dspXmit.wordCount == 0).  On a slot
//                  without ssi_tx_en it equals ssi_rx_frame.
//   ssi_rx_en      this slot clocks the DSP receiver (DSP_SsiWriteRxValue +
//                  DSP_SsiReceive_SC1(frame) + DSP_SsiReceive_SC0):
//                  ssi_rx_data is valid.
//   ssi_rx_frame   RX frame sync (SC1): 1 when the word is the first slot of
//                  the source's frame (DMA play: currentFrame == 0, DSP
//                  transmit: its own frame, ADC: left channel).
//   ssi_rx_data    16-bit word received by the DSP (DMA play 8-bit data are
//                  the sign extended byte, not scaled).  When the DSP's
//                  receiver is fed from its own transmitter ($FF8932 bits 6:5
//                  = 01) ssi_rx_data is ssi_tx_data of the same slot
//                  (combinational path, so ssi_tx_data must come from DSP
//                  registers and ssi_frame only).
//   ssi_tx_valid   used only in DSP -> DMA record handshake mode ($FF8932
//                  bits 3:0 = 0010): a record slot happens only when the DSP
//                  has data (Hatari dmaRecord.handshakeMode_Frame set by the
//                  DSP's PC5 / TX writes), sampled on the clock before the
//                  strobe; in that slot ssi_frame = 0 (Hatari does not call
//                  SC2 there).  In all other modes the crossbar takes
//                  ssi_tx_data on every ssi_tx_en slot like Hatari (the SSI
//                  re-sends its TX register on underrun).
//   ssi_hs_play_req  input, one clock pulse: the DSP calls
//                  DSP_SsiTransmit_SC1 (PC4 written to 1 with PCDDR bit 4
//                  set), i.e. Crossbar_DmaPlayInHandShakeMode: sets
//                  dmaPlay.handshakeMode_masterClk and handshakeMode_Frame.
//
// DMA play -> DSP receive handshake ($FF8932 bits 6:4 = 000):
//   A write to $FF8932 sets handshakeMode_Frame = (bits 6:4 == 000) and
//   clears masterClk.  While masterClk is clear the play runs on its clock
//   as usual.  Once the DSP requested (masterClk set), a DMA play slot only
//   transfers when handshakeMode_Frame is set (otherwise the slot does
//   nothing: no data, no frame position, no end check), and every word sent
//   to the DSP receiver (from any source, receiver connected) clears it, so
//   one word goes out per request.  With the DMA play clock set to 32 MHz
//   ($FF8930 bits 2:1 = 10) the nocrew special transfer applies: the word
//   sent (to the DSP, the DAC and DMA record) is
//   ((previous word << 2) | (word >> 14)) & $FFFF, the previous word being
//   the last raw word of this mode (0 after reset).
//   A request is applied in the clock it arrives; the transfer happens on
//   the next DMA play slot.
//
// Slot timing: tx and rx slots of the same crossbar clock come in the same
// strobe.  With DSP transmit and the DSP receive source on different clocks
// (e.g. DSP transmit 32 MHz, DMA play 25 MHz) a strobe can carry only one of
// them: a DSP that ignores ssi_tx_en / ssi_rx_en is then wrong; with both on
// the same clock (the normal case) every strobe has both when both paths are
// connected.  Slot spacing is >= CLK_HZ / 500000 clocks (64 at 32 MHz).
//
// ---------------------------------------------------------------------------
// Deviations from Hatari (hardware documentation wins / RTL needs):
//  - $FF8920 monitored track uses bits 5:4 (Hatari masks with decimal 30,
//    i.e. only bit 4 can select track 1).
//  - 8-bit mono plays each byte on both channels (Hatari reads byte n for
//    the left and byte n+1 for the right channel).
//  - The slot rate follows the number of play tracks immediately (Hatari
//    only recomputes it on $FF8921/$FF8935 writes).
//  - Codec adder and ADC gain results saturate to 16 bits (Hatari's int16
//    arithmetic wraps).
//  - ADC as a crossbar source (to DSP / DMA record / DAC) samples the input
//    selected by $FF8938 (PSG or microphone), without gain.  Hatari feeds
//    only the microphone there.
//  - DAC: the latest committed pair is played at the codec rate instead of
//    Hatari's resampling ring buffer; output 0 when no DAC data arrived in
//    the last sample period (Hatari: per host audio chunk).
//  - Reset: registers read back the values Crossbar_Reset writes ($FF8900 =
//    $05, start/count/end = $FFFFFE, $FF893C = $2401) and the internal
//    state is decoded from the registers ($FF8921 = $03 i.e. 8 bit stereo
//    50066 Hz, $FF8937 = $FF8938 = $03, $FF8930 = $FF8932 = 0).  Hatari
//    keeps stale internal state for some of them.
//  - Where Hatari skips DSP_SsiReceive_SC1 (dmaPlay.handshakeMode_Frame
//    set), the slot is sent with ssi_rx_frame = 0.
//  - Underrun (FIFO empty when a sample is due, only possible with extreme
//    DMA latency) plays 0 and drops the late words; dbg_underrun is sticky.
//  - A word record at an odd address (mode changed while recording) writes
//    the aligned word.
//  - LINE_PULSE idle pulse on looped frame ends (see above).
// ---------------------------------------------------------------------------

module falcon_crossbar #(
	parameter CLK_HZ     = 32000000,
	parameter LINE_PULSE = 32,
	parameter FIFO_AW    = 4            // prefetch FIFO = 2^FIFO_AW words
)(
	input                clk,
	input                reset,

	input                bus_cs,
	input                bus_stb,
	input                bus_we,
	input         [6:1]  bus_addr,
	input                bus_uds,
	input                bus_lds,
	input         [15:0] bus_din,
	output reg    [15:0] bus_dout,
	output reg           bus_ack,
	output reg           bus_berr,

	output reg           dma_req,
	output reg           dma_we,
	output reg    [23:1] dma_addr,
	output reg     [1:0] dma_be,
	output reg    [15:0] dma_wdata,
	input         [15:0] dma_rdata,
	input                dma_ack,

	output               sndint,
	output               soundint,

	input  signed [15:0] psg_audio,
	input  signed [15:0] mic_l,
	input  signed [15:0] mic_r,
	output reg signed [15:0] audio_l,
	output reg signed [15:0] audio_r,
	output reg           audio_stb,

	output reg           ssi_slot_stb,
	output reg           ssi_frame,
	output        [15:0] ssi_rx_data,
	input         [15:0] ssi_tx_data,
	input                ssi_tx_valid,
	input                ssi_hs_play_req,
	output reg           ssi_tx_en,
	output reg           ssi_rx_en,
	output reg           ssi_rx_frame,

	output reg           dbg_underrun
);

localparam FIFO_WORDS = 1 << FIFO_AW;
localparam integer MW_CLKS_I = (CLK_HZ / 2000000) > 0 ? (CLK_HZ / 2000000) : 1; // 8 cycles at 16 MHz
localparam [7:0] MW_CLKS = MW_CLKS_I[7:0];
localparam [7:0] LPULSE  = LINE_PULSE;

// ------------------------------------------------------------------------
// Tables (crossbar.c)
// ------------------------------------------------------------------------
function [16:0] ste_rate(input [1:0] f);
	case (f)
	2'd0: ste_rate = 17'd6258;
	2'd1: ste_rate = 17'd12517;
	2'd2: ste_rate = 17'd25033;
	default: ste_rate = 17'd50066;
	endcase
endfunction

function [16:0] rate_25(input [3:0] d);
	case (d)
	4'd1:  rate_25 = 17'd49170;
	4'd2:  rate_25 = 17'd32780;
	4'd3:  rate_25 = 17'd24585;
	4'd4:  rate_25 = 17'd19668;
	4'd5:  rate_25 = 17'd16390;
	4'd6:  rate_25 = 17'd14049;
	4'd7:  rate_25 = 17'd12292;
	4'd8:  rate_25 = 17'd10927;
	4'd9:  rate_25 = 17'd9834;
	4'd10: rate_25 = 17'd8940;
	4'd11: rate_25 = 17'd8195;
	4'd12: rate_25 = 17'd7565;
	4'd13: rate_25 = 17'd7024;
	4'd14: rate_25 = 17'd6556;
	4'd15: rate_25 = 17'd6146;
	default: rate_25 = 17'd0;
	endcase
endfunction

function [16:0] rate_32(input [3:0] d);
	case (d)
	4'd1:  rate_32 = 17'd62500;
	4'd2:  rate_32 = 17'd41666;
	4'd3:  rate_32 = 17'd31250;
	4'd4:  rate_32 = 17'd25000;
	4'd5:  rate_32 = 17'd20833;
	4'd6:  rate_32 = 17'd17857;
	4'd7:  rate_32 = 17'd15624;
	4'd8:  rate_32 = 17'd13889;
	4'd9:  rate_32 = 17'd12500;
	4'd10: rate_32 = 17'd11363;
	4'd11: rate_32 = 17'd10416;
	4'd12: rate_32 = 17'd9615;
	4'd13: rate_32 = 17'd8928;
	4'd14: rate_32 = 17'd8333;
	4'd15: rate_32 = 17'd7812;
	default: rate_32 = 17'd0;
	endcase
endfunction

// Crossbar_ADC_volume_table
function [15:0] adc_gain(input [3:0] g);
	case (g)
	4'd0:  adc_gain = 16'd3276;
	4'd1:  adc_gain = 16'd3894;
	4'd2:  adc_gain = 16'd4628;
	4'd3:  adc_gain = 16'd5500;
	4'd4:  adc_gain = 16'd6537;
	4'd5:  adc_gain = 16'd7769;
	4'd6:  adc_gain = 16'd9234;
	4'd7:  adc_gain = 16'd10975;
	4'd8:  adc_gain = 16'd13043;
	4'd9:  adc_gain = 16'd15502;
	4'd10: adc_gain = 16'd18424;
	4'd11: adc_gain = 16'd21897;
	4'd12: adc_gain = 16'd26025;
	4'd13: adc_gain = 16'd30931;
	4'd14: adc_gain = 16'd36761;
	default: adc_gain = 16'd43691;
	endcase
endfunction

// Crossbar_DAC_volume_table
function [15:0] dac_att(input [3:0] a);
	case (a)
	4'd0:  dac_att = 16'd65535;
	4'd1:  dac_att = 16'd55142;
	4'd2:  dac_att = 16'd46396;
	4'd3:  dac_att = 16'd39037;
	4'd4:  dac_att = 16'd32846;
	4'd5:  dac_att = 16'd27636;
	4'd6:  dac_att = 16'd23253;
	4'd7:  dac_att = 16'd19565;
	4'd8:  dac_att = 16'd16462;
	4'd9:  dac_att = 16'd13851;
	4'd10: dac_att = 16'd11654;
	4'd11: dac_att = 16'd9806;
	4'd12: dac_att = 16'd8250;
	4'd13: dac_att = 16'd6942;
	4'd14: dac_att = 16'd5841;
	default: dac_att = 16'd4915;
	endcase
endfunction

function [15:0] merge16(input [15:0] old, input [15:0] din, input uds, input lds);
	merge16 = { uds ? din[15:8] : old[15:8], lds ? din[7:0] : old[7:0] };
endfunction

function [15:0] sat16(input signed [19:0] v);
	if (v > 20'sd32767)       sat16 = 16'h7FFF;
	else if (v < -20'sd32768) sat16 = 16'h8000;
	else                      sat16 = v[15:0];
endfunction

// ------------------------------------------------------------------------
// Registers
// ------------------------------------------------------------------------
reg  [7:0]  r_8900, r_8901;
reg  [7:0]  sh_st [0:2];       // IoMem shadows: start H/M/L
reg  [7:0]  sh_en [0:2];       // end
reg  [23:0] play_start_reg, play_end_reg, rec_start_reg, rec_end_reg;
reg  [7:0]  r_8920, r_8921;
reg  [15:0] r_mw;
reg  [7:0]  mw_cnt;
reg  [15:0] r_src, r_dst;
reg  [7:0]  r_8934, r_8935, r_8936, r_8937, r_8938, r_8939;
reg  [15:0] r_att, r_codec, r_893e, r_gpdir, r_gpdat;

// Decoded settings
wire [3:0]  div      = r_8935[3:0];
wire        ste_mode = (div == 4'd0);
wire [2:0]  tracks   = {1'b0, r_8920[1:0]} + 3'd1;
wire [3:0]  slots    = {tracks, 1'b0};               // 2 * tracks
wire [1:0]  mon      = r_8920[5:4];
wire        m16      = r_8921[6];
wire        mstereo  = ~r_8921[7];
wire [1:0]  dx_clk   = r_src[6:5];                   // dspXmit_freq
wire [1:0]  dp_clk   = r_src[2:1];                   // dmaPlay_freq
wire        dx_on    = r_src[7];                     // DSP xmit connected (not tristated)
wire        dr_on    = r_dst[7];                     // DSP receive connected
wire [1:0]  dac_src  = r_dst[14:13];
wire [1:0]  dsp_src  = r_dst[6:5];
wire [1:0]  rec_src  = r_dst[2:1];
wire        rec_hs   = (r_dst[3:0] == 4'b0010);      // dmaRecord.isConnectedToDspInHandShakeMode
wire        dac_muted = (ste_mode && r_8921[1:0] == 2'd0) ||
                        div == 4'd6 || div == 4'd8 || div == 4'd10 || div >= 4'd12;

// ------------------------------------------------------------------------
// Slot clocks
// ------------------------------------------------------------------------
wire [16:0] r25 = ste_mode ? ste_rate(r_8921[1:0]) : rate_25(div);
wire [16:0] r32 = ste_mode ? 17'd0 : rate_32(div);
wire [20:0] inc25 = r25 * slots;
wire [20:0] inc32 = r32 * slots;
reg  [31:0] acc25, acc32;
wire [31:0] acc25_n = acc25 + {11'd0, inc25};
wire [31:0] acc32_n = acc32 + {11'd0, inc32};

// ------------------------------------------------------------------------
// State
// ------------------------------------------------------------------------
reg         t25_p, t32_p;       // pending slot ticks
reg         sndint_r, soundint_r;
assign sndint   = sndint_r;
assign soundint = soundint_r;

// DMA play
reg         play_run, play_loop, play_prime;
reg  [24:0] play_fstart, play_fend, play_pos;
reg  [2:0]  play_cf;
reg  [7:0]  play_rearm;
reg         play_hsf;           // dmaPlay.handshakeMode_Frame
reg         play_hsm;           // dmaPlay.handshakeMode_masterClk
reg  [15:0] play_save;          // crossbar.save_special_transfer
wire        play_hs_conn = (r_dst[6:4] == 3'b000);  // dmaPlay.isConnectedToDspInHandShakeMode
// prefetcher
reg  [24:0] pf_addr, pf_end, pf_nstart, pf_nend;
reg         pf_first, pf_crossed;
reg  [15:0] fifo_mem [0:FIFO_WORDS-1];
reg  [FIFO_AW-1:0] f_rd, f_wr;
reg  [FIFO_AW:0]   f_cnt;
reg  [3:0]  f_debt;
// DMA record
reg         rec_run, rec_loop;
reg  [24:0] rec_fstart, rec_fend, rec_pos;
reg  [7:0]  rec_rearm;
reg  [40:0] wq_mem [0:7];      // {addr[23:1], be[1:0], data[15:0]}
reg  [2:0]  wq_rd, wq_wr;
reg  [3:0]  wq_cnt;
// DMA port
reg         dma_busy, dma_kind_rd, dma_discard;
// DSP xmit / ADC
reg  [2:0]  dx_wc;
reg         adc_wc;
// TX capture (slot strobe -> routing on a later clock)
reg         txc_p, txc_dac, txc_rec;
reg  [2:0]  txc_pos;
reg  [15:0] txc_data;
reg         rx_from_tx;
reg  [15:0] rx_data_r;
assign ssi_rx_data = rx_from_tx ? ssi_tx_data : rx_data_r;
// DAC
reg  [15:0] dac_pl, dac_l, dac_r;
reg         dac_wr_since, dac_valid;
reg  [3:0]  afc;                // audio frame slot counter (25 MHz slots)
reg         aud_go;

// ------------------------------------------------------------------------
// Working variables (blocking, this always block only)
// ------------------------------------------------------------------------
reg         v_snd, v_scnt;
reg  [7:0]  v_8900, v_8901;
reg         v_play_run, v_play_loop, v_play_prime;
reg  [24:0] v_play_fstart, v_play_fend, v_play_pos;
reg  [2:0]  v_play_cf;
reg  [7:0]  v_play_rearm;
reg  [24:0] v_pf_addr, v_pf_end, v_pf_nstart, v_pf_nend;
reg         v_pf_first, v_pf_crossed;
reg         v_flush;
reg         v_pop;
reg         v_rec_run, v_rec_loop;
reg  [24:0] v_rec_fstart, v_rec_fend, v_rec_pos;
reg  [7:0]  v_rec_rearm;
reg         v_wq_push;
reg  [40:0] v_wq_ent;
reg  [2:0]  v_dx_wc;
reg         v_adc_wc;
reg  [15:0] v_dac_pl, v_dac_l, v_dac_r;
reg         v_dac_wr;
reg         v_stb, v_txen, v_txframe, v_rxen, v_rxframe, v_rxfromtx;
reg  [15:0] v_rxdata;
reg         v_txc_dac, v_txc_rec;
reg  [2:0]  v_txc_pos;
reg  [3:0]  v_afc;
reg         v_aud_go;
reg  [15:0] v_w;
reg  [7:0]  v_b;
reg  [15:0] head;
reg         v_empty;
reg  [24:0] v_tmp;
reg         v_rec_req;
reg  [15:0] v_rec_val;
reg         do_dx, do_dp;
reg         v_t25c, v_t32c;     // pending tick consumed in this clock
reg         v_play_hsf, v_play_hsm;
reg  [15:0] v_play_save, v_save;

// ---- line update (Crossbar_Update_DMA_Sound_Line) ----
task line_upd(input is_play, input bitv);
	reg en_g, en_t;
	begin
		en_g = is_play ? v_8900[0] : v_8900[1];
		en_t = is_play ? v_8900[2] : v_8900[3];
		v_snd  = en_g ? bitv : 1'b1;
		v_scnt = en_t ? bitv : 1'b1;
	end
endtask

// ---- Crossbar_SendDataToDAC ----
task dac_write(input [15:0] val, input [2:0] pos);
	begin
		v_dac_wr = 1'b1;
		if (pos == {mon, 1'b0})
			v_dac_pl = val;
		else if (pos == {mon, 1'b1}) begin
			v_dac_l = v_dac_pl;
			v_dac_r = val;
		end
	end
endtask

// ---- Crossbar_SendDataToDspReceive ----
task dsp_rx(input [15:0] val, input frame, input from_tx);
	begin
		if (dr_on) begin
			v_stb      = 1'b1;
			v_rxen     = 1'b1;
			// SC1 is not sent while dmaPlay.handshakeMode_Frame is set
			v_rxframe  = v_play_hsf ? 1'b0 : frame;
			v_rxdata   = val;
			v_rxfromtx = from_tx;
			v_play_hsf = 1'b0;
		end
	end
endtask

// ---- Crossbar_setDmaRecord_Settings ----
task rec_settings;
	begin
		v_rec_fstart = {1'b0, rec_start_reg};
		v_rec_fend   = {1'b0, rec_end_reg};
		v_rec_pos    = {1'b0, rec_start_reg};
		line_upd(1'b0, 1'b0);
	end
endtask

// ---- Crossbar_SendDataToDmaRecord ----
task rec_push(input [15:0] val);
	begin
		if (v_rec_run) begin
			if (m16 || mstereo) begin
				v_wq_ent  = {v_rec_pos[23:1], 2'b11, val};
				v_rec_pos = v_rec_pos + 25'd2;
			end else begin
				v_wq_ent  = {v_rec_pos[23:1], v_rec_pos[0] ? 2'b01 : 2'b10, val[7:0], val[7:0]};
				v_rec_pos = v_rec_pos + 25'd1;
			end
			v_wq_push = 1'b1;
			if (v_rec_pos >= v_rec_fend) begin
				line_upd(1'b0, 1'b1);
				if (v_rec_loop) begin
					v_rec_fstart = {1'b0, rec_start_reg};
					v_rec_fend   = {1'b0, rec_end_reg};
					v_rec_pos    = {1'b0, rec_start_reg};
					v_rec_rearm  = LPULSE;
				end else begin
					v_8901[4]  = 1'b0;
					v_rec_run  = 1'b0;
					v_rec_loop = 1'b0;
				end
			end
		end
	end
endtask

// ---- a word for DMA record (one per clock; executed after the slot processes) ----
task rec_request(input [15:0] val);
	begin
		v_rec_req = 1'b1;
		v_rec_val = val;
	end
endtask

// ---- prefetch restart at address s..e ----
task pf_restart(input [24:0] s, input [24:0] e);
	begin
		v_flush      = 1'b1;
		v_pf_addr    = s;
		v_pf_end     = e;
		v_pf_first   = 1'b1;
		v_pf_crossed = 1'b0;
		v_play_prime = 1'b1;
	end
endtask

// ---- Crossbar_setDmaPlay_Settings (start) ----
task play_settings_start;
	begin
		v_play_fstart = {1'b0, play_start_reg};
		v_play_fend   = {1'b0, play_end_reg};
		v_play_pos    = {1'b0, play_start_reg};
		v_play_cf     = 3'd0;
		pf_restart({1'b0, play_start_reg}, {1'b0, play_end_reg});
		line_upd(1'b1, 1'b0);
	end
endtask

// ---- Crossbar_Process_DSPXmit_Transfer ----
task proc_dspxmit;
	begin
		if (dx_on) begin
			if (rec_hs) begin
				// Crossbar_Process_DMARecord_HandshakeMode
				if (v_rec_run && ssi_tx_valid) begin
					v_stb     = 1'b1;
					v_txen    = 1'b1;
					v_txframe = 1'b0;
					v_txc_rec = 1'b1;
				end
			end
			else if (dac_src == 2'b01 || dsp_src == 2'b01 || rec_src == 2'b01) begin
				v_stb     = 1'b1;
				v_txen    = 1'b1;
				v_txframe = (v_dx_wc == 3'd0);
				v_txc_dac = (dac_src == 2'b01);
				v_txc_rec = (rec_src == 2'b01);
				v_txc_pos = v_dx_wc;
				if (dsp_src == 2'b01)
					dsp_rx(16'h0000, v_dx_wc == 3'd0, 1'b1);
				v_dx_wc = ({1'b0, v_dx_wc} + 4'd1 >= slots) ? 3'd0 : v_dx_wc + 3'd1;
			end
		end
	end
endtask

// ---- Crossbar_Process_DMAPlay_Transfer ----
task proc_dmaplay;
	reg [15:0] val;
	reg [15:0] dacval;
	reg        inc_b, inc_w, popit;
	begin
		val = 16'h0000; dacval = 16'h0000;
		inc_b = 1'b0; inc_w = 1'b0; popit = 1'b0;
		// handshake mode with the DSP as master clock: no transfer until the
		// DSP asked for a word (Crossbar_DmaPlayInHandShakeMode)
		if (v_play_run && !(v_play_prime && v_empty) &&
		    !(play_hs_conn && v_play_hsm && !v_play_hsf)) begin
			v_play_prime = 1'b0;
			inc_b = 1'b0; inc_w = 1'b0; popit = 1'b0;
			v_w = v_empty ? 16'h0000 : head;
			v_b = v_play_pos[0] ? v_w[7:0] : v_w[15:8];
			if (m16) begin
				val    = v_w;
				inc_w  = 1'b1;
				popit  = 1'b1;
			end else begin
				val    = {{8{v_b[7]}}, v_b};
				if (mstereo || v_play_cf[0]) begin
					inc_b = 1'b1;
					popit = v_play_pos[0];
				end
			end
			if (popit) v_pop = 1'b1;
			if (inc_w) v_play_pos = v_play_pos + 25'd2;
			if (inc_b) v_play_pos = v_play_pos + 25'd1;

			// nocrew special transfer: DMA play -> DSP receive in handshake
			// mode at 32 MHz shifts the data 2 bits left across words
			if (play_hs_conn && v_play_hsm && dp_clk == 2'b10) begin
				v_save = val;
				val    = {v_play_save[13:0], val[15:14]};
				v_play_save = v_save;
			end
			// Crossbar_SendDataToDAC(value * eightBits): 16-bit truncation
			dacval = m16 ? val : {val[9:0], 6'd0};

			if (rec_src == 2'b00)  rec_request(val);
			if (dac_src == 2'b00)  dac_write(dacval, v_play_cf);
			if (dsp_src == 2'b00)  dsp_rx(val, v_play_cf == 3'd0, 1'b0);

			v_play_cf = ({1'b0, v_play_cf} + 4'd1 >= slots) ? 3'd0 : v_play_cf + 3'd1;

			if (v_play_pos >= v_play_fend) begin
				line_upd(1'b1, 1'b1);
				// a half consumed word (illegal frame) is dropped
				if (!m16 && v_play_pos[0] && !popit) v_pop = 1'b1;
				if (v_play_loop) begin
					v_play_fstart = {1'b0, play_start_reg};
					v_play_fend   = {1'b0, play_end_reg};
					v_play_pos    = {1'b0, play_start_reg};
					v_play_cf     = 3'd0;
					v_play_rearm  = LPULSE;
					if (v_pf_crossed && v_pf_nstart == {1'b0, play_start_reg} &&
					    v_pf_nend == {1'b0, play_end_reg})
						v_pf_crossed = 1'b0;
					else
						pf_restart({1'b0, play_start_reg}, {1'b0, play_end_reg});
				end else begin
					v_8901[0]   = 1'b0;
					v_play_run  = 1'b0;
					v_play_loop = 1'b0;
					v_flush     = 1'b1;
				end
			end
		end
	end
endtask

// ---- Crossbar_Process_ADCXmit_Transfer ----
task proc_adc;
	reg [15:0] s;
	begin
		v_adc_wc = ~v_adc_wc;
		if (!v_adc_wc)
			s = r_8938[1] ? psg_audio : mic_l;
		else
			s = r_8938[0] ? psg_audio : mic_r;
		if (dsp_src == 2'b11) dsp_rx(s, !v_adc_wc, 1'b0);
		if (rec_src == 2'b11) rec_request(s);
		if (dac_src == 2'b11) dac_write(s, {2'b00, v_adc_wc});
	end
endtask

// ------------------------------------------------------------------------
// Bus read mux (combinational, registered at bus_stb)
// ------------------------------------------------------------------------
wire [5:0]  wa = bus_addr[6:1];
wire        sel_rec = r_8901[7];
wire [24:0] cnt_val = sel_rec ? rec_pos : play_pos;
wire [23:0] st_val  = sel_rec ? rec_start_reg : play_start_reg;
wire [23:0] en_val  = sel_rec ? rec_end_reg : play_end_reg;
reg  [15:0] rd_val;
reg         rd_berr;
always @* begin
	rd_berr = 1'b0;
	case (wa)
	6'h00: rd_val = {r_8900, r_8901};
	6'h01: rd_val = {8'hFF, st_val[23:16]};
	6'h02: rd_val = {8'hFF, st_val[15:8]};
	6'h03: rd_val = {8'hFF, st_val[7:0]};
	6'h04: rd_val = {8'hFF, cnt_val[23:16]};
	6'h05: rd_val = {8'hFF, cnt_val[15:8]};
	6'h06: rd_val = {8'hFF, cnt_val[7:0]};
	6'h07: rd_val = {8'hFF, en_val[23:16]};
	6'h08: rd_val = {8'hFF, en_val[15:8]};
	6'h09: rd_val = {8'hFF, en_val[7:0]};
	6'h10: rd_val = {r_8920, r_8921};
	6'h11: rd_val = 16'h0000;
	6'h12: rd_val = r_mw;
	6'h18: rd_val = r_src;
	6'h19: rd_val = r_dst;
	6'h1A: rd_val = {r_8934, r_8935};
	6'h1B: rd_val = {r_8936, r_8937};
	6'h1C: rd_val = {r_8938, r_8939};
	6'h1D: rd_val = r_att;
	6'h1E: rd_val = r_codec;
	6'h1F: rd_val = r_893e;
	6'h20: rd_val = r_gpdir;
	6'h21: rd_val = r_gpdat;
	default: begin rd_val = 16'hFFFF; rd_berr = 1'b1; end
	endcase
end

wire bus_wr = bus_stb & bus_we;

// ------------------------------------------------------------------------
// Main sequential block
// ------------------------------------------------------------------------
integer i;
always @(posedge clk) begin
	bus_ack      <= 1'b0;
	bus_berr     <= 1'b0;
	ssi_slot_stb <= 1'b0;
	ssi_tx_en    <= 1'b0;
	ssi_rx_en    <= 1'b0;
	aud_go       <= 1'b0;

	if (reset) begin
		r_8900 <= 8'h05; r_8901 <= 8'h00;
		for (i = 0; i < 3; i = i + 1) begin
			sh_st[i] <= (i == 2) ? 8'hFE : 8'hFF;
			sh_en[i] <= (i == 2) ? 8'hFE : 8'hFF;
		end
		play_start_reg <= 24'hFFFFFE; play_end_reg <= 24'hFFFFFE;
		rec_start_reg  <= 24'hFFFFFE; rec_end_reg  <= 24'hFFFFFE;
		r_8920 <= 8'h00; r_8921 <= 8'h03;
		r_mw <= 16'h0000; mw_cnt <= 8'd0;
		r_src <= 16'h0000; r_dst <= 16'h0000;
		r_8934 <= 8'h00; r_8935 <= 8'h00; r_8936 <= 8'h00;
		r_8937 <= 8'h03; r_8938 <= 8'h03; r_8939 <= 8'h00;
		r_att <= 16'h0000; r_codec <= 16'h2401; r_893e <= 16'h0000;
		r_gpdir <= 16'h0000; r_gpdat <= 16'h0000;
		acc25 <= 32'd0; acc32 <= 32'd0; t25_p <= 1'b0; t32_p <= 1'b0;
		sndint_r <= 1'b1; soundint_r <= 1'b1;
		play_run <= 1'b0; play_loop <= 1'b0; play_prime <= 1'b0;
		play_fstart <= 25'hFFFFFE; play_fend <= 25'hFFFFFE; play_pos <= 25'hFFFFFE;
		play_cf <= 3'd0; play_rearm <= 8'd0;
		play_hsf <= 1'b0; play_hsm <= 1'b0; play_save <= 16'd0;
		pf_addr <= 25'd0; pf_end <= 25'd0; pf_nstart <= 25'd0; pf_nend <= 25'd0;
		pf_first <= 1'b0; pf_crossed <= 1'b0;
		f_rd <= 0; f_wr <= 0; f_cnt <= 0; f_debt <= 4'd0;
		rec_run <= 1'b0; rec_loop <= 1'b0;
		rec_fstart <= 25'hFFFFFE; rec_fend <= 25'hFFFFFE; rec_pos <= 25'hFFFFFE;
		rec_rearm <= 8'd0;
		wq_rd <= 3'd0; wq_wr <= 3'd0; wq_cnt <= 4'd0;
		dma_busy <= 1'b0; dma_req <= 1'b0; dma_we <= 1'b0; dma_kind_rd <= 1'b0;
		dma_discard <= 1'b0; dma_addr <= 23'd0; dma_be <= 2'b00; dma_wdata <= 16'd0;
		dx_wc <= 3'd0; adc_wc <= 1'b0;
		txc_p <= 1'b0; txc_dac <= 1'b0; txc_rec <= 1'b0;
		txc_pos <= 3'd0; txc_data <= 16'd0;
		rx_from_tx <= 1'b0; rx_data_r <= 16'd0;
		ssi_frame <= 1'b0; ssi_rx_frame <= 1'b0;
		dac_pl <= 16'd0; dac_l <= 16'd0; dac_r <= 16'd0;
		dac_wr_since <= 1'b0; dac_valid <= 1'b0; afc <= 4'd0;
		dbg_underrun <= 1'b0;
	end else begin
		// ---------------- load working copies ----------------
		v_snd = sndint_r; v_scnt = soundint_r;
		v_8900 = r_8900; v_8901 = r_8901;
		v_play_run = play_run; v_play_loop = play_loop; v_play_prime = play_prime;
		v_play_fstart = play_fstart; v_play_fend = play_fend; v_play_pos = play_pos;
		v_play_cf = play_cf; v_play_rearm = play_rearm;
		v_play_hsf = play_hsf; v_play_hsm = play_hsm; v_play_save = play_save; v_save = 16'd0;
		v_pf_addr = pf_addr; v_pf_end = pf_end; v_pf_nstart = pf_nstart; v_pf_nend = pf_nend;
		v_pf_first = pf_first; v_pf_crossed = pf_crossed;
		v_flush = 1'b0; v_pop = 1'b0;
		v_rec_run = rec_run; v_rec_loop = rec_loop;
		v_rec_fstart = rec_fstart; v_rec_fend = rec_fend; v_rec_pos = rec_pos;
		v_rec_rearm = rec_rearm;
		v_wq_push = 1'b0; v_wq_ent = 41'd0;
		v_rec_req = 1'b0; v_rec_val = 16'd0;
		v_t25c = 1'b0; v_t32c = 1'b0;
		v_dx_wc = dx_wc; v_adc_wc = adc_wc;
		v_dac_pl = dac_pl; v_dac_l = dac_l; v_dac_r = dac_r; v_dac_wr = 1'b0;
		v_stb = 1'b0; v_txen = 1'b0; v_txframe = 1'b0; v_rxen = 1'b0; v_rxframe = 1'b0;
		v_rxfromtx = 1'b0; v_rxdata = 16'd0;
		v_txc_dac = 1'b0; v_txc_rec = 1'b0; v_txc_pos = 3'd0;
		v_afc = afc; v_aud_go = 1'b0;
		head = fifo_mem[f_rd];
		v_empty = (f_cnt == 0);

		// ---------------- slot clocks ----------------
		if (acc25_n >= CLK_HZ) acc25 <= acc25_n - CLK_HZ;
		else acc25 <= acc25_n;
		if (acc32_n >= CLK_HZ) acc32 <= acc32_n - CLK_HZ;
		else acc32 <= acc32_n;

		// ---------------- Microwire mask restore ----------------
		if (mw_cnt != 8'd0) begin
			mw_cnt <= mw_cnt - 8'd1;
			if (mw_cnt == 8'd1) r_mw <= ~r_mw;
		end

		// ---------------- register bus ----------------
		if (bus_stb) begin
			bus_ack  <= 1'b1;
			bus_dout <= rd_val;
			bus_berr <= rd_berr;
			if (!bus_we) begin
				// read side effects: copy the byte into the IoMem shadow
				if (bus_lds) case (wa)
					6'h01: sh_st[0] <= st_val[23:16];
					6'h02: sh_st[1] <= st_val[15:8];
					6'h03: sh_st[2] <= st_val[7:0];
					6'h07: sh_en[0] <= en_val[23:16];
					6'h08: sh_en[1] <= en_val[15:8];
					6'h09: sh_en[2] <= en_val[7:0];
					default: ;
				endcase
			end else begin
				case (wa)
				6'h00: begin
					if (bus_uds) v_8900 = bus_din[15:8];
					if (bus_lds) begin
						// Crossbar_DmaCtrlReg_WriteByte
						v_8901 = bus_din[7:0];
						if (!v_play_run && v_8901[0]) begin
							v_play_run  = 1'b1;
							v_play_loop = v_8901[1];
							play_settings_start;
						end else if (v_play_run && !v_8901[0]) begin
							v_play_run  = 1'b0;
							v_play_loop = 1'b0;
							v_flush     = 1'b1;
							v_play_rearm = 8'd0;
							line_upd(1'b1, 1'b1);
						end
						if (!v_rec_run && v_8901[4]) begin
							v_rec_run  = 1'b1;
							v_rec_loop = v_8901[5];
							rec_settings;
						end else if (v_rec_run && !v_8901[4]) begin
							v_rec_run  = 1'b0;
							v_rec_loop = 1'b0;
							v_rec_rearm = 8'd0;
							line_upd(1'b0, 1'b1);
						end
					end
				end
				6'h01, 6'h02, 6'h03: if (bus_lds) begin
					v_tmp = {1'b0, (wa == 6'h01) ? bus_din[7:0] : sh_st[0],
					               (wa == 6'h02) ? bus_din[7:0] : sh_st[1],
					               (wa == 6'h03) ? bus_din[7:0] : sh_st[2]};
					sh_st[wa[1:0] - 2'd1] <= bus_din[7:0];
					if (sel_rec) rec_start_reg  <= {v_tmp[23:1], 1'b0};
					else         play_start_reg <= {v_tmp[23:1], 1'b0};
				end
				// 6'h04..6'h06: frame count writes have no effect (Hatari stores
				// dmaPlay/Record_CurrentFrameCount, which nothing uses)
				6'h07, 6'h08, 6'h09: if (bus_lds) begin
					v_tmp = {1'b0, (wa == 6'h07) ? bus_din[7:0] : sh_en[0],
					               (wa == 6'h08) ? bus_din[7:0] : sh_en[1],
					               (wa == 6'h09) ? bus_din[7:0] : sh_en[2]};
					sh_en[wa[1:0] - 2'd3] <= bus_din[7:0];
					if (sel_rec) rec_end_reg  <= {v_tmp[23:1], 1'b0};
					else         play_end_reg <= {v_tmp[23:1], 1'b0};
				end
				6'h10: begin
					if (bus_uds) r_8920 <= bus_din[15:8];
					if (bus_lds) r_8921 <= bus_din[7:0];
				end
				6'h12: begin
					r_mw   <= ~merge16(r_mw, bus_din, bus_uds, bus_lds);
					mw_cnt <= MW_CLKS;
				end
				6'h18: r_src <= merge16(r_src, bus_din, bus_uds, bus_lds);
				6'h19: begin
					// Crossbar_DstControler_WriteWord
					v_w = merge16(r_dst, bus_din, bus_uds, bus_lds);
					r_dst <= v_w;
					v_play_hsf = (v_w[6:4] == 3'b000);
					v_play_hsm = 1'b0;
				end
				6'h1A: begin
					if (bus_uds) r_8934 <= bus_din[15:8];
					if (bus_lds) r_8935 <= bus_din[7:0];
				end
				6'h1B: begin
					if (bus_uds) r_8936 <= bus_din[15:8];
					if (bus_lds) r_8937 <= bus_din[7:0];
				end
				6'h1C: begin
					if (bus_uds) r_8938 <= bus_din[15:8];
					if (bus_lds) r_8939 <= bus_din[7:0];
				end
				6'h1D: r_att   <= merge16(r_att,   bus_din, bus_uds, bus_lds);
				6'h1E: r_codec <= merge16(r_codec, bus_din, bus_uds, bus_lds);
				6'h1F: r_893e  <= merge16(r_893e,  bus_din, bus_uds, bus_lds);
				6'h20: r_gpdir <= merge16(r_gpdir, bus_din, bus_uds, bus_lds);
				6'h21: r_gpdat <= merge16(r_gpdat, bus_din, bus_uds, bus_lds);
				default: ;
				endcase
			end
		end

		// ---------------- DSP request (DSP_SsiTransmit_SC1 -> Crossbar_DmaPlayInHandShakeMode) ----------------
		if (ssi_hs_play_req) begin
			v_play_hsm = 1'b1;
			v_play_hsf = 1'b1;
		end

		// ---------------- SNDINT/SOUNDINT re-arm after a looped frame end ----------------
		if (v_play_rearm != 8'd0) begin
			v_play_rearm = v_play_rearm - 8'd1;
			if (v_play_rearm == 8'd0 && v_play_run) line_upd(1'b1, 1'b0);
		end
		if (v_rec_rearm != 8'd0) begin
			v_rec_rearm = v_rec_rearm - 8'd1;
			if (v_rec_rearm == 8'd0 && v_rec_run) line_upd(1'b0, 1'b0);
		end

		// ---------------- slot processing (not in a bus write clock) ----------------
		// Each process runs at most once per clock: DAC, DSP receive and DMA
		// record each have a single source, so the processes of one slot are
		// independent and the order of crossbar.c only matters for the
		// SNDINT/SOUNDINT updates (record end is applied after play end).
		// DSP transmit words of the last strobe are routed in a clock of their
		// own; slot ticks stay pending meanwhile (slots are >= 64 clocks apart).
		if (!bus_wr) begin
			if (txc_p) begin
				txc_p <= 1'b0;
				if (txc_dac) dac_write(txc_data, txc_pos);
				if (txc_rec) rec_request(txc_data);
			end else if (t25_p || t32_p) begin
				v_t25c = t25_p;
				v_t32c = t32_p;
				do_dx = ste_mode ? t25_p :
				        ((dx_clk == 2'b00 && t25_p) || (dx_clk == 2'b10 && t32_p));
				do_dp = ste_mode ? t25_p :
				        ((dp_clk == 2'b00 && t25_p) || (dp_clk == 2'b10 && t32_p));
				if (t25_p) proc_adc;
				if (do_dx) proc_dspxmit;
				if (do_dp) proc_dmaplay;
				// codec sample rate: one DAC sample per 25 MHz frame
				if (t25_p) begin
					if ({1'b0, v_afc} + 5'd1 >= {1'b0, slots}) begin
						v_afc    = 4'd0;
						v_aud_go = 1'b1;
					end else
						v_afc = v_afc + 4'd1;
				end
			end
			if (v_rec_req) rec_push(v_rec_val);
		end

		// pending slot ticks: a new tick never gets lost under a consumed one
		t25_p <= (t25_p & ~v_t25c) | (acc25_n >= CLK_HZ);
		t32_p <= (t32_p & ~v_t32c) | (acc32_n >= CLK_HZ);

		// ---------------- TX capture at the strobe ----------------
		if (ssi_slot_stb && ssi_tx_en) begin
			txc_p    <= 1'b1;
			txc_data <= ssi_tx_data;
		end

		// ---------------- DMA port ----------------
		begin : dmaport
			reg        push_ok;
			reg        want_rd, can_rd;
			reg [FIFO_AW:0] occ;
			reg        rd_data, drop;
			push_ok = 1'b0;
			rd_data = 1'b0;
			if (dma_ack && dma_busy) begin
				dma_req  <= 1'b0;
				dma_busy <= 1'b0;
				if (dma_kind_rd) begin
					rd_data = !dma_discard && !v_flush;
					dma_discard <= 1'b0;
				end
			end else if (dma_busy && dma_kind_rd && v_flush)
				dma_discard <= 1'b1;
			// words owed to slots that played during an underrun are dropped
			drop    = rd_data && (f_debt != 4'd0);
			push_ok = rd_data && !drop;

			// FIFO pointers
			if (v_flush) begin
				f_rd   <= f_wr;
				f_cnt  <= 0;
				f_debt <= 4'd0;
			end else begin
				if (push_ok) begin
					fifo_mem[f_wr] <= dma_rdata;
					f_wr <= f_wr + 1'b1;
				end
				if (v_pop && !v_empty) f_rd <= f_rd + 1'b1;
				if (v_pop && v_empty) dbg_underrun <= 1'b1;
				if ((v_pop && v_empty) && !drop) begin
					if (f_debt != 4'hF) f_debt <= f_debt + 4'd1;
				end else if (drop && !(v_pop && v_empty))
					f_debt <= f_debt - 4'd1;
				f_cnt <= f_cnt + {{FIFO_AW{1'b0}}, push_ok} - {{FIFO_AW{1'b0}}, v_pop && !v_empty};
			end

			// new request
			occ = f_cnt + {{FIFO_AW{1'b0}}, dma_busy && dma_kind_rd};
			want_rd = 1'b0;
			if (v_play_run && !v_flush && play_run && occ < FIFO_WORDS - 1) begin
				if (v_pf_addr < v_pf_end || v_pf_first)
					want_rd = 1'b1;
				else if (!v_pf_crossed && v_play_loop) begin
					// run into the next frame with the live registers
					v_pf_crossed = 1'b1;
					v_pf_nstart  = {1'b0, play_start_reg};
					v_pf_nend    = {1'b0, play_end_reg};
					v_pf_addr    = {1'b0, play_start_reg};
					v_pf_end     = {1'b0, play_end_reg};
					v_pf_first   = 1'b1;
				end
			end
			can_rd = want_rd && (occ < (FIFO_WORDS / 2) || wq_cnt == 4'd0);
			if (!dma_busy && !(dma_ack)) begin
				if (can_rd) begin
					dma_req     <= 1'b1;
					dma_busy    <= 1'b1;
					dma_we      <= 1'b0;
					dma_kind_rd <= 1'b1;
					dma_addr    <= v_pf_addr[23:1];
					dma_be      <= 2'b11;
					v_pf_addr   = v_pf_addr + 25'd2;
					v_pf_first  = 1'b0;
				end else if (wq_cnt != 4'd0) begin
					dma_req     <= 1'b1;
					dma_busy    <= 1'b1;
					dma_we      <= 1'b1;
					dma_kind_rd <= 1'b0;
					dma_addr    <= wq_mem[wq_rd][40:18];
					dma_be      <= wq_mem[wq_rd][17:16];
					dma_wdata   <= wq_mem[wq_rd][15:0];
				end
			end

			// write queue
			begin : wqueue
				reg wq_pop;
				wq_pop = !dma_busy && !dma_ack && !can_rd && wq_cnt != 4'd0;
				if (wq_pop) wq_rd <= wq_rd + 3'd1;
				if (v_wq_push && wq_cnt != 4'd8) begin
					wq_mem[wq_wr] <= v_wq_ent;
					wq_wr <= wq_wr + 3'd1;
				end
				wq_cnt <= wq_cnt + ((v_wq_push && wq_cnt != 4'd8) ? 4'd1 : 4'd0)
				                 - (wq_pop ? 4'd1 : 4'd0);
			end
		end

		// ---------------- write back working copies ----------------
		sndint_r <= v_snd; soundint_r <= v_scnt;
		r_8900 <= v_8900; r_8901 <= v_8901;
		play_run <= v_play_run; play_loop <= v_play_loop; play_prime <= v_play_prime;
		play_fstart <= v_play_fstart; play_fend <= v_play_fend; play_pos <= v_play_pos;
		play_cf <= v_play_cf; play_rearm <= v_play_rearm;
		play_hsf <= v_play_hsf; play_hsm <= v_play_hsm; play_save <= v_play_save;
		pf_addr <= v_pf_addr; pf_end <= v_pf_end; pf_nstart <= v_pf_nstart; pf_nend <= v_pf_nend;
		pf_first <= v_pf_first; pf_crossed <= v_pf_crossed;
		rec_run <= v_rec_run; rec_loop <= v_rec_loop;
		rec_fstart <= v_rec_fstart; rec_fend <= v_rec_fend; rec_pos <= v_rec_pos;
		rec_rearm <= v_rec_rearm;
		dx_wc <= v_dx_wc; adc_wc <= v_adc_wc;
		dac_pl <= v_dac_pl; dac_l <= v_dac_l; dac_r <= v_dac_r;
		afc <= v_afc;

		if (v_aud_go) begin
			dac_valid    <= dac_wr_since | v_dac_wr;
			dac_wr_since <= 1'b0;
			aud_go       <= 1'b1;
		end else if (v_dac_wr)
			dac_wr_since <= 1'b1;

		if (v_stb) begin
			ssi_slot_stb <= 1'b1;
			ssi_tx_en    <= v_txen;
			ssi_rx_en    <= v_rxen;
			ssi_frame    <= v_txen ? v_txframe : v_rxframe;
			ssi_rx_frame <= v_rxframe;
			rx_from_tx   <= v_rxfromtx;
			if (v_rxen && !v_rxfromtx) rx_data_r <= v_rxdata;
			if (v_txen) begin
				txc_dac <= v_txc_dac;
				txc_rec <= v_txc_rec;
				txc_pos <= v_txc_pos;
			end
		end
	end
end

// ------------------------------------------------------------------------
// Codec output pipeline (Crossbar_GenerateSamples, one sample)
// ------------------------------------------------------------------------
reg         a1, a2, a3;
reg  signed [15:0] s1_adcl, s1_adcr, s1_dacl, s1_dacr;
reg  [15:0] s1_gl, s1_gr, s1_al, s1_ar;
reg  [1:0]  s1_src;
reg         s1_mute;
reg  signed [32:0] s2_dl, s2_dr;
reg  signed [15:0] s2_dacl, s2_dacr;
reg  [15:0] s2_al, s2_ar;
reg  [1:0]  s2_src;
reg         s2_mute;
reg  signed [15:0] s3_suml, s3_sumr;
reg  [15:0] s3_al, s3_ar;
reg         s3_mute;
wire signed [18:0] dir_l = s2_dl[32:14];
wire signed [18:0] dir_r = s2_dr[32:14];
wire signed [15:0] dir_ls = sat16({dir_l[18], dir_l});
wire signed [15:0] dir_rs = sat16({dir_r[18], dir_r});
wire signed [32:0] o_l = s3_suml * $signed({1'b0, s3_al});
wire signed [32:0] o_r = s3_sumr * $signed({1'b0, s3_ar});

always @(posedge clk) begin
	audio_stb <= 1'b0;
	if (reset) begin
		a1 <= 1'b0; a2 <= 1'b0; a3 <= 1'b0;
		audio_l <= 16'sd0; audio_r <= 16'sd0;
	end else begin
		// stage 1: operands
		a1      <= aud_go;
		s1_adcl <= r_8938[1] ? psg_audio : mic_l;
		s1_adcr <= r_8938[0] ? psg_audio : mic_r;
		s1_dacl <= dac_valid ? dac_l : 16'sd0;
		s1_dacr <= dac_valid ? dac_r : 16'sd0;
		s1_gl   <= adc_gain(r_8939[7:4]);
		s1_gr   <= adc_gain(r_8939[3:0]);
		s1_al   <= dac_att(r_att[11:8]);
		s1_ar   <= dac_att(r_att[7:4]);
		s1_src  <= r_8937[1:0];
		s1_mute <= dac_muted;
		// stage 2: ADC gain
		a2      <= a1;
		s2_dl   <= s1_adcl * $signed({1'b0, s1_gl});
		s2_dr   <= s1_adcr * $signed({1'b0, s1_gr});
		s2_dacl <= s1_dacl; s2_dacr <= s1_dacr;
		s2_al   <= s1_al;   s2_ar   <= s1_ar;
		s2_src  <= s1_src;  s2_mute <= s1_mute;
		// stage 3: 16 bit adder
		a3 <= a2;
		case (s2_src)
		2'd0: begin s3_suml <= 16'sd0;  s3_sumr <= 16'sd0;  end
		2'd1: begin s3_suml <= dir_ls;  s3_sumr <= dir_rs;  end
		2'd2: begin s3_suml <= s2_dacl; s3_sumr <= s2_dacr; end
		default: begin
			s3_suml <= sat16({{4{dir_ls[15]}}, dir_ls} + {{4{s2_dacl[15]}}, s2_dacl});
			s3_sumr <= sat16({{4{dir_rs[15]}}, dir_rs} + {{4{s2_dacr[15]}}, s2_dacr});
		end
		endcase
		s3_al <= s2_al; s3_ar <= s2_ar; s3_mute <= s2_mute;
		// stage 4: attenuation, output
		if (a3) begin
			audio_stb <= 1'b1;
			audio_l   <= s3_mute ? 16'sd0 : o_l[31:16];
			audio_r   <= s3_mute ? 16'sd0 : o_r[31:16];
		end
	end
end

endmodule
