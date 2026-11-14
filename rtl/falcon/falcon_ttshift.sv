// falcon_ttshift.sv - the TT030 shifter's registers, for the TT build, over the Videl
//
// Behavioural reference: Hatari src/video.c (Video_TTShiftMode_WriteWord,
// Video_TTColorRegs_Write, Video_TTColorRegs_STRegWrite, TT2STColor,
// Video_UpdateTTPalette, Video_Res_WriteByte), src/ioMemTabTT.c.
//
// Registers (the rest of $FF8200-$FF820F stays the Videl's):
//   $FF8240-$FF825E  ST palette: the 16 TT colours of the current bank, in the STe bit
//                    order (word writes only; byte writes have no effect, as Hatari)
//   $FF8260          ST resolution (byte): also sets the TT resolution
//   $FF8262          TT shifter mode: bit 15 sample/hold, bit 12 hyper mono, bits 10:8
//                    resolution (0 ST low, 1 ST mid, 2 ST high, 4 TT medium, 6 TT high,
//                    7 TT low), bits 3:0 colour bank
//   $FF8264/$FF8265  stored, no effect (Hatari); $FF8266-$FF827F read 0
//   $FF8400-$FF85FF  the TT palette, 256 x 12 bits (4:4:4)
// A resolution or bank change loads the Videl with that mode's VGA register set
// (falcon_videl tt_set); palette writes go to a shadow copy here (for reads) and to the
// Videl's Falcon palette, which the Videl always uses in the TT build.
// Not modelled: hyper mono, sample/hold.  TT high is the Videl's (falcon_videl tthi).

module falcon_ttshift (
	input             clk,
	input             reset,

	input             bus_cs,
	input             bus_stb,
	input             bus_we,
	input      [10:1] bus_addr,       // word offset from $FF8000
	input             bus_uds,
	input             bus_lds,
	input      [15:0] bus_din,
	output reg [15:0] bus_dout,
	output reg        bus_ack,

	output reg        tt_set,
	output      [2:0] tt_res,
	output      [3:0] tt_bank,
	output            tt_duo_inv,
	output reg        tt_pal_we,
	output reg  [7:0] tt_pal_wa,
	output reg [11:0] tt_pal_wd
);

reg  [15:0] mode;                // $FF8262
reg  [15:0] hsc;                 // $FF8264/5
reg         duo_inv;
assign tt_res     = mode[10:8];
assign tt_bank    = mode[3:0];
assign tt_duo_inv = duo_inv;

// the palette shadow: 256 x 12
(* ramstyle = "M10K, no_rw_check" *) reg [11:0] pal [0:255];
reg  [11:0] pal_q;
reg   [7:0] pal_ra;
always @(posedge clk) pal_q <= pal[pal_ra];
always @(posedge clk) if (tt_pal_we) pal[tt_pal_wa] <= tt_pal_wd;

wire [10:0] off = {bus_addr, 1'b0};                 // offset from $FF8000
wire is_stp = off >= 11'h240 && off <= 11'h25E;
wire is_ttp = off[10];                              // $FF8400-$FF85FF
wire [7:0] ent = is_ttp ? bus_addr[8:1] : {mode[3:0], bus_addr[4:1]};

function [11:0] st2tt(input [11:0] c);
	st2tt = ((c & 12'h777) << 1) | ((c & 12'h888) >> 3);
endfunction
function [11:0] tt2st(input [11:0] c);
	tt2st = ((c & 12'hEEE) >> 1) | ((c & 12'h111) << 3);
endfunction

reg        ph;                   // 1: second clock of a palette access (the shadow's word is out)
reg        a_we, a_u, a_l, a_st;
reg [15:0] a_din;
reg  [7:0] a_ent;
reg        init;                 // load the Videl once after reset

always @(posedge clk) begin
	bus_ack   <= 0;
	tt_set    <= 0;
	tt_pal_we <= 0;
	if (reset) begin
		mode <= 16'h0000; hsc <= 0; duo_inv <= 0; ph <= 0; init <= 1;
	end else begin
		if (init) begin init <= 0; tt_set <= 1; end
		if (ph) begin
			ph      <= 0;
			bus_ack <= 1;
			bus_dout <= a_st ? {4'd0, tt2st(pal_q)} : {4'd0, pal_q};
			if (a_we) begin
				if (a_st) begin
					if (a_u && a_l) begin tt_pal_we <= 1; tt_pal_wd <= st2tt(a_din[11:0]); end
				end else begin
					tt_pal_we <= 1;
					tt_pal_wd <= {a_u ? a_din[11:8] : pal_q[11:8], a_l ? a_din[7:0] : pal_q[7:0]};
					if (a_ent == 8'd0 && a_l) duo_inv <= a_din[1];
				end
				tt_pal_wa <= a_ent;
			end
		end else if (bus_stb && bus_cs) begin
			if (is_stp || is_ttp) begin
				ph <= 1; pal_ra <= ent; a_ent <= ent; a_st <= is_stp;
				a_we <= bus_we; a_u <= bus_uds; a_l <= bus_lds; a_din <= bus_din;
			end else begin
				bus_ack <= 1;
				case (off)
					11'h260: bus_dout <= {6'd0, mode[9:8], 8'h00};
					11'h262: bus_dout <= mode;
					11'h264: bus_dout <= hsc;
					default: bus_dout <= 16'h0000;
				endcase
				if (bus_we) case (off)
					11'h260: if (bus_uds) begin
						mode[10:8] <= {1'b0, bus_din[9:8]};
						tt_set <= 1;
					end
					11'h262: begin
						if (bus_uds) mode[15:8] <= bus_din[15:8] & 8'h97;
						if (bus_lds) mode[7:0]  <= bus_din[7:0] & 8'h0F;
						tt_set <= 1;
					end
					11'h264: begin
						if (bus_uds) hsc[15:8] <= bus_din[15:8];
						if (bus_lds) hsc[7:0]  <= bus_din[7:0];
					end
					default: ;
				endcase
			end
		end
	end
end

endmodule
