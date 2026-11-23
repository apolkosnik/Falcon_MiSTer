// falcon_ttdma.sv - the TT030's SCSI DMA ($FF8701-$FF8715), for the TT build
//
// Behavioural reference: Hatari src/ncr5380.c (dma_check, the TT branch;
// Ncr5380_TT_DMA_Ctrl_WriteWord), src/ioMemTabTT.c; the TT030 Hardware
// Reference Manual (June 1990), "SCSI DMA".
//
// Registers (byte wide on the odd addresses, the even ones read 0):
//   $FF8701-$FF8707  DMA address, bits 31..0 (high byte first); counts up
//   $FF8709-$FF870F  byte count, bits 31..0; counts down
//   $FF8710-$FF8713  residue data: the last bytes of a read that do not fill a
//                    longword (TOS and ASV copy them out themselves)
//   $FF8715          control: bit 0 direction (1 = memory to SCSI), bit 1 DMA
//                    enable, bit 6 byte count zero, bit 7 bus error
// The 5380 (falcon_scsi) asks for each byte with dma_push_req (SCSI to
// memory) or dma_pull_req (memory to SCSI); with the DMA enabled, the direction
// matching and the count not zero, the byte goes to or comes from memory.
//
// Deviations: the TT's DMA moves longwords through a 4 byte buffer; here every
// byte is its own memory cycle, and on a read each byte also goes to memory at
// once (Hatari likewise writes the residue bytes to RAM as well).  The DMA
// interrupt (TT-MFP GPIP5) is not raised, as in Hatari; the count-zero bit is.
// An address outside the RAM sets the bus error bit and stops the transfer.

module falcon_ttdma (
	input             clk,
	input             reset,

	// register bus ($FF8700-$FF871F)
	input             bus_cs,
	input             bus_stb,
	input             bus_we,
	input       [4:1] bus_addr,
	input             bus_uds,
	input             bus_lds,
	input      [15:0] bus_din,
	output reg [15:0] bus_dout,
	output            bus_ack,

	// the 5380's handshake
	input             push_req,
	input       [7:0] push_byte,
	output reg        push_ack,
	input             pull_req,
	output reg  [7:0] pull_byte,
	output reg        pull_ack,

	// memory master (16 bit)
	output            dma_req,
	output reg        dma_we,
	output     [28:1] dma_addr,
	output reg  [1:0] dma_be,
	output     [15:0] dma_wdata,
	input      [15:0] dma_rdata,
	input             dma_ack,
	input             dma_bad,       // with dma_ack: the address is not RAM

	output            active         // a transfer in progress (for the port arbiter)
);

reg [31:0] addr, count, residue;
reg  [7:0] ctrl;
reg  [7:0] wbyte;
reg        req_r;
reg  [1:0] gap;                     // clocks after a handshake before the next request is taken

assign bus_ack   = bus_stb;
assign dma_req   = req_r & ~dma_ack;
assign dma_addr  = addr[28:1];
assign dma_wdata = {wbyte, wbyte};
assign active    = req_r;

wire go       = ctrl[1] && count != 0 && !ctrl[7] && !req_r && gap == 0;
wire take_in  = go && !ctrl[0] && push_req;
wire take_out = go &&  ctrl[0] && pull_req;

always @* begin
	bus_dout = 16'h0000;
	case (bus_addr)
		4'd0: bus_dout[7:0] = addr[31:24];
		4'd1: bus_dout[7:0] = addr[23:16];
		4'd2: bus_dout[7:0] = addr[15:8];
		4'd3: bus_dout[7:0] = addr[7:0];
		4'd4: bus_dout[7:0] = count[31:24];
		4'd5: bus_dout[7:0] = count[23:16];
		4'd6: bus_dout[7:0] = count[15:8];
		4'd7: bus_dout[7:0] = count[7:0];
		4'd8: bus_dout = residue[31:16];
		4'd9: bus_dout = residue[15:0];
		4'd10: bus_dout[7:0] = ctrl;
		default: ;
	endcase
end

always @(posedge clk) begin
	push_ack <= 0;
	pull_ack <= 0;
	if (gap != 0) gap <= gap - 1'd1;

	if (reset) begin
		addr <= 0; count <= 0; residue <= 0; ctrl <= 0;
		req_r <= 0; gap <= 0;
	end else begin
		if (bus_stb && bus_we) case (bus_addr)
			4'd0: if (bus_lds) addr[31:24]  <= bus_din[7:0];
			4'd1: if (bus_lds) addr[23:16]  <= bus_din[7:0];
			4'd2: if (bus_lds) addr[15:8]   <= bus_din[7:0];
			4'd3: if (bus_lds) addr[7:0]    <= bus_din[7:0];
			4'd4: if (bus_lds) count[31:24] <= bus_din[7:0];
			4'd5: if (bus_lds) count[23:16] <= bus_din[7:0];
			4'd6: if (bus_lds) count[15:8]  <= bus_din[7:0];
			4'd7: if (bus_lds) count[7:0]   <= bus_din[7:0];
			4'd8: begin if (bus_uds) residue[31:24] <= bus_din[15:8]; if (bus_lds) residue[23:16] <= bus_din[7:0]; end
			4'd9: begin if (bus_uds) residue[15:8]  <= bus_din[15:8]; if (bus_lds) residue[7:0]   <= bus_din[7:0]; end
			4'd10: if (bus_lds) ctrl <= {bus_din[7:6], 4'd0, bus_din[1:0]};
			default: ;
		endcase

		if ((take_in || take_out) && addr[31:29] != 0) ctrl[7] <= 1;   // beyond the 29-bit memory map
		else if (take_in) begin
			wbyte  <= push_byte;
			dma_we <= 1;
			dma_be <= addr[0] ? 2'b01 : 2'b10;
			req_r  <= 1;
		end else if (take_out) begin
			dma_we <= 0;
			dma_be <= 2'b11;
			req_r  <= 1;
		end

		if (req_r && dma_ack) begin
			req_r <= 0;
			gap   <= 2'd3;
			if (dma_bad) ctrl[7] <= 1;
			else begin
				if (dma_we) begin
					push_ack <= 1;
					residue[8*(3 - addr[1:0]) +: 8] <= wbyte;   // its lane: an unfinished longword ends up left aligned
				end else begin
					pull_byte <= addr[0] ? dma_rdata[7:0] : dma_rdata[15:8];
					pull_ack  <= 1;
				end
				addr  <= addr + 1'd1;
				count <= count - 1'd1;
				if (count == 1) ctrl[6] <= 1;
			end
		end
	end
end

endmodule
