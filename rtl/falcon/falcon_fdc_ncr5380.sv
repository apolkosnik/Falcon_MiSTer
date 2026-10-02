// falcon_fdc_ncr5380.sv - NCR 5380 register model with no SCSI target
// attached, as seen through the Falcon DMA chip ($FF8604 with bit 3 of the
// DMA mode register set, register = mode bits 2..0).
//
// The Falcon has no ACSI port; HDC accesses of the DMA chip go to the 5380.
// Until a real SCSI module exists, this stub behaves like Hatari's
// ncr5380.c (ncr5380_bget / ncr5380_bput / raw_scsi_set_signal_phase) with
// no device enabled: arbitration and selection never find a target, BSY
// never comes back, so TOS / EmuTOS selection times out exactly as in Hatari.
//
// Deviation from Hatari: on reset all registers are cleared and no interrupt
// is raised (NCR 5380 datasheet); Hatari's ncr5380_reset() sets ICR = $80
// (assert RST) and raises the interrupt.
//
// irq_set: one clock pulse when the 5380 interrupt goes active (Hatari
//          ncr5380_set_irq -> FDC_SetIRQ(FDC_IRQ_SOURCE_HDC)).
// irq_clr: one clock pulse when register 7 is read (Hatari: FDC_ClearIRQ).

module falcon_fdc_ncr5380 (
	input            clk,
	input            reset,
	input            acc,        // one clock: register access
	input            we,
	input      [2:0] rs,
	input      [7:0] wdata,
	output reg [7:0] rdata,      // value for a read access (combinational)
	output reg       irq_set,
	output reg       irq_clr
);

localparam [1:0] PH_FREE = 2'd0, PH_ARBIT = 2'd1, PH_SEL1 = 2'd2;

reg [7:0] regs [0:7];
reg [1:0] phase;
reg       irq;
reg [7:0] data_write;

function integer popcnt(input [7:0] v);
	integer k;
	begin
		popcnt = 0;
		for (k = 0; k < 8; k = k + 1) popcnt = popcnt + v[k];
	end
endfunction

// read value (ncr5380_bget with no target: io = 0, bus_phase < 0)
always @* begin
	case (rs)
		3'd0: rdata = (phase == PH_ARBIT) ? data_write : 8'h00;
		3'd4: rdata = regs[1] & 8'h80;
		3'd5: rdata = {3'b000, irq, 1'b0, ((regs[2][2]) && phase == PH_FREE) ? 1'b1 : 1'b0, 2'b00} |
		              (regs[5] & 8'hA0);
		3'd6: rdata = 8'h00;
		default: rdata = regs[rs];
	endcase
end

// signal phase update (raw_scsi_set_signal_phase), no device present
function [1:0] set_phase(input [1:0] ph, input busy, input sel, input bus_out, input [7:0] dw);
	reg [1:0] p;
	begin
		p = ph;
		case (ph)
			PH_FREE: begin
				if (busy && !sel && !bus_out) begin
					if (popcnt(dw) == 1) p = PH_ARBIT;
				end else if (!busy && sel) begin
					if (!(popcnt(dw) > 2 || dw == 8'h00)) begin
						// SELECT_1, then evaluated again: no target answers
						p = PH_SEL1;
						if (!busy && !sel) p = PH_FREE;
					end
				end
			end
			PH_ARBIT: if (busy && sel) p = PH_SEL1;
			PH_SEL1: if (!busy && !sel) p = PH_FREE;
			default: p = PH_FREE;
		endcase
		set_phase = p;
	end
endfunction

integer i;
reg [7:0] old;
reg       new_irq;
reg [1:0] ph_n;

always @(posedge clk) begin
	irq_set <= 1'b0;
	irq_clr <= 1'b0;
	if (reset) begin
		for (i = 0; i < 8; i = i + 1) regs[i] <= 8'h00;
		phase <= PH_FREE;
		irq <= 1'b0;
		data_write <= 8'h00;
	end else if (acc) begin
		new_irq = 1'b0;
		ph_n = phase;
		if (!we) begin
			if (rs == 3'd6) begin
				// ncr5380_check_phase: DMA mode, phase mismatch -> interrupt
				if (regs[2][1] && !regs[2][6]) new_irq = 1'b1;
			end
			if (rs == 3'd7) begin
				irq <= 1'b0;
				irq_clr <= 1'b1;
			end
		end else begin
			old = regs[rs];
			case (rs)
				3'd0: begin
					regs[0] <= wdata;
					data_write <= wdata;
				end
				3'd1: begin
					regs[1] <= (wdata & 8'h9F) | (old & 8'h60);
					if (!wdata[7]) begin
						// raw_scsi_set_signal_phase(busy=bit3, select=bit2, atn=bit1)
						ph_n = set_phase(phase, wdata[3], wdata[2], wdata[0], data_write);
						if (phase == PH_FREE && !regs[1][0] && wdata[0] && regs[2][0])
							ph_n = set_phase(PH_SEL1, wdata[3], wdata[2], wdata[0], data_write);
					end else begin
						// RST: chip reset (datasheet: registers cleared)
						for (i = 0; i < 8; i = i + 1) regs[i] <= 8'h00;
						ph_n = PH_FREE;
					end
				end
				3'd2: begin
					regs[2] <= wdata;
					if (wdata[0] && !old[0]) begin
						// arbitrate: drive BSY, AIP set, LA cleared
						ph_n = set_phase(phase, 1'b1, 1'b0, 1'b0, data_write);
						regs[1] <= (regs[1] | 8'h40) & 8'hDF;
					end else if (!wdata[0] && old[0]) begin
						regs[1] <= regs[1] & 8'hBF;
					end
					if (!wdata[1]) regs[5] <= regs[5] & 8'h3F;
					else if (!old[1] && !wdata[6]) new_irq = 1'b1;   // check_phase on DMA enable
				end
				3'd5: ;   // start DMA send: register keeps its old value
				default: regs[rs] <= wdata;
			endcase
		end
		phase <= ph_n;
		if (new_irq && !irq) begin
			irq <= 1'b1;
			irq_set <= 1'b1;
		end
	end
end

endmodule
