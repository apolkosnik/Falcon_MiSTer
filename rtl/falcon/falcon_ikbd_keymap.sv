// falcon_ikbd_keymap.sv - PS/2 scan code set 2 to Atari ST/Falcon IKBD scan
// code (positional mapping).
//
// Input:  ext (E0 prefix) and the PS/2 set 2 code.
// Output: ST scan code (make code, 7 bits) or 8'hFF when the key has no ST
//         equivalent, registered (one clock latency) so that the table is
//         implemented as a ROM in block memory.
//
// The ST codes come from Hatari src/sdl/keymap.c
// (Keymap_PcToStScanCode for the physical keys, Keymap_SymbolicToStScanCode_default
// for the US legends).  Special keys:
//   F11 -> Help ($62), F12 -> Undo ($61)        (Hatari)
//   End -> Help ($62)                             (task choice; Hatari uses
//                                                  $2B / $61 depending on mode)
//   PageUp -> keypad ( ($63), PageDown -> keypad ) ($64)   (Hatari)
//   NumLock -> keypad ) ($64), PrintScreen -> Help, ScrollLock -> Undo (Hatari)
//   Home -> Clr/Home ($47), Insert ($52), Delete ($53)
//   ` ~ -> $29, \ | -> $2B (Hatari symbolic table, US TOS legends)
//   ISO key (102nd key, PS/2 $61) -> $60
//   Right Ctrl / Right Alt -> Control ($1D) / Alternate ($38)
// The E0 12 / E0 59 "fake shift" codes of PrintScreen and the navigation
// keys are ignored ($FF).  Pause (E1 sequence) and the Windows keys have no
// ST equivalent.

module falcon_ikbd_keymap (
    input            clk,
    input            ext,
    input      [7:0] code,
    output reg [7:0] st
);

function [7:0] map(input [8:0] a);
    reg       xe;
    reg [7:0] xc;
    begin
    xe   = a[8];
    xc   = a[7:0];
    map  = 8'hFF;
    if (!xe) begin
        case (xc)
        8'h76: map = 8'h01;  // Esc
        8'h16: map = 8'h02;  // 1
        8'h1E: map = 8'h03;  // 2
        8'h26: map = 8'h04;  // 3
        8'h25: map = 8'h05;  // 4
        8'h2E: map = 8'h06;  // 5
        8'h36: map = 8'h07;  // 6
        8'h3D: map = 8'h08;  // 7
        8'h3E: map = 8'h09;  // 8
        8'h46: map = 8'h0A;  // 9
        8'h45: map = 8'h0B;  // 0
        8'h4E: map = 8'h0C;  // -
        8'h55: map = 8'h0D;  // =
        8'h66: map = 8'h0E;  // Backspace
        8'h0D: map = 8'h0F;  // Tab
        8'h15: map = 8'h10;  // Q
        8'h1D: map = 8'h11;  // W
        8'h24: map = 8'h12;  // E
        8'h2D: map = 8'h13;  // R
        8'h2C: map = 8'h14;  // T
        8'h35: map = 8'h15;  // Y
        8'h3C: map = 8'h16;  // U
        8'h43: map = 8'h17;  // I
        8'h44: map = 8'h18;  // O
        8'h4D: map = 8'h19;  // P
        8'h54: map = 8'h1A;  // [
        8'h5B: map = 8'h1B;  // ]
        8'h5A: map = 8'h1C;  // Return
        8'h14: map = 8'h1D;  // Left Ctrl
        8'h1C: map = 8'h1E;  // A
        8'h1B: map = 8'h1F;  // S
        8'h23: map = 8'h20;  // D
        8'h2B: map = 8'h21;  // F
        8'h34: map = 8'h22;  // G
        8'h33: map = 8'h23;  // H
        8'h3B: map = 8'h24;  // J
        8'h42: map = 8'h25;  // K
        8'h4B: map = 8'h26;  // L
        8'h4C: map = 8'h27;  // ;
        8'h52: map = 8'h28;  // '
        8'h0E: map = 8'h29;  // `
        8'h12: map = 8'h2A;  // Left Shift
        8'h5D: map = 8'h2B;  // \ (also ISO #)
        8'h1A: map = 8'h2C;  // Z
        8'h22: map = 8'h2D;  // X
        8'h21: map = 8'h2E;  // C
        8'h2A: map = 8'h2F;  // V
        8'h32: map = 8'h30;  // B
        8'h31: map = 8'h31;  // N
        8'h3A: map = 8'h32;  // M
        8'h41: map = 8'h33;  // ,
        8'h49: map = 8'h34;  // .
        8'h4A: map = 8'h35;  // /
        8'h59: map = 8'h36;  // Right Shift
        8'h11: map = 8'h38;  // Left Alt
        8'h29: map = 8'h39;  // Space
        8'h58: map = 8'h3A;  // Caps Lock
        8'h05: map = 8'h3B;  // F1
        8'h06: map = 8'h3C;  // F2
        8'h04: map = 8'h3D;  // F3
        8'h0C: map = 8'h3E;  // F4
        8'h03: map = 8'h3F;  // F5
        8'h0B: map = 8'h40;  // F6
        8'h83: map = 8'h41;  // F7
        8'h0A: map = 8'h42;  // F8
        8'h01: map = 8'h43;  // F9
        8'h09: map = 8'h44;  // F10
        8'h78: map = 8'h62;  // F11 -> Help
        8'h07: map = 8'h61;  // F12 -> Undo
        8'h7E: map = 8'h61;  // Scroll Lock -> Undo
        8'h77: map = 8'h64;  // Num Lock -> keypad )
        8'h7B: map = 8'h4A;  // keypad -
        8'h79: map = 8'h4E;  // keypad +
        8'h7C: map = 8'h66;  // keypad *
        8'h61: map = 8'h60;  // ISO < > key
        8'h6C: map = 8'h67;  // keypad 7
        8'h75: map = 8'h68;  // keypad 8
        8'h7D: map = 8'h69;  // keypad 9
        8'h6B: map = 8'h6A;  // keypad 4
        8'h73: map = 8'h6B;  // keypad 5
        8'h74: map = 8'h6C;  // keypad 6
        8'h69: map = 8'h6D;  // keypad 1
        8'h72: map = 8'h6E;  // keypad 2
        8'h7A: map = 8'h6F;  // keypad 3
        8'h70: map = 8'h70;  // keypad 0
        8'h71: map = 8'h71;  // keypad .
        default: map = 8'hFF;
        endcase
    end else begin
        case (xc)
        8'h5A: map = 8'h72;  // keypad Enter
        8'h4A: map = 8'h65;  // keypad /
        8'h14: map = 8'h1D;  // Right Ctrl
        8'h11: map = 8'h38;  // Right Alt
        8'h70: map = 8'h52;  // Insert
        8'h6C: map = 8'h47;  // Home -> Clr/Home
        8'h7D: map = 8'h63;  // Page Up -> keypad (
        8'h71: map = 8'h53;  // Delete
        8'h69: map = 8'h62;  // End -> Help
        8'h7A: map = 8'h64;  // Page Down -> keypad )
        8'h75: map = 8'h48;  // Up
        8'h72: map = 8'h50;  // Down
        8'h6B: map = 8'h4B;  // Left
        8'h74: map = 8'h4D;  // Right
        8'h7C: map = 8'h62;  // Print Screen -> Help
        default: map = 8'hFF;
        endcase
    end
    end
endfunction

// ROM (block memory), registered output
reg [7:0] rom [0:511];
integer i;
initial for (i = 0; i < 512; i = i + 1) rom[i] = map(i[8:0]);
always @(posedge clk) st <= rom[{ext, code}];

endmodule
