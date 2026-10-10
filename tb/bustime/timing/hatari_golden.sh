#!/bin/bash
# The golden for the timing sequences: runs a TOS program in Hatari (its
# Falcon default: 68030 at 16 MHz, cycle exact, data cache, 68882, 14 MB,
# TOS 4.04, no DSP) headless and prints "<marker> <cycle counter>" for every
# word the program writes to $3F0 ($FFFF ends the run and is not printed).
# The counter is Hatari's CycleCounter (CyclesGlobalClockCounter, CPU
# clocks), read at the instruction boundary after the marker write.
#
#   HATARI=<hatari binary> TOS=<tos404.img> hatari_golden.sh <program.PRG>
#   MAXVBL (default 4000) bounds the run; TIMEOUT (seconds, default 600)
#
# Build Hatari out of tree (cmake -S <hatari source> -B <dir> -G Ninja;
# ninja -C <dir>), with SDL2.  Compare with the core: table.py.
set -u
PRG="$(readlink -f "${1:?usage: hatari_golden.sh program.PRG}")"
: "${HATARI:?set HATARI to the hatari binary}" "${TOS:?set TOS to tos404.img}"
MAXVBL="${MAXVBL:-4000}"
W="$(mktemp -d)"
trap 'rm -rf "$W"' EXIT
mkdir -p "$W/hd/AUTO" "$W/home"
cp "$PRG" "$W/hd/AUTO/TEST.PRG"
echo "echo @@ '(\$3f0).w' 'CycleCounter'" > "$W/marker.ini"
echo "q" > "$W/quit.ini"
cat > "$W/bp.ini" <<EOT
b (\$3f0).w ! (\$3f0).w && (\$3f0).w < \$ffff :trace :quiet :noinit :file $W/marker.ini
b (\$3f0).w = \$ffff :quiet :noinit :file $W/quit.ini
EOT
export HOME="$W/home" XDG_CONFIG_HOME="$W/home/.config" XDG_DATA_HOME="$W/home/.local/share"
export SDL_VIDEODRIVER=dummy SDL_AUDIODRIVER=dummy
timeout "${TIMEOUT:-600}" "$HATARI" \
  --machine falcon --cpulevel 3 --cpuclock 16 --cpu-exact true --compatible true \
  --data-cache true --mmu false --fpu 68882 --memsize 14 --ttram 0 \
  --dsp none --monitor vga --sound off --tos "$TOS" --patch-tos true \
  --fast-boot false --fast-forward true --fastfdc false --statusbar false \
  --natfeats false --confirm-quit false --harddrive "$W/hd" --protect-hd off \
  --parse "$W/bp.ini" --run-vbls "$MAXVBL" --window \
  </dev/null 2>&1 >/dev/null |
  sed -n "s/^@@ \\\$\\([0-9a-fA-F]*\\) \\\$\\([0-9a-fA-F]*\\).*/\\1 \\2/p" |
  while read -r m c; do echo "$((16#$m)) $((16#$c))"; done
