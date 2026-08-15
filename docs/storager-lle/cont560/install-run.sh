#!/bin/bash
# Headless SINIX install run for pcmx2 / storager.  Boots the install floppy, answers the
# "Antworten Sie mit" prompt with j, then reports how far the install got.
#
#   usage:  install-run.sh <label> [seconds_to_run] [extra keystrokes file]
#   media:  SCRATCH ONLY - floppy and HD are both fresh copies per run.  Never archival.
#
# NO REBUILD IS NEEDED FOR AN INSTALL.  Write routing is unconditional as of 3888995d028;
# there are no flags to flip for either an install or a read regression.
#
# The floppy is WRITABLE (host writes go through the firmware unconditionally), so reusing one
# image is not a clean repeat: the installer's own writes carry into the next run.  Copy every time.
set -u
cd /Users/dlr/src/mame

if pgrep -f "mame pcmx2" >/dev/null; then
  echo "REFUSING: a pcmx2 instance is already running:" >&2
  ps -eo pid,etime,command | grep "[m]ame pcmx2" >&2
  exit 1
fi

S=${STG_SCRATCH:-/private/tmp/claude-501/-Users-dlr-src-mame/2fa5b147-7a98-445e-9e69-0ced5b77b43c/scratchpad}
LABEL=${1:-run}
SECS=${2:-1400}
mkdir -p "$S"

# SOURCE FROM THE ARCHIVAL ORIGINAL, NOT A /tmp CACHE.  The floppy is
# genuinely writable, so any /tmp copy an earlier run touched is DIRTY - and a run booting a dirty
# install floppy stops at "warning: mounting unclean fs" waiting for an fsck answer, which reads
# exactly like a hang on boot.  Read the archival image, never write it.
#
# chmod u+w is required: the archival IMD is mode 444 (deliberately immutable), and MAME's
# floppy_image_device sets m_wpt from is_readonly() at load.  A 444 scratch copy is therefore
# WRITE PROTECTED - every WRCOMMIT fails with "WRSEC REFUSED: medium is WRITE PROTECTED",
# done stays 0, and the install stalls mid spare-track looking like a missing second IRQ5
# (measured inst3: 0 OK / 97 FAILED, WRDATA2 still firing).  cp preserves mode; unlock the copy.
cp siemens/set1/mx2-001.imd "$S/fl-$LABEL.imd" || exit 1
chmod u+w "$S/fl-$LABEL.imd" || exit 1
cp siemens/hd/blank-mc1325.img "$S/hd-$LABEL.img" || exit 1
rm -f "/tmp/pty-$LABEL.out" "/tmp/con-$LABEL.txt"

# Optional 3rd arg = debugscript.  The stalling spare-track write lands at t~95, well AFTER the
# Plattentyp dialog is answered (~t=78), so dbg-run.sh - which never answers that dialog - cannot
# reach it.  Run the debugger from this harness instead.  -debug needs a real window: dummy video
# + -video none + -debug segfaults on macOS, so the debug arm drops the headless flags.
DBG=${3:-}
if [ -n "$DBG" ]; then
  [ -f "$DBG" ] || { echo "no debugscript at $DBG" >&2; exit 1; }
  ./mame pcmx2 -window -nomaximize -debug -debugscript "$DBG" -nothrottle -log \
    -seconds_to_run "$SECS" -flop "$S/fl-$LABEL.imd" -hard1 "$S/hd-$LABEL.img" \
    -slot3:serad:port0 pty > "/tmp/pty-$LABEL.out" 2>&1 &
else
  SDL_VIDEODRIVER=dummy ./mame pcmx2 -video none -nothrottle -log -seconds_to_run "$SECS" \
    -flop "$S/fl-$LABEL.imd" -hard1 "$S/hd-$LABEL.img" \
    -slot3:serad:port0 pty > "/tmp/pty-$LABEL.out" 2>&1 &
fi
MPID=$!

for i in $(seq 1 60); do grep -q "Pty slave is" "/tmp/pty-$LABEL.out" 2>/dev/null && break; sleep 1; done
PTY=$(grep "Pty slave is" "/tmp/pty-$LABEL.out" 2>/dev/null | tail -1 | sed "s/.*Pty slave is //" | tr -d " \r")
if [ -z "$PTY" ] || [ ! -e "$PTY" ]; then
  echo "$LABEL: PTY-INVALID - MAME did not start"; kill -9 $MPID 2>/dev/null; exit 1
fi

socat -u "$PTY,raw,echo=0,nonblock" - > "/tmp/con-$LABEL.txt" 2>/dev/null &
SPID=$!
for i in $(seq 1 180); do grep -q "Antworten Sie mit" "/tmp/con-$LABEL.txt" 2>/dev/null && break; sleep 1; done
sleep 4
printf "j\r" > "$PTY"

# DISK-TYPE DIALOG.  "Es war dem System leider nicht moeglich, den Plattentyp automatisch zu
# erkennen" - SPACE cycles the NR field, ENTER hands the choice to the system.  NR 3 = MC1325,
# which is what -hard1 models, so three spaces then Enter.  (NR 4 is the free-entry slot and
# ends with END; we do not want it.)
for i in $(seq 1 900); do grep -q "Plattentyp" "/tmp/con-$LABEL.txt" 2>/dev/null && break; sleep 2; done
if grep -q "Plattentyp" "/tmp/con-$LABEL.txt" 2>/dev/null; then
  sleep 5
  printf " " > "$PTY"; sleep 2
  printf " " > "$PTY"; sleep 2
  printf " " > "$PTY"; sleep 2
  printf "\r" > "$PTY"
  echo "$LABEL: answered Plattentyp (NR 3 = MC1325)"
fi

while kill -0 $MPID 2>/dev/null; do sleep 5; done
kill -9 $SPID 2>/dev/null

# milestones, in the order the install reaches them
C="/tmp/con-$LABEL.txt"
echo "--- $LABEL ---"
for m in "Antworten Sie mit" "Boot: sa(22,0)sinix" "SINIX-M-C V2.0" "ipl 5" \
         "Plattentyp" "Spuren formatiert" "Ladeprogramm eingerichtet" "Minimalsystem"; do
  if grep -qF "$m" "$C" 2>/dev/null; then echo "  HIT   $m"; else echo "  ----  $m"; fi
done
echo "  cmd96=$(grep -c 'HOST GO: cmd=96' error.log 2>/dev/null) sense82=$(grep -c 'node+2=82' error.log 2>/dev/null)"
