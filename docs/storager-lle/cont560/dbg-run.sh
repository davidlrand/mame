#!/bin/bash
# Debugger run for pcmx2 / storager: boots the SINIX installer, answers the "Antworten Sie mit"
# prompt with j, and runs under -debug with a caller-supplied debugscript.
#
# LIVES IN THE REPO ON PURPOSE.  The earlier copy sat in the session scratchpad and was swept away
# by a temp-dir clean mid-campaign; two "measurements" then silently produced no log at all, because
# bash failed on the missing file, the wait loop saw no process and exited, and the absent error.log
# read like a failed probe rather than a run that never happened.  A stale /tmp/pty2.out timestamp
# was the tell.  If a run yields no error.log, check FIRST whether MAME actually started.
#
#   usage:  dbg-run.sh [debugscript]     (default: $STG_SCRATCH/dbg.txt)
#   media:  scratch only - floppy is a COPY, HD is a fresh blank.  Never archival.

set -u
cd /Users/dlr/src/mame

# ONE WRITER ONLY.  Three pcmx2 instances once shared one floppy, one HD image and one ./error.log -
# mutual media contention plus a log three processes were writing, which invalidated two runs.
if pgrep -f "mame pcmx2" >/dev/null; then
  echo "REFUSING: a pcmx2 instance is already running:" >&2
  ps -eo pid,etime,command | grep "[m]ame pcmx2" >&2
  echo "kill it by PID first (only ever your own)." >&2
  exit 1
fi

S=${STG_SCRATCH:-/private/tmp/claude-501/-Users-dlr-src-mame/2fa5b147-7a98-445e-9e69-0ced5b77b43c/scratchpad}
DBG=${1:-$S/dbg.txt}
mkdir -p "$S"
[ -f "$DBG" ] || { echo "no debugscript at $DBG" >&2; exit 1; }

# scratch media: a COPY of the install floppy and a fresh blank HD
# Archival IMD is mode 444; MAME treats is_readonly() as write-protect.  Unlock the write copy.
[ -f /tmp/mx2-001.imd ] || cp siemens/set1/mx2-001.imd /tmp/mx2-001.imd
cp /tmp/mx2-001.imd /tmp/mx2-wr.imd || exit 1
chmod u+w /tmp/mx2-wr.imd || exit 1
cp siemens/hd/blank-mc1325.img "$S/inst-dbg.img" || exit 1
rm -f /tmp/pty2.out /tmp/console2.txt

./mame pcmx2 -window -nomaximize -debug -debugscript "$DBG" -log -nothrottle -seconds_to_run 200 \
  -flop /tmp/mx2-wr.imd -hard1 "$S/inst-dbg.img" -slot3:serad:port0 pty > /tmp/pty2.out 2>&1 &
MPID=$!

for i in $(seq 1 60); do grep -q "Pty slave is" /tmp/pty2.out 2>/dev/null && break; sleep 1; done
PTY=$(grep "Pty slave is" /tmp/pty2.out 2>/dev/null | tail -1 | sed "s/.*Pty slave is //" | tr -d " \r")
if [ -z "$PTY" ] || [ ! -e "$PTY" ]; then
  echo "PTY-INVALID - MAME did not start (see /tmp/pty2.out)" >&2
  kill -9 $MPID 2>/dev/null; exit 1
fi
echo "pty=$PTY  mame pid=$MPID"

socat -u "$PTY,raw,echo=0,nonblock" - > /tmp/console2.txt 2>/dev/null &
SPID=$!
for i in $(seq 1 180); do grep -q "Antworten Sie mit" /tmp/console2.txt 2>/dev/null && break; sleep 1; done
sleep 4
printf "j\r" > "$PTY"
echo "sent j"

# wait for MAME to finish on its own (-seconds_to_run), then clean up the reader
while kill -0 $MPID 2>/dev/null; do sleep 5; done
kill -9 $SPID 2>/dev/null
[ -s ./error.log ] && echo "error.log: $(wc -l < ./error.log) lines" || echo "WARNING: no error.log - did MAME run?"
