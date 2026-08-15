#!/bin/bash
# $1 = seconds to wait after the "Antworten Sie mit" prompt before sending j
# $2 = lua script basename (default dippc.lua)
#
# SCRATCH MEDIA ONLY.  Same rule as install-run.sh / dbg-run.sh: never mount the archival
# siemens/set1/mx2-001.imd.  Now that the firmware write path is the default, a
# direct archival mount would either refuse writes (mode 444 → WPT) or, if someone unlocked the
# master, corrupt the only clean install floppy.  Copy + chmod u+w every run.
cd /Users/dlr/src/mame
# ONE WRITER ONLY.  Three pcmx2 instances once shared one floppy, one HD image and one
# ./error.log - races that silently merged timelines and invalidated two runs' worth of results.
if pgrep -f "mame pcmx2" >/dev/null; then
  echo "REFUSING: a pcmx2 instance is already running:" >&2
  ps -eo pid,etime,command | grep "[m]ame pcmx2" >&2
  exit 1
fi
# Override per session:  STG_SCRATCH=/path/to/scratchpad delay.sh 4
S=${STG_SCRATCH:-/private/tmp/claude-501/-Users-dlr-src-mame/2fa5b147-7a98-445e-9e69-0ced5b77b43c/scratchpad}
D=$1
LUA=${2:-dippc.lua}
mkdir -p "$S"
cp siemens/set1/mx2-001.imd "$S/fl-delay.imd" || exit 1
chmod u+w "$S/fl-delay.imd" || exit 1
cp siemens/hd/blank-mc1325.img "$S/inst-test.img" || exit 1
rm -f /tmp/pty.out /tmp/console.txt
SDL_VIDEODRIVER=dummy ./mame pcmx2 -video none -nothrottle -log \
  -autoboot_script $S/$LUA -seconds_to_run 200 \
  -flop "$S/fl-delay.imd" -hard1 $S/inst-test.img \
  -slot3:serad:port0 pty > /tmp/pty.out 2>&1 &
MPID=$!
for i in $(seq 1 40); do grep -q "Pty slave is" /tmp/pty.out && break; sleep 1; done
PTY=$(grep "Pty slave is" /tmp/pty.out | tail -1 | sed "s/.*Pty slave is //" | tr -d " \r")
if [ -z "$PTY" ] || [ ! -e "$PTY" ]; then echo "delay=$D PTY-INVALID"; kill -9 $MPID; exit 1; fi
socat -u "$PTY,raw,echo=0,nonblock" - > /tmp/console.txt 2>/dev/null &
SPID=$!
for i in $(seq 1 120); do
  grep -q "Antworten Sie mit" /tmp/console.txt 2>/dev/null && break
  sleep 1
done
sleep $D
printf "j\r" > "$PTY"
sleep 45
kill -9 $SPID 2>/dev/null; kill -9 $MPID 2>/dev/null
if grep -q "Plattentyp" /tmp/console.txt 2>/dev/null; then echo "delay=${D}s ADVANCED"; else echo "delay=${D}s *** HUNG ***"; fi
