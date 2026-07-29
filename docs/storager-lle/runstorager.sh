#!/bin/bash
# Storager LLE frontier run wrapper — cont.255l era: THE ENV-GATE FREEZE.
# Behavioral configuration is baked into storager.cpp (storager_getenv); no
# STORAGER_* env var changes modeled hardware. Only logging taps remain here.
# usage: runstorager.sh <runtag>   -> $S/run<tag>.out + run<tag>-error.log
set -u
S=/private/tmp/claude-501/-Users-dlr-src-mame/27438bdc-75b4-4ab3-8eb7-8d54c2cb9efd/scratchpad
TAG=${1:?runtag}
cd /Users/dlr/src/mame
export SDL_VIDEODRIVER=dummy
export STORAGER_PHASELOG=1 STORAGER_IAMRD=1   # logging only; STRIP pass removes these too
for try in 1 2 3; do
  rm -f error.log
  ./mame pcmx2 -video none -nothrottle -log -autoboot_script $S/fboot120.lua \
    -flop $S/mx2-001.imd > $S/run$TAG.out 2>&1
  if grep -q "DCENSUS TBLLOOK.*nodecmd=95" error.log && grep -q 'testend' $S/run$TAG.out; then
    echo "try$try: 95-DISPATCHED"
    break
  fi
  echo "try$try: retry"
done
cp error.log $S/run$TAG-error.log
echo "=== screen ==="
grep "@" $S/run$TAG.out | head -15
echo "=== 95 asks ==="
grep "IOPBDUMP cmd=95" $S/run$TAG-error.log | head -8
echo "=== JIT engagement ==="
grep -c "JITWARP" $S/run$TAG-error.log
grep "JITWARP" $S/run$TAG-error.log | head -6
echo "=== ledger endgame ==="
grep "DESCARM-LIVE" $S/run$TAG-error.log | tail -3
