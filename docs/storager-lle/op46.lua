-- op46.lua - is $2342 the CONTINUATION dispatch, and does op $46 ever run here?
--
-- Reconstruction under test: op $46 ($192+$46 -> $8AC8) returns $FFFF via $8F62 move.w #$ffff,D0 /
-- $8F72 rts.  The walker's $15C0 move.w D0,($4,A0) would store $FFFF at $7228, whose low byte
-- satisfies $231A cmpi.b #$ff and lets $2342 dispatch a builder.  That joins up with $9412's install,
-- which is reachable only from counter-mismatch branches - so install and dispatch would be two
-- halves of ONE continuation mechanism, not dead code.
-- It also predicts every negative measured this session: a single-chunk request clamps the remainder
-- at $738C, never enters continuation, so op $46 never runs, $231A never passes, $2342 never fires,
-- and ($6,A1) is neither written nor read.
--
-- PREDICTION: op $46 dispatches ZERO times in this workload, and $8F6E/$8F72 likewise.
-- Both taps are DEEP inside their routines, not at entry words (entry taps have been prefetch
-- artifacts repeatedly), and the $8F62->op-$46 link is an inference from routine boundaries - which
-- is what this measures rather than assumes.
--
-- USAGE: SDL_VIDEODRIVER=dummy CPUAP_RTCFIX=1 ./mame pcmx2 -video none -nothrottle -oslog \
--   -autoboot_script docs/storager-lle/op46.lua -flop siemens/set1/mx2-001.imd -hard1 <img>

local SC, STOP = ":slot1:storager:cpu", 45.0
local cpu, sp, osp, armed = nil, nil, nil, false
local taps, ev = {}, {}
local n46, nret, nff = 0, 0, 0

local function now() local ok,t = pcall(function() return manager.machine.time:as_double() end); return ok and t or 0 end
local function W(a) return sp:read_u16(a & 0xffff) end
local function R(n) local ok,v = pcall(function() return cpu.state[n].value end); return ok and v or 0 end
local function E(s) if #ev < 60 then ev[#ev+1] = string.format("%9.4f  %s", now(), s) end end

local function arm()
  if armed then return true end
  local ok, d = pcall(function() return manager.machine.devices[SC] end)
  if not ok or not d then return false end
  armed = true; cpu = d; sp = d.spaces["program"]
  osp = d.spaces["decrypted_opcodes"] or d.spaces["opcodes"] or sp
  -- op $46's body, one instruction past its entry
  taps[#taps+1] = osp:install_read_tap(0x8acc, 0x8acd, "op46", function()
    n46 = n46 + 1
    if n46 <= 6 then E(string.format("op $46 body reached (#%d)", n46)) end
  end)
  -- the $FFFF return path, deep (the SR restore, not the rts)
  taps[#taps+1] = osp:install_read_tap(0x8f6e, 0x8f6f, "ret", function()
    nret = nret + 1
    if nret <= 6 then E(string.format("$8F6E return path, D0=%08x", R("D0"))) end
  end)
  -- and any $FF landing in the gate cell, whoever wrote it
  taps[#taps+1] = sp:install_write_tap(0x7228, 0x7229, "gate", function(off, data, mask)
    local v = data & 0xffff
    if (v & 0xff) == 0xff then
      nff = nff + 1
      if nff <= 6 then E(string.format("$7228 <= %04x  *** low byte $FF - the gate would PASS ***", v)) end
    end
  end)
  print("op46 armed"); io.flush(); return true
end

emu.register_periodic(function()
  if not arm() then return end
  if now() >= STOP then
    print("=== op $46 / continuation dispatch ===")
    for _, l in ipairs(ev) do print(l) end
    print(string.format("=== op $46 body: %d   $8F6E return: %d   $7228 writes with low byte $FF: %d",
      n46, nret, nff))
    print("  prediction: all three ZERO in this single-chunk workload")
    io.flush(); manager.machine:exit()
  end
end)
