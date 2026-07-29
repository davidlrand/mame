-- d000trace.lua - every value the firmware loads into the D000 local-DMA address latch.
--
-- D000 is GENERIC: it is the local address for whichever control block the gate array is about to
-- move (e800 bit13 picks node/$18 vs UIB/$20), so it holds the NODE at some moments and the UIB at
-- others.  If the UIB base ever passes through D000, the gate array knows where UIB+$12 lives and
-- can set bit7 - the op42 guard bypass at $6C32 that no firmware instruction ever sets.
--
-- NO SRAM READS INSIDE THE TAP: storager.cpp taps $4000-$7FFF for its DMA snoop, gated on the CPU
-- being the executing device, which is exactly true inside a CPU write tap.  Log raw, resolve later.
--
-- USAGE: SDL_VIDEODRIVER=dummy CPUAP_RTCFIX=1 ./mame pcmx2 -video none -nothrottle \
--          -autoboot_script docs/storager-lle/d000trace.lua -flop siemens/set1/mx2-001.imd

local SC   = ":slot1:storager:cpu"
local STOP = 30.0

local cpu, sp, armed = nil, nil, false
local taps, raw = {}, {}

local function now() local ok,t = pcall(function() return manager.machine.time:as_double() end); return ok and t or 0 end
local function W(a) return sp:read_u16(a & 0xffff) end
local function B(a) return sp:read_u8(a & 0xffff) end
local function PC() local ok,v = pcall(function() return cpu.state["PC"].value end); return (ok and v or 0) & 0xffffff end

local function arm()
  if armed then return true end
  local ok, d = pcall(function() return manager.machine.devices[SC] end)
  if not ok or not d then return false end
  armed = true; cpu = d; sp = d.spaces["program"]
  -- unconditional: no cmd gate, so the setup-time loads are captured too
  taps[#taps+1] = sp:install_write_tap(0xffd000, 0xffd001, "D000", function(off, data, mask)
    if #raw < 400 then raw[#raw+1] = { t = now(), v = data & 0xffff, pc = PC() } end
  end)
  print("d000trace armed"); io.flush()
  return true
end

emu.register_periodic(function()
  local t = now()
  if not arm() then return end
  if t >= STOP then
    local uib, node = W(0x799a), W(0x71bc)
    print(string.format("=== UIB[799a]=%04x (>>1=%04x)   node[71bc]=%04x (>>1=%04x) ===",
      uib, uib >> 1, node, node >> 1))
    print("=== every D000 load ===")
    local seen = {}
    for _, r in ipairs(raw) do
      local tgt = (r.v << 1) & 0xffff
      local tag = (tgt == uib) and "*** UIB  - GA can reach UIB+$12 bit7 ***"
               or (tgt == node) and "node"
               or (tgt >= 0x4000 and tgt <= 0x7fff) and "SRAM" or ""
      print(string.format("%9.4f  D000 <= %04x  (<<1 = %04x)  pc=%06x  %s", r.t, r.v, tgt, r.pc, tag))
    end
    print(string.format("=== total D000 loads: %d ; UIB+12 = %04x, currently %02x (bit7=%d)",
      #raw, (uib + 0x12) & 0xffff, B((uib + 0x12) & 0xffff), (B((uib + 0x12) & 0xffff) >> 7) & 1))
    io.flush(); manager.machine:exit()
  end
end)
