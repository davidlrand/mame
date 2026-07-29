-- chunkadv.lua - why does the chunk pointer [$741e] never advance on the MFM read?
--
-- Three sites write [$741e] and two are dead: $79EE (in $7964, the pump callback - the pump never
-- runs) and $7AFE (in $7ACC, reached from $7A9A - same routine).  The live one is $7F10, on the ISR
-- accept path after $7EE0 -> $7EEA bsr $32AC -> $7EF8 -> $7F10.  So "never advances" is exactly one
-- of two things:
--   (a) $7EE0 is not reached per record - the accept path short-circuits before the allocation
--   (b) $32AC returns the SAME index every time - the allocation runs and yields a constant
-- Expect (b): $32AC's reclaim arm ($3320-$336C) fires when the free list is empty, scans the ledger
-- for a positive entry, stamps it $C0 and reuses that chunk.  With sparse staking on the MFM path
-- reclaim hands back the same chunk indefinitely - which is "every field lands in $4700".
-- [$74ac] (the free-list head) at entry says which arm ran, with no inference.
--
-- Registers read at the tap; SRAM reads deferred to the periodic.
--
-- USAGE: SDL_VIDEODRIVER=dummy CPUAP_RTCFIX=1 ./mame pcmx2 -video none -nothrottle -oslog \
--   -autoboot_script docs/storager-lle/chunkadv.lua -flop siemens/set1/mx2-001.imd

local SC, STOP = ":slot1:storager:cpu", 12.0
local cpu, sp, osp, armed = nil, nil, nil, false
local taps, ev, pend = {}, {}, {}
local n7ee0, n32ac, n7f10 = 0, 0, 0

local function now() local ok,t = pcall(function() return manager.machine.time:as_double() end); return ok and t or 0 end
local function W(a) return sp:read_u16(a & 0xffff) end
local function R(n) local ok,v = pcall(function() return cpu.state[n].value end); return ok and v or 0 end

local function arm()
  if armed then return true end
  local ok, d = pcall(function() return manager.machine.devices[SC] end)
  if not ok or not d then return false end
  armed = true; cpu = d; sp = d.spaces["program"]
  osp = d.spaces["decrypted_opcodes"] or d.spaces["opcodes"] or sp
  taps[#taps+1] = osp:install_read_tap(0x7ee4, 0x7ee5, "ee0", function() n7ee0 = n7ee0 + 1 end)
  -- $7EEE tst.w (-$6,A3): the allocation verdict, and (-$8,A3) is the index it returned
  taps[#taps+1] = osp:install_read_tap(0x7eee, 0x7eef, "res", function()
    n32ac = n32ac + 1
    pend[#pend+1] = { t = now(), a3 = R("A3") & 0xffff, n = n32ac }
  end)
  taps[#taps+1] = osp:install_read_tap(0x7f10, 0x7f11, "adv", function() n7f10 = n7f10 + 1 end)
  print("chunkadv armed"); io.flush(); return true
end

emu.register_periodic(function()
  if not arm() then return end
  while #pend > 0 do
    local p = table.remove(pend, 1)
    if #ev < 40 then
      ev[#ev+1] = string.format("%9.4f  alloc#%-3d verdict(-6,A3)=%04x  index(-8,A3)=%04x  [74ac]=%04x  [741e]=%04x",
        p.t, p.n, W((p.a3 - 6) & 0xffff), W((p.a3 - 8) & 0xffff), W(0x74ac), W(0x741e))
    end
  end
  if now() >= STOP then
    print("=== chunk-pointer advance ===")
    for _, l in ipairs(ev) do print(l) end
    print(string.format("=== $7EE0 reached: %d | $32AC returns: %d | $7F10 advances: %d",
      n7ee0, n32ac, n7f10))
    print("  [74ac]=0000 at entry => the free list was empty => $32AC took the RECLAIM arm")
    io.flush(); manager.machine:exit()
  end
end)
