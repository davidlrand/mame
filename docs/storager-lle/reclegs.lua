-- reclegs.lua - per record: which leg of the $7E8E fork, the walk's D0, the three compares, and the
-- $7ED0/$7E9A marker cell.  Answers whether [$7968] ever flips mid-read and what the walk lands on.
--
-- Prefetch note: a write reported at pc=$7EB8 is the instruction at $7EB2, and pc=$7E96 is $7E90.
-- Tap the OPCODE fetches, not the writes, so the leg is unambiguous.
--
-- USAGE: SDL_VIDEODRIVER=dummy CPUAP_RTCFIX=1 ./mame pcmx2 -video none -nothrottle \
--          -autoboot_script docs/storager-lle/reclegs.lua -flop siemens/set1/mx2-001.imd

local SC, STOP = ":slot1:storager:cpu", 30.0
local cpu, sp, osp, armed = nil, nil, nil, false
local taps, ev = {}, {}
local nrec = 0

local function now() local ok,t = pcall(function() return manager.machine.time:as_double() end); return ok and t or 0 end
local function W(a) return sp:read_u16(a & 0xffff) end
local function B(a) return sp:read_u8(a & 0xffff) end
local function R(n) local ok,v = pcall(function() return cpu.state[n].value end); return ok and v or 0 end
local function E(s) if #ev < 400 then ev[#ev+1] = string.format("%9.4f  %s", now(), s) end end

local function arm()
  if armed then return true end
  local ok, d = pcall(function() return manager.machine.devices[SC] end)
  if not ok or not d then return false end
  armed = true; cpu = d; sp = d.spaces["program"]
  osp = d.spaces["decrypted_opcodes"] or d.spaces["opcodes"] or sp
  -- entry to the walk
  taps[#taps+1] = osp:install_read_tap(0x7e6c, 0x7e6d, "walk", function()
    nrec = nrec + 1
    E(string.format("rec#%-2d walk ENTER  D0=%04x A0=%04x [7968]=%04x [7956]=%04x [7430]=%04x",
      nrec, R("D0") & 0xffff, R("A0") & 0xffff, W(0x7968), W(0x7956), W(0x7430)))
  end)
  -- the fork
  taps[#taps+1] = osp:install_read_tap(0x7e8a, 0x7e8b, "fork", function()
    E(string.format("       fork      D0=%04x [7968]=%04x -> %s", R("D0") & 0xffff, W(0x7968),
      (W(0x7968) == 0) and "$7EB2 (accept, decrements [7956])" or "$7E90 (marker leg)"))
  end)
  taps[#taps+1] = osp:install_read_tap(0x7e90, 0x7e91, "leg90", function() E("       LEG $7E90  ([7968] != 0)") end)
  taps[#taps+1] = osp:install_read_tap(0x7eb2, 0x7eb3, "legb2", function() E("       LEG $7EB2  ([7968] == 0)") end)
  taps[#taps+1] = osp:install_read_tap(0x7ea4, 0x7ea5, "lega4", function() E("       LEG $7EA4  ([79b6] != 0 early-out)") end)
  -- the two marker tests
  taps[#taps+1] = osp:install_read_tap(0x7e9a, 0x7e9b, "m9a", function()
    local a0, d0 = R("A0") & 0xffff, R("D0") & 0xffff
    E(string.format("       $7E9A cell[%d] @%04x = %02x  -> %s", d0, (a0+d0) & 0xffff, B(a0+d0),
      (B(a0+d0) == 0xaa) and "$AA -> $7ED8 clear [741c]" or "not $AA -> $7EE0 allocate"))
  end)
  taps[#taps+1] = osp:install_read_tap(0x7ed0, 0x7ed1, "md0", function()
    local a0, d0 = R("A0") & 0xffff, R("D0") & 0xffff
    E(string.format("       $7ED0 cell[%d] @%04x = %02x  -> %s", d0, (a0+d0) & 0xffff, B(a0+d0),
      (B(a0+d0) == 0xaa) and "$AA -> $7ED8 clear [741c]" or "not $AA -> $7EE0 ALLOCATE"))
  end)
  taps[#taps+1] = osp:install_read_tap(0x7ed8, 0x7ed9, "ed8", function() E("       => $7ED8  [741c]=0  (no allocate)") end)
  taps[#taps+1] = osp:install_read_tap(0x7ee0, 0x7ee1, "ee0", function() E("       => $7EE0  allocate via $32AC") end)
  taps[#taps+1] = osp:install_read_tap(0x7f14, 0x7f15, "f14", function() E("       => $7F14  [741c]=1  *** allocation OK ***") end)
  taps[#taps+1] = osp:install_read_tap(0x82b2, 0x82b3, "82b2", function() E("       => $82B2  [7968]=1  *** predicate SET ***") end)
  print("reclegs armed"); io.flush(); return true
end

emu.register_periodic(function()
  if not arm() then return end
  if now() >= STOP then
    print("=== per-record legs ===")
    for _, l in ipairs(ev) do print(l) end
    print(string.format("=== ledger $7654: %s",
      (function() local t={} for i=0,17 do t[#t+1]=string.format("%02x",B(0x7654+i)) end return table.concat(t," ") end)()))
    io.flush(); manager.machine:exit()
  end
end)
