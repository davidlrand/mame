-- binstall.lua - the REAL builder install, on the right descriptor.
--
-- $93FC movea.w $71c2,a0 / $9400 move.w ($20,a6),d0 / $9404 btst #$e,d0
--   bit14 SET   -> $940A move.w #$6102,($6,a0)
--   bit14 CLEAR -> $9412 move.w #$5fc0,($6,a0)     <- the read builder
-- The descriptor is [$71c2].  An earlier sweep used [$721a]+$6, a DIFFERENT cell, found it never
-- written and reading zero, and refuted the install hypothesis on that basis - wrong cell.
--
-- Labels come from the model's HOST GO logerror (same clock, no sampling), NOT from periodically
-- sampling the node command byte: two probes previously shared that method, agreed, and were both
-- wrong.  This one only timestamps; correlate against HOST GO afterwards.
--
-- USAGE: SDL_VIDEODRIVER=dummy CPUAP_RTCFIX=1 ./mame pcmx2 -video none -nothrottle -oslog \
--   -autoboot_script docs/storager-lle/binstall.lua -flop siemens/set1/mx2-001.imd -hard1 <img>

local SC, STOP = ":slot1:storager:cpu", 45.0
local cpu, sp, armed = nil, nil, false
local taps, ev = {}, {}
local dsc = 0

local function now() local ok,t = pcall(function() return manager.machine.time:as_double() end); return ok and t or 0 end
local function W(a) return sp:read_u16(a & 0xffff) end
local function E(s) if #ev < 200 then ev[#ev+1] = string.format("%9.4f  %s", now(), s) end end

local NAME = { [0x5fc0]="$5FC0 READ builder (24 28 56 58 .. 42 36 00)",
               [0x6102]="$6102 (bit14-set alternative)",
               [0x5e64]="$5E64 short builder (1c 22 16 00)",
               [0xa63e]="$A63E cmd 0x98 builder" }

local function arm()
  if armed then return true end
  local ok, d = pcall(function() return manager.machine.devices[SC] end)
  if not ok or not d then return false end
  armed = true; cpu = d; sp = d.spaces["program"]
  -- wide window over the plausible descriptor region; filter to [$71c2]+6 with a cached base
  taps[#taps+1] = sp:install_write_tap(0x7100, 0x72ff, "slot", function(off, data, mask)
    local a, v = off & 0xffff, data & 0xffff
    if a == ((dsc + 6) & 0xffff) then
      E(string.format("INSTALL [%04x+6] <= %04x   %s", dsc, v, NAME[v] or "(other)"))
    elseif NAME[v] then
      E(string.format("builder addr %04x written to [%04x]  %s", v, a, NAME[v]))
    end
  end)
  print("binstall armed"); io.flush(); return true
end

emu.register_periodic(function()
  if not arm() then return end
  local d = W(0x71c2)
  if d >= 0x4000 and d < 0x8000 then dsc = d end
  if now() >= STOP then
    print("=== builder installs on [$71c2]+6 ===")
    print(string.format("  [$71c2] = %04x   slot+6 now = %04x", dsc, W((dsc + 6) & 0xffff)))
    for _, l in ipairs(ev) do print(l) end
    print(string.format("=== %d event(s) ===", #ev))
    io.flush(); manager.machine:exit()
  end
end)
