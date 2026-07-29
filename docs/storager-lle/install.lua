-- install.lua - does the per-command builder install fire for a 0x95?
--
-- The dispatched builder does NOT come from the $92 table: at $0DE4 `move.w (A1)+,D7` / $0DE6
-- `beq $d60` the table's handler word is only a zero/validity test and D7 is unused after it.  The
-- builder that runs comes from the CHANNEL DESCRIPTOR:
--     $2316 movea.w $721a,a1 / $233A movea.w ($6,a1),a1 / $2342 jsr (a1)
-- so if [$721a]+$6 is not re-installed per command, a read runs whatever builder the PREVIOUS
-- command left there.  0x87 immediately precedes every read in the preamble, and 0x87's builder is
-- $5E64 - which is exactly what we measured running, emitting 1c 22 16 00.
--
-- Candidate writers of that slot: $0AA6, $27EE, $2AA4, $34BC.
-- If nothing fires between the 0x87 and the 0x95, that is the whole defect.
--
-- ONE tap owns the descriptor window and dispatches internally (overlapping taps do not both fire).
--
-- USAGE: SDL_VIDEODRIVER=dummy CPUAP_RTCFIX=1 ./mame pcmx2 -video none -nothrottle -oslog \
--   -autoboot_script docs/storager-lle/install.lua -flop siemens/set1/mx2-001.imd -hard1 <img>

local SC, STOP = ":slot1:storager:cpu", 45.0
local cpu, sp, armed = nil, nil, false
local taps, ev = {}, {}
local desc, lastcmd, nodebase = 0, -1, 0x71f0

local function now() local ok,t = pcall(function() return manager.machine.time:as_double() end); return ok and t or 0 end
local function B(a) return sp:read_u8(a & 0xffff) end
local function W(a) return sp:read_u16(a & 0xffff) end
local function E(s) if #ev < 220 then ev[#ev+1] = string.format("%9.4f  %s", now(), s) end end

local NAME = { [0x5e64] = "$5E64  short builder (1c 22 16 00) - cmd 0x87's",
               [0x5fc0] = "$5FC0  the READ builder (24 28 56 58 .. 42 36 00)",
               [0xa63e] = "$A63E  cmd 0x98's builder" }

local function arm()
  if armed then return true end
  local ok, d = pcall(function() return manager.machine.devices[SC] end)
  if not ok or not d then return false end
  armed = true; cpu = d; sp = d.spaces["program"]
  -- the descriptor lives around $7224; tap a window and filter to [$721a]+$6 using the cached base
  taps[#taps+1] = sp:install_write_tap(0x7200, 0x724f, "desc", function(off, data, mask)
    -- No dependence on a periodically-sampled [$721a]: log ANY write in the descriptor window whose
    -- value is builder-shaped (ROM text region), plus every write to the $7224 descriptor itself.
    local a, v = off & 0xffff, data & 0xffff
    if NAME[v] then
      E(string.format("BUILDER ADDR written: [%04x] <= %04x   %s", a, v, NAME[v]))
    elseif a >= 0x7224 and a <= 0x722e then
      E(string.format("desc [%04x] <= %04x", a, v))
    end
  end)
  print("install armed"); io.flush(); return true
end

emu.register_periodic(function()
  if not arm() then return end
  local n = W(0x71bc); if n >= 0x4000 and n < 0x8000 then nodebase = n end
  desc = W(0x721a)
  local c = B(nodebase)
  if c ~= lastcmd then
    if c ~= 0 then
      E(string.format("--- cmd %02x begins   [721a]=%04x  slot+6 currently = %04x  %s",
        c, desc, W((desc + 6) & 0xffff), NAME[W((desc + 6) & 0xffff)] or "(other)"))
    end
    lastcmd = c
  end
  if now() >= STOP then
    print("=== per-command builder install ===")
    for _, l in ipairs(ev) do print(l) end
    io.flush(); manager.machine:exit()
  end
end)
