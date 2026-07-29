-- gate742c.lua - does [$742C] latch and freeze the C800 re-arm on the MFM path?
--
-- $92B8 tst.w $742c / $92BC bne $92F6   <- non-zero SKIPS the re-arm
-- $92BE move.w $7434,$d800
-- $92C4 move.w $741e,$c800              <- the ONLY copy of the firmware pointer into the GA arm
--
-- Symptom: on the cyl-1 MFM read [$741e] advances (2000 -> 2080 -> ... -> 2200) while C800[0] stays
-- at $2380 (chunk $4700).  Hypothesis: with [$7968]==0 and nothing rejected, [$742C] latches at 1
-- after the first record and every later $92B4 bails.  FM never showed it because the rotational
-- start produces rejects, and each reject clears [$742C] via $7CA8.
--
-- [$742C] full enumeration (uncapped):
--   setters  $7162 $7BF6 $7EA4 $7EB2 $95DA   ($7EB2 is the [$7968]==0 leg - the MFM one)
--   clearers $7CA8 $7E42 $7E90
--   testers  $925E $92AC $92B8               (it gates THREE sites, not just the re-arm)
--
-- Registers/SRAM read in the periodic, never inside the tap.
--
-- USAGE: SDL_VIDEODRIVER=dummy CPUAP_RTCFIX=1 ./mame pcmx2 -video none -nothrottle -oslog \
--   -autoboot_script docs/storager-lle/gate742c.lua -flop siemens/set1/mx2-001.imd

local SC, STOP = ":slot1:storager:cpu", 12.0
local cpu, sp, osp, armed = nil, nil, nil, false
local taps, ev, pend = {}, {}, {}
local ntest, nskip, narm = 0, 0, 0
local setn, clrn = {}, {}

local function now() local ok,t = pcall(function() return manager.machine.time:as_double() end); return ok and t or 0 end
local function W(a) return sp:read_u16(a & 0xffff) end
local function E(s) if #ev < 60 then ev[#ev+1] = s end end

local function arm()
  if armed then return true end
  local ok, d = pcall(function() return manager.machine.devices[SC] end)
  if not ok or not d then return false end
  armed = true; cpu = d; sp = d.spaces["program"]
  osp = d.spaces["decrypted_opcodes"] or d.spaces["opcodes"] or sp
  -- the gate itself: sample [$742C]/[$7968] in the periodic via a deferred record
  taps[#taps+1] = osp:install_read_tap(0x92b8, 0x92b9, "gate", function()
    ntest = ntest + 1
    pend[#pend+1] = { t = now(), n = ntest }
  end)
  -- did the re-arm actually happen?  $92C4 is past the branch, so reaching it means it was NOT skipped
  taps[#taps+1] = osp:install_read_tap(0x92c4, 0x92c5, "arm", function() narm = narm + 1 end)
  -- every writer, so the latch/clear traffic is visible
  for _, a in ipairs({ 0x7162, 0x7bf6, 0x7ea4, 0x7eb2, 0x95da }) do
    taps[#taps+1] = osp:install_read_tap(a, a + 1, "set", function()
      setn[a] = (setn[a] or 0) + 1
    end)
  end
  for _, a in ipairs({ 0x7ca8, 0x7e42, 0x7e90 }) do
    taps[#taps+1] = osp:install_read_tap(a, a + 1, "clr", function()
      clrn[a] = (clrn[a] or 0) + 1
    end)
  end
  print("gate742c armed"); io.flush(); return true
end

emu.register_periodic(function()
  if not arm() then return end
  while #pend > 0 do
    local p = table.remove(pend, 1)
    local g, s = W(0x742c), W(0x7968)
    if g ~= 0 then nskip = nskip + 1 end
    if #ev < 60 then
      E(string.format("%9.4f  test#%-3d [742C]=%04x [7968]=%04x  C800[0]=%04x [741e]=%04x  -> %s",
        p.t, p.n, g, s, W(0xc800), W(0x741e),
        (g ~= 0) and "SKIP the re-arm" or "re-arm"))
    end
  end
  if now() >= STOP then
    print("=== the $92B8 gate ===")
    for _, l in ipairs(ev) do print(l) end
    print(string.format("=== $92B8 tests: %d | skipped (742C non-zero): %d | $92C4 re-arms: %d",
      ntest, nskip, narm))
    local o = {}
    for a, c in pairs(setn) do o[#o+1] = string.format("$%04x=%d", a, c) end
    print("    setters fired:  " .. (table.concat(o, " ") ~= "" and table.concat(o, " ") or "none"))
    o = {}
    for a, c in pairs(clrn) do o[#o+1] = string.format("$%04x=%d", a, c) end
    print("    clearers fired: " .. (table.concat(o, " ") ~= "" and table.concat(o, " ") or "none"))
    io.flush(); manager.machine:exit()
  end
end)
