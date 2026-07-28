-- stakes.lua - independent count of the ledger stakes, and the $A1CE parity exit.
--
-- (a) The model's accepted= figure comes from a write tap.  Count the STAKE INSTRUCTIONS instead
--     ($8120 move.b #$c0 / $8128 move.b d0) and compare - if they disagree, the over-accept is a
--     counting artifact, not firmware behaviour.
-- (b) $A1CE's three failure exits: both timeouts are an exhausted dbra over #$ffff = 65536 x ~16
--     cycles ~ 105ms at 10MHz, but the transaction measures ~1ms - so neither can run.  That leaves
--     PARITY.  Count $A20E (the data-bit accumulate): 16 per invocation => D5=16 => even => carry
--     clear at $A23C => $A240.  Both sites are inside the loop body, so no branch-shadow care needed.
--
-- USAGE: SDL_VIDEODRIVER=dummy CPUAP_RTCFIX=1 ./mame pcmx2 -video none -nothrottle -oslog \
--   -autoboot_script docs/storager-lle/stakes.lua -flop siemens/set1/mx2-001.imd -hard1 <img>

local SC, STOP = ":slot1:storager:cpu", 30.0
local cpu, sp, osp, armed = nil, nil, nil, false
local taps, ev = {}, {}
local n8120, n8128, nA20E, nser = 0, 0, 0, 0

local function now() local ok,t = pcall(function() return manager.machine.time:as_double() end); return ok and t or 0 end
local function R(n) local ok,v = pcall(function() return cpu.state[n].value end); return ok and v or 0 end
local function E(s) if #ev < 200 then ev[#ev+1] = string.format("%9.4f  %s", now(), s) end end

local function arm()
  if armed then return true end
  local ok, d = pcall(function() return manager.machine.devices[SC] end)
  if not ok or not d then return false end
  armed = true; cpu = d; sp = d.spaces["program"]
  osp = d.spaces["decrypted_opcodes"] or d.spaces["opcodes"] or sp
  taps[#taps+1] = osp:install_read_tap(0x8120, 0x8121, "s1", function() n8120 = n8120 + 1 end)
  taps[#taps+1] = osp:install_read_tap(0x8128, 0x8129, "s2", function() n8128 = n8128 + 1 end)
  -- $A1CE invocation boundaries, so the per-transaction bit count is attributable
  taps[#taps+1] = osp:install_read_tap(0xa1ce, 0xa1cf, "ser", function()
    nser = nser + 1
    E(string.format("$A1CE #%d ENTER  (stakes so far: $8120=%d $8128=%d)", nser, n8120, n8128))
    nA20E = 0
  end)
  taps[#taps+1] = osp:install_read_tap(0xa20e, 0xa20f, "acc", function() nA20E = nA20E + 1 end)
  -- deep taps: if $A1CE really runs, ALL of these must fire
  for _, a in ipairs({0xa1d2, 0xa1d8, 0xa1e4, 0xa1ee, 0xa1f2, 0xa202, 0xa214, 0xa224}) do
    taps[#taps+1] = osp:install_read_tap(a, a + 1, string.format("d%04x", a), function()
      E(string.format("   deep $%04X reached", a))
    end)
  end
  -- and the caller: who branches to $A1CE?
  taps[#taps+1] = osp:install_read_tap(0xa676, 0xa677, "call", function()
    E("   $A676 (the #$3000 branch that calls $A1CE) reached")
  end)
  taps[#taps+1] = osp:install_read_tap(0xa666, 0xa667, "cmp", function()
    E(string.format("   $A666 cmpi.w #$3000  D0=%08x", R("D0")))
  end)
  taps[#taps+1] = osp:install_read_tap(0xa23c, 0xa23d, "par", function()
    E(string.format("   $A23C parity check: $A20E fired %d times, D5=%04x -> %s",
      nA20E, R("D5") & 0xffff,
      ((R("D5") & 1) == 0) and "EVEN -> carry clear -> $A240 PARITY FAIL" or "ODD -> ok"))
  end)
  taps[#taps+1] = osp:install_read_tap(0xa240, 0xa241, "fail", function() E("   -> $A240 PARITY FAIL confirmed") end)
  taps[#taps+1] = osp:install_read_tap(0xa1fc, 0xa1fd, "t1", function() E("   -> $A1FC ack-low timeout") end)
  taps[#taps+1] = osp:install_read_tap(0xa232, 0xa233, "t2", function() E("   -> $A232 ack-high timeout") end)
  print("stakes armed"); io.flush(); return true
end

emu.register_periodic(function()
  if not arm() then return end
  if now() >= STOP then
    print("=== stakes + parity ===")
    for _, l in ipairs(ev) do print(l) end
    print(string.format("=== TOTAL stake instructions: $8120(c0)=%d  $8128(index)=%d  sum=%d",
      n8120, n8128, n8120 + n8128))
    io.flush(); manager.machine:exit()
  end
end)
