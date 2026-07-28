-- cmd98.lua - which unit does cmd 0x98 target, and does $A1CE's serial transaction time out?
--
-- $A1CE is a bit-banged ESDI serial read: CLOCK on E802 bit0, transfer-ack on F000 bit1, data on
-- F000 bit4, 16 bits, odd parity.  Failure at any stage -> D0=$201E -> status $82/$1E.
-- If 0x98 targets a unit this machine does not have, the faithful answer is "not present" and the
-- command should fail cleanly - NOT a responder emulating a drive that isn't there.
--
-- USAGE: SDL_VIDEODRIVER=dummy CPUAP_RTCFIX=1 ./mame pcmx2 -video none -nothrottle -oslog \
--          -autoboot_script docs/storager-lle/cmd98.lua -flop siemens/set1/mx2-001.imd

local SC, STOP = ":slot1:storager:cpu", 30.0
local cpu, sp, osp, armed = nil, nil, nil, false
local taps, ev = {}, {}
local nser = 0

local function now() local ok,t = pcall(function() return manager.machine.time:as_double() end); return ok and t or 0 end
local function W(a) return sp:read_u16(a & 0xffff) end
local function B(a) return sp:read_u8(a & 0xffff) end
local function R(n) local ok,v = pcall(function() return cpu.state[n].value end); return ok and v or 0 end
local function E(s) if #ev < 200 then ev[#ev+1] = string.format("%9.4f  %s", now(), s) end end

local function arm()
  if armed then return true end
  local ok, d = pcall(function() return manager.machine.devices[SC] end)
  if not ok or not d then return false end
  armed = true; cpu = d; sp = d.spaces["program"]
  osp = d.spaces["decrypted_opcodes"] or d.spaces["opcodes"] or sp

  -- 0x98's builder: which unit?
  taps[#taps+1] = osp:install_read_tap(0xa63e, 0xa63f, "b98", function()
    local node = W(0x71bc)
    E(string.format("$A63E cmd98 builder  node=%04x  node+4=%04x node+6=%04x  [7a0a]=%04x  F000=%04x",
      node, W(node + 4), W(node + 6), W(0x7a0a), W(0xf000)))
  end)
  -- the serial transaction
  taps[#taps+1] = osp:install_read_tap(0xa1ce, 0xa1cf, "ser", function()
    nser = nser + 1
    E(string.format("  $A1CE serial read #%d  [7a0a]=%04x F000=%04x (bit1=%d bit4=%d) [79f8]=%04x",
      nser, W(0x7a0a), W(0xf000), (W(0xf000) >> 1) & 1, (W(0xf000) >> 4) & 1, W(0x79f8)))
  end)
  -- the three failure exits
  taps[#taps+1] = osp:install_read_tap(0xa1fc, 0xa1fd, "to1", function()
    E("     -> $A1FC TIMEOUT waiting F000 bit1 CLEAR (ack low)")
  end)
  taps[#taps+1] = osp:install_read_tap(0xa232, 0xa233, "to2", function()
    E("     -> $A232 TIMEOUT waiting F000 bit1 SET  (ack high)  *** nothing drives it ***")
  end)
  taps[#taps+1] = osp:install_read_tap(0xa240, 0xa241, "par", function()
    E("     -> $A240 PARITY FAIL")
  end)
  taps[#taps+1] = osp:install_read_tap(0xa246, 0xa247, "okk", function()
    E(string.format("     -> $A246 SUCCESS  D1=%08x", R("D1")))
  end)
  -- and the other caller, on the completion path
  taps[#taps+1] = osp:install_read_tap(0x1a34, 0x1a35, "c2", function()
    E(string.format("  $1A34 calls $A1CE on the COMPLETION path  cmd=%02x", B(0x71f0)))
  end)
  print("cmd98 armed"); io.flush(); return true
end

emu.register_periodic(function()
  if not arm() then return end
  if now() >= STOP then
    print("=== cmd 0x98 / ESDI serial ===")
    for _, l in ipairs(ev) do print(l) end
    print(string.format("=== $A1CE invocations: %d", nser))
    io.flush(); manager.machine:exit()
  end
end)
