-- ctlcmp.lua - control-command comparison.
--
-- Static's discriminator: count $6ED2 (op4A) dispatches per command.  [$7968] is cleared at intake
-- ($155C/$190E), so op4A always sees 0 on a fresh command and the deferral leg can only be taken on a
-- SECOND invocation - and the only second invocation in the ROM is $948E, inside rec5's $9398, which
-- $70B4 unparks only when [$7958] != 0.
--
--   2 dispatches  -> the control has rec5 active ([$7958] != 0); the read's single-track,
--                    remainder-zero geometry is the whole difference
--   1, launches   -> [$74b4] was already non-empty at op4A; who filled it is the question
--
-- Also logs [$7958]/[$7abe]/[$7abc] at op58, which is where $738C clr.l $7abe clamps a request that
-- fits in one track.
--
-- USAGE: SDL_VIDEODRIVER=dummy CPUAP_RTCFIX=1 ./mame pcmx2 -video none -nothrottle -oslog \
--          -autoboot_script docs/storager-lle/ctlcmp.lua -flop siemens/set1/mx2-001.imd

local SC, STOP = ":slot1:storager:cpu", 30.0
local cpu, sp, osp, armed = nil, nil, nil, false
local taps, ev = {}, {}
local curcmd, n4a = 0, 0

local function now() local ok,t = pcall(function() return manager.machine.time:as_double() end); return ok and t or 0 end
local function W(a) return sp:read_u16(a & 0xffff) end
local function B(a) return sp:read_u8(a & 0xffff) end
local function L(a) return (W(a) << 16) | W(a + 2) end
local function E(s) if #ev < 300 then ev[#ev+1] = string.format("%9.4f  %s", now(), s) end end

local function arm()
  if armed then return true end
  local ok, d = pcall(function() return manager.machine.devices[SC] end)
  if not ok or not d then return false end
  armed = true; cpu = d; sp = d.spaces["program"]
  osp = d.spaces["decrypted_opcodes"] or d.spaces["opcodes"] or sp

  -- op4A entry.  $6ED2 is the handler's first instruction (move #$2700,sr), clear of any branch shadow.
  taps[#taps+1] = osp:install_read_tap(0x6ed2, 0x6ed3, "op4a", function()
    n4a = n4a + 1
    E(string.format("op4A #%d  cmd=%02x  [74b4]=%04x [791a]=%04x %s | [7968]=%04x [7b10]=%04x [7958]=%08x [7abc]=%04x",
      n4a, B(0x71f0), W(0x74b4), W(0x791a),
      (W(0x74b4) ~= 0 and W(0x791a) == 0) and "*** $70F4 WOULD FIRE ***"
        or ((W(0x791a) ~= 0) and "($791a set -> $70FA, $70EA never evaluated)" or "(queue empty)"),
      W(0x7968), W(0x7b10), L(0x7958), W(0x7abc)))
  end)
  -- $948E: the ROM's only second dispatch of op4A (inside rec5's $9398 state machine)
  taps[#taps+1] = osp:install_read_tap(0x948e, 0x948f, "948e", function()
    E(string.format("   $948E reached (rec5 second op4A)  cmd=%02x [7958]=%08x", B(0x71f0), L(0x7958)))
  end)
  -- rec5 unpark, the gate on [$7958]
  taps[#taps+1] = osp:install_read_tap(0x70b4, 0x70b5, "70b4", function()
    E(string.format("   $70B4 UNPARK rec5   [7958]=%08x  (drain succeeded)", L(0x7958)))
  end)
  -- op58: the clamp site
  taps[#taps+1] = osp:install_read_tap(0x7350, 0x7351, "op58", function()
    E(string.format("   op58  cmd=%02x [7958]=%08x [7abe]=%04x [7abc]=%04x",
      B(0x71f0), L(0x7958), W(0x7abe), W(0x7abc)))
  end)
  -- the launch itself
  taps[#taps+1] = osp:install_read_tap(0x70f4, 0x70f5, "70f4", function()
    E(string.format("   $70F4 -> $3DBC LAUNCH  cmd=%02x [74b4]=%04x", B(0x71f0), W(0x74b4)))
  end)
  print("ctlcmp armed"); io.flush(); return true
end

emu.register_periodic(function()
  if not arm() then return end
  if now() >= STOP then
    print("=== control comparison ===")
    for _, l in ipairs(ev) do print(l) end
    print(string.format("=== total op4A dispatches in run: %d", n4a))
    io.flush(); manager.machine:exit()
  end
end)
