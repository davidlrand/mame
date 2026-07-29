-- seekwin.lua - what happens during op28's 1.5s spin, and is an IRQ5 safe to deliver there?
--
-- The kick ($7A30 bit0) is armed at $6042 (ladder build) and destroyed at $70D4 (drain) before any
-- disk IRQ exists.  $3DBC's self-sustaining loop ($3DE2 re-arms) can only be entered by an ISR in
-- that window.  $833C lives in the IRQ5 DONE tail ($8018 -> $8214 -> $824A -> $8320), so an IRQ5
-- would enter it - but only if the IRQ5 vector [$7302] and the alternator [$7950] route there.
--
-- USAGE: SDL_VIDEODRIVER=dummy CPUAP_RTCFIX=1 ./mame pcmx2 -video none -nothrottle \
--          -autoboot_script docs/storager-lle/seekwin.lua -flop siemens/set1/mx2-001.imd

local SC   = ":slot1:storager:cpu"
local STOP = 30.0

local cpu, sp, armed = nil, nil, false
local taps, ev = {}, {}
local last = nil

local function now() local ok,t = pcall(function() return manager.machine.time:as_double() end); return ok and t or 0 end
local function W(a) return sp:read_u16(a & 0xffff) end
local function B(a) return sp:read_u8(a & 0xffff) end
local function PC() local ok,v = pcall(function() return cpu.state["PC"].value end); return (ok and v or 0) & 0xffffff end
local function E(s) if #ev < 200 then ev[#ev+1] = string.format("%9.4f  %s", now(), s) end end

local function arm()
  if armed then return true end
  local ok, d = pcall(function() return manager.machine.devices[SC] end)
  if not ok or not d then return false end
  armed = true; cpu = d; sp = d.spaces["program"]
  -- [$7a36] = settle complete; it is what releases op28's spin
  taps[#taps+1] = sp:install_write_tap(0x7a36, 0x7a37, "7a36", function(off, data, mask)
    E(string.format("[7a36] <= %04x pc=%06x   *** SETTLE COMPLETE - releases op28 ***", data & 0xffff, PC()))
  end)
  -- the IRQ5 vector the stub dispatches through, and the alternator that forks setup-vs-done
  taps[#taps+1] = sp:install_write_tap(0x7302, 0x7303, "7302", function(off, data, mask)
    E(string.format("[7302] IRQ5 vector <= %04x pc=%06x  (%s)", data & 0xffff, PC(),
      (data & 0xffff) == 0x7ba8 and "$7BA8 setup" or (data & 0xffff) == 0x86f6 and "$86F6" or "?"))
  end)
  -- the kick's whole life
  taps[#taps+1] = sp:install_write_tap(0x7a30, 0x7a31, "7a30", function(off, data, mask)
    local sz = (mask == 0xffff) and "WORD" or ((mask == 0xff00) and "byte-hi" or "byte-lo")
    E(string.format("[7a30] <= %04x %-7s pc=%06x", data & 0xffff, sz, PC()))
  end)
  print("seekwin armed"); io.flush()
  return true
end

emu.register_periodic(function()
  local t = now()
  if not arm() then return end
  -- Sample on change, but NOT on F000 bit11: that is the PIT1 ctr0 square wave toggling every
  -- ~13ms, which fills the event buffer at t=5.25s - before the window we care about (cap failure).
  -- Mask it out, and only log once a command is live.
  local c = B(0x71f0)
  if c ~= 0 then
    local s = string.format("cmd=%02x [7a36]=%04x [7302]=%04x [7950]=%04x [7a30]hi=%02x F000=%04x",
      c, W(0x7a36), W(0x7302), W(0x7950), B(0x7a30), W(0xf000) & ~0x0800)
    if s ~= last then last = s; E("STATE " .. s) end
  end
  if t >= STOP then
    print("=== the seek window ===")
    for _, l in ipairs(ev) do print(l) end
    io.flush(); manager.machine:exit()
  end
end)
