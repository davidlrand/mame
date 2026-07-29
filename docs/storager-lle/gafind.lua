-- gafind.lua - what ADDRESSES does the firmware hand the gate array during a read?
--
-- The completion deposit must be driven by a pointer the firmware PROGRAMMED, not by a constant
-- baked into the model.  This logs every CPU write into the gate-array register space and flags
-- any value that resolves (<<1) to the node or to node+$26 - the byte the watch pump gates on.
--
-- Word-address convention: the model already treats D800/C800 values as (addr >> 1).
--
-- USAGE: SDL_VIDEODRIVER=dummy CPUAP_RTCFIX=1 ./mame pcmx2 -video none -nothrottle \
--          -autoboot_script docs/storager-lle/gafind.lua -flop siemens/set1/mx2-001.imd

local SC   = ":slot1:storager:cpu"
local STOP = 30.0

local cpu, sp, armed = nil, nil, false
local taps, ev = {}, {}
local hits, raw = {}, {}            -- distinct (register,value) pairs, in order of first sight
local saw_read = false              -- did cmd 0x95 ever appear?  (guards against a no-read run)

local function now() local ok,t = pcall(function() return manager.machine.time:as_double() end); return ok and t or 0 end
local function W(a) return sp:read_u16(a & 0xffff) end
local function B(a) return sp:read_u8(a & 0xffff) end
local function PC() local ok,v = pcall(function() return cpu.state["PC"].value end); return (ok and v or 0) & 0xffffff end
-- g_reading is refreshed by the PERIODIC (outside any tap).  The tap must never call B()/W():
-- a Lua SRAM read inside a CPU write tap trips storager.cpp's own $4000-$7FFF DMA snoop.
local g_reading = false
local function reading() return g_reading end

local function arm()
  if armed then return true end
  local ok, d = pcall(function() return manager.machine.devices[SC] end)
  if not ok or not d then return false end
  armed = true; cpu = d; sp = d.spaces["program"]

  -- Every GA register window the firmware touches.  Ranges must END ON AN ODD address or
  -- install_write_tap rejects them (an even end silently aborts the whole script).
  -- The GA ranges are mapped with .mirror(0xff0000) and the firmware uses 68000 short-absolute,
  -- which sign-extends: the live accesses are at $FFxxxx.  Tap the MIRROR, not the base.
  local windows = {
    -- NARROW ONLY.  Blanketing $FFE000-$FFFFFF (the channel handler + the F000 comparator)
    -- made the board stop arming the read entirely - the instrument was mutating the model.
    {0xffc000, 0xffc7ff, "C000"}, {0xffc800, 0xffc9ff, "C800"},
    {0xffd000, 0xffd001, "D000"}, {0xffd800, 0xffd801, "D800"},
    -- NARROW E-windows: the op18 program area and the control regs.  Blanketing all of
    -- $FFE000-$FFFFFF stops the board arming, so keep these tight.
    {0xffe000, 0xffe0ff, "E000prog"}, {0xffe800, 0xffe80f, "E80x"},
  }
  -- CRITICAL: this handler must NOT read SRAM.  storager.cpp taps $4000-$7FFF for the DMA
  -- transfer-pointer snoop, gated on the CPU being the executing device - which is exactly the
  -- case inside a CPU write tap.  A Lua read of $799A here clobbers m_term_bit0 and the board
  -- stops arming the read entirely.  Record raw values only; resolve them after the run.
  for _, w in ipairs(windows) do
    taps[#taps+1] = sp:install_write_tap(w[1], w[2], w[3], function(off, data, mask)
      if not reading() then return end
      local v = data & 0xffff
      local k = string.format("%s+%04x=%04x", w[3], (off & 0xffff) - (w[1] & 0xffff), v)
      if not hits[k] then
        hits[k] = { reg = w[3], off = (off & 0xffff) - (w[1] & 0xffff), v = v, t = now(), pc = PC() }
        raw[#raw+1] = hits[k]
      end
    end)
  end
  print("gafind armed"); io.flush()
  return true
end

emu.register_periodic(function()
  local t = now()
  if not arm() then return end
  g_reading = (B(0x71f0) == 0x95)
  if g_reading then saw_read = true end
  if t >= STOP then
    print(string.format("=== read armed this run: %s ===", saw_read and "YES" or "*** NO - cmd 0x95 never seen, result meaningless ***"))
    print("=== ALL distinct GA register writes during cmd 95 ===")
    -- resolution happens HERE, outside any tap, so the SRAM reads are safe
    local uib, node = W(0x799a), W(0x71bc)
    for _, r in ipairs(raw) do
      local tgt = (r.v << 1) & 0xffff
      local tag = ""
      if tgt == uib then tag = "*** == UIB[799a] - the byte op42 tests ***"
      elseif tgt == ((uib + 0x12) & 0xffff) then tag = "*** == UIB+$12 ***"
      elseif tgt == node then tag = "== node[71bc]"
      elseif tgt >= 0x4000 and tgt <= 0x7fff then tag = "-> SRAM"
      end
      print(string.format("%9.4f  %-9s+%04x <= %04x  (<<1 = %04x)  pc=%06x  %s",
        r.t, r.reg, r.off, r.v, tgt, r.pc, tag))
    end
    print(string.format("=== UIB[799a]=%04x (>>1 = %04x)  UIB+12=%04x", uib, uib >> 1, (uib + 0x12) & 0xffff))
    print(string.format("=== node[71bc]=%04x  node+26=%04x (>>1 = %04x)  byte@node+26=%02x",
      node, (node + 0x26) & 0xffff, ((node + 0x26) & 0xffff) >> 1, B((node + 0x26) & 0xffff)))
    io.flush(); manager.machine:exit()
  end
end)
