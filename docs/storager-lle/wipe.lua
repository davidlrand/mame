-- wipe.lua - does $5E64 run during a read, and does it WIPE the field program?
--
-- $5E98 lea $e000,a1 / $5E9C move.w #$f,d2 / $5EA0 move.w #$23f,(a1)+ / dbra
--   = SIXTEEN words of $023F across E000-E01E.  $023F is the idle/terminate step that tails op18's
--   block, so this CLEARS the program; it does not load one.  (An earlier note here had it backwards.)
--
-- The model's ch_w treats any write to E000 offset 15 with a preceding offset>=1 as a load-complete
-- edge: it sets m_prog_loaded and calls start_field_program().  So a wipe is read as a LOAD and
-- ENGAGES the window - model and firmware would then disagree about what the gate array is running.
--
-- Two taps, both well past the prologue (entry taps have been prefetch artifacts four times):
--   $5E82 - the ladder emit (move.w #$1c,(A6)+)
--   $5EA0 - the E000 wipe loop body
-- A6 is the mechanism: $5E64 emits through A6, the dispatcher sets A6 at $2338, and the walker's
-- $15AC movem.w A0-A2,-(A7) preserves only A0-A2 - so A6 survives into a micro-op call and an op
-- dispatching $5E64 would emit into whatever ladder A6 still points at.
--
-- USAGE: SDL_VIDEODRIVER=dummy CPUAP_RTCFIX=1 ./mame pcmx2 -video none -nothrottle -oslog \
--   -autoboot_script docs/storager-lle/wipe.lua -flop siemens/set1/mx2-001.imd -hard1 <img>

local SC, STOP = ":slot1:storager:cpu", 45.0
local cpu, sp, osp, armed = nil, nil, nil, false
local taps, rows, cur = {}, {}, nil
local lastcmd, nodebase = -1, 0x71f0

local function now() local ok,t = pcall(function() return manager.machine.time:as_double() end); return ok and t or 0 end
local function B(a) return sp:read_u8(a & 0xffff) end
local function W(a) return sp:read_u16(a & 0xffff) end
local function R(n) local ok,v = pcall(function() return cpu.state[n].value end); return ok and v or 0 end

local function arm()
  if armed then return true end
  local ok, d = pcall(function() return manager.machine.devices[SC] end)
  if not ok or not d then return false end
  armed = true; cpu = d; sp = d.spaces["program"]
  osp = d.spaces["decrypted_opcodes"] or d.spaces["opcodes"] or sp
  taps[#taps+1] = osp:install_read_tap(0x5e82, 0x5e83, "emit", function()
    if cur then
      cur.emit = cur.emit + 1
      if not cur.a6 then cur.a6 = R("A6") & 0xffff end
    end
  end)
  taps[#taps+1] = osp:install_read_tap(0x5ea0, 0x5ea1, "wipe", function()
    if cur then cur.wipe = cur.wipe + 1 end
  end)
  -- and the model's own engagement, so divergence is visible in one row
  taps[#taps+1] = sp:install_write_tap(0x00e01e, 0x00e01f, "e01e", function(off, data, mask)
    if cur then cur.e01e = cur.e01e + 1; cur.lastv = data & 0xffff end
  end)
  taps[#taps+1] = sp:install_write_tap(0xffe01e, 0xffe01f, "e01e2", function(off, data, mask)
    if cur then cur.e01e = cur.e01e + 1; cur.lastv = data & 0xffff end
  end)
  print("wipe armed"); io.flush(); return true
end

emu.register_periodic(function()
  if not arm() then return end
  local n = W(0x71bc); if n >= 0x4000 and n < 0x8000 then nodebase = n end
  local c = B(nodebase)
  if c ~= lastcmd then
    if c ~= 0 then
      cur = { cmd = c, t = now(), emit = 0, wipe = 0, e01e = 0 }
      rows[#rows+1] = cur
    else cur = nil end
    lastcmd = c
  end
  if now() >= STOP then
    print("=== $5E64: ladder emit and E000 wipe, per command ===")
    print("   cmd    t=start   $5E82 emit   $5EA0 wipe   E01E writes (last value)   A6 at emit")
    for _, r in ipairs(rows) do
      print(string.format("   %02x   %8.4f   %8d     %8d      %5d (%04x)            %s",
        r.cmd, r.t, r.emit, r.wipe, r.e01e, r.lastv or 0,
        r.a6 and string.format("%04x", r.a6) or "-"))
    end
    print("  $5EA0 x16 = a full E000 wipe.  If that fires on a 0x95 while the model has a program")
    print("  'loaded', model and firmware have diverged about what the gate array is running.")
    io.flush(); manager.machine:exit()
  end
end)
