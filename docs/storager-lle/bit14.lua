-- bit14.lua - which record template does op54 install for the read?
--
-- $A35A movea.w $799a,a0 / $A35E move.w ($20,a0),d0 / $A362 btst #$e,d0 / $A366 beq -> template A.
--   A ($066A): rec1 flags=ffff PARKED, cb=$3DBC  - enterable only via the kick
--   B ($06A6): rec1 flags=0000 LIVE,   cb=$4362  - a complete launcher ($3ABC queue, $743a<-$7442, PIT0)
-- Read the SRAM directly at the $A362 opcode fetch (not D0) to stay clear of the prefetch shadow.
--
-- USAGE: SDL_VIDEODRIVER=dummy CPUAP_RTCFIX=1 ./mame pcmx2 -video none -nothrottle -oslog \
--          -autoboot_script docs/storager-lle/bit14.lua -flop siemens/set1/mx2-001.imd

local SC, STOP = ":slot1:storager:cpu", 30.0
local cpu, sp, osp, armed = nil, nil, nil, false
local taps, ev = {}, {}

local function now() local ok,t = pcall(function() return manager.machine.time:as_double() end); return ok and t or 0 end
local function W(a) return sp:read_u16(a & 0xffff) end
local function B(a) return sp:read_u8(a & 0xffff) end
local function E(s) if #ev < 200 then ev[#ev+1] = string.format("%9.4f  %s", now(), s) end end

local function arm()
  if armed then return true end
  local ok, d = pcall(function() return manager.machine.devices[SC] end)
  if not ok or not d then return false end
  armed = true; cpu = d; sp = d.spaces["program"]
  osp = d.spaces["decrypted_opcodes"] or d.spaces["opcodes"] or sp

  taps[#taps+1] = osp:install_read_tap(0xa362, 0xa363, "sel", function()
    local uib = W(0x799a)
    local v   = W((uib + 0x20) & 0xffff)
    local b14 = (v >> 14) & 1
    E(string.format("op54 TEMPLATE SELECT  cmd=%02x  [799a]=%04x  UIB+20 @%04x = %04x  bit14=%d -> template %s (rec1 %s, cb $%s)",
      B(0x71f0), uib, (uib + 0x20) & 0xffff, v, b14,
      (b14 == 1) and "B" or "A",
      (b14 == 1) and "LIVE" or "PARKED",
      (b14 == 1) and "4362" or "3DBC"))
  end)
  -- the seed fork, same bit
  taps[#taps+1] = osp:install_read_tap(0x73b0, 0x73b1, "seed", function()
    local uib = W(0x799a)
    E(string.format("   $73B0 seed fork    cmd=%02x  UIB+20=%04x bit14=%d  [7956]=%04x [7958]=%04x%04x",
      B(0x71f0), W((uib + 0x20) & 0xffff), (W((uib + 0x20) & 0xffff) >> 14) & 1,
      W(0x7956), W(0x7958), W(0x795a)))
  end)
  -- did rec1 land live or parked?
  taps[#taps+1] = sp:install_write_tap(0x7286, 0x7287, "rec1", function(off, data, mask)
    E(string.format("   rec1 flags <= %04x pc=%06x  %s", data & 0xffff,
      (function() local ok,v=pcall(function() return cpu.state["PC"].value end); return (ok and v or 0) & 0xffffff end)(),
      ((data & 0xffff) == 0) and "LIVE" or "PARKED"))
  end)
  -- and is $4362 ever entered?
  taps[#taps+1] = osp:install_read_tap(0x4362, 0x4363, "l4362", function()
    E(string.format("   *** $4362 LAUNCHER ENTERED ***  cmd=%02x", B(0x71f0)))
  end)
  print("bit14 armed"); io.flush(); return true
end

emu.register_periodic(function()
  if not arm() then return end
  if now() >= STOP then
    print("=== template selection ===")
    for _, l in ipairs(ev) do print(l) end
    io.flush(); manager.machine:exit()
  end
end)
