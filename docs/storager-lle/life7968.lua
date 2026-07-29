-- life7968.lua - the WHOLE-BOOT life of [$7968], [$7b10] and op4A, across every command.
--
-- WHY: within cmd 0x95, [$7968] is provably 0 at op4A, which forces the inline drain against an
-- empty ledger and deadlocks the completion chain.  But nothing REQUIRES it to start at 0 - it is
-- only cleared at $7C92 when a matched record finds it already set.  If an earlier command (INIT,
-- seek, the 0x87s) leaves it 1, op4A defers and the ISR drains after the records.  So trace it
-- across the entire boot, NOT gated on cmd 0x95.
--
-- Taps record RAW values only - a Lua SRAM read inside a CPU write tap trips storager.cpp's own
-- $4000-$7FFF DMA snoop.  Everything is resolved in the periodic.
--
-- USAGE: SDL_VIDEODRIVER=dummy CPUAP_RTCFIX=1 ./mame pcmx2 -video none -nothrottle \
--          -autoboot_script docs/storager-lle/life7968.lua -flop siemens/set1/mx2-001.imd

local SC   = ":slot1:storager:cpu"
local STOP = 30.0
local MAX  = 300

local cpu, sp, osp, armed = nil, nil, nil, false
local taps, ev = {}, {}
local cur_cmd = -1

local function now() local ok,t = pcall(function() return manager.machine.time:as_double() end); return ok and t or 0 end
local function W(a) return sp:read_u16(a & 0xffff) end
local function B(a) return sp:read_u8(a & 0xffff) end
local function PC() local ok,v = pcall(function() return cpu.state["PC"].value end); return (ok and v or 0) & 0xffffff end

local function log(tag, detail)
  if #ev < MAX then
    ev[#ev+1] = string.format("%9.4f [cmd %02x] %-12s %s", now(), cur_cmd & 0xff, tag, detail or "")
  end
end

local function arm()
  if armed then return true end
  local ok, d = pcall(function() return manager.machine.devices[SC] end)
  if not ok or not d then return false end
  armed = true; cpu = d; sp = d.spaces["program"]
  osp = d.spaces["decrypted_opcodes"] or d.spaces["opcodes"] or sp

  -- UNGATED: every write, whole boot
  taps[#taps+1] = sp:install_write_tap(0x7968, 0x7969, "7968", function(off, data, mask)
    local v, pc = data & 0xffff, PC()
    log("7968 <=", string.format("%04x  pc=%06x  %s", v, pc,
      (v ~= 0) and "*** SET ***" or
      ((pc >= 0x7c92 and pc <= 0x7c9c) and "clear $7C92 (matched record)" or
       (pc >= 0x70da and pc <= 0x70e4) and "clear $70DA (drain fail)" or
       (pc >= 0x82c6 and pc <= 0x82d0) and "clear $82C6" or
       (pc >= 0x155c and pc <= 0x1566) and "clear $155C (walker)" or
       (pc >= 0x79ca and pc <= 0x79d4) and "clear $79CA" or "clear")))
  end)
  taps[#taps+1] = sp:install_write_tap(0x7b10, 0x7b11, "7b10", function(off, data, mask)
    log("7b10 <=", string.format("%04x  pc=%06x", data & 0xffff, PC()))
  end)
  -- op4A's decision point: $6ED6 raises [$7b10]; the fork on [$7968] follows immediately
  taps[#taps+1] = osp:install_read_tap(0x6edc, 0x6edd, "op4A", function()
    log("op4A fork", string.format("[7968]=%04x -> %s", W(0x7968),
      (W(0x7968) == 0) and "INLINE drain (empty ledger)" or "*** DEFER to ISR ***"))
  end)
  print("life7968 armed"); io.flush()
  return true
end

emu.register_periodic(function()
  local t = now()
  if not arm() then return end
  local c = B(0x71f0)
  if c ~= cur_cmd then
    cur_cmd = c
    log("CMD", string.format("command byte now %02x   st=%04x  [7968]=%04x", c, W(0x71f2), W(0x7968)))
  end
  if t >= STOP then
    print("=== whole-boot life of [$7968] / [$7b10] / op4A ===")
    for _, l in ipairs(ev) do print(l) end
    print(string.format("=== final: cmd=%02x [7968]=%04x [7b10]=%04x [72d8]=%04x",
      B(0x71f0), W(0x7968), W(0x7b10), W(0x72d8)))
    io.flush(); manager.machine:exit()
  end
end)
