-- ladret.lua - the ladder as executed, with each micro-op's RETURN CODE.
--
-- The walker dispatches at $15BA (jsr (a1)) and the handler's return code lands in D0 at $15BC:
-- 0 = advance, $FE/$FD/$FF = retry (which is what makes the walker PARK into phase $0A and run the
-- watch pump).  The question this answers: can any op BEFORE op4A park?  If none can, then the pump
-- cannot run before op4A, [$7968] cannot be set, and op4A can never take its deferral arm.
--
-- USAGE: SDL_VIDEODRIVER=dummy CPUAP_RTCFIX=1 ./mame pcmx2 -video none -nothrottle \
--          -autoboot_script docs/storager-lle/ladret.lua -flop siemens/set1/mx2-001.imd

local SC, STOP = ":slot1:storager:cpu", 30.0
local cpu, sp, osp, armed = nil, nil, nil, false
local taps, ev = {}, {}
local curop = "?"
local lastkey, reps = nil, 0

local function now() local ok,t = pcall(function() return manager.machine.time:as_double() end); return ok and t or 0 end
local function R(n) local ok,v = pcall(function() return cpu.state[n].value end); return ok and v or 0 end
local function E(s) if #ev < 600 then ev[#ev+1] = string.format("%9.4f  %s", now(), s) end end

local function arm()
  if armed then return true end
  local ok, d = pcall(function() return manager.machine.devices[SC] end)
  if not ok or not d then return false end
  armed = true; cpu = d; sp = d.spaces["program"]
  osp = d.spaces["decrypted_opcodes"] or d.spaces["opcodes"] or sp
  -- $15B4 move.w (a0,d0.w),d0 : D0 carries the op code index just before dispatch
  taps[#taps+1] = osp:install_read_tap(0x15ba, 0x15bb, "disp", function()
    curop = string.format("%04x", R("A1") & 0xffff)
  end)
  -- return point: D0 = the handler's verdict
  taps[#taps+1] = osp:install_read_tap(0x15bc, 0x15bd, "ret", function()
    local d0 = R("D0") & 0xffff
    local verdict = (d0 == 0) and "ADVANCE" or
      ((d0 == 0xfe or d0 == 0xfd or d0 == 0xff or d0 == 0x00fe) and "RETRY/PARK" or "other")
    local key = string.format("%s/%04x", curop, d0)
    if key == lastkey then reps = reps + 1; return end
    if reps > 0 then
      if #ev > 0 then ev[#ev] = ev[#ev] .. string.format("   x%d", reps + 1) end
      reps = 0
    end
    lastkey = key
    E(string.format("handler $%s -> D0=%04x  %s", curop, d0, verdict))
  end)
  print("ladret armed"); io.flush(); return true
end

emu.register_periodic(function()
  if not arm() then return end
  if now() >= STOP then
    print("=== ladder as executed, with return codes ===")
    for _, l in ipairs(ev) do print(l) end
    io.flush(); manager.machine:exit()
  end
end)
