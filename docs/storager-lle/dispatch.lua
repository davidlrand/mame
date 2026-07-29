-- dispatch.lua - does $2342 jsr (A1) actually run, and with what?
--
-- $231A cmpi.b #$ff,($4,A1) gates it - a BYTE test on a field written only as WORDS ($0AA2, $0AB6,
-- $15C0, $27F4).  That is the same byte/word pairing the SRAM byte-order fix turned on, so this gate
-- may have changed state as a side effect of that fix.
--
-- A1 is read AT THE SITE - a register read, so no binning and no periodic sampling (the method that
-- made two wipe probes agree and both be wrong).  $2342 is well past $2330's branch.
--   fires, A1 non-zero -> the +$6 slot IS written; "never written, reads zero" was an artifact
--   never fires        -> $2342 is not the dispatch path and $231A's gate is where the trail starts
--
-- USAGE: SDL_VIDEODRIVER=dummy CPUAP_RTCFIX=1 ./mame pcmx2 -video none -nothrottle -oslog \
--   -autoboot_script docs/storager-lle/dispatch.lua -flop siemens/set1/mx2-001.imd -hard1 <img>

local SC, STOP = ":slot1:storager:cpu", 45.0
local cpu, sp, osp, armed = nil, nil, nil, false
local taps, ev = {}, {}
local n2342, n233a, n231a = 0, 0, 0

local function now() local ok,t = pcall(function() return manager.machine.time:as_double() end); return ok and t or 0 end
local function R(n) local ok,v = pcall(function() return cpu.state[n].value end); return ok and v or 0 end
local function E(s) if #ev < 120 then ev[#ev+1] = string.format("%9.4f  %s", now(), s) end end

local NAME = { [0x5fc0]="$5FC0 READ builder", [0x6102]="$6102", [0x5e64]="$5E64 short builder",
               [0xa63e]="$A63E cmd 0x98 builder" }

local function arm()
  if armed then return true end
  local ok, d = pcall(function() return manager.machine.devices[SC] end)
  if not ok or not d then return false end
  armed = true; cpu = d; sp = d.spaces["program"]
  osp = d.spaces["decrypted_opcodes"] or d.spaces["opcodes"] or sp
  taps[#taps+1] = osp:install_read_tap(0x231a, 0x231b, "gate", function() n231a = n231a + 1 end)
  taps[#taps+1] = osp:install_read_tap(0x233a, 0x233b, "load", function()
    n233a = n233a + 1
    if n233a <= 6 then E(string.format("$233A movea.w ($6,A1),A1   A1(desc)=%04x", R("A1") & 0xffff)) end
  end)
  taps[#taps+1] = osp:install_read_tap(0x2342, 0x2343, "jsr", function()
    n2342 = n2342 + 1
    local a1 = R("A1") & 0xffff
    if n2342 <= 12 then
      E(string.format("$2342 jsr (A1)   A1=%04x   %s", a1, NAME[a1] or ((a1 == 0) and "*** ZERO ***" or "(other)")))
    end
  end)
  print("dispatch armed"); io.flush(); return true
end

emu.register_periodic(function()
  if not arm() then return end
  if now() >= STOP then
    print("=== $2342 dispatch ===")
    for _, l in ipairs(ev) do print(l) end
    print(string.format("=== $231A gate reached: %d   $233A slot load: %d   $2342 jsr: %d",
      n231a, n233a, n2342))
    io.flush(); manager.machine:exit()
  end
end)
