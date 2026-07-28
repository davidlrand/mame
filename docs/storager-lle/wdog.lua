-- wdog.lua - which site arms the $7986 watchdog, and does op4A's deferral leg finally run?
--
-- $6EF0, $7B1C and $84E6 are byte-identical arms of one deferred write via $29F8: error $201C,
-- target [$71be], 50 ticks (x 26.1ms = 1.305s), pending cell $7986.  Only the first to arrive takes
-- it (each is guarded by tst.w $7986).  Cancel is $2ABA(#$7986), on $7A4C / $8432 / $9206.
--
-- ALL DATA TAPS.  Four opcode taps in this session turned out to be prefetch artifacts; a write tap
-- catches the arm whichever site did it and cannot be fooled.  The caller is recovered from the
-- return address on the stack, not from a PC.
--
-- PREDICTION under test: if $6EF0 armed it, [$7968] != 0 at op4A and [$7b10] survives past $6F3A for
-- the first time - i.e. the deferral leg is live.  If $7B1C/$84E6, [$7968] is 0 as always and the
-- deferral leg is still dead.
--
-- USAGE: SDL_VIDEODRIVER=dummy CPUAP_RTCFIX=1 ./mame pcmx2 -video none -nothrottle -oslog \
--   -autoboot_script docs/storager-lle/wdog.lua -flop siemens/set1/mx2-001.imd -hard1 <img>

local SC, STOP = ":slot1:storager:cpu", 30.0
local cpu, sp, armed = nil, nil, false
local taps, ev = {}, {}

local function now() local ok,t = pcall(function() return manager.machine.time:as_double() end); return ok and t or 0 end
local function W(a) return sp:read_u16(a & 0xffff) end
local function R(n) local ok,v = pcall(function() return cpu.state[n].value end); return ok and v or 0 end
local function E(s) if #ev < 200 then ev[#ev+1] = string.format("%9.4f  %s", now(), s) end end

local SITE = { [0x6ef4]="$6EF0  op4A DEFERRAL LEG  *** [7968]!=0 - the branch that was unreachable ***",
               [0x6ef0]="$6EF0  op4A DEFERRAL LEG  *** [7968]!=0 - the branch that was unreachable ***",
               [0x7b20]="$7B1C  record/completion chain", [0x7b1c]="$7B1C  record/completion chain",
               [0x84ea]="$84E6  record/completion chain", [0x84e6]="$84E6  record/completion chain" }

local function arm_taps()
  if armed then return true end
  local ok, d = pcall(function() return manager.machine.devices[SC] end)
  if not ok or not d then return false end
  armed = true; cpu = d; sp = d.spaces["program"]

  taps[#taps+1] = sp:install_write_tap(0x7986, 0x7987, "wd", function(off, data, mask)
    -- recover the caller: scan the top of stack for a return address near the three known arms
    local a7, who = R("A7") & 0xffffff, "?"
    local dump = {}
    for i = 0, 30, 2 do
      local w16 = W(a7 + i)
      dump[#dump+1] = string.format("%04x", w16)
      if SITE[w16] then who = SITE[w16] end
      -- return addresses sit in the low half of a pushed long
      if w16 >= 0x6e00 and w16 <= 0x8600 and who == "?" then
        who = string.format("candidate ret $%04x", w16)
      end
    end
    if who == "?" then who = "stack: " .. table.concat(dump, " ") end
    E(string.format("$7986 <= %04x  ARMED by %s | [7968]=%04x [7b10]=%04x [74b4]=%04x [7a64]=%04x",
      data & 0xffff, who, W(0x7968), W(0x7b10), W(0x74b4), W(0x7a64)))
  end)
  -- [$7b10]: set only by op4A; survives ONLY on the deferral leg
  taps[#taps+1] = sp:install_write_tap(0x7b10, 0x7b11, "b10", function(off, data, mask)
    E(string.format("   [7b10] <= %04x   %s  ([7968]=%04x)", data & 0xffff,
      ((data & 0xffff) ~= 0) and "REQUEST armed" or "consumed/cleared", W(0x7968)))
  end)
  taps[#taps+1] = sp:install_write_tap(0x7968, 0x7969, "968", function(off, data, mask)
    E(string.format("   [7968] <= %04x", data & 0xffff))
  end)
  -- the fire: the deferred write lands on [$71be]'s target
  taps[#taps+1] = sp:install_write_tap(0x71be, 0x71bf, "tgt", function(off, data, mask)
    E(string.format("   [71be] <= %04x (watchdog target)", data & 0xffff))
  end)
  -- the three arms all guard with tst.w $7986.  A READ tap is a data access (prefetch-immune), and
  -- $6EF0 / $7B1C / $84E6 are >1KB apart, so the PC cannot be mistaken between them.
  taps[#taps+1] = sp:install_read_tap(0x7986, 0x7987, "guard", function()
    local pc = R("PC") & 0xffffff
    local site = "?"
    if pc >= 0x6ee0 and pc <= 0x6f10 then site = "$6EF0 op4A DEFERRAL LEG"
    elseif pc >= 0x7b00 and pc <= 0x7b40 then site = "$7B1C record/completion"
    elseif pc >= 0x84d0 and pc <= 0x8500 then site = "$84E6 record/completion"
    elseif pc >= 0x2ab0 and pc <= 0x2b00 then site = "$2ABA CANCEL"
    elseif pc >= 0x29f0 and pc <= 0x2a40 then site = "$29F8 arm body" end
    if site == "$2ABA CANCEL" or site == "$29F8 arm body" then return end
    if site ~= "?" then
      E(string.format("   guard read of $7986 at pc=%06x -> %s  (current=%04x)", pc, site, W(0x7986)))
    end
  end)
  print("wdog armed"); io.flush(); return true
end

emu.register_periodic(function()
  if not arm_taps() then return end
  if now() >= STOP then
    print("=== watchdog arm/cancel ===")
    for _, l in ipairs(ev) do print(l) end
    io.flush(); manager.machine:exit()
  end
end)
