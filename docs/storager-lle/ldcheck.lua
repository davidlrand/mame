-- ldcheck.lua - the seed-immune acceptance test for program retention.
--
-- With the gate array retaining its field program, the SECOND read must show record traffic
-- (IRQ6/IRQ5) with NO C800/E000 program-load burst at all - because $5FF6 skips op18 entirely when
-- [$793e] already names the program the command needs.  That signature is structural within a single
-- command's trace: it does not care when the RTC seeded, and no other explanation produces it.
--   records + no load  -> retention works
--   silence            -> the implementation is wrong, whatever the boot count says
--
-- USAGE: SDL_VIDEODRIVER=dummy CPUAP_RTCFIX=1 ./mame pcmx2 -video none -nothrottle -oslog \
--   -autoboot_script docs/storager-lle/ldcheck.lua -flop siemens/set1/mx2-001.imd -hard1 <img>

local SC, STOP = ":slot1:storager:cpu", 30.0
local cpu, sp, armed = nil, nil, false
local taps = {}
local cmds = {}          -- one row per 0x95 command
local cur = nil
local lastcmd = 0

local function now() local ok,t = pcall(function() return manager.machine.time:as_double() end); return ok and t or 0 end
local function B(a) return sp:read_u8(a & 0xffff) end
local function W(a) return sp:read_u16(a & 0xffff) end

local function arm()
  if armed then return true end
  local ok, d = pcall(function() return manager.machine.devices[SC] end)
  if not ok or not d then return false end
  armed = true; cpu = d; sp = d.spaces["program"]
  -- E000-E01E block writes = the program load burst
  for _, base in ipairs({ 0x00e000, 0xffe000 }) do
    taps[#taps+1] = sp:install_write_tap(base, base + 0x1f, "e000", function(off, data, mask)
      if cur then
        local o = (off - base)
        if o >= 2 then cur.load = cur.load + 1 end   -- offset>=1 word: only the block copy writes these
      end
    end)
  end
  for _, base in ipairs({ 0x00c800, 0xffc800 }) do
    taps[#taps+1] = sp:install_write_tap(base, base + 0x1ff, "c800", function()
      if cur then cur.c800 = cur.c800 + 1 end
    end)
  end
  -- record traffic: the mark buffer and the two record ISRs
  taps[#taps+1] = sp:install_write_tap(0x7dac, 0x7db1, "mark", function()
    if cur then cur.marks = cur.marks + 1 end
  end)
  local osp = d.spaces["decrypted_opcodes"] or d.spaces["opcodes"] or sp
  taps[#taps+1] = sp:install_read_tap(0x7654, 0x76bf, "ledger", function()
    if cur then cur.ledger = cur.ledger + 1 end
  end)
  print("ldcheck armed"); io.flush(); return true
end

emu.register_periodic(function()
  if not arm() then return end
  local c = B(0x71f0)
  if c ~= lastcmd then
    if c == 0x95 then
      cur = { t = now(), load = 0, c800 = 0, marks = 0, ledger = 0 }
      cmds[#cmds+1] = cur
    elseif cur then
      cur = nil
    end
    lastcmd = c
  end
  if now() >= STOP then
    print("=== program-retention acceptance test ===")
    print("  read  t=start    E000 block words   C800 writes   mark writes   ledger reads   verdict")
    for i, r in ipairs(cmds) do
      local verdict
      if r.load == 0 and (r.marks > 0 or r.ledger > 0) then verdict = "RETAINED: records with NO load  <-- PASS"
      elseif r.load > 0 then verdict = "loaded its own program (op18 ran)"
      else verdict = "SILENCE: no load AND no records  <-- FAIL" end
      print(string.format("  #%d   %8.4f   %6d            %6d        %6d        %6d       %s",
        i, r.t, r.load, r.c800, r.marks, r.ledger, verdict))
    end
    io.flush(); manager.machine:exit()
  end
end)
