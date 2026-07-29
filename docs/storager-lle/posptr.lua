-- posptr.lua - what record layout does the firmware expect?
--
-- node+$c8/$ca/$cc/$ce are ABSOLUTE pointers into the capture record, so reading them names the
-- expected layout with no inference.  Two initialisations exist - $2DB4 (hardcoded $7DB1/$7DB3/$7DB4/
-- $7DBE, i.e. +5/+7/+8/+18 from $7DAC) and $2CF2 (base + ROM-table offsets) - and a previous trace
-- caught POSPTR as $7DAF on the first record and $7DB4 on later ones, so the read uses BOTH.
--
-- The model writes a PACKED record at $7DAC: [3x $A1 if MFM] $FE C H R N.  Under FM that is
-- $7DAC=FE $7DAD=C $7DAE=H $7DAF=R $7DB0=N and nothing at $7DB1-$7DB4; under MFM the fields shift by
-- three and $7DB4 is still never written.  So the sector pointer reads stale SRAM, the compare at
-- $7E58 is self-satisfying (via $7E0A re-stamping the same constant into [$7428]), every arriving
-- sector is accepted regardless of identity, and the label lands wherever it arrived - the +0x300
-- measured in the host buffer.
--
-- USAGE: SDL_VIDEODRIVER=dummy CPUAP_RTCFIX=1 ./mame pcmx2 -video none -nothrottle -oslog \
--   -autoboot_script docs/storager-lle/posptr.lua -flop siemens/set1/mx2-001.imd

local SC, STOP = ":slot1:storager:cpu", 14.0
local cpu, sp, osp, armed = nil, nil, nil, false
local taps, ev, pend = {}, {}, {}

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
  taps[#taps+1] = osp:install_read_tap(0x7d08, 0x7d09, "posptr", function()
    pend[#pend+1] = { t = now(), a1 = R("A1") & 0xffff, a0 = R("A0") & 0xffff }
  end)
  print("posptr armed"); io.flush(); return true
end

emu.register_periodic(function()
  if not arm() then return end
  while #pend > 0 do
    local p = table.remove(pend, 1)
    if #ev < 40 then
      local rec = {}
      for i = 0, 19 do rec[#rec+1] = string.format("%02x", B((0x7dac + i) & 0xffff)) end
      ev[#ev+1] = string.format(
        "t=%.4f  node=%04x  C=%04x H=%04x S=%04x X=%04x   A0(this read)=%04x -> %02x",
        p.t, p.a1, W((p.a1+0xca)&0xffff), W((p.a1+0xcc)&0xffff), W((p.a1+0xce)&0xffff),
        W((p.a1+0xc8)&0xffff), p.a0, B(p.a0))
      ev[#ev+1] = "          record $7DAC+0..19: " .. table.concat(rec, " ")
    end
  end
  if now() >= STOP then
    print("=== expected record layout, read from the pointers themselves ===")
    for _, l in ipairs(ev) do print(l) end
    print("  model writes (FM): 7dac=FE 7dad=C 7dae=H 7daf=R 7db0=N, nothing above")
    io.flush(); manager.machine:exit()
  end
end)
