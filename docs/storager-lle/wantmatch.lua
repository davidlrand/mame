-- wantmatch.lua - why does the firmware accept every record instead of rejecting the unwanted ones?
--
-- The host buffer gets the label 6 sectors late: sectors are placed by ARRIVAL, not identity, so
-- $0FC0DD holds a rotationally-earlier sector and the firmware finds no VOL1SINIX0 -> "no sys-floppy".
-- The payload itself is byte-correct, so this is upstream of the whole completion campaign.
--
-- The match:
--   $7D02 clr.l D0 / $7D04 movea.w ($ce,A1),A0 / $7D08 move.b (A0),D0   ; D0 = recovered sector byte
--   $7E50 cmpi.b #$ff,D0 / beq $7D4A                                    ; reject
--   $7E58 cmp.w $7428,D0 / bne $7D4A                                    ; the want-list match
-- Three ways to get universal acceptance, all separated here:
--   $7D4A count 0 with 8 accepts -> the compare passes; D0 vs [$7428] says which side is wrong
--   D0 CONSTANT across records   -> POSPTR aims at the wrong record offset (density layout: R sits at
--                                   +3 under FM, +6 under MFM; byte 0 would read $FE every time)
--   D0 varies but [$7428] follows -> the expected value is written from the arrival ($992E et al)
--
-- Registers are read at the tap (safe); SRAM reads are DEFERRED to the periodic - reading SRAM inside
-- a CPU tap trips the model's own $4000-$7FFF snoop.
--
-- USAGE: SDL_VIDEODRIVER=dummy CPUAP_RTCFIX=1 ./mame pcmx2 -video none -nothrottle -oslog \
--   -autoboot_script docs/storager-lle/wantmatch.lua -flop siemens/set1/mx2-001.imd

local SC, STOP = ":slot1:storager:cpu", 14.0
local cpu, sp, osp, armed = nil, nil, nil, false
local taps, ev, pend = {}, {}, {}
local nrej, nrec = 0, 0

local function now() local ok,t = pcall(function() return manager.machine.time:as_double() end); return ok and t or 0 end
local function B(a) return sp:read_u8(a & 0xffff) end
local function W(a) return sp:read_u16(a & 0xffff) end
local function R(n) local ok,v = pcall(function() return cpu.state[n].value end); return ok and v or 0 end
local function E(s) if #ev < 120 then ev[#ev+1] = s end end

local function arm()
  if armed then return true end
  local ok, d = pcall(function() return manager.machine.devices[SC] end)
  if not ok or not d then return false end
  armed = true; cpu = d; sp = d.spaces["program"]
  osp = d.spaces["decrypted_opcodes"] or d.spaces["opcodes"] or sp
  -- the compare site: D0 and A1 are registers, safe to read here
  taps[#taps+1] = osp:install_read_tap(0x7e50, 0x7e51, "cmp", function()
    nrec = nrec + 1
    pend[#pend+1] = { t = now(), n = nrec, d0 = R("D0") & 0xff, a1 = R("A1") & 0xffff }
  end)
  taps[#taps+1] = osp:install_read_tap(0x7d4a, 0x7d4b, "rej", function() nrej = nrej + 1 end)
  -- who writes the expected value, and what
  local nexp = 0
  taps[#taps+1] = sp:install_write_tap(0x7428, 0x7429, "exp", function(off, data, mask)
    nexp = nexp + 1
    if nexp <= 4 then E(string.format("   [7428] <= %04x (write #%d)", data & 0xffff, nexp)) end
  end)
  print("wantmatch armed"); io.flush(); return true
end

emu.register_periodic(function()
  if not arm() then return end
  while #pend > 0 do
    local p = table.remove(pend, 1)
    local posptr = W((p.a1 + 0xce) & 0xffff)
    local rec = {}
    for i = 0, 7 do rec[#rec+1] = string.format("%02x", B((0x7dac + i) & 0xffff)) end
    E(string.format("rec#%-2d t=%.4f  D0(recovered R)=%02x  [7428](expected)=%04x  POSPTR=%04x  markbuf: %s",
      p.n, p.t, p.d0, W(0x7428), posptr, table.concat(rec, " ")))
  end
  if now() >= STOP then
    print("=== want-list match, per record ===")
    for _, l in ipairs(ev) do print(l) end
    print(string.format("=== compares: %d   rejects ($7D4A): %d", nrec, nrej))
    print("  mark buffer is $7DAC; POSPTR should point at the recovered R within it.")
    io.flush(); manager.machine:exit()
  end
end)
