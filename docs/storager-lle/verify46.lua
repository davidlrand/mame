-- verify46.lua - is the MFM record REJECTED, or never judged at all?
--
-- $89FA move.b ($12,A0),D0   ; A0 = [$799a], the UIB
-- $89FE btst #$1,D0          ; bit ONE - the same bit1-vs-bit2 divergence flagged at $7C5C
-- $8A02 beq $8A38            ; clear -> SKIP the verify entirely
-- $8A14 lea $7dac,A0         ; the verify body (only reached if $8A02 falls through)
-- $8A38 clr.w $7950 / $8A3C subq.w #1,$79a4 / $8A40 bne $8AB8   ; drop unless the counter expires
--
-- [$79a4] is reloaded from [$79a2] ($7E0E, $7A06, $7B3E, $84D0).  A 4-of-16 accept rate would be
-- [$79a2] = 4 - a DIVIDER, not a verify failure.
--
-- PREDICTION (stated before the run): $8A02 is taken, $8A14 never fires, [$79a2] = 4.
-- If $8A14 DOES fire, the verify runs and the reject is the missing $FF at record byte +4 - the
-- firmware's MFM record is TEN bytes (A1 A1 A1 FE FF C C H R N), the model writes eight.
--
-- SRAM reads deferred to the periodic; only counters touched inside taps.
--
-- USAGE: SDL_VIDEODRIVER=dummy CPUAP_RTCFIX=1 ./mame pcmx2 -video none -nothrottle -oslog \
--   -autoboot_script docs/storager-lle/verify46.lua -flop siemens/set1/mx2-001.imd

local SC, STOP = ":slot1:storager:cpu", 12.0
local cpu, sp, osp, armed = nil, nil, nil, false
local taps, ev, pend = {}, {}, {}
local nverify, ndrop = 0, 0

local function now() local ok,t = pcall(function() return manager.machine.time:as_double() end); return ok and t or 0 end
local function B(a) return sp:read_u8(a & 0xffff) end
local function W(a) return sp:read_u16(a & 0xffff) end

local function arm()
  if armed then return true end
  local ok, d = pcall(function() return manager.machine.devices[SC] end)
  if not ok or not d then return false end
  armed = true; cpu = d; sp = d.spaces["program"]
  osp = d.spaces["decrypted_opcodes"] or d.spaces["opcodes"] or sp
  taps[#taps+1] = osp:install_read_tap(0x8a14, 0x8a15, "verify", function() nverify = nverify + 1 end)
  taps[#taps+1] = osp:install_read_tap(0x8a3c, 0x8a3d, "drop", function()
    ndrop = ndrop + 1
    pend[#pend+1] = { t = now(), n = ndrop }
  end)
  print("verify46 armed"); io.flush(); return true
end

emu.register_periodic(function()
  if not arm() then return end
  while #pend > 0 do
    local p = table.remove(pend, 1)
    if #ev < 30 then
      local uib = W(0x799a)
      local d12 = (uib >= 0x4000 and uib < 0x8000) and B(uib + 0x12) or 0
      ev[#ev+1] = string.format("%9.4f  drop#%-3d [79a2]=%04x [79a4]=%04x | UIB+12=%02x bit1=%d bit2=%d",
        p.t, p.n, W(0x79a2), W(0x79a4), d12, (d12 >> 1) & 1, (d12 >> 2) & 1)
    end
  end
  if now() >= STOP then
    print("=== verify vs divider ===")
    for _, l in ipairs(ev) do print(l) end
    print(string.format("=== $8A14 verify body: %d | $8A3C countdown: %d", nverify, ndrop))
    print(string.format("    final [79a2]=%04x [79a4]=%04x", W(0x79a2), W(0x79a4)))
    io.flush(); manager.machine:exit()
  end
end)
