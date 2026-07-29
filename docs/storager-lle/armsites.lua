-- armsites.lua - WHICH of the twelve C800 arm sites fires, on FM vs MFM?
--
-- The arm is a REPLICATED IDIOM, not a shared routine: `move.w $741e.w,$c800.w` appears at twelve
-- sites, each path carrying its own copy.  ($92C4 is one of twelve, not "the only copy" - and $92B4
-- is not a called routine despite its movem.l: its only predecessor is $2996 bra $92b4.)
--
-- The FM trace showed C800+000 written 306 times; on the MFM read C800[0] never moves off $2380.
-- So one or more of the twelve fires on FM and none fires on MFM.  Count all twelve - no inference.
--
-- Sites: 28f4 2920 293c 7fd0 7fee 8022 86a2 86f6 8754 926c 9284 92c4
-- Prime suspects are $7FD0/$7FEE/$8022 - the $7F5A->$8018 IRQ5 chain, with $801C/$8022 at the
-- data-done handler where a per-record arm belongs - but the count settles it without guessing.
--
-- Density is read per capture, so rows are labelled by the drive's actual cylinder.
--
-- USAGE: SDL_VIDEODRIVER=dummy CPUAP_RTCFIX=1 ./mame pcmx2 -video none -nothrottle -oslog \
--   -autoboot_script docs/storager-lle/armsites.lua -flop siemens/set1/mx2-001.imd

local SC, STOP = ":slot1:storager:cpu", 12.0
local cpu, sp, osp, armed = nil, nil, nil, false
local taps = {}
local SITES = { 0x28f4, 0x2920, 0x293c, 0x7fd0, 0x7fee, 0x8022,
                0x86a2, 0x86f6, 0x8754, 0x926c, 0x9284, 0x92c4 }
local hit = {}          -- [site] = { fm = n, mfm = n }
local fdd = nil

local function now() local ok,t = pcall(function() return manager.machine.time:as_double() end); return ok and t or 0 end
local function B(a) return sp:read_u8(a & 0xffff) end
local function W(a) return sp:read_u16(a & 0xffff) end

-- density is UIB+$12 bit2, read from the live UIB pointer [$799a]
local function is_mfm()
  local uib = W(0x799a)
  if uib < 0x4000 or uib >= 0x8000 then return false end
  return (B(uib + 0x12) & 0x04) ~= 0
end

local function arm()
  if armed then return true end
  local ok, d = pcall(function() return manager.machine.devices[SC] end)
  if not ok or not d then return false end
  armed = true; cpu = d; sp = d.spaces["program"]
  osp = d.spaces["decrypted_opcodes"] or d.spaces["opcodes"] or sp
  for _, a in ipairs(SITES) do
    hit[a] = { fm = 0, mfm = 0 }
    taps[#taps+1] = osp:install_read_tap(a, a + 1, string.format("s%04x", a), function()
      local h = hit[a]
      if is_mfm() then h.mfm = h.mfm + 1 else h.fm = h.fm + 1 end
    end)
  end
  print("armsites armed"); io.flush(); return true
end

emu.register_periodic(function()
  if not arm() then return end
  if now() >= STOP then
    print("=== C800 arm sites: which fires, FM vs MFM ===")
    print("   site     FM      MFM")
    local tf, tm = 0, 0
    for _, a in ipairs(SITES) do
      local h = hit[a]
      tf = tf + h.fm; tm = tm + h.mfm
      if h.fm > 0 or h.mfm > 0 then
        print(string.format("  $%04x  %6d  %7d%s", a, h.fm, h.mfm,
          (h.fm > 0 and h.mfm == 0) and "   <-- FM only: the divergence" or ""))
      end
    end
    print(string.format("  TOTAL  %6d  %7d", tf, tm))
    for _, a in ipairs(SITES) do
      if hit[a].fm == 0 and hit[a].mfm == 0 then io.write(string.format("  (silent: $%04x)\n", a)) end
    end
    io.flush(); manager.machine:exit()
  end
end)
