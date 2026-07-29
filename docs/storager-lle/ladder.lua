-- ladder.lua - WHO OWNS $7250 when the builder has already run?
--
-- Established: $5FC0's both-zero guard PASSES on every command (node+$a/$b match the host's bytes,
-- and the CPUAP's IOCB is known good from the HLE), so the read's builder RAN AND EMITTED.  Yet the
-- walker later finds `001c 0022 0016 0000 | 0018 0054 004a 0042` at $7250 - another command's ladder,
-- intact and zero-terminated - takes the op00 path on that zero ($158C move.w (A1),D0 / $158E bne:
-- op00 is detected by (A1) reading ZERO, not by position), stamps node+$26 = $000C and exits with
-- op18/op54/op4A/op42 unexecuted behind it.  That is the ~70ms "success" that captures nothing.
--
-- TAP HYGIENE: ONE write tap owns $7100-$72FF and dispatches internally by address.  A previous
-- attempt installed a build-side tap on $7220-$7270 while a $7100-$72FF tap already covered it;
-- overlapping write taps do not both fire, so the build side silently recorded nothing.
--
-- Per 0x95 command this reports:
--   BUILD  - every word the builder emitted into the ladder buffer, in order, with its address
--   AFTER  - the list image once the build settles
--   OP00   - A1 at the stamp, and the list image AT THAT MOMENT
-- Three outcomes, all decisive:
--   builder emitted 1c 22 16 00        -> the READ's builder is emitting the wrong ladder
--   emitted 24 28 56 58, image intact  -> something overwrote it between build and walk
--   emitted 24 28 56 58, walk elsewhere-> the walker is reading a different buffer than was built
--
-- USAGE: SDL_VIDEODRIVER=dummy CPUAP_RTCFIX=1 ./mame pcmx2 -video none -nothrottle -oslog \
--   -autoboot_script docs/storager-lle/ladder.lua -flop siemens/set1/mx2-001.imd -hard1 <img>

local SC, STOP = ":slot1:storager:cpu", 45.0
local LADLO, LADHI = 0x722c, 0x7270     -- the two ladder buffers ONLY.  $7228 is not one: it is
                                        -- hammered with $00fe thousands of times and floods the capture.

local cpu, sp, armed = nil, nil, false
local taps, rows, cur = {}, {}, nil
local pending = {}
local lastcmd, nodebase, livecmd = -1, 0x71f0, 0

local function now() local ok,t = pcall(function() return manager.machine.time:as_double() end); return ok and t or 0 end
local function B(a) return sp:read_u8(a & 0xffff) end
local function W(a) return sp:read_u16(a & 0xffff) end
local function R(n) local ok,v = pcall(function() return cpu.state[n].value end); return ok and v or 0 end

local function image(base)
  local t = {}
  for i = 0, 11 do t[#t+1] = string.format("%04x", W((base + i * 2) & 0xffff)) end
  return table.concat(t, " ")
end

local function arm()
  if armed then return true end
  local ok, d = pcall(function() return manager.machine.devices[SC] end)
  if not ok or not d then return false end
  armed = true; cpu = d; sp = d.spaces["program"]
  -- ONE tap. No SRAM reads inside it: registers only, snapshots deferred to the periodic.
  taps[#taps+1] = sp:install_write_tap(0x7100, 0x72ff, "space", function(off, data, mask)
    if not cur then return end
    local a, v = off & 0xffff, data & 0xffff
    if a >= LADLO and a <= LADHI then
      if #cur.build < 16 then cur.build[#cur.build + 1] = string.format("%04x:%04x", a, v) end
      cur.nbuild = cur.nbuild + 1
      cur.tbuild = now()
      return
    end
    -- node+$26 <- $000C has TWO writers: $15A0 move.w d1,($26,A2) is the WALKER's terminator, and
    -- $17F8 move.w #$c,($26,A0) is the phase-0x0C path.  Same value, same width - the only separator
    -- is which base register holds the node.  Earlier A1 captures ($9188/$7664/$79F6) were all $17F8.
    if a == ((nodebase + 0x26) & 0xffff) and mask == 0xffff and v == 0x000c then
      local a2 = R("A2") & 0xffff
      local walker = (a2 == nodebase)
      if walker and not cur.op00 then
        cur.op00 = { a1 = R("A1") & 0xffff, a2 = a2, t = now() }
        pending[#pending + 1] = cur
      elseif not walker then
        cur.phase0c = (cur.phase0c or 0) + 1
      end
    end
  end)
  print("ladder armed"); io.flush(); return true
end

emu.register_periodic(function()
  if not arm() then return end
  local node = W(0x71bc)
  if node >= 0x4000 and node < 0x8000 then nodebase = node end
  livecmd = B(nodebase)

  while #pending > 0 do
    local r = table.remove(pending, 1)
    if r.op00 and not r.op00.img then r.op00.img = image(r.op00.a1 - 6) end
  end
  -- once the build has settled (no writes for a sample), latch the resulting image
  if cur and cur.nbuild > 0 and not cur.after and cur.tbuild and (now() - cur.tbuild) > 0.002 then
    cur.after = image(cur.base or 0x7250)
  end

  if livecmd ~= lastcmd then
    if livecmd == 0x95 then
      cur = { t0 = now(), build = {}, nbuild = 0, base = nil }
      rows[#rows + 1] = cur
    else cur = nil end
    lastcmd = livecmd
  end
  if cur and cur.base == nil and #cur.build > 0 then
    cur.base = tonumber(cur.build[1]:sub(1, 4), 16)
  end

  if now() >= STOP then
    print("=== who owns $7250 ===")
    print("  expected read ladder from $5FC0: 0024 0028 0056 0058 [1a] 0018 0054 004a 0042 0036 0000")
    for i, r in ipairs(rows) do
      print(string.format("\n  read #%d  t=%.4f  builder emitted %d word(s)%s",
        i, r.t0, r.nbuild, (r.nbuild == 0) and "   *** BUILDER EMITTED NOTHING ***" or ""))
      if #r.build > 0 then print("     BUILD: " .. table.concat(r.build, " ")) end
      if r.after then print("     AFTER: " .. r.after) end
      if r.phase0c then print(string.format("     ($17F8 phase-0x0C writes: %d)", r.phase0c)) end
      if r.op00 then
        print(string.format("     WALKER op00: A1=%04x (A2=%04x=node) t=%.4f", r.op00.a1, r.op00.a2, r.op00.t))
        if r.op00.img then print("     IMAGE: " .. r.op00.img .. "   (from A1-6)") end
      else
        print(string.format("     WALKER op00: none%s",
          r.phase0c and string.format("  (%d phase-0x0C writes from $17F8, not the walker)", r.phase0c) or ""))
      end
    end
    io.flush(); manager.machine:exit()
  end
end)
