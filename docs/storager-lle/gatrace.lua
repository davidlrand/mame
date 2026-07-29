-- gatrace.lua - a human-readable trace of EVERY gate-array access during the 8-sector read (cmd 0x95),
-- grouped by the micro-op that was executing, with timestamps.
--
-- The ladder walker dispatches at $15BA jsr (A1) after $15B4 move.w (A0,D0.w),D0, so D0 at $15B4 is the
-- byte offset into the $192 jump table - that is the micro-op currently running.  Inline ops (0 = end,
-- $36 = park) never dispatch, so they appear as phase changes rather than sections.
--
-- MEASUREMENT RULES (see the storager-measurement-discipline memory):
--   * NO SRAM reads inside any tap.  storager.cpp taps $4000-$7FFF for its DMA snoop, and inside a CPU
--     access tap the CPU is the executing device, so a Lua read there corrupts m_term_bit0 and the
--     board stops arming the read.  Everything is recorded raw and resolved after the run.
--   * BOTH address bases.  Firmware reaches the GA by 68000 short-absolute (sign-extended -> $FFxxxx)
--     for some sites and long-absolute ($00xxxx) for others - op28/op42 use the latter for the PITs and
--     $9290 uses the former for E802.  Tapping one base silently misses half the traffic.
--   * Consecutive identical (op, addr, rw, data) accesses are COLLAPSED with a repeat count.  op28
--     polls F000 ~14600 times; printing each is noise, not information.  Counts are exact.
--
-- USAGE: SDL_VIDEODRIVER=dummy CPUAP_RTCFIX=1 ./mame pcmx2 -video none -nothrottle \
--          -autoboot_script docs/storager-lle/gatrace.lua -flop siemens/set1/mx2-001.imd
-- OUTPUT: docs/storager-lle/ga-trace-8sector.txt

local SC   = ":slot1:storager:cpu"
local STOP = 30.0
local OUT  = "docs/storager-lle/ga-trace-8sector.txt"
local MAXE = 40000

local cpu, sp, osp, armed, wrote = nil, nil, nil, false, false
local taps = {}
local ev   = {}          -- {t, op, addr, rw, data, n}
local cur_ctx = "(pre-command)"
local seq = 0            -- bumped on every context change, so repeated visits stay distinct
local g_on = false
local t_on = 0
local announced = false
local ladder = {}        -- every dispatch: {t, op} - including ops that touch no GA register

-- micro-op names (offsets into the $192 jump table)
local OPNAME = {
  [0x18]="op18  arm the field program (C800 bit-positions + E000 program)",
  [0x24]="op24  settle/spin-up arm",
  [0x28]="op28  seek + wait for spin-up ([$7a36]); sets [$798e]",
  [0x32]="op32  host-status handshake ($4EC2)",
  [0x42]="op42  guard/wait; UIB+$12 bit7 bypasses the timer",
  [0x48]="op48  ($73FA)",
  [0x4a]="op4A  drain request: arm [$7b10], fork on [$7968]",
  [0x54]="op54  copy the watch template ($66A/$6A6) + patch rec2's watch",
  [0x56]="op56  parse; sets [$796a] from UIB+$11 bit0 + cmd",
  [0x58]="op58  clamp the transfer count -> [$7abc]/[$7abe]",
}

local function now() local ok,t = pcall(function() return manager.machine.time:as_double() end); return ok and t or 0 end
local function W(a) return sp:read_u16(a & 0xffff) end
local function B(a) return sp:read_u8(a & 0xffff) end

-- the gate array's register map, for labelling after the run
local function reg(a)
  a = a & 0xffff
  if a >= 0xc000 and a <= 0xc7ff then return string.format("C000+%03x", a - 0xc000), "host address up-counter"
  elseif a >= 0xc800 and a <= 0xc9ff then return string.format("C800+%03x", a - 0xc800), "field/chunk parameters"
  elseif a == 0xd000 or a == 0xd001 then return "D000    ", "local DMA address latch (word addr)"
  elseif a == 0xd800 or a == 0xd801 then return "D800    ", "mark/ID buffer latch (word addr)"
  elseif a >= 0xe000 and a <= 0xe01f then return string.format("E000+%02x", a - 0xe000), "field step program"
  elseif a >= 0xe020 and a <= 0xe7ff then return string.format("E0xx+%03x", a - 0xe000), "channel window"
  elseif a == 0xe800 or a == 0xe801 then return "E800    ", "control: bit12 = host DMA request"
  elseif a == 0xe802 or a == 0xe803 then return "E802    ", "control: bit11 = per-record re-arm"
  elseif a == 0xe804 or a == 0xe805 then return "E804    ", "drive/head select (bits 8-11 one's-comp)"
  elseif a == 0xe806 or a == 0xe807 then return "E806    ", "control"
  elseif a >= 0xf000 and a <= 0xf001 then return "F000    ", "status: b1 seek, b4 index, b5/7 ready, b11 tick, b12 DMA match, b13 trk0"
  end
  return string.format("%04x    ", a), ""
end

local function rec(addr, rw, data)
  if not g_on or #ev >= MAXE then return end
  local last = ev[#ev]
  if last and last.seq == seq and last.addr == addr and last.rw == rw and last.data == data then
    last.n = last.n + 1
    return
  end
  ev[#ev+1] = { t = now(), ctx = cur_ctx, seq = seq, addr = addr, rw = rw, data = data, n = 1 }
end

-- a context switch: the ladder dispatching a micro-op, or an interrupt handler being entered
local function ctx(name)
  if name ~= cur_ctx then seq = seq + 1; cur_ctx = name end
end

local function arm()
  if armed then return true end
  local ok, d = pcall(function() return manager.machine.devices[SC] end)
  if not ok or not d then return false end
  armed = true; cpu = d; sp = d.spaces["program"]
  osp = d.spaces["decrypted_opcodes"] or d.spaces["opcodes"] or sp

  -- which micro-op is running.  D0 is a REGISTER read - no SRAM touched.
  taps[#taps+1] = osp:install_read_tap(0x15b4, 0x15b5, "ladder", function()
    local ok2, v = pcall(function() return cpu.state["D0"].value end)
    local o = ok2 and (v & 0xff) or -1
    if g_on and #ladder < 400 then
      local L = ladder[#ladder]
      if L and L.op == o then L.n = L.n + 1
      else ladder[#ladder+1] = { t = now(), op = o, n = 1, first = #ev + 1 } end
    end
    ctx(string.format("LADDER  micro-op $%02X  %s", o, OPNAME[o] or ""))
  end)

  -- Interrupt handlers.  All taps are placed DEEP (>=8 bytes past the entry) because the stubs are
  -- reached by branch and the 68000 prefetches past branches - a tap on the first instruction reports
  -- services that never ran.  These are where the gate array's per-record stimulus is answered.
  local isr = {
    [0x89fa] = "IRQ6    ID address mark -> $89F2 verify",
    [0x92be] = "IRQ6    ID address mark -> $92B4 stake (MFM path)",
    [0x7bb0] = "IRQ5 #1 data field armed -> $7BA8 setup",
    [0x8022] = "IRQ5 #2 data field done  -> $8018",
    [0x3c12] = "IRQ4    host transfer complete -> $3BFE",
    [0x2b60] = "IRQ1    system tick -> $2B58",
  }
  for a, nm in pairs(isr) do
    taps[#taps+1] = osp:install_read_tap(a, a + 1, "isr", function() ctx(nm) end)
  end

  -- every GA window, at BOTH bases, reads and writes
  local wins = {
    {0xc000,0xc7ff},{0xc800,0xc9ff},{0xd000,0xd001},{0xd800,0xd801},
    {0xe000,0xe7ff},{0xe800,0xe807},{0xf000,0xf001},
  }
  for _, w in ipairs(wins) do
    for _, base in ipairs({0x000000, 0xff0000}) do
      local lo, hi = base + w[1], base + w[2]
      taps[#taps+1] = sp:install_write_tap(lo, hi, "gaw", function(off, data, mask)
        rec(off & 0xffff, "W", data & 0xffff)
      end)
      taps[#taps+1] = sp:install_read_tap(lo, hi, "gar", function(off, data, mask)
        rec(off & 0xffff, "R", data & 0xffff)
      end)
    end
  end
  -- EXACT IOCB gate.  A periodic poll of $71F0 lagged the command's arrival by ~4ms and lost the
  -- first ladder dispatches (op24 was missing from the summary as a result).  A write tap fires on
  -- the instant the command byte lands.  No SRAM READ here - only the tap's own data argument.
  taps[#taps+1] = sp:install_write_tap(0x71f0, 0x71f1, "iocb", function(off, data, mask)
    if not g_on and ((mask == 0xffff and ((data >> 8) & 0xff) == 0x95) or
                     (mask == 0xff00 and ((data >> 8) & 0xff) == 0x95)) then
      g_on = true; t_on = now()
    end
  end)
  print("gatrace armed"); io.flush()
  return true
end

local OPNAME_UNUSED = {
  [0x18]="op18  arm the field program (C800 bit-positions + E000 program)",
  [0x24]="op24  settle/spin-up arm",
  [0x28]="op28  seek + wait for spin-up ([$7a36]); sets [$798e]",
  [0x2a]="op2A",
  [0x32]="op32  host-status handshake ($4EC2)",
  [0x42]="op42  guard/wait; UIB+$12 bit7 bypasses the timer",
  [0x48]="op48  ($73FA)",
  [0x4a]="op4A  drain request: arm [$7b10], fork on [$7968]",
  [0x54]="op54  copy the watch template ($66A/$6A6) + patch rec2's watch",
  [0x56]="op56  parse; sets [$796a] from UIB+$11 bit0 + cmd",
  [0x58]="op58  clamp the transfer count -> [$7abc]/[$7abe]",
}

emu.register_periodic(function()
  local t = now()
  if not arm() then return end
  if g_on and not announced then
    announced = true
    print(string.format("gatrace: IOCB 0x95 written at t=%.5f", t_on))
  end
  if t >= STOP and not wrote then
    wrote = true
    local f = io.open(OUT, "w")
    if not f then print("gatrace: cannot open " .. OUT); manager.machine:exit(); return end
    f:write("Storager gate-array access trace - cmd 0x95, 8-sector read\n")
    f:write("Grouped by the micro-op executing at the time (ladder dispatch $15B4/$15BA).\n")
    f:write("Consecutive identical accesses are collapsed as 'xN'; counts are exact.\n")
    f:write(string.format("IOCB 0x95 first seen at t=%.5f; everything below is within its lifetime.\n", t_on))
    f:write(string.rep("=", 100) .. "\n\n")
    local last_seq, total = nil, 0
    for _, e in ipairs(ev) do
      total = total + e.n
      if e.seq ~= last_seq then
        last_seq = e.seq
        f:write(string.format("\n%s\n--- t=%9.5f  %s\n%s\n",
          string.rep("-", 100), e.t, e.ctx, string.rep("-", 100)))
      end
      local r, note = reg(e.addr)
      f:write(string.format("  %9.5f  %s %s %s %04x%s%s\n",
        e.t, r, e.rw, e.rw == "W" and "<=" or "->", e.data,
        e.n > 1 and string.format("   x%d", e.n) or "",
        note ~= "" and ("   ; " .. note) or ""))
    end
    -- the ladder as executed, with how many GA accesses each op made.  Ops that touch no gate-array
    -- register (op54's template copy, op4A's drain, op42's guard on the FM path) show 0 - which is
    -- itself the point: the read's stimulus is almost entirely op18 plus the per-record ISRs.
    f:write(string.format("\n%s\nLADDER AS EXECUTED\n%s\n",
      string.rep("=", 100), string.rep("=", 100)))
    f:write("  'GA accesses in window' counts every access between this dispatch and the next - which\n")
    f:write("  INCLUDES interrupt traffic, since the record ISRs run while the ladder sits on an op.\n")
    f:write("  op42's huge count is the whole read: the ladder parks there while the records arrive.\n\n")
    for i, L in ipairs(ladder) do
      local upto = (ladder[i+1] and ladder[i+1].first or (#ev + 1)) - 1
      local acc = 0
      for j = L.first, math.min(upto, #ev) do acc = acc + ev[j].n end
      f:write(string.format("  %9.5f  micro-op $%02X %-62s dispatches=%-6d GA accesses in window=%d\n",
        L.t, L.op, (OPNAME[L.op] or ""):sub(1, 62), L.n, acc))
    end
    f:write(string.format("\n%s\ntotal accesses: %d, in %d distinct runs\n", string.rep("=", 100), total, #ev))
    f:close()
    print(string.format("gatrace: wrote %s (%d accesses, %d lines)", OUT, total, #ev))
    io.flush(); manager.machine:exit()
  end
end)
