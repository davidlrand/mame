-- chainprobe.lua — does the $92B4/$92F6 stake chain execute after each carry?
local CPU = ":slot1:storager:cpu"
local cpu = manager.machine.devices[CPU]
local sp = cpu.spaces["program"]
local ops = cpu.spaces["opcodes"] or sp
local function now() return manager.machine.time:as_double() end
local function pc() local ok,v = pcall(function() return cpu.state["PC"].value end); return ok and v or 0 end
_G.keep = {}
local n = 0
local sites = { [0x298c]="TOG298C", [0x29c0]="TOG29C0", [0x299a]="TOG299A", [0x92b4]="STAMP92B4", [0x92f6]="STAKE92F6", [0x26a8]="TRAMP26A8" }
local function mk(addr, name)
  return ops:install_read_tap(addr, addr+1, "t"..name, function(o,d,m)
    if pc() == addr and now() > 7.9 and n < 400 then n = n + 1
      print(string.format("%s @%.6f 7950=%04x 742c=%04x 7428=%04x", name, now(), sp:read_u16(0x7950), sp:read_u16(0x742c), sp:read_u16(0x7428))) end
  end)
end
local function arm() for k,t in pairs(_G.keep) do pcall(function() t:remove() end) end
  local i = 0
  for a,nm in pairs(sites) do i = i + 1; _G.keep[i] = mk(a, nm) end
end
arm()
emu.register_periodic(function()
  local ok,t = pcall(now); if not ok then return end
  arm(); io.flush()
  if t >= 12 then manager.machine:exit() end
end)
print("chainprobe armed"); io.flush()
