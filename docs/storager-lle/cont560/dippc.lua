-- DIP force + storager-CPU PC census.  The firmware goes silent after HOST POST on the
-- failing command; a PC histogram says WHERE it is idling, which no host-side tap can.
for tag,port in pairs(manager.machine.ioport.ports) do
  if tag:find("S7") then for f,fl in pairs(port.fields) do
    if f:find("Boot") then fl.user_value=0x00 end end end end
print("DIP: Disk forced [control]")

local cpu = nil
local hist = {}
local n = 0
local nextdump = 0

emu.register_periodic(function ()
  if not cpu then
    cpu = manager.machine.devices[":slot1:storager:cpu"]
    if not cpu then print("PCCENSUS: no cpu") return end
  end
  local t = manager.machine.time.seconds
  if t < 40 then return end
  local pc = cpu.state["PC"].value
  hist[pc] = (hist[pc] or 0) + 1
  n = n + 1
  if t >= nextdump then
    nextdump = t + 1
    local arr = {}
    for k,v in pairs(hist) do arr[#arr+1] = {k,v} end
    table.sort(arr, function(a,b) return a[2] > b[2] end)
    local s = ""
    for i = 1, math.min(8, #arr) do
      s = s .. string.format(" %06x:%d", arr[i][1], arr[i][2])
    end
    print(string.format("PCCENSUS t=%.2f n=%d%s", t, n, s))
    hist = {}
    n = 0
  end
end)
