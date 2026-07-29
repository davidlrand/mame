-- Boot stage-1 HD, at the SINIX1 prompt mount mx2-002 and confirm; observe recognition.
local AVDC=":slot3:serad:port0:s97801:term:avdc"
local DIR=os.getenv("HOME").."/src/mame/siemens/set1/"
local function scr()
  local ok,dev=pcall(function() return manager.machine.devices[AVDC] end); if not ok or not dev then return "" end
  local s=dev.spaces["charram"]; if not s then return "" end
  local o={}
  for b=0,0x1fff,80 do for i=0,79 do local c=s:read_u8(b+i); o[#o+1]=(c>=0x20 and c<0x7f) and string.char(c) or " " end end
  return table.concat(o)
end
local function post(s) manager.machine.natkeyboard.in_use=true; manager.machine.natkeyboard:post(s) end
local flop=nil
for tag,img in pairs(manager.machine.images) do if tostring(img.instance_name)=="floppydisk" then flop=img end end
local prev, lastact, phase = "", 0, 0
emu.register_periodic(function()
  local ok,t=pcall(function() return manager.machine.time:as_double() end); if not ok then return end
  local s=scr(); local tail=s:gsub("%s+"," "):gsub("%s+$",""):sub(-110)
  if tail~=prev then prev=tail; print(string.format("@%.1f |%s", t, tail)); io.flush() end
  if t<12 or t-lastact<4 then return end
  if phase==0 and tail:find("SINIX1") and tail:find("bestaetigen") then
    local p=DIR.."mx2-002.imd"
    pcall(function() flop:load(p) end)
    print("  >> mounted mx2-002 (SINIX1): "..p); io.flush()
    post("j\n"); lastact=t; phase=1
  elseif phase>=1 and tail:find("j/n") then post("j\n"); lastact=t
  elseif phase>=1 and tail:find("Weiter mit") then post(" "); lastact=t end
  if t>=140 then manager.machine:exit() end
end)
