// Upper stage: idle until the booster is gone, then light the Terrier and climb away prograde.
function has_booster {
  for e in ship:engines { if e:name:startswith("liquidEngine2") return true. }
  return false.
}
wait until not has_booster().
wait 2.5.
sas off.
lock steering to lookdirup(ship:velocity:surface, ship:facing:topvector).
for e in ship:engines { e:activate(). }
lock throttle to 1.
wait until ship:liquidfuel < 0.5.
lock throttle to 0.
wait until false.
