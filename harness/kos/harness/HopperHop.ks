// Suborbital hop: 5 deg east of vertical at full throttle for 5 s, coast, suicide-burn landing.
runoncepath("0:/harness/lib.ks").
prep().
lock steering to heading(90, 85).
lock throttle to 1.
wait 5.
lock throttle to 0.
wait until ship:verticalspeed < 0.
lock steering to up.
land_from_fall().
finish(30).
