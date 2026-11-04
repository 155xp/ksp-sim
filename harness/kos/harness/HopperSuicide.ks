// Vertical climb on full throttle for 6 s, coast to apex (~1.5 km), suicide-burn landing on the pad.
runoncepath("0:/harness/lib.ks").
prep().
lock steering to up.
lock throttle to 1.
wait 6.
lock throttle to 0.
wait until ship:verticalspeed < 0.
land_from_fall().
finish(30).
