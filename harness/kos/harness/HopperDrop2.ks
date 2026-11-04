// Drop test 2: hop ~3 m up, cut, free-fall onto the pad (touchdown ~8-9 m/s, below LT-2 crashTolerance 12).
runoncepath("0:/harness/lib.ks").
prep().
lock throttle to 0.45.
wait until alt:radar > base_ralt + 3.0.
lock throttle to 0.
finish(40).
