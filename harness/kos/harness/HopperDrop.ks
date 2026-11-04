// Drop test 1: hop ~1 m up on 45% throttle (no steering), cut, free-fall onto the pad legs.
runoncepath("0:/harness/lib.ks").
prep().
lock throttle to 0.45.
wait until alt:radar > base_ralt + 1.0.
lock throttle to 0.
finish(40).
