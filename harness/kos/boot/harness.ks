// ksp-sim harness boot: run the test script named after the vessel.
wait until ship:unpacked.
wait 1.
set config:ipu to 2000.
runpath("0:/harness/" + ship:name + ".ks").
