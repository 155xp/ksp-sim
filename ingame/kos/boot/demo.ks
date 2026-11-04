// In-game RTLS demo boot: the booster's RC-001S runs the flight, the upper stage's Mk1 pod waits
// for separation and flies away.
wait until ship:unpacked.
wait 0.5.
set config:ipu to 2000.
if core:part:name:startswith("mk1pod") {
  copypath("0:/demo/upper.ks", "1:/upper.ks").
  runpath("1:/upper.ks").
} else {
  copypath("0:/demo/booster.ks", "1:/booster.ks").
  runpath("1:/booster.ks").
}
