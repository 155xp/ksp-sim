// Tunables for booster.ks (edited between in-game runs).
global MECO_PROP is 1.8.      // t of booster propellant left at MECO
global PITCH_V0 is 50.        // m/s before the pitch program starts
global PITCH_H0 is 300.       // m, pitch program start height
global PITCH_H1 is 9000.      // m over which pitch goes 90 -> PITCH_END
global PITCH_END is 70.
global WARP_OK is true.
global WARP_ASCENT_H is 3000.
global WARP_COAST_H is 20000.
global WARP_AERO_H is 8000.
global AIM_LONG is 0.         // m beyond the pad along the return direction (boostback aim)
global BB_TAU is 1.5.         // s, boostback error time constant
global BB_TOL is 20.          // m, boostback cutoff
global KD_PRIOR is 1.8e-3.   // airbrakes deployed (flight 1 learned 1.8e-3)
global FLIP_Q is 1.5.         // kPa, start the flip below this dynamic pressure
global CORR_H_MAX is 14000.   // m, descent correction burns allowed between these heights
global CORR_H_MIN is 3000.
global CORR_ON is 40.         // m predicted miss to start a correction burn
global CORR_OFF is 8.         // m to stop it
global CORR_TILT is 30.       // deg off vertical while correcting
global CORR_TAU is 2.         // s
global AOA_MAX is 15.         // deg
global AOA_K is 0.01.         // deg per m of predicted miss
global AERO_SIGN is 1.        // +1: lean the top toward the miss (body lift pushes back)
global IGN_F is 0.5.          // fraction of max accel planned for the stop
global IGN_LAG is 0.4.        // s
global H_HOLD is 15.          // m above touchdown height where the terminal segment starts
global V_HOLD is 3.           // m/s at H_HOLD
global V_TD is 1.0.           // m/s touchdown rate
global TGO_MIN is 2.
global TILT_MAX is 15.
global TILT_TERM is 8.
global TERM_KP is 0.25.
global TERM_KD is 1.0.
global LEGS_H is 600.
global CAM_LAND_H is 2500.
global CAM_PAD is "ground -0.1087 -74.5617 85 0".
global CAM_ASCENT is "chase 200 -5 45 4".
global CAM_SEP is "chase 160 10 30 2".
global CAM_FLIP is "chase 180 25 120 4 4".
global CAM_COAST is "chase 200 35 70 4 2".
global CAM_AERO is "chase 270 65 40 4".
global CAM_LAND is "ground -0.1033 -74.5627 80.5 3".
