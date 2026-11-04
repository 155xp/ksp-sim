//! ksp-guidance: return-to-launch-site booster guidance on top of ksp-core (read-only).
//!
//! Every quantity the guidance uses is read from the vessel spec / sim state at run time (mass,
//! CoM, inertia, thrust and Isp at the current pressure, throttle limits, gimbal geometry, reaction
//! wheel torque, drag cubes), so the same code flies any booster `craft` can build.
pub mod craft;
pub mod ingame;
pub mod partlib;
pub mod nav;
pub mod predict;
pub mod attitude;
pub mod gfold;
pub mod mission;
pub mod landing;
pub mod fleet;
pub mod terminal;
