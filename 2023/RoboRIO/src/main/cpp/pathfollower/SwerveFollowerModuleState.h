
#pragma once

#include <wpi/SymbolExports.h>

#include "frc/geometry/Rotation2d.h"
#include "units/acceleration.h"
#include "units/angle.h"
#include "units/math.h"
#include "units/velocity.h"


// Stores a desired state of one swerve module
struct SwerveFollowerModuleState {
  //  Speed of the wheel of the module
  units::meters_per_second_t speed = 0_mps;

  //  Acceleration of the wheel of the module
  units::meters_per_second_squared_t acceleration = 0_mps_sq;

  // Angle of the module
  frc::Rotation2d angle;

//   /**
//    * Minimize the change in heading the desired swerve module state would
//    * require by potentially reversing the direction the wheel spins. If this is
//    * used with the PIDController class's continuous input functionality, the
//    * furthest a wheel will ever rotate is 90 degrees.
//    *
//    * @param desiredState The desired state.
//    * @param currentAngle The current module angle.
//    */
//   static SwerveModuleState Optimize(const SwerveModuleState& desiredState,
//                                     const Rotation2d& currentAngle);
};