//==============================================================================
// PathPlanner.h
//==============================================================================


#pragma once

class SwerveDrivebase;
class Manipulator;


namespace PathPlanner {
    // Default values for Path Planner PID controllers
    const double kDirectionP = 0.5;
    const double kDirectionD = 0.0;
    const double kRotationP = 0.5;
    const double kRotationD = 0.0;

    // Setup the Path Planner swerve auto builder with default PID parameters
    //
    // drivebase - Swerve direvebase to set up for
    // manipulator - Manipulator to set up for. Can is nullptr.
    void SetupSwerveAuto(SwerveDrivebase& drivebase, Manipulator* manipulator);

    // Setup the Path Planner swerve auto builder with custom PID parameters
    //
    // drivebase - Swerve direvebase to set up for
    // manipulator - Manipulator to set up for. Can is nullptr.
    // direction_P - P gain for the direction (X/Y) PID controller
    // direction_D - D gain for the direction (X/Y) PID controller
    // rotation_P - P gain for the rotation PID controller
    // rotation_D - D gain for the rotation PID controller
    void SetupSwerveAuto(SwerveDrivebase& drivebase, Manipulator* manipulator, double direction_P, double direction_D, double rotation_P, double rotation_D);
}