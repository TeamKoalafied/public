//==============================================================================
// IPathDriveBase.h
//==============================================================================

#pragma once

#include <string>
class PathSegment;
class RobotPath;


// IPathDriveBase is an interface for a drive base that is used by autonomous commands.
class IPathDriveBase {
public:

    //==========================================================================
    // DriveBase Control

    // Do tank drive in open loop mode
    //
    // left_output - proportional signal for the left motor in the range -1 to + 1
    // right_output - proportional signal for the right motor in the range -1 to + 1
    virtual void TankDriveOpenLoop(double left_output, double right_output) = 0;

    // Stops the drive base
    virtual void Stop() = 0;

    // Set the brake mode for the drive base motors
    //
    // brake - Whether to set to brake (true) or cruise (false)
    virtual void SetBrakeMode(bool brake) = 0;

    // Do an autonomous action (such as rotating to a target)
    //
    // action - Action to perform
    //
    // Returns true if the action is complete
    virtual bool DoPathAction(const std::string& action) = 0;


    //==========================================================================
    // DriveBase Sensors

    // Get the distance travelled by the left and right wheels, as measured by the encoders,
    // but converted into metres
    //
    // left_distance_m - Returns the left distance in metres
    // right_distance_m - Returns the right distance in metres
    virtual void GetWheelDistancesM(double& left_distance_m, double& right_distance_m) = 0;

    // Get velocity of the left and right wheels
    //
    // left_velocity - Returns the left wheel velocity
    // right_velocity - Returns the right wheel velocity
    virtual void GetWheelVelocity(double& left_velocity, double& right_velocity) = 0;


    // Get the current heading from the Pigeon IMU device
    //
    // Returns the heading in degrees, or kHeadingError if the value cannot be read. Positive is anticlockwise.
    virtual double GetPigeonHeading() = 0;

    // Reset the dead reconning position. A position and heading can be specified.
    //
    // position_x_inch - The x position in inches
    // position_y_inch - The y position in inches
    // heading_degrees - The 'position' heading in degrees (this is the not the pigeon heading)
    virtual void ResetPosition(double position_x_inch = 0.0, double position_y_inch = 0.0, double heading_degrees = 0.0) = 0;

    // Get the dead reconning position, in inches
    //
    // position_x_inch - Returns the x position in inches
    // position_y_inch - Returns the y position in inches
    // heading_degrees - Returns the 'position' heading in degrees (this is the not the pigeon heading)
    //
    // Dead recconing used standard mathematical convention for x and y axis. Zero heading is along the x-axis
    // and angles increase in the anitclockwise direction.
    virtual void GetPositionInch(double& position_x_inch, double& position_y_inch, double& heading_degrees) = 0;

    // Get the dead reconning position, in metres
    //
    // position_x_inch - Returns the x position in metres
    // position_y_inch - Returns the y position in metres
    // heading_degrees - Returns the 'position' heading in degrees (this is the not the pigeon heading)
    //
    // Dead recconing used standard mathematical convention for x and y axis. Zero heading is along the x-axis
    // and angles increase in the anitclockwise direction.
    virtual void GetPositionM(double& position_x_m, double& position_y_m, double& heading_degrees) = 0;

};
