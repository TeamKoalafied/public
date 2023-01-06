//
// FindTargetControl.h
//

#ifndef FindTargetControl_H
#define FindTargetControl_H


#include <frc/Joystick.h>
#include <frc/Timer.h>
class HapticController;
class DriveBase;

//
class FindTargetControl
{
public:
    //==========================================================================
    // Construction and Destruction

    // Constructor
    //
    // drive_base - Drive base to control
    FindTargetControl(DriveBase& drive_base);

    // Destructor
    ~FindTargetControl();

    //==========================================================================
    // Operation

    // Update the information about the target heading. This function must be called
    // every robot period so that the target information is up to date.
    void UpdateTargetHeading();

    // Perform finding and rotating to the target under joystick control
    //
    // joystick - Joystick being used for drivebase control
    // haptic_controller - Controller for doing haptic feedback to the driver
    // haptic_controller2 - Controller for doing haptic feedback to the operator
    //
    // Returns whether normal driving operation should be performed in this period update.
    bool DoFindTargetJoystick(frc::Joystick* joystick, HapticController* haptic_controller, HapticController* haptic_controller2);

    // Rotate to find the target if possible
    //
    // Returns true if the target is found and the robot has rotated to face it
    bool AutoRotateToTarget();

    // Get the distance to the target
    //
    // distance - Returns the distance to the target in metres
    //
    // Returns whether the distance is valid
    bool GetTargetDistance(double& distance_m) const;


    //==========================================================================
    // Dashboard Setup

    // Setup the dashboard for vision control parameters. Called when robot is initialised.
    static void SetupDashboard();


private:
    //==========================================================================
    // Private Nested Types

    // State of finding the target
    enum class State {
        kIdle,              // Idle. Driver is not trying to rotate to target
        kSignalNoTarget,    // Driver tried to rotate but there was no target visible
        kRotatingToTarget,  // Currently rotating to the target
        kReachedTarget      // Target had been rotated to
    };


    //==========================================================================
    // Implementation

    // Rotate the drivebase towards the target
    //
    // Returns
    bool RotateToTarget();


    //==========================================================================
    // Constants and Member Variables

    // Number heading/distance samples recorded in the buffers
//    static const int HISTORY_LENGTH = 50;
    static const int HISTORY_LENGTH = 25;

    // Number of valid heading/distance samples required for the target to be valid     
    static const int HISTORY_COUNT_FOR_VALID = 5;

    DriveBase& m_drive_base;                    // Drive base being controlled
    State m_state;                              // Current state of finding the target
    double m_target_headings[HISTORY_LENGTH];   // Circular buffer of target headings 
    double m_target_distances_m[HISTORY_LENGTH];// Circular buffer of target distances in metres
    int m_target_history_index;                 // Index to insert the enxt entry into the buffer
    bool m_target_valid;                        // Whether the target heading/distance is currently valid
    double m_target_heading;                    // Current heading of the target in drive base pigeon degrees, may not be valid
    double m_target_distance_m;                 // Current distance to the target in metres, may not be valid

    static constexpr double kErrorHeading = 999.0;
    static constexpr double kErrorDistance = 0.0;
};

#endif
