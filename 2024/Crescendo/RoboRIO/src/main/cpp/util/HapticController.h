//==============================================================================
// HapticController.h
//==============================================================================

#pragma once

#include <frc/GenericHID.h>
#include <vector>



// A controller for doing haptic feedback to a joystick.
class HapticController {
public:
    //==========================================================================
    // Construction

    // Constructor
    //
    // joystick - The joystick to be controlled by this object
    HapticController(frc::GenericHID* joystick);

    // Destructor
    ~HapticController();


    //==========================================================================
    // Updating

    // Perform periodic updates to the haptic feedback
    void Periodic();


    //==========================================================================
    // Haptick Feedback

    // Play a haptic rumble with a given pattern
    //
    // values - Array of values for the rumble, or nullptr so stop any rumble. Each value is 'played' for 100ms.
    // length - Length of the 'values' array
    void DoFeedback(const double* values, int length);

    // Play a haptic rumble with a given value
    //
    // time_s - Length of time to rumble for in seconds
    // value - Value to play
    void DoContinuousFeedback(double time_s, double value);

    // Play a haptic rumble that indicates success
    void DoSuccessFeedback();

    // Play a haptic rumble that indicates failure
    void DoFailureFeedback();


    //==========================================================================
    // Testing
    
    // Run different test rumbles if the POV buttons on the joystick are pressed
    void DoJoystickTestControl();

private:

    //==========================================================================
    // Member Variables

    frc::GenericHID* m_joystick;                // The joystick to be controlled by this object
    const double* m_values;                     // Array of rumble values being played
    int m_length;                               // Length of the 'm_values' array
    int m_period_counter;                       // Count of update periods since the current rumble started
    std::vector<double> m_value_buffer;         // Buffer used for continuous rumbles

    static const int PERIODS_PER_VALUE = 5;     // Update periods to 'play' each value for
};
