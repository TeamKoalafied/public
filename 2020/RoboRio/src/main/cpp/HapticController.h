//==============================================================================
// HapticController.h
//==============================================================================


#ifndef SRC_HapticController_H_
#define SRC_HapticController_H_

#include <frc/Joystick.h>



// A controller for doing haptic feedback to a joystick
class HapticController {
public:
    //==========================================================================
    // Construction

    // Constructor
    //
    // joystick - The joystick to be controlled by this object
    HapticController(frc::Joystick* joystick);

    // Destructor
    ~HapticController();


    //==========================================================================
    // Updating

    // Perform periodic updates to the haptic feedback
    void Periodic();


    //==========================================================================
    // Haptick Feedback

    void DoFeedback(double* values, int length);

    void DoContinuousFeedback(double time_s, double value);

private:

    //==========================================================================
    // Member Variables

    frc::Joystick* m_joystick;              // The joystick to be controlled by this object
    double* m_values;
    int m_length;
    int m_period_counter;
    std::vector<double> m_value_buffer;

    static const int PERIODS_PER_VALUE = 5;     // Update periods to 'play' each value for
};

#endif /* SRC_PERIODICTIMER_H_ */
