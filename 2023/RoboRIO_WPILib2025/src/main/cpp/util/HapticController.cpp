//==============================================================================
// HapticController.cpp
//==============================================================================

#include "HapticController.h"

#include "../RobotConfiguration.h"

#include <iostream>
#include <iomanip>

namespace RC = RobotConfiguration;


//==============================================================================
// Construction

HapticController::HapticController(frc::GenericHID* joystick) {
    m_joystick = joystick;
    m_values = NULL;
    m_length = 0;
    m_period_counter = 0;
}

HapticController::~HapticController() {

}


//==============================================================================
// Updating

void HapticController::Periodic() {
    if (m_values == NULL) {
        // If there is no haptic feedback in progress, just ensure that the rumble is off
        m_joystick->SetRumble(frc::GenericHID::RumbleType::kBothRumble, 0.0);
    }
    else {
        // Work out the index in the value array that we are up to
        int value_position = m_period_counter / PERIODS_PER_VALUE;
        if (value_position < m_length) {
            // If still playing set the rumble and update the period counter
            m_joystick->SetRumble(frc::GenericHID::RumbleType::kBothRumble, m_values[value_position]);
            m_period_counter++;
        }
        else {
            // If passed the end of the array stop the rumble and clear the state to not playing
            m_joystick->SetRumble(frc::GenericHID::RumbleType::kBothRumble, 0.0);
            m_values = NULL;
            m_length = 0;
            m_period_counter = 0;
        }
    }
}


//==============================================================================
// Haptic Feedback

void HapticController::DoFeedback(double* values, int length) {
    m_values = values;
    m_length = length;
    m_period_counter = 0;
}

void HapticController::DoContinuousFeedback(double time_s, double value) {
    int length = (int)(time_s * 50 / PERIODS_PER_VALUE);
    m_value_buffer.resize(length);
    for (int i = 0; i < length; i++) m_value_buffer[i] = value;

    DoFeedback(m_value_buffer.data(), length);
}

//==============================================================================
// Testing

void HapticController::DoJoystickTestControl()
{
    static int previous_pov_angle = 0;
	int pov_angle = m_joystick->GetPOV(0);
    if (pov_angle != previous_pov_angle) {
        previous_pov_angle = pov_angle;
        switch (pov_angle) {
            case RC::kJoystickPovUp: {
                DoContinuousFeedback(1.0, 1.0);
                break;
            }
            case RC::kJoystickPovLeft: {
                DoContinuousFeedback(1.0, 0.5);
                break;
            }
            case RC::kJoystickPovDown: {
                DoContinuousFeedback(1.0, 0.25);
                break;
            }
            case RC::kJoystickPovRight: {
                static double PATTERN[10] = { 1.0, 1.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 1.0, 1.0 };
                DoFeedback(PATTERN, 10);
                break;
            }
        }
    }
}
