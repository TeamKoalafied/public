//==============================================================================
// HapticController.cpp
//==============================================================================

#include "HapticController.h"
#include <iostream>
#include <iomanip>



//==============================================================================
// Construction

HapticController::HapticController(frc::Joystick* joystick) {
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
        m_joystick->SetRumble(frc::GenericHID::RumbleType::kLeftRumble, 0.0);
    }
    else {
        int value_position = m_period_counter / PERIODS_PER_VALUE;
        if (value_position < m_length) {
            m_joystick->SetRumble(frc::GenericHID::RumbleType::kLeftRumble, m_values[value_position]);
            m_period_counter++;
        }
        else {
            m_joystick->SetRumble(frc::GenericHID::RumbleType::kLeftRumble, 0.0);
            m_values = NULL;
            m_length = 0;
            m_period_counter = 0;
        }
    }
}

//==============================================================================
// Haptick Feedback

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
