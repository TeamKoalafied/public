//==============================================================================
// PovFilter.h
//==============================================================================

#pragma once

#include "../RobotConfiguration.h"

namespace RC = RobotConfiguration;


class PovFilter {
public:
    // Filter input from the POV so that it only returns the four major directions,
    // but also returns the previous major direction if the user slips onto a diagonal
    // direction.
    //
    // pov - Current POV value from the joystick
    //
    // Returns a filtered POV value
    int Filter(int pov) {
        switch (pov) {
            case RC::kJoystickPovUp:
            case RC::kJoystickPovDown:
            case RC::kJoystickPovLeft:
            case RC::kJoystickPovRight:
                // If a major direction is pressed just return it and update the previous
                m_previous_pov = pov;
                return pov;
            case RC::kJoystickPovUpLeft:
                // If a diagonal is pressed then if we were previously on an adjacent major direction
                // return that. Otherwise return that nothing is pressed (-1). Note that we do not
                // update the previous position.
                if (m_previous_pov == RC::kJoystickPovUp || m_previous_pov == RC::kJoystickPovLeft) {
                    return m_previous_pov;
                }
                return -1;
            case RC::kJoystickPovUpRight:
                if (m_previous_pov == RC::kJoystickPovUp || m_previous_pov == RC::kJoystickPovRight) {
                    return m_previous_pov;
                }
                return -1;
            case RC::kJoystickPovDownLeft:
                if (m_previous_pov == RC::kJoystickPovDown || m_previous_pov == RC::kJoystickPovLeft) {
                    return m_previous_pov;
                }
                return -1;
            case RC::kJoystickPovDownRight:
                if (m_previous_pov == RC::kJoystickPovDown || m_previous_pov == RC::kJoystickPovRight) {
                    return m_previous_pov;
                }
                return -1;
            default:
                // For any other value just return it and update the previous. This
                // should only be -1, indicating nothing pressed.
                m_previous_pov = pov;
                return pov;
        }
    }

private:
    // Member Variables
    int m_previous_pov = -1;        // Previous major direction POV value or none (-1)
};