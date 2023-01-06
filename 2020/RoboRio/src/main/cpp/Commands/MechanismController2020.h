//==============================================================================
// MechanismController2020.h
//==============================================================================

#pragma once

#include "RobotPath/IMechanismController.h"

// MechanismController2020 implements mechanism actions on autonomous paths for
// the 2020 robot
class MechanismController2020 : public IMechanismController {
public:
    //==========================================================================
    // Overrides from IMechanismController

    virtual void DoAction(const std::string& action);
};
