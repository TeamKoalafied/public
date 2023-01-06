//==============================================================================
// MechanismController2020.h
//==============================================================================

#pragma once

#include "RobotPath/IMechanismController.h"

// MechanismController2022 implements mechanism actions on autonomous paths for
// the 2022 robot
class MechanismController2022 : public IMechanismController {
public:
    //==========================================================================
    // Construction

    // Constructor
    MechanismController2022();

    //==========================================================================
    // Overrides from IMechanismController

    virtual void DoAction(const std::string& action);
    virtual bool AreAllActionsDone();


private:
    bool m_shooting;
};
