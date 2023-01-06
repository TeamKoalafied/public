//==============================================================================
// IMechanismController.h
//==============================================================================

#pragma once

#include <string>

// IMechanismController defines an interface for an object that implements mechanism
// actions, for use with autonomous paths
class IMechanismController {
public:
    // Do a given action with the robot's mechanism
    //
    // action - the action to implement
    virtual void DoAction(const std::string& action) = 0;

    // Get whether all actions have been completed
    virtual bool AreAllActionsDone() = 0;
};