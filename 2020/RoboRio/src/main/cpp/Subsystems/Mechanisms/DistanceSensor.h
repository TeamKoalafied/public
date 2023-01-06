//==============================================================================
// DistanceSensor.h
//==============================================================================

#ifndef DistanceSensor_H
#define DistanceSensor_H

#include <frc/smartdashboard/SendableChooser.h>
#include "rev/Rev2mDistanceSensor.h"
#include "../../RobotConfiguration.h"

namespace frc { 
    class Joystick;
}

// The DistanceSensor mechanism is part of the Manipulator subsystem. It controls the
// 
// 
class DistanceSensor  {
public:
    //==========================================================================
    // Construction

    // Constructor
    DistanceSensor();

    // Destructor
    virtual ~DistanceSensor();


    //==========================================================================
    // Mechanism Lifetime - Setup, Shutdown and Periodic

    // Setup the DistanceSensor for operation
    void Setup();

    // // Shutdown the DistanceSensor
    void Shutdown();

    // Perform periodic updates for the DistanceSensor
    //
    // show_dashboard - whether to show debugging information on the dashboard
    void Periodic(bool show_dashboard);

    double GetIntakeDistance();
    double GetShooterDistance();

private:

    //==========================================================================
    // Member Variables

    rev::Rev2mDistanceSensor *RevSensor;
    rev::Rev2mDistanceSensor *RevSensor2;
};

#endif  // DistanceSensor_H
