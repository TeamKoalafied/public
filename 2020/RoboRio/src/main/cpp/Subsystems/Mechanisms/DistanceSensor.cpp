//==============================================================================
// DistanceSensor.cpp
//
// From: https://github.com/REVrobotics/2m-Distance-Sensor/
//       blob/master/Examples/C%2B%2B/Read%20Range/src/main/cpp/Robot.cpp
//==============================================================================

#include "DistanceSensor.h"

#include "../../RobotConfiguration.h"

#include <frc/smartdashboard/SmartDashboard.h>
#include <iostream>
#include "rev/Rev2mDistanceSensor.h"



//==============================================================================
// Construction

DistanceSensor::DistanceSensor()  {
}

DistanceSensor::~DistanceSensor() {
    Shutdown();
}


//==============================================================================
// Mechanism Lifetime - Setup, Shutdown and Periodic

void DistanceSensor::Setup() {
    std::cout << "DistanceSensor::Setup()\n";
    RevSensor = new rev::Rev2mDistanceSensor{rev::Rev2mDistanceSensor::Port::kOnboard, rev::Rev2mDistanceSensor::DistanceUnit::kInches};

    RevSensor->SetAutomaticMode(true);
    RevSensor->SetEnabled(true);

    RevSensor2 = new rev::Rev2mDistanceSensor{rev::Rev2mDistanceSensor::Port::kMXP, rev::Rev2mDistanceSensor::DistanceUnit::kInches};
    RevSensor2->SetAutomaticMode(true);
    RevSensor2->SetEnabled(true);
}

void DistanceSensor::Shutdown() {
    std::cout << "DistanceSensor::Shutdown()\n";
}

void DistanceSensor::Periodic(bool show_dashboard)
{   
    // if (!RevSensor->IsRangeValid()) {
    //     std::cout << "splippity boop" << std::endl;

    //     delete RevSensor;
    //     RevSensor = new rev::Rev2mDistanceSensor{rev::Rev2mDistanceSensor::Port::kOnboard, 
    //                                             rev::Rev2mDistanceSensor::DistanceUnit::kInches};
    //     std::cout << "boppity splip" << std::endl;
    //     RevSensor->SetAutomaticMode(true);
    //     RevSensor->SetEnabled(true);
    // }

    
    if (show_dashboard) {
        bool isValid = RevSensor->IsRangeValid();
        bool isValid2 = RevSensor2->IsRangeValid();

        frc::SmartDashboard::PutBoolean("Distance Sensor 1 Valid", isValid);
        frc::SmartDashboard::PutBoolean("Distance Sensor 2 Valid", isValid2);

        if (isValid) {
            frc::SmartDashboard::PutNumber("RevDistance (in)", RevSensor->GetRange());

            frc::SmartDashboard::PutNumber("Distance Timestamp", RevSensor->GetTimestamp());
        } else {
            frc::SmartDashboard::PutNumber("RevDistance (in)", -1);
        }

        if (isValid2) {
            frc::SmartDashboard::PutNumber("2nd RevDistance (in)", RevSensor2->GetRange());

            frc::SmartDashboard::PutNumber("2nd Distance Timestamp", RevSensor2->GetTimestamp());
        } else {
            frc::SmartDashboard::PutNumber("2nd RevDistance (in)", -1);
        }
    }
}

double DistanceSensor::GetIntakeDistance() {
    return RevSensor->GetRange();
}

double DistanceSensor::GetShooterDistance() {
    return RevSensor2->GetRange();
}