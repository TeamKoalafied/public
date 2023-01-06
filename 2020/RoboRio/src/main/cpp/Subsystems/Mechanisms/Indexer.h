//==============================================================================
// Indexer.h
//==============================================================================

#ifndef Indexer_H
#define Indexer_H

#include <ctre/Phoenix.h>
#include <frc/Timer.h>
#include "../../RobotConfiguration.h"

namespace frc { 
    class Joystick;
}

// The Indexer mechanism is part of the Manipulator subsystem. It feeds and stores
// the balls from the intake and delivers them to the shooter, via the kicker. The
// indexer consists of the following actuators and sensors:
//
//  - Indexer motor controlled by a Talon SRX with magnetic encoder
class Indexer  {
public:
    //==========================================================================
    // Construction

    // Constructor
    Indexer();

    // Destructor
    virtual ~Indexer();


    //==========================================================================
    // Mechanism Lifetime - Setup, Shutdown and Periodic

    // Setup the Indexer for operation
    void Setup();

    // Shutdown the Indexer
    void Shutdown();

    // Perform periodic updates for the Indexer
    void Periodic();


    //==========================================================================
    // Operations

    // Manually drive the indexer at a given percentage of motor output
    //
    // percentage_output - Percentage output to drive at. Positive moves balls up from intake to shooter.
    void ManualDriveIndexer(double percentage_output);

    // Drive the indexer with close loop velocity control at a given percentage of peak
    //
    // percentage_speed - Percentage of peak spped to drive at. Positive moves balls up from intake to shooter.
    void VelocityDriveIndexer(double percentage_speed);

    // Set the indexer position to zero. Allow measurements that follow to be relative.
    void ZeroIndexerPosition();

    // Get the position of the indexer in inches
    double GetIndexerPositionInch();

    // Perform testing of the indexer using the joystick. This function is only for testing the
    // indexer and may use any controls of the joystick in an ad hoc fashion.
    //
    // joystick - Joystick to use
    void TestDriveIndexer(frc::Joystick* joystick);

private:

    //==========================================================================
    // Member Variables

    TalonSRX* m_indexer_speed_controller;   // Motor controller for the indexer
};

#endif  // Indexer_H
