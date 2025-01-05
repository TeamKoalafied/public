//==============================================================================
// Diverter.h
//==============================================================================
#pragma once

#include "../../Phoenix5Header.h"


class Diverter {
public:
    //==========================================================================
    // Construction

    // Constructor
    Diverter();

    // Destructor
    ~Diverter();

    //==========================================================================
    // Mechanism Lifetime - Setup, Shutdown and Periodic

    // Setup the Diverter for operation
    void Setup();

    // Shutdown the Diverter
    void Shutdown();

    // Perform periodic updates for the Diverter
    void Periodic();

    //==========================================================================
    // Operations

    // Drive the shooter in open-loop
    //
    // percentage_output - percentage of total output to supply the motor
    void DivertToPath(bool amp_trap, double scale_factor);

    // Runs the rollers to push the note into the shooter rollers
    void KickToShooter();


    void ManualDriveDiverter(double percent_output);

    double GetCurrent() const;
    double GetOutput() const;

private:
    TalonSRX* m_diverter_controller;
};