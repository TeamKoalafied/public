//==============================================================================
// Trampler.h
//==============================================================================

#pragma once
#include "../../Phoenix5Header.h"


class Trampler {
public:
    //==========================================================================
    // Construction

    // Constructor
    Trampler();

    // Destructor
    ~Trampler();

    //==========================================================================
    // Mechanism Lifetime - Setup, Shutdown and Periodic

    // Setup the Trampler for operation
    void Setup();

    // Shutdown the Trampler
    void Shutdown();

    // Perform periodic updates for the Trampler
    void Periodic();

    //==========================================================================
    // Operations

    void ManualDriveTrampler(double percent_output);

    double GetCurrent() const;
    double GetOutput() const;

private:   
    TalonSRX* m_trampler_controller;

};