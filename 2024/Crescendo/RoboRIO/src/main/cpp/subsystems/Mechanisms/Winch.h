//==============================================================================
// Winch.h
//==============================================================================

#pragma once
#include "../../Phoenix5Header.h"
#include <units/length.h>
#include <numbers>

class ManipulatorShuffleboard;

class Winch {
public:
    //==========================================================================
    // Public Constants

    static constexpr units::inch_t kForwardLimit = 11.8_in;
    static constexpr units::inch_t kReverseLimit = -12.5_in;


    //==========================================================================
    // Construction

    // Constructor
    Winch();

    // Destructor
    ~Winch();

    //==========================================================================
    // Mechanism Lifetime - Setup, Shutdown and Periodic

    // Setup the Intake for operation
    void Setup();

    // Shutdown the Intake
    void Shutdown();

    // Perform periodic updates
    //
    // shuffleboard - Shuffleboard tab for access to the winch limit settings
    void Periodic(ManipulatorShuffleboard* shuffleboard);

    //==========================================================================
    // Operations
    void ManualDriveWinch(double percentage_output);

    double GetOutput() const;
    double GetCurrent() const;
    units::inch_t GetWinchPosition() const;
    void HoldPosition(units::inch_t position);
    bool IsExtensionSet() const;
    
private:

    TalonSRX* m_winch_controller;
    units::inch_t m_winch_forward_limit;
    units::inch_t m_winch_reverse_limit;

    static constexpr units::inch_t kWinchDiameter = 1.5_in;

    static constexpr units::inch_t kWinchInchPerEncoder = kWinchDiameter * std::numbers::pi / 4096;
    const units::inch_t kWinchExtensionNotSet = -1000.0_in;
    units::inch_t m_winch_extension_set_inch = kWinchExtensionNotSet;
};