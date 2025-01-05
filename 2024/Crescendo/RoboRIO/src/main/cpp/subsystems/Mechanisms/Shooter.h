//==============================================================================
// Shooter.h
//==============================================================================
#pragma once

#include <ctre/phoenix6/TalonFX.hpp>
#include <units/angular_velocity.h>


class Shooter {
public:
    //==========================================================================
    // Construction

    // Constructor
    Shooter();

    // Destructor
    ~Shooter();

    //==========================================================================
    // Mechanism Lifetime - Setup, Shutdown and Periodic

    // Setup the Intake for operation
    void Setup();

    // Shutdown the Intake
    void Shutdown();

    // Perform periodic updates for the Intake
    void Periodic();


    //==========================================================================
    // State

    // Get the shooter motor current in amps
    units::ampere_t GetCurrent() const;

    // Get the pivot motor drive as a fraction [-1, 1]
    double GetOutput() const; 

    // Get the shooter motor speed in RPM
    units::revolutions_per_minute_t GetSpeed() const;


    //==========================================================================
    // Operations

    // Drive the shooter in open-loop
    //
    // percentage_output - percentage of total output to supply the motor
    void DriveShooterOpenLoop(double percentage_output);

    // Drive the shooter in a velocity closed-loop
    //
    // target_rpm - target rpm of the flywheel
    void DriveShooterClosedLoop(units::revolutions_per_minute_t target_rpm);
    

private:
    ctre::phoenix6::hardware::TalonFX* m_shooter_controller;


};