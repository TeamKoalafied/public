//==============================================================================
// Shooter.cpp
//==============================================================================
#include "Shooter.h"

#include <iostream>

#include <ctre/phoenix6/configs/Configs.hpp>
#include <units/angular_velocity.h>

#include "../../RobotConfiguration.h"
#include "../../util/KoalafiedUtilities.h"

namespace RC = RobotConfiguration;

//==============================================================================
// Construction

Shooter::Shooter() {

}

Shooter::~Shooter() {
    Shutdown();
}

//==============================================================================
// Mechanism Lifetime - Setup, Shutdown and Periodic

void Shooter::Setup() {
    m_shooter_controller = new ctre::phoenix6::hardware::TalonFX(RC::kShooterTalonId);
    // Talon config
    ctre::phoenix6::configs::TalonFXConfiguration shooter_configuration;

    shooter_configuration.WithMotorOutput(ctre::phoenix6::configs::MotorOutputConfigs()
                                            .WithPeakForwardDutyCycle(1.0)
                                            .WithPeakReverseDutyCycle(-1.0));

    shooter_configuration.WithCurrentLimits(ctre::phoenix6::configs::CurrentLimitsConfigs()
                                            .WithSupplyCurrentLimitEnable(true)
                                            .WithSupplyCurrentLimit(RC::kShooterMotorPeakCurrentLimit)
                                            .WithSupplyCurrentThreshold(RC::kShooterMotorContinuousCurrentLimit)
                                            .WithSupplyTimeThreshold(RC::kShooterMotorPeakCurrentDurationS));

    shooter_configuration.WithSlot0(ctre::phoenix6::configs::Slot0Configs()
                                    .WithKV(0.11291)
                                    .WithKP(0.4) // Was 0.25873, but increase to reduce closed loop error
                                    .WithKI(0.0)
                                    .WithKD(0.0));

    shooter_configuration.WithOpenLoopRamps(ctre::phoenix6::configs::OpenLoopRampsConfigs()
                                                .WithVoltageOpenLoopRampPeriod(RC::kShooterMotorOpenLoopRampRateS));
    shooter_configuration.WithClosedLoopRamps(ctre::phoenix6::configs::ClosedLoopRampsConfigs()
                                                .WithVoltageClosedLoopRampPeriod(RC::kShooterMotorClosedLoopRampRateS));


    double max_steer_rps = RC::kModuleMaxAngularVelocity / (2.0_rad_per_s * std::numbers::pi);
    double max_steer_motor_rps = max_steer_rps * RC::kDriveBaseSteerGearRatio;

    shooter_configuration.WithMotionMagic(ctre::phoenix6::configs::MotionMagicConfigs()
                                            .WithMotionMagicCruiseVelocity(max_steer_motor_rps)
                                            .WithMotionMagicAcceleration(4.0*max_steer_motor_rps));
                                            
    auto error = m_shooter_controller->GetConfigurator().Apply(shooter_configuration);
    if (error != 0) {
        std::cout << "Configuration of the shooter Falcon failed with code:  " << error << "\n";

    }
    m_shooter_controller->SetNeutralMode(ctre::phoenix6::signals::NeutralModeValue::Coast);
}

void Shooter::Periodic() {

}

void Shooter::Shutdown() {

}


//==============================================================================
// State

units::ampere_t Shooter::GetCurrent() const {
    units::ampere_t current = m_shooter_controller->GetSupplyCurrent().GetValue();
    return current;
}

units::revolutions_per_minute_t Shooter::GetSpeed() const {
    units::revolutions_per_minute_t rpm = m_shooter_controller->GetVelocity().GetValue();
    return rpm;
}

double Shooter::GetOutput() const {
    return m_shooter_controller->Get();
}


//==============================================================================
// Operations

void Shooter::DriveShooterOpenLoop(double percentage_output) {
    // voltage compensated
    
    m_shooter_controller->SetControl(ctre::phoenix6::controls::VoltageOut(percentage_output*12_V,false));
}

void Shooter::DriveShooterClosedLoop(units::revolutions_per_minute_t target_rpm) {
    m_shooter_controller->SetControl(ctre::phoenix6::controls::VelocityVoltage(target_rpm)
                                        .WithEnableFOC(false));
    // velocity voltage
}

