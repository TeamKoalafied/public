//==============================================================================
// SwerveModule.cpp
//==============================================================================


#include "SwerveModule.h"

#include "../RobotConfiguration.h"
#include "../util/KoalafiedUtilities.h"

#include <ctre/phoenix6/configs/Configs.hpp>

#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/shuffleboard/Shuffleboard.h>
#include <frc/shuffleboard/ShuffleboardContainer.h>
#include <frc/shuffleboard/WidgetType.h>
#include <frc/system/plant/DCMotor.h>
#include <units/angular_velocity.h>
#include <iostream>
#include <math.h>
#include <units/voltage.h>
//#include <units/constants.h>

namespace RC = RobotConfiguration;


SwerveModule::SwerveModule(std::string_view name, int drive_falcon_id, int steer_falcon_id, int steer_encoder_id,
                           units::radian_t zero_point) :
    m_drive_controller(drive_falcon_id),
    m_steer_controller(steer_falcon_id)
 {
    // Set name and zero point of the module according to supplied parameters
    m_name = name;
    m_zero_point = zero_point;

    // Create the steer CANcoder if a valid id is given, otherwise initialise to null pointer
    m_steer_encoder = steer_encoder_id < 0 ? nullptr : new ctre::phoenix6::hardware::CANcoder(steer_encoder_id);

    // Initialise to no error
    m_can_error = false;

    // Setup the drive Falcon FX controller ------------------------------------
    DriveTalonSetup();

    // Setup the steer Falcon FX controller ------------------------------------
    SteerTalonSetup();


    // Initialise widget pointers to the null pointer
    m_canbus_error_widget = nullptr;
    m_drive_current_widget = nullptr;
    m_drive_temp_widget = nullptr;
    m_drive_speed_widget = nullptr;
    m_steer_current_widget = nullptr;
    m_steer_temp_widget = nullptr;
}

frc::SwerveModuleState SwerveModule::GetState() const {
    // Calculate the wheel speed from the drive Falcon encoder velocity.
    //      - First from native units to motor rpm, then to wheel rpm, and finally m/s
    // double drive_motor_rpm = KoalafiedUtilities::TalonFXVelocityNativeToRpm(m_drive_controller.GetSelectedSensorVelocity());
    units::revolutions_per_minute_t drive_motor_rpm = m_drive_controller.GetRotorVelocity().GetValue();
    units::revolutions_per_minute_t drive_wheel_rpm = drive_motor_rpm / RC::kDriveBaseGearRatio;
    units::meters_per_second_t speed = units::scalar_t(std::numbers::pi) * RC::kWheelDiameterInch * drive_wheel_rpm.value() / 60.0_s;

    // Calculate the angle of the module, from the CANcoder absolute angle then subtract the zero point 
    // units::radian_t steer_encoder_angle = units::degree_t(m_steer_encoder->GetAbsolutePosition());
    units::radian_t steer_encoder_angle = m_steer_encoder->GetAbsolutePosition().GetValue();

    steer_encoder_angle -= m_zero_point;
    return frc::SwerveModuleState{speed, frc::Rotation2d(steer_encoder_angle)};
}

frc::SwerveModuleState SwerveModule::GetDesiredState() const {
    return m_desired_state;
}

frc::SwerveModulePosition SwerveModule::GetPosition() const {
    // Calculate the wheel speed from the drive Falcon encoder revolutions.
    //      - First from native units to motor rpm, then to wheel rpm, and finally m/s
    // double drive_motor_revs = m_drive_controller.GetSelectedSensorPosition() / RC::kTalonFXEnocderCounts;
    double drive_motor_revs = m_drive_controller.GetRotorPosition().GetValueAsDouble();
    double drive_wheel_revs = drive_motor_revs / RC::kDriveBaseGearRatio;
    units::meter_t distance = units::scalar_t(std::numbers::pi) * RC::kWheelDiameterInch * drive_wheel_revs;

    // Set the steer encoder angle to the absolute value of the steer encoder, then subtract the zero point
    units::radian_t steer_encoder_angle = m_steer_encoder->GetAbsolutePosition().GetValue();
    steer_encoder_angle -= m_zero_point;

    return {distance, frc::Rotation2d(steer_encoder_angle)};
}

void SwerveModule::SetDesiredState(const frc::SwerveModuleState& state, bool dry_steer_allowed) {
    // Record the desired state for debugging
    m_desired_state = state;

    // If speed is zero and are not doing dry steering, set motor outputs to zero and do nothing else
    if (state.speed == 0.0_mps) {
        m_drive_controller.SetControl(ctre::phoenix6::controls::DutyCycleOut(0.0).WithEnableFOC(false));
        if (!dry_steer_allowed) {
            m_steer_controller.SetControl(ctre::phoenix6::controls::DutyCycleOut(0.0).WithEnableFOC(false));
            return;
        }
    }

    // Use the current state of the module to calculate an optimatised state. This means that the
    // module will turn the steering through the smallest angle, reversing the direction that the
    // wheel is driven if it required.
    frc::SwerveModuleState current_state = GetState();
    frc::SwerveModuleState optimized_state = frc::SwerveModuleState::Optimize(state, current_state.angle.Radians());

    // Convert speed in m/s into native units. This is done in 3 steps.
    //  1. Convert the linear wheel speed to wheel RPM, using the wheel circumference
    //  2. Convert the wheel RPM into motor RPM, using the module gear ratio
    //  3. Convert the motor RPM into native units
    if (state.speed != 0.0_mps) {
        double drive_wheel_rpm = optimized_state.speed * 60.0_s / (units::scalar_t(std::numbers::pi) * RC::kWheelDiameterInch);
        units::revolutions_per_minute_t drive_motor_rpm = units::revolutions_per_minute_t(drive_wheel_rpm * RC::kDriveBaseGearRatio);
        m_drive_controller.SetControl(ctre::phoenix6::controls::VelocityVoltage(drive_motor_rpm)
                                        .WithEnableFOC(false));
        // double drive_motor_native = KoalafiedUtilities::TalonFXVelocityRpmToNative(drive_motor_rpm);
        // m_drive_controller.Set(ControlMode::Velocity, drive_motor_native);
    }

    // Steer the wheel to the desired angle, using Motion Magic for a smooth controlled motion
    SteerToAngleMotionMagic(optimized_state.angle.Radians());
}

void SwerveModule::SetDesiredFollowerState(const SwerveFollowerModuleState& state) {
    // TODO Calculate the feed forward, plus a P term based on the velocity error
    

    // If speed is zero and are not doing dry steering, set motor outputs to zero and do nothing else
    // if (state.speed == 0.0_mps) {
    //     m_drive_controller.Set(ControlMode::PercentOutput, 0.0);
    //     m_steer_controller.Set(ControlMode::PercentOutput, 0.0);
    //     return;
    // }

    // Use the current state of the module to calculate an optimatised state. This means that the
    // module will turn the steering through the smallest angle, reversing the direction that the
    // wheel is driven if it required.
    frc::SwerveModuleState current_state = GetState();


    units::meters_per_second_t desired_speed = state.speed;
    units::meters_per_second_squared_t desired_acceleration = state.acceleration;
    frc::Rotation2d desired_angle = state.angle;


    frc::Rotation2d delta = desired_angle - current_state.angle;
    if (units::math::abs(delta.Degrees()) > 90_deg) {
        desired_speed = -desired_speed;
        desired_acceleration = -desired_acceleration;
        desired_angle = desired_angle + frc::Rotation2d{180_deg};
    }


    // Setup sensible default values for the parameters
   	// Velocity = 1.17ft/s/V => 14.04ft/s for 12V => 4.28m/s for 12V
 	// Acceleration = ~5ft/s2/V => 60ft/s2 for 12V => 18.29m/s2 for 12V

    auto KV = 2.80_V/1_mps;
    auto KA = 0.110_V/1_mps_sq;
    units::volt_t KV_offset = 0.0749_V;
    auto P = 1.0/1_mps;

    // Calculate the error and accumulated total error
	units::meters_per_second_t error = desired_speed - current_state.speed;

	// Calculate the feed forward term. It contains 3 parts.
	//  1. A term proportional to the desired velocity
	//  2. A fixed term in the direction of the desired velocity
	//  3. A term proportional to the desired acceleration
	units::volt_t feed_forward = KV * desired_speed + KA * desired_acceleration;
    if (desired_speed != 0.0_mps) {
        if (desired_speed > 0.0_mps) {
            feed_forward += KV_offset;
        } else {
            feed_forward -= KV_offset;
        }
    } else if (desired_acceleration != 0.0_mps_sq) {
        if (desired_acceleration > 0.0_mps_sq) {
            feed_forward += KV_offset;
        } else {
            feed_forward -= KV_offset;
        }
    }

	// Calculate the PID term. It contins 3 parts.
	//	1. P term 'proportional' is based on the current error
	//	2. I term 'integral' is based on the accumulated total error
	//	3. D term 'derivative' is based on the change in the errorr
	double pid_value = P * error;

	// The require output is the sum of the feed forward and PID values
    m_drive_controller.SetControl(ctre::phoenix6::controls::VoltageOut(feed_forward + pid_value*12_V).WithEnableFOC(false));
//    m_drive_controller.Set(ControlMode::PercentOutput, 1.0);

    // VElocity mode works
    // desired_speed += 0.5_mps;
    // double drive_wheel_rpm = desired_speed * 60.0_s / (units::scalar_t(std::numbers::pi) * RC::kWheelDiameterInch);
    // double drive_motor_rpm = drive_wheel_rpm * RC::kDriveBaseGearRatio;
    // double drive_motor_native = KoalafiedUtilities::TalonFXVelocityRpmToNative(drive_motor_rpm);
    // m_drive_controller.Set(ControlMode::Velocity, drive_motor_native);



    // Steer the wheel to the desired angle, using Motion Magic for a smooth controlled motion
    SteerToAngleMotionMagic(desired_angle.Radians());

}


units::degree_t SwerveModule::GetAbsoluteSteerEncoderPosition() {
    if (m_steer_encoder == nullptr) return 0_deg;

    return m_steer_encoder->GetAbsolutePosition().GetValue();
}

void SwerveModule::SetSteerEncoderCalibration(units::degree_t absolute_encoder_zero_position) {
    m_zero_point = absolute_encoder_zero_position;
//    m_zero_point = 0_deg;
}

void SwerveModule::SetBrakeMode(bool brake) {
    m_drive_controller.SetNeutralMode(brake ? NeutralMode::Brake : NeutralMode::Coast);
}

units::degree_t SwerveModule::GetScrub() {
    return m_scrub_metric;
}

void SwerveModule::SetScrub(units::degree_t scrub) {
    m_scrub_metric = scrub;
}

//==============================================================================
// Shuffleboard

void SwerveModule::CreateShuffleboardSummary(frc::ShuffleboardTab& shuffleboard_tab, int x_pos, int y_pos) const {
    // Create a new 9x6 grid-type layout block at the given position
    frc::ShuffleboardLayout &layout =
        shuffleboard_tab.GetLayout(m_name,frc::BuiltInLayouts::kGrid)
            .WithPosition(x_pos, y_pos)
            .WithSize(6, 6)
            // .WithProperties(wpi::StringMap<std::shared_ptr<nt::Value>>{
            //     std::make_pair("Number of columns", nt::Value::MakeDouble(3.0)),
            //     std::make_pair("Number of rows", nt::Value::MakeDouble(3.0))})
            ;

    // Populate the layout with the motor current, temperature and drive speed
    m_drive_current_widget = &layout.Add("Drive Current", 0.0).WithPosition(0, 0).WithSize(3, 2).WithWidget(frc::BuiltInWidgets::kTextView);
    m_drive_temp_widget = &layout.Add("Drive Temperature", 0.0).WithPosition(0, 1).WithSize(3, 2).WithWidget(frc::BuiltInWidgets::kTextView);
    m_drive_speed_widget = &layout.Add("Drive Speed", 0.0).WithPosition(0, 2).WithSize(3, 2).WithWidget(frc::BuiltInWidgets::kTextView);



    m_steer_current_widget = &layout.Add("Steer Current", 0.0).WithPosition(1, 0).WithSize(3, 2).WithWidget(frc::BuiltInWidgets::kTextView);
    m_steer_temp_widget = &layout.Add("Steer Temperature", 0.0).WithPosition(1, 1).WithSize(3, 2).WithWidget(frc::BuiltInWidgets::kTextView);
    m_scrub_widget = &layout.Add("Scrub", 0.0).WithPosition(1,2).WithSize(3,2).WithWidget(frc::BuiltInWidgets::kTextView);
    // m_canbus_error_widget = &layout.Add("CAN bus error", 0.0)
    //                         .WithWidget(frc::BuiltInWidgets::kBooleanBox)
    //                         .WithPosition(1, 2)
    //                         .WithSize(3, 2);
}

void SwerveModule::AddGyroSendable(frc::ShuffleboardTab& shuffleboard_tab, int x_pos, int y_pos) const {
    shuffleboard_tab.Add(m_name + " Heading", m_heading_sendable)
                    .WithPosition(x_pos, y_pos)
                    .WithSize(5, 5)
                    .WithWidget(frc::BuiltInWidgets::kGyro)
                    .WithProperties({{"Counter clockwise", nt::Value::MakeBoolean(true)}}); 
}  

void SwerveModule::CreateShuffleboardDetailedTab(std::string_view name) const {
    // Make a new shuffleboard tab
    frc::ShuffleboardTab& shuffleboard_tab = frc::Shuffleboard::GetTab(name);
    
    // Initialise the detailed widgets struct and create a shorthand pointer
    m_detailed_widgets = new DetailedWidgets;
    DetailedWidgets* dw = m_detailed_widgets;

    //Populate the tab with encoder data and heading
    dw->m_absolute_encoder_widget = &shuffleboard_tab.Add("Absolute Angle", 0.0).WithPosition(5, 0).WithSize(3, 2).WithWidget(frc::BuiltInWidgets::kTextView);
    dw->m_steer_angle_module_widget = &shuffleboard_tab.Add("Steer Angle", 0.0).WithPosition(5, 2).WithSize(3, 2).WithWidget(frc::BuiltInWidgets::kTextView);
    dw->m_zero_point_widget = &shuffleboard_tab.Add("Zero Point", 0.0).WithPosition(14, 0).WithSize(3, 2).WithWidget(frc::BuiltInWidgets::kTextView);

    dw->m_speed_widget = &shuffleboard_tab.Add("Speed", 0.0).WithPosition(17, 0).WithSize(3, 2).WithWidget(frc::BuiltInWidgets::kTextView);
    dw->m_desired_speed_widget = &shuffleboard_tab.Add("Desired Speed", 0.0).WithPosition(8, 0).WithSize(3, 2).WithWidget(frc::BuiltInWidgets::kTextView);
    dw->m_desired_angle_widget = &shuffleboard_tab.Add("Desired Angle", 0.0).WithPosition(8, 2).WithSize(3, 2).WithWidget(frc::BuiltInWidgets::kTextView);
    dw->m_steer_output_widget = &shuffleboard_tab.Add("Steer Output", 0.0).WithPosition(14, 2).WithSize(3, 2).WithWidget(frc::BuiltInWidgets::kTextView);
    dw->m_steer_position_widget = &shuffleboard_tab.Add("Steer Position", 0.0).WithPosition(11, 2).WithSize(3, 2).WithWidget(frc::BuiltInWidgets::kTextView);
    dw->m_drive_position_widget = &shuffleboard_tab.Add("Drive Position", 0.0).WithPosition(11, 0).WithSize(3, 2).WithWidget(frc::BuiltInWidgets::kTextView);

    dw->m_steer_output_graph_widget = &shuffleboard_tab.Add("Steer Graph", 0.0).WithPosition(7, 5).WithSize(7, 7).WithWidget(frc::BuiltInWidgets::kGraph);


    shuffleboard_tab.Add("Heading Sendable", m_heading_sendable)
                      .WithPosition(0, 0)
                      .WithSize(5, 5)
                      .WithWidget(frc::BuiltInWidgets::kGyro)
                      .WithProperties({{"Counter clockwise", nt::Value::MakeBoolean(true)}}); 

    dw->m_heading_graph_widget = &shuffleboard_tab.Add("Heading Graph", 0.0)
                      .WithPosition(0, 5)
                      .WithSize(7, 7)
                      .WithWidget(frc::BuiltInWidgets::kGraph);

    
    dw->m_speed_graph = &shuffleboard_tab.Add("Speed Graph", 0.0)
                      .WithPosition(21, 5)
                      .WithSize(7, 7)
                      .WithWidget(frc::BuiltInWidgets::kGraph);
    dw->m_drive_current_graph = &shuffleboard_tab.Add("Drive Current Graph", 0.0)
                      .WithPosition(28,5)
                      .WithSize(7, 7)
                      .WithWidget(frc::BuiltInWidgets::kGraph);  
    dw->m_steer_current_graph = &shuffleboard_tab.Add("Steer Current Graph", 0.0)
                      .WithPosition(14, 5)
                      .WithSize(7, 7)
                      .WithWidget(frc::BuiltInWidgets::kGraph);  
}

void SwerveModule::UpdateShuffleboard() const {
   
    frc::SwerveModuleState state = SwerveModule::GetState();

    // Get the heading of the robot and update the sendable value
    double heading_degrees = state.angle.Degrees().value();
    m_heading_sendable.Set(heading_degrees);

    // Update each of the shuffleboard widgets if they have been initialised
    if (m_drive_current_widget != nullptr) {
        m_drive_current_widget->GetEntry()->SetDouble(m_drive_controller.GetSupplyCurrent().GetValueAsDouble());
    }
    if (m_drive_temp_widget != nullptr) {
        //m_drive_temp_widget->GetEntry()->SetDouble(m_drive_controller.GetTemperature());
    }
    if (m_drive_speed_widget != nullptr) {
        m_drive_speed_widget->GetEntry()->SetDouble(units::feet_per_second_t(state.speed).value());
    }
    if (m_steer_current_widget != nullptr) {
        m_steer_current_widget->GetEntry()->SetDouble(m_steer_controller.GetSupplyCurrent().GetValueAsDouble());
    }
    if (m_steer_temp_widget != nullptr) {
        //m_steer_temp_widget->GetEntry()->SetDouble(m_steer_controller.GetTemperature());
    }
    if (m_scrub_widget != nullptr) {
        m_scrub_widget->GetEntry()->SetDouble(m_scrub_metric.value());
    }

    if (m_detailed_widgets != nullptr) {
        DetailedWidgets* dw = m_detailed_widgets;
        dw->m_zero_point_widget->GetEntry()->SetBoolean(m_can_error);
        dw->m_absolute_encoder_widget->GetEntry()->SetDouble(m_steer_encoder->GetAbsolutePosition().GetValueAsDouble());
        dw->m_steer_angle_module_widget->GetEntry()->SetDouble(state.angle.Degrees().value());
        dw->m_zero_point_widget->GetEntry()->SetDouble(units::degree_t(m_zero_point).value());
        dw->m_desired_speed_widget->GetEntry()->SetDouble(m_desired_state.speed.value());
        dw->m_desired_angle_widget->GetEntry()->SetDouble(m_desired_state.angle.Degrees().value());
        dw->m_steer_output_widget->GetEntry()->SetDouble(m_steer_controller.Get());
        dw->m_steer_output_graph_widget->GetEntry()->SetDouble(m_steer_controller.Get());
        dw->m_steer_position_widget->GetEntry()->SetDouble(m_steer_controller.GetRotorPosition().GetValueAsDouble());
        dw->m_drive_position_widget->GetEntry()->SetDouble(m_drive_controller.GetRotorPosition().GetValueAsDouble());
        dw->m_heading_graph_widget->GetEntry()->SetDouble(heading_degrees);
        dw->m_speed_widget->GetEntry()->SetDouble(units::feet_per_second_t(state.speed).value());
        dw->m_speed_graph->GetEntry()->SetDouble(units::feet_per_second_t(state.speed).value());
        dw->m_drive_current_graph->GetEntry()->SetDouble(m_drive_controller.GetSupplyCurrent().GetValueAsDouble());
        dw->m_steer_current_graph->GetEntry()->SetDouble(m_steer_controller.GetSupplyCurrent().GetValueAsDouble());
    }

}


//==============================================================================
// Simulation Operations

void SwerveModule::UpdateSimulation(units::second_t update_time) {
    // NOTE: This simulation is not intended to be realistic. The aim is to have the drivebase
    //       work in the simulator so we can test code, but we do not expect to correctly
    //       predict exact robot behaviour.

    // Get the 'sim collection' for the motors
	ctre::phoenix6::sim::TalonFXSimState& drive_sim = m_drive_controller.GetSimState();
	ctre::phoenix6::sim::TalonFXSimState& steer_sim = m_steer_controller.GetSimState();
    ctre::phoenix6::sim::CANcoderSimState& encoder_sim = m_steer_encoder->GetSimState();

    // Get the voltage being applied to the drive and steer motors
    units::volt_t drive_voltage = drive_sim.GetMotorVoltage();
    units::volt_t steer_voltage = steer_sim.GetMotorVoltage();

    // Get the motor voltage characteristic parameters. These can be measured but for now
    // assume an offset of 1V and then the Kv from the WPILib values for a Falcon500.
    using radians_per_second_per_volt_t =
        units::unit_t<units::compound_unit<units::radians_per_second,
                                           units::inverse<units::volt>>>;
    units::volt_t KV_offset = 0.0749_V;
    radians_per_second_per_volt_t k_v = frc::DCMotor::Falcon500().Kv;

    // Calculate a motor speed from the applied voltage for the drive and steer motors
    units::radians_per_second_t drive_angular_speed = units::radians_per_second_t(0);
    if (drive_voltage > KV_offset) {
        drive_angular_speed = (drive_voltage - KV_offset) * k_v;
    }
    else if (drive_voltage < -KV_offset) {
        drive_angular_speed = (drive_voltage + KV_offset) * k_v;
    }
    units::radians_per_second_t steer_angular_speed = units::radians_per_second_t(0);
    if (steer_voltage > KV_offset) {
        steer_angular_speed = (steer_voltage - KV_offset) * k_v;
    }
    else if (steer_voltage < -KV_offset) {
        steer_angular_speed = (steer_voltage + KV_offset) * k_v;
    }

    // Calculate an updated sensor position, from the current position and the velocity
    units::turn_t drive_position = m_drive_controller.GetRotorPosition().GetValue();
    drive_position += drive_angular_speed * update_time;
    units::turn_t steer_position = m_steer_controller.GetRotorPosition().GetValue();
    steer_position += steer_angular_speed * update_time;

    // Update the sensor position and velocity for the drive and steer motors
    drive_sim.SetRawRotorPosition(drive_position);
    drive_sim.SetRotorVelocity(drive_angular_speed);
    steer_sim.SetRawRotorPosition(steer_position);
    steer_sim.SetRotorVelocity(steer_angular_speed);

    // Calculate the CANcoder position from the new steer motor position
    //  - Adjust for the 4096 counts for the CANcoder versus the 2048 counts for the Falcon
    //  - Adjust for the gearing between the motor and the module axis
    //  - Convert to an absolute value by removing multiples of the 4096 counts
    units::turn_t encoder_position = steer_position / RC::kDriveBaseSteerGearRatio + m_zero_point;
    units::turn_t encoder_position_absolute = units::turn_t(KoalafiedUtilities::Modulus(encoder_position.value(), 1.0));
    encoder_sim.SetRawPosition(-encoder_position_absolute);
}


//==============================================================================
// Test Operations

void SwerveModule::TuneDriveTalonFX(double drive, double max_rpm, bool close_loop) {
    KoalafiedUtilities::TuneDriveTalonFX(&m_drive_controller, m_name.c_str(), drive, max_rpm, close_loop);
}

void SwerveModule::TuneSteerTalonFX(double drive, double max_rpm, bool close_loop) {
    KoalafiedUtilities::TuneDriveTalonFX(&m_steer_controller, (m_name + " Steer").c_str(), drive, max_rpm, close_loop);
}

void SwerveModule::ManualDriveMotors(double drive_fraction, double steer_fraction) {

    // m_drive_controller.Set(ControlMode::PercentOutput, drive_fraction);
    // m_drive_controller.Set(drive_fraction);
    m_drive_controller.SetControl(ctre::phoenix6::controls::VoltageOut(drive_fraction*12_V).WithEnableFOC(false));


    units::degree_t target_steer_motor_angle = units::turn_t(0.25 * steer_fraction * RC::kDriveBaseSteerGearRatio);
    frc::SmartDashboard::PutNumber(m_name + " Current Angle (Deg)", units::degree_t(m_steer_controller.GetRotorPosition().GetValue()).value());
    // frc::SmartDashboard::PutNumber(m_name + " Target Angle (Native)", target_steer_motor_angle_native);
    frc::SmartDashboard::PutNumber(m_name + " Target Angle (Deg)", target_steer_motor_angle.value());
    // m_steer_controller.Set(ControlMode::MotionMagic, target_steer_motor_angle_native);
    m_steer_controller.SetControl(ctre::phoenix6::controls::MotionMagicVoltage(target_steer_motor_angle).WithEnableFOC(false));

}

void SwerveModule::CharacterisationDrive(const frc::Rotation2d& angle, double drive_fraction) {

    frc::SwerveModuleState desired_state;
    desired_state.angle = angle;
    desired_state.speed = 1_mps;

    // Use the current state of the module to calculate an optimatised state. This means that the
    // module will turn the steering through the smallest angle, reversing the direction that the
    // wheel is driven if it required.
    frc::SwerveModuleState current_state = GetState();
    frc::SwerveModuleState optimized_state = frc::SwerveModuleState::Optimize(desired_state, current_state.angle.Radians());
    if (optimized_state.speed < 0_mps) {
        drive_fraction = -drive_fraction;
    }

    m_drive_controller.SetControl(ctre::phoenix6::controls::DutyCycleOut(drive_fraction).WithEnableFOC(false));

    // Steer the wheel to the desired angle, using Motion Magic for a smooth controlled motion
    SteerToAngleMotionMagic(optimized_state.angle.Radians());
}


void SwerveModule::ResetSteerEncoder() {
    m_steer_controller.SetPosition(0_deg);
}

units::volt_t SwerveModule::GetDriveVoltage() const {
    return m_drive_controller.GetMotorVoltage().GetValue();
}

units::ampere_t SwerveModule::GetDriveCurrent() const {
    return m_drive_controller.GetSupplyCurrent().GetValue();
}



//==============================================================================
// Motor Controller Setup

void SwerveModule::SteerTalonSetup() {
    ctre::phoenix6::configs::TalonFXConfiguration steer_configuration;

    steer_configuration.WithMotorOutput(ctre::phoenix6::configs::MotorOutputConfigs()
                                            .WithPeakForwardDutyCycle(1.0)
                                            .WithPeakReverseDutyCycle(-1.0));
    // No ramp rate as it interferes with the closed loop control
    //steer_configuration.closedloopRamp = RC::kDriveMotorRampRateS;
    //steer_configuration.openloopRamp = RC::kDriveMotorRampRateS;

    steer_configuration.WithCurrentLimits(ctre::phoenix6::configs::CurrentLimitsConfigs()
                                            .WithSupplyCurrentLimitEnable(true)
                                            .WithSupplyCurrentLimit(RC::kDriveMotorPeakCurrentLimitA)
                                            .WithSupplyCurrentLowerLimit(RC::kDriveMotorContinuousCurrentLimitA)
                                            .WithSupplyCurrentLowerTime(RC::kDriveMotorPeakCurrentDurationMs));
    steer_configuration.WithSlot0(ctre::phoenix6::configs::Slot0Configs()
                                    .WithKV(RC::kSteerPidF)
                                    .WithKP(0.2002)
                                    .WithKI(RC::kSteerPidI)
                                    .WithKD(RC::kSteerPidD));
    steer_configuration.WithSlot1(ctre::phoenix6::configs::Slot1Configs()
                                    .WithKV(0.0)
                                    .WithKP(1.001)
                                    .WithKI(0.0)
                                    .WithKD(0.0));

    units::turns_per_second_t max_steer_tps = RC::kModuleMaxAngularVelocity.convert<units::turns_per_second>();
    units::turns_per_second_t max_steer_motor_tps = max_steer_tps * RC::kDriveBaseSteerGearRatio;
    units::turns_per_second_squared_t max_steer_motor_tpsq = 4.0 * max_steer_motor_tps/1_s;
    //max_steer_motor_rpm *= 0.2;
    steer_configuration.WithMotionMagic(ctre::phoenix6::configs::MotionMagicConfigs()
                                            .WithMotionMagicCruiseVelocity(max_steer_motor_tps)
                                            .WithMotionMagicAcceleration(max_steer_motor_tpsq));

    auto error = m_steer_controller.GetConfigurator().Apply(steer_configuration);
    if (error != 0) {
        std::cout << "Configuration of the " << m_name << " steer Falcon failed with code:  " << error << "\n";
        m_can_error = true;
    }
    m_steer_controller.SetNeutralMode(NeutralMode::Brake);
}

void SwerveModule::DriveTalonSetup() {
    ctre::phoenix6::configs::TalonFXConfiguration drive_configuration;
    
    drive_configuration.WithMotorOutput(ctre::phoenix6::configs::MotorOutputConfigs()
                                            .WithPeakForwardDutyCycle(1.0)
                                            .WithPeakReverseDutyCycle(-1.0));
    drive_configuration.WithOpenLoopRamps(ctre::phoenix6::configs::OpenLoopRampsConfigs()
                                                .WithDutyCycleOpenLoopRampPeriod(RC::kDriveMotorRampRateS));
    drive_configuration.WithClosedLoopRamps(ctre::phoenix6::configs::ClosedLoopRampsConfigs()
                                                .WithDutyCycleClosedLoopRampPeriod(RC::kDriveMotorRampRateS));

    drive_configuration.WithCurrentLimits(ctre::phoenix6::configs::CurrentLimitsConfigs()
                                        .WithSupplyCurrentLimitEnable(true)
                                        .WithSupplyCurrentLimit(RC::kDriveMotorPeakCurrentLimitA)
                                        .WithSupplyCurrentLowerLimit(RC::kDriveMotorContinuousCurrentLimitA)
                                        .WithSupplyCurrentLowerTime(RC::kDriveMotorPeakCurrentDurationMs));

    drive_configuration.WithSlot0(ctre::phoenix6::configs::Slot0Configs()
                                    .WithKV(RC::kDriveBasePidV)
                                    .WithKP(RC::kDriveBasePidP)
                                    .WithKI(RC::kDriveBasePidI)
                                    .WithKD(RC::kDriveBasePidD));
    
    drive_configuration.WithVoltage(ctre::phoenix6::configs::VoltageConfigs()
                                        .WithPeakForwardVoltage(12.0_V)
                                        .WithPeakReverseVoltage(12.0_V));

#ifdef _WIN32
    // For simulation tweak the PID values to be stable. This is necessary because the simulation is not
    // very realistic.
    drive_configuration.Slot0.kP = 0.01;
#endif


    auto error = m_drive_controller.GetConfigurator().Apply(drive_configuration);
    if (error != 0) {
        std::cout << "Configuration of the " << m_name << " drive Falcon failed with code:  " << error << "\n";
        m_can_error = true;
    }
    m_drive_controller.SetNeutralMode(NeutralMode::Coast);
}

//==============================================================================
// Steering Control

void SwerveModule::SteerToAngleMotionMagic(units::radian_t target_angle_module) {
    // When controlling the module steering we want to do two things:
    //  1. Using the CANcoder on the module shaft to know the absolute angle of the module relative
    //     to the robot.
    //  2. Use the fast control loop on the FalconFX to control the motor.
    // To do this we use the CANcoder to calculate an angle delta that we need to move and
    // then use the FalconFX to control the movement by this delta. Note that we use 'Motion Magic'
    // control as this gives smooth controlled motion, accoring to a configured maximum speed and acceleration.

    // Get the current state of the module
    frc::SwerveModuleState current_state = GetState();

    // Express the required change in angle for the module in the smallest number of rotations
    units::degree_t angle_delta = units::degree_t(target_angle_module) - current_state.angle.Degrees();
    while (angle_delta > 180.0_deg) angle_delta -= 360.0_deg;
    while (angle_delta <= -180.0_deg) angle_delta += 360.0_deg;

    // Convert the angle that we need to move into a delta in the steer Falcon encoder units.
    // This happens in 2 steps:
    //  1. Convert to a motor delta, using the gearing ratio
    //  2. Convert the angle into native units, using the encoder counts per 360 degree rotation
    units::degree_t motor_angle_delta = angle_delta * RC::kDriveBaseSteerGearRatio;
    // double motor_angle_delta_native = -motor_angle_delta * RC::kTalonFXEnocderCounts / 360_deg;

    // Calculate the target angle of the motor and set the steer controller to move to that angle
    // double target_steer_motor_angle_native = m_steer_controller.GetSelectedSensorPosition() + motor_angle_delta_native;
    units::degree_t target_steer_motor_angle = m_steer_controller.GetRotorPosition().GetValue() - motor_angle_delta;


    if (units::math::abs(angle_delta) > 4_deg) {
        // If the error is large drive to the position with Motion Magic so that the speed and
        // acceleration are limited.
        // m_steer_controller.SelectProfileSlot(0, RC::kTalonPidIdx);
        // m_steer_controller.Set(ControlMode::MotionMagic,target_steer_motor_angle);
        m_steer_controller.SetControl(ctre::phoenix6::controls::MotionMagicVoltage(target_steer_motor_angle)
                                        .WithSlot(1)
                                        .WithEnableFOC(false));
    }
    else {
        // If the error is small drive to the position with position close loop control and a different
        // set of PID parameters to minimise the error.
        // m_steer_controller.SelectProfileSlot(1, RC::kTalonPidIdx);
        // m_steer_controller.Set(ControlMode::Position, target_steer_motor_angle);
        m_steer_controller.SetControl(ctre::phoenix6::controls::PositionVoltage(target_steer_motor_angle)
                                        .WithSlot(1)
                                        .WithEnableFOC(false));
    }
}
