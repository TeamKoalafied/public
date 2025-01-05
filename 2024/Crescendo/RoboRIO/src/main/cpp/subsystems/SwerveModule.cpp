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
        m_drive_controller.SetControl(ctre::phoenix6::controls::DutyCycleOut(0.0, false));
        if (!dry_steer_allowed) {
            m_steer_controller.SetControl(ctre::phoenix6::controls::DutyCycleOut(0.0, false));
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
        m_target_drive_motor_rpm = drive_motor_rpm;
        // double drive_motor_native = KoalafiedUtilities::TalonFXVelocityRpmToNative(drive_motor_rpm);
        // m_drive_controller.Set(ControlMode::Velocity, drive_motor_native);
    }

    // Steer the wheel to the desired angle, using Motion Magic for a smooth controlled motion
    SteerToAngleMotionMagic(optimized_state.angle.Radians());

    // If not logging yet open the file and write the header
    const bool LOG_SWERVE_MODULE = false;
    if (LOG_SWERVE_MODULE && m_csv_log_file == nullptr) {
        const char* const RESULT_FILENAME = "SwerveModule.csv";
        m_csv_log_file = new Logging::CsvFile();
        m_csv_log_file->OpenLogFile(RESULT_FILENAME, std::ios::out | std::ios::app);
        if (m_csv_log_file->Fail()) {
            // If opening the file fails stop logging
            std::cout << "ERROR Failed to open log file\n";
            delete m_csv_log_file;
            m_csv_log_file = nullptr;
        }
        else {
            // *m_csv_log_file << "Steering" << "\n";
            // *m_csv_log_file << "Desired" << "Current" << "MTarget" << "MCurrent" << "Voltage" << "Speed" << "Position" << "\n";
            *m_csv_log_file << "Speed" << "\n";
            *m_csv_log_file << "Desired" << "Current" << "MTarget" << "MCurrent" << "Voltage" << "Speed" << "Position" << "\n";
        }
    }
}

units::degree_t SwerveModule::GetAbsoluteSteerEncoderPosition() {
    if (m_steer_encoder == nullptr) return 0_deg;

    return m_steer_encoder->GetAbsolutePosition().GetValue();
}

void SwerveModule::SetSteerEncoderCalibration(units::degree_t absolute_encoder_zero_position) {
    m_zero_point = absolute_encoder_zero_position;
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
    m_drive_current_widget = &layout.Add("Drive Current", 0.0).WithPosition(0, 0).WithSize(3, 2);
    m_drive_temp_widget = &layout.Add("Drive Temperature", 0.0).WithPosition(0, 1).WithSize(3, 2);
    m_drive_speed_widget = &layout.Add("Drive Speed", 0.0).WithPosition(0, 2).WithSize(3, 2);



    m_steer_current_widget = &layout.Add("Steer Current", 0.0).WithPosition(1, 0).WithSize(3, 2);
    m_steer_temp_widget = &layout.Add("Steer Temperature", 0.0).WithPosition(1, 1).WithSize(3, 2);
    m_scrub_widget = &layout.Add("Scrub", 0.0).WithPosition(1,2).WithSize(3,2);
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
    dw->m_absolute_encoder_widget = &shuffleboard_tab.Add("Absolute Angle", 0.0).WithPosition(5, 0).WithSize(3, 2);
    dw->m_steer_angle_module_widget = &shuffleboard_tab.Add("Steer Angle", 0.0).WithPosition(5, 2).WithSize(3, 2);
    dw->m_zero_point_widget = &shuffleboard_tab.Add("Zero Point", 0.0).WithPosition(14, 0).WithSize(3, 2);

    dw->m_speed_widget = &shuffleboard_tab.Add("Speed", 0.0).WithPosition(17, 0).WithSize(3, 2);
    dw->m_desired_speed_widget = &shuffleboard_tab.Add("Desired Speed", 0.0).WithPosition(8, 0).WithSize(3, 2);
    dw->m_desired_angle_widget = &shuffleboard_tab.Add("Desired Angle", 0.0).WithPosition(8, 2).WithSize(3, 2);
    dw->m_steer_output_widget = &shuffleboard_tab.Add("Steer Output", 0.0).WithPosition(14, 2).WithSize(3, 2);
    dw->m_steer_position_widget = &shuffleboard_tab.Add("Steer Position", 0.0).WithPosition(11, 2).WithSize(3, 2);
    dw->m_drive_position_widget = &shuffleboard_tab.Add("Drive Position", 0.0).WithPosition(11, 0).WithSize(3, 2);

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
        // m_drive_speed_widget->GetEntry()->SetDouble(units::feet_per_second_t(state.speed).value());
        m_drive_speed_widget->GetEntry()->SetDouble(m_drive_controller.Get());
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

    // // Get the voltage being applied to the drive and steer motors
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

    // If logging add a new record to the file
    if (m_csv_log_file != nullptr) {
        // *m_csv_log_file << "Desired" << "Current" << "MTarget" << "MCurrent" << "Voltage" << "Speed" << "Position" << "\n";

        // *m_csv_log_file << m_desired_state.angle.Degrees().value();
        // *m_csv_log_file << GetState().angle.Degrees().value();
        // *m_csv_log_file << m_target_steer_motor_angle.value();
        // *m_csv_log_file << ((units::degree_t)m_steer_controller.GetPosition().GetValue()).value();
        // *m_csv_log_file << steer_voltage.value();
        // *m_csv_log_file << steer_angular_speed.value();
        // *m_csv_log_file << ((units::degree_t)steer_position).value();
        // *m_csv_log_file << "\n";

        // *m_csv_log_file << "Desired" << "Current" << "MTarget" << "MCurrent" << "Voltage" << "Speed" << "Position" << "\n";

        *m_csv_log_file << m_desired_state.speed.value();
        *m_csv_log_file << GetState().speed.value();
        *m_csv_log_file << m_target_drive_motor_rpm.value();
        *m_csv_log_file << ((units::revolutions_per_minute_t)m_drive_controller.GetRotorVelocity().GetValue()).value();
        *m_csv_log_file << drive_voltage.value();
        *m_csv_log_file << ((units::revolutions_per_minute_t)drive_angular_speed).value();
        *m_csv_log_file << ((units::degree_t)drive_position).value();
        *m_csv_log_file << "\n";
    }
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
    // If logging close the file
    if (m_csv_log_file != nullptr) {
        *m_csv_log_file << "\n";
        m_csv_log_file->Close();
        delete m_csv_log_file;
        m_csv_log_file = nullptr;
    }

    m_drive_controller.SetControl(ctre::phoenix6::controls::VoltageOut(drive_fraction*12_V, false));


    units::degree_t target_steer_motor_angle = units::turn_t(0.25 * steer_fraction * RC::kDriveBaseSteerGearRatio);
    frc::SmartDashboard::PutNumber(m_name + " Current Angle (Deg)", units::degree_t(m_steer_controller.GetRotorPosition().GetValue()).value());
    frc::SmartDashboard::PutNumber(m_name + " Target Angle (Deg)", target_steer_motor_angle.value());
    m_steer_controller.SetControl(ctre::phoenix6::controls::MotionMagicVoltage(target_steer_motor_angle, false));

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

    m_drive_controller.SetControl(ctre::phoenix6::controls::DutyCycleOut(drive_fraction, false));

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

    steer_configuration.WithCurrentLimits(ctre::phoenix6::configs::CurrentLimitsConfigs()
                                            .WithSupplyCurrentLimitEnable(true)
                                            .WithSupplyCurrentLimit(RC::kDriveMotorPeakCurrentLimitA)
                                            .WithSupplyCurrentThreshold(RC::kDriveMotorContinuousCurrentLimitA)
                                            .WithSupplyTimeThreshold(RC::kDriveMotorPeakCurrentDurationS));

    steer_configuration.WithSlot0(ctre::phoenix6::configs::Slot0Configs()
                                    .WithKV(RC::kSteerPidF)
                                    .WithKP(0.2002)
                                    .WithKI(RC::kSteerPidI)
                                    .WithKD(RC::kSteerPidD));
    // steer_configuration.WithSlot0(ctre::phoenix6::configs::Slot0Configs()
    //                                 .WithKV(RC::kSteerPidF)
    //                                 .WithKP(0.2002)
    //                                 .WithKI(RC::kSteerPidI)
    //                                 .WithKD(RC::kSteerPidD));
 
    steer_configuration.WithSlot1(ctre::phoenix6::configs::Slot1Configs()
                                    .WithKV(0.0)
                                    .WithKP(0.3)
                                    .WithKI(0.0)
                                    .WithKD(0.015));

   // double max_steer_rps = RC::kModuleMaxAngularVelocity / (2.0_rad_per_s * std::numbers::pi);
    double max_steer_rps = 2.0;
    double max_steer_motor_rps = max_steer_rps * RC::kDriveBaseSteerGearRatio;
    double max_steer_motor_acceleration = max_steer_motor_rps / 0.150;
    //max_steer_motor_rpm *= 0.2;
    steer_configuration.WithMotionMagic(ctre::phoenix6::configs::MotionMagicConfigs()
                                            .WithMotionMagicCruiseVelocity(max_steer_motor_rps)
                                            .WithMotionMagicAcceleration(max_steer_motor_acceleration));


#ifdef _WIN32
    // For simulation tweak the PID values to be stable. This is necessary because the simulation is not
    // very realistic.
    steer_configuration.Slot1.kD = 0.0;
#endif

    auto error = m_steer_controller.GetConfigurator().Apply(steer_configuration);
    if (error != 0) {
        std::cout << "Configuration of the " << m_name << " steer Falcon failed with code:  " << error << "\n";
        m_can_error = true;
    }
    m_steer_controller.SetNeutralMode(ctre::phoenix6::signals::NeutralModeValue::Brake);
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
                                        .WithSupplyCurrentThreshold(RC::kDriveMotorContinuousCurrentLimitA)
                                        .WithSupplyTimeThreshold(RC::kDriveMotorPeakCurrentDurationS));

    drive_configuration.WithSlot0(ctre::phoenix6::configs::Slot0Configs()
                                    .WithKV(RC::kDriveBasePidV)
                                    .WithKP(RC::kDriveBasePidP)
                                    .WithKI(RC::kDriveBasePidI)
                                    .WithKD(RC::kDriveBasePidD));
    
    drive_configuration.WithVoltage(ctre::phoenix6::configs::VoltageConfigs()
                                        .WithPeakForwardVoltage(12.0)
                                        .WithPeakReverseVoltage(12.0));

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
    m_drive_controller.SetNeutralMode(ctre::phoenix6::signals::NeutralModeValue::Coast);

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

    bool use_motion_magic = false;

    if (units::math::abs(angle_delta) < 1_deg) {
        m_steer_controller.SetControl(ctre::phoenix6::controls::DutyCycleOut(0.0, false));
    } else if (use_motion_magic) {
        m_steer_controller.SetControl(ctre::phoenix6::controls::MotionMagicVoltage(target_steer_motor_angle)
                                    .WithSlot(0)
                                    .WithEnableFOC(false));
    } else {
        m_steer_controller.SetControl(ctre::phoenix6::controls::PositionDutyCycle(target_steer_motor_angle)
                                    .WithSlot(1)
                                    .WithEnableFOC(false));
    }

    m_target_steer_motor_angle = target_steer_motor_angle;

// #ifdef _WIN32
//     m_steer_controller.SetControl(ctre::phoenix6::controls::MotionMagicVoltage(target_steer_motor_angle)
//                                     .WithSlot(0)
//                                     .WithEnableFOC(false));
// #else
//     if (units::math::abs(angle_delta) > 4_deg) {
//         // If the error is large drive to the position with Motion Magic so that the speed and
//         // acceleration are limited.
//         m_steer_controller.SetControl(ctre::phoenix6::controls::MotionMagicVoltage(target_steer_motor_angle)
//                                         .WithSlot(0)
//                                         .WithEnableFOC(false));
//     }
//     else {
//         // If the error is small drive to the position with position close loop control and a different
//         // set of PID parameters to minimise the error.
//         m_steer_controller.SetControl(ctre::phoenix6::controls::PositionVoltage(target_steer_motor_angle)
//                                         .WithSlot(1)
//                                         .WithEnableFOC(false));
//     }
// #endif
}
