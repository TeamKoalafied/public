//==============================================================================
// SwerveSteeringCalibration.h
//==============================================================================

#pragma once

#include <units/angle.h>

class SwerveSteeringCalibration {
public:
    //==========================================================================
    // Constants

    // File used for steering calibration.
    const char* CALIBRATION_FILENAME = "SwerveCalibration.txt";


    //==========================================================================
    // Construction

    // Constructor
    //
    // left_front_angle - Calibration angle for the left front swerve module
    // right_front_angle - Calibration angle for the right front swerve module
    // left_back_angle - Calibration angle for the left back swerve module
    // right_back_angle - Calibration angle for the right back swerve module
    SwerveSteeringCalibration(units::degree_t left_front_angle = 0_deg, units::degree_t right_front_angle = 0_deg,
                            units::degree_t left_back_angle = 0_deg, units::degree_t right_back_angle = 0_deg) {
        m_left_front_angle = left_front_angle;
        m_right_front_angle = right_front_angle;
        m_left_back_angle = left_back_angle;
        m_right_back_angle = right_back_angle;
    }


    //==========================================================================
    // Properties

    // Calibration angle for the left front swerve module
    units::degree_t GetLeftFrontAngle() const { return m_left_front_angle; }

    // Calibration angle for the right front swerve module
    units::degree_t GetRightFrontAngle() const { return m_right_front_angle; }

    // Calibration angle for the left back swerve module
    units::degree_t GetLeftBackAngle() const { return m_left_back_angle; }

    // Calibration angle for the right back swerve module
    units::degree_t GetRightBackAngle() const { return m_right_back_angle; }


    //==========================================================================
    // Operations

    // Save this steering calibrations to a timestamped file
    void Save();

    // Load this steering calibrations from the file
    //
    // Returns whether loading is successful
    bool Load();

    // Log this steering calibrations to the console output
    void Log();

private:
    //==========================================================================
    // Member Variables

    units::degree_t m_left_front_angle;         // Calibration angle for the left front swerve module
    units::degree_t m_right_front_angle;        // Calibration angle for the right front swerve module
    units::degree_t m_left_back_angle;          // Calibration angle for the left back swerve module
    units::degree_t m_right_back_angle;         // Calibration angle for the right back swerve module
};