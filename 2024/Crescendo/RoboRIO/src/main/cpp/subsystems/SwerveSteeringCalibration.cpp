//==============================================================================
// SwerveSteeringCalibration.cpp
//==============================================================================

#include "SwerveSteeringCalibration.h"

#include "../util/Logging.h"

#include <iostream>
#include <iomanip>


void SwerveSteeringCalibration::Save() {
    auto now = std::chrono::system_clock::now();

    auto in_time_t = std::chrono::system_clock::to_time_t(now);
    std::stringstream datetime;
    datetime << std::put_time(std::localtime(&in_time_t), "_%Y_%m_%d_%H_%M_%S");

    // Open the calibration file for writing discarding the existing contents
    std::filesystem::path calibration_filepath = Logging::GetPathformBasePath();
    std::filesystem::path calibration_filename(CALIBRATION_FILENAME);
    calibration_filepath /= calibration_filename.stem().string() + datetime.str() + calibration_filename.extension().string();

    std::ofstream calibration_file;
    calibration_file.open(calibration_filepath, std::ios::out | std::ios::trunc);
    if (calibration_file.fail()) {
        std::cout << "ERROR: Failed to open calibration file for writing " << calibration_filepath << "\n";
        return;
    }

    // Write the values one per line, with a comment to explain the format
    calibration_file << "# This file holds the swerve drive calibration with one value per line in\n";
    calibration_file << "# the order left front, right front, left back, right back\n";
    calibration_file << std::setprecision(4) << m_left_front_angle.value() << "\n";
    calibration_file << std::setprecision(4) << m_right_front_angle.value() << "\n";
    calibration_file << std::setprecision(4) << m_left_back_angle.value() << "\n";
    calibration_file << std::setprecision(4) << m_right_back_angle.value() << "\n";

    std::cout << "Steering calibration  written to " << calibration_filepath << "\n";
}

bool SwerveSteeringCalibration::Load() {
    // Open the calibration file for reading
    std::filesystem::path calibration_filepath = Logging::GetPathformBasePath();
    calibration_filepath /= CALIBRATION_FILENAME;
    std::ifstream calibration_file;
    calibration_file.open(calibration_filepath, std::ios::in);
    if (calibration_file.fail()) {
        std::cout << "ERROR: Failed to open calibration file for reading" << calibration_filepath << "\n";
        return false;
    }

    // Read the file line by line, looking for 4 calibration angle values
    std::string line;
    int index = 0;
    while (std::getline(calibration_file, line)) {
        // Skip comment lines and empty lines
        if (line[0] == '#') continue;
        if (line.empty()) continue;

        // Try to read the line as a single number. If not valid there is a problem.
        char* end;
        double value = strtod(line.c_str(), &end);
        if (*end != '\0') {
            std::cout << "ERROR: Calibration angle number " << index << " is illegal\n";
            std::cout << line << "\n";
            return false;
        }

        // Assign the number read to the appropriate calibration angle
        switch (index) {
            case 0: m_left_front_angle = units::degree_t(value); break;
            case 1: m_right_front_angle = units::degree_t(value); break;
            case 2: m_left_back_angle = units::degree_t(value); break;
            case 3: m_right_back_angle = units::degree_t(value); break;
        }
        
        // Move to the next corner. If we have 4 we are done.
        index++;
        if (index == 4) return true;
    }

    // Did not find 4 values
    std::cout << "ERROR: Calibration file only had " << index << " values\n";
    return false;
}

void SwerveSteeringCalibration::Log() {
    std::cout << "Left Front:  " << m_left_front_angle.value() << "\n";
    std::cout << "right Front: " << m_right_front_angle.value() << "\n";
    std::cout << "Left Back:   " << m_left_back_angle.value() << "\n";
    std::cout << "Right Back:  " << m_right_back_angle.value() << "\n";
}
