//==============================================================================
// Logging.cpp
//==============================================================================


#include "Logging.h"

#include <frc/geometry/Pose2d.h>

#include <iomanip>
#include <iostream>
#ifdef _WIN32
#include <Windows.h>
#endif
#include <filesystem>


//==============================================================================
// Utility Helpers

// Get the base path for the platform. For RoboRIO this is users home director ("/home/lvuser/")
// and on Windows it is the root of the RoboRIO code.
std::filesystem::path Logging::GetPathformBasePath() {
#ifdef _WIN32
    // For Windows get the executable path and go up a lot of directory levels to
    // get the root of the RoboRIO code.
    CHAR exe_path[MAX_PATH];
    GetModuleFileName(NULL, exe_path, MAX_PATH);
    std::filesystem::path full_file_path = std::filesystem::path(exe_path);
    full_file_path = full_file_path.parent_path().parent_path().parent_path().parent_path().
                                    parent_path().parent_path().parent_path();
    return full_file_path;
#else
    // On the RoboRIO the base is the users home directory
    return std::filesystem::path("/home/lvuser/");
#endif

}



//==============================================================================
// Construction

Logging::CsvFile::CsvFile() {

}


//==============================================================================
// File Open/Close

void Logging::CsvFile::Open(const char* filename, std::ios_base::openmode mode) {
    m_file.open(filename, mode);
    m_at_line_start = true;
}

void Logging::CsvFile::OpenLogFile(const char* filename, std::ios_base::openmode mode) {
    // Use a 'logging' sub-directory in the platform base directory, ensuring it exists. This is neater that filling
    std::filesystem::path full_file_path = GetPathformBasePath();
    full_file_path /= "logging";
    std::filesystem::create_directory(full_file_path);

    // Add the filename to form the fill path and open the file.
    // NOTE: reinterpret_cast is needed due to crazy C++ stuff. Should work out how to make
    // is nice.
    full_file_path /= filename;
    std::cout << "Opening log file " << full_file_path << "\n";
    Open(reinterpret_cast<const char*>(full_file_path.u8string().c_str()), mode);
}

bool Logging::CsvFile::Fail() {
    return m_file.fail();
}

void Logging::CsvFile::Close() {
    m_file.close();
}


//==============================================================================
// Stream Operations

Logging::CsvFile& Logging::CsvFile::operator<<(double value) {
    // Add a separator and write the value in quotes
    AddSeparator();
    m_file << "\"" << value << "\"";
    return *this;
}

Logging::CsvFile& Logging::CsvFile::operator<<(int value) {
    // Add a separator and write the value in quotes
    AddSeparator();
    m_file << "\"" << value << "\"";
    return *this;
}

Logging::CsvFile& Logging::CsvFile::operator<<(std::string value) {
    return operator<<(value.c_str());
}

Logging::CsvFile& Logging::CsvFile::operator<<(const char* value) {
    // Test if this is a newline
    if (value[0] == '\n' && value[1] == '\0') {
        // Terminate the line and record we are starting another
        m_file << "\n";
        m_at_line_start = true;
    } else {
        // Add a separator and write the value in quotes
        AddSeparator();
        m_file << "\"" << value << "\"";
    }
    return *this;
}

void Logging::CsvFile::SetPrecision(int precision, bool fixed) {
    if (fixed) {
        m_file << std::fixed;
    } else {
    	m_file << std::defaultfloat;
    }

    m_file << std::setprecision(precision);
}


//==============================================================================
// FRC Streaming Operators

std::ostream& Logging::operator<<(std::ostream& os, const frc::Pose2d& pose) {
    os << "[X: " << pose.X().value() << " Y: " << pose.Y().value() << " Rot: " << pose.Rotation().Degrees().value() << "]";
    return os;
}
