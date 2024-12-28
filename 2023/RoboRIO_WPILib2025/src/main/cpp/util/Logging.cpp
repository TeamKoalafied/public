//==============================================================================
// Loggin.cpp
//==============================================================================


#include "Logging.h"

#include <iomanip>
#include <iostream>
#ifdef _WIN32
#include <Windows.h>
#endif
#include <filesystem>

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
    // Deterimine the directory to put the log file in depending on the platform.
    std::filesystem::path full_file_path;
#ifdef _WIN32
    // For Windows get the executable path and go up a lot of directory levels to
    // get the RoboRIO directory.
    CHAR exe_path[MAX_PATH];
    GetModuleFileName(NULL, exe_path, MAX_PATH);
    full_file_path = std::filesystem::path(exe_path);
    full_file_path = full_file_path.parent_path().parent_path().parent_path().parent_path().
                                    parent_path().parent_path().parent_path();
#else
    // On the RoboRIO start the users home directory
    full_file_path = std::filesystem::path("/home/lvuser/");
#endif

    // Use a 'logging' sub-directory, ensuring it exists. This is neater that filling
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
