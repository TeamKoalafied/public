//==============================================================================
// Logging.h
//==============================================================================

#pragma once

#include <fstream>
#include <filesystem>

namespace frc { class Pose2d; }

namespace Logging
{

//==============================================================================
// Utility Helpers

// Get the base path for the platform. For RoboRIO this is users home director ("/home/lvuser/")
// and on Windows it is the root of the RoboRIO code.
std::filesystem::path GetPathformBasePath();


// CsvFile provides a simple interface for writing CSV files. It handles the separators
// and quoting the values. It does not do anything very clever.
// NOTE: Currently quotes in the value are not escaped!
//
// When using the streaming operators each invocation of an operator is a single value.
// The newline must be a separate value. For example,
//
// file << "Index" << 1 << "value" << 7.5 << "\n";
//
// Results in the following line in the file
//
// "Index","1","value","7.5"
//
class CsvFile {
public:
    //==========================================================================
    // Construction

    // Default constructor
    CsvFile();

    //==========================================================================
    // File Open/Close

    // Open the file
    //
    // filename - Full path of the file to open
    // mode - Mode open the file as. Same meaning as for std::ofstream.
    void Open(const char* filename, std::ios_base::openmode mode);

    // Open the file for logging, putting the file in a logging directory
    //
    // filename - Name of the file to open, relative to the logging directory
    // mode - Mode open the file as. Same meaning as for std::ofstream.
    void OpenLogFile(const char* filename, std::ios_base::openmode mode);

    // Close the file
    void Close();

    // Whether the file failed to open
    bool Fail();

    //==========================================================================
    // Stream Operations

    CsvFile& operator<<(double value);
    CsvFile& operator<<(int value);
    CsvFile& operator<<(const char* value);
    CsvFile& operator<<(std::string value);

    // Set the precision for writing floating point (i.e. double) numbers
    //
    // precision - Number of digits of precision to use
    // fixed - Whether to use 'fixed' output, meaning we get 'precision' decimal
    //      places, otherwise it is floating output and we get 'precision'
    //      significant figures.
    void SetPrecision(int precision, bool fixed);

private:
    //==========================================================================
    // Implementation

    // Add a separator to the current line if required
    void AddSeparator() {
        if (!m_at_line_start) {
            m_file << ",";
        }
        else {
            m_at_line_start = false;
        }
    }


    //==========================================================================
    // Member Variables

    std::ofstream m_file;           // Underlying file stream to write to
    bool m_at_line_start = true;    // Flag indicating if we are at the start of a line
};


//==============================================================================
// FRC Streaming Operators

std::ostream& operator<<(std::ostream& os, const frc::Pose2d& pose);
}