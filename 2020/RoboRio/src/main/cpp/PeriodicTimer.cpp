//==============================================================================
// PeriodicTimer.cpp
//==============================================================================

#include "PeriodicTimer.h"
#include <iostream>
#include <iomanip>



//==============================================================================
// Construction

PeriodicTimer::PeriodicTimer(const char* name) {
    m_name = name;
    m_delta_index = -1;
    m_period_timer.Start();
    m_processing_timer.Start();
}

PeriodicTimer::~PeriodicTimer() {
}

//==========================================================================
// Operation

void PeriodicTimer::Init() {
    // Set index such that the first value is not recorded
    m_delta_index = -1;
}

void PeriodicTimer::PeriodicStart() {
    m_processing_timer.Reset();
}

void PeriodicTimer::PeriodicEnd() {
    // The first time this function is called just reset the timer and set the index to the start of the delta array
    if (m_delta_index == -1) {
        m_period_timer.Reset();
        m_delta_index = 0;
        return;
    }

    // Record the time delta since the last time this function was called and resets the timer
    // Record the time spent processing for this period
    m_period_deltas[m_delta_index] = m_period_timer.Get();
    m_period_timer.Reset();
    m_processing_times[m_delta_index] = m_processing_timer.Get();
    m_delta_index++;

    // If the delta array is full, log information and reset the index
    if (m_delta_index == TOTAL_DELTAS) {
//      LogAllDeltas();
//        LogSummary(false);
        m_delta_index = 0;
    }
}

//==========================================================================
// Logging

void PeriodicTimer::LogAllDeltas() {
    // Log all delta values on a single line
    printf("%s deltas: ", m_name);
    for (int i = 0; i< TOTAL_DELTAS; i++) {
        printf("%d, ", (int)(m_period_deltas[i]*1000.0));
    }
    printf("\n");
}

void PeriodicTimer::LogSummary(bool warnings_only) {
    // Iterate the deltas and finding the minimum, maximum and average
    double sum_period = 0;
    double min_period = m_period_deltas[0];
    double max_period = m_period_deltas[0];
    double sum_processing = 0;
    double min_processing = m_processing_times[0];
    double max_processing = m_processing_times[0];
    for (int i = 0; i< TOTAL_DELTAS; i++) {
        sum_period += m_period_deltas[i];
        if (min_period > m_period_deltas[i]) {
            min_period = m_period_deltas[i];
        }
        if (max_period < m_period_deltas[i]) {
            max_period = m_period_deltas[i];
        }
        sum_processing += m_processing_times[i];
        if (min_processing > m_processing_times[i]) {
            min_processing = m_processing_times[i];
        }
        if (max_processing < m_processing_times[i]) {
            max_processing = m_processing_times[i];
        }
    }
    double average_period = sum_period/TOTAL_DELTAS;
    double average_processing = sum_processing/TOTAL_DELTAS;

    // Convert the minimum, maximum and average to milliseconds
    average_period *= 1000.0;
    min_period *= 1000.0;
    max_period *= 1000.0;
    average_processing *= 1000.0;
    min_processing *= 1000.0;
    max_processing *= 1000.0;
    
    // Do warning messages if things are longer then expected
    const double kWarningAverage = 23;
    const double kWarningMax = 32;
    if (average_period > kWarningAverage) {
        std::cout << std::setprecision(3) << m_name << " WARNING: High Average Period " << average_period << "ms\n";
    }
    if (max_period > kWarningMax) {
        std::cout << std::setprecision(3) << m_name << " WARNING: High Max Period " << max_period << "ms\n";
    }
    if (average_processing > kWarningAverage) {
        std::cout << std::setprecision(3) << m_name << " WARNING: High Average Processing " << average_processing << "ms\n";
    }
    if (max_processing > kWarningMax) {
        std::cout << std::setprecision(3) << m_name << " WARNING: High Max Processing " << max_processing << "ms\n";
    }

    // If not only doing warnings then log everything
    if (!warnings_only) {
    	std::cout << std::setprecision(3) << m_name << " Average Period " << average_period << " Min " << min_period << " Max " << max_period << "\n";
    	std::cout << std::setprecision(3) << m_name << " Average Processing " << average_processing << " Min " << min_processing << " Max " << max_processing << "\n";
    }
}

