//==============================================================================
// PeriodicTimer.cpp
//==============================================================================

#include <PeriodicTimer.h>
#include <iostream>
#include <iomanip>

//==============================================================================
// Construction

PeriodicTimer::PeriodicTimer() {
    m_delta_index = -1;
    m_timer.Start();
}

PeriodicTimer::~PeriodicTimer() {
}

//==========================================================================
// Operation

void PeriodicTimer::Init() {
    // Set index such that the first value is not recorded
    m_delta_index = -1;
}

void PeriodicTimer::Periodic() {
    // The first time this function is called just reset the timer and set the index to the start of the delta array
    if (m_delta_index == -1) {
        m_timer.Reset();
        m_delta_index = 0;
        return;
    }

    // Record the time delta since the last time this function was called and resets the timer
    m_period_deltas[m_delta_index++] = m_timer.Get();
    m_timer.Reset();

    // If the delta array is full, log information and reset the index
    if (m_delta_index == TOTAL_DELTAS) {
//      LogAllDeltas();
        LogSummary(true);
        m_delta_index = 0;
    }
}

//==========================================================================
// Logging

void PeriodicTimer::LogAllDeltas() {
    // Log all delta values on a single line
    printf("TeleopPeriodic deltas: ");
    for (int i = 0; i< TOTAL_DELTAS; i++) {
        printf("%d, ", (int)(m_period_deltas[i]*1000.0));
    }
    printf("\n");
}

void PeriodicTimer::LogSummary(bool warnings_only) {
    // Iterate the deltas and finding the minimum, maximum and average
    double sum = 0;
    double min = m_period_deltas[0];
    double max = m_period_deltas[0];
    for (int i = 0; i< TOTAL_DELTAS; i++) {
        sum += m_period_deltas[i];
        if (min > m_period_deltas[i]) {
            min = m_period_deltas[i];
        }
        if (max < m_period_deltas[i]) {
            max = m_period_deltas[i];
        }
    }
    double average = sum/TOTAL_DELTAS;

    // Convert the minimum, maximum and average to milliseconds
    average*= 1000;
    min *= 1000;
    max *= 1000;

    // Do warning messages if things are longer then expected
    const double kWarningAverage = 23;
    const double kWarningMax = 32;
    if (average > kWarningAverage) {
        std::cout << std::setprecision(3) << "WARNING: High Average Period " << average << "ms\n";
    }
    if (max > kWarningMax) {
        std::cout << std::setprecision(3) << "WARNING: High Max Period " << max << "ms\n";
    }

    // If not only doing warnings then log everything
    if (!warnings_only) {
    	std::cout << std::setprecision(3) << "Average " << average << " Min " << min << " Max " << max << "\n";
    }
}

