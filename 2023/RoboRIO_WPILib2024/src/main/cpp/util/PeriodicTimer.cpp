//==============================================================================
// PeriodicTimer.cpp
//==============================================================================

#include "PeriodicTimer.h"
#include <iostream>
#include <iomanip>
#include <frc/shuffleboard/Shuffleboard.h>

using namespace frc;


int PeriodicTimer::ms_widget_count = 0;


//==============================================================================
// Construction

PeriodicTimer::PeriodicTimer(const char* name, Mode mode, int display_frequency, bool enabled)
 {
    m_name = name;
    m_mode = mode;
    m_display_frequency = display_frequency;
    m_enabled = enabled;

    m_total_processing = 0.0_s;
    m_min_processing = 0.0_s;
    m_max_processing = 0.0_s;
    m_total_periodic = 0.0_s;
    m_min_periodic = 0.0_s;
    m_max_periodic = 0.0_s;
    m_total_periods = 0;

    m_warning_processing_max = 20_ms;
    m_warning_processing_ave = 15_ms;
    m_warning_periodic_max = 30_ms;
    m_warning_periodic_ave = 25_ms;

    m_periodic_widgets = nullptr;
    m_processing_widgets = nullptr;

    m_period_timer.Start();
    m_processing_timer.Start();

    if (m_enabled) {
        SetupShuffleboard();
    }

    Init();
}

PeriodicTimer::~PeriodicTimer() {
}

//==========================================================================
// Operation

void PeriodicTimer::Init() {
    m_first_period = true;
}

void PeriodicTimer::PeriodicStart() {
    m_processing_timer.Reset();
}

void PeriodicTimer::PeriodicEnd() {
    if (!m_enabled) return;

    // The first time this function is called just reset the timer and set the index to the start of the delta array
    if (m_first_period) {
        m_first_period = false;
        m_period_timer.Reset();
        return;
    }

    // Get the time delta since the last time this function was called and resets the timer
    // Get the time spent processing for this period
    units::second_t period_time = m_period_timer.Get();
    m_period_timer.Reset();
    units::second_t processing_time = m_processing_timer.Get();

    // Get the period and processing time into the statistics
    m_total_periodic += period_time;
    if (m_min_periodic > period_time) m_min_periodic = period_time;
    if (m_max_periodic < period_time) m_max_periodic = period_time;
    m_total_processing += processing_time;
    if (m_min_processing > processing_time) m_min_processing = processing_time;
    if (m_max_processing < processing_time) m_max_processing = processing_time;
    m_total_periods++;

    UpdateShuffleboard(processing_time, period_time);

    // Determine if the warnings/Statistics should be displayed based on the update mode
    bool do_display = false;
    switch (m_mode & UpdateMask) {
        case Manual:
            break;
        case AutoPeriods:
            do_display = m_total_periods >= m_display_frequency;
            break;
        case AutoMilliseconds:
            do_display = m_total_periodic >= (units::millisecond_t)m_display_frequency;
            break;
    }

    // Display the warnings/Statistics if required and reset the statistics
    if (do_display) {
//      LogAllDeltas();
//        LogSummary(m_mode);
        ResetStatistics();
    }
}

void PeriodicTimer::SetMode(Mode mode, int display_frequency) {
    m_mode = mode;
    m_display_frequency = display_frequency;
}

void PeriodicTimer::SetProcessingLimits(units::second_t warning_processing_max,
                                        units::second_t warning_processing_ave) {
    m_warning_processing_max = warning_processing_max;
    m_warning_processing_ave = warning_processing_ave;
}

void PeriodicTimer::SetPeriodicLimits(units::second_t warning_periodic_max,
                                      units::second_t warning_periodic_ave) {
    m_warning_periodic_max = warning_periodic_max;
    m_warning_periodic_ave = warning_periodic_ave;
}

void PeriodicTimer::DoManualDisplay(Mode mode) {
    LogSummary(mode);
    ResetStatistics();
}


//==========================================================================
// Logging

void PeriodicTimer::ResetStatistics() {
    m_total_processing = 0.0_s;
    m_min_processing = 100000.0_s;
    m_max_processing = 0.0_s;
    m_total_periodic = 0.0_s;
    m_min_periodic = 100000.0_s;
    m_max_periodic = 0.0_s;
    m_total_periods = 0;
}

// void PeriodicTimer::LogAllDeltas() {
//     // Log all delta values on a single line
//     printf("%s deltas: ", m_name);
//     for (int i = 0; i< TOTAL_DELTAS; i++) {
//         printf("%d, ", (int)(m_period_deltas[i]*1000.0));
//     }
//     printf("\n");
// }

void PeriodicTimer::LogSummary(Mode mode) {
    // Avoid divide by zero errors (should never happen if the class is used properly)
    if (m_total_periods == 0) {
        std::cout << "Zero periods in  PeriodicTimer::LogSummary\n";
        return;
    }

    // Convert the minimum, maximum and average to milliseconds
    units::millisecond_t average_period = m_total_periodic / m_total_periods;
    units::millisecond_t average_processing = m_total_processing / m_total_periods;
    
    // Do warning messages if things are longer then expected
    if ((mode & ShowPeriodicWarnings) != 0 && average_period > m_warning_periodic_ave) {
        std::cout << std::setprecision(3) << m_name << " WARNING: High Average Periodic " << average_period.value() << "ms\n";
    }
    if ((mode & ShowPeriodicWarnings) != 0 && m_max_periodic > m_warning_periodic_max) {
        std::cout << std::setprecision(3) << m_name << " WARNING: High Max Periodic " << m_max_periodic.value() << "ms\n";
    }
    if ((mode & ShowProcessingWarnings) != 0 && average_processing > m_warning_processing_ave) {
        std::cout << std::setprecision(3) << m_name << " WARNING: High Average Processing " << average_processing.value() << "ms\n";
    }
    if ((mode & ShowProcessingWarnings) != 0 && m_max_processing > m_warning_processing_max) {
        std::cout << std::setprecision(3) << m_name << " WARNING: High Max Processing " << m_max_processing.value() << "ms\n";
    }

    // Log stats if any
    if ((mode & ShowStats) != 0) {
        // Output values as a CSV line for easier importing intoa spreadsheet.
        // Start with the name of the timer
    	std::cout << std::setprecision(3) << m_name;

        // Log the periodic stats if required
        if ((mode & ShowPeriodicStats) != 0) {
            std::cout << " Period (Average/Min/Max)," << average_period.value() << ","
                     << m_min_periodic.value() << "," << m_max_periodic.value();

            if ((mode & ShowPeriodicStats) != 0) {
                std::cout << ",";
            }
        }

        // Log the processing stats if required
        if ((mode & ShowProcessingStats) != 0) {
            std::cout << ", Processing  (Average/Min/Max)," << average_processing.value() << ","
                    << m_min_processing.value() << "," << m_max_processing.value() << "\n";
        }
        std::cout << "\n";
    }
}

//==============================================================================
// Shuffleboard

void PeriodicTimer::SetupShuffleboard() {
    frc::ShuffleboardTab& shuffleboard_tab = frc::Shuffleboard::GetTab("Timers");

    if ((m_mode & ShowPeriodic) != 0 && m_periodic_widgets == nullptr) {
        m_periodic_widgets = SetupTimerWidgets(shuffleboard_tab, " Periodic");
    }
    if ((m_mode & ShowProcessing) != 0 && m_processing_widgets == nullptr) {
        m_processing_widgets = SetupTimerWidgets(shuffleboard_tab, " Processing");
    }
}


PeriodicTimer::TimerWidgets* PeriodicTimer::SetupTimerWidgets(frc::ShuffleboardTab& shuffleboard_tab, std::string type) {
    const int kPanelWidth = 6;
    const int kPanelHeight = 5;
    const int kPanelsPerColumn = 1;
    const int kGraphHeight = 6;

    int column = ms_widget_count / kPanelsPerColumn;
    int row = ms_widget_count % kPanelsPerColumn;
    ms_widget_count++;
    int x_pos = column * kPanelWidth;
    int y_pos = row * kPanelHeight;

    // Create a new 9x6 grid-type layout block at the given position
    frc::ShuffleboardLayout& layout =
        shuffleboard_tab.GetLayout(m_name + type, frc::BuiltInLayouts::kGrid)
            .WithPosition(x_pos, y_pos)
            .WithSize(kPanelWidth, kPanelHeight);

    TimerWidgets* widgets = new TimerWidgets;

    widgets->m_current_graph_widget = &shuffleboard_tab.Add(m_name + type + "Graph (ms)", 0.0)
                                .WithPosition(x_pos, y_pos + kPanelHeight).WithSize(kPanelWidth, kGraphHeight)
                                .WithWidget(frc::BuiltInWidgets::kGraph);
    widgets->m_current_widget = &layout.Add("Current (ms)", 0.0).WithPosition(0, 0).WithSize(3, 2);
    widgets->m_average_widget = &layout.Add("Average (ms)", 0.0).WithPosition(1, 0).WithSize(3, 2);
    widgets->m_max_widget = &layout.Add("Max (ms)", 0.0).WithPosition(0, 1).WithSize(3, 2);
    widgets->m_min_widget = &layout.Add("Min (ms)", 0.0).WithPosition(1, 1).WithSize(3, 2);


    return widgets;
}

void PeriodicTimer::UpdateShuffleboard(units::millisecond_t processing, units::millisecond_t periodic) {
    if (m_periodic_widgets) {
        UpdateTimerWidgets(m_periodic_widgets, periodic, m_total_periodic / m_total_periods,
                           m_min_periodic, m_max_periodic);
    }
    if (m_processing_widgets) {
        UpdateTimerWidgets(m_processing_widgets, processing, m_total_processing / m_total_periods,
                           m_min_processing, m_max_processing);
    }
}

void PeriodicTimer::UpdateTimerWidgets(TimerWidgets* timer_widgets, units::millisecond_t current, units::millisecond_t average,
                        units::millisecond_t min, units::millisecond_t max)
{
	timer_widgets->m_current_graph_widget->GetEntry()->SetDouble(current.value());
	timer_widgets->m_current_widget->GetEntry()->SetDouble(current.value());
	timer_widgets->m_average_widget->GetEntry()->SetDouble(average.value());
	timer_widgets->m_max_widget->GetEntry()->SetDouble(max.value());
	timer_widgets->m_min_widget->GetEntry()->SetDouble(min.value());
}
