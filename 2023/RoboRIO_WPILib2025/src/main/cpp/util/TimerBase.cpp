// TimerBase.cpp
//==============================================================================
// TimerBase.cpp
//==============================================================================

#include "TimerBase.h"
#include <iostream>
#include <iomanip>
#include <frc/shuffleboard/Shuffleboard.h>

using namespace frc;


int TimerBase::ms_widget_count = 0;


//==============================================================================
// Construction

TimerBase::TimerBase(const char* name, bool enabled, Mode mode, int display_frequency,
                     units::second_t warning_max, units::second_t warning_ave)
 {
    // Record the timer parameters
    m_name = name;
    m_mode = mode;
    m_display_frequency = display_frequency;
    m_enabled = enabled;

    // Initialise the timer statistics
    m_total_time = 0.0_s;
    m_min_time = 0.0_s;
    m_max_time = 0.0_s;
    m_time_count = 0;

    // Setup the shuffleboard if the timer is enabled
    m_widgets = nullptr;
    if (m_enabled) {
        SetupShuffleboard();
    }
}

TimerBase::~TimerBase() {
}


//==========================================================================
// Operation


void TimerBase::Update(units::millisecond_t current_time)
{
    // Get the period and processing time into the statistics
    m_total_time += current_time;
    if (m_min_time > current_time) m_min_time = current_time;
    if (m_max_time < current_time) m_max_time = current_time;
    m_time_count++;

    // Update the shuffleboartd display
    UpdateShuffleboard(current_time);

    // Determine if the display frequency has expired, depending on the mode
    bool do_display = false;
    switch (m_mode & UpdateMask) {
        case AutoPeriods:
            do_display = m_time_count >= m_display_frequency;
            break;
        case AutoMilliseconds:
            do_display = m_total_time >= (units::millisecond_t)m_display_frequency;
            break;
    }

    // Log the warnings/statistics to the console if required and reset the statistics
    if (do_display) {
        LogSummary(m_mode);

        m_total_time = 0.0_s;
        m_min_time = 100000.0_s;
        m_max_time = 0.0_s;
        m_time_count = 0;
    }
}


//==========================================================================
// Logging

void TimerBase::LogSummary(Mode mode) {
    // Avoid divide by zero errors (should never happen if the class is used properly)
    if (m_time_count == 0) {
        std::cout << "Zero periods in  TimerBase::LogSummary\n";
        return;
    }

    // Convert the minimum, maximum and average to milliseconds
    units::millisecond_t average = m_total_time / m_time_count;
    
    // Do warning messages if things are longer then expected, and warning are enabled in the mode
    if ((mode & LogWarnings) != 0 && average > m_warning_ave) {
        std::cout << std::setprecision(3) << m_name << " WARNING: High Average Periodic " << average.value() << "ms\n";
    }
    if ((mode & LogWarnings) != 0 && m_max_time > m_warning_max) {
        std::cout << std::setprecision(3) << m_name << " WARNING: High Max Periodic " << m_max_time.value() << "ms\n";
    }

    // Log stats if set in the mode
    if ((mode & LogStats) != 0) {
        // Output values as a CSV line for easier importing into a spreadsheet.
        // Start with the name of the timer
    	std::cout << std::setprecision(3) << m_name;
        std::cout << " (Average/Min/Max)," << average.value() << ","
                  << m_min_time.value() << "," << m_max_time.value() << "\n";
    }
}


//==============================================================================
// Shuffleboard

void TimerBase::SetupShuffleboard() {
    // If the shuffleboard is not being used bail out
    if ((m_mode & Shuffleboard) == 0) return;

    frc::ShuffleboardTab& shuffleboard_tab = frc::Shuffleboard::GetTab("Timers");

    // Constants for the size of the shuffleboard items
    const int kPanelWidth = 7;
    const int kPanelHeight = 4;
    const int kPanelsPerColumn = 1;
    const int kGraphHeight = 6;

    // Calculate a position for this timer on the shuffleboard. They are just laid out left to right.
    int x_pos = ms_widget_count * kPanelWidth;
    int y_pos = 0;
    ms_widget_count++;

    // Create a new 2x2 grid layout block at the calculated position
    frc::ShuffleboardLayout& layout =
        shuffleboard_tab.GetLayout(m_name, frc::BuiltInLayouts::kGrid)
            .WithPosition(x_pos, y_pos)
            .WithSize(kPanelWidth, kPanelHeight)
            .WithProperties({
                {"Number of columns", nt::Value::MakeInteger(2)},
                {"Number of rows", nt::Value::MakeInteger(2)}
            });

    // Create the widget struct and setup and record all the widgets. The graph is placed under the
    // grid.
    m_widgets = new TimerWidgets;
    // m_widgets->m_current_graph_widget = &shuffleboard_tab.Add(m_name + "Graph (ms)", 0.0)
    //                             .WithPosition(x_pos, y_pos + kPanelHeight).WithSize(kPanelWidth, kGraphHeight)
    //                             .WithWidget(frc::BuiltInWidgets::kGraph)
    //                             .WithProperties({
    //                                 {"Unit", nt::Value::MakeString("ms")}
    //                             });
    m_widgets->m_current_widget = &layout.Add("Current (ms)", 0.0).WithPosition(0, 0).WithSize(3, 2).WithWidget(frc::BuiltInWidgets::kTextView);
    m_widgets->m_average_widget = &layout.Add("Average (ms)", 0.0).WithPosition(1, 0).WithSize(3, 2).WithWidget(frc::BuiltInWidgets::kTextView);
    m_widgets->m_max_widget = &layout.Add("Max (ms)", 0.0).WithPosition(0, 1).WithSize(3, 2).WithWidget(frc::BuiltInWidgets::kTextView);
    m_widgets->m_min_widget = &layout.Add("Min (ms)", 0.0).WithPosition(1, 1).WithSize(3, 2).WithWidget(frc::BuiltInWidgets::kTextView);
}

void TimerBase::UpdateShuffleboard(units::millisecond_t current_time) {
    // If the shuffleboard is not being used bail out
    if (m_widgets == nullptr) return;

    // Ccalculate the average time and update all the widget values
    units::millisecond_t average = m_total_time / m_time_count;
	// m_widgets->m_current_graph_widget->GetEntry()->SetDouble(current_time.value());
	m_widgets->m_current_widget->GetEntry()->SetDouble(current_time.value());
	m_widgets->m_average_widget->GetEntry()->SetDouble(average.value());
	m_widgets->m_max_widget->GetEntry()->SetDouble(m_max_time.value());
	m_widgets->m_min_widget->GetEntry()->SetDouble(m_min_time.value());
}
