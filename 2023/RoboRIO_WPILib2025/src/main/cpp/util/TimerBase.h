//==============================================================================
// TimerBase.h
//==============================================================================

#pragma once

#include <string>
#include <units/time.h>

namespace frc { class SimpleWidget; }


// A timer for monitoring how long some repeated thing takes. Derived classes offer different
// ways of timing.
// Timing results can be displayed on the shuffleboard and/or logged to the console. There is a
// 'display frequency', measured in either periods or milliseconds depending on the mode. When the
// display frequency elapses the statistics (average/min/max) are rest and console logging occurs,
// if enabled. Shuffleboard display is refreshed for every update.
class TimerBase {
public:
    //
    enum Mode {
        AutoPeriods             =  1,  // Updates occur automatically after a set number of periods
        AutoMilliseconds        =  2,  // Updates occur automatically after a set millisecond duration
        UpdateMask              =  3,  // Mask for extracting when updates occur
        Shuffleboard            =  4,  // Show the timer on the shuffleboard
        LogWarnings             =  8,  // Log warnings about timers overruns to the console
        LogStats                = 16,  // Log statistics for timing to the console
        DefaultMode             = AutoPeriods | Shuffleboard
    };

    //==========================================================================
    // Construction

    // Constructor
    //
    // name - Name of the timer. Shown in warnings and statistics.
    // enable - Whether the timer is enabled. If disable nothing is display on the shuffleboard of console
    // mode - Mode flags that control display
    // display_frequency - Frequency of automatic updates (either periods or milliseconds
    //      depending on the mode).
    // warning_max - Maximum time warning level for console logging
    // warning_ave - Average time warning level for console logging
    TimerBase(const char* name, bool enabled = true, Mode mode = DefaultMode, int display_frequency = 5000,
              units::second_t warning_max = 25_ms, units::second_t warning_ave = 20_ms);

    // Destructor
    ~TimerBase();


protected:
    //==========================================================================
    // Timer Updating


    // Update statistics and display for a given new time measurement
    //
    // current_time - Current time measurement
    void Update(units::millisecond_t current_time);


private:
    //==========================================================================
    // Private Nested Types

    // Struct containing widgets for a single timer position
    struct TimerWidgets {   
        frc::SimpleWidget* m_current_graph_widget;
        frc::SimpleWidget* m_current_widget;
        frc::SimpleWidget* m_average_widget;
        frc::SimpleWidget* m_max_widget;
        frc::SimpleWidget* m_min_widget;
    };


    //==========================================================================
    // Logging

    // Log summary information to the console (Min, Max, Average)
    //
    // mode - Mode flags that control display
    void LogSummary(Mode mode);


    //==========================================================================
    // Shuffleboard

    // Setup any shuffle board widgets required
    void SetupShuffleboard();

    // Update the display on the shuffleboard
    //
    // current_time - Current measured time
    void UpdateShuffleboard(units::millisecond_t current_time);


    //==========================================================================
    // Member Variables

    std::string m_name;                     // Name of the timer
    bool m_enabled;                         // Whether the timer is enabled
    Mode m_mode;                            // Mode flags that control display
    int m_display_frequency;                // Frequency or automatic updates (either periods
                                            //  or milliseconds depending on the mode).
    units::millisecond_t m_warning_max;     // Maximum time warning level
    units::millisecond_t m_warning_ave;     // Average time warning level

    units::millisecond_t m_total_time;      // Total time recorded (seconds)
    units::millisecond_t m_min_time;        // Minimum time recorded (seconds)
    units::millisecond_t m_max_time;        // Maximum time recorded (seconds)
    int m_time_count;                       // Number of times recorded

    TimerWidgets* m_widgets;                // Widgets for timing. nullptr if not used.

    static int ms_widget_count;             // Number of sets of timer widgets create. Used to lay them out.
};

