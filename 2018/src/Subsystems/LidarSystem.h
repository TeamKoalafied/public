//==============================================================================
// LidarSystem.h
//==============================================================================

#ifndef LidarSystem_H
#define LidarSystem_H

#include "wpilib.h"
#include "TSingleton.h"
//#include <Commands/Subsystem.h>
//#include <support/mutex.h>
#include <thread>
#include <Sweep.hpp>

namespace frc {
class Compressor;
}

enum LidarMode {
	kLidarIdle,
	kLidarScan,
	kLidarSetSpeed
};

struct LidarPolar {
	double range;
	double angle;
};

struct LidarRanges {
	LidarPolar sw;
	LidarPolar w;
	LidarPolar nw;
	LidarPolar n;
	LidarPolar ne;
	LidarPolar e;
	LidarPolar se;
	LidarPolar s;
};

// The vision subsystem processes the image from the camera
//class LidarSystem : public TSingleton<LidarSystem>, public frc::Subsystem {
class LidarSystem : public TSingleton<LidarSystem>, public frc::Subsystem {
public:
    //==========================================================================
    // Construction

    // Constructor
    LidarSystem();

    // Destructor
    virtual ~LidarSystem();


    //==========================================================================
    // frc::Subsystem Function Overrides
    virtual void InitDefaultCommand() override;
    //==========================================================================


    //==========================================================================
    // Setup and Shutdown

    // Setup the vision for operation
    void Setup();

    // Shutdown the vision
    void Shutdown();

    //==========================================================================
    // Utility functions

    //==========================================================================
    // Get closest points in each of 8 quadrants SE, E, NE, N, NW, W, SW, S
    // These quadrants are 45 degrees wide and are centred on the compass points
    // Range is in inches, angle is a bearing (0 is forward, 270 is due West)

    void GetRanges(LidarRanges *ranges);

private:
    // Constants
	const LidarRanges kdefault_range = {
			{10000.0,  225.0},    // sw
			{10000.0,  270.0},    // w
			{10000.0,  315.0},    // nw
			{10000.0,  0.0},      // n
			{10000.0,  45.0},     // ne
			{10000.0,  90.0},     // e
			{10000.0,  135.0},    // se
			{10000.0,  180.0}     // s
	};


    //==========================================================================
    // Vision Thread

    // Main function of the vision thread
    void LidarThread();

    //==========================================================================
    // Member Variables

    mutable wpi::mutex m_mutex;     // Mutex to control access to variables shared between threads
    std::thread* m_thread;          // Thread that the vision runs on
    sweep::Sweep *m_lidar;          // The lidar object
	cs::CvSource m_output_stream;   // The video stream to write the results to
	LidarRanges m_ranges;           // Minimum distance in SW, W, NW, N, NE, E, SE, S directions

    //==========================================================================
    // Set m_ranges
    void SetRanges(LidarRanges *ranges);

};

#endif  // LidarSystem_H
