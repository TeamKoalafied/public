//==============================================================================
// LidarSystem.cpp
//==============================================================================


#include "LidarSystem.h"
#include <CameraServer.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/core/types.hpp>
#include <unistd.h>
#include <iostream>


//==============================================================================
// Construction

LidarSystem::LidarSystem() :
    TSingleton<LidarSystem>(this),
    frc::Subsystem("LidarSystem") {
	m_lidar = NULL;
	m_ranges = kdefault_range;
	m_thread = NULL;
}

LidarSystem::~LidarSystem() {
    Shutdown();
}


//==============================================================================
// frc::Subsystem Function Overrides


void LidarSystem::InitDefaultCommand() {
    // No default command
}


//==============================================================================
// Setup and Shutdown

void LidarSystem::Setup() {
    printf("LidarSystem::Setup()\n");

	// Setup a stream called "Lidar" for sending images to the Dashboard.
	m_output_stream = CameraServer::GetInstance()->PutVideo("Lidar", 160, 120);

	// Connect to the LIDAR on /dev/ttyUSB0
	m_lidar = new sweep::Sweep("/dev/ttyUSB0", 115200);

	// New 2018 library handles device and communication delays properly
	m_lidar->SetMotorSpeed(5);
	m_lidar->StartScanning();

	// Start the lidar thread
    m_thread = new std::thread(&LidarSystem::LidarThread, this);
	m_thread->detach();
}

void LidarSystem::Shutdown() {
    printf("LidarSystem::Shutdown()\n");
}


//==========================================================================
// Get closest points in each of 8 quadrants SE, E, NE, N, NW, W, SW, S

void LidarSystem::GetRanges(LidarRanges *ranges) {
    // Lock the mutex for the scope of this function so we can access variable
    // shared between threads
    std::lock_guard<wpi::mutex> lock(m_mutex);
    *ranges = m_ranges;
    return;
}


//==========================================================================
// Get closest points in each of 8 quadrants SE, E, NE, N, NW, W, SW, S
void LidarSystem::SetRanges(LidarRanges *ranges) {
    // Lock the mutex for the scope of this function so we can access variable
    // shared between threads
    std::lock_guard<wpi::mutex> lock(m_mutex);
    m_ranges = *ranges;
    return;
}


//==========================================================================
// Lidar thread. Grabs Lidar data and displays lidar map on the dashboard.

void LidarSystem::LidarThread() {
	LidarRanges local_ranges;

	// Mats are very memory expensive. Lets reuse this Mat.
	cv::Mat mat(120, 160, CV_8UC1);

	std::cout << "Entering scanning loop\n";
	while (true) {
		// read the scan results - it blocks until a scan is available
	    std::vector<sweep::SweepSample> scan = m_lidar->GetScan();

		// Blank out the image
		mat = cv::Scalar(0);

		// Clear the local copy of ranges to max range
		local_ranges = kdefault_range;

		// Step through all the samples of one lidar scan
		for (auto sample : scan) {
			double distance;
			double radians;
			int x;
			int y;
			double inches = sample.distance * .25;  // range in inches
			double degrees = sample.angle * 0.001;   // angle in degrees

			// Lidar returns 1 if it could not get a reflection. Set this to max
			if (sample.distance == 1)
				inches = 10000.0;

			// Compute minimum distance for each quadrant.
			// quadrants are:
			//   sw 225.0 +/- 22.5
			//   w  270.0 +/- 22.5
			//   nw 315.0 +/- 22.5
			//   n  360.0-22.5 to 22.5
			//   ne 45.0 +/- 22.5
			//   e  90.0 +/- 22.5
			//   se 135.0 +/- 22.5
			//   s  180.0+/- 22.5
			if ((degrees >= (225.0-22.5)) && (degrees < (225.0+22.5))) {
				if (inches < local_ranges.s.range) {
					local_ranges.sw.range = inches;
					local_ranges.sw.angle = degrees;
				}
			}
			if ((degrees >= (270.0-22.5)) && (degrees < (270.0+22.5))) {
				if (inches < local_ranges.s.range) {
					local_ranges.w.range = inches;
					local_ranges.w.angle = degrees;
				}
			}
			if ((degrees >= (315.0-22.5)) && (degrees < (315.0+22.5))) {
				if (inches < local_ranges.s.range) {
					local_ranges.nw.range = inches;
					local_ranges.nw.angle = degrees;
				}
			}
			if (((degrees >= (360.0-22.5)) && (degrees <= 360.0)) ||
				((degrees >= 0.0) && (degrees < (0.0+22.5)))) {
				if (inches < local_ranges.s.range) {
					local_ranges.n.range = inches;
					local_ranges.n.angle = degrees;
				}
			}
			if ((degrees >= (45.0-22.5)) && (degrees < (45.0+22.5))) {
				if (inches < local_ranges.s.range) {
					local_ranges.ne.range = inches;
					local_ranges.ne.angle = degrees;
				}
			}
			if ((degrees >= (90.0-22.5)) && (degrees < (90.0+22.5))) {
				if (inches < local_ranges.s.range) {
					local_ranges.e.range = inches;
					local_ranges.e.angle = degrees;
				}
			}
			if ((degrees >= (135.0-22.5)) && (degrees < (135.0+22.5))) {
				if (inches < local_ranges.s.range) {
					local_ranges.se.range = inches;
					local_ranges.se.angle = degrees;
				}
			}
			if ((degrees >= (180.0-22.5)) && (degrees < (180.0+22.5))) {
				if (inches < local_ranges.s.range) {
					local_ranges.s.range = inches;
					local_ranges.s.angle = degrees;
				}
			}

			// Write the polar lidar samples to the cartesian image
			distance = 0.4 * inches; // scaled to fit a room into the image
			radians = (-0.000017453292519943295 * sample.angle);
			x = 80 + (distance * std::cos(radians));
			y = 60 + (distance * std::sin(radians));
			if (x < 0)
				x = 0;
			if (x > 159)
				x = 159;
			if (y < 0)
				y = 0;
			if (y > 119)
				y = 119;
			mat.at<uchar>(y, x) = 255;
		}

		// Write the computed range data
		LidarSystem::SetRanges(&local_ranges);

		// Write the image to the "lidar" stream
		m_output_stream.PutFrame(mat);
	}
}
