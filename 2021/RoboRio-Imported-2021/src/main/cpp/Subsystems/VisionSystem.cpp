//==============================================================================
// Leds.cpp
//==============================================================================

#include "VisionSystem.h"
#include "frc/Relay.h"

#include "../RobotConfiguration.h"
#include "Vision/GripOutput.h"

#include <cameraserver/CameraServer.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/core/types.hpp>



//
// Timing code from http://www.firstobject.com/getmillicount-milliseconds-portable-c++.htm
// Usage:
//   int start = GetMilliCount();
//   ... do processing ...
//   int elapsed = GetMilliSpan(start);
//
#include <sys/timeb.h>

int GetMilliCount()
{
  // Something like GetTickCount but portable
  // It rolls over every ~ 12.1 days (0x100000/24/60/60)
  // Use GetMilliSpan to correct for rollover
  timeb tb;
  ftime(&tb);
  int nCount = tb.millitm + (tb.time & 0xfffff) * 1000;
  return nCount;
}

int GetMilliSpan( int nTimeStart )
{
  int nSpan = GetMilliCount() - nTimeStart;
  if ( nSpan < 0 )
    nSpan += 0x100000 * 1000;
  return nSpan;
}

//==============================================================================
// Construction

VisionSystem::VisionSystem() :
    TSingleton<VisionSystem>(this),
    frc::Subsystem("VisionSystem") {

    m_found_object = false;
    m_distance_to_object = 0.0;
    m_direction_of_object = 0.0;
    m_dark_image = false;
    m_thread = 0;
}

VisionSystem::~VisionSystem() {
    Shutdown();
}


//==============================================================================
// frc::Subsystem Function Overrides


void VisionSystem::InitDefaultCommand() {
    // No default command
}


//==============================================================================
// Setup and Shutdown

void VisionSystem::Setup() {
    printf("VisionSystem::Setup()\n");

//    m_camera = frc::CameraServer::GetInstance()->StartAutomaticCapture();
    // Set the resolution
//    m_camera.SetResolution(160, 120);
    // Stop the camera flicker
//    m_camera.SetFPS(25); //max - was 10
    // Start as a normal camera by default
//	m_camera.SetExposureAuto();
//	m_camera.SetWhiteBalanceAuto();

    // Start the vision thread
    m_thread = new std::thread(&VisionSystem::VisionThread, this);

}

void VisionSystem::Shutdown() {
    printf("VisionSystem::Shutdown()\n");
}

//==========================================================================
// Vision Tracking Status

bool VisionSystem::GetCubeFound(double& distance, double& direction) {
    // Lock the mutex for the scope of this function so we can access variable
    // shared between threads
    std::lock_guard<wpi::mutex> lock(m_mutex);
    distance = m_distance_to_object;
    direction = m_direction_of_object;
    return m_found_object;
}


//==========================================================================
// Vision Thread


void VisionSystem::VisionThread() {

	int m_exposure = 20;  // (0-100%) 0 or 1 for IR camera, 50 or more for Microsoft webcam
	int m_threshold = 250;
	int m_line_length = 20;
	int m_fps = 20;
	int m_brightness = 50; // (0-100%)
	int m_change = 1;
	int m_debug = 0;

	const int kWidth = 160; // 160, 320, 640
	const int kHeight = 120; // 120, 240, 480

	const int whiteWidthMax = 5;
	const int blackWidthMin = 10;
	const int blackWidthMax = 60;

	// Get the USB camera from CameraServer
	// See https://robotdotnet.github.io/api/CSCore.UsbCamera.html for details
	cs::UsbCamera camera = frc::CameraServer::GetInstance()->StartAutomaticCapture();

	// Set the capture resolution
	camera.SetResolution(kWidth, kHeight);

	// Fix the colour cast
	//camera.SetWhiteBalanceManual(4800);    // 4800 is ok for yellow-box finder

	// Get a CvSink. This will capture Mats from the Camera
	cs::CvSink cvSink = frc::CameraServer::GetInstance()->GetVideo();

	// Setup a CvSource. This will send unprocessed images back to the Dashboard
	cs::CvSource outputStream = frc::CameraServer::GetInstance()->PutVideo("Live", kWidth, kHeight);

	// Setup a CvSource. This will send processed images back to the Dashboard
	cs::CvSource outputStream2 = frc::CameraServer::GetInstance()->PutVideo("Processed", kWidth, kHeight);

	// Create a Mat for image capture. The capture routines allocate space for it.
	cv::Mat mat;

	// Create a monochrome image. Note that CV_8UC1 is an enumeration, not a struct
	cv::Mat mono(kHeight, kWidth, CV_8UC1);

	while (true) {
		int tStart;
		int tDuration1;
		int tDuration2;
		int tDuration3;

		tStart = GetMilliCount();
		if (m_change) {
			// To maintain FPS, only change settings if needed. Changing exposure takes 250ms

			if (m_exposure == -1)
				// Set camera to expose automatically
				camera.SetExposureAuto();
			else
				if (m_exposure == -2)
					// Hold the current exposure (use after auto exposure has settled)
					camera.SetExposureHoldCurrent();
				else
					// Fix the camera exposure (0-100%) (see wpilib\cpp\current\include\cscore_oo.h)
					camera.SetExposureManual(m_exposure);

			// Set the framerate
			camera.SetFPS(m_fps);

			// Fix the camera exposure (0-100%) (see wpilib\cpp\current\include\cscore_oo.h
			camera.SetBrightness(m_brightness); //mjn

			m_change = 0;
		}

		tDuration1 = GetMilliSpan(tStart);

		// Grab a frame and save to mat
		if (cvSink.GrabFrame(mat) == 0) {
			// Send the output the error
			outputStream.NotifyError(cvSink.GetError());
			// skip the rest of the current iteration
			continue;
		}

		tDuration2 = GetMilliSpan(tStart) - tDuration1;

		// create a type for accessing blue green red pixels
		struct Pixbgr {
			unsigned char b;
			unsigned char g;
			unsigned char r;
		};

		// Threshold image to find very bright pixels (eg reflective tape)
		for(int i = 0; i < kHeight; i++) {
			// Get pointers to the rows of the input and output images
			struct Pixbgr *mi = mat.ptr<struct Pixbgr>(i);
			unsigned char *mo = mono.ptr<unsigned char>(i);
			for(int j = 0; j < kWidth; j++) {
				if ((mi[j].b * mi[j].b + mi[j].g * mi[j].g + mi[j].r * mi[j].r) > (m_threshold * m_threshold))
					mo[j] = 255;  // use 255 so it is visible in the video stream
				else
					mo[j] = 0;
			}
		}

		// This is a very crude search for two vertical lines. It searches for two white lines
		// that are wide enough and have a suitably sized black space between them. If these
		// criteria are met on enough consecutive rows then the vertical lines are deemed long
		// enough, and the target found.
		int matchStart = -1;
		int matchEnd = -1;
		int left;
		int leftEnd;
		int right;
		int rightEnd;
		struct {
			int t;
			int b;
			int r;
			int l;
		} box = {0,0,0,0};
		enum {
			kStarting,
			kLookingForLeftLine,
			kLookingForGap,
			kLookingForRightLine,
			kLookingForEnd,
			kAllDone
		} state;

		for(int i = 0; i < kHeight; i++) {
			// Get a pointer to the current row of the output image
			unsigned char *mo = mono.ptr<unsigned char>(i);
			int j = 0;

			left = 0;
			leftEnd = 0;
			right = 0;
			rightEnd = 0;
			state = kLookingForLeftLine;

			for (j = 0; j < kWidth; j++) {
				switch (state) {
				case kStarting:
					// Need to start with black to prevent detecting "off-page" blocks of white
					if (!mo[j]) {
						state = kLookingForLeftLine;
					}
					break;
				case kLookingForLeftLine:
					if (mo[j]) {
						// Found the possible start of a line
						left = j;
						state = kLookingForGap;
					}
					break;
				case kLookingForGap:
					if (!mo[j]) {
						if ((j - left) <= whiteWidthMax) {
							// Line is within tolerance - look for next line
							leftEnd = j;
							state = kLookingForRightLine;
						} else {
							// Line is too wide - start again
							left = 0;
							state = kLookingForLeftLine;
						}
					}
					break;
				case kLookingForRightLine:
					if (mo[j]) {
						if (((j - leftEnd) > blackWidthMin) &&
							((j - leftEnd) < blackWidthMax)) {
							// Gap is within tolerance - look for end of line
							right = j;
							state = kLookingForEnd;
						} else {
							// Gap is out of tolerance - start again from here
							left = j;
							leftEnd = 0;
							state = kLookingForGap;
						}
					}
					break;
				case kLookingForEnd:
					if (!mo[j]) {
						if ((j - right) <= whiteWidthMax) {
							// Line is within tolerance - we're done!
							rightEnd = j;
							state = kAllDone;
						} else {
							// Line is too wide - start again from the start
							left = 0;
							leftEnd = 0;
							right = 0;
							state = kLookingForLeftLine;
						}
					}
					break;
				case kAllDone:
				default:
					break;
				}
				if (state == kAllDone)
					break;
			}

			if (left && leftEnd && right && rightEnd) {
				// We found a matching row
				if (matchStart < 0) {
					matchStart = i;
					box.t = matchStart;
					box.l = left;
				}
				matchEnd = i;
				box.b = i;
				box.r = right;
			} else {
				// We didn't find a matching row
				if ((matchEnd - matchStart) < m_line_length) {
					// lines too short so start again
					matchStart = -1;
					matchEnd = 0;
				} else {
					// We've found what we were looking for, so break out of the search
					break;
				}
			}
		}

		if (m_debug) {
			std::cout << "  \tl=" << left << ", leftEnd=" << leftEnd << "  \t";
			std::cout << "r=" << right << ", rightEnd=" << rightEnd;
		}


		bool found = false;
        double distance = 0.0;
        double direction = 0.0;
		if ((matchEnd - matchStart) >= m_line_length) {

			found = true;

            // Draw a white rectangle around the blob
			rectangle(mono, cv::Point(box.l, box.t), cv::Point(box.r, box.b), cv::Scalar(255), 1);

            // find the average position of the rectangle and convert it to a +/-1.0 offset
            // This needs to be negative to match the motor direction
            direction = ((1.0 * (box.r + box.l)) / (kWidth - 1)) - 1.0;

            //
            distance = (1.0 * box.t) / (kHeight);
            // Give the output stream the new image to display
        }
        else {
            found = false;
        }
        // Update the status of whether the cube is found. This is done in a single action so that
        // the main thread never gets partial information.
        SetCubeFound(found, distance, direction);

		// Send the source image to the "Live" stream
		outputStream.PutFrame(mat);

		// Send the processed image to the "Processed" stream
		outputStream2.PutFrame(mono);

		tDuration3 = GetMilliSpan(tStart) - tDuration2 - tDuration1;

		if (m_debug)
			std::cout << "  \tt1=" << tDuration1 << ", t2=" << tDuration2 << ", t3=" << tDuration3 << "\n";
	}
}

void VisionSystem::SetCubeFound(bool found, double distance, double direction) {
    // Lock the mutex for the scope of this function so we can access variable
    // shared between threads
    std::lock_guard<wpi::mutex> lock(m_mutex);
    m_found_object = found;
    m_distance_to_object = distance;
    m_direction_of_object = direction;
}


bool VisionSystem::GetDarkImage() {
    std::lock_guard<wpi::mutex> lock(m_mutex);
	return m_dark_image;
}


void VisionSystem::SetDarkImage(bool value) {
    std::lock_guard<wpi::mutex> lock(m_mutex);
	m_dark_image = value;
}
