//==============================================================================
// Leds.cpp
//==============================================================================

#include "VisionSystem.h"
#include "Relay.h"

#include "../RobotConfiguration.h"
#include "Vision/GripOutput.h"

#include <CameraServer.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/core/types.hpp>


//==============================================================================
// Construction

VisionSystem::VisionSystem() :
    TSingleton<VisionSystem>(this),
    frc::Subsystem("VisionSystem") {

    m_found_object = false;
    m_distance_to_object = 0.0;
    m_direction_of_object = 0.0;
    m_dark_image = false;

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

    // Start the vision thread
//    m_thread = new std::thread(&VisionSystem::VisionThread, this);

    cs::UsbCamera camera = CameraServer::GetInstance()->StartAutomaticCapture();
    // Set the resolution
    camera.SetResolution(160, 120);
    // Stop the camera flicker
    camera.SetFPS(25); //max - was 10
    // Start as a normal camera by default
	camera.SetExposureAuto();
	camera.SetWhiteBalanceAuto();

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
    //double *vision_distance_to_object, double *vision_direction_of_object, bool *vision_found_object) {
    int object_min_x = 0; //the 0 in this declaration represents how many rows UP the FIRST white pixel is!
    int object_max_x = 0; //the 0 in this declaration represents how many rows UP the LAST white pixel is!
    int object_min_y = 0; //the 0 in this declaration represents how many Columns (from left to right) the first white pixel is!
    int object_max_y = 0; //the 0 in this declaration represents how many Columns (from left to right) the last white pixel is!
    bool dark_image;
    bool old_dark_image = false;

    // Get the USB camera from CameraServer
    cs::UsbCamera camera = CameraServer::GetInstance()->StartAutomaticCapture();
    // Set the resolution
    camera.SetResolution(160, 120);


    // Fix the camera exposure to suit GRIP for finding yellow cubes with spotlight

    // DEBUG!
    // Stop the camera flicker
    camera.SetFPS(25); //max - was 10

    // Start as a normal camera by default
	camera.SetExposureAuto();
	camera.SetWhiteBalanceAuto();

    // Get a CvSink. This will capture Mats from the Camera
    cs::CvSink cvSink = CameraServer::GetInstance()->GetVideo();

    // Setup a CvSource. This will send images back to the Dashboard. Camera selection is CubeDetect
    cs::CvSource outputStream = CameraServer::GetInstance()->PutVideo("CubeDetect", 160, 120);
    // Match the frame rate to the camera frame rate
    outputStream.SetFPS(10); //max //was 10 //debug

    // Setup an instance of the vision processing pipeline you created with the GRIP tool.
    grip::GripOutput* MyPipeline = new grip::GripOutput();

    // Mat is to be re-used, mat is a mat
    cv::Mat mat;

    while (true) {

    	// Decide what to do with the exposure. Only do this if something changed.
    	dark_image = GetDarkImage();
    	if (dark_image != old_dark_image) {
    		if (dark_image) {
    			// Underexpose the image so that only illuminated objects show up
    			camera.SetExposureManual(5); //max - was 75
    			// Fix the colour cast to suit GRIP for finding yellow cubes with spotlight
    			camera.SetWhiteBalanceManual(5800); //max
    		}
    		else {
    			camera.SetExposureAuto();
    			camera.SetWhiteBalanceAuto();
    		}
    	}
    	old_dark_image = dark_image;

        // Tell the CvSink to grab a frame from the camera and put it
        // in the source mat.  If there is an error notify the output.
        if (cvSink.GrabFrame(mat) == 0) {
            // Send the output the error.
            outputStream.NotifyError(cvSink.GetError());
            // skip the rest of the current iteration
            continue;
        }

        // Run the whole vision processing process on the frame we just grabbed
        MyPipeline->Process(mat);

        // Copy the resulting image back to mat
        mat = *(MyPipeline->GetHslThresholdOutput());
        //mat = cv::Scalar(0);

        object_min_y = mat.rows - 1;
        object_max_y = 0;

        object_min_x = mat.cols - 1;
        object_max_x = 0;
        const int thresh = 10;

        // images from USB cameras are in Blue:Green:Red (bgr) format. Each pixel has 3 unsigned
        // 8-bit values. Define a type that allows us to access each colour.
        struct Pixbgr {
            unsigned char b:8;
            //unsigned char g:8;
            //unsigned char r:8;
        };
        for(int i = 0; i < mat.rows; i++) {
            // const CV_8UC3 *Mi = mat.ptr<CV_8UC3>(i);
            // Make a pixel pointer that lets us see the red green and blue components
            const struct Pixbgr* Mi = mat.ptr<struct Pixbgr>(i);
            for(int j = 0; j < mat.cols; j++)
                // If average of blue, green and red is large enough, then we've found a blob
                if (Mi[j].b > thresh) {
                    if (j < object_min_x)
                        object_min_x = j;
                    if (j > object_max_x)
                        object_max_x = j;
                    if (i < object_min_y)
                        object_min_y = i;
                    if (i > object_max_y)
                        object_max_y = i;
                    }
        }
        bool found = false;
        double distance = 0.0;
        double direction = 0.0;
        if ((object_max_x > (object_min_x + 2)) && (object_max_y > (object_min_y +2))) { //will create offsets for blobs bigger than 3x3 pixels
            // Draw a white rectangle around the blob
            found = true;
            rectangle(mat, cv::Point(object_min_x, object_min_y), cv::Point(object_max_x, object_max_y),
                    cv::Scalar(255, 255, 255), 1);

            // find the average position of the rectangle and convert it to a +/-1.0 offset
            // This needs to be negative to match the motor direction
            direction = ((1.0 * (object_max_x + object_min_x)) / (mat.cols - 1)) - 1.0;

            //new code here
            distance = (1.0 / 120.0) * (120 - object_max_y);
            // Give the output stream the new image to display
        }
        else {
            found = false;
        }
        // Update the status of whether the cube is found. This is done in a single action so that
        // the main thread never gets partial information.
        SetCubeFound(found, distance, direction);

        outputStream.PutFrame(mat);
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
