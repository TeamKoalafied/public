//==============================================================================
// VisionSystem.h
//==============================================================================

#ifndef VisionSystem_H
#define VisionSystem_H

#include "../TSingleton.h"
#include <frc/commands/Subsystem.h>
#include <wpi/mutex.h>
#include <thread>
#include <cameraserver/CameraServer.h>



namespace frc {
class Compressor;
}

// The vision subsystem processes the image from the camera
class VisionSystem : public TSingleton<VisionSystem>, public frc::Subsystem {
public:
    //==========================================================================
    // Construction

    // Constructor
    VisionSystem();

    // Destructor
    virtual ~VisionSystem();


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

    // Utility functions

    //==========================================================================
    // Return the status of the camera settings - is it set to dark for cube
    // grab or not
    bool GetDarkImage();

    //==========================================================================
    // Set the dark image flag. True makes the image dark for cube grab. False
    // makes it auto exposure
    void SetDarkImage(bool value);


    //==========================================================================
    // Vision Tracking Status

    // Get whether the cube has been found, and if so where
    //
    // distance - Returns the distance to the cube from bottom of image, 0.0 to 1.0. Only
    //      valid if the function returns true.
    // direction - Returns the direction of the cube, -1.0 to 1.0. Only valid if the function returns true.
    //
    // Returns whether the cube as been found
    bool GetCubeFound(double& distance, double& direction);

private:
    //==========================================================================
    // Vision Thread

    // Main function of the vision thread
    void VisionThread();

    // Set whether the cube has been found, and if so where
    //
    // found - Whether the cube has been found
    // distance - The distance to the cube from bottom of image, 0.0 to 1.0. Only valid if 'found' is true.
    // direction - The direction of the cube, -1.0 to 1.0. Only valid if 'found' is true.
    void SetCubeFound(bool found, double distance, double direction);

    //==========================================================================
    // Member Variables

    mutable wpi::mutex m_mutex;     // Mutex to control access to variables shared between threads
    std::thread* m_thread;          // Thread that the vision runs on

    bool m_found_object;            // Whether the cube is currently found
    double m_distance_to_object;    // The distance to the cube from bottom of image, 0.0 to 1.0
    double m_direction_of_object;   // The direction of the cube, -1.0 to 1.0
    bool m_dark_image;              // Sets the exposure to dark for cube finding
    cs::UsbCamera m_camera;			// USB camera object

};

#endif  // VisionSystem_H
