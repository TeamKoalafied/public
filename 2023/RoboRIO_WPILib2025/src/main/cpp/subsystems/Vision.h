//==============================================================================
// Vision.h
//==============================================================================

#pragma once

#include "../util/Logging.h"

#include <photon/PhotonPoseEstimator.h>
#include <photon/PhotonCamera.h>
#include <photon/simulation/VisionSystemSim.h>
#include <photon/simulation/VisionTargetSim.h>

#include <frc/estimator/SwerveDrivePoseEstimator.h>
#include <frc/Timer.h>

#include <units/angle.h>
#include <units/length.h>

class SwerveDrivebase;

// Vision handles all the calculation of robot position from vision information
class Vision {
public:
    //==========================================================================
    // Constants

    // Values for the standard deviation used to blend a vision pose into the pose estimator
    static constexpr double kStdDevX = 0.2;
    static constexpr double kStdDevY = 0.2;
    static constexpr double kStdDevAngle = 5;


    static constexpr units::meter_t kFieldLength = 54_ft;
    static constexpr units::meter_t kFieldWidth = 27_ft;


    //==========================================================================
    // Construction

    // Constructor
    //
    // drivebase - Drivebase that this vision system is for
    Vision(const SwerveDrivebase* drivebase);


    //==========================================================================
    // Lifetime Functions

    // Setup the vision system
    void Setup();

    // Update the vision system
    void Update(wpi::array<double, 3> std_devs);

    // Setup the vision simulation
    void SetupSimulation();

    // Update the vision simulation
    void UpdateSimulation();


    //==========================================================================
    // Querries

    // Get the robot pose from fused vision and odometry information
    const frc::Pose2d GetPose() const;

    // PhotonLib pose estimation from the last update
    std::optional<photon::EstimatedRobotPose> GetLatestPhotonPose() const { return m_latest_photon_pose; }

    // PhotonLib result from the last update, only valid of there is a pose
    photon::PhotonPipelineResult GetLatestPhotonResult() const { return m_latest_photon_result; }

    // Whether the pose was updated in the last cycle
    bool GetUpdatingPose() const { return m_updating_pose; }

    // Beginning targing the speaker
    void StartSpeakerTargeting() const;

    // TODO Document angle and distance convention
    // Get information about the current position of our alliance's speaker centre april tag
    //
    // ambiguity - Returns the pose ambiguity of the speaker centre april tag (-1 if not visible)
    // angle - Returns the angle to the speaker centre april tag
    // distance - Returns the distance to the speaker centre april tag
    //
    // Returns whether the speaker centre april tag is currently visible
    bool GetSpeakerTargetInfo(double& ambiguity, units::degree_t& angle, units::meter_t& distance) const;

    // Stop targing the speaker
    void StopSpeakerTargeting() const;

    // Get information on making a lob shot
    //
    // angle - Angle the robot is away from the correct direction for a lob shot
    // distance - Returns the distance to lob to the target point (in the amp corner)
    //
    // Returns whether a lob shot is valid
    bool GetLobTargetInfo(units::degree_t& angle, units::meter_t& distance) const;


    //==========================================================================
    // April Tag Ids

    // Get the April tag id for the tag at the centre of our speaker
    int GetSpeakerCentreTagId() const;

    // Get the April tag id for the tag at the side of our speaker
    int GetSpeakerSideTagId() const;

    // Get the April tag id for the tag at over the amp
    int GetAmpTagId() const;

    // Get an April tag from its ID
    //
    // id - Fucial ID of the tag
    const frc::AprilTag* GetAprilTag(int id) const;

    //==========================================================================
    // Operations

    // Reset the robot pose
    void ResetPose(const frc::Pose2d& pose);

    // Get whether updating the pose from April tags is enabled
    bool GetUpdatePoseEnabled() const {
        return m_update_pose_enabled;
    }

    // Set whether updating the pose from April tags is enabled
    void SetUpdatePoseEnabled(bool set) {
        m_update_pose_enabled = set;
    }

private:
    //==========================================================================
    // Private Nested Types

    struct TargetInfo {
        double m_ambiguity;
        units::degree_t m_angle;
        units::meter_t m_distance;
        units::meter_t m_3d_distance;
        units::degree_t m_gyro_angle;
        units::second_t m_latency;
        units::second_t m_time;
        units::degree_t m_pitch;
        double m_area;
    };


    //==========================================================================
    // Vision Updates

    // Record the current information about the speaker target
    void RecordTargetInfo();

    // Update pose any time that PhotoLib gives us one
    void UpdatePoseAlways(wpi::array<double, 3> std_devs);

    // Update pose only if we have at least two tags that match 
    void UpdatePoseForMultipleTags(wpi::array<double, 3> std_devs);


    //==========================================================================
    // Member Variables

    static const int TARGET_HISTORY_LENGTH = 10;                 // Length of the buffer for target history

    const SwerveDrivebase* m_drivebase = nullptr;               //  Drivebase that this vision system is for
    bool m_update_pose_enabled = true;                          // Whether updating the pose from April tags is enabled
    frc::SwerveDrivePoseEstimator<4>* m_swerve_estimator = nullptr;
                                                                // Pose estimator that tracks the robot position
    photon::PhotonPoseEstimator* m_photon_estimator = nullptr;  // Photon Vision pose estimator
    std::vector<frc::AprilTag> m_april_tags;                    // List of April tags on the field
    std::optional<photon::EstimatedRobotPose> m_latest_photon_pose;
                                                                // PhotonLib pose estimation from the last update
    bool m_updating_pose;                                       // Whether the pose was updated in the last cycle
    photon::PhotonCamera m_camera{"april"};
    photon::PhotonCameraSim m_april_sim{&m_camera};
    photon::PhotonPipelineResult m_latest_photon_result;        // PhotonLib result from the last update, only valid of there is a pose
    TargetInfo m_target_history_buffer[TARGET_HISTORY_LENGTH];
    int m_target_history_buffer_pos;

    photon::VisionSystemSim* m_sim_vision = nullptr;            // Simulation system for the vision
    mutable Logging::CsvFile* m_csv_log_file = nullptr;         // CSV file for logging debugging stuff
    mutable frc::Timer m_log_timer;                             // Time for logging
};

