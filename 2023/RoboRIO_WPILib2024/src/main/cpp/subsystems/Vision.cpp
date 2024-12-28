//==============================================================================
// Vision.cpp
//==============================================================================

#include "Vision.h"

#include "SwerveDrivebase.h"

#include "../RobotConfiguration.h"
#include "../util/KoalafiedUtilities.h"
#include "../util/Logging.h"

#include <algorithm>
#include <iostream>
#include <frc/DriverStation.h>
#include <photon/PhotonUtils.h>

namespace RC = RobotConfiguration;
using namespace Logging;


//==============================================================================
// Construction

Vision::Vision(const SwerveDrivebase* drivebase) {
    m_drivebase = drivebase;

    // Initialise the target history buffer to 'no tag visible' and set the position
    // for the next element to the beginning.
    for (int i = 0; i < TARGET_HISTORY_LENGTH; i++) {
        m_target_history_buffer[i].m_ambiguity = -1;
    }
    m_target_history_buffer_pos = 0;

    m_updating_pose = false;
}


//==============================================================================
// Lifetime Functions

void Vision::Setup() {
    // Get the swerve module positions
    const wpi::array<const SwerveModule*,4>& modules = m_drivebase->GetModules();
    const wpi::array<frc::SwerveModulePosition,4U> module_positions{
        modules[0]->GetPosition(),
        modules[1]->GetPosition(),
        modules[2]->GetPosition(),
        modules[3]->GetPosition()
    };

    // Initialise the estimator from the kinematics, pigeon and starting pose
    m_swerve_estimator = new frc::SwerveDrivePoseEstimator(m_drivebase->GetKinematics(),
                                                           m_drivebase->GetPigeonRotation(), 
                                                           module_positions,
                                                           frc::Pose2d{0_m, 0_m, 0_rad});                                                           

    // Create a list of april tags on the field. This will be used for simulation too.
    /* TODO Make some way to initialise for the game or testing
    m_april_tags.push_back({1, frc::Pose3d(593.68_in, 9.68_in, 53.88_in,    
                                frc::Rotation3d(0_deg, 0_deg, 120_deg))});
    m_april_tags.push_back({2, frc::Pose3d(637.21_in, 34.79_in, 53.88_in, 
                                frc::Rotation3d(0_deg, 0_deg, 120_deg))});
    m_april_tags.push_back({3, frc::Pose3d(652.73_in, 196.17_in, 57.13_in,  // 3: Red speaker side
                                frc::Rotation3d(0_deg, 0_deg, 180_deg))});
    m_april_tags.push_back({4, frc::Pose3d(652.73_in, 218.42_in, 57.13_in,  // 4: Red speaker centre
                                frc::Rotation3d(0_deg, 0_deg, 180_deg))});
    m_april_tags.push_back({5, frc::Pose3d(578.77_in, 323_in, 53.38_in,     // 5: Red amp
                                frc::Rotation3d(0_deg, 0_deg, 270_deg))});
    m_april_tags.push_back({6, frc::Pose3d(72.5_in, 323_in, 53.88_in,       // 6: Blue amp
                                frc::Rotation3d(0_deg, 0_deg, 270_deg))});
    m_april_tags.push_back({7, frc::Pose3d(-1.5_in, 218.42_in, 57.13_in,  // 7: Blue speaker centre
                               frc::Rotation3d(0_deg, 0_deg, 0_deg))});
    m_april_tags.push_back({8, frc::Pose3d(-1.5_in, 196.17_in, 57.13_in,  // 8: Blue speaker side
                               frc::Rotation3d(0_deg, 0_deg, 0_deg))});

    m_april_tags.push_back({9, frc::Pose3d(593.68_in, 9.68_in, 53.88_in,    
                                frc::Rotation3d(0_deg, 0_deg, 120_deg))});
    m_april_tags.push_back({10, frc::Pose3d(593.68_in, 9.68_in, 53.88_in,    
                                frc::Rotation3d(0_deg, 0_deg, 120_deg))});
    m_april_tags.push_back({11, frc::Pose3d(593.68_in, 9.68_in, 53.88_in,    
                                frc::Rotation3d(0_deg, 0_deg, 120_deg))});
    m_april_tags.push_back({12, frc::Pose3d(593.68_in, 9.68_in, 53.88_in,    
                                frc::Rotation3d(0_deg, 0_deg, 120_deg))});
    m_april_tags.push_back({13, frc::Pose3d(593.68_in, 9.68_in, 53.88_in,    
                                frc::Rotation3d(0_deg, 0_deg, 120_deg))});
    m_april_tags.push_back({14, frc::Pose3d(593.68_in, 9.68_in, 53.88_in,    
                                frc::Rotation3d(0_deg, 0_deg, 120_deg))});
    m_april_tags.push_back({15, frc::Pose3d(593.68_in, 9.68_in, 53.88_in,    
                                frc::Rotation3d(0_deg, 0_deg, 120_deg))});
    m_april_tags.push_back({16, frc::Pose3d(593.68_in, 9.68_in, 53.88_in,    
                                frc::Rotation3d(0_deg, 0_deg, 120_deg))});
*/



#ifdef _WIN32
            // // In the simulator angles are reversed
            // target_info.m_angle = -target_info.m_angle;
            // CAMERA_PITCH_RADIANS = -CAMERA_PITCH_RADIANS;
    // For testing we have a single tag 1m back from the origin and 0.7m off the ground
    m_april_tags.push_back({3, frc::Pose3d(-1_m, 0_m, 0.7_m,
                               frc::Rotation3d(0_deg, 0_deg, 180_deg))});
#else
    // For testing we have a single tag 1m back from the origin and 0.7m off the ground
    m_april_tags.push_back({3, frc::Pose3d(-1_m, 0_m, 0.7_m,
                               frc::Rotation3d(0_deg, 0_deg, 0_deg))});

#endif


    // Create the photon pose estimator
    frc::AprilTagFieldLayout aprilTags{m_april_tags, kFieldLength, kFieldWidth};
    m_photon_estimator = new photon::PhotonPoseEstimator(
//        aprilTags, photon::AVERAGE_BEST_TARGETS, photon::PhotonCamera{"april"}, RC::kRobotToCam);
        aprilTags, photon::CLOSEST_TO_REFERENCE_POSE, photon::PhotonCamera{"april"}, RC::kRobotToCam);

}

void Vision::Update(wpi::array<double, 3> std_devs) {
    // Get the swerve module positions
    const wpi::array<const SwerveModule*,4>& modules = m_drivebase->GetModules();
    const wpi::array<frc::SwerveModulePosition,4U> module_positions{
        modules[0]->GetPosition(),
        modules[1]->GetPosition(),
        modules[2]->GetPosition(),
        modules[3]->GetPosition()
    };

    // Update the estimator from the pigeon values and swerve module state 
    m_swerve_estimator->Update(m_drivebase->GetPigeonRotation(), module_positions);

    // Update the vision
    m_latest_photon_pose = m_photon_estimator->Update();
    m_latest_photon_result = m_photon_estimator->GetCamera()->GetLatestResult();

    // Update the pose if enabled
    m_updating_pose = false;
    if (m_update_pose_enabled) {
        UpdatePoseAlways(std_devs);
        //UpdatePoseForMultipleTags(std_devs);
    }

    // Record the current information about the speaker target
    RecordTargetInfo();
}

void Vision::SetupSimulation() {
    // The simulated camera does not set the version, because it is commented out in the code
    // https://github.com/PhotonVision/photonvision/blob/master/photon-lib/src/main/native/include/photonlib/SimPhotonCamera.h
    // So we disable the version check.
    // NOTE: This still does not work, so we get a bunch of warning messages when simulating!
    // Warning messages can be fixed by hacking the code in the SimPhotonCamera.h header file
    photon::PhotonCamera::SetVersionCheckEnabled(false);

    units::degree_t camFOV = 70_deg;
    // frc::Transform3d robotToCam =
    //                 frc::Transform3d(frc::Translation3d(0.34_m, 0_m, 0.49_m),
    //                 frc::Rotation3d(0_rad, 0_rad, 0_rad));
    units::meter_t led_range = 20_m;
    int camResolutionWidth = 1280; // pixels
    int camResolutionHeight = 800; // pixels
    double minTargetArea = 10; // square pixels
    m_sim_vision = new photon::SimVisionSystem("april", camFOV, RC::kRobotToCam.Inverse(), led_range,
                                                 camResolutionWidth, camResolutionHeight, minTargetArea);
    
    // Add the April tags to the simulation
    for (frc::AprilTag april_tag : m_april_tags) {
        m_sim_vision->AddSimVisionTarget(photon::SimVisionTarget(april_tag.pose, 8_in, 8_in, april_tag.ID));
    }
}

void Vision::UpdateSimulation() {
    m_sim_vision->ProcessFrame(GetPose());
}


//==============================================================================
// Querries

const frc::Pose2d Vision::GetPose() const {
    return m_swerve_estimator->GetEstimatedPosition();
}

void Vision::StartSpeakerTargeting() const {
    // If not logging yet open the file and write the header
    const bool LOG_TARGETING = false;
    if (LOG_TARGETING && m_csv_log_file == nullptr) {
        const char* const RESULT_FILENAME = "VisionTargetInfo.csv";
        m_csv_log_file = new Logging::CsvFile();
        m_csv_log_file->OpenLogFile(RESULT_FILENAME, std::ios::out | std::ios::app);
        if (m_csv_log_file->Fail()) {
            // If opening the file fails stop logging
            std::cout << "ERROR Failed to open log file\n";
            delete m_csv_log_file;
            m_csv_log_file = nullptr;
        }
        else {
            *m_csv_log_file << "Vision" << "\n";
            *m_csv_log_file << "Time" << "Ambiguity" << "Area" << "Angle" << "Pitch";
            *m_csv_log_file << "Distance" << "Distance3D" << "Gyro" << "Latency" << "\n";
        }
        m_log_timer.Restart();
    }
}

bool Vision::GetSpeakerTargetInfo(double& ambiguity, units::degree_t& angle, units::meter_t& distance) const {

    units::degree_t current_gyro_angle = m_drivebase->GetPigeonHeading();

    // Loop through the history buffer from the oldest to newest. We start at the the position in
    // the history buffer where the next element will go as that is the oldest. Sum the total
    // angle, distance and ambiguity, but weight the most recent entries higher.
    int pos = m_target_history_buffer_pos;
    int total_weight = 0;
    angle = 0_deg;
    distance = 0_m;
    for (int i = 1; i <= TARGET_HISTORY_LENGTH; i++) {
        const TargetInfo target_info = m_target_history_buffer[pos];

        // Ignore items with -1 ambiguity as they are not valid
        if (target_info.m_ambiguity != -1) {
            int weight = i;

            // Adjust the target angle information for any rotation of the robot (as measured by the gyro) since
            // the measurement was taken.
            units::degree_t adjusted_angle = target_info.m_angle + current_gyro_angle - target_info.m_gyro_angle;
            adjusted_angle = KoalafiedUtilities::NormaliseAngle(adjusted_angle);

            ambiguity += target_info.m_ambiguity * weight;
            angle += adjusted_angle * weight;
            distance += target_info.m_distance * weight;
            total_weight += weight;
        }

        // Move to the next element in the circular history buffer
        pos++;
        if (pos >= TARGET_HISTORY_LENGTH) pos = 0;
    }

    // If not valid items were found return that there is not target info
    if (total_weight == 0) {
        ambiguity = -1;
        return false;
    }

    // Divide by the total weight to get the weighted average values
    ambiguity /= total_weight;
    angle /= total_weight;
    distance /= total_weight;

    // Apply a linear adjustment to the measured distance based on test data
    // =(B3+0.046)/1.194
    //distance = (distance + 0.046_m)/1.194;

    // Reject targeting information over a maximum distance away. It won't be accurate and we cannot
    // shoot that far anyway. This mainly happens in the simulator.
    const units::meter_t MAX_SHOOTING_DISTANCE = 5_m;   
    if (distance > MAX_SHOOTING_DISTANCE) return false;

    // Return that the target is valid
    return true;
}

void Vision::StopSpeakerTargeting() const {
    if (m_csv_log_file != nullptr) {
        m_csv_log_file->Close();
        m_csv_log_file = nullptr;
    } 
}

bool Vision::GetLobTargetInfo(units::degree_t& angle, units::meter_t& distance) const {
    // TODO implement lob targeting using pose information
    // frc::Pose2d lob_target;
    // bool blue = KoalafiedUtilities::IsBlueAlliance();
    // if (blue) {
    //     lob_target = {40_in, 283_in, 0_deg};
    // } else {
    //     lob_target = {610_in, 283_in, 0_deg};
    // }
    // units::inch_t dx = lob_target.X() - m_swerve_estimator->GetEstimatedPosition().X();
    // units::inch_t dy = lob_target.Y() - m_swerve_estimator->GetEstimatedPosition().Y();;
    // units::inch_t distance = units::math::sqrt(units::math::pow<2>(dx)+units::math::pow<2>(dy));
    // if (distance < 3_m) {
    //     return false;
    // }
    // angle = units::math::atan2(dy,dx);


    // Get the point on the field that we aim lob shots towards and the X position of the end of the
    // field we are aiming at (our end). Start with Blue alliance and flip them if we are Red. 
    frc::Translation2d lob_target {40_in, 283_in};
    units::meter_t target_end { 0_m };
    if (!KoalafiedUtilities::IsBlueAlliance()) {
        lob_target = frc::Translation2d(kFieldLength - lob_target.X(), lob_target.Y());
        target_end = kFieldLength - target_end;
    }

    // Distance from our end of the field to the other alliance wing
    const units::meter_t OTHER_WING_DISTANCE = 76.1_in + 345.91_in; // From LayoutMarkingDiagram.pdf

    // 'Radius' of the bummpers. That is furthest distance the bumpers go from the centre of the robot.
    units::meter_t drive_base_radius = units::math::hypot(RC::kDriveBaseBumperLength, RC::kDriveBaseBumperWidth) / 2.0;

    // If the current robot position is such that we could have bumpers in the other alliance wing then lobbing is
    // not allowed (G414 - Foul).
    const frc::Pose2d& robot_pose = m_swerve_estimator->GetEstimatedPosition();
    if (units::math::abs(robot_pose.X() - target_end) + drive_base_radius > OTHER_WING_DISTANCE) {
        angle = 0_deg;
        distance = 0_m;
        return false;
    }

    // If we are too close to the lob target then lobbing is not allowed. We will could overshoot out of the field.
    frc::Translation2d delta = lob_target - robot_pose.Translation();
    const units::meter_t MIN_LOB_DISTANCE = 6_m;
    distance = delta.Norm();
    if (distance < MIN_LOB_DISTANCE) {
        return false;
    }

    // Calculate the angle to the log target and then convert it to be relative to the shooter (that is
    // 180 degrees from the robot rotation direction)
    units::degree_t log_field_relative_angle = delta.Angle().Degrees();
    angle = -KoalafiedUtilities::NormaliseAngleDiff(log_field_relative_angle, robot_pose.Rotation().RotateBy(180_deg).Degrees());

    // Lob is allowed
    return true;
}


//==============================================================================
// April Tag Ids

int Vision::GetSpeakerCentreTagId() const {
    return KoalafiedUtilities::IsBlueAlliance() ? 7 : 4;
}

int Vision::GetSpeakerSideTagId() const {
    return KoalafiedUtilities::IsBlueAlliance() ? 8 : 3;
}

int Vision::GetAmpTagId() const {
    return KoalafiedUtilities::IsBlueAlliance() ? 6 : 5;
}

const frc::AprilTag* Vision::GetAprilTag(int id) const {
    std::vector<frc::AprilTag>::const_iterator it = std::find_if(m_april_tags.cbegin(), m_april_tags.cend(),
                                                           [id] (const frc::AprilTag& t) { return t.ID == id; } );
    if (it == m_april_tags.end()) {
        return nullptr;
    }
    else {
        return &(*it);
    }                                             
}


//==============================================================================
// Operations

void Vision::ResetPose(const frc::Pose2d& pose) {
    // Get the swerve module positions
    const wpi::array<const SwerveModule*,4>& modules = m_drivebase->GetModules();
    const wpi::array<frc::SwerveModulePosition,4U> module_positions {
        modules[0]->GetPosition(),
        modules[1]->GetPosition(),
        modules[2]->GetPosition(),
        modules[3]->GetPosition()
    };
    
    m_swerve_estimator->ResetPosition(m_drivebase->GetPigeonRotation(),
                                       module_positions,
                                        pose);
}


//==============================================================================
// Vision Updates

void Vision::RecordTargetInfo() {
    const units::meter_t CAMERA_HEIGHT_METERS = RC::kRobotToCam.Z();
    const units::meter_t TARGET_HEIGHT_METERS = 57.13_in;
    units::radian_t CAMERA_PITCH_RADIANS = -RC::kRobotToCam.Rotation().Y();

    // Get the next target info to record to and move the position forward
    TargetInfo& target_info = m_target_history_buffer[m_target_history_buffer_pos];
    m_target_history_buffer_pos++;
    if (m_target_history_buffer_pos >= TARGET_HISTORY_LENGTH) m_target_history_buffer_pos = 0;

    // Record the gyro angle
    target_info.m_gyro_angle = m_drivebase->GetPigeonHeading();

    // Speaker centre target was not found so record that
    target_info.m_ambiguity = -1.0;
    target_info.m_angle = 0_deg;
    target_info.m_distance = 0_m;
    target_info.m_3d_distance = 0_m;
    target_info.m_latency = 0_s;
    target_info.m_time = m_log_timer.Get();
    target_info.m_pitch = 0_deg;
    target_info.m_area = 0.0;

    // Search throught the current targets for the speaker centre target
    int speaker_id = GetSpeakerCentreTagId();
    for (const photon::PhotonTrackedTarget& target :  m_latest_photon_result.GetTargets()) {
        if (target.fiducialId == speaker_id) {
            // Found the speaker centre target so return information about it
            target_info.m_ambiguity = target.poseAmbiguity;
            target_info.m_angle = units::degree_t(target.yaw);
            target_info.m_latency = m_latest_photon_result.GetLatency();
            target_info.m_pitch = units::degree_t(target.pitch);
            target_info.m_area = target.area;

#ifdef _WIN32
            // In the simulator angles are reversed
            target_info.m_angle = -target_info.m_angle;
            CAMERA_PITCH_RADIANS = -CAMERA_PITCH_RADIANS;
#endif

            // Calculate the distance to the target using just the anglular position (pitch) of the
            // target in the camera.
            // https://docs.photonvision.org/en/latest/docs/programming/photonlib/using-target-data.html#calculating-distance-to-target
            target_info.m_distance = photon::PhotonUtils::CalculateDistanceToTarget(
                                CAMERA_HEIGHT_METERS,
                                TARGET_HEIGHT_METERS,
                                CAMERA_PITCH_RADIANS,
                                units::degree_t(target_info.m_pitch));
            target_info.m_3d_distance = units::math::sqrt(units::math::pow<2>(target.bestCameraToTarget.X()) + units::math::pow<2>(target.bestCameraToTarget.Y()));
            break;
        }
    }

    if (m_csv_log_file != nullptr) {
        *m_csv_log_file << target_info.m_time.value() << target_info.m_ambiguity << target_info.m_area << target_info.m_angle.value()
                        << target_info.m_pitch.value() << target_info.m_distance.value() << target_info.m_3d_distance.value()
                         << target_info.m_gyro_angle.value()<< target_info.m_latency.value() << "\n";
    }
}

void Vision::UpdatePoseAlways(wpi::array<double, 3> std_devs) {
    if (m_latest_photon_pose) {
        // If there is a result use it to update the estimated position, as long as the area is large enough
        bool is_autonomous = frc::DriverStation::IsAutonomous();
        bool is_target_large = m_latest_photon_result.GetBestTarget().GetArea() > 0.1;

        bool m_use_april_tags = true;
        if (m_use_april_tags && !is_autonomous && is_target_large) {
            // Using the time from the estimated pose in the simulator causes a crash in AddVisionMeasurement()
            // so to prevent that we just use the current time.
            m_updating_pose = true;
            m_swerve_estimator->AddVisionMeasurement(m_latest_photon_pose.value().estimatedPose.ToPose2d(),
                                                     m_latest_photon_result.GetTimestamp(), std_devs);
        }
    }
}

void Vision::UpdatePoseForMultipleTags(wpi::array<double, 3> std_devs) {
    if (!m_latest_photon_pose) return;

    // Create a list of poses for the robot from any tags that meet a minimum area and
    // maximum ambiguity requirement. 
    const double MIN_TARGET_AREA = 0.1;
    const double MAX_POSE_AMBIGUITY = 0.2;
    std::vector<frc::Pose2d> pose_list;
    for (const photon::PhotonTrackedTarget& target :  m_latest_photon_result.GetTargets()) {
        // Skip tags that are too small or ambiguous
        if (target.area < MIN_TARGET_AREA) continue;
        if (target.poseAmbiguity > MAX_POSE_AMBIGUITY) continue;

        // Skip target that are not on our April tag list       
        const frc::AprilTag* tag = Vision::GetAprilTag(target.fiducialId);
        if (tag == nullptr) continue;

        if (tag->ID < 3 || tag->ID > 8) continue;

        // Robot pose is 'tag' pose + 'tag to camera' transform + 'camera to robot' transform
        frc::Transform3d target_to_camera = target.bestCameraToTarget.Inverse();
        frc::Pose3d robot_pose = tag->pose + target_to_camera + RC::kRobotToCam.Inverse();

        // Record the robot pose
        pose_list.push_back(robot_pose.ToPose2d());
    }

    // Do not update unless there are at least two tags visible
    if (pose_list.size() < 2) return;

    // Calculate the minimum, maximum and sum values for the X, Y and angle. For the angle
    // calculate everything as a delta from the angle of the first pose. This prevents any
    // issues with wrap around messing up the sum. (For example consider the case of averaging
    // angles of -179_deg and 179_deg. The average should be 180_deg, rather than 0_deg.)
    frc::Pose2d base_pose = pose_list[0];
    units::meter_t x_min = base_pose.X();
    units::meter_t x_max = base_pose.X();
    units::meter_t x_sum = base_pose.X();
    units::meter_t y_min = base_pose.Y();
    units::meter_t y_max = base_pose.Y();
    units::meter_t y_sum = base_pose.Y();
    units::degree_t angle_delta_min = 0_deg;
    units::degree_t angle_delta_max = 0_deg;
    units::degree_t angle_delta_sum = 0_deg;
    for (int i = 1; i < (int)pose_list.size(); i++) {
        frc::Pose2d& pose = pose_list[i];
        if (x_min > pose.X()) x_min = pose.X();
        if (x_max < pose.X()) x_max = pose.X();
        x_sum += pose.X();
        if (y_min > pose.Y()) y_min = pose.Y();
        if (y_max < pose.Y()) y_max = pose.Y();
        y_sum += pose.Y();

        units::degree_t angle_delta = (pose.Rotation() - base_pose.Rotation()).Degrees();
        if (angle_delta_min > angle_delta) angle_delta_min = angle_delta;
        if (angle_delta_max < angle_delta) angle_delta_max = angle_delta;
        angle_delta_sum += angle_delta;
    }

    units::meter_t x_range = x_max - x_min;
    units::meter_t y_range = y_max - y_min;
    units::degree_t angle_delta_range = angle_delta_max - angle_delta_min;

    units::meter_t x_average = x_sum / pose_list.size();
    units::meter_t y_average = y_sum / pose_list.size();
    units::degree_t angle_average = base_pose.Rotation().Degrees() + angle_delta_sum / pose_list.size();

    // Output information about the current targets and pose, but only every second
    /*
    static int counter = 0;
    const int LOG_PERIOD = 50;
    if ((counter++ % LOG_PERIOD) == 0) {
        std::cout << "\nRobot " << m_latest_photon_pose.value().estimatedPose.ToPose2d() << "\n";

        for (const photon::PhotonTrackedTarget& target :  m_latest_photon_result.GetTargets()) {
            std::cout << "    Target " << target.fiducialId;
            
            const frc::AprilTag* tag = Vision::GetAprilTag(target.fiducialId);
            if (tag != nullptr) {
                frc::Transform3d target_to_camera = target.bestCameraToTarget.Inverse();
                frc::Pose3d robot_pose = tag->pose + target_to_camera + RC::kRobotToCam.Inverse();
                std::cout << robot_pose.ToPose2d();
            }
            std::cout << "\n";
        }

        std::cout << "X Range " << x_range.value() << "\n";
        std::cout << "Y Range " << y_range.value() << "\n";
        std::cout << "Angle Range " << angle_delta_range.value() << "\n";
        std::cout << "Combined X " << x_average.value() << "Y " << y_average.value() << " Angle " << angle_average.value() << "\n";
    }
    */

    // Do not update if the difference bewtween the poses is too large
    const units::meter_t DISTANCE_LIMIT = 0.1_m;
    const units::degree_t ANGLE_LIMIT = 3_deg;
    if (x_range > DISTANCE_LIMIT || y_range > DISTANCE_LIMIT || angle_delta_range > ANGLE_LIMIT) return;

    // Don't update in autonomous
    bool is_autonomous = frc::DriverStation::IsAutonomous();
    if (is_autonomous) return;

    // Update the estimated pose from the average of the poses from the tags
    m_updating_pose = true;
    frc::Pose2d combined_pose(x_average, y_average, angle_average);
    m_swerve_estimator->AddVisionMeasurement(combined_pose, m_latest_photon_result.GetTimestamp(), std_devs);
}

