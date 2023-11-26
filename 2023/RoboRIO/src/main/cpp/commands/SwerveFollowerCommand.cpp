// SwerveFollowerCommand.cpp

#include "SwerveFollowerCommand.h"


#include <wpi/MathExtras.h>
#include <units/math.h>

// SwerveFollowerCommand::SwerveFollowerCommand(std::initializer_list<frc2::Subsystem*> requirements) {
//     AddRequirements(requirements);
    
// }



SwerveFollowerCommand::SwerveFollowerCommand(SwerveTrajectory trajectory, frc::SwerveDriveKinematics<4>& kinematics,
                                             std::function<frc::Pose2d()> get_pose_function,
                                             std::function<void(std::array<SwerveFollowerModuleState, 4>)> set_module_states_function,
                                             std::initializer_list<frc2::Subsystem*> requirements) :
 // m_trajectory(trajectory),
  m_kinematics(kinematics),
  m_get_pose_function(get_pose_function),
  m_set_module_states_function(set_module_states_function),
  m_follower(trajectory, kinematics, "SwerveFollowerCommand.csv") {

    AddRequirements(requirements);


    // Setup sensible default values for the parameters
   	// Velocity = 1.17ft/s/V => 14.04ft/s for 12V => 4.28m/s for 12V
 	// Acceleration = ~5ft/s2/V => 60ft/s2 for 12V => 18.29m/s2 for 12V
    // m_follower_parameters.m_kp = 1.0;
    // m_follower_parameters.m_ki = 0.0;
    // m_follower_parameters.m_kd = 0.0;
    // m_follower_parameters.m_kv = 1.0/4.28;
    // m_follower_parameters.m_kv_offset = 0.104;
    // m_follower_parameters.m_ka =  1.0/18.29;
    // m_follower_parameters.m_period_s = 0.02; // 20ms
    // m_follower_parameters.m_wheelbase_width_m = 0.71;
    // m_follower_parameters.m_path_point_spacing = 0.2;  // Spacing between generated path points
    // m_follower_parameters.m_max_velocity = 0.5;
    // m_follower_parameters.m_max_acceleration = 0.25;
    // m_follower_parameters.m_max_velocity_curve = 2.0;
    // m_follower_parameters.m_lookahead_distance = 0.5_m;
    // m_follower_parameters.m_lookahead_factor = 1.5;
    // m_follower_parameters.m_lookahead_curvature_gain = 1.0;
    // m_follower_parameters.m_path_curvature_gain = 1.0;
    // m_follower_parameters.m_lookalong_time = 0.0;

}


void SwerveFollowerCommand::Initialize() {
//   if (m_desiredRotation == nullptr) {
//     m_desiredRotation = [&] {
//       return m_trajectory.States().back().pose.Rotation();
//     };
//   }

//    m_closest_index = 0;
//    m_segment_finished = false;

    m_timer.Reset();
    m_timer.Start();

    m_follower.Initialize();
}

void SwerveFollowerCommand::Execute() {

    frc::Pose2d robot_position = m_get_pose_function();

    std::array<SwerveFollowerModuleState, 4> target_module_states = m_follower.Update(robot_position);  
    m_set_module_states_function(target_module_states);
}

void SwerveFollowerCommand::End(bool interrupted) {
    m_timer.Stop();
    m_follower.End();
}

bool SwerveFollowerCommand::IsFinished() {

    return m_follower.IsFinished();
}
