// // SwerveTrajectoryTestSuite.cpp


// #include <gtest/gtest.h>

// #include "../../main/cpp/pathfollower/SwerveTrajectoryGenerator.h"

// #include <iostream>

// #include <vector>
// #include <units/acceleration.h>
// #include <units/angle.h>
// #include <units/angular_acceleration.h>
// #include <units/angular_velocity.h>
// #include <units/length.h>
// #include <units/velocity.h>


// TEST(SwerveTrajectoryGeneratorTestSuite, TestStraight) {
//     SwerveTrajectoryGenerator::Parameters parameters;
//     parameters.m_path_point_spacing = 0.1_m;
//     parameters.m_max_velocity = 2.0_mps;
//     parameters.m_max_acceleration = 1.0_mps_sq;
//     parameters.m_max_velocity_curve = 1.0_mps;

//     std::vector<Bezier3_2D<units::meter_t> > bezier_list {
//         { { 0.0_m, 0.0_m }, { 1.0_m, 0.0_m }, { 2.0_m, 0.0_m }, { 3.0_m, 0.0_m }}
//     };
//     std::vector<units::degree_t> angles {
//         {  0.0_deg, 0.0_deg }
//     };

//     SwerveTrajectory trajectory;
//     SwerveTrajectoryGenerator::Generate(trajectory, bezier_list, angles, parameters);
//     trajectory.LogToFile("Q:\\Dev\\FRC\\Code\\Koalafied_2023\\ChargedUp\\RoboRIO\\testresults\\TrajectoryStraight.csv");
//     std::cout << "Trajectory has " << trajectory.GetPointCount() << " points V2\n";
// }

// TEST(SwerveTrajectoryGeneratorTestSuite, TestSpin) {
//     SwerveTrajectoryGenerator::Parameters parameters;
//     parameters.m_path_point_spacing = 0.1_m;
//     parameters.m_max_velocity = 2.0_mps;
//     parameters.m_max_acceleration = 1.0_mps_sq;
//     parameters.m_max_velocity_curve = 1.0_mps;

//     std::vector<Bezier3_2D<units::meter_t> > bezier_list {
//         { { 0.0_m, 0.0_m }, { 1.0_m, 0.0_m }, { 2.0_m, 0.0_m }, { 3.0_m, 0.0_m }}
//     };
//     std::vector<units::degree_t> angles {
//         {  0.0_deg, 90.0_deg }
//     };


//     SwerveTrajectory trajectory;
//     SwerveTrajectoryGenerator::Generate(trajectory, bezier_list, angles, parameters);
//     trajectory.LogToFile("Q:\\Dev\\FRC\\Code\\Koalafied_2023\\ChargedUp\\RoboRIO\\testresults\\TrajectorySpin.csv");
//     std::cout << "Trajectory has " << trajectory.GetPointCount() << " points V2\n";
// }