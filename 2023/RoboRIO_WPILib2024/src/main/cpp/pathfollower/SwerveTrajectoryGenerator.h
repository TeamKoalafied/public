//==============================================================================
// SwerveTrajectoryGenerator.h
//==============================================================================
#pragma once

#include "Bezier3_2D.h"
#include "SwerveTrajectory.h"

#include <vector>
#include <units/acceleration.h>
#include <units/angle.h>
#include <units/angular_acceleration.h>
#include <units/angular_velocity.h>
#include <units/length.h>
#include <units/velocity.h>



class SwerveTrajectoryGenerator {
public:
    //==========================================================================
    // Public Nested Types

    // Parameters that control how the path is generated and followed
    struct Parameters {
        units::meter_t m_path_point_spacing;                    // Spacing between generated path points
        units::meters_per_second_t m_max_velocity;              // Maximum velocity allowed on the path
        units::meters_per_second_squared_t m_max_acceleration;  // Maximum acceleration allowed on the path
        units::meters_per_second_t m_max_velocity_curve;        // Scale factor to determine maximum cornering velocity from curvature
    };

    // TODO This function should probably return the trajectory in some clever C++ way that avoids copying

    static void Generate(SwerveTrajectory& trajectory, const std::vector<Bezier3_2D<units::meter_t> >& bezier_list,
                         const std::vector<units::degree_t>& angles, const Parameters& parameters);


private:

};