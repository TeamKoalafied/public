//==============================================================================
// RobotPath.h
//==============================================================================

#include "Bezier3_2D.h"
#include <vector>
#include <units/length.h>

#pragma once

class RobotPath {
public:
    std::vector<Bezier3_2D<units::meter_t> > m_bezier_list;
    std::vector<units::degree_t> m_angles;
};
