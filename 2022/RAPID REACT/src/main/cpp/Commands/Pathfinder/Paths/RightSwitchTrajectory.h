#include <pathfinder.h>

Segment g_right_switch_left_trajectory[] = {
	{ 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0 },
	{ 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0 },
	{ 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0 },
	{ 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0 },
};

Segment g_right_switch_right_trajectory[] = {
	{ 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0 },
	{ 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0 },
	{ 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0 },
	{ 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0 },
};

	
int g_right_switch_trajectory_length = sizeof(g_right_switch_left_trajectory)/
								       sizeof(g_right_switch_left_trajectory[0]);
