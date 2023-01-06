//==============================================================================
// PathController.cpp
//==============================================================================

#include "PathController.h"

#include <fstream>
#include <iostream>
#include <iomanip>

//==============================================================================
// Construction and Destruction

PathController::PathController() {
	m_record_samples = true;
}

PathController::~PathController() {
}



//==============================================================================
// Test Logging

void PathController::WriteTestSampleToFile()
{
	if (!m_record_samples) return;

	const char* const RESULT_FILENAME = "/home/lvuser/TestPathController.csv";
	std::ofstream results_file;
	results_file.open(RESULT_FILENAME, std::ios::out | std::ios::app);
	if (results_file.fail()) {
		std::cout << "PathController::WriteTestSampleToFile() - Failed to open result file\n";
		return;
	}

	// Write the test heading
	WriteSampleHeading(results_file);

	// Write the sample times in a single line
	results_file << "\"Time\"";
	int total_samples = m_sample_list.size();
	for (int i = 0; i < total_samples; i++) results_file << "," << std::setprecision(3) << m_sample_list[i].m_time_s;
	results_file << "\n";

	// Write the left and right encoder positions
	results_file << "\"Left Distance\"";
	for (int i = 0; i < total_samples; i++) results_file << "," << m_sample_list[i].m_left_distance_m;
	results_file << "\n";
	results_file << "\"Right Distance\"";
	for (int i = 0; i < total_samples; i++) results_file << "," << m_sample_list[i].m_right_distance_m;
	results_file << "\n";

	// Write the heading
	results_file << "\"Gyro Heading\"";
	for (int i = 0; i < total_samples; i++) results_file << "," << std::setprecision(3) << m_sample_list[i].m_gyro_heading_deg;
	results_file << "\n";

	// Write the left and right outputs
	results_file << "\"Left Output\"";
	for (int i = 0; i < total_samples; i++) results_file << "," << std::setprecision(3) << m_sample_list[i].m_left_output;
	results_file << "\n";
	results_file << "\"Right Output\"";
	for (int i = 0; i < total_samples; i++) results_file << "," << std::setprecision(3) << m_sample_list[i].m_right_output;
	results_file << "\n";

	// Write the mechanism action
	results_file << "\"Mechanism Action\"";
	for (int i = 0; i < total_samples; i++) {
		results_file << ",";
		if (m_sample_list[i].m_mechanism_action != NULL) results_file << m_sample_list[i].m_mechanism_action;
	}
	results_file << "\n";

	// Write a completely blank line to mark the end of this test
	results_file << "\n";
}


//==========================================================================
// Sample Recording

void PathController::SetupSampleRecording() {
	if (!m_record_samples) return;

	// Clear the list of samples
	m_sample_list.clear();

	// Reset and start the timer
	m_timer.Reset();
	m_timer.Start();
}

void PathController::RecordSample(double left_distance_m, double right_distance_m, double gyro_heading_deg,
				  	  	  	      double left_output, double right_output, const std::string* mechanism_action) {
	if (!m_record_samples) return;

	// Record the data sample for this time step
	Sample sample;
	sample.m_time_s = m_timer.Get();
	sample.m_left_distance_m = left_distance_m;
	sample.m_right_distance_m = right_distance_m;
	sample.m_gyro_heading_deg = gyro_heading_deg;
	sample.m_left_output = left_output;
	sample.m_right_output = right_output;
	sample.m_mechanism_action = mechanism_action;
	m_sample_list.push_back(sample);
}

void PathController::WriteSampleHeading(std::ofstream& results_file)
{
	// Do nothing. Derived classes can override this to write a heading for the sample data,
	// which will probably be a description of the path and what parameters were used.
}


