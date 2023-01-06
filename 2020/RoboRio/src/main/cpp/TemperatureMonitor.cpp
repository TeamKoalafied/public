/*
 * TemperatureMonitor.cpp
 *
 *  Created on: 5 Mar. 2018
 *      Author: Nick
 */

#include "TemperatureMonitor.h"

#include <iostream>
#include <string>



TemperatureMonitor::TemperatureMonitor() {
	m_temperature = 30.0;
	m_temperature_read_in_progress = false;
	m_led_pattern = 0;
}

TemperatureMonitor::~TemperatureMonitor() {
}

void TemperatureMonitor::PeriodicUpdate() {
	frc::Timer update_timer;
	update_timer.Start();

	if (!m_temperature_read_in_progress) {
		char buffer[2];
//		buffer[0] = 'L';
//		buffer[1] = '0' + m_led_pattern;
//		m_serial_port.Write(buffer, 2);
//		char response_buffer[100];
//		int bytes_read = m_serial_port.Read(response_buffer, 100);



		buffer[0] = 'T';
		m_serial_port.Write(buffer, 1);
		m_temperature_read_in_progress = true;

		m_timer.Reset();
		m_timer.Start();
	} else {
		if (m_serial_port.GetBytesReceived() > 0) {
			char response_buffer[100];
			int bytes_read = m_serial_port.Read(response_buffer, 100);
//			std::cout << "Time " << m_timer.Get() << "\n";

//			std::cout << "Bytes read " << bytes_read << "\n";
			response_buffer[bytes_read] = 0;
//			std::cout << "Response: " << response_buffer << "\n";

			m_temperature_read_in_progress = false;

			m_temperature = std::stod(response_buffer);

//			std::cout << "Temp" << m_temperature << "\n";
		}
	}

	if (update_timer.Get() > 0.001) {
		std::cout << "TemperatureMonitor::PeriodicUpdate() took" << (update_timer.Get()*1000) << "ms\n";
	}
}

double TemperatureMonitor::GetTemperature() {
	return m_temperature;
}

void TemperatureMonitor::SetLedPattern(int pattern) {
	m_led_pattern = pattern;
}
