/*
 * TemperatureMonitor.h
 *
 *  Created on: 5 Mar. 2018
 *      Author: Nick
 */

#ifndef SRC_TEMPERATUREMONITOR_H_
#define SRC_TEMPERATUREMONITOR_H_

#include <frc/Timer.h>
#include <frc/SerialPort.h>



class TemperatureMonitor {
public:
	TemperatureMonitor();
	virtual ~TemperatureMonitor();

	void PeriodicUpdate();
	double GetTemperature();
	void SetLedPattern(int pattern);
	int GetLedPattern() { return m_led_pattern; }

	frc::SerialPort m_serial_port { 115200, frc::SerialPort::Port::kMXP };
	int m_counter = 0;
	frc::Timer m_timer;
	bool m_temperature_read_in_progress;

	double m_temperature;
	int m_led_pattern;
};

#endif /* SRC_TEMPERATUREMONITOR_H_ */
