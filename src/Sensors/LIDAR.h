/*
 * LIDAR.h
 *
 *  Created on: Feb 7, 2015
 *      Author: HW Robotics
 */

#ifndef SRC_SENSORS_LIDAR_H_
#define SRC_SENSORS_LIDAR_H_

#include "WPILib.h"
#include "NetworkTables/NetworkTable.h"
#include <thread>
#include <mutex>

class LIDAR : public PIDSource {
public:
	LIDAR();
	~LIDAR();

private:
	I2C * i2c;
	std::thread *updateThread;
	volatile bool runThread;
	void threadMain();
	volatile int threadPeriod;
	unsigned char* buf;
	volatile unsigned short dist;
	std::mutex mtx;

public:
	int getDistance();
	double PIDGet();
	void start();
	void start(int period);
	void stop();
	void update();
};

#endif /* SRC_SENSORS_LIDAR_H_ */
