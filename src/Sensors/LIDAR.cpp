/*
 * LIDAR.cpp
 *
 *  Created on: Feb 7, 2015
 *      Author: HW Robotics
 */

#include "LIDAR.h"

LIDAR::LIDAR() {
	i2c = new I2C(I2C::Port::kOnboard, 0x62);
	runThread = false;
	buf = (unsigned char*)malloc(2*sizeof(unsigned char));
}

LIDAR::~LIDAR() {
	stop();
	free(buf);
}

void LIDAR::start(int period)
{
	if(runThread) stop();
	threadPeriod = period;
	runThread = true;
	updateThread = new std::thread(&LIDAR::threadMain, this);
}

void LIDAR::start()
{
	start(100);
}

double LIDAR::PIDGet()
{
	return getDistance();
}

int LIDAR::getDistance()
{
	mtx.lock();
	int res = (int)dist;
	mtx.unlock();
	return res;
}

void LIDAR::stop()
{
	if(updateThread != NULL)
	{
		runThread = false;
		updateThread->join();
		delete updateThread;
	}
}

void LIDAR::threadMain()
{
	std::chrono::milliseconds per( threadPeriod );
	std::chrono::microseconds poll(40);
	while(runThread)
	{
		mtx.lock();
		i2c->Write(0x00, 0x04);
		std::this_thread::sleep_for(poll);
		i2c->Read(0x8f, 2, buf);
		dist = (buf[0] << 8) | buf[1];
		SmartDashboard::PutNumber("Distance", dist);
		mtx.unlock();
		std::this_thread::sleep_for(per);
	}
}
