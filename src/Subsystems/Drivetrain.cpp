// RobotBuilder Version: 1.5
//
// This file was generated by RobotBuilder. It contains sections of
// code that are automatically generated and assigned by robotbuilder.
// These sections will be updated in the future when you export to
// C++ from RobotBuilder. Do not put any code or make any change in
// the blocks indicating autogenerated code or it will be lost on an
// update. Deleting the comments indicating the section will prevent
// it from being updated in the future.




#include "Drivetrain.h"
#include "../RobotMap.h"
#include <vector>
#include <algorithm>

// BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=INCLUDES
// END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=INCLUDES

Drivetrain::Drivetrain() : Subsystem("Drivetrain") {
	// BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DECLARATIONS
	gyro = RobotMap::drivetrainGyro;

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DECLARATIONS

	frontWheel = new WheelSubsystem(0);
	leftWheel = new WheelSubsystem(3);
	rightWheel = new WheelSubsystem(1);
	backWheel = new WheelSubsystem(2);
}
    
void Drivetrain::InitDefaultCommand() {
	// Set the default command for a subsystem here.
	//SetDefaultCommand(new MySpecialCommand());
	// BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DEFAULT_COMMAND

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DEFAULT_COMMAND
}


// Put methods for controlling this subsystem
// here. Call the bse from Commands.

//X is required x movement speed
//Y is required forward movement speed
//Turn is -1,1 left / right rotation
static bool abs_compare(double a, double b)
{
    return (std::abs(a) < std::abs(b));
}

void Drivetrain::DriveWithInputs(double x, double y, double turn)
{
	y*=-1;

	//Gyro input
#ifdef USE_GYRO
	if(usegyro)
	{
		double gyroInput = gyro->GetAngle();
		//Convert X and Y into an angle
		double angle = atan2(x,y)-(gyroInput%360.0);
		double mag = std::max(std::abs(x), std::abs(y));
		x = cos(angle)*mag;
		y = sin(angle)*mag;
	}
#endif

	//xf, xb, yl, yr
	std::vector<double> speeds{x+turn,-x+turn,y+turn,-y+turn};
	std::vector<double>::iterator result;
	result = std::max_element(speeds.begin(), speeds.end(), abs_compare);
	double maxs = speeds[std::distance(speeds.begin(), result)];
	if(maxs < 1) maxs = 1;

	turn *= std::abs(turn);

	int i = 0;
	for(double ix : speeds){
		speeds[i] = (ix/maxs);
		speeds[i] *= std::abs(speeds[i]);
		i++;
	}

	frontWheel->SetRawMotorSpeed(speeds[0]);
	backWheel->SetRawMotorSpeed(speeds[1]);
	leftWheel->SetRawMotorSpeed(speeds[2]);
	rightWheel->SetRawMotorSpeed(speeds[3]);
}

void Drivetrain::Stop()
{
	frontWheel->SetRawMotorSpeed(0);
	backWheel->SetRawMotorSpeed(0);
	leftWheel->SetRawMotorSpeed(0);
	rightWheel->SetRawMotorSpeed(0);
}
