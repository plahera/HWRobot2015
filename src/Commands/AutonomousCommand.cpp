// RobotBuilder Version: 1.5
//
// This file was generated by RobotBuilder. It contains sections of
// code that are automatically generated and assigned by robotbuilder.
// These sections will be updated in the future when you export to
// C++ from RobotBuilder. Do not put any code or make any change in
// the blocks indicating autogenerated code or it will be lost on an
// update. Deleting the comments indicating the section will prevent
// it from being updated in the future.


#include "AutonomousCommand.h"

AutonomousCommand::AutonomousCommand() {
	// Use requires() here to declare subsystem dependencies
	// eg. requires(chassis);
	// BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=REQUIRES
	Requires(Robot::drivetrain);

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=REQUIRES
}
using namespace std::chrono;

// Called just before this Command runs the first time
void AutonomousCommand::Initialize() {
	tstart = high_resolution_clock::now();
	Robot::boxGrabber->SetClosed(false);
}

// Called repeatedly when this Command is scheduled to run
void AutonomousCommand::Execute() {
	Robot::drivetrain->DriveWithInputs(0, 0.5, 0);
}

// Make this return true when this Command no longer needs to run execute()
bool AutonomousCommand::IsFinished() {
	high_resolution_clock::time_point t2 = high_resolution_clock::now();
	duration<double> time_span = duration_cast<duration<double>>(t2 - tstart);
	return time_span.count() > 2;
}

// Called once after isFinished returns true
void AutonomousCommand::End() {
	Robot::drivetrain->DriveWithInputs(0, 0, 0);
}

// Called when another command which requires one or more of the same
// subsystems is scheduled to run
void AutonomousCommand::Interrupted() {
	Robot::drivetrain->DriveWithInputs(0, 0, 0);
}
