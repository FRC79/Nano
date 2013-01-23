
#include "RobotDrive.h"
#include "RobotTurnPIDOutput.h"
#include "PIDOutput.h"

RobotTurnPIDOutput::RobotTurnPIDOutput(RobotDrive *drive)
{
	r_drive = drive;
}

RobotTurnPIDOutput::~RobotTurnPIDOutput()
{
	delete r_drive;
}

void RobotTurnPIDOutput::PIDWrite(float output)
{
	// -1.0 is max left, +1.0 is max right. The right motor is
	// inverted so that whatever value is fed in makes sense.
	r_drive->SetLeftRightMotorOutputs(output, output * -1);
}
