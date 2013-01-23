#ifndef ROBOTTURNPIDOUTPUT_H
#define ROBOTTURNPIDOUTPUT_H

#include "PIDOutput.h"
#include "RobotDrive.h"

/* This class controls the direction and turning
 * of a RobotDrive system from a PIDController
 */

class RobotTurnPIDOutput : public PIDOutput
{
public:
	RobotTurnPIDOutput(RobotDrive *drive);
	virtual ~RobotTurnPIDOutput();
	virtual void PIDWrite(float output);
	
private:
	RobotDrive *r_drive;
};

#endif
