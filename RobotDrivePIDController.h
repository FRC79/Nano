#ifndef ROBOTDRIVEPIDCONTROLLER_H
#define ROBOTDRIVEPIDCONTROLLER_H

#include "CANJaguar.h"
#include "Base.h"
#include "semLib.h"
#include "Controller.h"
#include "LiveWindow/LiveWindow.h"

class PIDOutput;
class PIDSource;
class Notifier;

class RobotDrivePIDController
{
public:
	RobotDrivePIDController(float Kp, float Ki, float Kd,
			CANJaguar *FR_jag, CANJaguar *FL_jag,
			CANJaguar *RR_jag, CANJaguar *RL_jag, float period = 0.05);
	 ~RobotDrivePIDController();
	 float Get();
	 void SetContinuous(bool continuous = true);
	 void SetInputRange(float minimumInput, float maximumInput);
	 void SetOutputRange(float mimimumOutput, float maximumOutput);
	 void SetPID(float p, float i, float d);
	 float GetP();
	 float GetI();
	 float GetD();
	
	 void SetSetpoint(float setpoint);
	 float GetSetpoint();

	 float GetError();
	
	 void SetTolerance(float percent);
	 bool OnTarget();
	
	 void Enable();
	 void Disable();
	 bool IsEnabled();
	
	 void Reset();

private:
	float m_P;			// factor for "proportional" control
	float m_I;			// factor for "integral" control
	float m_D;			// factor for "derivative" control
	float m_maximumOutput;	// |maximum output|
	float m_minimumOutput;	// |minimum output|
	float m_maximumInput;		// maximum input - limit setpoint to this
	float m_minimumInput;		// minimum input - limit setpoint to this
	bool m_continuous;	// do the endpoints wrap around? eg. Absolute encoder
	bool m_enabled; 			//is the pid controller enabled
	float m_prevError;	// the prior sensor input (used to compute velocity)
	double m_totalError; //the sum of the errors for use in the integral calc
	float m_tolerance;	//the percetage error that is considered on target
	float m_setpoint;
	float m_error;
	float m_result;
	float m_period;
	bool m_InvertOutputs;
	
	
	SEM_ID m_semaphore;
	
	CANJaguar *m_jagFR;
	CANJaguar *m_jagFL;
	CANJaguar *m_jagRR;
	CANJaguar *m_jagRL;
	Notifier *m_controlLoop;

	static void CallCalculate(void *controller);
	void Calculate();
	DISALLOW_COPY_AND_ASSIGN(RobotDrivePIDController);
};

#endif
