
#include "RobotDrivePIDController.h"

#include "Notifier.h"
#include "PIDOutput.h"
#include "CANJaguar.h"
#include <math.h>
#include "Synchronized.h"
/**
 * Allocate a PID object with the given constants for P, I, D
 * @param Kp the proportional coefficient
 * @param Ki the integral coefficient
 * @param Kd the derivative coefficient
 * @param period the loop time for doing calculations. This particularly effects calculations of the
 * integral and differental terms. The default is 50ms.
 */
RobotDrivePIDController::RobotDrivePIDController(float Kp, float Ki, float Kd,
							CANJaguar *FR_jag, CANJaguar *FL_jag,
							CANJaguar *RR_jag, CANJaguar *RL_jag, float period) :
	m_semaphore (0)
{
	m_semaphore = semBCreate(SEM_Q_PRIORITY, SEM_FULL);

	m_controlLoop = new Notifier(RobotDrivePIDController::CallCalculate, this);
	
	m_P = Kp;
	m_I = Ki;
	m_D = Kd;
	m_maximumOutput = 1.0;
	m_minimumOutput = -1.0;
	
	m_maximumInput = 0;
	m_minimumInput = 0;
	
	m_continuous = false;
	m_enabled = false;
	m_setpoint = 0;

	m_prevError = 0;
	m_totalError = 0;
	m_tolerance = .05;

	m_result = 0;
	
	m_jagFR = FR_jag;
	m_jagFL = FL_jag;
	m_jagRR = RR_jag;
	m_jagRL = RL_jag;
	m_period = period;

	m_controlLoop->StartPeriodic(m_period);
}

/**
 * Free the PID object
 */
RobotDrivePIDController::~RobotDrivePIDController()
{
	semFlush(m_semaphore);
	delete m_controlLoop;
}

/**
 * Call the Calculate method as a non-static method. This avoids having to prepend
 * all local variables in that method with the class pointer. This way the "this"
 * pointer will be set up and class variables can be called more easily.
 * This method is static and called by the Notifier class.
 * @param controller the address of the PID controller object to use in the background loop
 */
void RobotDrivePIDController::CallCalculate(void *controller)
{
	RobotDrivePIDController *control = (RobotDrivePIDController*) controller;
	control->Calculate();
}

 /**
  * Read the input, calculate the output accordingly, and write to the output.
  * This should only be called by the Notifier indirectly through CallCalculate
  * and is created during initialization.
  */	
void RobotDrivePIDController::Calculate()
{
	bool enabled;

	CRITICAL_REGION(m_semaphore)
	{
		if (m_jagFR == 0) return;
		if (m_jagFL == 0) return;
		if (m_jagRR == 0) return;
		if (m_jagRL == 0) return;
		enabled = m_enabled;
	}
	END_REGION;

	if (enabled)
	{
		// Get positions of the encoders on the robot (currently the front encoders)
		float jagEncoderLeftPos = m_jagFL->GetPosition();
		float jagEncoderRightPos = m_jagFR->GetPosition();
		float resultLeft, resultRight;
		
		{
			Synchronized sync(m_semaphore);
			m_error = m_setpoint - (jagEncoderLeftPos + jagEncoderRightPos)/2;
			if (m_continuous)
			{
				if (fabs(m_error) > (m_maximumInput - m_minimumInput) / 2)
				{
					if (m_error > 0)
					{
						m_error = m_error - m_maximumInput + m_minimumInput;
					}
					else
					{
						m_error = m_error + m_maximumInput - m_minimumInput;
					}
				}
			}

			double potentialIGain = (m_totalError + m_error) * m_I;
			if (potentialIGain < m_maximumOutput)
			{
				if (potentialIGain > m_minimumOutput)
					m_totalError += m_error;
				else
					m_totalError = m_minimumOutput / m_I;
			}
			else
			{
				m_totalError = m_maximumOutput / m_I;
			}

			m_result = m_P * m_error + m_I * m_totalError + m_D * (m_error - m_prevError);
			m_prevError = m_error;

			if (m_result > m_maximumOutput) m_result = m_maximumOutput;
			else if (m_result < m_minimumOutput) m_result = m_minimumOutput;

			resultLeft = m_result / jagEncoderLeftPos;
			resultRight = m_result / jagEncoderRightPos;
		}
		
		m_jagFL->Set(resultLeft);
		m_jagRL->Set(resultLeft);
		m_jagFR->Set(resultRight);
		m_jagRR->Set(resultRight);
	}
}

/**
 * Set the PID Controller gain parameters.
 * Set the proportional, integral, and differential coefficients.
 * @param p Proportional coefficient
 * @param i Integral coefficient
 * @param d Differential coefficient
 */
void RobotDrivePIDController::SetPID(float p, float i, float d)
{
	CRITICAL_REGION(m_semaphore)
	{
		m_P = p;
		m_I = i;
		m_D = d;
	}
	END_REGION;
}

/**
 * Get the Proportional coefficient
 * @return proportional coefficient
 */
float RobotDrivePIDController::GetP()
{
	CRITICAL_REGION(m_semaphore)
	{
		return m_P;
	}
	END_REGION;
}

/**
 * Get the Integral coefficient
 * @return integral coefficient
 */
float RobotDrivePIDController::GetI()
{
	CRITICAL_REGION(m_semaphore)
	{
		return m_I;
	}
	END_REGION;
}

/**
 * Get the Differential coefficient
 * @return differential coefficient
 */
float RobotDrivePIDController::GetD()
{
	CRITICAL_REGION(m_semaphore)
	{
		return m_D;
	}
	END_REGION;
}

/**
 * Return the current PID result
 * This is always centered on zero and constrained the the max and min outs
 * @return the latest calculated output
 */
float RobotDrivePIDController::Get()
{
	float result;
	CRITICAL_REGION(m_semaphore)
	{
		result = m_result;
	}
	END_REGION;
	return result;
}

/**
 *  Set the PID controller to consider the input to be continuous,
 *  Rather then using the max and min in as constraints, it considers them to
 *  be the same point and automatically calculates the shortest route to
 *  the setpoint.
 * @param continuous Set to true turns on continuous, false turns off continuous
 */
void RobotDrivePIDController::SetContinuous(bool continuous)
{
	CRITICAL_REGION(m_semaphore)
	{
		m_continuous = continuous;
	}
	END_REGION;

}

/**
 * Sets the maximum and minimum values expected from the input.
 * 
 * @param minimumInput the minimum value expected from the input
 * @param maximumInput the maximum value expected from the output
 */
void RobotDrivePIDController::SetInputRange(float minimumInput, float maximumInput)
{
	CRITICAL_REGION(m_semaphore)
	{
		m_minimumInput = minimumInput;
		m_maximumInput = maximumInput;	
	}
	END_REGION;

	SetSetpoint(m_setpoint);
}

/**
 * Sets the minimum and maximum values to write.
 * 
 * @param minimumOutput the minimum value to write to the output
 * @param maximumOutput the maximum value to write to the output
 */
void RobotDrivePIDController::SetOutputRange(float minimumOutput, float maximumOutput)
{
	CRITICAL_REGION(m_semaphore)
	{
		m_minimumOutput = minimumOutput;
		m_maximumOutput = maximumOutput;
	}
	END_REGION;
}

/**
 * Set the setpoint for the PIDController
 * @param setpoint the desired setpoint
 */
void RobotDrivePIDController::SetSetpoint(float setpoint)
{
	CRITICAL_REGION(m_semaphore)
	{
		if (m_maximumInput > m_minimumInput)
		{
			if (setpoint > m_maximumInput)
				m_setpoint = m_maximumInput;
			else if (setpoint < m_minimumInput)
				m_setpoint = m_minimumInput;
			else
				m_setpoint = setpoint;
		}
		else
		{
			m_setpoint = setpoint;
		}
	}
	END_REGION;	
}

/**
 * Returns the current setpoint of the PIDController
 * @return the current setpoint
 */
float RobotDrivePIDController::GetSetpoint()
{
	float setpoint;
	CRITICAL_REGION(m_semaphore)
	{
		setpoint = m_setpoint;
	}
	END_REGION;
	return setpoint;
}

/**
 * Retruns the current difference of the input from the setpoint
 * @return the current error
 */
float RobotDrivePIDController::GetError()
{
	float error;
	CRITICAL_REGION(m_semaphore)
	{
		error = m_error;
	}
	END_REGION;
	return error;
}

/*
 * Set the percentage error which is considered tolerable for use with
 * OnTarget.
 * @param percentage error which is tolerable
 */
void RobotDrivePIDController::SetTolerance(float percent)
{
	CRITICAL_REGION(m_semaphore)
	{
		m_tolerance = percent;
	}
	END_REGION;
}

/*
 * Return true if the error is within the percentage of the total input range,
 * determined by SetTolerance. This asssumes that the maximum and minimum input
 * were set using SetInput.
 */
bool RobotDrivePIDController::OnTarget()
{
	bool temp;
	CRITICAL_REGION(m_semaphore)
	{
		temp = fabs(m_error) < (m_tolerance / 100 * 
			(m_maximumInput - m_minimumInput));
	}
	END_REGION;
	return temp;
}

/**
 * Begin running the PIDController
 */
void RobotDrivePIDController::Enable()
{
	CRITICAL_REGION(m_semaphore)
	{			
		m_enabled = true;
	}
	END_REGION;	
}
/**
 * Stop running the PIDController, this sets the output to zero before stopping.
 */
void RobotDrivePIDController::Disable()
{
	CRITICAL_REGION(m_semaphore)
	{
		m_jagFR->Set(0.0f);
		m_jagFL->Set(0.0f);
		m_jagRR->Set(0.0f);
		m_jagRL->Set(0.0f);
		m_enabled = false;
	}
	END_REGION;
}

/**
 * Return true if PIDController is enabled.
 */
bool RobotDrivePIDController::IsEnabled()
{
	bool enabled;
	CRITICAL_REGION(m_semaphore)
	{
		enabled = m_enabled;
	}
	END_REGION;
	return enabled;
}

/**
 * Reset the previous error,, the integral term, and disable the controller.
 */
void RobotDrivePIDController::Reset()
{
	Disable();

	CRITICAL_REGION(m_semaphore)
	{
		m_prevError = 0;
		m_totalError = 0;
		m_result = 0;
	}
	END_REGION;
}
