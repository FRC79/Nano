#include "WPILib.h"
#include "CANJaguar.h"
#include "Joystick.h"
#include "Gyro.h"
#include "RobotTurnPIDOutput.h"
#include "PIDController.h"
#include "RobotConfiguration.h"
#include "SmartDashboard/SmartDashboard.h"
#include <string>
#include <math.h>
#include "CSVReader.h"
#include "RobotDrivePIDController.h"

class CK16_Main : public IterativeRobot
{
    // Declare CSV readers
    CSVReader *PWM_CSV, *AnalogInputs_CSV, *DigitalIO_CSV, *CAN_IDS_CSV;
    
	// SmartDashboard Keys
	std::string FOUND_KEY, AZIMUTH_KEY, RANGE_KEY, EXP_KEY;
	
	// Declare variable for the robot drive system
	RobotDrive *m_robotDrive;		// robot will use PWM 1-4 for drive motors
	
	//Robot will use CAN bus for motor control
	CANJaguar *Front_R, *Front_L, *Rear_R, *Rear_L;
	
	// Declare local variable that will hold the exponent for mapping joystick to jaguars
	double exp;
	
	// Declare left and right encoder drive PIDControllers
	RobotDrivePIDController *Drive_PID_Controller;
	
	// Declare Gyro that will be used to determine left and right robot rotation
//	Gyro *Yaw_Gyro;
	
	// Declare RobotTurnPIDOutput that will control the robot turning aspect of the goal alignment
//	RobotTurnPIDOutput *Robot_Turn;
	
	// Declare PID Controller that will handle aligning the robot with the goal
//	PIDController *Goal_Align_PID;
	
	// State boolean that represents if robot is driving with joystick input or using auto align
	bool autoPilot;
	
	// Declare a variable to use to access the driver station object
	DriverStation *m_ds;						// driver station object
	UINT32 m_priorPacketNumber;					// keep track of the most recent packet number from the DS
	UINT8 m_dsPacketsReceivedInCurrentSecond;	// keep track of the ds packets received in the current second
	
	// Declare variables for the two joysticks being used on port 1
	Joystick *operatorGamepad;			// joystick 1 (arcade stick or right tank stick)
	
	
	bool buttonWasDown;

	// Local variables to count the number of periodic loops performed
	UINT32 m_autoPeriodicLoops;
	UINT32 m_disabledPeriodicLoops;
	UINT32 m_telePeriodicLoops;
    
		
public:
/**
 * Constructor for this "CK16_Main" Class.
 * 
 * The constructor creates all of the objects used for the different inputs and outputs of
 * the robot.  Essentially, the constructor defines the input/output mapping for the robot,
 * providing named objects for each of the robot interfaces. 
 */
	CK16_Main(void)	{
		printf("CK16_Main Constructor Started\n");
        
        // Configuration files
        printf("Loading the configuration files.\n");
        AnalogInputs_CSV = new CSVReader("AnalogInputs.csv");
        CAN_IDS_CSV = new CSVReader("CAN_IDs.csv");
        DigitalIO_CSV = new CSVReader("DigitalIO.csv");
        PWM_CSV = new CSVReader("PWM.csv");
                                 
		// Initialize SmartDashboard Keys
		FOUND_KEY = "found";
		AZIMUTH_KEY = "azimuth";
		RANGE_KEY = "range";
		EXP_KEY = "exp";
		
		// Initialize the CAN Jaguars
		Front_R = new CANJaguar((UINT8)CAN_IDS_CSV->GetValue("FR_CAN_ID"), CANJaguar::kPosition);
		Front_L = new CANJaguar((UINT8)CAN_IDS_CSV->GetValue("FL_CAN_ID"), CANJaguar::kPosition);
		Rear_R = new CANJaguar((UINT8)CAN_IDS_CSV->GetValue("RR_CAN_ID"), CANJaguar::kPosition);
		Rear_L = new CANJaguar((UINT8)CAN_IDS_CSV->GetValue("RL_CAN_ID"), CANJaguar::kPosition);
		printf("TEAM 79 FOR THE WIN!\n");
		
		// Initialize Robot Drive System Using Jaguars
		m_robotDrive = new RobotDrive(Front_L, Rear_L, Front_R, Rear_R);

		// Jags on the right side will show full reverse even when going full forward PLEASE BE AWARE
		
		// Initialize left and right encoder drive PIDControllers
//		Drive_PID_Controller = new RobotDrivePIDController(1.0, 1.0, 1.0, Front_R, Front_L, Rear_R, Rear_L);	
		
		// Initialize Gyro
//		Yaw_Gyro = new Gyro(AnalogInputs_CSV->GetValue("YAW_GYRO_ID"));
		
		// Initialize the RobotTurnPIDOutput
//		Robot_Turn = new RobotTurnPIDOutput(m_robotDrive);
		
		// Initialize Goal Alignment PID Controller
//		Goal_Align_PID = new PIDController(1.0, 0.0, 1.0, Yaw_Gyro, Robot_Turn);
		
		// Acquire the Driver Station object
		m_ds = DriverStation::GetInstance();
		m_priorPacketNumber = 0;
		m_dsPacketsReceivedInCurrentSecond = 0;

		// Define joysticks being used at USB port #1 on the Drivers Station
		operatorGamepad = new Joystick(1);

		// Initialize counters to record the number of loops completed in autonomous and teleop modes
		m_autoPeriodicLoops = 0;
		m_disabledPeriodicLoops = 0;
		m_telePeriodicLoops = 0;

		printf("CK16_Main Constructor Completed\n");
	}
	
	
	/********************************** Init Routines *************************************/

	void RobotInit(void) {
		// Actions which would be performed once (and only once) upon initialization of the
		// robot would be put here.
		
		// Test to see if Dashboard is connected---------------------------------------------------------
//		printf("HUE MIN: %d\n", SmartDashboard::GetNumber("HUE MIN"));
		
		// Initialize exponent value from SmartDashboard
		exp = 3.0;
		
		buttonWasDown = false;
		// Initialize settings for encoder drive PIDControllers
//		Drive_PID_Controller->SetOutputRange(-0.2, 0.2);
//		Drive_PID_Controller->SetTolerance(0.1);
		
		// Initialize settings for RobotTurn
//		Goal_Align_PID->SetOutputRange(-0.2, 0.2);
//		Goal_Align_PID->SetTolerance(0.1);
        
		// Set encoders
        Front_R->ConfigEncoderCodesPerRev(360);
        Front_L->ConfigEncoderCodesPerRev(360);
		
        // Set each drive motor to have an encoder to be its friend
        Front_R->SetPositionReference(CANJaguar::kPosRef_QuadEncoder);
        Front_L->SetPositionReference(CANJaguar::kPosRef_QuadEncoder);
        
		
		printf("RobotInit() completed.\n");
	}
	
	void DisabledInit(void) {
		m_disabledPeriodicLoops = 0;			// Reset the loop counter for disabled mode
		// Move the cursor down a few, since we'll move it back up in periodic.
		printf("\x1b[2B");
	}

	void AutonomousInit(void) {
		m_autoPeriodicLoops = 0;				// Reset the loop counter for autonomous mode
		
		// Enable left and right encoder PID
//		Drive_PID_Controller->Disable(); // stops previous enables
//		Drive_PID_Controller->Enable();
//		Drive_PID_Controller->SetSetpoint(0.0);
		
		// Enable Goal Align PID
//		Goal_Align_PID->Disable(); // Stop previous enables
//		Goal_Align_PID->Enable();
//		Goal_Align_PID->SetSetpoint(0.0);
		
	}

	void TeleopInit(void) {
		m_telePeriodicLoops = 0;				// Reset the loop counter for teleop mode
		m_dsPacketsReceivedInCurrentSecond = 0;	// Reset the number of dsPackets in current second
		
		// Default autoPilot to off
		autoPilot = false;
		
		Front_R->EnableControl();
		Front_L->EnableControl();
		
		// Enable Goal Align PID
//		Goal_Align_PID->Disable(); // Stop previous enables
//		Goal_Align_PID->Enable();
//		Goal_Align_PID->SetSetpoint(0.0);
	}

	/********************************** Periodic Routines *************************************/
	
	void DisabledPeriodic(void)  {
		static INT32 printSec = (INT32)GetClock() + 1;
		static const INT32 startSec = (INT32)GetClock();


		// increment the number of disabled periodic loops completed
		m_disabledPeriodicLoops++;
		
//		Drive_PID_Controller->Disable();
		
		// while disabled, printout the duration of current disabled mode in seconds
		if (GetClock() > printSec) {
			// Move the cursor back to the previous line and clear it.
			printf("\x1b[1A\x1b[2K");
			printf("Disabled seconds: %d\r\n", printSec - startSec);			
			printSec++;
		}
	}

	void AutonomousPeriodic(void) {
		m_autoPeriodicLoops++;
		
		switch(m_autoPeriodicLoops)
		{
		case 1:
			// Drive 2 feet
//			Drive_PID_Controller->SetSetpoint((2 * 12.0) * 360.0 / RobotConfiguration::WHEEL_CIRCUMFERENCE);
			//DRIVE PID CONTOLLER DIVIDES BY ZERO
			break;
		default:
			break;
		}
	}

	double calculateOutputFromFunction(double input, double exponent)
	{
		if(floor(exponent) < exponent && ceil(exponent) > exponent) // If not a whole number
		{
			double mapping;
			mapping = pow(fabs(input), exponent); // Do exponent mapping to positive side
			mapping = (input >= 0) ? mapping : -mapping; // Change to negative if the input was negative
			return mapping;
		}
		else // Whole number
		{
			if((UINT8)exponent % 2 > 0) // Odd Exponent
			{
				return pow(input, exponent);
			}
			else // Even Exponent
			{
				double mapping;
				mapping = pow(fabs(input), exponent); // Do exponent mapping to positive side
				mapping = (input >= 0) ? mapping : -mapping; // Change to negative if the input was negative
				return mapping;
			}
		}
	}
	
	double calculateDriveOutputForTeleop(double input)
	{
//		if(fabs(input) < 0.05)
//		{
//			// Stop if stick is near zero
//			return 0.0;
//		}
//		else
//		{
//			double mapping;
//			mapping = 1.08 * ((3 * pow(fabs(input), 4)) + 0.2);
//			mapping = (input >= 0) ? mapping : -mapping; // Change to negative if the input was negative
//			return mapping;
//		}
		
		if(fabs(input) < 0.05)
		{
			// Stop if stick is near zero
			return 0.0;
		}
		else
		{
			double mapping;
			
			if(fabs(input) <= 0.7)
			{
				mapping = 0.33 * pow(fabs(input), 2) + 0.2;
				mapping = (input >= 0) ? mapping : -mapping; // Change to negative if the input was negative
				return mapping;
			}
			else
			{
				mapping = 3.57 * fabs(input) - 2.14;
				mapping = (input >= 0) ? mapping : -mapping; // Change to negative if the input was negative
				return mapping;
			}
		}
	}
	
	void TeleopPeriodic(void) {
		// increment the number of teleop periodic loops completed
		m_telePeriodicLoops++;
		GetWatchdog().Feed();

//		if(autoPilot == true)
//		{
			// Auto Align Disable Button
//			if(operatorGamepad->GetButton(Joystick::kTopButton) == 2)
//			{
//				Goal_Align_PID->Disable(); // Stop outputs
//				Goal_Align_PID->Enable(); // Start PIDContoller up again
//				Goal_Align_PID->SetSetpoint(0.0);
//				autoPilot = false;
//			}
//		}
//		else
//		{
			// Calculate jaguar output based on exponent we pass from SmartDashboard
			double leftOutput, rightOutput;
			leftOutput = calculateDriveOutputForTeleop(operatorGamepad->GetRawAxis(2));
			rightOutput = calculateDriveOutputForTeleop(operatorGamepad->GetRawAxis(5));
			m_robotDrive->SetLeftRightMotorOutputs(leftOutput, rightOutput);
			
			if(operatorGamepad->GetRawButton(1) && !buttonWasDown)
			{
				printf("LEFT ENCODER: %f\n", Front_L->GetPosition());
				printf("RIGHT ENCODER: %f\n", Front_R->GetPosition());
			}
			
			buttonWasDown = operatorGamepad->GetRawButton(1);
			
			// Auto Align Button
//			if(operatorGamepad->GetButton(Joystick::kTopButton) == 1)
//			{
//				// Turn Auto Align on if we see a goal and we know the azimuth
//				if(SmartDashboard::GetBoolean(FOUND_KEY) == true)
//				{
//					Goal_Align_PID->SetSetpoint(SmartDashboard::GetNumber(AZIMUTH_KEY));
//					autoPilot = true;
//				}
//			}
//		}
		
	} 
	
	//TeleopPeriodic(void)
			
};
START_ROBOT_CLASS(CK16_Main);
