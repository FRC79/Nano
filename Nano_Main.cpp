#include "WPILib.h"
#include "CANJaguar.h"
#include "Joystick.h"
#include "Logger.h"

class CK16_Main : public IterativeRobot
{
	// Declare variable for the robot drive system
	RobotDrive *m_robotDrive;		// robot will use PWM 1-4 for drive motors
	
	Log *m_Log;
	//Robot will use CAN bus for motor control
	CANJaguar *Front_R, *Front_L, *Rear_R, *Rear_L;
	
	// Declare a variable to use to access the driver station object
	DriverStation *m_ds;						// driver station object
	UINT32 m_priorPacketNumber;					// keep track of the most recent packet number from the DS
	UINT8 m_dsPacketsReceivedInCurrentSecond;	// keep track of the ds packets received in the current second
	
	// Declare variables for the two joysticks being used
	Joystick *operatorGamepad;			// joystick 1 (arcade stick or right tank stick)
	

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
		
		// Initialize the CAN Jaguars
		Front_R = new CANJaguar(6);
		Front_L = new CANJaguar(2);
		Rear_R = new CANJaguar(4);
		Rear_L = new CANJaguar(3);
		printf("TEAM 79 FOR THE WIN!\n");
		
		// Initialize Robot Drive System Using Jaguars
		m_robotDrive = new RobotDrive(Front_L, Rear_L, Front_R, Rear_R);
		m_Log = new Log("Hello.txt");
		// Jags on the right side will show full reverse even when going full forward PLEASE BE AWARE
		// Initialize the PID loop to control the drive wheel speed
		// *NOTE: Placeholder values for Kp, Ki, and Kd until we start loading from CSV
		
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
		
		// Set Minimum and Maximum Input Values for Drive PID Controllers (Encoder Values)
		// *NOTE: Placeholder values until we figure out real values
		//Left_Drive_PID->SetInputRange(-360.0, 360.0);
		//Right_Drive_PID->SetInputRange(-360.0, 360.0);
		m_Log->addLine("Hello World!");
		m_Log->closeLog();		
		printf("RobotInit() completed.\n");
	}
	
	void DisabledInit(void) {
		m_disabledPeriodicLoops = 0;			// Reset the loop counter for disabled mode
		// Move the cursor down a few, since we'll move it back up in periodic.
		printf("\x1b[2B");
	}

	void AutonomousInit(void) {
		m_autoPeriodicLoops = 0;				// Reset the loop counter for autonomous mode
		
		// Enable Drive PID Controllers
		//Left_Drive_PID->Enable();
//		Right_Drive_PID->Enable();
//		Left_Drive_PID->SetSetpoint(0.0);
//		Right_Drive_PID->SetSetpoint(0.0);
	}

	void TeleopInit(void) {
		m_telePeriodicLoops = 0;				// Reset the loop counter for teleop mode
		m_dsPacketsReceivedInCurrentSecond = 0;	// Reset the number of dsPackets in current second
		
//		Left_Drive_PID->Disable();
//		Right_Drive_PID->Disable();
	}

	/********************************** Periodic Routines *************************************/
	
	void DisabledPeriodic(void)  {
		static INT32 printSec = (INT32)GetClock() + 1;
		static const INT32 startSec = (INT32)GetClock();


		// increment the number of disabled periodic loops completed
		m_disabledPeriodicLoops++;
		
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

		// Commented out until we figure out the initialize details
	}

	
	void TeleopPeriodic(void) {
		// increment the number of teleop periodic loops completed
		m_telePeriodicLoops++;
		GetWatchdog().Feed();

		m_robotDrive->SetLeftRightMotorOutputs(operatorGamepad->GetRawAxis(2), operatorGamepad->GetRawAxis(5));
	}  
	
	//TeleopPeriodic(void)
			
};
START_ROBOT_CLASS(CK16_Main);
