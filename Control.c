/**
 * Interpret Controls from Wii remote to commands understood by the Servos
 *
*/
#include "WiimoteControl.h"
#include "ServoControl.h"
#include <math.h>
#include <time.h>

// Variables specifically for Gripper Motor
#define GRIPPER_SPEED 0.5
#define GRIPPER_ACCEL 20
#define GRIPPER_MIN_ANGLE 100
#define GRIPPER_MAX_ANGLE 200
#define GRIPPER_MAX_SPEED 100

// Variables for all other Base Motor
#define BASE_SPEED 1
#define BASE_ACCEL 20
#define BASE_MIN_ANGLE 60
#define BASE_MAX_ANGLE 240

// Speed of Servos Bicep, Elbow , and Wrist
#define MOVEMENT_SCALE 1.2
#define BICEP_ACCEL 20
#define ELBOW_ACCEL 20
#define WRIST_ACCEL 20

//Structure for storing each individual sero motor position
typedef struct {
	unsigned int base;
	unsigned int bicep;
	unsigned int elbow;
	unsigned int wrist;
	double gripper;
} Robot;

//Print out the current position of each servo motor
void debug_robot_positions(Robot *r) {
	printf("Robot DEBUG:\n--------------------------\n");
	printf("Robot base:    %d\n", r->base);
	printf("Robot bicep:   %d\n", r->bicep);
	printf("Robot elbow:   %d\n", r->elbow);
	printf("Robot wrist:   %d\n", r->wrist);
	printf("Robot gripper: %f\n\n", r->gripper);
}

// Check that the Base Servo Position are within the boundary limits of motor
int checkBoundaryBase(int  pos, int dir) {
	switch(dir) {
		case 0:
			if ((pos - BASE_SPEED ) >= BASE_MIN_ANGLE && (pos - BASE_SPEED) <= BASE_MAX_ANGLE) {return 0;}
			else {return -1;}
		case 1:
			if ((pos + BASE_SPEED ) >= BASE_MIN_ANGLE && (pos + BASE_SPEED) <= BASE_MAX_ANGLE) {return 0;}
			else {return-1;}
		default:
			return -1;

	}
}

// Check that the Gripper  Servo Position are within the boundary limits of motor
int checkBoundaryGripper(double pos, int dir) {
	switch(dir) {
		case 0:
			if ((pos - GRIPPER_SPEED ) >= GRIPPER_MIN_ANGLE && (pos - GRIPPER_SPEED) <= GRIPPER_MAX_ANGLE) {return 0;}
			else {return -1;}
		case 1:
			if ((pos + GRIPPER_SPEED ) >= GRIPPER_MIN_ANGLE && (pos + GRIPPER_SPEED) <= GRIPPER_MAX_ANGLE) {return 0;}
			else {return -1;}
		default:
			return -1;
	}
}

/****************** Functions ************************/
/**
 * Move base sero motor by set increment
 * @param dir = desired direction to move
 * @param int pos = current position of the base motor
*/
int moveBase(int dir, int pos) {
	switch (dir) {
		case 0: 	// CCW motion
			servo_move(BASE_SERVO, pos - BASE_SPEED, BASE_ACCEL);
			return pos - BASE_SPEED;
		case 1:	// CW motion
			servo_move(BASE_SERVO, pos + BASE_SPEED, BASE_ACCEL);
			return pos + BASE_SPEED;
	 	default:	// Error with argument, do nothing
			return pos;
	}
}

/**
 * Move base sero motor by set increment
 * @param dir = desired direction to move
 * @param int pos = current position of the gripper motor
*/
double moveGripper(int dir, double pos) {
	switch (dir) {
		case 0: 	//close gripper
			servo_move(GRIPPER_SERVO, pos - GRIPPER_SPEED, GRIPPER_ACCEL);
			return pos - GRIPPER_SPEED;
	 	case 1: 	//open gripper
			servo_move(GRIPPER_SERVO, pos + GRIPPER_SPEED, GRIPPER_ACCEL);
			return pos + GRIPPER_SPEED;
	 	default:	//Error with argument, do nothing
			return pos;
	}
}

/**
 * Make the arm pick up an obecjt at a predetermined location and throw it
 *
*/
void protocolThrow() {
	// Moves to the object with a open gripper
	servo_move(BASE_SERVO, 145, BASE_ACCEL);
	servo_move(BICEP_SERVO, 75, BICEP_ACCEL);
	servo_move(ELBOW_SERVO, 215, ELBOW_ACCEL);
	servo_move(WRIST_SERVO, 130, WRIST_ACCEL);
	servo_move(GRIPPER_SERVO, 155, GRIPPER_ACCEL);


	printf("Preparing to move in 2 Seconds...");
	sleep(2);
	printf("Ready!\n");

	printf("Closing gripper...");
	servo_move(GRIPPER_SERVO, GRIPPER_MIN_ANGLE, GRIPPER_ACCEL);
	sleep(3);
	printf("Gripper is closed!");

	//The combination of these commands create a throwing motion
	servo_move(BICEP_SERVO, 160, 100);
	servo_move(ELBOW_SERVO, 240, 40);
	servo_move(WRIST_SERVO, 160, 40);
	usleep(4500);
	servo_move(5, GRIPPER_MAX_ANGLE, 20);
	sleep(2); //This allows the arm to complete the full swing
}

int main(void) {
	Robot robot;
	// Set initial positions for servo motors
	robot.base = 105;
	robot.bicep = 80;
	robot.elbow = 200;
	robot.wrist = 150;
	robot.gripper = 120.0;

	tWiiMoteButton button;
	tWiiMoteAccel accel;

	// Keep track of buttons being pressed
	unsigned char aBtnPressed     = 0;
	unsigned char bBtnPressed     = 0;
	unsigned char leftBtnPressed  = 0;
	unsigned char rightBtnPressed = 0;
	unsigned char upBtnPressed    = 0;
	unsigned char downBtnPressed  = 0;
	unsigned char twoBtnPressed   = 0;


	// Wiimote initialization
	printf("Initializing Wiimote connection...");
	//Check if initialized properly
	if (wiimote_init() != 0) {
		printf("Failed to init WiiMote\n");
		return -1;
	} else {
		printf("connected!\n");
	}

	// Servo initialization
	printf("Initializing Servos...");
	if (servo_init(robot.base, robot.bicep, robot.elbow, robot.wrist, robot.gripper) != 0) {
		printf("Failed to initialize the servos.\n");
		return -2; // exit if init fails
	}
	printf("done!\n");


	do {
		// read acceleration (blocking)
		accel = wiimote_accelGet();
		// read button events that have accumulated since then.
		button = wiimote_buttonGet();

		switch (button.code) {
			case WIIMOTE_BTN_A:
				printf("A Pressed!\n");
				aBtnPressed = button.value;
				break;
			case WIIMOTE_BTN_B:
				printf("B pressed!\n");
				bBtnPressed = button.value;
				break;
			case WIIMOTE_BTN_LEFT:
				printf("Left pressed!\n");
				leftBtnPressed = button.value;
				break;
			case WIIMOTE_BTN_RIGHT:
				printf("Right pressed!\n");
				rightBtnPressed = button.value;
				break;
			case WIIMOTE_BTN_UP:
				printf("Up pressed!\n");
				upBtnPressed = button.value;
				break;
			case WIIMOTE_BTN_DOWN:
				printf("Down pressed!\n");
				downBtnPressed = button.value;
				break;
			case WIIMOTE_BTN_TWO:
				printf("Two pressed!\n");
				twoBtnPressed = button.value;
				break;
			case WIIMOTE_BTN_ONE:
				if (button.value == 1) {
					debug_robot_positions(&robot);
				}
				break;
			default:
				// We don't care about other buttons
				break;
		}

		// Check if any directional buttons are pressed and respond accordingly
		if (upBtnPressed && checkBoundaryBase(robot.base, 0)==0) { //Rotate Base CCW
			robot.base = moveBase(0, robot.base);

		} else if (downBtnPressed && checkBoundaryBase(robot.base, 1) == 0) { //Rotate Base CW
			robot.base = moveBase(1, robot.base);

		} else if (rightBtnPressed && checkBoundaryGripper(robot.gripper, 0) == 0) { // Close Gripper
			robot.gripper = moveGripper(0, robot.gripper);

		} else if (leftBtnPressed && checkBoundaryGripper(robot.gripper, 1) == 0) { // Open gripper
			robot.gripper = moveGripper(1, robot.gripper);

		} else if (twoBtnPressed) { 	// Robot Arm initiates throw protocol
			protocolThrow();
			//servo_move(GRIPPER_SERVO, GRIPPER_MIN_ANGLE, GRIPPER_MAX_SPEED);
			//printf("Quick release of gripper!\n");

		}

		//Check if any buttons are pressed and switch IMU to control the specificed servo motor
		if (accel.code == WIIMOTE_EVT0_ACCEL_X) {
			int accel_val = (int) fmax(-90.0, fmin(90.0, (accel.value * -1) * MOVEMENT_SCALE)) + 150;

			if (aBtnPressed) { // Move wrist

				servo_move(WRIST_SERVO, accel_val, WRIST_ACCEL);
				robot.wrist = accel_val;

			} else if (bBtnPressed) { // Move elbow

				servo_move(ELBOW_SERVO, accel_val, ELBOW_ACCEL);
				robot.elbow = accel_val;

			} else { // Move Bicep

				servo_move(BICEP_SERVO, accel_val, BICEP_ACCEL);
				robot.bicep = accel_val;

			}
		}


	// repeat until "Home" button is pressed (or relased)
	} while(button.code != WIIMOTE_BTN_HOME);

	printf("Closing connections...\n");
	wiimote_close();
	printf("Wiimote closed!\n");
	servo_release();
	printf("Servos released!\n");

	printf("Exiting.\n");
	return 0;
}
