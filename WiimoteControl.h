/**
 * This program reads values from event2 (button events) and prints the code and the value.
 *
 */

#include <stdio.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#include <errno.h>


/************ constants ****************/

/** Event 2 is of 32 chars in size */
#define WIIMOTE_EVT2_PKT_SIZE 32

/** device file name for event 2 */
#define WIIMOTE_EVT2_FNAME "/dev/input/event2"

/** the code is placed in byte 10 */
#define WIIMOTE_EVT2_CODE 10
/** value is placed in byte 12 */
#define WIIMOTE_EVT2_VALUE 12


/** Event 2 is of 32 chars in size */
#define WIIMOTE_EVT0_PKT_SIZE 16

/** device file name for event 2 */
#define WIIMOTE_EVT0_FNAME "/dev/input/event0"

/** the code is placed in byte 10 */
#define WIIMOTE_EVT0_CODE 10

/** code for X acceleration */
#define WIIMOTE_EVT0_ACCEL_X 3
/** code for Y acceleration */
#define WIIMOTE_EVT0_ACCEL_Y 4
/** code for Z acceleration */
#define WIIMOTE_EVT0_ACCEL_Z 5

/** high portion of value is placed in byte 13	*/
#define WIIMOTE_EVT0_VALUE_H 13

/** low portion of value is placed in byte 12	*/
#define WIIMOTE_EVT0_VALUE_L 12

/** The button code hex values */
#define WIIMOTE_BTN_UP    0x67
#define WIIMOTE_BTN_DOWN  0x6C
#define WIIMOTE_BTN_LEFT  0x69
#define WIIMOTE_BTN_RIGHT 0x6A
#define WIIMOTE_BTN_A     0x30
#define WIIMOTE_BTN_B     0x31
#define WIIMOTE_BTN_ONE   0x01
#define WIIMOTE_BTN_TWO   0x02
#define WIIMOTE_BTN_PLUS  0x97
#define WIIMOTE_BTN_MINUS 0x9C
#define WIIMOTE_BTN_HOME  0x3C


/************ types ****************/

/**
 * WiiMote Button Event
 */
typedef struct {
	unsigned char code;  /// event 2 code
	unsigned char value; /// event 2 value

} tWiiMoteButton;

/**
 * WiiMote accelerator Event
 */
typedef struct {
	unsigned char code;  /// event 0 code
	signed short value; /// event  0 value
} tWiiMoteAccel;


/**
 * structure for WiiMote object
 */
typedef struct {
	int fileEvt2; // file desriptor for event 2
	int fileEvt0; // file descriptor for event 0
} tWiiMote;


/** instantiate one WiiMote object. It is assumed to be a singleton
 * for this application (i.e. exactly one instance)
 */
tWiiMote gWiiMote;


/************ functions ****************/

/**
 * Initialize WiiMote accelerometer and button read
 * @return 0 on success, != 0 otherwise.
 */
int wiimote_init(void){
	//Opens the event2 file in read only mode
	// also set O_NONBLOCK which makes read calls non blocking, i.e. if less data is available
	// than asked for, the read call returns and errno is set to EWOULDBLOCK
	gWiiMote.fileEvt2 = open(WIIMOTE_EVT2_FNAME , O_RDONLY | O_NONBLOCK);

	// Check if file open correctly
	if (gWiiMote.fileEvt2 == -1) {
		printf("Could not open event file '%s'\n", WIIMOTE_EVT2_FNAME);
		return -1;
	}


	// open file for accelerometer -- blocking
	gWiiMote.fileEvt0 = open(WIIMOTE_EVT0_FNAME , O_RDONLY); //Opens the event2 file in read only mode

	// failed to open file?
	if (gWiiMote.fileEvt0  == -1) {
		// error handling, print error and exit with code
		printf("Could not open event file '%s'\n", WIIMOTE_EVT0_FNAME);
		return -1;
	}


	return 0;
}

/**
 * get event codes for wii buttons
 *
 * @return button even, if no button code detected return (0,0)
 */
tWiiMoteButton wiimote_buttonGet(){
	unsigned char buf[WIIMOTE_EVT2_PKT_SIZE]; // allocate data for packet buffer
	tWiiMoteButton button;

	// start out with nothing received
	button.code = 0;
	button.value = 0;

	// blocking read of  32 bytes from  file and put them into buf
	// Without O_NONBLOCK read blocks (does not return) until sufficient bytes are available
	// with O_NONBLOCK read immediately returns even if fewer bytes than asked for are availble, 
	//but it sets to errno then.

	// only continue if we got all bytes asked for.
	if (WIIMOTE_EVT2_PKT_SIZE == read(gWiiMote.fileEvt2, buf, WIIMOTE_EVT2_PKT_SIZE)) {

		// extract code from packet
		button.code = buf[WIIMOTE_EVT2_CODE];

		// extract value from packet
		button.value = buf[WIIMOTE_EVT2_VALUE];

	} else {
		// got fewer bytes ...
		// if error is different than it would block then report
		if (errno != EWOULDBLOCK) {
			printf("Unknown error %d\n", errno);
		}
	}

	// return received button code (or none if no code received)
	return button;

}

/**
 * get acceleration events from wiimote
 * @return acceleration event
 */
tWiiMoteAccel wiimote_accelGet() {
	unsigned char buf[WIIMOTE_EVT0_PKT_SIZE]; //each packet of data is 16 bytes
	unsigned char evt0ValueL; /// event 0 value Low
	unsigned char evt0ValueH; /// event 0 value High
	tWiiMoteAccel accel;

	accel.code = 0; // start out with nothing received

	// read 16 bytes from the file and put it in the buffer
	read(gWiiMote.fileEvt0, buf, WIIMOTE_EVT0_PKT_SIZE);

	accel.code = buf[WIIMOTE_EVT0_CODE];       // extract code byte

	// if we got an accel change
	if (accel.code != 0) {
		evt0ValueH = buf[WIIMOTE_EVT0_VALUE_H];  // extract high byte of accel
		evt0ValueL = buf[WIIMOTE_EVT0_VALUE_L];  // extract low byte of accel
		accel.value = evt0ValueH << 8 | evt0ValueL;
	}

	// return read accel up (may be with code 0).
	return accel;
}


/**
 * close the wiimote connection
 */
void wiimote_close() {
	close(gWiiMote.fileEvt0); // close acceleration file
	close(gWiiMote.fileEvt2); // close button file
}



/**
 * Main function
 * @return 0 upon success, -1 on error
 */
/*
int main(int argc, char* argv[]) {
	tWiiMoteButton button;
	tWiiMoteAccel accel;
	unsigned char accelX =0;
	unsigned char accelY =0;

	if (wiimote_init() != 0) {
		printf("Failed to init WiiMote\n");
		return -1;
	}


	do {
		printf("Getting acceleration\n");
		// read acceleration (blocking)
		accel = wiimote_accelGet();
		printf("Getting button\n");
		// read button events that have accumulated since then.
		button = wiimote_buttonGet();

		// did we get an accel event?
		if (accel.code != 0) { // ignore the zeroes
			if( (accelX && accel.code == WIIMOTE_EVT0_ACCEL_X)
			 || (accelY && accel.code == WIIMOTE_EVT0_ACCEL_Y ) ) {
				//print content
				printf("code: %X, value %d\n", accel.code, accel.value);
			}
		}

		switch (button.code) {
		case 0: // nothing received
			break;
		case 0x97: // - key
			// enable / disable printing of X accel depending on if pushed / released
			accelX = button.value;
			break;
		case 0x9C: // + key
			// enable / disable printing of X accel depending on if pushed / released
			accelY = button.value;
			break;
		}


	// repeat until "Home" button is pressed (or relased)
	} while(button.code != 0x3C);

	wiimote_close();
	return 0;
}

*/
