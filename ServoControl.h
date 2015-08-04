/**
 * Servo Control from FPGA with Hardware Controlled Speed
 *
 */
#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h>
#include <sys/mman.h>
#include <time.h>

#define BASE_ADDRESS 0x400D0000

//Servo motor offsets
#define Base_OFFSET 0x100
#define Bicep_OFFSET 0x104
#define Elbow_OFFSET 0x108
#define Wrist_OFFSET 0x10C
#define Gripper_OFFSET 0x110

//Servo Motor Numbers
#define BASE_SERVO 1
#define BICEP_SERVO 2
#define ELBOW_SERVO 3
#define WRIST_SERVO 4
#define GRIPPER_SERVO 5

#define REG_WRITE(addr, off, val) (*(volatile int*)(addr+off)=(val))

/**
 * data structure for servo instance
 */
typedef struct {
	unsigned char *test_base; /// base address of mapped virtual space
	int fd;                   /// file desrcriptor for memory map
	int map_len;              /// size of mapping window

} tServo;


/**
 * global variable for all servos
 */
tServo gServos;

/**
 * Initialize servos
 * @return 0 upon success, 1 otherwise
 */
int servo_init(int base, int bicep, int elbow, int wrist, int gripper) {

	//Open the file regarding memory mapped IO to write values for the FPGA
	gServos.fd = open( "/dev/mem", O_RDWR);

	unsigned long int PhysicalAddress = BASE_ADDRESS;
	gServos.map_len= 0xFF;  //size of mapping window

	// map physical memory startin at BASE_ADDRESS into own virtual memory
	gServos.test_base = (unsigned char*)mmap(NULL, gServos.map_len, PROT_READ | PROT_WRITE, MAP_SHARED, gServos.fd, (off_t)PhysicalAddress);

	// check if Initialization work properly
	if(gServos.test_base == MAP_FAILED)	{
		perror("Mapping memory for absolute memory access failed -- Test Try\n");
		return 1;
	}

	//Initialize all servo motors to starting positions
	REG_WRITE(gServos.test_base, Base_OFFSET,    (10 << 8) + base);
	REG_WRITE(gServos.test_base, Bicep_OFFSET,   (10 << 8) + bicep);
	REG_WRITE(gServos.test_base, Elbow_OFFSET,   (10 << 8) + elbow);
	REG_WRITE(gServos.test_base, Wrist_OFFSET,   (10 << 8) + wrist);
 	REG_WRITE(gServos.test_base, Gripper_OFFSET, (10 << 8) + gripper);

 	return 0;
}


/**
 * This function takes the servo number and the position, and writes the values in
 * appropriate address for the FPGA
 * @param test_base			base pointer for servos
 * @param servo_number			servo number to manipulate
 * @param position			new postion in degree (0 .. 180)
 * @param speed				speed to move in degree / 20ms
 */
void servo_move(unsigned char servo_number, unsigned char position, unsigned char speed) {

	/* writeValue bits 0..7    position
	 * 	      bits 8..15   speed
	 * 	      bits 16..31  all 0
	 */
	unsigned int writeValue = (speed << 8) + position;

	switch (servo_number) {
        	case 1:  //Base
                	REG_WRITE(gServos.test_base, Base_OFFSET, writeValue);
                	break;

           	case 2:  //Bicep
                	REG_WRITE(gServos.test_base, Bicep_OFFSET, writeValue);
                	break;

          	case 3:  //Elbow
                	REG_WRITE(gServos.test_base, Elbow_OFFSET, writeValue);
                	break;

           	case 4:  //Wrist
                	REG_WRITE(gServos.test_base, Wrist_OFFSET, writeValue);
                	break;

           	case 5:  //Gripper
                	REG_WRITE(gServos.test_base, Gripper_OFFSET, writeValue);
                	break;

           	default:
                	break;
	}
}

/**
 * Deinitialize Servos
 */
void servo_release(){
	// Releasing the mapping in memory
	munmap((void *)gServos.test_base, gServos.map_len);
	close(gServos.fd);
}

