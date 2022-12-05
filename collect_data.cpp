#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"

#include <dirent.h>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/stat.h>
#include <sys/time.h>
#include <sys/timeb.h>
#include <unistd.h>
#include <wiringPi.h>

#define LED_GREEN	  29
#define LED_RED		  25
#define BUTTON		  0
#define INTERRUPT_PIN 27

#define WAIT_TIME 25000

#define HARDWARE_INTERRUPT

MPU6050 mpu;

// MPU control/status vars
bool	dmpReady = false; // set true if DMP init was successful
uint8_t mpuIntStatus;	  // holds actual interrupt status byte from MPU
uint8_t devStatus;		  // return status after each device operation (0 = success, !0
						  // = error)
uint16_t packetSize;	  // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;		  // count of all bytes currently in FIFO
uint8_t	 fifoBuffer[64];  // FIFO storage buffer

int pps			= 0;
int packetCount = 0;
int lastUpdate	= 0;
// orientation/motion vars
Quaternion	q;	 // [w, x, y, z]         quaternion container
VectorInt16 acc; // [x, y, z]            accel sensor measurements
VectorInt16 gyr; // [x, y, z]            gyroscopy sensor measurements

int	 press_time = 0;
bool state		= false;
// packet structure for InvenSense teapot demo
int			dmp_rate = 0;
FILE	   *output_file;
std::string file_name = "data.csv";

long recording_start = 0, timestart = 0;

// ================================================================
// ===                      INITIAL SETUP                       ===
// ================================================================

void _get_dmp_data( void ) {
	if( !state ) {
		return;
	}
	mpuIntStatus = mpu.getIntStatus();

	// does the FIFO have data in it?
	if( ( mpuIntStatus & 0x02 ) < 1 ) {
		return;
	}

	fifoCount = mpu.getFIFOCount();

	if( fifoCount < 42 ) {
		return;
	} else if( fifoCount > 1024 ) {
		mpu.resetFIFO();
		return;
	}

	mpu.getFIFOBytes( fifoBuffer, packetSize );
	mpu.dmpGetQuaternion( &q, fifoBuffer );
	mpu.dmpGetAccel( &acc, fifoBuffer );
	mpu.dmpGetGyro( &gyr, fifoBuffer );

	// ======= ======= ======== The start of the processing block ======== =======

	// ======= ======= ========  The end of the processing block  ======== =======

	// count packets per second

	if( lastUpdate == 0 ) {
		lastUpdate = millis();
	} else {
		if( millis() - lastUpdate >= 1000 ) {
			pps			= packetCount;
			packetCount = 0;
			lastUpdate += 1000;
			printf( "Packets per second: %d \n", pps );
		}
	}
	packetCount++;

#ifdef HARDWARE_INTERRUPT
	fprintf( output_file, "%ld, %7d, %7d, %7d, %7d, %7d, %7d, %7.5f, %7.5f, %7.5f, %7.5f\n", millis() - recording_start,
		acc.x, acc.y, acc.z, gyr.x, gyr.y, gyr.z, q.w, q.x, q.y, q.z );
#endif
	return;
}

const std::string currentDateTime() {
	time_t	  now = time( 0 );
	struct tm tstruct;
	char	  buf[80];
	tstruct = *localtime( &now );

	strftime( buf, sizeof( buf ), "%Y%m%d%X", &tstruct );

	return buf;
}

void buttonPressed( void ) {
	// debounce the button

	if( millis() - press_time < 300 ) {
		return;
	}

	press_time = millis();

	if( state ) {
		state = false;
		if( output_file != NULL ) {
			fclose( output_file );
			printf( "done and closed!\n" );
		}
		digitalWrite( LED_RED, HIGH );
	} else {
		if( timestart < WAIT_TIME ) {
			return;
		}
		state = true;
		// Prep new file
		file_name = currentDateTime() + "_data.csv";

		output_file = fopen( file_name.c_str(), "w" );

		// Write header
		fprintf( output_file, "time, acc_x, acc_y, acc_z, gyr_x, gyr_y, gyr_z, q_w, q_x, q_y, q_z\n" );
		printf( "Writing to: %s...\n", file_name.c_str() );
		digitalWrite( LED_RED, LOW );
		recording_start = millis();
	}
}

void setup() {
	timestart = millis();
	// initialize device
	printf( "Initializing I2C devices...\n" );
	mpu.initialize();

	// wiringPi initialize
	wiringPiSetup();
	pinMode( LED_RED, OUTPUT );
	pinMode( LED_GREEN, OUTPUT );
	pinMode( BUTTON, INPUT );

	// interrupt for the button
	if( wiringPiISR( BUTTON, INT_EDGE_RISING, &buttonPressed ) < 0 ) {
		printf( "Error setting up button interrupt\n" );
	}

	digitalWrite( LED_RED, HIGH );
	digitalWrite( LED_GREEN, LOW );

	// verify connection
	printf( "Testing device connections...\n" );
	printf( mpu.testConnection() ? "MPU6050 connection successful\n" : "MPU6050 connection failed\n" );

	// load and configure the DMP
	printf( "Initializing DMP...\n" );

	devStatus = mpu.dmpInitialize( dmp_rate );

	mpu.setXAccelOffset( -4094 );
	mpu.setYAccelOffset( 1792 );
	mpu.setZAccelOffset( 1228 );

	mpu.setXGyroOffset( 161 );
	mpu.setYGyroOffset( -41 );
	mpu.setZGyroOffset( 20 );

	// make sure it worked (returns 0 if so)
	if( devStatus == 0 ) {
		// turn on the DMP, now that it's ready
		printf( "Enabling DMP...\n" );
		mpu.setDMPEnabled( true );

// enable hardware Interrupt on WiringPi
#ifdef HARDWARE_INTERRUPT
		if( wiringPiISR( INTERRUPT_PIN, INT_EDGE_RISING, &_get_dmp_data ) < 0 ) {
			printf( "Error setting up DMP interrupt\n" );
		}
#endif
		mpuIntStatus = mpu.getIntStatus();
		printf( "DMP ready!\n" );
		dmpReady   = true;
		packetSize = mpu.dmpGetFIFOPacketSize();
	} else {
		printf( "DMP Initialization failed (code %d)\n", devStatus );
	}
}

// ================================================================
// ===                    MAIN PROGRAM LOOP                     ===
// ================================================================

void loop() {
	// Get start time

	if( timestart > WAIT_TIME && !state )
		digitalWrite( LED_GREEN, HIGH );

#ifndef HARDWARE_INTERRUPT
	// If state is true, start recording
	if( state ) {
		// Collecting data here with no interrupt
		_get_dmp_data();

		// print the data to the file
		fprintf( output_file, "%ld, %7d, %7d, %7d, %7d, %7d, %7d, %7.5f, %7.5f, %7.5f, %7.5f\n",
			millis() - recording_start, accReal.x, accReal.y, accReal.z, gyr.x, gyr.y, gyr.z, q.w, q.x, q.y, q.z );
	}

#endif
}

int main( int argc, char *argv[] ) {
	// check that user has provided a rate
	if( argc < 2 ) {
		printf( "Usage: %s <rate>\n", argv[0] );
		printf( "Output data rate is calculated as: 200/(rate+1)\n\n" );
		printf( " Output Rate  |  Rate\n" );
		printf( "--------------+-------\n" );
		printf( "     200Hz    |  0\n" );
		printf( "     100Hz    |  1\n" );
		printf( "     67Hz     |  2\n" );
		printf( "     50Hz     |  3\n" );
		printf( "     40Hz     |  4\n" );
		printf( "     33Hz     |  5\n" );
		printf( "     20Hz     |  9\n" );
		printf( "     10Hz     |  19\n" );
		printf( "     5Hz      |  39\n" );
		printf( "     1Hz      |  199\n" );

		return 1;
	}

	// set the rate
	dmp_rate = atoi( argv[1] );

	printf( "Expect %d packets per second\n", 200 / ( dmp_rate + 1 ) );
	setup();
	while( 1 ) {
		loop();
	}

	return 0;
}