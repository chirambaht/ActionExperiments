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

#define WAIT_TIME 30000

MPU6050 mpu;

#define OUTPUT_READABLE_ACCEL
#define OUTPUT_READABLE_GYRO
#define OUTPUT_READABLE_QUATERNION
//#define OUTPUT_READABLE_EULER
//#define OUTPUT_READABLE_YAWPITCHROLL
// #define OUTPUT_READABLE_REALACCEL
//#define OUTPUT_READABLE_WORLDACCEL

// MPU control/status vars
bool	dmpReady = false; // set true if DMP init was successful
uint8_t mpuIntStatus;	  // holds actual interrupt status byte from MPU
uint8_t devStatus;		  // return status after each device operation (0 = success, !0
						  // = error)
uint16_t packetSize;	  // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;		  // count of all bytes currently in FIFO
uint8_t	 fifoBuffer[64];  // FIFO storage buffer

// orientation/motion vars
Quaternion	q;		  // [w, x, y, z]         quaternion container
VectorInt16 acc;	  // [x, y, z]            accel sensor measurements
VectorInt16 gyr;	  // [x, y, z]            gyroscopy sensor measurements
VectorInt16 accReal;  // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 accWorld; // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;  // [x, y, z]            gravity vector
float		euler[3]; // [psi, theta, phi]    Euler angle container
float		ypr[3];	  // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector
int			press_time = 0;
bool		state	   = false;
// packet structure for InvenSense teapot demo

FILE	   *output_file;
std::string file_name = "data.csv";

struct timeval start, end, startc, endc, startb, endb;
long		   mtime, seconds, useconds, timestart, secondsb, usecondsb, timestartb;

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

	if( fifoCount < packetSize ) {
		return;
	}

	mpu.getFIFOBytes( fifoBuffer, packetSize );

	mpu.dmpGetAccel( &acc, fifoBuffer );
	mpu.dmpGetQuaternion( &q, fifoBuffer );
	mpu.dmpGetGyro( &gyr, fifoBuffer );

#ifdef HARDWARE_INTERRUPT
	fprintf( output_file, "%ld, %7d, %7d, %7d, %7d, %7d, %7d, %7.5f, %7.5f, %7.5f, %7.5f\n", mtime, acc.x, acc.y, acc.z,
		gyr.x, gyr.y, gyr.z, q.w, q.x, q.y, q.z );

#endif

	printf( "%ld, %7d, %7d, %7d, %7d, %7d, %7d, %7.5f, %7.5f, %7.5f, %7.5f\n", mtime, acc.x, acc.y, acc.z, gyr.x, gyr.y,
		gyr.z, q.w, q.x, q.y, q.z );

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
		printf( "Writing to: %s...", file_name.c_str() );
		digitalWrite( LED_RED, LOW );
	}
}

void setup() {
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

	devStatus = mpu.dmpInitialize();

	mpu.setXAccelOffset( -4103 );
	mpu.setYAccelOffset( 1803 );
	mpu.setZAccelOffset( 1225 );

	mpu.setXGyroOffset( 162 );
	mpu.setYGyroOffset( -41 );
	mpu.setZGyroOffset( -500 );

	// make sure it worked (returns 0 if so)
	if( devStatus == 0 ) {
		// turn on the DMP, now that it's ready
		printf( "Enabling DMP...\n" );
		mpu.setDMPEnabled( true );

		// enable hardware Interrupt on WiringPi
		// if( wiringPiISR( INTERRUPT_PIN, INT_EDGE_RISING, &_get_dmp_data ) < 0 ) {
		// 	printf( "Error setting up DMP interrupt\n" );
		// }

		mpuIntStatus = mpu.getIntStatus();

		// set our DMP Ready flag so the main loop() function knows it's okay to use
		// it
		printf( "DMP ready!\n" );
		dmpReady = true;

		// get expected DMP packet size for later comparison
		packetSize = mpu.dmpGetFIFOPacketSize();
	} else {
		// ERROR!
		// 1 = initial memory load failed
		// 2 = DMP configuration updates failed
		// (if it's going to break, usually the code will be 1)
		printf( "DMP Initialization failed (code %d)\n", devStatus );
	}

	gettimeofday( &start, NULL );
	gettimeofday( &startc, NULL );
}

// ================================================================
// ===                    MAIN PROGRAM LOOP                     ===
// ================================================================

void loop() {
	// Get start time
	gettimeofday( &end, NULL );
	seconds	  = end.tv_sec - start.tv_sec;
	useconds  = end.tv_usec - start.tv_usec;
	timestart = ( ( seconds ) *1000 + useconds / 1000.0 ) + 0.5;

	if( timestart > WAIT_TIME && !state )
		digitalWrite( LED_GREEN, HIGH );

#ifndef HARDWARE_INTERRUPT
	// If state is true, start recording
	if( state ) {
		// Collecting data here with no interrupt
		_get_dmp_data();

		// print the data to the file
		fprintf( output_file, "%ld, %7d, %7d, %7d, %7d, %7d, %7d, %7.5f, %7.5f, %7.5f, %7.5f\n", mtime, acc.x, acc.y,
			acc.z, gyr.x, gyr.y, gyr.z, q.w, q.x, q.y, q.z );
	}

#endif

	gettimeofday( &endc, NULL );
	seconds	 = endc.tv_sec - startc.tv_sec;
	useconds = endc.tv_usec - startc.tv_usec;

	mtime = ( ( seconds ) *1000 + useconds / 1000.0 ) + 0.5; // current time in ms
}

int main() {
	setup();
	while( 1 ) {
		loop();
	}

	return 0;
}
