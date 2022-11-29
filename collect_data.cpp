#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"

#include <dirent.h>
#include <filesystem>
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
#define BUTTON		  7
#define INTERRUPT_PIN 27

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
bool		state	   = 0;
// packet structure for InvenSense teapot demo
uint8_t teapotPacket[14] = { '$', 0x02, 0, 0, 0, 0, 0, 0, 0, 0, 0x00, 0x00, '\r', '\n' };

FILE *arq_Accel, *arq_Gyro, *arq_Quaternions, *arq_Euler, *arq_YawPitchRoll, *arq_LinearAcc, *arq_WorldAcc;

std::string namepaste = "";

struct timeval start, end, startc, endc, startb, endb;
long		   mtime, seconds, useconds, timestart, secondsb, usecondsb, timestartb;

// ================================================================
// ===                      INITIAL SETUP                       ===
// ================================================================

void _get_dmp_data() {
	if( state ) {
		return;
	}
	uint8_t _device_interrupt_status = mpu.getIntStatus();

	// does the FIFO have data in it?
	if( ( _device_interrupt_status & 0x02 ) < 1 ) {
		return;
	}

	fifoCount = mpu.getFIFOCount();

	if( fifoCount < packetSize ) {
		return;
	}

	mpu.getFIFOBytes( fifoBuffer, packetSize );

	printf( "%ld, %7d, %7d, %7d\n", mtime, acc.x, acc.y, acc.z );
	printf( "%ld, %7d, %7d, %7d\n", mtime, gyr.x, gyr.y, gyr.z );
	printf( "%ld, %7.5f, %7.5f, %7.5f, %7.5f\n", mtime, q.w, q.x, q.y, q.z );
	return;
}

void buttonPressed() {
	// debounce the button
	if( millis() - press_time < 1000 ) {
		return;
	}

	press_time = millis();

	if( state ) {
		state = 0;
	} else {
		state = 1;
	}
	printf( "Button pressed\n" );
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
	wiringPiISR( BUTTON, INT_EDGE_RISING, &buttonPressed );

	digitalWrite( LED_RED, HIGH );
	digitalWrite( LED_GREEN, LOW );

	// verify connection
	printf( "Testing device connections...\n" );
	printf( mpu.testConnection() ? "MPU6050 connection successful\n" : "MPU6050 connection failed\n" );

	// load and configure the DMP
	printf( "Initializing DMP...\n" );

	while( digitalRead( BUTTON ) == 1 ) {
	}

	devStatus = mpu.dmpInitialize();

	mpu.setXAccelOffset( -4099 );
	mpu.setYAccelOffset( 1793 );
	mpu.setZAccelOffset( 1227 );

	mpu.setXGyroOffset( 161 );
	mpu.setYGyroOffset( -40 );
	mpu.setZGyroOffset( 0 );

	// make sure it worked (returns 0 if so)
	if( devStatus == 0 ) {
		// turn on the DMP, now that it's ready
		printf( "Enabling DMP...\n" );
		mpu.setDMPEnabled( true );

		// enable Interrupt on WiringPi
		wiringPiISR( INTERRUPT_PIN, INT_EDGE_RISING, &_get_dmp_data );

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

	if( timestart > 5000 )
		digitalWrite( LED_GREEN, HIGH );

	if( digitalRead( BUTTON ) == true && timestart > 25000 ) {
		gettimeofday( &startb, NULL );

		while( digitalRead( BUTTON ) ) {
			gettimeofday( &endb, NULL );
			secondsb   = endb.tv_sec - startb.tv_sec;
			usecondsb  = endb.tv_usec - startb.tv_usec;
			timestartb = ( ( secondsb ) *1000 + usecondsb / 1000.0 ) + 0.5;
		}

		if( state ) {
			fclose( arq_Accel );
			fclose( arq_Gyro );
			fclose( arq_Quaternions );

			digitalWrite( LED_RED, HIGH );
		} else {
			digitalWrite( LED_RED, LOW );

			DIR *d;

			std::string data_folder = "Data"; // if Data folder doesn't exist, create it

			if( opendir( data_folder.c_str() ) == NULL ) {
				mkdir( data_folder.c_str(), 0777 );
			}

			struct dirent *dir;
			d			= opendir( data_folder.c_str() );
			int dir_len = 0;
			if( d != NULL ) {
				while( ( dir = readdir( d ) ) != NULL )
					dir_len++;
				( void ) closedir( d );
				printf( "Directory opened. Number of files: %d\n", dir_len );
			} else {
				perror( "Couldn't open the directory" );
			}

			printf( "%d", dir_len );
			namepaste = data_folder + std::to_string( dir_len - 2 );
			printf( namepaste.c_str() );

			std::string new_dir		 = namepaste;
			std::string existing_dir = ".";
			struct stat atributes;
			stat( existing_dir.c_str(), &atributes );
			mkdir( new_dir.c_str(), atributes.st_mode );

			std::string Data_Accel = namepaste + "/Data_Accel.txt", Data_Gyro = namepaste + "/Data_Gyro.txt",
						Data_Quaternions = namepaste + "/Data_Quaternions.txt";

			arq_Accel = fopen( Data_Accel.c_str(), "wt" );
			fprintf( arq_Accel, "time,accx,accy,accz\n" );

			arq_Gyro = fopen( Data_Gyro.c_str(), "wt" );
			fprintf( arq_Gyro, "time,gyrx,gyry,gyrz\n" );

			arq_Quaternions = fopen( Data_Quaternions.c_str(), "wt" );
			fprintf( arq_Quaternions, "time,qw,qx,qy,qz\n" );

			gettimeofday( &startc, NULL );
		}
		state = !state;
		printf( "Change state " );
		printf( "%d\n", state );
	}
	// if programming failed, don't try to do anything
	// if( !dmpReady )
	// 	return;
	// // get current FIFO count
	// fifoCount = mpu.getFIFOCount();

	// if( fifoCount == 1024 ) {
	// 	// reset so we can continue cleanly
	// 	mpu.resetFIFO();
	// 	printf( "FIFO overflow!\n" );

	// 	// otherwise, check for DMP data ready interrupt (this should happen
	// 	// frequently)
	// } else if( fifoCount >= 42 ) {
	// 	if( state )
	// 		digitalWrite( LED_GREEN, LOW );

	// 	// read a packet from FIFO
	// 	mpu.getFIFOBytes( fifoBuffer, packetSize );

	// 	// get the time to create the millis function
	gettimeofday( &endc, NULL );
	seconds	 = endc.tv_sec - startc.tv_sec;
	useconds = endc.tv_usec - startc.tv_usec;

	mtime = ( ( seconds ) *1000 + useconds / 1000.0 ) + 0.5; // current time in ms
															 // 	// display time in milliseconds

	// 	// Start of the processing block
	// 	// End of the processing block

	// 	printf( "\n" );
	// 	if( state ) {
	// 		fprintf( arq_Accel, "%ld,%6d,%6d,%6d\n", mtime, acc.x, acc.y, acc.z );
	// 		fprintf( arq_Gyro, "%ld,%6d,%6d,%6d\n", mtime, gyr.x, gyr.y, gyr.z );
	// 		fprintf( arq_Quaternions, "%ld,%7.5f,%7.5f,%7.5f,%7.5f\n", mtime, q.w, q.x, q.y, q.z );
	// 	}
	// }
}

int main() {
	setup();
	while( 1 ) {
		loop();
	}

	return 0;
}
