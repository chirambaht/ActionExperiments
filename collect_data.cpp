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

#define LED_RED	  25
#define LED_GREEN 29
#define BUTTON	  0

MPU6050 mpu;

#define OUTPUT_READABLE_ACCEL
#define OUTPUT_READABLE_GYRO
#define OUTPUT_READABLE_QUATERNION
//#define OUTPUT_READABLE_EULER
//#define OUTPUT_READABLE_YAWPITCHROLL
#define OUTPUT_READABLE_REALACCEL
//#define OUTPUT_READABLE_WORLDACCEL

// MPU control/status vars
bool	 dmpReady = false; // set true if DMP init was successful
uint8_t	 mpuIntStatus;	   // holds actual interrupt status byte from MPU
uint8_t	 devStatus;		   // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;	   // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;		   // count of all bytes currently in FIFO
uint8_t	 fifoBuffer[64];   // FIFO storage buffer

// orientation/motion vars
Quaternion	q;		  // [w, x, y, z]         quaternion container
VectorInt16 acc;	  // [x, y, z]            accel sensor measurements
VectorInt16 gyr;	  // [x, y, z]            gyroscopy sensor measurements
VectorInt16 accReal;  // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 accWorld; // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;  // [x, y, z]            gravity vector
float		euler[3]; // [psi, theta, phi]    Euler angle container
float		ypr[3];	  // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

bool state = 0;

int fifo_rate = 1;
int fps;

#ifdef DMP_FIFO_RATE_DIVISOR
int fifo_rate = DMP_FIFO_RATE_DIVISOR;
#else
int fifo_rate = 0;
#endif

FILE *arq_Accel, *arq_Gyro, *arq_Quaternions, *arq_All, *arq_fps;

std::string namepaste = "";

struct timeval start, end, startc, endc, startb, endb, startd, endd;
long		   mtime, seconds, useconds, timestart, secondsb, usecondsb, timestartb, secondsd, usecondsd, timestartd;

// ================================================================
// ===                      INITIAL SETUP                       ===
// ================================================================

void setup() {
	// initialize device
	printf( "Initializing I2C devices...\n" );
	mpu.initialize();

	// wiringPi initialize
	wiringPiSetup();
	pinMode( LED_RED, OUTPUT );
	pinMode( LED_GREEN, OUTPUT );
	pinMode( BUTTON, INPUT );

	digitalWrite( LED_RED, HIGH );
	digitalWrite( LED_GREEN, LOW );

	// verify connection
	printf( "Testing device connections...\n" );
	printf( mpu.testConnection() ? "MPU6050 connection successful\n" : "MPU6050 connection failed\n" );

	// load and configure the DMP
	printf( "Initializing DMP...\n" );
	devStatus = mpu.dmpInitialize( fifo_rate );

	mpu.setXAccelOffset( -4029 );
	mpu.setYAccelOffset( 1758 );
	mpu.setZAccelOffset( 1234 );

	mpu.setXGyroOffset( 1600 );
	mpu.setYGyroOffset( -39 );
	mpu.setZGyroOffset( 18 );

	// My offsets
	//            XAccel			     YAccel				   ZAccel			               XGyro YGyro ZGyro
	//[-2679,-2678] --> [-15,1]	[398,398] --> [-1,2]	[1205,1206] --> [16381,16391]	[-116,-115] --> [-4,2]	[69,70]
	//--> [0,3]	[-93,-92] --> [0,3]

	// make sure it worked (returns 0 if so)
	if( devStatus == 0 ) {
		// turn on the DMP, now that it's ready
		printf( "Enabling DMP...\n" );
		mpu.setDMPEnabled( true );

		mpuIntStatus = mpu.getIntStatus();

		// set our DMP Ready flag so the main loop() function knows it's okay to use it
		printf( "DMP ready!\n" );
		dmpReady = true;

		// get expected DMP packet size for later comparison
		packetSize = mpu.dmpGetFIFOPacketSize();
	} else {
		printf( "DMP Initialization failed (code %d)\n", devStatus );
	}

	gettimeofday( &start, NULL );
	gettimeofday( &startc, NULL );
}

// ================================================================
// ===                    MAIN PROGRAM LOOP                     ===
// ================================================================

void loop() {
	gettimeofday( &end, NULL );
	seconds	  = end.tv_sec - start.tv_sec;
	useconds  = end.tv_usec - start.tv_usec;
	timestart = ( ( seconds ) *1000 + useconds / 1000.0 ) + 0.5;

	if( timestart > 25000 )
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
			fclose( arq_All );
			fclose( arq_fps );
			digitalWrite( LED_RED, HIGH );
		} else {
			digitalWrite( LED_RED, LOW );
			DIR			*d;
			struct dirent *dir;
			d			= opendir( "Datas" );
			int dir_len = 0;
			if( d != NULL ) {
				while( ( dir = readdir( d ) ) != NULL )
					dir_len++;
				( void ) closedir( d );
			} else {
				perror( "Couldn't open the directory" );
			}

			printf( "%d", dir_len );
			namepaste = "Datas/data_" + std::to_string( dir_len - 2 );
			printf( namepaste.c_str() );

			std::string new_dir		 = namepaste;
			std::string existing_dir = ".";
			struct stat atributes;
			stat( existing_dir.c_str(), &atributes );
			mkdir( new_dir.c_str(), atributes.st_mode );

			std::string Data_Accel = namepaste + "/Data_Accel.csv", Data_Gyro = namepaste + "/Data_Gyro.csv",
						Data_Quaternions = namepaste + "/Data_Quaternions.csv", Data_All = namepaste + "/Data_All.csv",
						Data_fps = namepaste + "/Data_fps.csv";

			arq_Accel = fopen( Data_Accel.c_str(), "wt" );
			fprintf( arq_Accel, "time,accx,accy,accz\n" );

			arq_Gyro = fopen( Data_Gyro.c_str(), "wt" );
			fprintf( arq_Gyro, "time,gyrx,gyry,gyrz\n" );

			arq_Quaternions = fopen( Data_Quaternions.c_str(), "wt" );
			fprintf( arq_Quaternions, "time,qw,qx,qy,qz\n" );

			arq_All = fopen( Data_All.c_str(), "wt" );
			fprintf( arq_All, "time,accx,accy,accz,gyrx,gyry,gyrz,qw,qx,qy,qz\n" );

			arq_fps = fopen( Data_fps.c_str(), "wt" );
			fprintf( arq_fps, "time,fps\n" );

			gettimeofday( &startc, NULL );
		}
		state = !state;
		printf( "Change state " );
		printf( "%d\n", state );
	}
	// if programming failed, don't try to do anything
	if( !dmpReady )
		return;
	// get current FIFO count
	fifoCount = mpu.getFIFOCount();

	if( fifoCount == 1024 ) {
		// reset so we can continue cleanly
		mpu.resetFIFO();

		// otherwise, check for DMP data ready interrupt (this should happen frequently)
	} else if( fifoCount >= 42 ) {
		if( state )
			digitalWrite( LED_GREEN, LOW );
		// read a packet from FIFO
		mpu.getFIFOBytes( fifoBuffer, packetSize );
		// get the time to create the millis function
		gettimeofday( &endc, NULL );
		seconds	 = endc.tv_sec - startc.tv_sec;
		useconds = endc.tv_usec - startc.tv_usec;
		mtime	 = ( ( seconds ) *1000 + useconds / 1000.0 ) + 0.5;
		// display time in milliseconds

		mpu.dmpGetAccel( &acc, fifoBuffer );
		mpu.dmpGetGyro( &gyr, fifoBuffer );
		mpu.dmpGetQuaternion( &q, fifoBuffer );

		if( state ) {
			fprintf( arq_Accel, "%ld,%6d,%6d,%6d\n", mtime, acc.x, acc.y, acc.z );
			fprintf( arq_Gyro, "%ld,%6d,%6d,%6d\n", mtime, gyr.x, gyr.y, gyr.z );
			fprintf( arq_Quaternions, "%ld,%7.5f,%7.5f,%7.5f,%7.5f\n", mtime, q.w, q.x, q.y, q.z );
			fprintf( arq_All, "%ld,%6d,%6d,%6d,%6d,%6d,%6d,%7.5f,%7.5f,%7.5f,%7.5f\n", mtime, acc.x, acc.y, acc.z,
				gyr.x, gyr.y, gyr.z, q.w, q.x, q.y, q.z );

			gettimeofday( &endd, NULL );
			secondsd   = endd.tv_sec - startd.tv_sec;
			usecondsd  = endd.tv_usec - startd.tv_usec;
			timestartd = ( ( secondsd ) *1000 + usecondsd / 1000.0 ) + 0.5;
			if( timestartd - timestartb >= 1000 ) {
				fprintf( arq_fps, "%ld,%d\n", timestartd, fps );
				fps = 0;
				gettimeofday( &startd, NULL );
				timestartb = timestartd;
			} else {
				fps++;
			}
		}
	}
}

int main( int argc, char **argv ) {
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
	int fifo_rate = atoi( argv[1] );
	setup();
	while( 1 ) {
		loop();
	}

	return 0;
}
