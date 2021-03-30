#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <stdint.h>
#include <string.h>
#include <math.h>
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include <time.h>
#include "MechaQMC5883.h"

// class default I2C address is 0x68
// specific I2C addresses may be passed as a parameter here
// AD0 low = 0x68 (default for SparkFun breakout and InvenSense evaluation board)
// AD0 high = 0x69
MPU6050 mpu;
MechaQMC5883 compass;
int16_t x_n , y_n,z_n;
float angle;
double Xm,Ym,Zm,heading,headingDegrees,headingFiltered;
// uncomment "OUTPUT_READABLE_QUATERNION" if you want to see the actual
// quaternion components in a [w, x, y, z] format (not best for parsing
// on a remote host such as Processing or something though)
//#define OUTPUT_READABLE_WORLDACCEL

// uncomment "OUTPUT_READABLE_EULER" if you want to see Euler angles
// (in degrees) calculated from the quaternions coming from the FIFO.
// Note that Euler angles suffer from gimbal lock (for more info, see
// http://en.wikipedia.org/wiki/Gimbal_lock)
//#define OUTPUT_READABLE_EULER

// uncomment "OUTPUT_READABLE_YAWPITCHROLL" if you want to see the yaw/
// pitch/roll angles (in degrees) calculated from the quaternions coming
// from the FIFO. Note this also requires gravity vector calculations.
// Also note that yaw/pitch/roll angles suffer from gimbal lock (for
// more info, see: http://en.wikipedia.org/wiki/Gimbal_lock)
#define OUTPUT_READABLE_YAWPITCHROLL

// uncomment "OUTPUT_READABLE_REALACCEL" if you want to see acceleration
// components with gravity removed. This acceleration reference frame is
// not compensated for orientation, so +X is always +X according to the
// sensor, just without the effects of gravity. If you want acceleration
// compensated for orientation, us OUTPUT_READABLE_WORLDACCEL instead.
#define OUTPUT_READABLE_REALACCEL

// uncomment "OUTPUT_READABLE_WORLDACCEL" if you want to see acceleration
// components with gravity removed and adjusted for the world frame of
// reference (yaw is relative to initial orientation, since no magnetometer
// is present in this case). Could be quite handy in some cases.
//#define OUTPUT_READABLE_WORLDACCEL

// uncomment "OUTPUT_TEAPOT" if you want output that matches the
// format used for the InvenSense teapot demo
//#define OUTPUT_TEAPOT

// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector
double tx,ty,tz,p_ty;
// packet structure for InvenSense teapot demo
uint8_t teapotPacket[14] = { '$', 0x02, 0,0, 0,0, 0,0, 0,0, 0x00, 0x00, '\r', '\n' };
time_t now;
double count_vel = 0;
double pcount_displacement,count_displacement=0;
double gaussx ,gaussy,gaussz;
	int16_t mag_x, mag_y, mag_z;
	int16_t acc_x, acc_y, acc_z;
	double roll;
	double pitch;
	double azimuth;
	double X_h, Y_h;
    double x,y,p_x,p_y;

 struct timespec tstart={0,0}, tend={0,0};
// ================================================================
// ===                      INITIAL SETUP                       ===
// ================================================================

void setup() {
    // initialize device
    //printf("Initializing I2C devices...\n");
    mpu.initialize();
    compass.setAddress(0x0d);
    compass.init();

    // verify connection
   // printf("Testing device connections...\n");
   // printf(mpu.testConnection() ? "MPU6050 connection successful\n" : "MPU6050 connection failed\n");

    // load and configure the DMP
   // printf("Initializing DMP...\n");
    devStatus = mpu.dmpInitialize();
    
    // make sure it worked (returns 0 if so)
    if (devStatus == 0) {
        // turn on the DMP, now that it's ready
        //printf("Enabling DMP...\n");
        mpu.setDMPEnabled(true);

        // enable Arduino interrupt detection
        //Serial.println(F("Enabling interrupt detection (Arduino external interrupt 0)..."));
        //attachInterrupt(0, dmpDataReady, RISING);
        mpuIntStatus = mpu.getIntStatus();

        // set our DMP Ready flag so the main loop() function knows it's okay to use it
        //printf("DMP ready!\n");
        dmpReady = true;

        // get expected DMP packet size for later comparison
        packetSize = mpu.dmpGetFIFOPacketSize();
    } else {
        // ERROR!
        // 1 = initial memory load failed
        // 2 = DMP configuration updates failed
        // (if it's going to break, usually the code will be 1)
       // printf("DMP Initialization failed (code %d)\n", devStatus);
    }
}


// ================================================================
// ===                    MAIN PROGRAM LOOP                     ===
// ================================================================

void loop() {
    // if programming failed, don't try to do anything
    if (!dmpReady) return;
    // get current FIFO count
    fifoCount = mpu.getFIFOCount();

    if (fifoCount == 1024) {
        // reset so we can continue cleanly
        mpu.resetFIFO();
       // printf("FIFO overflow!\n");

    // otherwise, check for DMP data ready interrupt (this should happen frequently)
    } else if (fifoCount >= 42) {
        // read a packet from FIFO
        mpu.getFIFOBytes(fifoBuffer, packetSize);
        
        #ifdef OUTPUT_READABLE_QUATERNION
            // display quaternion values in easy matrix form: w x y z
            mpu.dmpGetQuaternion(&q, fifoBuffer);
            printf("quat %7.2f %7.2f %7.2f %7.2f    ", q.w,q.x,q.y,q.z);
        #endif

        #ifdef OUTPUT_READABLE_EULER
            // display Euler angles in degrees
            mpu.dmpGetQuaternion(&q, fifoBuffer);
            mpu.dmpGetEuler(euler, &q);
            printf("euler %7.2f %7.2f %7.2f    ", euler[0] * 180/M_PI, euler[1] * 180/M_PI, euler[2] * 180/M_PI);
        #endif

        #ifdef OUTPUT_READABLE_YAWPITCHROLL
            // display Euler angles in degrees
            mpu.dmpGetQuaternion(&q, fifoBuffer);
            mpu.dmpGetGravity(&gravity, &q);
            mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
         //   printf("%7.2f %7.2f %7.2f   ", ypr[0] * 180/M_PI+180, ypr[1] * 180/M_PI, ypr[2] * 180/M_PI);
            //in radians
          //  printf("%7.2f %7.2f %7.2f ", ypr[0], ypr[1], ypr[2]);
            // magnetometer
            compass.read(&x_n,&y_n,&z_n,&angle);
            /*
            gaussx = double(x_n);
            gaussy = double(y_n);
            gaussz = double(z_n);
            
            Xm = double(gaussx)*cos(ypr[1]) + double(gaussy)*sin(ypr[2])*sin(ypr[1]) + double(gaussz)*cos(ypr[2])*sin(ypr[1]);
            Ym = double(gaussy)*cos(ypr[2]) - double(gaussz)*sin(ypr[2]);
            printf(" %d %d %d   ",x_n,y_n,z_n);
            
            */
           
            
            mag_x = x_n;
            mag_y = y_n;
            mag_z = z_n;
            pitch = ypr[1];
            roll =  ypr[2];
            //printf(" witout %f %f ",Xm,Ym);
            
            /*
            heading = atan2(Ym, Xm);
            
            //heading = atan2(gaussy, gaussx);
                       //heading = atan2(gaussy, gaussx);
            //heading = atan2(gaussy, gaussx);
            if(heading < 0) {	// Convert Azimuth in the range (0, 2pi) 
			heading = 2*M_PI + heading;
		}
            //headingFiltered = headingDegrees;
            headingFiltered = headingFiltered*0.85 + headingDegrees*0.15;
            printf("  %f  ",heading);
            */
        X_h = (double)mag_x*cos(pitch) + (double)mag_y*sin(roll)*sin(pitch) + (double)mag_z*cos(roll)*sin(pitch);
		Y_h = (double)mag_y*cos(roll) - (double)mag_z*sin(roll);
		azimuth = atan2(Y_h, X_h);
		if(azimuth < 0) {	/* Convert Azimuth in the range (0, 2pi) */
			azimuth = 2*M_PI + azimuth;
		}
        //printf("  %d  ",(int)(azimuth*180/PI));
        //printf(" %d %d ",(int)(32 + 24 * sin(azimuth)),(int)(32 - 24 * cos(azimuth)));

        #endif

        #ifdef OUTPUT_READABLE_REALACCEL
            // display real acceleration, adjusted to remove gravity
            mpu.dmpGetQuaternion(&q, fifoBuffer);
            mpu.dmpGetAccel(&aa, fifoBuffer);
            mpu.dmpGetGravity(&gravity, &q);
            mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
            tx = (double(aa.x)/8192)*9.806-(sin(ypr[1]))*9.806;
            ty = ((double(aa.y)/8192)*9.806-(sin(ypr[2]))*9.806);
            tz = (double(aa.z)/8192)*9.806-(cos(ypr[2]))*9.806;
            //time(&now);
            clock_gettime(CLOCK_MONOTONIC, &tstart);
            //printf("%7.2f %7.2f %7.2f %.5f", tx,ty,tz,((double)tend.tv_sec + 1.0e-9*tend.tv_nsec) - ((double)tstart.tv_sec + 1.0e-9*tstart.tv_nsec));
            //printf("%f %f %f", tx,ty,tz);
            tend.tv_nsec =  tstart.tv_nsec;
            tend.tv_sec = tstart.tv_sec;
            


           // printf("  %f  ",heading*180/PI);
           //printf("  %f  ",angle);
           // /*printf("%7.2f ",ty);
           if(ty>0.2){
               count_vel += 0.03*ty;
           }
           else if(ty<0&&count_vel>0){
               count_vel += 0.035*ty;
               if(count_vel<0){
                   count_vel =0;
               }
           }
           if(fabsf(p_ty-ty)<0.009)count_vel =0;
           count_displacement += 0.03*count_vel;
           
           p_ty=ty;
           x = ((count_displacement-pcount_displacement)*sin(ypr[0]))+p_x;
           y = ((count_displacement-pcount_displacement)*cos(ypr[0]))+p_y;


           p_x = x;
           p_y = y;
           pcount_displacement = count_displacement;
           printf("%f %f ",x,y);
           printf("%f %f %f %f",ypr[0]*180/PI+180,ty,count_vel,count_displacement*100);
            
        #endif
        printf("\n");
    }
}

int main() {
    setup();
    usleep(100000);
    for (;;)
        loop();

    return 0;
}
