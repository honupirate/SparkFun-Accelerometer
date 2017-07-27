/* 
MPU6050_Live_Plotter
This program is a combination of the two programs/libraries referenced below.  The program 
requires theArduino IDE, Processing IDE, and the libraries assoiated with the two origin
codes referenced below.  The program allows you to use the Arduino Plotter (it is an 
external plotter that runs through the Processing IDE using the code provided on Github 
by Devin Conley at the link above,) to graph live data from the Arduino and MPU6050 
Accel/Gyro Sensor without using the Arduino IDE Serial Plotter.  When coupled with the 
correct Processing IDE program and libraries (provided by Devin Conley at the link below,)
the user can output a live graph of the YPR Data from the MPU6050.

This program can also be used to serial print a "step monitor" using a dynamic acceleration
threshold filter to determine wether or not a "step" has been taken.  The code can be edited
by the user to adjust threshold values and determine acceptable values for the ypr intergers.
By: James Laville
======================================================================================================================
// I2C device class (I2Cdev) demonstration Arduino sketch for MPU6050 class using DMP (MotionApps v2.0)
// 6/21/2012 by Jeff Rowberg <jeff@rowberg.net>
// Updates should (hopefully) always be available at https://github.com/jrowberg/i2cdevlib
//
// Changelog:
//      2013-05-08 - added seamless Fastwire support
//                 - added note about gyro calibration
//      2012-06-21 - added note about Arduino 1.0.1 + Leonardo compatibility error
//      2012-06-20 - improved FIFO overflow handling and simplified read process
//      2012-06-19 - completely rearranged DMP initialization code and simplification
//      2012-06-13 - pull gyro and accel data from FIFO packet instead of reading directly
//      2012-06-09 - fix broken FIFO read sequence and change interrupt detection to RISING
//      2012-06-05 - add gravity-compensated initial reference frame acceleration output
//                 - add 3D math helper file to DMP6 example sketch
//                 - add Euler output and Yaw/Pitch/Roll output formats
//      2012-06-04 - remove accel offset clearing for better results (thanks Sungon Lee)
//      2012-06-01 - fixed gyro sensitivity to be 2000 deg/sec instead of 250
//      2012-05-30 - basic DMP initialization working

/* ============================================
I2Cdev device library code is placed under the MIT license
Copyright (c) 2012 Jeff Rowberg

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
THE SOFTWARE.
===========================================================================================
===========================================================================================
Example to demonstrate multi-variable plotting against time
-------------------------------------------------------------------------------------------
Plotter
v2.3.0
https://github.com/devinaconley/arduino-plotter
by Devin Conley
===========================================================================================
 */
#include "MPU6050_6Axis_MotionApps20.h"
//Uncomment the following line to use the Processing IDE Plotter
#include "Plotter.h"

MPU6050 mpu;

#define OUTPUT_READABLE_YAWPITCHROLL
#define LED_PIN 13 
bool blinkState = false;

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
VectorFloat gravity;    // [x, y, z]            gravity vector
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

double x;
double y;
double z;

// Uncomment the following line to run and declare the Processing IDE Plotter as global
 Plotter p;

volatile bool mpuInterrupt = false;     
void dmpDataReady() {
    mpuInterrupt = true;
}
/* Teachers and Students the following is the void setup piece of the program, if
 *  you plan to run the Processing IDE Plotter you will need to uncommnet 'Plotter p;'
 *  above, therefore declaring it as a global library.  Next you will need to uncomment any 
 *  line of code that begins with 'p.' in the void setup.
 */
void setup() {
  Serial.begin (115200);
// to start plotter using Processing IDE uncomment the following line
   p.Begin();
  
// to add 3 variable time graph using Processing IDE uncomment the next two lines
   p.AddTimeGraph( "3 variable time graph", 1000, "Roll", x,
   "Pitch", y, "Yaw", z );
       
    mpu.initialize();
    devStatus = mpu.dmpInitialize();
    mpu.setXGyroOffset(220);
    mpu.setYGyroOffset(76);
    mpu.setZGyroOffset(-85);
    mpu.setZAccelOffset(1788); 
   
    if (devStatus == 0) {
        mpu.setDMPEnabled(true);
        attachInterrupt(0, dmpDataReady, RISING);
        mpuIntStatus = mpu.getIntStatus();
        dmpReady = true;
        packetSize = mpu.dmpGetFIFOPacketSize();
    }

    pinMode(LED_PIN, OUTPUT);
}
/* Teachers and Students the following is the void loop piece of the program, this 
 *  is the piece of your program that loops eternally until you terminate it.  This 
 *  is where your program collects the data from the MPU 6050 sensor and decides 
 *  where and how to output that data.  If you would like to plot the data using 
 *  the Processing IDE plotter you must first uncomment any line of code that begins 
 *  with 'p.' You will need to first run this Program on the Arduino and then begin 
 *  the Processing IDE Plotter program to ensure that your graph appears.
 *  If you would like to adjust the threshold values of yaw, pitch, and roll,  (z, 
 *  y, or x), you can also do so below.  There are three lines of code written 
 *  specifically to serial print the value of each of the integers.  You will have
 *  to uncommnet the line you wish to print out, and it is reccomended that you
 *  comment out the if and else statements that follow.  This will help you avoid an
 *  overflow of information in the serial monitor.
 */
void loop() {
//delay the loop 20 milliseconds
    delay (20); 
    
    if (!dmpReady) return;

    while (!mpuInterrupt && fifoCount < packetSize) {
    }
    mpuInterrupt = false;
    mpuIntStatus = mpu.getIntStatus();
    fifoCount = mpu.getFIFOCount();

    if ((mpuIntStatus & 0x10) || fifoCount == 1024) {mpu.resetFIFO();
        }
          else if (mpuIntStatus & 0x02) {
        while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();

        mpu.getFIFOBytes(fifoBuffer, packetSize);
        fifoCount -= packetSize;

        #ifdef OUTPUT_READABLE_YAWPITCHROLL
           mpu.dmpGetQuaternion(&q, fifoBuffer);
           mpu.dmpGetGravity(&gravity, &q);
           mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
          #endif

// define yaw, pitch, and roll as intergers
        x= ypr[2]; //roll 
        y= ypr[1]; //pitch 
        z= ypr[0]; //yaw 
        
// to print the value of z (yaw) uncomment the line below
        //Serial.println (z); 
// to print the value of y (pitch)uncomment the line below
        //Serial.println (y);
// to print the value of x (roll) uncomment the line below
        //Serial.println (x);
       
/* The following code prints "step" when a step is taken according to the
*  provided dynamic acceleration threshold (also know as a filter).  
*  The first line sets the dynamic acceleration threshold,the second line tells 
*  the controller to print "step" if the threshold is reached.  The third line 
*  tells the controller that if anything else but the threshold occurs to print 
*  "Not Walking".  You can adjust the values of z, y, and x to adjust the dynamix 
*  acceleration threshold.  If you are using the Processing IDE to plot the data,
*  you will need to comment out all of the Plotter code comments and uncomment 
*  the following if/else statement for the serial monitor to function without 
*  printing confusing data.
*/ 
       //if (z > 2.5 or y < -0.33 or x > 0.5)
            //Serial.println("step");
      //else 
            //Serial.println("nowalking");

// to plot yaw, pitch, and roll data using Processing IDE uncomment the line below
        p.Plot();
       
        blinkState = !blinkState;
        digitalWrite(LED_PIN, blinkState);
    }
}
