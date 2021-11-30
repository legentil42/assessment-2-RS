#include <Kalman.h>


#include "beep.h"
#include "motors.h"
#include "encoders.h"
#include "kinematics.h"
#include "bumper.h"
#define BAUD_RATE 9600

#include <Wire.h>
#include <LSM6.h>
#include <LIS3MDL.h>



LSM6 imu;
LIS3MDL mag;

#define ROTATION_ANGLE_THRESHOLD 1 //degree
#define ROTATION_SPEED 20 //PWM for turning on the spot                                  low : 20 // medium : 60 // HIGH : 110
#define FORWARD_SPEED 30 // PWM for going forward                                        low : 30  // medium : 80 // HIGH : 140
#define GO_STRAIGHT_K_p 20 //20 before //K_p for the P controller of go straight dist function          low 20 med 40 high 60?
#define ROTATION_K_p 0.4 //K_p for the P controller of go straight dist function
#define THRESHOLD_REACH_X_Y 20
#define LP_acc_mag 0.5
#define PI 3.1415
float acc_mag_stationary = 0;
bool stationary;
Beep_c Buzzer;
Motors_c Motors;
kinematics kine;
BumpSensor_c bump;
float Sum_gyrZ = 0;
// the setup function runs once when you press reset or power the board
void setup() {
    //Start a serial connection
    Serial.begin(BAUD_RATE);
    // Wait for stable connection, report reset.
    delay(1000);
    Serial.println("***RESET***");
    //Buzzer.buzz(1911,100);
    setupEncoder0();
    setupEncoder1();
    delay(500);
    bump.initialise();
    
    Wire.begin();
    if (!imu.init())
    {
    Serial.println("Failed to detect and initialize IMU!");
    while (1);
    }
        if (!mag.init())
    {
    Serial.println("Failed to detect and initialize magnetometer!");
    while (1);
    }

    mag.enableDefault();
    imu.enableDefault();
    calibrate_acc_mag_stationary();
    calculate_offset_gyroZ();
    Buzzer.buzz(1517,50);
    kine.locate();

    
}



unsigned long past_time = millis();
unsigned long time = millis();
unsigned long dt;

float acc_mag = 0;
float accX,accY,accZ;
float gyrX,gyrY,gyrZ;
float gyr_angleX = 0;
float gyr_angleY = 0;
float gyr_angleZ = 0;
float alpha,beta,gamma,heading;

float vx = 0;
float vy = 0;
float vz = 0;
float x = 0;
float y = 0;
float z = 0;



#define weight 0.5
#define LOW_PASS 0.
void loop(){  
    time = millis();
    if (time-past_time > 10) {

        dt = time-past_time;
        check_if_stationary();
        update_theta();
        imu.read();

        accX = (1-LOW_PASS) * accX + LOW_PASS * (imu.a.x*0.061*9.81/1000);

        accY = imu.a.y*0.061*9.81/1000;
        accZ = imu.a.z*0.061*9.81/1000;

        if (stationary == false) {
            vx = vx + accX * dt/1000;
            vy = vy + accY * dt/1000;
            vz = vz + accZ * dt/1000;
            }
            x = x + vx*dt;
            y = y + vy*dt;
            z = z + vz*dt;
        

            Serial.println(gyr_angleZ);
            //Serial.print(",");
            /*
            Serial.print(accY);
            Serial.print(",");
            Serial.print(accZ);
            Serial.print(",");

*/

            //Serial.print(vx);
            //Serial.print(",");
            /*
            Serial.print(vy);
            Serial.print(",");
            Serial.print(vz);
            Serial.print(",");
            */
            //Serial.println(x);
            /*
            Serial.print(",");
            Serial.print(y);
            Serial.print(",");
            Serial.println(z);
*/
        past_time = millis();
}

}

void calibrate_acc_mag_stationary(){

    float calib_start_time = millis();
    float reading_start_time = millis();

    while (millis()-calib_start_time < 2000) {

        if (millis()-reading_start_time > 10) {

            imu.read();
            mag.read();

            accX = imu.a.x;
            accY = imu.a.y;
            accZ = imu.a.z;
            
            acc_mag = acc_mag * (1-LP_acc_mag) + LP_acc_mag*sqrt(accX*accX + accY*accY + accZ*accZ);
           

            if (acc_mag > acc_mag_stationary){
                acc_mag_stationary = acc_mag;
                 Serial.print(acc_mag_stationary);
                }

            reading_start_time = millis();
            }

    }
        acc_mag_stationary = acc_mag_stationary*1.01;
}


void calculate_offset_gyroZ() {

    for (int i=0;i<500;i++){

            Sum_gyrZ += imu.g.z*8.75 / 1000;
    }

    Sum_gyrZ = Sum_gyrZ/500;


}

void update_theta(){

            imu.read();

            accX = imu.a.x;
            accY = imu.a.y;
            accZ = imu.a.z;

            gyrX = imu.g.x*8.75 /1000; //degree per second
            gyrY = imu.g.y*8.75 /1000;
            gyrZ = (imu.g.z*8.75 / 1000) - Sum_gyrZ ;


            if (stationary == false || abs(gyrZ) > 4){
            gyr_angleZ += gyrZ*dt/1000;
            }

            alpha = atan2(accX,sqrt(accY*accY+accZ*accZ))*180/PI;
            beta = atan2(accY,sqrt(accX*accX+accZ*accZ))*180/PI;

    /*
            Serial.print(alpha);
            Serial.print(",");
            Serial.print(beta);
            Serial.print(",");
            Serial.println(gyr_angleZ);
            */

        }



void check_if_stationary() {

        imu.read();

            accX = imu.a.x;
            accY = imu.a.y;
            accZ = imu.a.z;
            acc_mag = acc_mag * (1-LP_acc_mag) + LP_acc_mag*sqrt(accX*accX + accY*accY + accZ*accZ);
            
            stationary = acc_mag < acc_mag_stationary;
}