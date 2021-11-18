
#include "beep.h"
#include "motors.h"
#include "encoders.h"
#include "kinematics.h"
#include "bumper.h"
#define BAUD_RATE 9600

#include <Wire.h>
#include <LSM6.h>

LSM6 imu;

#define ROTATION_ANGLE_THRESHOLD 1 //degree
#define ROTATION_SPEED 20 //PWM for turning on the spot                                  low : 20 // medium : 60 // HIGH : 110
#define FORWARD_SPEED 30 // PWM for going forward                                        low : 30  // medium : 80 // HIGH : 140
#define GO_STRAIGHT_K_p 20 //20 before //K_p for the P controller of go straight dist function          low 20 med 40 high 60?
#define ROTATION_K_p 0.4 //K_p for the P controller of go straight dist function
#define THRESHOLD_REACH_X_Y 20

Beep_c Buzzer;
Motors_c Motors;
kinematics kine;
BumpSensor_c bump;

// the setup function runs once when you press reset or power the board
void setup() {
    //Start a serial connection
    Serial.begin(BAUD_RATE);
    // Wait for stable connection, report reset.
    delay(1000);
    Serial.println("***RESET***");
    Buzzer.buzz(1911,100);
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
    imu.enableDefault();

    
    Buzzer.buzz(1517,100);
    kine.locate();

    
}

float accel[3] = {0,0,0};
float accel_angle[3] = {0,0,0};
float gyro[3] = {0,0,0};
float gyro_angle[3] = {0,0,0};

float combined_angle[3] = {0,0,0};

unsigned long past_time = millis();
unsigned long time = millis();
int dt;

float alpha = 0.95; // how much of accelero we take


void loop()
{   
    time = millis();
    if (time-past_time > 100) {

        imu.read();
        accel[1] = imu.a.x *0.061;
        accel[2] = imu.a.y *0.061;
        accel[3] = imu.a.z *0.061; // mg milli gravity


        accel_angle[1] = 360*atan2(accel[2],accel[3])/(2*PI);
        accel_angle[2] = 360*atan2(accel[1],accel[3])/(2*PI);
        accel_angle[3] = 0; // mg milli gravity


        gyro[1] = imu.g.x * 4.375/1000;
        gyro[2] = imu.g.y * 4.375/1000; //degre / seconde
        gyro[3] = imu.g.z * 4.375/1000;

        gyro_angle[1] = gyro_angle[1] + (millis()-past_time)*gyro[1]/1000;
        gyro_angle[2] = gyro_angle[2] + (millis()-past_time)*gyro[2]/1000;
        gyro_angle[3] = gyro_angle[3] + (millis()-past_time)*gyro[3]/1000;

        combined_angle[1] = (1-alpha) * gyro_angle[1] + alpha * accel_angle[1];
        combined_angle[2] = (1-alpha) * gyro_angle[2] + alpha * accel_angle[2];
        combined_angle[3] = 0; //gyro_angle[3];

/*
        Serial.print("A: ");
        Serial.print(accel_angle[1]);
        Serial.print(" ");
        Serial.print(accel_angle[2]);
        Serial.print(" ");
        Serial.print(accel_angle[3]);




        Serial.print("\t G:  ");
        Serial.print(gyro_angle[1]);
        Serial.print(" ");
        Serial.print(gyro_angle[2]);
        Serial.print(" ");
        Serial.println(gyro_angle[3]);
*/

        Serial.print(combined_angle[1]);
        Serial.print(",");
        Serial.print(combined_angle[2]);
        Serial.print(",");
        Serial.println(combined_angle[3]);


        past_time = millis();
    }

}


