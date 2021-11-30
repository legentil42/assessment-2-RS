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

float XG,YG;
int VTheta;
float Glob, Ang;


LSM6 imu;
LIS3MDL mag;

#define ROTATION_ANGLE_THRESHOLD 1 //degree
#define ROTATION_SPEED 110 //PWM for turning on the spot                                  low : 20 // medium : 60 // HIGH : 110
#define FORWARD_SPEED 140 // PWM for going forward                                        low : 30  // medium : 80 // HIGH : 140
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

    go_to_X_Y(2000,0);
}



unsigned long past_time = millis();
unsigned long time = millis();
unsigned long dt;
float reading_start_time = millis();
float acc_mag = 0;
float accX,accY,accZ;
float gyrX,gyrY,gyrZ;
float gyr_angleX = 0;
float gyr_angleY = 0;
float gyr_angleZ = 0;
float alpha,beta,gamma,heading;

float theta_weight;

float theta_combined = 0;

#define weight 0.5
#define LOW_PASS 0.

void loop(){  }

void compute_new_theta(){  
    time = millis();
    if (time-past_time > 10) {

        dt = time-past_time;
        kine.locate();

        check_if_stationary();
        update_theta();
        imu.read();

       
        

        theta_combined = 0.8*kine.ThetaD + 0.2*gyr_angleZ;

        Serial.print(theta_combined);
        Serial.print(",");
        Serial.print(kine.ThetaD);
        Serial.print(",");
        Serial.println(gyr_angleZ);
        
        past_time = millis();

        Motors.L_speed = -theta_combined*2;
        Motors.R_speed = theta_combined*2;
        Motors.update_motors();

}




}

void calibrate_acc_mag_stationary(){

    float calib_start_time = millis();
    reading_start_time = millis();

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
            gyrZ = -( (imu.g.z*8.75 / 1000) - Sum_gyrZ) ;


            if (abs(gyrZ)>2){
            gyr_angleZ += gyrZ*(millis()-past_time)/1000;
            }
            else {

                gyr_angleZ -= 0.01*(gyr_angleZ-kine.ThetaD);
                kine.ThetaGlobal -= 0.01*(kine.ThetaD-gyr_angleZ) *(PI/180);

            }

            if (abs(gyr_angleZ-kine.ThetaD) > 10) {
                kine.ThetaGlobal = (gyr_angleZ) *(PI/180);

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


void go_to_X_Y(float X_goal, float Y_goal) {
    XG = X_goal;
    YG = Y_goal;
    float range = sqrt(sq(X_goal-kine.XGlobal)+sq(Y_goal-kine.YGlobal));
    float OldPosX = kine.XGlobal;
    float OldPosY = kine.YGlobal;
    float SRange = sqrt(sq(OldPosX-kine.XGlobal)+sq(OldPosY-kine.YGlobal));
    float limit = 100;
    float Elimit = 200;
    kine.locate();//find where you are
    bump.readBump();//check bum sensors
    compute_new_theta();
    Calculate();
    //rotation(Ang);
    turn();

    float goal_angle = atan2(Y_goal-kine.YGlobal,X_goal-kine.XGlobal);//find goal angle
    while (range > THRESHOLD_REACH_X_Y) {//while distance from goal is bigger than threshold
    
        bump.readBump();
        compute_new_theta();


        if (bump.L_val >1000 || bump.R_val >1000) {//generic bump behaviour
            float forward_angle = theta_combined;
            go_straight_for_distance(50,true);
            rotation(90+forward_angle);
            go_straight_for_distance(100,false);
            rotation(forward_angle);
        }

        kine.locate();//update position
        goal_angle = atan2(Y_goal-kine.YGlobal,X_goal-kine.XGlobal);//update angle
        range = sqrt(sq(X_goal-kine.XGlobal)+sq(Y_goal-kine.YGlobal));
        SRange = sqrt(sq(OldPosX-kine.XGlobal)+sq(OldPosY-kine.YGlobal));

        if (theta_combined*(PI/180)-goal_angle>PI) {//set goal angle to between -180 and 180
            goal_angle += 2*PI;
        }
        if (theta_combined*(PI/180)-goal_angle<-PI) {
            goal_angle -= 2*PI;
        }


        Motors.L_speed = FORWARD_SPEED + 1*(Ang);
        Motors.R_speed = FORWARD_SPEED - 1*(Ang); 
        //Motors.L_speed = FORWARD_SPEED + GO_STRAIGHT_K_p*(Ang);
        //Motors.R_speed = FORWARD_SPEED + GO_STRAIGHT_K_p*(Ang);
               
        if(range < Elimit){
            Motors.L_speed = (Motors.L_speed) * (range/(limit*2.5));
            Motors.R_speed = (Motors.R_speed) * (range/(limit*2.5));
            if(Motors.L_speed < 15 && Motors.R_speed < 15){
              Motors.L_speed = 15;
              Motors.R_speed = 15;
            }
        }
        else{}
        
        
        if(SRange < limit){
            Motors.L_speed = ((Motors.L_speed) * (SRange/limit));
            Motors.R_speed = ((Motors.R_speed) * (SRange/limit));
            if(Motors.L_speed < 15 && Motors.R_speed < 15){
              Motors.L_speed = 15;
              Motors.R_speed = 15;
            }
        }
        else{}
  
        Motors.update_motors();
        Calculate();
        if(Ang > 10){
          turn();
        }
    }
    Motors.stop_motors();
}

void turn(){
  while(abs(Ang) > 0.1){
      kine.locate();
      Calculate();
      if(Ang >0){
        //turn right
        Motors.L_speed = FORWARD_SPEED/5;
        Motors.R_speed = -FORWARD_SPEED/5;
      }
      else if(Ang < 0){
        //turn left
        Motors.L_speed = -FORWARD_SPEED/5;
        Motors.R_speed = FORWARD_SPEED/5;
      }
      else{
        Motors.L_speed = 0;
        Motors.R_speed = 0;
      }
      Motors.update_motors();
  }
}

void Calculate(){
    int Angle = atan2(abs(XG-kine.XGlobal),abs(YG-kine.YGlobal))*(180/PI);// to find angle relative to position
    Serial.println(Angle);
    if((XG-kine.XGlobal)>=0 && (YG-kine.YGlobal)>=0){//top right
      VTheta = 90 - Angle;
    }
    if((XG-kine.XGlobal)<0 && (YG-kine.YGlobal)<0){//bottom left
      VTheta = 180 + (90 - Angle);
    }
    if((XG-kine.XGlobal)>=0 && (YG-kine.YGlobal)<0){//top left
      VTheta = 360 - (90-Angle);
    }
    if((XG-kine.XGlobal)<0 && (YG-kine.YGlobal)>=0){//bottom right
      VTheta = 90 + Angle;
    }
    Serial.println(VTheta);
    Glob = theta_combined*(PI/180);//global theta in degrees between -180 and 180
    Ang = VTheta - Glob;
    if(Ang > 180){
        Ang = -(360 - Ang);
    }
    else if(Ang < -180){
        Ang = 360 + Ang;
    }
    else{}
    Serial.println(Ang);
}


void rotation(float angle_D) {
        compute_new_theta();
        while  (abs(theta_combined-angle_D) > ROTATION_ANGLE_THRESHOLD ){
            kine.locate();
            compute_new_theta();
            if (theta_combined<angle_D) {
                Motors.L_speed = ROTATION_SPEED;
                Motors.R_speed = -ROTATION_SPEED;

                Motors.update_motors();
                    
                }

                else {
                Motors.L_speed = -ROTATION_SPEED;
                Motors.R_speed = +ROTATION_SPEED;

                Motors.update_motors();
                
                }
        }

    Motors.stop_motors();
}


void go_straight_for_distance(float desired_dist, bool reverse){
        kine.locate();
        compute_new_theta();
        int  reverse_sign = 1;
        if (reverse == true){
            reverse_sign = -1;
        }

        float initial_X = kine.XGlobal;
        float initial_Y = kine.YGlobal;
        float initial_Theta = theta_combined * (PI/180);

        
        while (sqrt(sq(kine.XGlobal-initial_X)+sq(kine.YGlobal-initial_Y)) < desired_dist) {
        

        compute_new_theta();
        Motors.L_speed = reverse_sign * FORWARD_SPEED - GO_STRAIGHT_K_p*(theta_combined * (PI/180)-initial_Theta);
        Motors.R_speed = reverse_sign * FORWARD_SPEED + GO_STRAIGHT_K_p*(theta_combined * (PI/180)-initial_Theta);

        Motors.update_motors();
        kine.locate();

        }


        Motors.stop_motors();

    }
