
#include "beep.h"
#include "motors.h"
#include "encoders.h"
#include "kinematics.h"
#include "bumper.h"
#define BAUD_RATE 9600

#define ROTATION_ANGLE_THRESHOLD 1 //degree
#define ROTATION_SPEED 80 //PWM for turning on the spot                                  low : 20 // medium : 60 // HIGH : 110
#define FORWARD_SPEED 60 // PWM for going forward                                        low : 30  // medium : 80 // HIGH : 140
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
    setupEncoder0();
    setupEncoder1();
    delay(500);
    bump.initialise();

    Buzzer.buzz(1911,100);
    Buzzer.buzz(1517,100);
    kine.locate();
    go_to_X_Y(300,0);

    
}


void loop() {
    
    kine.locate();

/*
    bump.readBump();
        Serial.print(bump.L_val);
        Serial.print(",");
        Serial.println(bump.R_val); */


  }

  
    
void rotation(float angle_D) {
        
        while  (abs(kine.ThetaD-angle_D) > ROTATION_ANGLE_THRESHOLD ){
            kine.locate();

            if (kine.ThetaD<angle_D) {
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




  
void follow_direction(float goal_angle_D){

    if (kine.ThetaD>goal_angle_D) {
        Motors.L_speed = FORWARD_SPEED-1;
        Motors.R_speed = FORWARD_SPEED+1;

        Motors.update_motors();
        
    }

    else if (kine.ThetaD<goal_angle_D) {
        Motors.L_speed = FORWARD_SPEED+1;
        Motors.R_speed = FORWARD_SPEED-1;

        Motors.update_motors();
        
    }

    else{
        Motors.L_speed = FORWARD_SPEED;
        Motors.R_speed = FORWARD_SPEED;

        Motors.update_motors();

    }
}



void go_straight_for_distance(float desired_dist, bool reverse){
        kine.locate();
        int  reverse_sign = 1;
        if (reverse == true){
            reverse_sign = -1;
        }

        float initial_X = kine.XGlobal;
        float initial_Y = kine.YGlobal;
        float initial_Theta = kine.ThetaGlobal;

        
        while (sqrt(sq(kine.XGlobal-initial_X)+sq(kine.YGlobal-initial_Y)) < desired_dist) {
        

        
        Motors.L_speed = reverse_sign * FORWARD_SPEED - GO_STRAIGHT_K_p*(kine.ThetaGlobal-initial_Theta);
        Motors.R_speed = reverse_sign * FORWARD_SPEED + GO_STRAIGHT_K_p*(kine.ThetaGlobal-initial_Theta);

        Motors.update_motors();
        kine.locate();

        }


        Motors.stop_motors();

    }

    }




void go_to_X_Y(float X_goal, float Y_goal) {

    kine.locate();
    bump.readBump();

    float goal_angle = atan2(Y_goal-kine.YGlobal,X_goal-kine.XGlobal);

    while (sqrt(sq(X_goal-kine.XGlobal)+sq(Y_goal-kine.YGlobal)) > THRESHOLD_REACH_X_Y) {
        bump.readBump();
        if (bump.L_val >1000 || bump.R_val >1000) {
            float forward_angle = kine.ThetaD;

            go_straight_for_distance(50,true);

            rotation(90+forward_angle);

            go_straight_for_distance(100,false);

            rotation(forward_angle);


        }

        kine.locate();
        goal_angle = atan2(Y_goal-kine.YGlobal,X_goal-kine.XGlobal);

        if (kine.ThetaGlobal-goal_angle>PI) {
            goal_angle += 2*PI;
        }
        if (kine.ThetaGlobal-goal_angle<-PI) {
            goal_angle -= 2*PI;
        }

        Motors.L_speed = FORWARD_SPEED - GO_STRAIGHT_K_p*(kine.ThetaGlobal-goal_angle);
        Motors.R_speed = FORWARD_SPEED + GO_STRAIGHT_K_p*(kine.ThetaGlobal-goal_angle);
    
        Motors.update_motors();


    }



    Motors.stop_motors();
}