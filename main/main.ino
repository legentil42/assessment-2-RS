
#include "beep.h"
#include "motors.h"
#include "encoders.h"
#include "kinematics.h"
#define BAUD_RATE 9600

#define ROTATION_ANGLE_THRESHOLD 1 //degree
#define ROTATION_SPEED 20 //PWM for turning on the spot
#define FORWARD_SPEED 30 // PWM for going forward
#define GO_STRAIGHT_K_p 50 //K_p for the P controller of go straight dist function
#define ROTATION_K_p 0.4 //K_p for the P controller of go straight dist function


Beep_c Buzzer;
Motors_c Motors;
kinematics kine;

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
    Buzzer.buzz(1911,100);
    Buzzer.buzz(1517,100);
    kine.locate();
    rotation(90);
    
    go_straight_for_distance(200);
    rotation(180);
    go_straight_for_distance(200);
    rotation(270);
    go_straight_for_distance(200);
    rotation(360);
    go_straight_for_distance(200);
    
}


void loop() {
    
    kine.locate();
    //follow_direction_2(90);


  }
    
void rotation(float angle_D) {
    /*
    if (angle_D>180){
        angle_D += -360;
    }
    if (angle_D<-180){
        angle_D += 360;
    }
*/
        
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

void go_straight_for_distance(float desired_dist){
        kine.locate();
        float initial_X = kine.XGlobal;
        float initial_Y = kine.YGlobal;
        float initial_Theta = kine.ThetaGlobal;
        
        while (sqrt(sq(kine.XGlobal-initial_X)+sq(kine.YGlobal-initial_Y)) < desired_dist) {

        Motors.L_speed = FORWARD_SPEED-GO_STRAIGHT_K_p*(kine.ThetaGlobal-initial_Theta);
        Motors.R_speed = FORWARD_SPEED+GO_STRAIGHT_K_p*(kine.ThetaGlobal-initial_Theta);

        Motors.update_motors();
        kine.locate();

        }


        Motors.stop_motors();

    }
