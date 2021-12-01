
#include "beep.h"
#include "motors.h"
#include "encoders.h"
#include "kinematics.h"
#include "bumper.h"
#define BAUD_RATE 9600

#define ROTATION_ANGLE_THRESHOLD 1 //degree
#define ROTATION_SPEED 20 //PWM for turning on the spot                                  low : 20 // medium : 30 // HIGH : 40
#define FORWARD_SPEED 30 // PWM for going forward                                        low : 30  // medium : 80 // HIGH : 140
#define GO_STRAIGHT_K_p 20 //20 before //K_p for the P controller of go straight dist function          low 20 med 40 high 60?
#define ROTATION_K_p 0.4 //K_p for the P controller of go straight dist function
#define THRESHOLD_REACH_X_Y 20

Beep_c Buzzer;
Motors_c Motors;
kinematics kine;
BumpSensor_c bump;

float XG,YG;
int VTheta;
float Glob, Ang;

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

    //Buzzer.buzz(1911,100);
    //Buzzer.buzz(1517,100);
    kine.locate();//calculte where you are
    turn();
    go_to_X_Y(1000,0);//in millimeter
    /*
    go_to_X_Y(2000,1000);
    go_to_X_Y(3000,1000);
    go_to_X_Y(3000,-1000);
    go_to_X_Y(4000,-1000);
    go_to_X_Y(4000,0);
    go_to_X_Y(6000,0);
*/
    
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
    
    Calculate();
    //rotation(Ang);
    turn();

    float goal_angle = atan2(Y_goal-kine.YGlobal,X_goal-kine.XGlobal);//find goal angle
    while (range > THRESHOLD_REACH_X_Y) {//while distance from goal is bigger than threshold
        bump.readBump();
        if (bump.L_val >1000 || bump.R_val >1000) {//generic bump behaviour
            float forward_angle = kine.ThetaD;
            go_straight_for_distance(50,true);
            rotation(90+forward_angle);
            go_straight_for_distance(100,false);
            rotation(forward_angle);
        }

        kine.locate();//update position
        goal_angle = atan2(Y_goal-kine.YGlobal,X_goal-kine.XGlobal);//update angle
        range = sqrt(sq(X_goal-kine.XGlobal)+sq(Y_goal-kine.YGlobal));
        SRange = sqrt(sq(OldPosX-kine.XGlobal)+sq(OldPosY-kine.YGlobal));

        if (kine.ThetaGlobal-goal_angle>PI) {//set goal angle to between -180 and 180
            goal_angle += 2*PI;
        }
        if (kine.ThetaGlobal-goal_angle<-PI) {
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
    Glob = kine.ThetaInBetweenRangeD;//global theta in degrees between -180 and 180
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

/*
 * if(range < Elimit){
            VAL = (FORWARD_SPEED) * (range/(limit*2.5));
            if(VAL < 15){
              VAL = 15;
            }
        }
        else{}
        
        
        if(SRange < limit){
            VAL = ((FORWARD_SPEED) * (SRange/limit));
            if(VAL < 15){
              VAL = 15;
            }
        }
        else{}

        if(range > Elimit && SRange > limit){
          VAL = FORWARD_SPEED;
        }
        else{}

        
        Motors.L_speed = (VAL) - GO_STRAIGHT_K_p*(kine.ThetaGlobal-goal_angle);
        Motors.R_speed = (VAL) + GO_STRAIGHT_K_p*(kine.ThetaGlobal-goal_angle);
        Motors.update_motors();
 */
