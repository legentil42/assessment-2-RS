// this #ifndef stops this file
// from being included mored than
// once by the compiler. 

#ifndef _ROBOT_ACTIONS_H
#define _ROBOT_ACTIONS_H
#define PI 3.1415926535897932384626433832795
#include "pid.h"
PID_c Left_PID;
PID_c Right_PID;
PID_c Home_angle_PID;
PID_c Home_dist_PID;
class Robot_actions_c {
  public:
  
    #include "motors.h"
    Motors_c Motors;
    #include "linesensor.h"
    LineSensor_c Sensors;
    #include "beep.h"
    Beep_c Buzzer;
    #include "kinematics.h"
    Kinematics_c Kin;
    bool is_on_line = false;
    unsigned long lost_line_time = millis();
    
    
    void go_straight(float theta_goal) {
        Kin.update();
          Sensors.white_calibration();
          Sensors.update_readings();   

        Motors.L_speedo = 30- (Kin.theta-theta_goal)*20;
        Motors.R_speedo = 30+ (Kin.theta-theta_goal)*20;
      
        Motors.update_motors();
        
    }

    void turn_on_spot() {
       
        float old_theta = Kin.theta;
        reset_count();
        while (Kin.theta-old_theta < 3) {
        Motors.L_speedo = 30+ (abs(count_e0)-abs(count_e1))*1;
        Motors.R_speedo = -30- (abs(count_e1)-abs(count_e0))* 1;
        Motors.update_motors();
        Kin.update();
        }
        reset_count();
    }

    void follow_line() {

          Sensors.white_calibration();
          Sensors.update_readings();      
          Motors.L_speedo =  10+Sensors.DN4_NORM*0.5;// + (Sensors.DN3_NORM-100)*0.20;
          Motors.R_speedo =  10+Sensors.DN2_NORM*0.5;//  + (Sensors.DN3_NORM-100)*0.20;
          /*
          Serial.print(Sensors.DN2_VALUE-Sensors.WHITE_MEAN[0]);
          Serial.print(", ");
          Serial.print(Sensors.DN3_VALUE-Sensors.WHITE_MEAN[1]);
          Serial.print(", ");
          Serial.println(Sensors.DN4_VALUE-Sensors.WHITE_MEAN[2]);
          */
          if (Sensors.DN4_NORM> Sensors.DN3_NORM && Sensors.DN4_NORM> Sensors.DN2_NORM) { Motors.R_speedo = -20;}
          if (Sensors.DN2_NORM> Sensors.DN3_NORM && Sensors.DN2_NORM> Sensors.DN4_NORM) { Motors.L_speedo = -20;}
          Motors.update_motors();

    }


    void reset_count() {
        count_e0 = 0;
        count_e1 = 0;
        Kin.old_e1 = 0;
        Kin.old_e0 = 0;
    }


    bool check_for_line() {
      Sensors.update_readings();  
      
        if (Sensors.DN4_VALUE-Sensors.WHITE_MEAN[2]> 600 && Sensors.DN3_VALUE-Sensors.WHITE_MEAN[1]> 600 && Sensors.DN2_VALUE-Sensors.WHITE_MEAN[0]> 600){
            is_on_line = true;
            return true;
        }
        else if (Sensors.DN4_VALUE-Sensors.WHITE_MEAN[2]< 600 && Sensors.DN3_VALUE-Sensors.WHITE_MEAN[1]< 600 && Sensors.DN2_VALUE-Sensors.WHITE_MEAN[0]< 600)
        { 
          if (is_on_line == true) {
                is_on_line = false;
          }
          
                
          return false;}
        
    }

    void go_distance(float desired_distance) {
    Kin.update();
    float old_X = Kin.X;
    float old_Y = Kin.Y;
    float old_pos = sqrt(pow(Kin.X-old_X,2)+pow(Kin.Y-old_Y,2));
    
    
    Home_dist_PID.K_p = 2;
    Home_dist_PID.K_i = 0.0001;
    Home_dist_PID.K_d = 0;
    Home_dist_PID.demand = desired_distance;
    
    Left_PID.K_p = 10;
    Left_PID.K_i = 0.0004;
    Left_PID.K_d = 1;

    Right_PID.K_p = 10;
    Right_PID.K_i = 0.0004;
    Right_PID.K_d = 1;    
    unsigned long t0 = millis();
  while (abs(sqrt(pow(Kin.X-old_X,2)+pow(Kin.Y-old_Y,2))-old_pos-desired_distance) > 10 ) {
    Kin.update();
    if (millis()-t0 > 40){
    t0 = millis();
    
    Home_dist_PID.value =  sqrt(pow(Kin.X-old_X,2)+pow(Kin.Y-old_Y,2));
    Home_dist_PID.calculate_command();


    Left_PID.demand = 0.005*Home_dist_PID.command;
    Right_PID.demand = 0.005*Home_dist_PID.command;

/*
    Serial.print(sqrt(pow(Kin.X-old_X,2)+pow(Kin.Y-old_Y,2)));
    Serial.print(",");
    Serial.print(Home_dist_PID.value);
    Serial.print(",");
    Serial.println(Home_dist_PID.demand);
    */

    
    Left_PID.value = Kin.Ang_speed_e1;
    Left_PID.calculate_command();
    Motors.L_speedo = 2*Left_PID.command;

    Right_PID.value = Kin.Ang_speed_e0;
    Right_PID.calculate_command();
    Motors.R_speedo = 2*Right_PID.command;

    Motors.update_motors();
    
    }
  }
        Motors.L_speedo = 0;
        Motors.R_speedo = 0;
      
        Motors.update_motors();
        
    }
    

    float turn_theta_rad(float desired_theta) {
    Home_angle_PID.sum_of_err = 0;
    Left_PID.sum_of_err = 0;
    Right_PID.sum_of_err = 0;
    Kin.update();
    float old_theta = Kin.theta;
    
    Home_angle_PID.K_p = 3;
    Home_angle_PID.K_i = 0.0005;
    Home_angle_PID.K_d = 0;
    Home_angle_PID.demand = desired_theta;
    
    Left_PID.K_p = 10;
    Left_PID.K_i = 0.001;
    Left_PID.K_d = 2;

    Right_PID.K_p = 10;
    Right_PID.K_i = 0.001;
    Right_PID.K_d = 2;    
    unsigned long t0 = millis();
  while (abs(Kin.theta - desired_theta) > 0.2 ) {
    Kin.update();
    if (millis()-t0 > 40){
    t0 = millis();
    
    Home_angle_PID.value = Kin.theta;
    Home_angle_PID.calculate_command();


    Left_PID.demand = 0.1*Home_angle_PID.command;
    Right_PID.demand = -0.1*Home_angle_PID.command;
/*
    Serial.print(Left_PID.command);
    Serial.print(",");
    Serial.print(Left_PID.demand);
    Serial.print(",");
    Serial.print(Home_angle_PID.value);
    Serial.print(",");
    Serial.println(Home_angle_PID.demand);
    */

    
    Left_PID.value = Kin.Ang_speed_e1;
    Left_PID.calculate_command();
    Motors.L_speedo = 2*Left_PID.command;

    Right_PID.value = Kin.Ang_speed_e0;
    Right_PID.calculate_command();
    Motors.R_speedo = -2*Left_PID.command;

    Motors.update_motors();
    
    }
  }
        Motors.L_speedo = 0;
        Motors.R_speedo = 0;
      
        Motors.update_motors();
        return -0.35+desired_theta;
    }
    void show_pos() {

      Kin.update();
    }
};

#endif
