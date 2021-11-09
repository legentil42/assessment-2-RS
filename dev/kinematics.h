/// this #ifndef stops this file
// from being included mored than
// once by the compiler. 

#define _KINEMATICS_H
#define PI 3.1415926535897932384626433832795

// Class to track robot position.
class Kinematics_c {
  public:
        float L = 44.6 ;
        float r = 16.0;
        float circum = 2*r*3.1415;
        float phi_e1;
        float phi_e0;
        float Xpr;
        float Ypr;
        float thetapr;
        float X=0;
        float Y=0;
        float theta=0;
        float Ang_speed_e0;
        float Ang_speed_e1;
        
        unsigned long t0 = millis();
        unsigned long t1 = millis();
        unsigned long Dt;
        volatile long old_e1;
        volatile long old_e0;
    // Constructor, must exist.
    Kinematics_c() {

    } 

    // Use this function to update
    // your kinematics
    void update() {
          if (millis()-t0 > 20){
          Dt = millis()-t0;
          phi_e1 = float((count_e1 - old_e1)/358.3);
          phi_e0 = float((count_e0 - old_e0)/358.3);

          Ang_speed_e0 = float((count_e0 - old_e0)/358.3)/(Dt/1000.0);
          Ang_speed_e1 = float((count_e1 - old_e1)/358.3)/(Dt/1000.0);
          /*
          Serial.print(count_e0);
          Serial.print(", ");
          Serial.print(count_e1);
          Serial.print(", ");
          */
          old_e1 = count_e1;
          old_e0 = count_e0;
          t0 = millis();

          Xpr = 2.0*PI*r*(phi_e0+phi_e1)/2;
          Ypr = 0;

          thetapr = 2*PI*(r/(2*L))*(phi_e1-phi_e0); //e0 : right side




          theta = theta + thetapr;      //*(Dt/1000.0);
          Y = Y + Xpr * sin(theta);       //*(Dt/1000.0);
          X = X + Xpr * cos(theta);       //*(Dt/1000.0);

          

          Serial.print(X);
          Serial.print(", ");
          Serial.print(Y);
          Serial.print(", ");
          Serial.println(theta);
          
          }
    }

};
