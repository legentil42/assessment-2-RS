// this #ifndef stops this filese
// from being included mored than
// once by the compiler. SEE IF USEFULL OR NOT
//#ifndef _MOTORS_H
#define _MOTORS_H

#define L_PWM_PIN 10
#define L_DIR_PIN 16
#define R_PWM_PIN  9
#define R_DIR_PIN 15

#define FORWARD 0
#define BACKWARD 1


class Motors_c {
  public:
     float L_speed = 0;
     float R_speed = 0;

    // Constructor, must exist.
    Motors_c() {
        pinMode(6, OUTPUT );
        analogWrite( L_PWM_PIN, 0);
        analogWrite( R_PWM_PIN, 0);
    } 

    void L_go() {

        if (L_speed < 0){
            // Set direction depending on signe of L_speed
            digitalWrite( L_DIR_PIN, BACKWARD );
            }
            
        else{
            digitalWrite( L_DIR_PIN, FORWARD );
            }
            
        if (abs(L_speed) >= 255) { //if value above 255 or under -255, set it to +-255
            L_speed = 255;
            analogWrite( L_PWM_PIN, L_speed);
        }
        else { //else just set the speed
        analogWrite( L_PWM_PIN, abs(L_speed));
        }    
        }



    void R_go() {

        if (R_speed < 0){
            // Set direction depending on signe of L_speed
            digitalWrite( R_DIR_PIN, BACKWARD );
            }
            
        else{
            
            digitalWrite( R_DIR_PIN, FORWARD );
            }

        if (abs(R_speed) >= 255) { //if value above 255 or under -255, set it to +-255
            R_speed = 255;
            analogWrite( R_PWM_PIN, R_speed);
        }
        else{ //else just set the speed
            analogWrite( R_PWM_PIN, abs(R_speed));
        }

        }


    void update_motors(){
        L_go();
        R_go();
    }



    void stop_motors() {
    L_speed = 0;
    R_speed = 0;

    update_motors(); 
    }
};
//same as line 1-3
//#endif
