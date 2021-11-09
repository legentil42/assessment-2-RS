// this #ifndef stops this filese
// from being included mored than
// once by the compiler. 
//#ifndef _MOTORS_H
#define _MOTORS_H

#define L_PWM_PIN 10
#define L_DIR_PIN 16
#define R_PWM_PIN  9
#define R_DIR_PIN 15

#define FORWARD 0
#define BACKWARD 1


// Class to operate the BUZZER
class Motors_c {
  public:
     float L_speedo = 0;
     float R_speedo = 0;
    // Constructor, must exist.
    Motors_c() {
      pinMode(6, OUTPUT );
    } 

    void L_go() {

        if (L_speedo < 0){
            // Set initial direction for l
            digitalWrite( L_DIR_PIN, BACKWARD );
            }
            
        else{
            digitalWrite( L_DIR_PIN, FORWARD );
            }
            
        if (abs(L_speedo) >= 255) {
            L_speedo = 255 * L_speedo/abs(L_speedo);
            analogWrite( L_PWM_PIN, L_speedo);
        }
        else {
        analogWrite( L_PWM_PIN, int(abs(L_speedo)) % 255 );
        }    
        }



    void R_go() {

        if (R_speedo < 0){
            // Set initial direction for l
            digitalWrite( R_DIR_PIN, BACKWARD );
            }
            
        else{
            // Set initial direction for l
            digitalWrite( R_DIR_PIN, FORWARD );
            }

        if (abs(R_speedo) >= 255) {
            R_speedo = 255 * R_speedo/abs(R_speedo);
            analogWrite( R_PWM_PIN, R_speedo);
        }
        else{
            analogWrite( R_PWM_PIN, int(abs(R_speedo)) % 255);
        }

        }

    void update_motors(){
        L_go();
        R_go();
    }

};



//#endif
