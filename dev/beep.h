// this #ifndef stops this file
// from being included mored than
// once by the compiler. 
#ifndef _BEEP_H
#define _BEEP_H


// Class to operate the BUZZER
class Beep_c {
  public:
  
    // Constructor, must exist.
    Beep_c() {
      pinMode(6, OUTPUT );
    } 

    void buzz (float pitch, int duree) {
      
      unsigned long TT_duree = millis();
      unsigned long TT_pitch = micros();

      while (millis() - TT_duree < duree) {
                if (micros()-TT_pitch > pitch) {
                digitalWrite(6,!digitalRead(6));
                TT_pitch = micros();
                        }
        }
        digitalWrite(6,0);
        delay(duree/8);
}




};



#endif
