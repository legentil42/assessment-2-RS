// this #ifndef stops this file
// from being included mored than
// once by the compiler. 
#ifndef _BUMPSENSOR_H
#define _BUMPSENSOR_H


// Class to operate the linesensor(s).
class BumpSensor_c {
  public:

    // Constructor, must exist.
    BulpSensor_c() {

    }
    #define BUMP_SENSOR_L 4
    #define BUMP_SENSOR_R 5
    #define EMIT 11
    #define NUMBER_SENSORS 1
    #define time_out_val 3000

    float e_line;
    float W_right;
    float W_left;
    float L_val;
    float R_val;
    float min_white[3]={3000,3000,3000};


    // Use this function to 
    // initialise the pins and 
    // state of your motor(s).
    void initialise() {
      pinMode(EMIT, OUTPUT);
      pinMode(BUMP_SENSOR_L,INPUT);
      pinMode(BUMP_SENSOR_R,INPUT);

      Serial.begin(9600);
      delay(1000);
      Serial.println("**RESET**");

      digitalWrite(EMIT,LOW);
    }

    void readBump(){
      pinMode( BUMP_SENSOR_L, OUTPUT );
      digitalWrite( BUMP_SENSOR_L, HIGH );
      pinMode( BUMP_SENSOR_R, OUTPUT );
      digitalWrite( BUMP_SENSOR_R, HIGH );

      // Tiny delay for capacitor to charge.
      delayMicroseconds(10);

      //  Turn input pin back to an input
      pinMode( BUMP_SENSOR_L, INPUT );
      pinMode( BUMP_SENSOR_R, INPUT );

      // Places to store microsecond count
      unsigned long start_time; // t_1
      unsigned long end_time[NUMBER_SENSORS];   // t_2


      // Store current microsecond count
      start_time = micros();

      // Calculate elapsed time
      //unsigned long elapsed_time[NUMBER_SENSORS];
      unsigned long time_out;

      bool flag_L = LOW;
      bool flag_R = LOW;


      // Stay in a loop whilst the capacitor
      // is still registering as "HIGH".
      while(flag_L == LOW || flag_R == LOW) {
        time_out=micros();
        if(time_out-start_time > time_out_val){
          Serial.println("**ABORT**");
          break;
        }   
        if( digitalRead( BUMP_SENSOR_L ) == LOW){
          if(flag_L == LOW){
            flag_L = HIGH;
            end_time[0]=micros();
          }
        }
        if( digitalRead( BUMP_SENSOR_R ) == LOW){
          if(flag_R == LOW){
            flag_R = HIGH;
            end_time[1]=micros();
          }
        }
      }

      L_val = end_time[0] - start_time;
      R_val = end_time[1]- start_time;
    }

};



#endif