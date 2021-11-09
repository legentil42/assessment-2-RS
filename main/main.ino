
#include "beep.h"
#include "motors.h"
#include "encoders.h"
#include "robot_actions.h"
#define BAUD_RATE 9600

#define PI 3.1415926535897932384626433832795
Beep_c Buzzer;
Robot_actions_c Actions;
Motors_c Motors;


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

}

void loop() {
    
  }
    
    
  
