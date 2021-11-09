// this #ifndef stops this file
// from being included mored than
// once by the compiler. 
#ifndef _PID_H
#define _PID_H

// Class to contain generic PID algorithm.
class PID_c {
  
  public:
        float demand;
        float feedback;
        float value;
        float K_p;
        float K_i;
        float K_d;
        float command; 
        float sum_of_err=0;
        float last_feedback = 0;
        float derivate_of_err;
        unsigned long t0 = millis();
        
    // Constructor, must exist.
    PID_c() {

    }

    void calculate_command() {
      feedback = demand - value;
      sum_of_err = sum_of_err+feedback*float(millis() - t0);
      derivate_of_err = (feedback - last_feedback)/float(millis() - t0);
      t0 = millis();
      last_feedback = feedback;
      command = K_p * feedback + K_i*sum_of_err + K_d * derivate_of_err;

    }

};



#endif
