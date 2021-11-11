class kinematics{
  public:
    float RWidth = 41.5 ;//distance from P
    float WWidth = 16.25;//radius of wheel
    float L_phi,R_phi,XLocal,ThetaC,XGlobal,YGlobal,ThetaGlobal,ThetaD;
    int Range;

    unsigned long T = millis();
    unsigned long PT = millis();
    unsigned long dT;
    volatile long old_count_e1;
    volatile long old_count_e0;

  void locate(){
      PT = millis();
      if (PT-T > 20){

          dT = PT-T;
          L_phi = float((count_e1 - old_count_e1)/358.3);
          R_phi = float((count_e0 - old_count_e0)/358.3); //e0 = Rgiht
          old_count_e1 = count_e1;
          old_count_e0 = count_e0;
          T = millis();

          XLocal = 2.0*PI*WWidth*(R_phi+L_phi)/2;
          ThetaC = 2*PI*(WWidth/(2*RWidth))*(L_phi-R_phi); //e0 : right side
          ThetaGlobal = ThetaGlobal + ThetaC;//in radians
          YGlobal = YGlobal + XLocal * sin(ThetaGlobal);
          XGlobal = XGlobal + XLocal * cos(ThetaGlobal);
          ThetaD = ThetaGlobal*(180/PI);//in degrees
         
          if(ThetaGlobal >6.28319){
            ThetaGlobal = ThetaGlobal - 6.28319;
          }
          else if(ThetaGlobal<-6.28319){
            ThetaGlobal = ThetaGlobal + 6.28319;
          }
          Range = sqrt((XGlobal*XGlobal)+(YGlobal*YGlobal)); 
      }
  }

 
  };
