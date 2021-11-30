class kinematics{
  public:
    float RWidth = 41.5 ;//distance from P
    float WWidth = 16.25;//radius of wheel
    float XGlobal = 0;
    float L_phi,R_phi,XLocal,ThetaC,YGlobal,ThetaGlobal,ThetaD,ThetaInBetweenRangeD;
    int Range;

    unsigned long T = millis();
    unsigned long PT = millis();
    unsigned long dT;
    volatile long old_count_e1;
    volatile long old_count_e0;

  void locate(){
      PT = millis();
      if (PT-T > 15){

          dT = PT-T;
          T = millis();
          L_phi = float((count_e1 - old_count_e1)/358.3);
          R_phi = float((count_e0 - old_count_e0)/358.3); //e0 = Rgiht
          old_count_e1 = count_e1;
          old_count_e0 = count_e0;
          

          XLocal = 2.0*PI*WWidth*(R_phi+L_phi)/2;
          ThetaC = 2*PI*(WWidth/(2*RWidth))*(L_phi-R_phi); //e0 : right side
          ThetaGlobal = ThetaGlobal + ThetaC;//in radians
          YGlobal = YGlobal + XLocal * sin(ThetaGlobal);
          XGlobal = XGlobal + XLocal * cos(ThetaGlobal);
          ThetaD = ThetaGlobal*(180/PI);//in degrees
          ThetaInBetweenRangeD = ThetaD;

          if(ThetaGlobal >PI){
            ThetaInBetweenRangeD = ThetaInBetweenRangeD - 360;
          }
          else if(ThetaGlobal<-PI){
            ThetaInBetweenRangeD = ThetaInBetweenRangeD + 360;
          }

          Range = sqrt((XGlobal*XGlobal)+(YGlobal*YGlobal)); 
      }

/*
    Serial.print(XGlobal);
    Serial.print(",");
    Serial.print(YGlobal);
    Serial.print(",");
    Serial.println(ThetaGlobal);
*/



  }

    void reset_kinematics() {

    XGlobal = 0;
    L_phi =0;

    R_phi=0;
    XLocal= 0;
    ThetaC= 0;
    YGlobal= 0;
    ThetaGlobal= 0;
    ThetaD= 0;
    ThetaInBetweenRangeD= 0;
    Range= 0;

    T = millis();
    PT = millis();
    old_count_e1= 0;
    old_count_e0= 0;
    count_e0= 0;
    count_e1= 0;
    }
 
  };
