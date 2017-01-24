/*
* Last updated 2-26-16, changed integral sum from 10 to 100
#               9-2-16, added constrain parameter to addIntegral(f,i,f)
*
*/

#ifndef PID_h
#define PID_h

class PID
{
  public:
    PID();
    float errorSetpoint(float,float); 
    void addIntegral(float[], int, float); 
    void addIntegral(float intgArray);
    float getNewCommand();  
    void calcDerv(float, float); 
    void setKp(float);
    void setKi(float);
    void setKd(float);
    float getKp();
    float getKi();
    float getKd();
    //float getContribute(int); //0,1,3 Kp,Ki,Kd
    float getKpCon();
    float getKiCon();
    float getKdCon(); 


  private:
    float _Kp; 
    float _Ki;
    float _Kd;
    float _errorInt[3];
    int _iCounter; 
    //int _kError; 
    float _error; 
    float _pError;
    float _iErrorSum; 
    float _dError; 
    float _sumError; 
    float _newPWMcommand; 
    float _oldDerError; 
    
};


#endif
