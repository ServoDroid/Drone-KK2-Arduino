/* Last updated 2-26-16, changed integral sum from 10 to 100
*				3-3-16, removed (round) from new command return
*
*/
#include "Arduino.h"
#include "PID.h"

 PID::PID()
{
	_Kp = 0; 
	_Ki = 0; //.5
	_Kd = 0; //.5
	//_errorInt[] = 0;
	_error = 0; 
	_iErrorSum = 0; 
	_iCounter = 0; 
	_dError = 0; 
	_newPWMcommand = 0; 
	_oldDerError = 0; 

}

float PID::errorSetpoint(float currVal, float setpoint)
{
  _error = setpoint - currVal; 
  _pError = _Kp * _error; 
  _oldDerError = _error; 
  /*
  if(_iCounter > 5){
  		_iCounter = 0;
  		_iErrorSum = 0;
  }else{
  		_errorInt[_iCounter] = _error; 
  		_iCounter++;
  } 
  */
  return _error; 
  //shift in the current error for integral sum

}

void PID::addIntegral(float intgArray)
{
		_iErrorSum = _Ki * intgArray;	
}

void PID::addIntegral(float intgArray[], int aSize, float iLimit)
{
	//for(int i = 0; i <= 5; i++){
	//	_iErrorSum = _iErrorSum + _errorInt[i];
	//}
	//_iErrorSum = _Ki * _iErrorSum;
	for(int i = 0; i < aSize; i++){
		//_iErrorSum = _iErrorSum + (_Ki * intgArray[i]);
		_iErrorSum = _iErrorSum + intgArray[i];
	}
	_iErrorSum = constrain(_iErrorSum * _Ki, -iLimit, iLimit);
	
}

void PID::calcDerv(float lastErr, float currErr)
{

		_dError = _Kd * (lastErr - currErr); 

}

float PID::getNewCommand()
{
	_newPWMcommand = _pError + _iErrorSum + _dError; 
	return (_newPWMcommand); 
	//return round(_newPWMcommand); 
}

void PID::setKp(float p)
{
	_Kp = p;
}

void PID::setKi(float i)
{
	_Ki = i;
}

void PID::setKd(float d)
{
	_Kd = d;
}

float PID::getKp()
{
	return _Kp; 
}

float PID::getKi()
{
	return _Ki; 
}

float PID::getKd()
{
	return _Kd; 
}

//0,1,3 Kp,Ki,Kd
/*
float PID::getContribute(int contr){
	float error;
    switch(contr){ 
      case 0: { error = _pError; } 
      case 1: { error =  _iErrorSum; }
      case 2: { error =  _dError; }
    } // switch 
    return error; 
} 
*/

float PID::getKpCon()
{
	return _pError;
}

float PID::getKiCon()
{
	return _iErrorSum;
}

float PID::getKdCon()
{
	return _dError;
}