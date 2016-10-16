/*
    TODO:
                -SYSTEM checkout upon boot (sensors, etc)
                -RC Transmitter watchdog
                -Additional sensors for atitude
                  --IMU for roll/pitch/yaw
                  --uSonic sensors to maintain distance all around

9/24/16    1.8.5.2
9/24/16     1.8.5.1 -__****Kd of 10 for S4 tested and wors, check youtube and KST vid
                    -tunned for all batteries  
                    -accidently tested s4 battery with s3s and worked, but did not do endurance test           
9/22/16    1.8.5.0 will try and compensate for roll/pitch 
                    --did not get compesnation to work, 
                    --****WAS ABLE TO CAPTURE S3S/S3B PARAMETERS!  
                    
9/14/16    1.8.4.2-Altworks-PC --9/24 works good enough with S4 battery basehover set by volt, and gains are ok. 
                            
                    -modification to ping recieved. now pass ping status and set to zero in Alt()
                    -moved new_lift_thrust calculation inside if(ping) so it only updates when there is a ping
9/9/16    1.8.4.2   -tweeked gains with s4 battery, more aggressive KdAlt and KiAlt
                  -modified basehover to be proportional to battery voltage (untested)
                  -dynamic gains and hover

8/18/16   1.8.2.0  --would like to test this with small s3 battery....and with voltage comp.
                   --Set alt hold position based on height when switched
                  -currAlt
                  -return zero when checking flightmode switch if out of range
                  -update from 1060 to 1040 and 1120 to 1170 for position 1
                  -update from 1460 to 1440 and 1520 to 1570 for position 2
8/15/16   1.8.1.3 WORKS! - 3s bat PID(.002,.001,0 without cable)(.004.001 with cable) CSV print, print 200ms, clean up and removed checkflightmode again,

     */
#include <Servo.h>
//#include "SRF05.h"
#include "PID.h"
#include <Arduino.h>
#include <NewPing.h>
#include <Wire.h>

#include <Adafruit_Sensor.h>
#include <Adafruit_LSM303_U.h>
#include <Adafruit_BMP085_U.h>
#include <Adafruit_L3GD20_U.h>
#include <Adafruit_10DOF.h>

/* Assign a unique ID to the sensors */
Adafruit_10DOF                dof   = Adafruit_10DOF();
Adafruit_LSM303_Accel_Unified accel = Adafruit_LSM303_Accel_Unified(30301);
Adafruit_LSM303_Mag_Unified   mag   = Adafruit_LSM303_Mag_Unified(30302);
Adafruit_BMP085_Unified       bmp   = Adafruit_BMP085_Unified(18001);

/* Update this with the correct SLP for accurate altitude measurements */
float seaLevelPressure = SENSORS_PRESSURE_SEALEVELHPA;


    sensors_event_t accel_event;
  sensors_event_t mag_event;
  sensors_event_t bmp_event;
  sensors_vec_t   orientation;
  float comp_uSonic_bottom = 0; 

  const float pi = 3.14159267;

float todegrees(float radians) {
  return (radians / 2 / pi * 360);
}
float toradians(float degrees) {
  return degrees / 360 * 2 * pi;
}

/**************************************************************************/
/*!
    @brief  Initialises all the sensors used by this example
*/
/**************************************************************************/
void initSensors()
{
  if(!accel.begin())
  {
    /* There was a problem detecting the LSM303 ... check your connections */
    Serial.println(F("Ooops, no LSM303 detected ... Check your wiring!"));
    while(1);
  }
  if(!mag.begin())
  {
    /* There was a problem detecting the LSM303 ... check your connections */
    Serial.println("Ooops, no LSM303 detected ... Check your wiring!");
    while(1);
  }
  if(!bmp.begin())
  {
    /* There was a problem detecting the BMP180 ... check your connections */
    Serial.println("Ooops, no BMP180 detected ... Check your wiring!");
    while(1);
  }
}


float roll,pitch, heading, iroll, ipitch,iheading; 
   
#define TRIGGER_PIN   49  // trigger pin on ping sensor
#define ECHO_PIN      49  // cho pin on ping sensor
#define FTRIGGER_PIN  48  // front trigger pin
#define FECHO_PIN     46  // front echo pin 

#define SONAR_NUM     2 // Number or sensors.
#define MAX_DISTANCE 300 // 645 for MB1040 EZ4. Maximum distance we want to ping for (in centimeters). Maximum sensor distance is Altd at 400-500cm. Ping is rated for 300cm


bool front_trig = 0;  
float KpAlt = 0; // 0.3; //.005 was past good, .002, .0005 for s4
float KiAlt = 0; 
float KdAlt = 0; 
float basehover = 0;
float pid_cmd_Alt = 0;
float currAltCm = 0; 
float lastDerr = 0; 
float currDerr = 0; 
int temp = 0; 
 
float vCompRange = .05; // not used. //1.5, min = .55, max = 1.25
float Altset = 0;
float test;
float vComp = 0;
int voltagePin = 14;       //analog voltage measurement
int volt;// = 0;
 
int dt_ping, dt_ping_sec;
const int numReadings = 3;
byte readIndex = 0;              // the index of the current reading
float intgrAltError[numReadings];
float uSonic[numReadings];      // currently not used
float pass_lift_thrust = 0;
bool airBorn = 0;
 
unsigned long timer[7];
byte last_channel[6];
int input[6];
 
/*
 * The pulseIn Function 
 * RCV off values 968 - 1496 - 1492 - 1492 - 0 - 0
 * Throttle down 1088 - 1496 - 1496 - 1496 - 0 - 0
 */
int PWM_PIN_THROTTLE = 53;         //min 1081, neutral 1114, max 1908
int PWM_PIN_ROLL = 52;             //min 1917, neutral 1519, max 1115
int PWM_PIN_PITCH = 51;            //min 1962, neutral 1564, max 1160
int PWM_PIN_YAW = 50;              //min 1894, neutral 1496, max 1092
int PWM_PIN_GEAR = 10;  //FM       //min 1092, neutral 1493, max 1894
int PWM_PIN_PANIC = 11;            //min 1092, neutral 1893, max 1895
 
int PWM_POUT_THROTTLE = 7; // 3;//22;
int PWM_POUT_ROLL = 6;      //ali
int PWM_POUT_PITCH = 5;     //elev
int PWM_POUT_YAW = 4;       //rudd
int PWM_POUT_GEAR = 3; //fm //
int PWM_POUT_PANIC = 2;
 
 float pwm_value_throttle, pwm_value_roll, pwm_value_pitch, pwm_value_pitch_front, pwm_value_yaw, pwm_value_gear, pwm_value_panic, flight_mode_switch;

/*
 * Servo output to motors
 */
Servo servo_throttle, servo_roll, servo_pitch, servo_yaw, servo_gear, servo_panic;
 
 
//*****SENSORS********************************************//
 
/*
 * New uSonic
 */
unsigned long pingTimer[SONAR_NUM]; // When each pings.
unsigned int cm[SONAR_NUM]; // Store ping distances.
uint8_t currentSensor = 0; // Which sensor is active.

NewPing sonar[SONAR_NUM] = { // Sensor object array.
  NewPing(TRIGGER_PIN, ECHO_PIN, MAX_DISTANCE),
  NewPing(FTRIGGER_PIN, FECHO_PIN, MAX_DISTANCE),
};
//NewPing sonar(TRIGGER_PIN, ECHO_PIN, MAX_DISTANCE); // NewPing setup of pins and maximum distance.
//NewPing sonarfront(FTRIGGER_PIN, FECHO_PIN, MAX_DISTANCE); // NewPing setup of pins and maximum distance.
unsigned int pingSpeed = 33; // 70 works for EZ sensor. How frequently are we going to send out a ping (in milliseconds). 50ms would be 20 times a second.
//unsigned long pingTimer;     // Holds the next ping time.
float uSonic_bottom, old_uSonic_bottom, uSonic_front, old_uSonic_front;
int ping_received, ping_received_front;
int old_ping_time, new_ping_time, old_ping_time_front, new_ping_time_front;
 
/*
 * PID
 */
PID pidAlt;
float old_lift_thrust = 0;
float new_lift_thrust = 0;
float KpContribR, KiContribR, KdContribR;
 
uint32_t print_timer = millis();
uint32_t base_timer = millis(); 
 
//*****SETUP*****************************************************************************************************//
void setup() {
 //pinMode(voltagePin, INPUT);
   /* Initialise the sensors */
  initSensors();


  /* Read the accelerometer and magnetometer */
  accel.getEvent(&accel_event);
  mag.getEvent(&mag_event);

if(dof.fusionGetOrientation(&accel_event, &mag_event, &orientation)){
  iroll = orientation.roll; 
  ipitch = orientation.pitch; 
  iheading = orientation.heading;  
}

  pingTimer[0] = millis() + 75; // First ping start in ms.
  for (uint8_t i = 1; i < SONAR_NUM; i++)
    pingTimer[i] = pingTimer[i - 1] + pingSpeed;
    
  // Initialize arrays
  for (int thisReading = 0; thisReading < numReadings; thisReading++) {
    intgrAltError[thisReading] = 0;  //change from numReadings to thisReading 8/25/16
    uSonic[thisReading] = 0;  //currently not used.
  }
 
  //** Interrupt for RC receiver
  PCICR  |= (1 << PCIE0);
  PCMSK0 |= (1 << PCINT0);
  PCMSK0 |= (1 << PCINT1);
  PCMSK0 |= (1 << PCINT2);
  PCMSK0 |= (1 << PCINT3);
  PCMSK0 |= (1 << PCINT4);
  PCMSK0 |= (1 << PCINT5);
 
  //** PWM input pin
  pinMode(PWM_PIN_THROTTLE, INPUT);
  pinMode(PWM_PIN_ROLL, INPUT);
  pinMode(PWM_PIN_PITCH, INPUT);
  pinMode(PWM_PIN_YAW, INPUT);
  pinMode(PWM_PIN_GEAR, INPUT);
  pinMode(PWM_PIN_PANIC, INPUT);
 
  //** PWM output pin
  pinMode(PWM_POUT_THROTTLE, OUTPUT);
  pinMode(PWM_POUT_ROLL, OUTPUT);
  pinMode(PWM_POUT_PITCH, OUTPUT);
  pinMode(PWM_POUT_YAW, OUTPUT);
  pinMode(PWM_POUT_GEAR, OUTPUT);
  pinMode(PWM_POUT_PANIC, OUTPUT);
 
  //**attach servos
  servo_throttle.attach(PWM_POUT_THROTTLE);
  servo_roll.attach(PWM_POUT_ROLL);
  servo_pitch.attach(PWM_POUT_PITCH);
  servo_yaw.attach(PWM_POUT_YAW);
  servo_gear.attach(PWM_POUT_GEAR);
  servo_panic.attach(PWM_POUT_PANIC);
  
  Serial.begin(57600);    // in case I want to print to arduino's serial
  Serial2.begin(57600);   // 3DR telemetry kit default baudrate is 57600, Tx/Rx reversed
  delay(2000);
 
  //** Print header
  Serial2.println("ready");
  Serial2.println("FM, Altset, uSonic_bottom, new_lift_thrust, KpContribR, KiContribR, KdContribR, pwm_value_throttle, Kp,Ki,Kd,dVolt, est batt");

  // New ping lib
//  pingTimer = millis(); // Start now. 
 
} //setup
 
/*
 * Interupt subroutine, this interupt subreoutine was modified to allow reading of multiple channels. 
 * Example from another YouTuber's code. Sorry I forgot your name. 
 */
ISR(PCINT0_vect) {
  timer[0] = micros();
  // channel 1 ---------------
  if (last_channel[0] == 0 && PINB & B00000001 ) {
    last_channel[0] = 1;
    timer[1] = timer[0];
  }
  else if (last_channel[0] == 1 && !(PINB & B00000001) ) {
    last_channel[0] = 0;
    input[0] = timer[0] - timer[1];
    pwm_value_throttle = input[0];
  }
 
  // channel 2 ---------------
  if (last_channel[1] == 0 && PINB & B00000010 ) {
    last_channel[1] = 1;
    timer[2] = timer[0];
  }
  else if (last_channel[1] == 1 && !(PINB & B00000010) ) {
    last_channel[1] = 0;
    input[1] = timer[0] - timer[2];
    pwm_value_roll = input[1];
  }
 
  // channel 3 ---------------
  if (last_channel[2] == 0 && PINB & B00000100 ) {
    last_channel[2] = 1;
    timer[3] = timer[0];
  }
  else if (last_channel[2] == 1 && !(PINB & B00000100) ) {
    last_channel[2] = 0;
    input[2] = timer[0] - timer[3];
    pwm_value_pitch = input[2];
  }
 
  // channel 4 ---------------
  if (last_channel[3] == 0 && PINB & B00001000 ) {
    last_channel[3] = 1;
    timer[4] = timer[0];
  }
  else if (last_channel[3] == 1 && !(PINB & B00001000) ) {
    last_channel[3] = 0;
    input[3] = timer[0] - timer[4];
    pwm_value_yaw = input[3];
  }
  // channel 5 ---------------
  if (last_channel[4] == 0 && PINB & B00010000 ) {
    last_channel[4] = 1;
    timer[5] = timer[0];
  }
  else if (last_channel[4] == 1 && !(PINB & B00010000) ) {
    last_channel[4] = 0;
    input[4] = timer[0] - timer[5];
    flight_mode_switch = input[4];
  }
  // channel 6 ---------------
  if (last_channel[5] == 0 && PINB & B00100000 ) {
    last_channel[5] = 1;
    timer[6] = timer[0];
  }
  else if (last_channel[5] == 1 && !(PINB & B00100000) ) {
    last_channel[5] = 0;
    input[5] = timer[0] - timer[6];
    pwm_value_gear = input[5];
  }
}
 
//****************************************************
//****************************************************
//****************************************************
//**Functions****************************************
//****************************************************
//****************************************************
 
 
// Used in manul mode to pass thru values from receiver to flight controller
void pass_thru() {
  servo_throttle.write(pwm_value_throttle);
  servo_roll.write(pwm_value_roll);
  servo_pitch.write(pwm_value_pitch);
  servo_yaw.write(pwm_value_yaw);
  servo_gear.write(flight_mode_switch);
  servo_panic.write(pwm_value_gear);
}
 
// Used by Alt/position loops to pass new throttle commands to flight controller, yaw/pitch/roll remain manual
//void pass_thru_altitude(float thrust, float pitch) {
void pass_thru_altitude(float thrust, float pitch) {
  servo_throttle.write(thrust);
  servo_roll.write(pwm_value_roll);
  //servo_pitch.write(pwm_value_pitch);
    servo_pitch.write(pitch);

  servo_yaw.write(pwm_value_yaw);
  servo_gear.write(flight_mode_switch);
  servo_panic.write(pwm_value_gear);
}
 
 

 
int Check_flight_mode() {
  if (flight_mode_switch > 1040 && flight_mode_switch < 1170) {
    return 1;
  }
  if (flight_mode_switch > 1440 && flight_mode_switch < 1570) {
    return 2;
  }
  //return 0; // this means it is out of range and dropped
}
 

 
void AltLoop(float throt,  int Alt, int ping) {

  /*
   * Throttle deadband settings and altitude adjustments. Altitude in cm. 
   */
  int throtDeadBand = 1500 * .20;
  int throtUpDB = 1500 + throtDeadBand;
  int throtDownDB = 1500 - throtDeadBand;
  int lowAlt = 10;
  int hiAlt = 180;
 
  if (pwm_value_throttle > throtUpDB ) {
    Altset = constrain(map(pwm_value_throttle,throtUpDB,1900,Alt,hiAlt), Alt, hiAlt); // hiAlt;
  }
  if (pwm_value_throttle < throtDownDB) {
    Altset = constrain(map(pwm_value_throttle,1084, throtDownDB,Altset,lowAlt), Alt, lowAlt); lowAlt;
  }
  if (pwm_value_throttle > throtDownDB && pwm_value_throttle < throtUpDB) {
    //Altset = Alt; // 0;
  }
 

 /*
  * Used to enable or disable appropriate gains. Used during testing and tunning. 
  */
  float Kpf = 1;
  float Kif = 1;
  float Kdf = 1;
 
 /*
  * Voltage compensation. Need to finish
  */
  vComp = 0;

  /*
   * Set gains for pid.
   */
  pidAlt.setKp(KpAlt + vComp); //.25
  pidAlt.setKi(KiAlt * Kif);  //consider turning off KiAlt if the error is large to prevent OS
  pidAlt.setKd(KdAlt * Kdf);// + (Kdf * vComp));
 
  /*
   * Set thrust, basehover, and altitude on first switchover to altitude hold.
   * Set thrust/basehover to current throttle when switch was made. 
   * Set altitude of current height. 
   */

  int s4hiVolt = 858; //900 good s4//799
  int s4loVolt = 550;
  int s4hiCmd = 1615;
  int s4loCmd = 1470; //1490 init
  int s4constrainLo = 1450; 
  int s4constrainHi = 1630; 

  int s3bhiVolt = 630; //799
  int s3bloVolt = 555;
  int s3bhiCmd = 1705;
  int s3bloCmd = 1580;
  int s3bconstrainLo = 1580; 
  int s3bconstrainHi = 1670; 

  int s3shiVolt = 630; //799
  int s3sloVolt = 555;
  int s3shiCmd = 1610;
  int s3sloCmd = 1540;
  int s3sconstrainLo = 1530; 
  int s3sconstrainHi = 1650;
/*
 * 
 *      *  min throt to lift (to hover) = 1544
     *  max throt to lift (to hover) = 1610
     *  delta pulse = 66
     *  dv = 1 - 227
     *  dv/pulse = 227/66 = 3.42
     *  
     *  
  full s3 big = 3.152 
      ...dolt = 628, volt = 12.46
      ...ratio 12.46/628 = .01984

           *  min = 1580
     *  max = 1664
*/  
     

  if (!airBorn) {
    airBorn = 1;
    new_lift_thrust = throt;   
    
    Altset = Alt; 
   // volt = analogRead(voltagePin); 
      pass_lift_thrust = new_lift_thrust; 
      pwm_value_pitch_front = pwm_value_pitch;
    if(volt > 0){
      basehover = constrain(map(volt, s4hiVolt, s4loVolt, s4loCmd, s4hiCmd), s4constrainLo,s4constrainHi); //for s4
      //basehover = constrain(map(volt, s3shiVolt, s3sloVolt, s3sloCmd, s3shiCmd), s3sconstrainLo,s3sconstrainHi); //for s4
      //basehover = constrain(map(volt, s3bhiVolt, s3bloVolt, s3bloCmd, s3bhiCmd), s3bconstrainLo,s3bconstrainHi); //for s4
    }else{
      basehover = throt;
    }
  }

  /*     Very low battery: 1617 to lift at 10.9v at 551

    Full bat first flight: 789, 15.6V, 1512
   *  Remember past thrust value. 
   */


  /*
   * Upon receipt of a ping: 
   *    Calculate error, store error in array, remember last error, adjust the gains based on magnitude of error, 
   *    calculate each contribution of gains to the command, send command, and update index for array. 
   */
  if (ping) {
    int volt_past = volt; 
   // volt = analogRead(voltagePin); 
    //volt = (volt_past + vtemp) / 2;
    
    
    if (millis() - base_timer > 1000) {
      //volt = analogRead(voltagePin); 
      base_timer = millis();
      if(volt > 0 && volt < 1000){
        basehover = constrain(map(volt, s4hiVolt, s4loVolt, s4loCmd, s4hiCmd), s4constrainLo,s4constrainHi); //for s4
        //basehover = constrain(map(volt, s3shiVolt, s3sloVolt, s3sloCmd, s3shiCmd), s3sconstrainLo,s3sconstrainHi); //for s4
        //basehover = constrain(map(volt, s3bhiVolt, s3bloVolt, s3bloCmd, s3bhiCmd), s3bconstrainLo,s3bconstrainHi); //for s4
      }
    }

  
  /*
   * If object in front is < 140 cm, move back until clear
   */
  if(uSonic_front <  140.0 && uSonic_front > 0){
    pwm_value_pitch_front = 1600;//pwm_value_pitch_front - 5;
    pass_thru_altitude(new_lift_thrust, pwm_value_pitch_front);
  }else
    pwm_value_pitch_front = pwm_value_pitch;
  
  
    
  pidAlt.errorSetpoint(uSonic_bottom, Altset); // Setpoint and calc KpAlt. PID for KpAlt is (setpoint - current value)
  /*
   *  Dynamic Gain
   *  If error is large, use larger gains (0.xxx). If low, use smaller gains (.00xxx). 
   *  
   */

   intgrAltError[readIndex] = Altset - uSonic_bottom;
   lastDerr = currDerr;
   currDerr = Altset - uSonic_bottom;

    
  float intLimit = 0; 
  float hoverlimit = 0;
  
  // try 2:1 ratio for Kp and Ki
  /*
   * Adjust the gains based on the magantide of the error. 
   */

/*
  if(abs(currDerr) > 20){
    KpAlt = .0005; 
    KiAlt = 0;//0.0005;
    KdAlt = .005;
    intLimit = .5; 
    hoverlimit = 20;// can be as tight as 3?
   
  }else{           //  These comments where good!  
    KpAlt = .005;//.004; // .005
    KiAlt = .0005; // .001
    KdAlt = 3;  // 1.5 
    intLimit = .25;// .05 
    hoverlimit = 10; //15 
   }
*/
    KpAlt = .5; //.5//.004; // .005, .5 s3s
    KiAlt = .005; // .001
    KdAlt = 10;  // 1.5 for S4, 3 for s3b, 10 s3s
    intLimit = .1;// .05 
    hoverlimit = 20; //20 was ok //15 for s4, , 20 for s3s
  int tempLimit = 0; 
  
  pidAlt.setKp(KpAlt); 
  pidAlt.setKi(KiAlt);
  pidAlt.setKd(KdAlt);
  
  pidAlt.addIntegral(intgrAltError, numReadings, intLimit);            
  pidAlt.calcDerv(currDerr, lastDerr);           
//  pidAlt.calcDerv(lastDerr,currDerr );           

  // Get contribution of gains for print
  KpContribR = pidAlt.getKpCon();      
  KiContribR = pidAlt.getKiCon();
  KdContribR = pidAlt.getKdCon();

  // Get new command
  pid_cmd_Alt = pidAlt.getNewCommand();

  new_lift_thrust = constrain(pass_lift_thrust + pid_cmd_Alt, basehover - hoverlimit, basehover + hoverlimit); //1800 max throt of ~80%,
  ping_received = 0;
  readIndex = readIndex + 1;
  if (readIndex > numReadings) {
    readIndex = 0;
  }//if
 }//ping
 
//  pass_thru_altitude(new_lift_thrust);
    pass_thru_altitude(new_lift_thrust, pwm_value_pitch_front);

}//Alt hold

/*
 *  Check if ping received. If so, store how long it took, remember last distance, update distance. 
 */
void echoCheck() { // Timer2 interrupt calls this function every 24uS where you can check the ping status.
 // if (sonar.check_timer()) {
 if (sonar[currentSensor].check_timer())
    cm[currentSensor] = sonar[currentSensor].ping_result / US_ROUNDTRIP_CM;
    if(currentSensor == 0){
       old_uSonic_bottom = uSonic_bottom;
        uSonic_bottom = cm[currentSensor]; 
    }
    if(currentSensor == 1){
      old_uSonic_front = uSonic_front;
      uSonic_front = cm[currentSensor]; 
      
    }
    //old_ping_time = new_ping_time;
   // new_ping_time = millis();
    
   // uSonic_bottom = (sonar.ping_result / US_ROUNDTRIP_CM); // Ping returned, uS result in ping_result, convert to cm with US_ROUNDTRIP_CM.
    ping_received = 1;
    //currAltCm = uSonic_bottom;

  //}else {}
 
  // Don't do anything here!
}

void loop() {
  for (uint8_t i = 0; i < SONAR_NUM; i++) {
    if (millis() >= pingTimer[i]) {
      pingTimer[i] += pingSpeed * SONAR_NUM;
      if (i == 0 && currentSensor == SONAR_NUM - 1){
//        oneSensorCycle(); // Do something with results.
      }
      sonar[currentSensor].timer_stop();
      currentSensor = i;
      cm[currentSensor] = 0;
      sonar[currentSensor].ping_timer(echoCheck);
    }
  }


    comp_uSonic_bottom = uSonic_bottom * cos((toradians(pitch))); 

  /* Read the accelerometer and magnetometer */
  accel.getEvent(&accel_event);
  mag.getEvent(&mag_event);

if(dof.fusionGetOrientation(&accel_event, &mag_event, &orientation)){
  pitch = iroll - orientation.roll; 
  roll = ipitch -orientation.pitch; 
  heading = iheading - orientation.heading;  
}
/*
 * Check ping.
 */
/* 
if (millis() >= pingTimer) {   // pingSpeed milliseconds since last ping, do another ping.
  pingTimer += pingSpeed;      // Set the next ping time.
  sonar.ping_timer(echoCheck); // Send out the ping, calls "echoCheck" function every 24uS where you can check the ping status.
  //sonarfront.ping_timer(echoCheck); 
}
*/

/*
  Get flight mode and set to 1/2/3.
              Execution time = 4 to 8 uSec.
*/
switch (Check_flight_mode()) {
  case 1:   {
      pass_thru();
      airBorn = 0;
    }    break;   
  case 2:   {
      AltLoop( pwm_value_throttle, uSonic_bottom, ping_received);
    }    break;  
} // switch


/*
* Print to serial and check battery voltage.
*/
if (millis() - print_timer > 100) {
//  if(ping_received){    // was too fast for 57600 baudrate
  print_timer = millis(); // reset the timer
  print_csv();
  
}//if
volt = analogRead(voltagePin);  //read battery volt. 
} //loop
 
 

/*
 * Print flight log for analysis
 */
void print_csv() {
  Serial2.print(Check_flight_mode()); //7 FM integer
  Serial2.print(", ");
  Serial2.print(Altset,2);
  Serial2.print(", ");
  Serial2.print(uSonic_front);
  Serial2.print(", ");
  Serial2.print(uSonic_bottom);
  Serial2.print(", ");
  Serial2.print(comp_uSonic_bottom);
  Serial2.print(", "); 
  Serial2.print(new_lift_thrust);
  Serial2.print(", ");
  Serial2.print(KpContribR, 4);
  Serial2.print(", ");
  Serial2.print(KiContribR, 4);
  Serial2.print(", ");
  Serial2.print(KdContribR, 4);
  Serial2.print(", ");
  Serial2.print(pwm_value_throttle);
  Serial2.print(", ");
  Serial2.print(KpAlt, 5);
  Serial2.print(", ");
  Serial2.print(KiAlt, 5);
  Serial2.print(", ");
  Serial2.print(KdAlt, 5);
  Serial2.print(", ");
  Serial2.print(volt);  //analog in
  Serial2.print(", ");
  Serial2.print(((volt) * 0.0198), 1); // Print battery voltage, calibrated with DVM
  Serial2.print(", estimated throt: ");
  Serial2.print(basehover);
  Serial2.print(", ");
  Serial2.print(roll);
  Serial2.print(", ");
  Serial2.print(pitch);
  Serial2.print(", ");
  Serial2.print(heading); 
  Serial2.print(", ");
  Serial2.print(pwm_value_pitch_front);   
  Serial2.println(", ");
}
 
