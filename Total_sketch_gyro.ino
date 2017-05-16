#include <PID_AutoTune_v0.h>
#include <PID_v1.h>
#include <PinChangeInt.h>
#include <Servo.h>
#include <Wire.h>        
#include <troyka-imu.h>    
#include "Kalman.h"

#define MOTOR_PIN1 30
#define MOTOR_PIN2 32
#define MOTOR_PIN3 34
#define MOTOR_PIN4 36 
#define MIN_ROLL_PULSE 1112
#define MAX_ROLL_PULSE 1972
#define MIN_PITCH_PULSE 1116
#define MAX_PITCH_PULSE 1924
#define MIN_YAW_PULSE 1016
#define MAX_YAW_PULSE 1872
#define MIN_PULSE 1000
#define PROCENT_PITCH 200
#define PROCENT_ROLL 200
#define PROCENT_YAW 200
#define Kp 0.16
#define Ki 0.01
#define Kd 0.07

Servo motor1;
Servo motor2;
Servo motor3;
Servo motor4;

int led = 13;
/*int ch6_pin=11;
int ch5_pin=10;*/
int ch4_pin=53;
int ch3_pin=52;
int ch2_pin=51;
int ch1_pin=50;
float rollCoef;
float pitchCoef;
float yawCoef;
int motor1_output;
int motor2_output;
int motor3_output;
int motor4_output;
float aX, aY, aZ;    
float gX, gY, gZ;     
float accelAngleX;        
double Setpoint_pitch;
double Setpoint_roll;
double Input_pitch;
double Input_roll;
float kalmanAngleX;
float kalmanAngleY;
float kalmanAngleZ;
uint32_t timer;
float kalman_angle_X=0;
float kalman_angle_Y=0;
float kalman_angle_Z=0;
unsigned long tune_timer_start;
unsigned long time_elapsed;
Gyroscope gyro; 
Accelerometer accel; 
Kalman kalmanX;
Kalman kalmanY;
Kalman kalmanZ;

volatile static uint16_t ch4_static_count;//local count
volatile static uint16_t ch3_static_count;//local count
volatile static uint16_t ch2_static_count;//local count
volatile static uint16_t ch1_static_count;//local count

// global values for all the channels
/*volatile uint16_t ch6_start=0, ch6_global_count=0;
volatile uint16_t ch5_start=0, ch5_global_count=0;*/
volatile uint16_t ch4_start=0, ch4_global_count=0;
volatile uint16_t ch3_start=0, ch3_global_count=0;
volatile uint16_t ch2_start=0, ch2_global_count=0;
volatile uint16_t ch1_start=0, ch1_global_count=0;
 
volatile uint8_t flag=LOW;//global flag
 
/*volatile uint8_t ch6_global_flag=LOW;//global flag
volatile uint8_t ch5_global_flag=LOW;//global flag*/
volatile uint8_t ch4_global_flag=LOW;//global flag
volatile uint8_t ch3_global_flag=LOW;//global flag
volatile uint8_t ch2_global_flag=LOW;//global flag
volatile uint8_t ch1_global_flag=LOW;//global flag

double PID_Output_pitch;
double PID_Output_roll;
double PID_Output_yaw;

PID pid_pitch(&Input_pitch, &PID_Output_pitch, &Setpoint_pitch, Kp, Ki, Kd, DIRECT);
PID pid_roll(&Input_roll, &PID_Output_roll, &Setpoint_roll ,Kp ,Ki ,Kd, DIRECT);
PID_ATune pitch_aTune(&Input_pitch, &PID_Output_pitch);
//PID pid_yaw(&Input_yaw, &Output_yaw, &Setpoint_yaw,Kp,Ki,Kd, DIRECT);

 
float calc_krenCoef(int kren){
  rollCoef = map(kren,MIN_ROLL_PULSE,MAX_ROLL_PULSE,-PROCENT_ROLL,PROCENT_ROLL);
  //rollCoef=rollCoef/100.0;
  Serial.print("Roll coef=");
  Serial.println(rollCoef);
  if ((kren<=1510)&&(kren>=1495))
  return 0;
  else
  return rollCoef;
}

float calc_tangazhCoef(int tangazh){
  pitchCoef = map(tangazh,MIN_PITCH_PULSE,MAX_PITCH_PULSE,-PROCENT_PITCH,PROCENT_PITCH);
  //pitchCoef=pitchCoef/100.0;
  Serial.print("Pitch coef=");
  Serial.println(pitchCoef);
  if ((tangazh<=1522)&&(tangazh>=1500))
  return 0;
  else
  return pitchCoef;
}

float calc_ryskCoef(int yaw){
  yawCoef = map(yaw,MIN_YAW_PULSE,MAX_YAW_PULSE,-PROCENT_YAW,PROCENT_YAW);
  //yawCoef=yawCoef/100.0;
  Serial.print("Yaw coef=");
  Serial.println(yawCoef);
  if ((yaw<=1450)&&(yaw>=1430))
  return 0;
  else
  return yawCoef;
}

// the setup routine runs once when you press reset:
void setup()
{
    // initialize the digital pin as an output.
    Setpoint_pitch=180;
    Setpoint_roll=180;
    pinMode(led, OUTPUT);
    Serial.begin(9600);
    gyro.begin(); 
    accel.begin();
    kalmanX.setAngle(180);
    kalmanY.setAngle(180);
    timer = micros();
    motor1.attach(MOTOR_PIN1);
    motor2.attach(MOTOR_PIN2);
    motor3.attach(MOTOR_PIN3);
    motor4.attach(MOTOR_PIN4);
    pid_pitch.SetMode(AUTOMATIC);
    pid_roll.SetMode(AUTOMATIC);
    pid_pitch.SetSampleTime(10);
    pid_roll.SetSampleTime(10);
    pid_pitch.SetOutputLimits(-200,200);
    pid_roll.SetOutputLimits(-200,200);
    pitch_aTune.SetOutputStep(10);
    pitch_aTune.SetControlType(1);
    pitch_aTune.SetNoiseBand(1);
    tune_timer_start = millis()/1000.0;
    /*PCintPort::attachInterrupt(ch6_pin,ch6_count,CHANGE);
    PCintPort::attachInterrupt(ch5_pin,ch5_count,CHANGE);*/
    PCintPort::attachInterrupt(ch4_pin,ch4_count,CHANGE);
    PCintPort::attachInterrupt(ch3_pin,ch3_count,CHANGE);
    PCintPort::attachInterrupt(ch2_pin,ch2_count,CHANGE);
    PCintPort::attachInterrupt(ch1_pin,ch1_count,CHANGE);
    //Serial.println("press the button");
}
 
// the loop routine runs over and over again forever:
void loop() {
    /*volatile static uint16_t ch6_static_count;//local count
    volatile static uint16_t ch5_static_count;//local count*/
    volatile static uint16_t ch4_static_count;//local count
    volatile static uint16_t ch3_static_count;//local count
    volatile static uint16_t ch2_static_count;//local count
    volatile static uint16_t ch1_static_count;//local count
     
    volatile static uint8_t updateflags;//lcoal flag
    /*volatile static uint8_t ch6_update_flag;//lcoal flag
    volatile static uint8_t ch5_update_flag;//lcoal flag*/
    volatile static uint8_t ch4_update_flag;//lcoal flag
    volatile static uint8_t ch3_update_flag;//lcoal flag
    volatile static uint8_t ch2_update_flag;//lcoal flag
    volatile static uint8_t ch1_update_flag;//lcoal flag
     
    if(flag)
    {
    noInterrupts();
    updateflags=flag;
    /*ch6_update_flag=ch6_global_flag;
    ch5_update_flag=ch5_global_flag;*/
    ch4_update_flag=ch4_global_flag;
    ch3_update_flag=ch3_global_flag;
    ch2_update_flag=ch2_global_flag;
    ch1_update_flag=ch1_global_flag;
     
    /*if(ch6_update_flag)
    {
        ch6_static_count=ch6_global_count;
    }
    if(ch5_update_flag)
    {
        ch5_static_count=ch5_global_count;
    }*/
    if(ch4_update_flag)
    {
        ch4_static_count=ch4_global_count;
    }
    if(ch3_update_flag)
    {
        ch3_static_count=ch3_global_count;
    }
    if(ch2_update_flag)
    {
        ch2_static_count=ch2_global_count;
    }
    if(ch1_update_flag)
    {
        ch1_static_count=ch1_global_count;
    }
    //Serial.print("ch6: ");
    //Serial.println(ch6_static_count);
    //Serial.print("\t");
     
    // Serial.print("ch5: ");
    //Serial.println(ch5_static_count);
    // Serial.print("\t");
     
    Serial.print("ch4: ");
    Serial.println(ch4_static_count);
    Serial.print("\t");
    if (ch3_static_count<1900)
        ch3_static_count=ch3_static_count+100;
    Serial.print("ch3: ");
    Serial.println(ch3_static_count);
    Serial.print("\t");
     
    Serial.print("ch2: ");
    Serial.println(ch2_static_count);
    Serial.print("\t");
     
    Serial.print("ch1: ");
    Serial.println(ch1_static_count);
    Serial.print("\n");
     
    /*ch6_global_count=0;
    ch6_global_flag=0;
    ch6_update_flag=0;
     
    ch5_global_count=0;
    ch5_global_flag=0;
    ch5_update_flag=0;*/
     
    ch4_global_count=0;
    ch4_global_flag=0;
    ch4_update_flag=0;
     
    ch3_global_count=0;
    ch3_global_flag=0;
    ch3_update_flag=0;
     
    ch2_global_count=0;
    ch2_global_flag=0;
    ch2_update_flag=0;
     
    ch1_global_count=0;
    ch1_global_flag=0;
    ch1_update_flag=0;
    flag=0;
    interrupts();
     
    //use all the ch*_static_count for computation here.
    }
    
    pitchCoef=calc_tangazhCoef(ch2_static_count); 
    rollCoef=calc_krenCoef(ch1_static_count);
    yawCoef=calc_ryskCoef(ch4_static_count);
    gX = gyro.readX_DegPerSec(); 
    gY = gyro.readY_DegPerSec();
    gZ = gyro.readZ_DegPerSec();
    aX = accel.readX_G(); 
    aY = accel.readY_G(); 
    aZ = accel.readZ_G();
    kalman_angle_X = (atan2(aY,aZ)+PI)*RAD_TO_DEG;
    kalman_angle_Y = (atan2(aX,aZ)+PI)*RAD_TO_DEG;
    kalman_angle_Z = (atan2(aX,aY)+PI)*RAD_TO_DEG;
    kalmanAngleX=kalmanX.getAngle(kalman_angle_X, gX, (double)(micros()-timer)/1000000);
    kalmanAngleY=kalmanY.getAngle(kalman_angle_Y, gY, (double)(micros()-timer)/1000000);
    kalmanAngleZ=kalmanZ.getAngle(kalman_angle_Z, gZ, (double)(micros()-timer)/1000000);
    Input_pitch=kalmanAngleX;
    Input_roll=kalmanAngleY;
    pitch_aTune.Runtime();
    /*Uncomment code below for setting PID parameters automatically*/
    /*if (pitch_aTune.Runtime()==1)
      {
           pid_pitch.SetTunings(pitch_aTune.GetKp(),pitch_aTune.GetKi(),pitch_aTune.GetKd());
           Serial.println("Tuned parametres");
           Serial.println(Kp);
           Serial.println(Ki);
           Serial.println(Kd);
       }
       else 
       {
            pid_pitch.SetTunings(Kp,Ki,Kd);
       }
    Serial.print("pitch_aTKP = ");
    Serial.println(pitch_aTune.GetKp());
    Serial.print("pitch_aTKI = ");
    Serial.println(pitch_aTune.GetKi());
    Serial.print("pitch_aTKD = ");
    Serial.println(pitch_aTune.GetKd());
    if (pitch_aTune.GetKp()!=0)
    {
        time_elapsed = millis()/1000.0;
        Serial.print("Counting time: ");
        Serial.println(time_elapsed-tune_timer_start);
    }
    pid_pitch.Compute();
    pid_roll.Compute();*/
    /*motor1_output=round(ch3_static_count*(1-PID_Output_pitch)*(1+rollCoef)*(1-yawCoef));
    motor2_output=round(ch3_static_count*(1-PID_Output_pitch)*(1-rollCoef)*(1+yawCoef));
    motor3_output=round(ch3_static_count*(1+PID_Output_pitch)*(1+rollCoef)*(1+yawCoef));
    motor4_output=round(ch3_static_count*(1+PID_Output_pitch)*(1-rollCoef)*(1-yawCoef));*/
    if ((ch1_static_count<1485)||(ch1_static_count>1511))
        PID_Output_roll=rollCoef;
    if ((ch2_static_count<1500)||(ch2_static_count>1525))
        PID_Output_pitch=pitchCoef;
    Serial.print("PID_ROLL:");
    Serial.println(PID_Output_roll);
    Serial.print("PID PITCH:");
    Serial.println(PID_Output_pitch);
    yawCoef=0;
    motor1_output=round(ch3_static_count+PID_Output_pitch-PID_Output_roll-yawCoef);
    motor2_output=round(ch3_static_count+PID_Output_pitch+PID_Output_roll+yawCoef);
    motor3_output=round(ch3_static_count-PID_Output_pitch-PID_Output_roll+yawCoef);
    motor4_output=round(ch3_static_count-PID_Output_pitch+PID_Output_roll-yawCoef);
    motor1.writeMicroseconds(motor1_output);
    motor2.writeMicroseconds(motor2_output);
    motor3.writeMicroseconds(motor3_output);
    motor4.writeMicroseconds(motor4_output);
    Serial.print("Motor1 Out:");
    Serial.println(motor1_output);
    /*Serial.print("Motor1 Out PID:");
    Serial.println(Output1);*/
    Serial.print("Motor2 Out:");
    Serial.println(motor2_output);
    /*Serial.print("Motor2 Out PID:");
    Serial.println(Output2);*/
    Serial.print("Motor3 Out:");
    Serial.println(motor3_output);
    //Serial.print("Motor3 Out PID:");
    //Serial.println(Output3);
    Serial.print("Motor4 Out:");
    Serial.println(motor4_output);
    //Serial.print("Motor4 Out PID:");
    //Serial.println(Output4);
    Serial.print("kalmanAngleX: \t"); Serial.print(kalmanAngleX); Serial.print(", \t"); Serial.println("");
    Serial.print("kalmanAngleY: \t"); Serial.print(kalmanAngleY); Serial.print(", \t"); Serial.println("");
    Serial.print("kalmanAngleZ: \t"); Serial.print(kalmanAngleZ); Serial.print(", \t"); Serial.println("");
    delay(1200);
    /*digitalWrite(led, HIGH); 
    delay(200); 
    digitalWrite(led, LOW); 
    delay(200);*/
}
/*void ch6_count()
{
    //Serial.println(millis());
    if(digitalRead(ch6_pin)==HIGH)
    {
      ch6_start= micros();
    }
else
  {
    ch6_global_count=(uint16_t)(micros()-ch6_start);
    flag=HIGH;
    ch6_global_flag=HIGH;
  }
}
void ch5_count()
{
    //Serial.println(millis());
    if(digitalRead(ch5_pin)==HIGH)
    {
      ch5_start= micros();
    }
    else
    {
      ch5_global_count=(uint16_t)(micros()-ch5_start);
      flag=HIGH;
      ch5_global_flag=HIGH;
    }
}*/
void ch4_count()
{
    //Serial.println(millis());
    if(digitalRead(ch4_pin)==HIGH)
  {
    ch4_start= micros();
  }
    else
      {
        ch4_global_count=(uint16_t)(micros()-ch4_start);
        flag=HIGH;
        ch4_global_flag=HIGH;
      }
}
void ch3_count()
{
    //Serial.println(millis());
    if(digitalRead(ch3_pin)==HIGH)
    {
        ch3_start= micros();
    }
    else
    {
        ch3_global_count=(uint16_t)(micros()-ch3_start);
        flag=HIGH;
        ch3_global_flag=HIGH;
    }
}
void ch2_count()
{
    //Serial.println(millis());
    if(digitalRead(ch2_pin)==HIGH)
    {
        ch2_start= micros();
    }
    else
    {
        ch2_global_count=(uint16_t)(micros()-ch2_start);
        flag=HIGH;
        ch2_global_flag=HIGH;
    }
}
void ch1_count()
{
    //Serial.println(millis());
    if(digitalRead(ch1_pin)==HIGH)
    {
        ch1_start= micros();
    }
    else
    {
        ch1_global_count=(uint16_t)(micros()-ch1_start);
        flag=HIGH;
        ch1_global_flag=HIGH;
    }
}

