/*
 * Date: 4/18/18
 * Author: Collin Timmons
 * This code was written for the TCPA research group to control their muscle twister. 
 * 
 * The code is designed to work with an Arduino Mega 2560 R3 with a rep-rap GPL3 3D printer motor control shield.
 * 
 * Connections:
 *   Power: 
 *   Motors: Each motor requires a A4988 motor driver board inserted into a motor port (X,Y,Z,E0,E1) of the GPL3 board. The motor is then connected to the corresponding motor output port. 
 *      Inside the code the tokens LeadMotor and TwistMotor should be #defined. The default definitions #define TwistMotor X and #define LeadMotor Z.
 *   Limit Switches:
 *   Load Cell: 
 *   
 *Issues: The impact on the peripherals due to using Timer3 is not fully understood at this point in time. Some software PWM outputs will be non-operational.
 *  This issue can be fixed by using a different timer.
*/

//Comment out to remove debug mode.
//#define DebugMode

//Pin Defines
#define INPUT_PIN A9
#define ROT_ENCODER_PIN_1 31
#define ROT_ENCODER_PIN_2 33

//Constant Defines
#define IN 1
#define OUT 0
#define SETPOINT 200
#define SETPOINTRANGE 10
#define SETPOINTMAX SETPOINT + SETPOINTRANGE
#define SETPOINTMIN SETPOINT - SETPOINTRANGE

//Other Defines
//#define ROT_ENCODER_PIN_BUT 

//#define LeadMotor Z
#include "TCPATwister.h"

//PID Library
#include <PID_v1.h>

// Variables used for PID controller
double elapsedTime, time, timePrev, PD, error, previous_error, Input, Output, PosOutput ;
int SetOutput;
double Setpoint = SETPOINT;
double pid_p=0;
double pid_d=0;
/////////////////PD CONSTANTS/////////////////
double kp=3.55;//3.55
double kd=2.05;//2.05
///////////////////////////////////////////////

TCPATwister *Twister;
bool EncoderUpdate = false;

void setup() {
    Twister = &TCPATwister::SingletonTwister();
    Twister->Initialize();
    Twister->setTwistSetCount(20);
    Twister->setLeadSetCount(20);
    Twister->StartTimer(50);
    Twister->toggleLeadDir();
    InitEncoder();

    time = millis(); //Start counting time in milliseconds
	Input = analogRead(INPUT_PIN);
		#ifdef DebugMode
			Serial.begin(9600);
		#endif
    
	previous_error = Input - Setpoint;
	Output = 0;
	SetOutput = 0;
	Twister->setLeadSetCount(300-SetOutput);
    Twister->setTwistEnable(1);     //Active low enable, Twister Motor is diasablled
	Twister->setLeadEnable(1); 
}

void loop(){
  timePrev = time;  // the previous time is stored before the actual time read
  time = millis();  // actual time read
  elapsedTime = (time - timePrev) / 1000; 
  Input = analogRead(INPUT_PIN); //This is the value comming from the load cell
  //checking input as a precaution.
  if (Input >= 550){
    Twister->setTwistEnable(1);     //Active low enable, 1 disables
    Twister->setLeadDir(IN);
    Twister->setLeadSetCount(20);
  }
  error = Input-Setpoint;
  pid_p = kp*error;
  pid_d = kd*((error - previous_error)/elapsedTime);
  /*Generate the PD Output*/
  PD = pid_p + pid_d;
  /*We need to setup the upper and lower limit of the PD output*/
  if(PD < -1024){
    PD=-1024;
  }
  if(PD > 1024){
    PD=1024;
  }
  /*Update Motor*/
  Output = Output + PD;
  /*Check for output values*/

  //grabbing output and making it positive
  PosOutput=abs(Output);
  PosOutput=50*(PosOutput/1024);

 //digitalWrite(1,!digitalRead(1));
  if (Output < 0) {
   
    //code to reverse motor direction
	Twister->setLeadEnable(1);
	delay(100);
    Twister->setLeadDir(IN);
	Twister->setLeadEnable(0);
	
  }
  else{
	Twister->setLeadEnable(1);
	delay(100);
    Twister->setLeadDir(OUT);
	Twister->setLeadEnable(0);
  }
  SetOutput= (int) PosOutput;
  Twister->setLeadSetCount(50-SetOutput);
  previous_error = error; //Remember to store the previous error.
}


//Rotary encoder code, Not needed for Twister.
void InitEncoder(){
  pinMode(ROT_ENCODER_PIN_1,INPUT_PULLUP);
  pinMode(ROT_ENCODER_PIN_2,INPUT_PULLUP);
//  pinMode(ROT_ENCODER_PIN_BUT,INPUT_PULLUP);
}
void CheckEncoder(){
  if(digitalRead(ROT_ENCODER_PIN_1) == LOW && !EncoderUpdate){
    EncoderUpdate = true;
    if(digitalRead(ROT_ENCODER_PIN_2) == LOW){
      Encoder_CW();
    }else{
      Encoder_CCW();
    }
  }else if(digitalRead(ROT_ENCODER_PIN_1) == HIGH && digitalRead(ROT_ENCODER_PIN_2) == HIGH){
    EncoderUpdate = false;
  }
}
void Encoder_CW(){
 Twister->setTwistSetCount(Twister->getTwistSetCount() + 1);  
}
void Encoder_CCW(){
 Twister->setTwistSetCount(Twister->getTwistSetCount() - 1);
}
  



