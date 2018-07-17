#include <PID_v1.h>

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
#define INPUT_PIN A9
#define ROT_ENCODER_PIN_1 31
#define ROT_ENCODER_PIN_2 33
#define IN 1
#define OUT 0
//#define ROT_ENCODER_PIN_BUT 


//#define LeadMotor Z
#include "TCPATwister.h"

//PID Library
#include <PID_v1.h>

//Define Variables we'll be connecting to
double Setpoint, Input, Output, PosOutput;
int SetOutput;
char watch = 1;

//Specify the links and initial tuning parameters
PID myPID(&Input, &Output, &Setpoint,.2,0,0, DIRECT);


TCPATwister *Twister;
bool EncoderUpdate = false;

void setup() {
  pinMode(1,OUTPUT);
//initialize the variables we're linked to
//input is output from loadcell.  I picked an arbitray pin for now
  Input = analogRead(INPUT_PIN);
//setpoint is also pretty arbitrary as of right now.
  Setpoint=102;
  //Setpoint = 100;
 myPID.SetOutputLimits(-1023, 1023); 

//Serial.begin(9600);

  //turn the PID on
  myPID.SetMode(AUTOMATIC);

  
  Twister = &TCPATwister::SingletonTwister();
  Twister->Initialize();
  Twister->setTwistSetCount(20);
  Twister->setLeadSetCount(20);
  Twister->StartTimer(50);
  Twister->toggleLeadDir();
//Serial.println("h");
  InitEncoder();
  Twister->setTwistEnable(1);     //Active low enable, 1 disables
  while(Input < Setpoint-10 || Input > Setpoint+10){
    PID_Contr();
    Input = analogRead(INPUT_PIN);
//Serial.println(Input);
  }
  Twister->setTwistEnable(0);     //Active low enable, 1 disables
  
}
void loop(){
PID_Contr();

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




void PID_Contr(){

 
  //input is output from loadcell.  I picked an arbitray pin for now
  Input = analogRead(INPUT_PIN);
 //Serial.println(Input);
  //Output=(Output,0,255,1023,0);
//checking input as a precaution.
  if (Input >= 550){
    Twister->setTwistEnable(1);     //Active low enable, 1 disables
    Twister->setLeadDir(IN);
    Twister->setLeadSetCount(20);
  }
  myPID.Compute(); 
//grabbing output and making it positive
  PosOutput=abs(Output);
  PosOutput=50*(PosOutput/550);
 // Serial.println(Output);
 watch = !watch;
 digitalWrite(1,watch);
 //digitalWrite(1,!digitalRead(1));
  if (Output < 0) {
   
    //code to reverse motor direction
    Twister->setLeadDir(IN);
  }
  else{
    Twister->setLeadDir(OUT);
    }
    
  SetOutput= (int) PosOutput;
Twister->setLeadSetCount(50-SetOutput);

}
  


