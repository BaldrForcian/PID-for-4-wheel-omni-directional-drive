#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

//BACK LEFT
int PWM_CCW_BL = 8;                             //back left motor spinning counterclockwise
int PWM_CW_BL = 6;                              //back left motor spinning clockwise

//FRONT RIGHT
int PWM_CCW_FR  = 5;                            //front right motor spinning counterclockwise
int PWM_CW_FR = 11;                             //front right motor spinning clockwise

 //FRONT LEFT
int PWM_CCW_FL = 12;                            //front left motor spinning counterclockwise
int PWM_CW_FL = 10;                             //front left motor spinning clockwise

//BACK RIGHT
int PWM_CCW_BR = 7;                             //back right motor spinning counterclockwise
int PWM_CW_BR = 9;                              //back right motor spinning clockwise

////////////////////////////////////////////////////////////////////////////////////////////
const int consthitrost = 150;                                             //constant speed
////////////////////////////////////////////////////////////////////////////////////////////
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28, &Wire);                   //Gyro
int kotZ;                                                                 //Yaw angle
//////////PID//////////
double deltaT, last_time;                                                 //deltaT-change of time, last_time-variable for storing last time of loop
double integral, last_error, OutputPID = 0, OutputPIDmovements = 0;       //OutputPID is for straightening only with cw/ccw movements   //OutputPIDmovements is for straightening while moving in a direction
double Kp, Ki, Kd;                                                        //PID parameters
double setpoint = 0;                                                      //the angle we wish to straighten to  
double now = 0;                                                           //variable for storing the now time with millis
double error = 0;                                                         //difference between setpoint and kotZ
double proportional;
double derivative;


void setup() {

Serial.begin(9600);
////////////////////Gyro setting////////////////////////////////////////
  if (!bno.begin()){}
  bno.setExtCrystalUse(true);
////////////////////motor setting///////////////////////////////////////
pinMode(PWM_CCW_FL,OUTPUT); 
pinMode(PWM_CW_FL, OUTPUT);
                                          
pinMode(PWM_CCW_BL,OUTPUT); 
pinMode(PWM_CW_BL, OUTPUT);
                                           
pinMode(PWM_CCW_FR,OUTPUT); 
pinMode(PWM_CW_FR, OUTPUT);
                                          
pinMode(PWM_CCW_BR,OUTPUT); 
pinMode(PWM_CW_BR, OUTPUT);

pinMode(47,INPUT_PULLUP); //Start switch

//////////pid nastavitev//////////////////////////////////////////////////////////////////////////
  Kp = 6;                             //proportional paramter
  Ki = 0.2;                           //integral paramter
  Kd = 0.06;                          //derivative parameter
  last_time = 0;                      //last_time is reset
//////////////////////////////////////////////////////////////////////////////////////////////////
}

void loop() {

CalculatePID();                                   //used to calculate all the pwm values for PID

if(digitalRead(47) == HIGH){                      //on switch    

  if (kotZ > 0){                                  //if the yaw angle is bigger than 0 degrees we turn it clockwise
    cw();
  }
  else if(kotZ < 0){                              //if the yaw angle is smaller than 0 degrees we turn it couterclockwise
    ccw();
  }
  else{                                           //if its excatly 0 degrees the robot stops
    stop();
  }

}
else{                                             //stop if switch is not on
  stop();
}

}

void CalculatePID(){                                                            //subprogram for calculating PID values
  now = millis();                                                               //we track the current time
  deltaT = (now - last_time)/1000.00;                                           //we get the time diference between the current and the previous cycle in seconds
  last_time = now;                                                              //time of previous cycle becomes now

  sensors_event_t event;                                                        
  bno.getEvent(&event);                                                         //gyro sensing
  kotZ = (event.orientation.x);                                                 //we write yaw angle to kotZ
  
  if (kotZ >= 180) kotZ = map(kotZ,360,180,0,-180);                             //maping the angle so the range is from -180 to 180 
  
  error = setpoint - kotZ;                                                      //error is the difference between our setpoint and our yaw angle
  
  proportional = error;                                                         //the proportional part is equal to our error
  integral += error * deltaT;                                                   //the integral part is equal to integral + error*deltaT(change of error over time)
  derivative = (error - last_error) / deltaT;                                   //the derivative part is equal to (error-last_error)/deltaT
  last_error = error;                                                           //last error is equal to current error
  OutputPID = (Kp * proportional) + (Ki * integral) + (Kd * derivative) ;       //we calculate OutputPID with this equation

  OutputPIDmovements = map(OutputPID, -255, 255, -100, 100);                    //we map the PID for movements so they go from -100 to 100 since we are adding or subracting them from the consant speed
  OutputPIDmovements = constrain(OutputPIDmovements, -100, 100);                //constraining the map so it doesn't go over or under
  
  if (OutputPID < 0){                                                           //for negative OutputPID we take the absolute value(-4 -> 4)
    OutputPID = abs(OutputPID);                                                 
  }
  
  if (OutputPID < 30){                                                          //if OutputPID is lower than 30 we set it to 0 as our motors stop working at those pwms and squeal
    OutputPID = 0;                                                              //does not need to be included
  }
  OutputPID = constrain(OutputPID, 0, 255);                                     //constraining OutputPID

}


void cw(){                                                                     //Clockwise
  analogWrite(PWM_CCW_FL,0);
  analogWrite(PWM_CW_FL,OutputPID);
 
  analogWrite(PWM_CCW_BL,0);
  analogWrite(PWM_CW_BL,OutputPID);

  analogWrite(PWM_CCW_FR,0);
  analogWrite(PWM_CW_FR,OutputPID);

  analogWrite(PWM_CCW_BR,0);
  analogWrite(PWM_CW_BR,OutputPID);
}
void ccw(){                                                                   //CounterClockwise
  analogWrite(PWM_CCW_FL,OutputPID);
  analogWrite(PWM_CW_FL,0);
 
  analogWrite(PWM_CCW_BL,OutputPID);
  analogWrite(PWM_CW_BL,0);

  analogWrite(PWM_CCW_FR,OutputPID);
  analogWrite(PWM_CW_FR,0);

  analogWrite(PWM_CCW_BR,OutputPID);
  analogWrite(PWM_CW_BR,0);
}
void stop(){                                                                 
  analogWrite(PWM_CCW_FL,0);
  analogWrite(PWM_CW_FL,0);
 
  analogWrite(PWM_CCW_BL,0);
  analogWrite(PWM_CW_BL,0);

  analogWrite(PWM_CCW_FR,0);
  analogWrite(PWM_CW_FR,0);

  analogWrite(PWM_CCW_BR,0);
  analogWrite(PWM_CW_BR,0);
  delay (30);
}
void forward(){
  analogWrite(PWM_CCW_FL,consthitrost+OutputPIDmovements);
  analogWrite(PWM_CW_FL,0);
 
  analogWrite(PWM_CCW_BL,consthitrost+OutputPIDmovements);
  analogWrite(PWM_CW_BL,0);

  analogWrite(PWM_CCW_FR,0);
  analogWrite(PWM_CW_FR,consthitrost-OutputPIDmovements);

  analogWrite(PWM_CCW_BR,0);
  analogWrite(PWM_CW_BR,(consthitrost-OutputPIDmovements));

}
void backwards(){
  analogWrite(PWM_CW_FL,consthitrost-OutputPIDmovements);
  analogWrite(PWM_CCW_FL,0);
 
  analogWrite(PWM_CW_BL,consthitrost-OutputPIDmovements);
  analogWrite(PWM_CCW_BL,0);

  analogWrite(PWM_CW_FR,0);
  analogWrite(PWM_CCW_FR,consthitrost+OutputPIDmovements);

  analogWrite(PWM_CW_BR,0);
  analogWrite(PWM_CCW_BR,consthitrost+OutputPIDmovements);

}
void leftp(){
  analogWrite(PWM_CW_FL,consthitrost-OutputPIDmovements);
  analogWrite(PWM_CCW_FL,0);
 
  analogWrite(PWM_CCW_BL,consthitrost+OutputPIDmovements);
  analogWrite(PWM_CW_BL,0);

  analogWrite(PWM_CCW_FR,0);
  analogWrite(PWM_CW_FR,consthitrost-OutputPIDmovements);

  analogWrite(PWM_CW_BR,0);
  analogWrite(PWM_CCW_BR,consthitrost+OutputPIDmovements);

}
void rightp(){
  analogWrite(PWM_CCW_FL,consthitrost+OutputPIDmovements);
  analogWrite(PWM_CW_FL,0);
 
  analogWrite(PWM_CW_BL,consthitrost-OutputPIDmovements);
  analogWrite(PWM_CCW_BL,0);

  analogWrite(PWM_CW_FR,0);
  analogWrite(PWM_CCW_FR,consthitrost+OutputPIDmovements);

  analogWrite(PWM_CCW_BR,0);
  analogWrite(PWM_CW_BR,consthitrost-OutputPIDmovements);

}
void forwardleft(){
  analogWrite(PWM_CCW_FL,100+OutputPIDmovements);
  analogWrite(PWM_CW_FL,0);
 
  analogWrite(PWM_CCW_BL,consthitrost+OutputPIDmovements);
  analogWrite(PWM_CW_BL,0);

  analogWrite(PWM_CCW_FR,0);
  analogWrite(PWM_CW_FR,consthitrost-OutputPIDmovements);

  analogWrite(PWM_CCW_BR,0);
  analogWrite(PWM_CW_BR,100-OutputPIDmovements);
}
void forwardright(){
  analogWrite(PWM_CCW_FL,consthitrost+OutputPIDmovements);
  analogWrite(PWM_CW_FL,0);
 
  analogWrite(PWM_CCW_BL,100+OutputPIDmovements);
  analogWrite(PWM_CW_BL,0);

  analogWrite(PWM_CCW_FR,0);
  analogWrite(PWM_CW_FR,100-OutputPIDmovements);

  analogWrite(PWM_CCW_BR,0);
  analogWrite(PWM_CW_BR,consthitrost-OutputPIDmovements);
}
