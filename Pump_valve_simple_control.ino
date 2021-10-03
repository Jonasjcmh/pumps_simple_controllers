#include "Arduino.h"
#include "ads.h"


//Define the input pins for the sensors

#define ADS_RESET_PIN      (18)           // Bendlabs sensor Pin number attached to ads reset line.
#define ADS_INTERRUPT_PIN  (19)           // Bendlabs sensor.  

#define RUBBER_SENSOR (A0)               //  Adafruit Rubber Cord Sensor analog input
#define PRESSURE_SENSOR (A1)             //  MPX5100 Series Integrated Silicon Pressure Sensor analog input (0 to 100 kPa)

#define PRESSURE_SENSOR (A1)             //  MPX5100 Series Integrated Silicon Pressure Sensor analog input (0 to 100 kPa)

//Arduino PWM Speed Control：
int E1 = 3;    ///<Pump 1 Speed
int M1 = 4;    ///<Pump 1 Direction

int E2 = 11;   ///<Valve 1 Enable
int M2 = 12;  ///<Valve  1 State

const int E3 = 5; ///<Pump 2 Speed
const int M3 = 8; ///<Pump 2 Direction

const int E4 = 6; ///<Valve 2 Enable
const int M4 = 7; ///<Valve 1 Direction

int timecounter = 1;  // Auxiliar variable for controlling the time of the process

//Pressure sensor calibration factors  MPX5100 Series Integrated Silicon Pressure Sensor analog input (0 to 100 kPa)  Vout=Vs(P * 0.009 + 0.04),  Vs=5V = 1024,  P = 
 
const float SensorOffset = 4.44;  //pressure sensor offset
const float SensorGain = 0.109;   // pressure sensor proportional relation


void setup()
{
  //Motor control shield  
    pinMode(M1, OUTPUT);
    pinMode(M2, OUTPUT);
    pinMode(M3, OUTPUT);
    pinMode(M4, OUTPUT);

    pinMode(PRESSURE_SENSOR, INPUT);  // Defining sensor inputs for ADC (Analog digital converter)
    
    Serial.begin(115200);             // Starting Serial communication with computer baudrate 115200 bps

}

void loop()
{
int motorspeed=100;      
float Setpoint=20;

// sensing pressure status
float  pressure_sensorValue = (analogRead(PRESSURE_SENSOR)*SensorGain-SensorOffset); //Do maths for calibration

// Introducion air to the PneuNets
motor_1_on(250);
valve_1_on();

motor_2_on(150);
valve_2_on();

//Stop pump
motor_1_off();
motor_2_off();

// Release the air
valve_1_off();
valve_2_off();

//Controlling the time of the process
if (timecounter <= 30)     // 3 seconds == 30 * 100 ms
{
  // Introducion air to the PneuNets
     motor_1_on(250);
     valve_1_on();

     timecounter++;
  }
 else if ( timecounter > 30 and timecounter<= 60 )
 {
  motor_1_off();
  valve_1_on();
  timecounter++;
  }
  else if ( timecounter > 60 and timecounter<= 90)
  {
  motor_1_off();
  valve_1_off();
  timecounter++;
    }
  else if ( timecounter > 90)
  {
    timecounter=0;
    }

  
    //Print parameters of the system


    Serial.print(pressure_sensorValue);    // pressure data in kpa
    Serial.print(",");

    
    delay(100);   // defining sample time = 100 miliseconds

   // Check for received hot keys on the com port
  if(Serial.available())
  {
    //parse_com_port();
  double incomingValue = Serial.parseFloat();
  //Serial.println(incomingValue*2);
  if (incomingValue != 0)
  {
    Setpoint=incomingValue;
    }
  }
    
}


// Motor control functions 

void motor_1_on(int motorspeed)
{
  analogWrite(E1, motorspeed);   //PWM Speed Control   value
  digitalWrite(M1,HIGH);
  }

void motor_1_off(void)
{
  analogWrite(E1, 0);   //PWM Speed Control   value
  digitalWrite(M1,HIGH);
  }

  void valve_1_on(void)
{
  analogWrite(E2, 255);   //PWM Speed Control   value
  digitalWrite(M2,HIGH);
  }

void valve_1_off(void)
{
  analogWrite(E2, 0);   //PWM Speed Control   value
  digitalWrite(M2,HIGH);
  }

void motor_2_on(int motorspeed)
{
  analogWrite(E3, motorspeed);   //PWM Speed Control   value
  digitalWrite(M3,HIGH);
  }

void motor_2_off(void)
{
  analogWrite(E3, 0);   //PWM Speed Control   value
  digitalWrite(M3,HIGH);
  }

  void valve_2_on(void)
{
  analogWrite(E4, 255);   //PWM Speed Control   value
  digitalWrite(M4,HIGH);
  }

void valve_2_off(void)
{
  analogWrite(E4, 0);   //PWM Speed Control   value
  digitalWrite(M4,HIGH);
  }
