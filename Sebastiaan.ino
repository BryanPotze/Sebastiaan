#include <NewPing.h>
#define TRIG_PIN 7
#define ECHO_PIN 6
#define MAX_DISTANCE 200
NewPing sonar(TRIG_PIN, ECHO_PIN, MAX_DISTANCE);
const int motorA1 = 3;
const int motorA2 = 9;
const int motorB1 = 10;
const int motorB2 = 11;
const int buttonPinA = 8;
const int buttonPinB = 2;
const int gripper = 5;
const int lineSensor[] = {A0, A1, A2, A3, A4, A5, A6, A7};
const int adjustInterval = 10; 
int lineSensorValue[8] = {0};
int buttonStateA;
int buttonStateB;
int flagGone = 0;
int distance = 1;
int forwardsMillis;
int adjustMillis;
int sonarMillis;
int buttonMillis;


void setup() 
{
  Serial.begin(9600);
  pinMode(motorA1, OUTPUT);
  pinMode(motorA2, OUTPUT);
  pinMode(motorB1, OUTPUT);
  pinMode(motorB2, OUTPUT);
  pinMode(buttonPinA, INPUT);
  pinMode(buttonPinB, INPUT);
  for (int i = 0; i < 8; i++) 
  {
    pinMode(lineSensor[i], INPUT);
  }
}

void loop() 
{
  readButtons();
  readSonar();
  readSensors();
  flagReset();
  drive();
}

void goForwards() 
{
  analogWrite(motorA2, 255);
  analogWrite(motorB1, 255);
}

void goBackwards() 
{
  analogWrite(motorA1, 255);
  analogWrite(motorB2, 255);
}
void stop()
{
    analogWrite(motorA1, 0);
    analogWrite(motorA2, 0);
    analogWrite(motorB1, 0);
    analogWrite(motorB2, 0);
} 

void adjustAngleOutside1() 
{
  if (lineSensorValue[5] <= lineSensorValue[2]) 
  {
    analogWrite(motorA2, 255);
    analogWrite(motorB1, 40); 
  } 
  else if (lineSensorValue[2] <= lineSensorValue[5]) 
  {
    analogWrite(motorB1, 255);
    analogWrite(motorA2, 40); 
  }
}

void adjustAngleOutside2() 
{
  if (lineSensorValue[6] <= lineSensorValue[1]) 
  {
    analogWrite(motorA2, 255);
    analogWrite(motorB2, 20);
  } 
  else if (lineSensorValue[1] <= lineSensorValue[6]) 
  {
    analogWrite(motorB1, 255);
    analogWrite(motorA2, 20);
  }
}

void adjustAngleOutside3() 
{
  if (lineSensorValue[7] <= lineSensorValue[0]) 
  {
    analogWrite(motorA2, 100);
    analogWrite(motorB2, 0);
  } 
  else if (lineSensorValue[0] <= lineSensorValue[7]) 
  {
    analogWrite(motorB1, 100);
    analogWrite(motorA2, 0);
  }
}
void readSensors() 
{
  for (int i = 0; i < 8; i++) 
  {
    lineSensorValue[i] = analogRead(lineSensor[i]);
  }
}

void flagReset()
{
  if (buttonStateA == LOW)
  {
    flagGone = 0;
  }
  if (flagGone == 0)
  {
    if (distance >= 30 || distance == 0)
    {
      flagGone = 1;
    }
  }
}

void drive()
{
  if (flagGone == 1)
  {
    bool needToAdjustOutside1 = (lineSensorValue[2] >= 800) || (lineSensorValue[5] >= 800);
    bool needToAdjustOutside2 = (lineSensorValue[1] >= 800) || (lineSensorValue[6] >= 800);
    bool needToAdjustOutside3 = (lineSensorValue[0] >= 800) || (lineSensorValue[7] >= 800);
    if (distance <= 20 && distance >= 1)
    {
    }
  
      else if (millis() >= adjustMillis) 
      {
        adjustMillis = millis() + adjustInterval;
        if (needToAdjustOutside1) 
        { 
          adjustAngleOutside1();
        } 
        else if (needToAdjustOutside2) 
        {
          adjustAngleOutside2();
        } 
        else if (needToAdjustOutside3) 
        {
          adjustAngleOutside3();
        } 
        else 
        {
          goForwards();
        }
      }
  }
  else
  {
  stop(); 
  } 
}

void readSonar()
{
   if (millis() >= sonarMillis) 
   {
      sonarMillis = millis() + 100;
      distance = sonar.ping_cm();
   }
}

void readButtons()
{
   if (millis() >= buttonMillis) 
   {
      buttonMillis = millis() + adjustInterval;
      buttonStateA = digitalRead(buttonPinA);
      buttonStateB = digitalRead(buttonPinB);
   }
}
