#include <NewPing.h>
#define TRIG_PIN 7
#define ECHO_PIN 6
#define MAX_DISTANCE 200
NewPing sonar(TRIG_PIN, ECHO_PIN, MAX_DISTANCE);
const int motorA1 = 3; // left motor backwards
const int motorA2 = 9; // left motor forwards
const int motorB1 = 10; // right motor forwards
const int motorB2 = 11; // right motor backwards
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
int motorAFullSpeed = 255;
int motorBFullSpeed = 255;
int motorStop = 0;
int colorBlack = 700;


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
  analogWrite(motorA2, motorAFullSpeed);
  analogWrite(motorB1, motorBFullSpeed);
  analogWrite(motorA1, motorStop);
  analogWrite(motorB2, motorStop);
}

void goBackwards() 
{
  analogWrite(motorA1, motorAFullSpeed);
  analogWrite(motorB2, motorBFullSpeed);
  analogWrite(motorA2, motorStop);
  analogWrite(motorB1, motorStop);
}
void stop()
{
    analogWrite(motorA1, motorStop);
    analogWrite(motorA2, motorStop);
    analogWrite(motorB1, motorStop);
    analogWrite(motorB2, motorStop);
} 

void adjustAngleOutside1() 
{
  if (lineSensorValue[5] <= lineSensorValue[2]) 
  {
    analogWrite(motorA2, motorAFullSpeed);
    analogWrite(motorB1, 40); 
  } 
  else if (lineSensorValue[2] <= lineSensorValue[5]) 
  {
    analogWrite(motorB1, motorBFullSpeed);
    analogWrite(motorA2, 40); 
  }
}

void adjustAngleOutside2() 
{
  if (lineSensorValue[6] <= lineSensorValue[1]) 
  {
    analogWrite(motorA2, motorAFullSpeed);
    analogWrite(motorB2, 20);
  } 
  else if (lineSensorValue[1] <= lineSensorValue[6]) 
  {
    analogWrite(motorB1, motorBFullSpeed);
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
    bool canGoForwards = (lineSensorValue[3] >= colorBlack) && (lineSensorValue[4] >= colorBlack);
    bool needToAdjustOutside1 = (lineSensorValue[2] >= colorBlack) || (lineSensorValue[5] >= colorBlack);
    bool needToAdjustOutside2 = (lineSensorValue[1] >= colorBlack) || (lineSensorValue[6] >= colorBlack);
    bool needToAdjustOutside3 = (lineSensorValue[0] >= colorBlack) || (lineSensorValue[7] >= colorBlack);
    if (distance <= 20 && distance >= 1)
    {
    }
  
      else if (millis() >= adjustMillis) 
      {
        adjustMillis = millis() + adjustInterval;
        Serial.println(canGoForwards);
        if (canGoForwards)
        {
          goForwards();
        }
        else if (needToAdjustOutside1) 
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
