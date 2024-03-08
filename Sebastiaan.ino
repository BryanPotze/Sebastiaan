// sonar
#include <NewPing.h>
#define TRIG_PIN 7
#define ECHO_PIN 6
#define MAX_DISTANCE 200
NewPing sonar(TRIG_PIN, ECHO_PIN, MAX_DISTANCE);
int flagGone = 0;
int distance = 1;

//linesensor
const int lineSensor[] = {A0, A1, A2, A3, A4, A5, A6, A7};
int lineSensorValue[8] = {0};
int colorBlack = 800;
int colorWhite = 600;
int finishReached = 0;
int maxSensorIndex = 0;

// motors
const int motorA1 = 3; // left motor backwards
const int motorA2 = 9; // left motor forwards
const int motorB1 = 10; // right motor forwards
const int motorB2 = 11; // right motor backward
int motorAFullSpeed = 255;
int motorBFullSpeed = 255;
int motorStop = 0;


// buttons
const int buttonPinA = 8;
const int buttonPinB = 2;
int buttonStateA;
int buttonStateB;

//gripper
const int gripper = 5;

//millis
const int millisInterval = 5; 
const int stopWhite = 1000;
unsigned long driveMillis;
unsigned long sonarMillis;
unsigned long buttonMillis;
int stopWhiteMillis;





void setup() 
{
  Serial.begin(9600);
  pinMode(motorA1, OUTPUT);
  pinMode(motorA2, OUTPUT);
  pinMode(motorB1, OUTPUT);
  pinMode(motorB2, OUTPUT);
  pinMode(buttonPinA, INPUT);
  pinMode(buttonPinB, INPUT);
  for (int i = 0; i < 7; i++) 
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
  turnBackToWhite();
  drive();
}

void goForwards() 
{
  analogWrite(motorA2, motorAFullSpeed);
  analogWrite(motorB1, motorBFullSpeed);
  analogWrite(motorA1, motorStop);
  analogWrite(motorB2, motorStop);
  Serial.println("forwards");
}

void goBackwards() 
{
  analogWrite(motorA1, motorAFullSpeed);
  analogWrite(motorB2, motorBFullSpeed);
  analogWrite(motorA2, motorStop);
  analogWrite(motorB1, motorStop);
  Serial.println("backwards");
}
void stopDriving()
{
   analogWrite(motorA1, motorStop);
   analogWrite(motorA2, motorStop);
   analogWrite(motorB1, motorStop);
   analogWrite(motorB2, motorStop);
   Serial.println("stopping");
} 

void adjustAngleOutside1() 
{
  if (lineSensorValue[5] <= lineSensorValue[2]) 
  {
    analogWrite(motorA2, motorAFullSpeed);
    analogWrite(motorB1, 100);
    Serial.println("adjusting1");
  } 
  else if (lineSensorValue[2] <= lineSensorValue[5]) 
  {
    analogWrite(motorB1, motorBFullSpeed);
    analogWrite(motorA2, 100); 
    Serial.println("adjusting1.2");
  }
}

void adjustAngleOutside2() 
{
  if (lineSensorValue[6] <= lineSensorValue[1]) 
  {
    analogWrite(motorA2, motorAFullSpeed);
    analogWrite(motorB2, 20);
    Serial.println("adjusting2");
  } 
  else if (lineSensorValue[1] <= lineSensorValue[6]) 
  {
    analogWrite(motorB1, motorBFullSpeed);
    analogWrite(motorA2, 20);
    Serial.println("adjusting2.1");
  }
}

void adjustAngleOutside3() 
{
  if (lineSensorValue[7] <= lineSensorValue[0]) 
  {
    analogWrite(motorA2, 100);
    analogWrite(motorB2, 0);
    Serial.println("adjusting3");
  } 
  else if (lineSensorValue[0] <= lineSensorValue[7]) 
  {
    analogWrite(motorB1, 100);
    analogWrite(motorA2, 0);
    Serial.println("adjusting3.1");
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
void turnBackToWhite() 
{
  bool allSensorsBelow800 = true;

  for (int i = 0; i < 8; i++)
  {
    if (lineSensorValue[i] >= 800)
    {
      allSensorsBelow800 = false;
      break;
    }
  }

  if (allSensorsBelow800)
  {
    analogWrite(motorB1, motorStop);
    analogWrite(motorA2, motorStop);

    for (int i = 1; i < 8; i++)
    {
      if (lineSensorValue[i] > lineSensorValue[maxSensorIndex])
      {
        maxSensorIndex = i;
      }
    }

    if (maxSensorIndex == 0)
    {
      if (millis() >= stopWhiteMillis)
      {
        analogWrite(motorA2, motorStop);
        analogWrite(motorB1, motorBFullSpeed);
      }
      else
      {
        stopWhiteMillis = millis() + 1000; 
      }
    }
    else
    {
      if (millis() >= stopWhiteMillis)
      {
        analogWrite(motorA2, motorAFullSpeed);
        analogWrite(motorB1, motorStop);
      }
      else
      {
        stopWhiteMillis = millis() + 1000; 
      }
    }
  }
}
//void suicidePrevention()
//{
//  bool aboutToCommitSuicide = (lineSensorValue[1] >= colorBlack) 
//                             && (lineSensorValue[2] >= colorBlack) 
//                             && (lineSensorValue[3] >= colorBlack) 
//                             && (lineSensorValue[4] >= colorBlack) 
//                             && (lineSensorValue[5] >= colorBlack) 
//                             && (lineSensorValue[6] >= colorBlack) 
//                             && (lineSensorValue[7] >= colorBlack) 
//                             && (lineSensorValue[0] >= colorBlack);
//
//  if (aboutToCommitSuicide)
//  {
//    goForwards();
//    delay(50);
//
//    aboutToCommitSuicide = (lineSensorValue[1] >= colorBlack) 
//                        && (lineSensorValue[2] >= colorBlack) 
//                        && (lineSensorValue[3] >= colorBlack) 
//                        && (lineSensorValue[4] >= colorBlack) 
//                        && (lineSensorValue[5] >= colorBlack) 
//                        && (lineSensorValue[6] >= colorBlack) 
//                        && (lineSensorValue[7] >= colorBlack) 
//                        && (lineSensorValue[0] >= colorBlack);
//
//    if (aboutToCommitSuicide)
//    {
//      stopDriving();
//      delay(1000);
//      goBackwards();
//      delay(1000);
//      stopDriving();
//      delay(10000);
//    }
//  }
//}
void drive()
{
  if (flagGone == 1)
  {
//    suicidePrevention();
    bool needToAdjustOutside1 = (lineSensorValue[2] >= colorBlack) || (lineSensorValue[5] >= colorBlack);
    bool needToAdjustOutside2 = (lineSensorValue[1] >= colorBlack) || (lineSensorValue[6] >= colorBlack);
    bool needToAdjustOutside3 = (lineSensorValue[0] >= colorBlack) || (lineSensorValue[7] >= colorBlack);
    if (distance <= 20 && distance >= 1)
    {
        Serial.println("program failure");
    }
    else if (millis() >= driveMillis) 
    {
      Serial.println(driveMillis);
      driveMillis = millis() + millisInterval;
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
    else
    {
      Serial.println("this shouldnt happen");
    }
  }
  else
  {
    stopDriving();
    Serial.println("program failure");
  } 
}

void readSonar()
{
   if (millis() >= sonarMillis) 
   {
      sonarMillis = millis() + 200;
      distance = sonar.ping_cm();
   }
}

void readButtons()
{
   if (millis() >= buttonMillis) 
   {
      buttonMillis = millis() + millisInterval;
      buttonStateA = digitalRead(buttonPinA);
      buttonStateB = digitalRead(buttonPinB);
   }
}
