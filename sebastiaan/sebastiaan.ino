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
int colorBlack = 900;
int colorWhite = 600;
int finishReached = 0;
bool allBlack;
bool allWhite;

// motors
const int motorA1 = 3; // left motor backwards
const int motorA2 = 9; // left motor forwards
const int motorB1 = 10; // right motor forwards
const int motorB2 = 11; // right motor backward
const int motorR1 = 4;
const int motorR2 = 13;
int motorAFullSpeed = 255;
int motorBFullSpeed = 255;
int motorStop = 0;
int motor1PulsesDone = 0; 
int motor2PulsesDone = 0;
int previous1Pulse = 2;
int previous2Pulse = 2;

// buttons
const int buttonPinA = 8;
const int buttonPinB = 2;
int buttonStateA;
int buttonStateB;

//gripper
#define gripper 5
#define gripperOpen 1600
#define gripperClosed 950
#define servoDelay 20

//millis
const int millisInterval = 10; 
unsigned long driveMillis;
unsigned long sonarMillis;
unsigned long buttonMillis;

//neopixel
#include <Adafruit_NeoPixel.h>
#define MAX_DISTANCE 200
#define NUM_PIXELS 4
#define PIXEL_PIN 13
#define IDLE_BREATHE_DURATION 800
Adafruit_NeoPixel pixels(NUM_PIXELS, PIXEL_PIN, NEO_RGB + NEO_KHZ800);

//NeoPixels Idle
unsigned long neoPixelsIdleLastUpdateTime = 0;
uint8_t neoPixelsIdleBrightness = 0;
int neoPixelsIdleDirection = 1;

//NeoPixels Backwards
unsigned long neoPixelsBackwardsPreviousMillis = 0;
const long neoPixelsBackwardsInterval = 1000;

//NeoPixels Left
unsigned long neoPixelsLeftPreviousMillis = 0;
const long neoPixelsLeftInterval = 500;

//NeoPixels Right
unsigned long neoPixelsRightPreviousMillis = 0;
const long neoPixelsRightInterval = 500;




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
   pinMode(gripper, OUTPUT);
   digitalWrite(gripper, LOW);
}

void loop() 
{
  readButtons();
  readSonar();
  readSensors();
//  readRotation();
  flagReset();
  drive();
}

void goForwards() 
{
  analogWrite(motorA2, motorAFullSpeed);
  analogWrite(motorB1, motorBFullSpeed);
  analogWrite(motorA1, motorStop);
  analogWrite(motorB2, motorStop);
  goForwardsNeoPixels();
//  Serial.println("forwards");
}

void goForwardsNeoPixels()
{
  setAllPixels(0, 255, 0);
}

void goBackwards() 
{
  analogWrite(motorA1, motorAFullSpeed);
  analogWrite(motorB2, motorBFullSpeed);
  analogWrite(motorA2, motorStop);
  analogWrite(motorB1, motorStop);
  Serial.println("backwards");
  goBackwardsNeoPixels();
}

void goBackwardsNeoPixels()
{
  unsigned long neoPixelsBackwardsCurrentMillis = millis();

  if (neoPixelsBackwardsCurrentMillis - neoPixelsBackwardsPreviousMillis >= neoPixelsBackwardsInterval) {
    neoPixelsBackwardsPreviousMillis = neoPixelsBackwardsCurrentMillis;

    static boolean neoPixelsBackwardsOn = false;
    if (neoPixelsBackwardsOn) {
      setAllPixels(0, 0, 0);
    } else {
      setAllPixels(255, 0, 0);
    }
    pixels.show();
    neoPixelsBackwardsOn = !neoPixelsBackwardsOn;
  }
}

void stopDriving()
{
   analogWrite(motorA1, motorStop);
   analogWrite(motorA2, motorStop);
   analogWrite(motorB1, motorStop);
   analogWrite(motorB2, motorStop);
   Serial.println("stopping");
} 

void goRightNeoPixels()
{
  pixels.setPixelColor(3, pixels.Color(0, 255, 0));
  pixels.setPixelColor(2, pixels.Color(255, 50, 0));
  pixels.show();
  pixels.setPixelColor(1, pixels.Color(255, 50, 0));
  pixels.show();
  pixels.setPixelColor(0, pixels.Color(0, 255, 0));
}

void goLeftNeoPixels()
{
  pixels.setPixelColor(3, pixels.Color(255, 50, 0));
  pixels.show();
  pixels.setPixelColor(2, pixels.Color(0, 255, 0));
  pixels.setPixelColor(1, pixels.Color(0, 255, 0));
  pixels.setPixelColor(0, pixels.Color(255, 50, 0));
  pixels.show();
} 

void adjustAngleOutside1() 
{
  if (lineSensorValue[5] <= lineSensorValue[2]) 
  {
    analogWrite(motorA2, motorAFullSpeed);
    analogWrite(motorB1, 40);
    Serial.println("adjusting1");
    goRightNeoPixels();
  } 
  else if (lineSensorValue[2] <= lineSensorValue[5]) 
  {
    analogWrite(motorB1, motorBFullSpeed);
    analogWrite(motorA2, 40); 
    Serial.println("adjusting1.2");
    goLeftNeoPixels();
  }
  else
  {
    setAllPixels(0, 255, 0);
  }
}

void adjustAngleOutside2() 
{
  if (lineSensorValue[6] <= lineSensorValue[1]) 
  {
    analogWrite(motorA2, motorAFullSpeed);
    analogWrite(motorB2, 20);
    Serial.println("adjusting2");
    goRightNeoPixels();
  } 
  else if (lineSensorValue[1] <= lineSensorValue[6]) 
  {
    analogWrite(motorB1, motorBFullSpeed);
    analogWrite(motorA2, 20);
    Serial.println("adjusting2.1");
    goLeftNeoPixels();
  }
  else
  {
    setAllPixels(0, 255, 0);
  }
}

void adjustAngleOutside3() 
{
  if (lineSensorValue[7] <= lineSensorValue[0]) 
  {
    analogWrite(motorA2, 100);
    analogWrite(motorB2, 0);
    Serial.println("adjusting3");
    goRightNeoPixels();
  } 
  else if (lineSensorValue[0] <= lineSensorValue[7]) 
  {
    analogWrite(motorB1, 100);
    analogWrite(motorA2, 0);
    Serial.println("adjusting3.1");
    goLeftNeoPixels();
  }
  else
  {
    setAllPixels(0, 255, 0);
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
      servo(gripperClosed);
      flagGone = 1;
    }
  }
}

void colorCheck()
{
   allBlack = (lineSensorValue[1] >= colorBlack) 
           && (lineSensorValue[2] >= colorBlack) 
           && (lineSensorValue[3] >= colorBlack) 
           && (lineSensorValue[4] >= colorBlack) 
           && (lineSensorValue[5] >= colorBlack) 
           && (lineSensorValue[6] >= colorBlack) 
           && (lineSensorValue[7] >= colorBlack) 
           && (lineSensorValue[0] >= colorBlack);
           
    allWhite = (lineSensorValue[1] <= colorWhite) 
           && (lineSensorValue[2] <= colorWhite) 
           && (lineSensorValue[3] <= colorWhite) 
           && (lineSensorValue[4] <= colorWhite) 
           && (lineSensorValue[5] <= colorWhite) 
           && (lineSensorValue[6] <= colorWhite) 
           && (lineSensorValue[7] <= colorWhite) 
           && (lineSensorValue[0] <= colorWhite);


  
}

void stopWhenNeeded()
{
  colorCheck();

  if (allBlack)
  {
    goForwards();
    delay(200);
    readSensors();
    colorCheck();

    if (allBlack)
    {
      servo(gripperOpen);
      stopDriving();
      delay(1000);
      goBackwards();
      delay(1000);
      stopDriving();
      delay(10000);
    }
  }
  if (allWhite)
  {
    goForwards();
    delay(500);
    readSensors();
    colorCheck();

    if (allWhite)
    {
      stopDriving();
      delay(100);
      goBackwards();
      delay(1000);
      stopDriving();
      delay(100);
    }
  }
}
void drive()
{
  if (flagGone == 1)
  {
    stopWhenNeeded();
    bool needToAdjustOutside1 = (lineSensorValue[2] >= colorBlack) || (lineSensorValue[5] >= colorBlack);
    bool needToAdjustOutside2 = (lineSensorValue[1] >= colorBlack) || (lineSensorValue[6] >= colorBlack);
    bool needToAdjustOutside3 = (lineSensorValue[0] >= colorBlack) || (lineSensorValue[7] >= colorBlack);
    if (distance <= 20 && distance >= 1)
    {
        Serial.println("program failure");
    }
    else if (millis() >= driveMillis) 
    {
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

void readRotation()
{
  int rotation1 = digitalRead(motorR1);
  int rotation2 = digitalRead(motorR2);
  if (previous1Pulse != rotation1)
    {
      motor1PulsesDone++;
      previous1Pulse = rotation1;
    }

    if (previous2Pulse != rotation2)
    {
      motor2PulsesDone++;
      previous2Pulse = rotation2;
    }
  int totalRotations = motor1PulsesDone + motor2PulsesDone;
  while (totalRotations < 10)
  {
    goForwards();
    Serial.println(totalRotations);
  }
} 

void servo(int pulse)
{
      digitalWrite(gripper, HIGH);
      delayMicroseconds(pulse);
      digitalWrite(gripper, LOW);
}

void setAllPixels(uint8_t red, uint8_t green, uint8_t blue) 
{
  for (int i = 0; i < pixels.numPixels(); i++)  
  {
    pixels.setPixelColor(i, pixels.Color(red, green, blue));
  }
  pixels.show();
}

void idleNeoPixels()
{
  unsigned long neoPixelsIdleCurrentUpdateTime = millis();
  
  // Check if it's time to update the brightness
  if (neoPixelsIdleCurrentUpdateTime - neoPixelsIdleLastUpdateTime >= IDLE_BREATHE_DURATION / 255) {
    // Update brightness level
    neoPixelsIdleBrightness += neoPixelsIdleDirection;
    
    // Check if brightness reaches the limits
    if (neoPixelsIdleBrightness == 0 || neoPixelsIdleBrightness == 255) {
      // Change direction when reaching the limits
      neoPixelsIdleDirection *= -1;
    }
    
    // Update all NeoPixels with the new brightness
    setAllPixels(neoPixelsIdleBrightness, 0, 0); // Red breathe effect
    
    // Update last update time
    neoPixelsIdleLastUpdateTime = neoPixelsIdleCurrentUpdateTime;
  }
}
