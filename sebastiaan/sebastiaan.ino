// sonar
#include <NewPing.h>
#define TRIG_PIN 7
#define ECHO_PIN 12
#define MAX_DISTANCE 200
NewPing sonar(TRIG_PIN, ECHO_PIN, MAX_DISTANCE);
int flagGone = 0;
int distance = 1;

//linesensor
const int lineSensor[] = {A0, A1, A2, A3, A4, A5, A6, A7};
int lineSensorValue[8] = {0};
int colorBlack = 900;
int colorWhite = 700;
int finishReached = 0;
bool allBlack;
bool allWhite; 

// motors
const int motorA1 = 5; // left motor backwards
const int motorA2 = 9; // left motor forwards
const int motorB1 = 10; // right motor forwards
const int motorB2 = 6; // right motor backward
const int motorR1 = 2;
const int motorR2 = 3;
int r1Rotations = 0;
int r2Rotations = 0;
int motorAFullSpeed = 255;
int motorBFullSpeed = 255;
int motorStop = 0;
int lastSensor = 0;


// buttons
const int buttonPinA = 8;
const int buttonPinB = 11;
int buttonStateA;
int buttonStateB;

//gripper
#define gripper 4
#define gripperOpen 1600
#define gripperClosed 950
#define servoDelay 20

//millis
const int millisInterval = 30; 
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
  pixels.begin();
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
   attachInterrupt(digitalPinToInterrupt(motorR1), rotateR1, CHANGE);
   attachInterrupt(digitalPinToInterrupt(motorR2), rotateR2, CHANGE);
}

void loop() 
{
  flagReset();
  readButtons();
  readSonar();
  readSensors();
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
    analogWrite(motorB1,70);
    Serial.println("adjusting1");
    goRightNeoPixels();
    lastSensor = 1;
  } 
  else if (lineSensorValue[2] <= lineSensorValue[5]) 
  {
    analogWrite(motorB1, motorBFullSpeed);
    analogWrite(motorA2, 70); 
    Serial.println("adjusting1.2");
    goLeftNeoPixels();
    lastSensor = 2;
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
    lastSensor = 1;
  } 
  else if (lineSensorValue[1] <= lineSensorValue[6]) 
  {
    analogWrite(motorB1, motorBFullSpeed);
    analogWrite(motorA2, 20);
    Serial.println("adjusting2.1");
    goLeftNeoPixels();
    lastSensor = 2;
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
    analogWrite(motorA2, 255);
    analogWrite(motorB1, 0);
    analogWrite(motorB2, 110);
    analogWrite(motorA1, 0);
    Serial.println("adjusting3");
    goRightNeoPixels();
    lastSensor = 1;
  } 
  else if (lineSensorValue[0] <= lineSensorValue[7]) 
  {
    analogWrite(motorB1, 255);
    analogWrite(motorA2, 0);
    analogWrite(motorB2, 0);
    analogWrite(motorA1, 110);
    Serial.println("adjusting3.1");
    goLeftNeoPixels();
    lastSensor = 2;
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
  readButtons();
  if (flagGone == 0)
  {
    if (distance >= 20 || distance == 0)
    {
      flagGone = 1;
      Serial.println(distance);
      startSequence();
    }
    else
    {
    
      Serial.println("Standing at flag");
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

    if (allBlack)
    {
      Serial.println(lineSensorValue[4]);
    }
           
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
  readButtons();
  if (flagGone == 1)
  {
    if (allBlack)
    {
      goForwards();
      delay(100);
      readSensors();
      colorCheck();
      Serial.println(allBlack);
      delay(100);
      readSensors();
      colorCheck();
      Serial.println(allBlack);
      
  
      if (allBlack)
      {
        Serial.println("why no please stop");
        
        stopDriving();
        delay(500);
        goBackwards();
        delay(300);
        servo(gripperOpen);
        goBackwards();
        delay(1000);
        stopDriving();
        delay(10000);
      }
    }
    if (allWhite)
    {
        Serial.println(lastSensor);
        if (lastSensor == 1)
        {
          analogWrite(motorA2, motorAFullSpeed);
          analogWrite(motorB1, 0);
          Serial.println("adjusting1");
          goRightNeoPixels();
        }
        if (lastSensor == 2)
        {
          analogWrite(motorA2, 0);
          analogWrite(motorB1, motorAFullSpeed);
          Serial.println("adjusting1");
          goLeftNeoPixels();
        }
    }
  }
}
 
void goAroundObject() 
{
    stopDriving();
    unsigned long startTime = millis();
    goBackwards();
    while (millis() - startTime < 250) 
    {
    }

    startTime = millis();
    while (millis() - startTime < 1000) 
    {
        analogWrite(motorA2, 160);
        analogWrite(motorB1, 255); 
        analogWrite(motorB2, 0);
        analogWrite(motorA1, 0);
    }

    bool blackDetected = false;

    while (!blackDetected) 
    {
        startTime = millis();
        while (millis() - startTime < 500)
        {
            readSensors();
            if (anyBlack()) 
            {
                blackDetected = true;
                break; 
            }

            analogWrite(motorA2, 255); 
            analogWrite(motorB1, 150); 
            analogWrite(motorB2, 0);
            analogWrite(motorA1, 0);
        }

        if (!blackDetected) 
        {
            while (!anyBlack()) 
            {
                analogWrite(motorA2, 255); 
                analogWrite(motorB1, 120); 
                analogWrite(motorB2, 0);
                analogWrite(motorA1, 0);
                readSensors();
            }
        }
    }

    stopDriving();
    lastSensor = 1;
}

bool anyBlack() 
{
    for (int i = 0; i < 8; i++) 
    {
        if (lineSensorValue[i] >= colorBlack) 
        {
            return true;
        }
    }
    return false;
}

void drive()
{
  if (flagGone == 1)
  {
    stopWhenNeeded();
    bool goForwards1 = (lineSensorValue[0] >=colorBlack) && (lineSensorValue[1] >=colorBlack);
    bool needToAdjustOutside1 = (lineSensorValue[2] >= colorBlack) || (lineSensorValue[5] >= colorBlack);
    bool needToAdjustOutside2 = (lineSensorValue[1] >= colorBlack) || (lineSensorValue[6] >= colorBlack);
    bool needToAdjustOutside3 = (lineSensorValue[0] >= colorBlack) || (lineSensorValue[7] >= colorBlack);
    if (distance <= 20 && distance >= 1)
    {
      for (int i = 0; i < 11; i++)
      {
        if (!distance <=20 && !distance >=1)
        {
          break;
        }
        if (i == 10)
        {
          goAroundObject();
        } 
        delay(10);
      }
    }
    else if (millis() >= driveMillis) 
    {
      driveMillis = millis() + millisInterval;
      if (goForwards1)
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
    stopDriving();
    Serial.println("program failure");
    flagReset();
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
  if (buttonStateA == LOW)
  {
    flagGone = 0;
  }
}

void rotateR1() 
{
    r1Rotations++;
}

void rotateR2() 
{
  r2Rotations++;
}

void startSequence()
{

  if (flagGone == 1)
  {
    r1Rotations = 0;
    while(distance < 3)
    {
      readSonar();
    }
    while ((r1Rotations < 60) && (flagGone == 1))
    {
        readButtons();
        static bool startupDone = false;
        if (!startupDone)
        {
          for (int i = 100; i < 200; i++)
          {
            readButtons();
            analogWrite(motorA2, i);
            analogWrite(motorB1, i);
            analogWrite(motorA1, motorStop);
            analogWrite(motorB2, motorStop);
          }
          startupDone = true;
        }
  
        analogWrite(motorA2, 250);
        analogWrite(motorB1, 250);
        analogWrite(motorA1, motorStop);
        analogWrite(motorB2, motorStop);
        Serial.println(r1Rotations);
    }
      delay(100);
      stopDriving();
      r2Rotations = 0;
      servo(gripperClosed);
  
    while ((r2Rotations < 25) && (flagGone == 1))
    {
      Serial.println(r2Rotations);
      readButtons();
      analogWrite(motorA1, 255);
      analogWrite(motorB1, 255);
      analogWrite(motorA2, motorStop);
      analogWrite(motorB2, motorStop);
    }
    Serial.println("Done");
  }  
} 

void servo(int pulse)
{
      for (int i = 0; i < 8; i++)
      {
        digitalWrite(gripper, HIGH);
        delayMicroseconds(pulse);
        digitalWrite(gripper, LOW);
      }

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
  
  if (neoPixelsIdleCurrentUpdateTime - neoPixelsIdleLastUpdateTime >= IDLE_BREATHE_DURATION / 255) {

    neoPixelsIdleBrightness += neoPixelsIdleDirection;
    
    if (neoPixelsIdleBrightness == 0 || neoPixelsIdleBrightness == 255) {
      neoPixelsIdleDirection *= -1;
    }

    setAllPixels(neoPixelsIdleBrightness, 0, 0); 
    

    neoPixelsIdleLastUpdateTime = neoPixelsIdleCurrentUpdateTime;
  }
}
