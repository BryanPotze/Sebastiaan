// sonar
#include <NewPing.h> //sonar library
#define TRIG_PIN 7 // recieve pin
#define ECHO_PIN 12 // send pin
#define MAX_DISTANCE 200 //maximum distance of reading
NewPing sonar(TRIG_PIN, ECHO_PIN, MAX_DISTANCE);
int flagGone = 0;
int distance = 1; 

//linesensor
const int lineSensor[] = {A0, A1, A2, A3, A4, A5, A6, A7}; // array for the line line sensor pins
int lineSensorValue[8] = {0}; // array for saving the values from the line sensor
int colorBlack = 850; // value for black
int colorWhite = 700; // value for white
bool allBlack;
bool allWhite; 

// motors
const int MOTOR_A1 = 5; // left motor backward
const int MOTOR_A2 = 9; // left motor forward
const int MOTOR_B1 = 10; // right motor forward
const int MOTOR_B2 = 6; // right motor backward
const int MOTOR_R1 = 2; // rotation sensor left
const int MOTOR_R2 = 3; // rotation sensor right
int r1Rotations = 0;
int r2Rotations = 0;
int lastSensor = 0; 
//motor speeds
#define MOTOR_A_FULL_SPEED = 255; 
#define MOTOR_A_STOP = 0;
#define MOTOR_B_FULL_SPEED = 255;
#define MOTOR_B_STOP = 0;



// buttons
#define buttonPinA = 8; //pin for B1
#define buttonPinB = 11; // pin for B2
int buttonStateA;
int buttonStateB;

//gripper
#define gripper 4 // gripper pin
#define gripperOpen 1600 // value for gripper being open
#define gripperClosed 950 // value for gripper being closed
#define servoDelay 20 

//millis
const int millisInterval = 5; 
unsigned long driveMillis;
unsigned long sonarMillis;
unsigned long buttonMillis;

//neopixel
#include <Adafruit_NeoPixel.h> //neopixel library
#define NUM_PIXELS 4 //number of neopixels
#define PIXEL_PIN 13 //pin for the neopixels
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




void setup() {
  Serial.begin(9600);
  pixels.begin();
  pinMode(MOTOR_A1, OUTPUT);
  pinMode(MOTOR_A2, OUTPUT);
  pinMode(MOTOR_B1, OUTPUT);
  pinMode(MOTOR_B2, OUTPUT);
  pinMode(buttonPinA, INPUT);
  pinMode(buttonPinB, INPUT);
  for (int i = 0; i < 7; i++) {
    pinMode(lineSensor[i], INPUT);  // sets every linesensor pin on the correct pinmode
  }
   pinMode(gripper, OUTPUT);
   digitalWrite(gripper, LOW);
   attachInterrupt(digitalPinToInterrupt(MOTOR_R1), rotateR1, CHANGE); //interrupt activates when rotation sensor changes
   attachInterrupt(digitalPinToInterrupt(MOTOR_R2), rotateR2, CHANGE); //interrupt activates when rotation sensor changes
}

void loop() {
  flagReset(); //function to reset Sebastiaan to its starting phase
  readButtons(); //function to read button states
  readSonar(); // function for reading the sonar state
  readSensors(); // function for reading the line sensor state
  drive(); // function for driving
}

//sets the forward motors on the set speed while stopping the motors that would make sebastiaan go backwards
void goForwards() {
  analogWrite(MOTOR_A2, MOTOR_A_FULL_SPEED);
  analogWrite(MOTOR_B1, MOTOR_B_FULL_SPEED);
  analogWrite(MOTOR_A1, MOTOR_A_STOP);
  analogWrite(MOTOR_B2, MOTOR_B_STOP);
  goForwardsNeoPixels(); 
  servo(gripperClosed); //closes the gripper
}

//neopixel configuration for going forwards
void goForwardsNeoPixels() {
  setAllPixels(0, 255, 0); 
}

//sets the backwards motors on the set speed while stopping the motors that would make sebastiaan go forwards
void goBackwards() {
  analogWrite(MOTOR_A1, MOTOR_A_FULL_SPEED);
  analogWrite(MOTOR_B2, MOTOR_B_FULL_SPEED);
  analogWrite(MOTOR_A2, MOTOR_B_STOP);
  analogWrite(MOTOR_B1, MOTOR_B_STOP);
  goBackwardsNeoPixels();
}

//sets the neopixels for when driving backwards
void goBackwardsNeoPixels() {
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

//stops all the motors
void stopDriving() {
   analogWrite(MOTOR_A1, MOTOR_A_STOP);
   analogWrite(MOTOR_A2, MOTOR_A_STOP);
   analogWrite(MOTOR_B1, MOTOR_B_STOP);
   analogWrite(MOTOR_B2, MOTOR_B_STOP);
} 

//sets the right most neopixels
void goRightNeoPixels() {
  pixels.setPixelColor(3, pixels.Color(0, 255, 0));
  pixels.setPixelColor(2, pixels.Color(255, 50, 0));
  pixels.show();
  pixels.setPixelColor(1, pixels.Color(255, 50, 0));
  pixels.show();
  pixels.setPixelColor(0, pixels.Color(0, 255, 0));
}

//sets the left most neopixels
void goLeftNeoPixels() {
  pixels.setPixelColor(3, pixels.Color(255, 50, 0));
  pixels.show();
  pixels.setPixelColor(2, pixels.Color(0, 255, 0));
  pixels.setPixelColor(1, pixels.Color(0, 255, 0));
  pixels.setPixelColor(0, pixels.Color(255, 50, 0));
  pixels.show();
} 

//adjusts the robot a little based on which linesensor is higher
void adjustAngleOutside1() {
  if (lineSensorValue[5] <= lineSensorValue[2]) {
    analogWrite(MOTOR_A2, MOTOR_A_FULL_SPEED);
    analogWrite(MOTOR_B1,30);
    Serial.println("adjusting1");
    goRightNeoPixels();
    lastSensor = 1;
  } 
  else if (lineSensorValue[2] <= lineSensorValue[5]) {
    analogWrite(MOTOR_B1, MOTOR_B_FULL_SPEED);
    analogWrite(MOTOR_A2, 30); 
    Serial.println("adjusting1.2");
    goLeftNeoPixels();
    lastSensor = 2;
  } else {
    setAllPixels(0, 255, 0);
  }
}

//adjusts the robot a bit more based on which linesensor is higher
void adjustAngleOutside2() 
{
  if (lineSensorValue[6] <= lineSensorValue[1]) {
    analogWrite(MOTOR_A2, MOTOR_A_FULL_SPEED);
    analogWrite(MOTOR_B1, 60);
    Serial.println("adjusting2");
    goRightNeoPixels();
    lastSensor = 1;
  } 
  else if (lineSensorValue[1] <= lineSensorValue[6]) {
    analogWrite(MOTOR_B1, MOTOR_B_FULL_SPEED);
    analogWrite(MOTOR_A2, 60);
    Serial.println("adjusting2.1");
    goLeftNeoPixels();
    lastSensor = 2;
  }
  else{
    setAllPixels(0, 255, 0);
  }
}
//adjusts the robot a lot based on which linesensor is higher
void adjustAngleOutside3() {
  if (lineSensorValue[7] <= lineSensorValue[0]) {
    analogWrite(MOTOR_A2, MOTOR_A_FULL_SPEED);
    analogWrite(MOTOR_B1, MOTOR_B_STOP);
    analogWrite(MOTOR_B2, 110);
    analogWrite(MOTOR_A1, MOTOR_A_STOP);
    Serial.println("adjusting3");
    goRightNeoPixels();
    lastSensor = 1;
  } 
  else if (lineSensorValue[0] <= lineSensorValue[7]) {
    analogWrite(MOTOR_B1, MOTOR_B_FULL_SPEED);
    analogWrite(MOTOR_A2, MOTOR_A_STOP);
    analogWrite(MOTOR_B2, MOTOR_B_STOP);
    analogWrite(MOTOR_A1, 110);
    Serial.println("adjusting3.1");
    goLeftNeoPixels();
    lastSensor = 2;
  }
  else{
    setAllPixels(0, 255, 0);
  }
}

//reads the linesensor and stores all the values in the array
void readSensors() 
{
  for (int i = 0; i < 8; i++) {
    lineSensorValue[i] = analogRead(lineSensor[i]);
  }
}


void flagReset()
{
  readButtons(); //reads the button states and sets flaggone to 0 if needed
  if (flagGone == 0) {
    if (distance >= 20 || distance == 0) { //looks at if theres an object more than 20cm away from the sonar
      flagGone = 1;
      startSequence();
    }
    else {
      Serial.println("Standing at flag");
    }
  }
}

void colorCheck() {
   allBlack = (lineSensorValue[1] >= colorBlack) 
           && (lineSensorValue[2] >= colorBlack) 
           && (lineSensorValue[3] >= colorBlack) 
           && (lineSensorValue[4] >= colorBlack) 
           && (lineSensorValue[5] >= colorBlack) 
           && (lineSensorValue[6] >= colorBlack) 
           && (lineSensorValue[7] >= colorBlack) 
           && (lineSensorValue[0] >= colorBlack); //true if all the line sensor bits are looking at black
           
    allWhite = (lineSensorValue[1] <= colorWhite) 
           && (lineSensorValue[2] <= colorWhite) 
           && (lineSensorValue[3] <= colorWhite) 
           && (lineSensorValue[4] <= colorWhite) 
           && (lineSensorValue[5] <= colorWhite) 
           && (lineSensorValue[6] <= colorWhite) 
           && (lineSensorValue[7] <= colorWhite) 
           && (lineSensorValue[0] <= colorWhite); //true if all the line sensor bits are looking at white
  
}

//function for stopping the motor at the end of the track
void stopWhenNeeded() {
  colorCheck();
  readButtons();
  if (flagGone == 1) {
    //drives forwards for 200ms when the sensors see all black
    if (allBlack) {
      goForwards();
      delay(100);
      readSensors();
      colorCheck();
      delay(100);
      readSensors();
      colorCheck();
      
      //looks if it sees black again, then stops before doing the end routine
      if (allBlack) {
        stopDriving();
        delay(500);
        goBackwards();
        delay(300);
        servo(gripperOpen);
        goBackwards();
        delay(1000);
        stopDriving();
        int randomColors = 0;
        while (randomColors < 200) { //randomized the neopixels 200 times
          setRandomNeoPixels();
          randomColors++;
        }
        setAllPixels(0,0,0);
        delay(10000);
      }
    }

    //goes back to the last read linesensor when all linesensor bits see white
    if (allWhite) {
        Serial.println(lastSensor);
        if (lastSensor == 1) {
          analogWrite(MOTOR_A2, 255);
          analogWrite(MOTOR_B1, 0);
          analogWrite(MOTOR_B2, 255);
          analogWrite(MOTOR_A1, 0);
          goRightNeoPixels();
        }
        if (lastSensor == 2) {
          analogWrite(MOTOR_B1, 255);
          analogWrite(MOTOR_A2, 0);
          analogWrite(MOTOR_B2, 0);
          analogWrite(MOTOR_A1, 255);
          goLeftNeoPixels();
        }
    }
  }
}
 
//drives around the object then goes back the other way till it sees the line
void goAroundObject() {
    stopDriving();
    unsigned long startTime = millis();
    goBackwards();
    while (millis() - startTime < 250) {
    }

    startTime = millis();
    while (millis() - startTime < 1000) {
        analogWrite(MOTOR_A2, 160);
        analogWrite(MOTOR_B1, 255); 
        analogWrite(MOTOR_B2, 0);
        analogWrite(MOTOR_A1, 0);
    }

    bool blackDetected = false;

    while (!blackDetected) {
        startTime = millis();
        while (millis() - startTime < 2000){
            readSensors();
            if (anyBlack()) {
                blackDetected = true;
                break; 
            }

            analogWrite(MOTOR_A2, 255); 
            analogWrite(MOTOR_B1, 130); 
            analogWrite(MOTOR_B2, 0);
            analogWrite(MOTOR_A1, 0);
        }

        if (!blackDetected) 
        {
            while (!anyBlack()) 
            {
                analogWrite(MOTOR_A2, 255); 
                analogWrite(MOTOR_B1, 110); 
                analogWrite(MOTOR_B2, 0);
                analogWrite(MOTOR_A1, 0);
                readSensors();
            }
        }
    }

    stopDriving();
    lastSensor = 1;
}

// looks if one of the line sensor bits see black
bool anyBlack() {
    for (int i = 0; i < 8; i++) 
    {
        if (lineSensorValue[i] >= colorBlack) 
        {
            return true;
        }
    }
    return false;
}

// main driving function
void drive() {
  if (flagGone == 1) {
    stopWhenNeeded();
    bool goForwards1 = (lineSensorValue[0] >=colorBlack) && (lineSensorValue[1] >=colorBlack); //true if the middle sensors see black
    bool needToAdjustOutside1 = (lineSensorValue[2] >= colorBlack) || (lineSensorValue[5] >= colorBlack); // true if any of the outer bits see black
    bool needToAdjustOutside2 = (lineSensorValue[1] >= colorBlack) || (lineSensorValue[6] >= colorBlack); // true if any of the more outer bits see black
    bool needToAdjustOutside3 = (lineSensorValue[0] >= colorBlack) || (lineSensorValue[7] >= colorBlack); // true if any of the most outer bits see black
    if (distance <= 20 && distance >= 1) { 

      //looks multiple times at the distance as a failsave before going around the object if one is detected infront
      for (int i = 0; i < 11; i++) {
        if (!distance <=20 && !distance >=1) {
          break;
        }
        if (i == 10) {
          goAroundObject();
        } 
        delay(10);
      }
    }
    //calls the needed functions when the bool is true
    else if (millis() >= driveMillis) {
      driveMillis = millis() + millisInterval;
      if (goForwards1) {
        goForwards();
      }
      else if (needToAdjustOutside1) { 
        adjustAngleOutside1();
      } 
      else if (needToAdjustOutside2) {
        adjustAngleOutside2();
      } 
      else if (needToAdjustOutside3) {
        adjustAngleOutside3();
      } 
      else {
        goForwards();
      }
    }
  }
}

//reads the sonar
void readSonar() {
   if (millis() >= sonarMillis) {
      sonarMillis = millis() + 200;
      distance = sonar.ping_cm();
   }
}

//reads the button states
void readButtons() {
   if (millis() >= buttonMillis) {
      buttonMillis = millis() + millisInterval;
      buttonStateA = digitalRead(buttonPinA);
      buttonStateB = digitalRead(buttonPinB);
   }
  if (buttonStateA == LOW) {
    flagGone = 0;
  }
}

//adds to r1Rotations on interrupt
void rotateR1() {
    r1Rotations++;
}

//adds to r2Rotations on interrupt
void rotateR2() {
  r2Rotations++;
}

//start of the track
void startSequence() {

  if (flagGone == 1) {
    r1Rotations = 0;

    //makes it so sebastiaan doesnt drive if theres a flag infront of him
    while(distance < 3) {
      readSonar(); 
    }

    //drives forwards untill r1Rotations gets set to 60
    while ((r1Rotations < 60) && (flagGone == 1)) { 
        readButtons();
        static bool startupDone = false;
        if (!startupDone) {
          //slowly activates the motors
          for (int i = 100; i < 200; i++) {
            readButtons();
            analogWrite(MOTOR_A2, i);
            analogWrite(MOTOR_B1, i);
            analogWrite(MOTOR_A1, MOTOR_A_STOP);
            analogWrite(MOTOR_B2, MOTOR_B_STOP);
          }
          startupDone = true;
        }
  
        analogWrite(MOTOR_A2, 250);
        analogWrite(MOTOR_B1, 250);
        analogWrite(MOTOR_A1, MOTOR_A_STOP);
        analogWrite(MOTOR_B2, MOTOR_B_STOP);
    }
      delay(100);
      stopDriving();
      r2Rotations = 0;
      servo(gripperClosed); //closes the gripper
    
    //spins to the left untill r2Rotations gets to 25
    while ((r2Rotations < 25) && (flagGone == 1)) {
      Serial.println(r2Rotations);
      readButtons();
      analogWrite(MOTOR_A1, 255);
      analogWrite(MOTOR_B1, 255);
      analogWrite(MOTOR_A2, 0);
      analogWrite(MOTOR_B2, 0);
    }
  }  
} 

//sets the gripper
void servo(int pulse) {
      for (int i = 0; i < 8; i++) {
        digitalWrite(gripper, HIGH);
        delayMicroseconds(pulse);
        digitalWrite(gripper, LOW);
      }

}

//sets the neopixels
void setAllPixels(uint8_t red, uint8_t green, uint8_t blue) {
  for (int i = 0; i < pixels.numPixels(); i++)  {
    pixels.setPixelColor(i, pixels.Color(red, green, blue));
  }
  pixels.show();
}

//neopixels when idle
void idleNeoPixels() {
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

//random neopixels
void setRandomNeoPixels()
{
  for(int i=0; i < NUM_PIXELS; i++)
  {
    uint32_t randomColor = pixels.Color(random(256), random(256), random(256));

    pixels.setPixelColor(i, randomColor);
  }

  pixels.show();
  delay(500);
}
