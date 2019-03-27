#include <Pixy2.h>
#include <NewPing.h>
#include <PID_v1.h>

#define TRIGGER_PIN  7  // Arduino pin tied to trigger pin on the ultrasonic sensor.
#define ECHO_PIN     6  // Arduino pin tied to echo pin on the ultrasonic sensor.
#define MAX_DISTANCE 200 // Maximum distance we want to ping for (in centimeters). Maximum sensor distance is rated at 400-500cm.

NewPing sonar(TRIGGER_PIN, ECHO_PIN, MAX_DISTANCE); // NewPing setup of pins and maximum distance.

// This is the main Pixy object
Pixy2 pixy;

//audio trigger pins
const int ID = A2;
const int gotcha = A0;
const int bleep = A1;
const int bloop = A3;

//variables
int droid = 1;
int direct = 0;
int centered = 0;
double deviation;
double pixyInput;
double targetValue;
unsigned long scanTimer = millis();
unsigned long turnTimer = millis();

//track motor pins
const int trackRightPos = 2;
const int trackRightNeg = 8;
const int trackLeftPos = 3;
const int trackLeftNeg = 9;

//elevator pin
const int uppy = 4;

//firing pin
const int gun = 5;

//pwm calculation
float dutyCycle = 80.0;
float frequency = 100.0;
float onFactor = (dutyCycle / 100.0);
float offFactor = (100.0 - dutyCycle) / 100.0;
float onTime = (1000.0 / frequency) * onFactor;
float offTime = (1000.0 / frequency) * offFactor;

//Define Variables we'll be connecting to
double Setpoint, Input, Output;

//Specify the links and initial tuning parameters
double Kp = 1, Ki = 0.01, Kd = 1; ///////////////////////////////////////TUNE PID CONTROL LOOP HERE
PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);

void setup()
{
  Serial.begin(115200);

  pinMode(gun, OUTPUT);
  pinMode(uppy, OUTPUT);

  pinMode(ID, OUTPUT);
  pinMode(gotcha, OUTPUT);
  pinMode(bleep, OUTPUT);
  pinMode(bloop, OUTPUT);

  digitalWrite(ID, HIGH);
  digitalWrite(gotcha, HIGH);
  digitalWrite(bleep, HIGH);
  digitalWrite(bloop, HIGH);

  pinMode(trackRightPos, OUTPUT);//track1
  pinMode(trackRightNeg, OUTPUT);

  pinMode(trackLeftPos, OUTPUT);//track2
  pinMode(trackLeftNeg, OUTPUT);

  digitalWrite(trackLeftPos, HIGH); digitalWrite(trackLeftNeg, LOW); digitalWrite(trackRightPos, LOW); digitalWrite(trackRightNeg, HIGH); //forward, full speed

  pixy.init(); //initialize pixy

  Input = pixy.ccc.blocks[0].m_x; //input is the x value of the first target aquired by the pixy
  Setpoint = 160; //put target in center of pixy screen. Pixy x value goes from 0 to 320

  //turn the PID on
  myPID.SetMode(AUTOMATIC);
}

void loop()
{
  //Serial.println(sonar.ping_cm());
  int i;
  // grab blocks!
  pixy.ccc.getBlocks();

  if (pixy.ccc.numBlocks && centered == 0) //if we have a target and it isnt centered
  {
    pixyInput = pixy.ccc.blocks[0].m_x; //grab x value from pixy
    if (pixyInput >= 160)
    {
      deviation = pixyInput - 160; //calculate how far we are from centered
      targetValue = 160 - deviation; //invert value for PID logic
    }
    else if (pixyInput < 160)
    {
      deviation = 160 - pixyInput; //calculate how far we are from centered
      targetValue = 160 - deviation; //invert value for PID logic
    }
    Input = targetValue; //get value ready to send to PID
    myPID.Compute(); //calculate Output with PID
    dutyCycle = map(Output, 0.0, 255.0, 40.0, 100.0); //set duty cycle between 40 and 100% based on output which is scaled from 0 to 255
    Serial.println(Output);
    onFactor = (dutyCycle / 100.0);
    offFactor = (100.0 - dutyCycle) / 100.0;
    onTime = (1000.0 / frequency) * onFactor;
    offTime = (1000.0 / frequency) * offFactor;
  }
  else if (centered == 1) //if centered, full speed forward
  {
    dutyCycle = 100.0;
    Serial.println("centered");
    Serial.println(dutyCycle);
    onFactor = (dutyCycle / 100.0);
    offFactor = (100.0 - dutyCycle) / 100.0;
    onTime = (1000.0 / frequency) * onFactor;
    offTime = (1000.0 / frequency) * offFactor;
  }
  else if (droid == 4) //if target lost, set duty cycle to 60% for scan routine
  {
    dutyCycle = 60.0;
    onFactor = (dutyCycle / 100.0);
    offFactor = (100.0 - dutyCycle) / 100.0;
    onTime = (1000.0 / frequency) * onFactor;
    offTime = (1000.0 / frequency) * offFactor;
  }
  else //set idle speed
  {
    dutyCycle = 60.0;
    onFactor = (dutyCycle / 100.0);
    offFactor = (100.0 - dutyCycle) / 100.0;
    onTime = (1000.0 / frequency) * onFactor;
    offTime = (1000.0 / frequency) * offFactor;
  }


  switch (droid)
  {
    case 1://idle driving
      dutyCycle = 75.0;
      onFactor = (dutyCycle / 100.0);
      offFactor = (100.0 - dutyCycle) / 100.0;
      onTime = (1000.0 / frequency) * onFactor;
      offTime = (1000.0 / frequency) * offFactor;

      digitalWrite(trackLeftPos, HIGH); digitalWrite(trackLeftNeg, LOW); digitalWrite(trackRightPos, LOW); digitalWrite(trackRightNeg, HIGH);// go forward
      delay(onTime);
      digitalWrite(trackLeftPos, LOW); digitalWrite(trackLeftNeg, LOW); digitalWrite(trackRightPos, LOW); digitalWrite(trackRightNeg, LOW);
      delay(offTime);

      if (pixy.ccc.numBlocks) //if target aquired, stop, make a noise and enter targeting mode
      {
        digitalWrite(trackLeftPos, LOW); digitalWrite(trackLeftNeg, LOW); digitalWrite(trackRightPos, LOW); digitalWrite(trackRightNeg, LOW);
        droid = 3;
        digitalWrite(bleep, LOW);
        delay(250);
        digitalWrite(bleep, HIGH);
      }

      if (millis() - turnTimer >= 5000) //if no targets after 5 seconds, scan for targets
      {
        turnTimer = millis();
        droid = 4;
        scanTimer = millis();
      }

      if (sonar.ping_cm() <= 25 && sonar.ping_cm() != 0) //if we're about to run into something, stop, back up, turn around and continue
      {
        digitalWrite(trackLeftPos, LOW); digitalWrite(trackLeftNeg, LOW); digitalWrite(trackRightPos, LOW); digitalWrite(trackRightNeg, LOW);
        delay(1000);
        digitalWrite(trackLeftPos, LOW); digitalWrite(trackLeftNeg, HIGH); digitalWrite(trackRightPos, HIGH); digitalWrite(trackRightNeg, LOW);
        delay(750);
        digitalWrite(trackLeftPos, LOW); digitalWrite(trackLeftNeg, LOW); digitalWrite(trackRightPos, LOW); digitalWrite(trackRightNeg, LOW);
        delay(1000);
        direct = random(0, 2); //pick a random direction to turn
        if (direct == 0)
        {
          turnTimer = millis();
          digitalWrite(trackLeftPos, HIGH); digitalWrite(trackLeftNeg, LOW); digitalWrite(trackRightPos, HIGH); digitalWrite(trackRightNeg, LOW);
          delay(500);
          digitalWrite(trackLeftPos, HIGH); digitalWrite(trackLeftNeg, LOW); digitalWrite(trackRightPos, LOW); digitalWrite(trackRightNeg, HIGH);
        }
        else if (direct == 1)
        {
          turnTimer = millis();
          digitalWrite(trackLeftPos, LOW); digitalWrite(trackLeftNeg, HIGH); digitalWrite(trackRightPos, LOW); digitalWrite(trackRightNeg, HIGH);
          delay(500);
          digitalWrite(trackLeftPos, HIGH); digitalWrite(trackLeftNeg, LOW); digitalWrite(trackRightPos, LOW); digitalWrite(trackRightNeg, HIGH);
        }
      }
      break;

    case 3://targeting
      if (pixy.ccc.numBlocks)//if target available
      {
        if (pixy.ccc.blocks[0].m_x < 140) //if target is off center, turn accordingly
        {
          direct = 0;
          digitalWrite(trackRightPos, LOW); digitalWrite(trackRightNeg, HIGH); digitalWrite(trackLeftPos, LOW); digitalWrite(trackLeftNeg, HIGH);
          delay(onTime);
          digitalWrite(trackLeftPos, LOW); digitalWrite(trackLeftNeg, LOW); digitalWrite(trackRightPos, LOW); digitalWrite(trackRightNeg, LOW);
          delay(offTime);
          centered = 0;
        }
        else if (pixy.ccc.blocks[0].m_x > 180)//if target is off center the other direction, turn accordingly
        {
          direct = 1;
          digitalWrite(trackLeftPos, HIGH); digitalWrite(trackLeftNeg, LOW); digitalWrite(trackRightPos, HIGH); digitalWrite(trackRightNeg, LOW);
          delay(onTime);
          digitalWrite(trackLeftPos, LOW); digitalWrite(trackLeftNeg, LOW); digitalWrite(trackRightPos, LOW); digitalWrite(trackRightNeg, LOW);
          delay(offTime);
          centered = 0;
        }
        else if (pixy.ccc.blocks[0].m_x >= 140 && pixy.ccc.blocks[0].m_x <= 180 && sonar.ping_cm() < 45 && sonar.ping_cm() != 0) //if target centered and we are within range, wait and enter firing mode
        {
          digitalWrite(trackLeftPos, LOW); digitalWrite(trackLeftNeg, LOW); digitalWrite(trackRightPos, LOW); digitalWrite(trackRightNeg, LOW);
          centered = 1;
          //Serial.println("centered");
          delay(500);
          droid = 5;
        }
        else if (pixy.ccc.blocks[0].m_x >= 140 && pixy.ccc.blocks[0].m_x <= 180 && (sonar.ping_cm() >= 45 || sonar.ping_cm() == 0)) // if target centered and out of range, move closer
        {
          //Serial.println("centered");
          digitalWrite(trackLeftPos, HIGH); digitalWrite(trackLeftNeg, LOW); digitalWrite(trackRightPos, LOW); digitalWrite(trackRightNeg, HIGH);
          delay(onTime);
          digitalWrite(trackLeftPos, LOW); digitalWrite(trackLeftNeg, LOW); digitalWrite(trackRightPos, LOW); digitalWrite(trackRightNeg, LOW);
          delay(offTime);
          centered = 1;
        }
        if (sonar.ping_cm() <= 10 && sonar.ping_cm() != 0) //if we've gotten too close to the target, back up and reapproach
        {
          digitalWrite(trackLeftPos, LOW); digitalWrite(trackLeftNeg, LOW); digitalWrite(trackRightPos, LOW); digitalWrite(trackRightNeg, LOW);
          delay(500);
          digitalWrite(trackLeftPos, LOW); digitalWrite(trackLeftNeg, HIGH); digitalWrite(trackRightPos, HIGH); digitalWrite(trackRightNeg, LOW);
          delay(750);
          digitalWrite(trackLeftPos, LOW); digitalWrite(trackLeftNeg, LOW); digitalWrite(trackRightPos, LOW); digitalWrite(trackRightNeg, LOW);
        }
      }
      else//target lost, scan for target
      {
        digitalWrite(trackLeftPos, LOW); digitalWrite(trackLeftNeg, LOW); digitalWrite(trackRightPos, LOW); digitalWrite(trackRightNeg, LOW);
        droid = 4;
        dutyCycle = 60.0;
        onFactor = (dutyCycle / 100.0);
        offFactor = (100.0 - dutyCycle) / 100.0;
        onTime = (1000.0 / frequency) * onFactor;
        offTime = (1000.0 / frequency) * offFactor;
        scanTimer = millis();
      }
      break;

    case 4: //target lost: reaquire
      dutyCycle = 50.0;
      onFactor = (dutyCycle / 100.0);
      offFactor = (100.0 - dutyCycle) / 100.0;
      onTime = (1000.0 / frequency) * onFactor;
      offTime = (1000.0 / frequency) * offFactor;
      
      if (millis() - scanTimer >= 5000)//if we've scanned for 5 seconds and theres still no target, make a noise and enter idle roaming mode
      {
        droid = 1;
        digitalWrite(bloop, LOW);
        delay(250);
        digitalWrite(bloop, HIGH);
        turnTimer = millis();
        digitalWrite(trackLeftPos, HIGH); digitalWrite(trackLeftNeg, LOW); digitalWrite(trackRightPos, LOW); digitalWrite(trackRightNeg, HIGH);
      }
      else if (direct == 1)//if target was last seen on one side, turn accordingly to reaquire
      {
        digitalWrite(trackLeftPos, HIGH); digitalWrite(trackLeftNeg, LOW); digitalWrite(trackRightPos, HIGH); digitalWrite(trackRightNeg, LOW);
        delay(onTime);
        digitalWrite(trackLeftPos, LOW); digitalWrite(trackLeftNeg, LOW); digitalWrite(trackRightPos, LOW); digitalWrite(trackRightNeg, LOW);
        delay(offTime);
      }
      else if (direct == 0)//if target was last seen on the other side, turn accordingly to reaquire
      {
        digitalWrite(trackRightPos, LOW); digitalWrite(trackRightNeg, HIGH); digitalWrite(trackLeftPos, LOW); digitalWrite(trackLeftNeg, HIGH);
        delay(onTime);
        digitalWrite(trackLeftPos, LOW); digitalWrite(trackLeftNeg, LOW); digitalWrite(trackRightPos, LOW); digitalWrite(trackRightNeg, LOW);
        delay(offTime);
      }

      if (pixy.ccc.numBlocks)//if target reaquired, enter targeting mode
      {
        digitalWrite(trackLeftPos, LOW); digitalWrite(trackLeftNeg, LOW); digitalWrite(trackRightPos, LOW); digitalWrite(trackRightNeg, LOW);
        droid = 3;
        digitalWrite(bleep, LOW);
        delay(250);
        digitalWrite(bleep, HIGH);
      }
      break;

    case 5://target locked
      if (pixy.ccc.blocks[0].m_x >= 140 && pixy.ccc.blocks[0].m_x <= 180 && sonar.ping_cm() < 45 && sonar.ping_cm() != 0)//target is locked and within range, open fire
      {
        digitalWrite(ID, LOW);//ask for identification
        delay(500);
        digitalWrite(ID, HIGH);
        delay(2000);
        digitalWrite(gun, HIGH);//shoot anyway
        delay(1400);
        digitalWrite(gotcha, LOW);//brag about it
        delay(1000);
        digitalWrite(gun, LOW);
        digitalWrite(gotcha, HIGH);
        delay(1000);
        digitalWrite(trackLeftPos, LOW); digitalWrite(trackLeftNeg, HIGH); digitalWrite(trackRightPos, HIGH); digitalWrite(trackRightNeg, LOW);//back up
        delay(750);
        digitalWrite(trackLeftPos, LOW); digitalWrite(trackLeftNeg, LOW); digitalWrite(trackRightPos, LOW); digitalWrite(trackRightNeg, LOW);
        delay(1000);
        direct = random(0, 2);//pick which way to turn
        if (direct == 0) //turn around
        {
          turnTimer = millis();
          digitalWrite(trackLeftPos, HIGH); digitalWrite(trackLeftNeg, LOW); digitalWrite(trackRightPos, HIGH); digitalWrite(trackRightNeg, LOW);
          delay(500);
          digitalWrite(trackLeftPos, HIGH); digitalWrite(trackLeftNeg, LOW); digitalWrite(trackRightPos, LOW); digitalWrite(trackRightNeg, HIGH);
        }
        else if (direct == 1)//turn around
        {
          turnTimer = millis();
          digitalWrite(trackLeftPos, LOW); digitalWrite(trackLeftNeg, HIGH); digitalWrite(trackRightPos, LOW); digitalWrite(trackRightNeg, HIGH);
          delay(500);
          digitalWrite(trackLeftPos, HIGH); digitalWrite(trackLeftNeg, LOW); digitalWrite(trackRightPos, LOW); digitalWrite(trackRightNeg, HIGH);
        }
        droid = 1;//leave and enter idle roaming mode
        digitalWrite(bloop, LOW);//make a bloop noise
        delay(250);
        digitalWrite(bloop, HIGH);
      }
      else
      {
        droid = 3;//Target no longer locked, retarget 
      }
      break;
  }
}

