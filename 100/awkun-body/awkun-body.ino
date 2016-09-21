/*
  The MIT License (MIT)

  Copyright (c) 2013 - 2014 Timothy D Meadows II

  Permission is hereby granted, free of charge, to any person obtaining a copy
  of this software and associated documentation files (the "Software"), to deal
  in the Software without restriction, including without limitation the rights
  to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
  copies of the Software, and to permit persons to whom the Software is
  furnished to do so, subject to the following conditions:

  The above copyright notice and this permission notice shall be included in
  all copies or substantial portions of the Software.

  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
  IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
  FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
  AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
  LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
  OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
  THE SOFTWARE.
  
  Codename: Awkun
  Version: 1.00
*/

#include <Wire.h>
#include <Ultrasonic.h>
#include <HMC5883L.h>
#include <Servo.h>

const short distanceOfBlockage = 8;

const int echoPin = 2;
const int trigPin = 3;
Ultrasonic ultrasonic(trigPin, echoPin);

HMC5883L compass;
enum COMPASS_DIRECTION
{
  NORTH = 360,
  NORTHEAST = 45,
  EAST = 90,
  SOUTHEAST = 135,
  SOUTH = 180,
  SOUTHWEST = 225,
  WEST = 270,
  NORTHWEST = 315
};

const int accelerometerXPin = 4;
const int accelerometerYPin = 6;
const int accelerometerZPin = 10;
const int accelerometerStableMin = 265;
const int accelerometerStableMax = 402;
struct AccelerometerData
{
  int x;
  int y;
  int z;
};

const int mercuryTiltSwitchPin = 11;

const int leftFrontMotorPin = 8;
const int leftBackMotorPin = 9;
const int rightFrontMotorPin = 6;
const int rightBackMotorPin = 7;
enum MOVEMENT_STATE
{
  MOVEMENT_STOP = 0,
  MOVEMENT_START = 1,
  MOVEMENT_FORWARD = 2,
  MOVEMENT_BACKWARD = 3,
  MOVEMENT_LEFT = 4,
  MOVEMENT_RIGHT = 5
};

const int headServoPin = 13;
enum HEAD_STATE
{
  HEAD_UNKNOWN = 0,
  HEAD_CENTER = 1,
  HEAD_LEFT = 2,
  HEAD_RIGHT = 3
};

Servo headServo;
HEAD_STATE headState;
MOVEMENT_STATE moveState;

void setup()
{
  Serial.begin(9600); // baud
  Wire.begin();
  
  for (int i = 3; i < 11; i++) {
    pinMode(i, OUTPUT); // set pins 3 to 11 as outputs(sensors + motors)
  }
  
  int error = compass.SetScale(1.3);
  if(error != 0) {
    Serial.println(compass.GetErrorText(error));
  }
  
  error = compass.SetMeasurementMode(Measurement_Continuous);
  if(error != 0) {
    Serial.println(compass.GetErrorText(error));
  }
  
  headServo.attach(headServoPin);
  moveState = MOVEMENT_START;
}

void loop()
{
  think();
  // TODO: check for RX serial data to override thinking process
}

void think()
{
  int movement_direction = scan_movement_direction();
  Serial.println(movement_direction);
  
  switch(movement_direction) {
    case MOVEMENT_STOP:
      movement_stop(0);
      break;
    case MOVEMENT_FORWARD:
      movement_forward(200);
      break;
    case MOVEMENT_BACKWARD:
      movement_backward(800);
      break;
    case MOVEMENT_LEFT:
      movement_left(800);
      break;
    case MOVEMENT_RIGHT:
      movement_right(800);
      break;
  }
}

long scan_nearest_object()
{
  return ultrasonic.Ranging(INC); // inches
}

float scan_heading()
{
  MagnetometerScaled reading = sensor_compass();
  float heading = atan2(-reading.YAxis, reading.XAxis) + 0.0842; // declination for crestline, ca
  
  if(heading < 0) {
    heading = heading + 360;
  }
  
  return heading;
}

short scan_degrees()
{
  return scan_heading() / M_PI * 180;
}

int scan_direction()
{
  short degrees = scan_degrees();
  int direction = NORTH;
  int cardinals[7] = { 45, 90, 135, 180, 225, 270, 315 };
  
  for (int i = 1; i <= 7; i++) {
    direction = abs(cardinals[i] - degrees) < abs(direction - degrees) ? cardinals[i] : direction;
  }
  
  return direction;
}

int scan_movement_direction()
{
  if (look_center() <= distanceOfBlockage) {
    if (rng(1, 2) == 1) {
      if (look_left() <= distanceOfBlockage) {
        if (look_right() <= distanceOfBlockage) {
          return MOVEMENT_BACKWARD;
        }
        
        return MOVEMENT_RIGHT;
      }
      
      return MOVEMENT_LEFT;
    }
    
    if (look_right() <= distanceOfBlockage) {
      if (look_left() <= distanceOfBlockage) {
        return MOVEMENT_BACKWARD;
      }
      
      return MOVEMENT_LEFT;
    }
    
    return MOVEMENT_RIGHT;
  }
  
  return MOVEMENT_FORWARD;
}

void movement_stop(int duration)
{
  if (moveState == MOVEMENT_STOP) {
    return;
  }
  
  moveState = MOVEMENT_STOP;
  digitalWrite(leftFrontMotorPin, HIGH);
  digitalWrite(leftBackMotorPin, HIGH);
  digitalWrite(rightFrontMotorPin, HIGH);
  digitalWrite(rightBackMotorPin, HIGH);
  
  if (duration != 0) {
    delay(duration);
  }
}

void movement_forward(int duration)
{
  if (moveState == MOVEMENT_FORWARD) {
    return;
  }
  
  moveState = MOVEMENT_FORWARD;
  digitalWrite(leftFrontMotorPin, HIGH);
  digitalWrite(leftBackMotorPin, LOW);
  digitalWrite(rightFrontMotorPin, HIGH);
  digitalWrite(rightBackMotorPin, LOW);
  delay(duration);
}

void movement_backward(int duration)
{  
  moveState = MOVEMENT_BACKWARD;
  digitalWrite(leftFrontMotorPin, LOW);
  digitalWrite(leftBackMotorPin, HIGH);
  digitalWrite(rightFrontMotorPin, LOW);
  digitalWrite(rightBackMotorPin, HIGH);
  delay(duration);
}

void movement_left(int duration)
{
  movement_backward(150);
  
  moveState = MOVEMENT_LEFT;
  digitalWrite(leftFrontMotorPin, LOW);
  digitalWrite(leftBackMotorPin, HIGH);
  digitalWrite(rightFrontMotorPin, HIGH);
  digitalWrite(rightBackMotorPin, LOW);
  delay(duration);
}

void movement_right(int duration)
{
  movement_backward(150);
  
  moveState = MOVEMENT_RIGHT;
  digitalWrite(leftFrontMotorPin, HIGH);
  digitalWrite(leftBackMotorPin, LOW);
  digitalWrite(rightFrontMotorPin, LOW);
  digitalWrite(rightBackMotorPin, HIGH);
  delay(duration);
}

void head_center()
{
  if (headState == HEAD_CENTER) {
    return;
  }
  
  headState = HEAD_CENTER;
  headServo.write(115);
  delay(200);
}

void head_left()
{
  if (headState == HEAD_LEFT) {
    return;
  }
  
  headState = HEAD_LEFT;
  headServo.write(180);
  delay(200);
}

void head_right()
{
  if (headState == HEAD_RIGHT) {
    return;
  }
  
  headState = HEAD_RIGHT;
  headServo.write(50);
  delay(200);
}

long look_center()
{
  head_center();
  return scan_nearest_object();
}

long look_left()
{
  head_left();
  return scan_nearest_object();
}

long look_right()
{
  head_right();
  return scan_nearest_object();
}

MagnetometerScaled sensor_compass()
{
  return compass.ReadScaledAxis();
}

struct AccelerometerData sensor_accelerometer()
{
  AccelerometerData data;
  
  data.x = analogRead(accelerometerXPin);
  data.y = analogRead(accelerometerYPin);
  data.z = analogRead(accelerometerZPin);
  
  return data;
}

boolean sensor_tilt()
{
  return digitalRead(mercuryTiltSwitchPin) != HIGH;
}

char* to_compass_direction(int direction)
{
  switch (direction) {
    case NORTH:
      return "NORTH";
    case NORTHEAST:
      return "NORTH EAST";
    case EAST:
      return "EAST";
    case SOUTHEAST:
      return "SOUTH EAST";
    case SOUTH:
      return "SOUTH";
    case SOUTHWEST:
      return "SOUTH WEST";
    case WEST:
      return "WEST";
    case NORTHWEST:
      return "NORTH WEST";
  }
}

long rng(long minimum, long maximum)
{
  randomSeed(analogRead(0)); // seed on voltage level of pin 0, need more entropy!
  return random(minimum - 1, maximum) + 1;
}
