/*
    MrHeadTracker Switchable based on the GY-521/MPU6050 sensor

    Copyright (C) 2016-2017  Michael Romanov, Daniel Rudrich

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/


#include <MIDI.h>
#include <EEPROM.h>
#include <Wire.h>
#include <I2Cdev.h>
#include <MPU6050_6Axis_MotionApps20.h>
#include "utility/imumaths.h"

// MIDI
MIDI_CREATE_DEFAULT_INSTANCE();

// Pins and Settings
#define LED 13
#define button 3 // <-- should be an interrupt pin
#define invSwitch 4       //switch #2 on tracker
#define quatSwitch 5      //switch #1 on tracker
#define pi 3.1415926536
#define radTo14 2607.594587617613379
#define oneTo14 8191
#define qCalAddr 0 // EEPROM qCal address
#define debounceDelay 50


#define MODE_SERIAL
//#define MODE_MIDI


volatile unsigned long lastChangeTime, lastPressTime, lastReleaseTime = 0;
volatile bool buttonState = 1;
volatile bool newButtonState = 0;
int action = 0;
int calibrationState = 0;

uint16_t lastW = 63;
uint16_t newW = 63;
uint16_t lastX = 63;
uint16_t newX = 63;
uint16_t lastY = 63;
uint16_t newY = 63;
uint16_t lastZ = 63;
uint16_t newZ = 63;

// MPU control/status vars
MPU6050 mpu;
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// Quaternions and Vectors
Quaternion qCal, qCalLeft, qCalRight, qIdleConj = {1, 0, 0, 0};
Quaternion qGravIdle, qGravCal, quat, steering, qRaw;

VectorInt16 gRaw;         //
const VectorFloat refVector = {1, 0, 0};
VectorFloat vGravIdle, vGravCal;
float ypr[3]; //yaw pitch and roll angles

// Interrupt detection - sensor
volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
  mpuInterrupt = true;
}



// ================================================================
// ===                      INITIAL SETUP                       ===
// ================================================================

void setup() {
  // join I2C bus (I2Cdev library doesn't do this automatically)
  Wire.begin();


  
  #ifdef MODE_SERIAL
  Serial.begin(115200, SERIAL_8E1);
  #endif

  #ifdef MODE_MIDI
  MIDI.begin(MIDI_CHANNEL_OMNI); // use this with HIDUINO
  #endif

  pinMode(button, INPUT_PULLUP);
  pinMode(invSwitch, INPUT_PULLUP);
  pinMode(quatSwitch, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(button), buttonChange, CHANGE);

  pinMode(LED, OUTPUT);
  digitalWrite(LED, LOW);
  EEPROM.get(qCalAddr, qCal); // read qCal from EEPROM and print values
  resetOrientation();

  // initialize device
  mpu.initialize();
  // wait for ready
  while (Serial.available() && Serial.read()); // empty buffer again
  // load and configure the DMP
  devStatus = mpu.dmpInitialize();
  // make sure it worked (returns 0 if so)
  if (devStatus == 0) {
    // turn on the DMP, now that it's ready
    delay(5);
    // Serial.println(F("Enabling DMP..."));
    mpu.setDMPEnabled(true);

    // enable Arduino interrupt detection
    //Serial.println(F("Enabling interrupt detection (Arduino external interrupt 0)..."));
    attachInterrupt(digitalPinToInterrupt(2), dmpDataReady, RISING);
    mpuIntStatus = mpu.getIntStatus();

    // set our DMP Ready flag so the main loop() function knows it's okay to use it
    //Serial.println(F("DMP ready! Waiting for first interrupt..."));
    dmpReady = true;
    // get expected DMP packet size for later comparison
    packetSize = mpu.dmpGetFIFOPacketSize();
  }
}



// ================================================================
// ===                    MAIN PROGRAM LOOP                     ===
// ================================================================

void loop() {
  // ============== BUTTON CHECK ROUTINE ==========================
  action = 0; // do nothing, just chill... for now!

  if (newButtonState && (millis() - lastChangeTime > debounceDelay)) {
    switch (buttonState) {
      case 0: // pressed
        lastPressTime = lastChangeTime;
        if (millis() - lastPressTime > 1000) {
          action = 2; //held longer than ^ ms
          newButtonState = 0; //only once!
        }
        break;
      case 1: // released
        newButtonState = 0;
        lastReleaseTime = lastChangeTime;
        action = 1; // short button click
        if (lastReleaseTime - lastPressTime > 1000) {
          action = 3; //release after hold > ^ ms
        }
        break;
    }
  }


  // if programming failed, don't try to do anything
  if (!dmpReady) return;
  // wait for MPU interrupt or extra packet(s) available
  while (!mpuInterrupt && fifoCount < packetSize) {
  }

  // reset interrupt flag and get INT_STATUS byte
  mpuInterrupt = false;
  mpuIntStatus = mpu.getIntStatus();

  // get current FIFO count
  fifoCount = mpu.getFIFOCount();

  // check for overflow (this should never happen unless our code is too inefficient)
  if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
    digitalWrite(LED, HIGH);
    // reset so we can continue cleanly
    mpu.resetFIFO();
    //Serial.println(F("FIFO overflow!"));

    // otherwise, check for DMP data ready interrupt (this should happen frequently)
  } else if (mpuIntStatus & 0x02) {
    // wait for correct available data length, should be a VERY short wait
    while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();

    // read a packet from FIFO
    mpu.getFIFOBytes(fifoBuffer, packetSize);

    // track FIFO count here in case there is > 1 packet available
    // (this lets us immediately read more without waiting for an interrupt)
    fifoCount -= packetSize;


  // ============== QUATERNION DATA ROUTINE ======================
    
    // get quaternion values in easy matrix form: w x y z
    mpu.dmpGetQuaternion(&qRaw, fifoBuffer);
    mpu.dmpGetAccel(&gRaw, fifoBuffer);

    steering = qIdleConj.getProduct(qRaw);
    quat = qCalLeft.getProduct(steering);
    quat = quat.getProduct(qCalRight);

    if (digitalRead(invSwitch) == LOW) {
      quat = quat.getConjugate();
    }




 // ============== SEND MIDI ROUTINE ===========================
    if (digitalRead(quatSwitch) == LOW) { //send quaternion data
    
    #ifdef MODE_MIDI  
      newW = (uint16_t)(oneTo14 * (quat.w + 1));
      newX = (uint16_t)(oneTo14 * (quat.x + 1));
      newY = (uint16_t)(oneTo14 * (quat.y + 1));
      newZ = (uint16_t)(oneTo14 * (quat.z + 1));
    #endif

    #ifdef MODE_SERIAL
      newW = (uint16_t)(1000 * (quat.w + 1));
      newX = (uint16_t)(1000 * (quat.x + 1));
      newY = (uint16_t)(1000 * (quat.y + 1));
      newZ = (uint16_t)(1000 * (quat.z + 1));
    #endif
      
    #ifdef MODE_SERIAL
    if (newW != lastW || newX != lastX || newY != lastY || newZ != lastZ ) {
      
      sendQuaternion(newW, newX, newY, newZ);
      
    }
    #endif
    
    #ifdef MODE_MIDI
    if (newW != lastW) {  
      MIDI.sendControlChange(48, newW & 0x7F,  1);
      MIDI.sendControlChange(16, (newW >> 7) & 0x7F, 1);
    }
    if (newX != lastX) {  
      MIDI.sendControlChange(49, newX & 0x7F,  1);
      MIDI.sendControlChange(17, (newX >> 7) & 0x7F, 1);
    }
    if (newY != lastY) {
      MIDI.sendControlChange(50, newY & 0x7F, 1);
      MIDI.sendControlChange(18, (newY >> 7) & 0x7F, 1);  
    }
    if (newZ != lastZ) {
      MIDI.sendControlChange(51, newZ & 0x7F, 1);
      MIDI.sendControlChange(19, (newZ >> 7) & 0x7F, 1);
    }
    #endif

      lastW = newW;
      lastX = newX;
      lastY = newY;
      lastZ = newZ;
    }
    else //send yaw pitch roll data
    {

      quat2ypr(ypr, &quat);
      newZ = (uint16_t)(radTo14 * ((ypr[0] + pi)));
      newY = (uint16_t)(radTo14 * ((ypr[1] + pi)));
      newX = (uint16_t)(radTo14 * ((ypr[2] + pi)));
    
    #ifdef MODE_SERIAL
    if (newZ != lastZ || newY != lastY || newX != lastX ) {
      
      sendYPR(newZ, newY, newX);
      
    }
    #endif
    
    #ifdef MODE_MIDI
    if (newZ != lastZ) {  
      MIDI.sendControlChange(48, newZ & 0x7F,  1);
      MIDI.sendControlChange(16, (newZ >> 7) & 0x7F, 1);
    }
    if (newY != lastY) {
      MIDI.sendControlChange(49, newY & 0x7F, 1);
      MIDI.sendControlChange(17, (newY >> 7) & 0x7F, 1);  
    }
    if (newX != lastX) {
      MIDI.sendControlChange(50, newX & 0x7F, 1);
      MIDI.sendControlChange(18, (newX >> 7) & 0x7F, 1);
    }
    #endif

      
      lastX = newX;
      lastY = newY;
      lastZ = newZ;
    }


    mpu.resetFIFO();
  }


  // ============== BUTTON ACTION ROUTINE ======================
  
  //now do some action!
  switch (action) {
    case 1: // short button click
      if (calibrationState == 1) {
        vGravCal = VectorFloat(gRaw.x, gRaw.y, gRaw.z);
        calibrate();
        calibrationState = 0;
      }
      else {
        qIdleConj = qRaw.getConjugate();
      }
      break;
    case 2: // long hold
      if (!calibrationState) {
        calibrationState = 1;
        qIdleConj = qRaw.getConjugate();
        vGravIdle = VectorFloat(gRaw.x, gRaw.y, gRaw.z);
      }
  }

}

void calibrate() {
  VectorFloat g, gCal, x, y, z;
  //g = refVector.getRotated(&qGravIdle); //g = qGravIdle.rotateVector(refVector);
  g = vGravIdle;
  z = g; // z = opposite(g); not necessary
  z.normalize();

  //gCal = refVector.getRotated(&qGravCal); //gCal = qGravCal.rotateVector(refVector);
  gCal = vGravCal;
  y = cross(gCal, g);
  y.normalize();

  x = cross(y, z);
  x.normalize();

  imu::Matrix<3> rot;
  rot.cell(0, 0) = x.x;
  rot.cell(1, 0) = x.y;
  rot.cell(2, 0) = x.z;
  rot.cell(0, 1) = y.x;
  rot.cell(1, 1) = y.y;
  rot.cell(2, 1) = y.z;
  rot.cell(0, 2) = z.x;
  rot.cell(1, 2) = z.y;
  rot.cell(2, 2) = z.z;

  qCal = RotMat2Quat(rot);
  EEPROM.put(qCalAddr, qCal);

  resetOrientation();
}

void buttonChange() {
  lastChangeTime = millis();
  buttonState = digitalRead(button);
  newButtonState = 1;
}

void resetOrientation() {
  qCalLeft = qCal.getConjugate();
  qCalRight = qCal;
}

VectorFloat opposite(VectorFloat v) {
  v.x *= -1;
  v.y *= -1;
  v.z *= -1;
  return v;
}

VectorFloat cross(VectorFloat p, VectorFloat v)
{
  VectorFloat ret;
  ret.x = p.y * v.z - p.z * v.y;
  ret.y = p.z * v.x - p.x * v.z;
  ret.z = p.x * v.y - p.y * v.x;
  return ret;
}


Quaternion RotMat2Quat(imu::Matrix<3>& m)
{
  Quaternion ret;
  double tr = m.trace();

  double S;
  if (tr > 0)
  {
    S = sqrt(tr + 1.0) * 2;
    ret.w = 0.25 * S;
    ret.x = (m(2, 1) - m(1, 2)) / S;
    ret.y = (m(0, 2) - m(2, 0)) / S;
    ret.z = (m(1, 0) - m(0, 1)) / S;
  }
  else if (m(0, 0) > m(1, 1) && m(0, 0) > m(2, 2))
  {
    S = sqrt(1.0 + m(0, 0) - m(1, 1) - m(2, 2)) * 2;
    ret.w = (m(2, 1) - m(1, 2)) / S;
    ret.x = 0.25 * S;
    ret.y = (m(0, 1) + m(1, 0)) / S;
    ret.z = (m(0, 2) + m(2, 0)) / S;
  }
  else if (m(1, 1) > m(2, 2))
  {
    S = sqrt(1.0 + m(1, 1) - m(0, 0) - m(2, 2)) * 2;
    ret.w = (m(0, 2) - m(2, 0)) / S;
    ret.x = (m(0, 1) + m(1, 0)) / S;
    ret.y = 0.25 * S;
    ret.z = (m(1, 2) + m(2, 1)) / S;
  }
  else
  {
    S = sqrt(1.0 + m(2, 2) - m(0, 0) - m(1, 1)) * 2;
    ret.w = (m(1, 0) - m(0, 1)) / S;
    ret.x = (m(0, 2) + m(2, 0)) / S;
    ret.y = (m(1, 2) + m(2, 1)) / S;
    ret.z = 0.25 * S;
  }
  return ret;
}

void quat2ypr(float *ypr, Quaternion *q) {
  //CONVERSION FROM QUATERNION DATA TO TAIT-BRYAN ANGLES yaw, pitch and roll
  //IMPORTANT: rotation order: yaw, pitch, roll (intrinsic rotation: z-y'-x'') !!
  //MNEMONIC: swivel on your swivel chair, look up/down, then tilt your head left/right...
  //           ... thats how we yaw, pitch'n'roll.
  double ysqr = q->y * q->y;

  // yaw (z-axis rotation)
  double t0 = 2.0 * (q->w * q->z + q->x * q->y);
  double t1 = 1.0 - 2.0 * (ysqr + q->z * q->z);
  ypr[0] = atan2(t0, t1);

  // pitch (y-axis rotation)
  t0 = 2.0 * (q->w * q->y - q->z * q->x);
  t0 = t0 > 1.0 ? 1.0 : t0;
  t0 = t0 < -1.0 ? -1.0 : t0;
  ypr[1] = asin(t0);

  // roll (x-axis rotation)
  t0 = 2.0 * (q->w * q->x + q->y * q->z);
  t1 = 1.0 - 2.0 * (q->x * q->x + ysqr);
  ypr[2] = atan2(t0, t1);
}

void sendQuaternion(uint16_t qW, uint16_t qX, uint16_t qY, uint16_t qZ){


  Serial.print(qW);
  Serial.print(' ');
  Serial.print(qX);
  Serial.print(' ');
  Serial.print(qY);
  Serial.print(' ');
  Serial.print(qZ);
  Serial.print('\n');

}

void sendYPR(uint16_t yaw, uint16_t pitch, uint16_t roll){

  Serial.print(yaw);
  Serial.print(' ');
  Serial.print(pitch);
  Serial.print(' ');
  Serial.print(roll);
  Serial.print('\n');

}
