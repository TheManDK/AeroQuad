/*
  AeroQuad v1.8 - June 2010
  www.AeroQuad.com
  Copyright (c) 2010 Ted Carancho.  All rights reserved.
  An Open Source Arduino based multicopter.
 
  This program is free software: you can redistribute it and/or modify 
  it under the terms of the GNU General Public License as published by 
  the Free Software Foundation, either version 3 of the License, or 
  (at your option) any later version. 

  This program is distributed in the hope that it will be useful, 
  but WITHOUT ANY WARRANTY; without even the implied warranty of 
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the 
  GNU General Public License for more details. 

  You should have received a copy of the GNU General Public License 
  along with this program. If not, see <http://www.gnu.org/licenses/>. 
*/

//***************************************************************************************************
//********************************** Serial Commands ************************************************
//***************************************************************************************************
void readSerialCommand() {
  // Check for serial message
  if (Serial.available()) {
    digitalWrite(LEDPIN, LOW);
    queryType = Serial.read();
    switch (queryType) {
    case 'A': // Receive roll and pitch gyro PID
      PID[ROLL].P = readFloatSerial();
      PID[ROLL].I = readFloatSerial();
      PID[ROLL].D = readFloatSerial();
      PID[ROLL].lastPosition = 0;
      PID[ROLL].integratedError = 0;
      PID[PITCH].P = readFloatSerial();
      PID[PITCH].I = readFloatSerial();
      PID[PITCH].D = readFloatSerial();
      PID[PITCH].lastPosition = 0;
      PID[PITCH].integratedError = 0;
      minAcro = readFloatSerial();
      break;
    case 'C': // Receive yaw PID
      PID[YAW].P = readFloatSerial();
      PID[YAW].I = readFloatSerial();
      PID[YAW].D = readFloatSerial();
      PID[YAW].lastPosition = 0;
      PID[YAW].integratedError = 0;
      PID[HEADING].P = readFloatSerial();
      PID[HEADING].I = readFloatSerial();
      PID[HEADING].D = readFloatSerial();
      PID[HEADING].lastPosition = 0;
      PID[HEADING].integratedError = 0;
      headingHoldConfig = readFloatSerial();
      heading = 0;
      currentHeading = 0;
      headingHold = 0;
      break;
    case 'E': // Receive roll and pitch auto level PID
      PID[LEVELROLL].P = readFloatSerial();
      PID[LEVELROLL].I = readFloatSerial();
      PID[LEVELROLL].D = readFloatSerial();
      PID[LEVELROLL].lastPosition = 0;
      PID[LEVELROLL].integratedError = 0;
      PID[LEVELPITCH].P = readFloatSerial();
      PID[LEVELPITCH].I = readFloatSerial();
      PID[LEVELPITCH].D = readFloatSerial();
      PID[LEVELPITCH].lastPosition = 0;
      PID[LEVELPITCH].integratedError = 0;
      PID[LEVELGYROROLL].P = readFloatSerial();
      PID[LEVELGYROROLL].I = readFloatSerial();
      PID[LEVELGYROROLL].D = readFloatSerial();
      PID[LEVELGYROROLL].lastPosition = 0;
      PID[LEVELGYROROLL].integratedError = 0;
      PID[LEVELGYROPITCH].P = readFloatSerial();
      PID[LEVELGYROPITCH].I = readFloatSerial();
      PID[LEVELGYROPITCH].D = readFloatSerial();
      PID[LEVELGYROPITCH].lastPosition = 0;
      PID[LEVELGYROPITCH].integratedError = 0;
      break;
    case 'G': // Receive auto level configuration
      levelLimit = readFloatSerial();
      levelOff = readFloatSerial();
      break;
    case 'I': // Receive flight control configuration
      windupGuard = readFloatSerial();
      xmitFactor = readFloatSerial();
      break;
    case 'K': // Receive data filtering values
      gyro.setSmoothFactor(readFloatSerial());
      accel.setSmoothFactor(readFloatSerial());
      timeConstant = readFloatSerial();
      flightMode = readFloatSerial();
      break;
    case 'M': // Receive transmitter smoothing values
      smoothTransmitter[ROLL] = readFloatSerial();
      smoothTransmitter[PITCH] = readFloatSerial();
      smoothTransmitter[YAW] = readFloatSerial();
      smoothTransmitter[THROTTLE] = readFloatSerial();
      smoothTransmitter[MODE] = readFloatSerial();
      smoothTransmitter[AUX] = readFloatSerial();
      break;
    case 'O': // Receive transmitter calibration values
      mTransmitter[ROLL] = readFloatSerial();
      bTransmitter[ROLL] = readFloatSerial();
      mTransmitter[PITCH] = readFloatSerial();
      bTransmitter[PITCH] = readFloatSerial();
      mTransmitter[YAW] = readFloatSerial();
      bTransmitter[YAW] = readFloatSerial();
      mTransmitter[THROTTLE] = readFloatSerial();
      bTransmitter[THROTTLE] = readFloatSerial();
      mTransmitter[MODE] = readFloatSerial();
      bTransmitter[MODE] = readFloatSerial();
      mTransmitter[AUX] = readFloatSerial();
      bTransmitter[AUX] = readFloatSerial();
      break;
    case 'W': // Write all user configurable values to EEPROM
      writeFloat(PID[ROLL].P, PGAIN_ADR);
      writeFloat(PID[ROLL].I, IGAIN_ADR);
      writeFloat(PID[ROLL].D, DGAIN_ADR);
      writeFloat(PID[PITCH].P, PITCH_PGAIN_ADR);
      writeFloat(PID[PITCH].I, PITCH_IGAIN_ADR);
      writeFloat(PID[PITCH].D, PITCH_DGAIN_ADR);
      writeFloat(PID[LEVELROLL].P, LEVEL_PGAIN_ADR);
      writeFloat(PID[LEVELROLL].I, LEVEL_IGAIN_ADR);
      writeFloat(PID[LEVELROLL].D, LEVEL_DGAIN_ADR);
      writeFloat(PID[LEVELPITCH].P, LEVEL_PITCH_PGAIN_ADR);
      writeFloat(PID[LEVELPITCH].I, LEVEL_PITCH_IGAIN_ADR);
      writeFloat(PID[LEVELPITCH].D, LEVEL_PITCH_DGAIN_ADR);
      writeFloat(PID[YAW].P, YAW_PGAIN_ADR);
      writeFloat(PID[YAW].I, YAW_IGAIN_ADR);
      writeFloat(PID[YAW].D, YAW_DGAIN_ADR);
      writeFloat(PID[HEADING].P, HEADING_PGAIN_ADR);
      writeFloat(PID[HEADING].I, HEADING_IGAIN_ADR);
      writeFloat(PID[HEADING].D, HEADING_DGAIN_ADR);
      writeFloat(PID[LEVELGYROROLL].P, LEVEL_GYRO_ROLL_PGAIN_ADR);
      writeFloat(PID[LEVELGYROROLL].I, LEVEL_GYRO_ROLL_IGAIN_ADR);
      writeFloat(PID[LEVELGYROROLL].D, LEVEL_GYRO_ROLL_DGAIN_ADR);
      writeFloat(PID[LEVELGYROPITCH].P, LEVEL_GYRO_PITCH_PGAIN_ADR);
      writeFloat(PID[LEVELGYROPITCH].I, LEVEL_GYRO_PITCH_IGAIN_ADR);
      writeFloat(PID[LEVELGYROPITCH].D, LEVEL_GYRO_PITCH_DGAIN_ADR);
      writeFloat(windupGuard, WINDUPGUARD_ADR);  
      writeFloat(levelLimit, LEVELLIMIT_ADR);   
      writeFloat(levelOff, LEVELOFF_ADR); 
      writeFloat(xmitFactor, XMITFACTOR_ADR);
      writeFloat(gyro.getSmoothFactor(), GYROSMOOTH_ADR);
      writeFloat(accel.getSmoothFactor(), ACCSMOOTH_ADR);
      writeFloat(smoothTransmitter[THROTTLE], THROTTLESMOOTH_ADR);
      writeFloat(smoothTransmitter[ROLL], ROLLSMOOTH_ADR);
      writeFloat(smoothTransmitter[PITCH], PITCHSMOOTH_ADR);
      writeFloat(smoothTransmitter[YAW], YAWSMOOTH_ADR);
      writeFloat(smoothTransmitter[MODE], MODESMOOTH_ADR);
      writeFloat(smoothTransmitter[AUX], AUXSMOOTH_ADR);
      writeFloat(timeConstant, FILTERTERM_ADR);
      writeFloat(mTransmitter[THROTTLE], THROTTLESCALE_ADR);
      writeFloat(bTransmitter[THROTTLE], THROTTLEOFFSET_ADR);
      writeFloat(mTransmitter[ROLL], ROLLSCALE_ADR);
      writeFloat(bTransmitter[ROLL], ROLLOFFSET_ADR);
      writeFloat(mTransmitter[PITCH], PITCHSCALE_ADR);
      writeFloat(bTransmitter[PITCH], PITCHOFFSET_ADR);
      writeFloat(mTransmitter[YAW], YAWSCALE_ADR);
      writeFloat(bTransmitter[YAW], YAWOFFSET_ADR);
      writeFloat(mTransmitter[MODE], MODESCALE_ADR);
      writeFloat(bTransmitter[MODE], MODEOFFSET_ADR);
      writeFloat(mTransmitter[AUX], AUXSCALE_ADR);
      writeFloat(bTransmitter[AUX], AUXOFFSET_ADR);
      writeFloat(smoothHeading, HEADINGSMOOTH_ADR);
      writeFloat(aref, AREF_ADR);
      writeFloat(flightMode, FLIGHTMODE_ADR);
      writeFloat(headingHoldConfig, HEADINGHOLD_ADR);
      writeFloat(minAcro, MINACRO_ADR);
      zeroIntegralError();
      // Complementary filter setup
      for (axis = ROLL; axis < YAW; axis++)
        angle[axis].initialize(axis); // defined in FlightAngle.h
      break;
    case 'Y': // Initialize EEPROM with default values
      PID[ROLL].P = 5;
      PID[ROLL].I = 0;
      PID[ROLL].D = -10;
      PID[PITCH].P = 5;
      PID[PITCH].I = 0;
      PID[PITCH].D = -10;
      PID[YAW].P = 12.0;
      PID[YAW].I = 0;
      PID[YAW].D = 0;
      PID[LEVELROLL].P = 12;
      PID[LEVELROLL].I = 0;
      PID[LEVELROLL].D = -1;
      PID[LEVELPITCH].P = 12;
      PID[LEVELPITCH].I = 0;
      PID[LEVELPITCH].D = -1;
      PID[HEADING].P = 3;
      PID[HEADING].I = 0;
      PID[HEADING].D = 0;
      PID[LEVELGYROROLL].P = 7;
      PID[LEVELGYROROLL].I = 0;
      PID[LEVELGYROROLL].D = -25;
      PID[LEVELGYROPITCH].P = 7;
      PID[LEVELGYROPITCH].I = 0;
      PID[LEVELGYROPITCH].D = -25;
      windupGuard = 2000.0;
      xmitFactor = 0.20;  
      levelLimit = 1.0;
      levelOff = 0.0;
      gyro.setSmoothFactor(0.50);
      accel.setSmoothFactor(0.50);
      timeConstant = 4.0;   
      for (channel = ROLL; channel < LASTCHANNEL; channel++) {
        mTransmitter[channel] = 1.0;
        bTransmitter[channel] = 0.0;
      }
      smoothTransmitter[THROTTLE] = 1.0;
      smoothTransmitter[ROLL] = 1.0;
      smoothTransmitter[PITCH] = 1.0;
      smoothTransmitter[YAW] = 0.5;
      smoothTransmitter[MODE] = 1.0;
      smoothTransmitter[AUX] = 1.0;
      smoothHeading = 1.0;
      flightMode = ACRO;
      headingHoldConfig = OFF;
      minAcro = 1300;

      gyro.calibrate();
      accel.calibrate();
      aref = 3.0; // Use 2.8 if you are using an AeroQuad Shield < v1.7
      zeroIntegralError();
      break;
    case '1': // Calibrate ESCS's by setting Throttle high on all channels
      armed = 0;
      calibrateESC = 1;
      break;
    case '2': // Calibrate ESC's by setting Throttle low on all channels
      armed = 0;
      calibrateESC = 2;
      break;
    case '3': // Test ESC calibration
      armed = 0;
      testCommand = readFloatSerial();
      calibrateESC = 3;
      break;
    case '4': // Turn off ESC calibration
      armed = 0;
      calibrateESC = 0;
      testCommand = 1000;
      break;
    case '5': // Send individual motor commands (motor, command)
      armed = 0;
      calibrateESC = 5;
      for (motor = FRONT; motor < LASTMOTOR; motor++)
        remoteCommand[motor] = readFloatSerial();
      break;
    case 'a': // Enable/disable fast data transfer of sensor data
      queryType = 'X'; // Stop any other telemetry streaming
      if (readFloatSerial() == 1)
        fastTransfer = ON;
      else
        fastTransfer = OFF;
      break;
    case 'b': // calibrate gyros
      gyro.calibrate();
      break;
    case 'c': // calibrate accels
      accel.calibrate();
      break;
    case 'd': // send aref
      aref = readFloatSerial();
      break;
    }
  digitalWrite(LEDPIN, HIGH);
  }
}

//***************************************************************************************************
//********************************* Serial Telemetry ************************************************
//***************************************************************************************************
void sendSerialTelemetry() {
  update = 0;
  switch (queryType) {
  case '=': // Reserved debug command to view any variable from Serial Monitor
    Serial.print(accel.getRaw(ROLL));
    comma();
    Serial.print(accel.getZero(ROLL));
    comma();
    Serial.print(accel.getData(ROLL));
    comma();
    Serial.print(accel.getSmoothFactor());
    comma();
    Serial.println(accel.angleDeg(ROLL));
    queryType = 'X';
    break;
  case 'B': // Send roll and pitch gyro PID values
    Serial.print(PID[ROLL].P);
    comma();
    Serial.print(PID[ROLL].I);
    comma();
    Serial.print(PID[ROLL].D);
    comma();
    Serial.print(PID[PITCH].P);
    comma();
    Serial.print(PID[PITCH].I);
    comma();
    Serial.print(PID[PITCH].D);
    comma();
    Serial.println(minAcro);
    queryType = 'X';
    break;
  case 'D': // Send yaw PID values
    Serial.print(PID[YAW].P);
    comma();
    Serial.print(PID[YAW].I);
    comma();
    Serial.print(PID[YAW].D);
    comma();
    Serial.print(PID[HEADING].P);
    comma();
    Serial.print(PID[HEADING].I);
    comma();
    Serial.print(PID[HEADING].D);
    comma();
    Serial.println(headingHoldConfig, BIN);
    queryType = 'X';
    break;
  case 'F': // Send roll and pitch auto level PID values
    Serial.print(PID[LEVELROLL].P);
    comma();
    Serial.print(PID[LEVELROLL].I);
    comma();
    Serial.print(PID[LEVELROLL].D);
    comma();
    Serial.print(PID[LEVELPITCH].P);
    comma();
    Serial.print(PID[LEVELPITCH].I);
    comma();
    Serial.print(PID[LEVELPITCH].D);
    comma();
    Serial.print(PID[LEVELGYROROLL].P);
    comma();
    Serial.print(PID[LEVELGYROROLL].I);
    comma();
    Serial.print(PID[LEVELGYROROLL].D);
    comma();
    Serial.print(PID[LEVELGYROPITCH].P);
    comma();
    Serial.print(PID[LEVELGYROPITCH].I);
    comma();
    Serial.println(PID[LEVELGYROPITCH].D);
    queryType = 'X';
    break;
  case 'H': // Send auto level configuration values
    Serial.print(levelLimit);
    comma();
    Serial.println(levelOff);
    queryType = 'X';
    break;
  case 'J': // Send flight control configuration values
    Serial.print(windupGuard);
    comma();
    Serial.println(xmitFactor);
    queryType = 'X';
    break;
  case 'L': // Send data filtering values
    Serial.print(gyro.getSmoothFactor());
    comma();
    Serial.print(accel.getSmoothFactor());
    comma();
    Serial.print(timeConstant);
    comma();
    Serial.println(flightMode, DEC);
   queryType = 'X';
    break;
  case 'N': // Send motor smoothing values
    for (axis = ROLL; axis < AUX; axis++) {
      Serial.print(smoothTransmitter[axis]);
      comma();
    }
    Serial.println(smoothTransmitter[AUX]);
    queryType = 'X';
    break;
  case 'P': // Send transmitter calibration data
    for (axis = ROLL; axis < AUX; axis++) {
      Serial.print(mTransmitter[axis]);
      comma();
      Serial.print(bTransmitter[axis]);
      comma();
    }
    Serial.print(mTransmitter[AUX]);
    comma();
    Serial.println(bTransmitter[AUX]);
    queryType = 'X';
    break;
  case 'Q': // Send sensor data
    for (axis = ROLL; axis < LASTAXIS; axis++) {
      Serial.print(gyro.getData(axis));
      comma();
    }
    for (axis = ROLL; axis < LASTAXIS; axis++) {
      Serial.print(accel.getData(axis));
      comma();
    }
    for (axis = ROLL; axis < YAW; axis++) {
      Serial.print(levelAdjust[axis]);
      comma();
    }
    Serial.print(flightAngle[ROLL]);
    comma();
    Serial.print(flightAngle[PITCH]);
    Serial.println();
    break;
  case 'R': // Send raw sensor data
    Serial.print(analogRead(ROLLRATEPIN));
    comma();
    Serial.print(analogRead(PITCHRATEPIN));
    comma();
    Serial.print(analogRead(YAWRATEPIN));
    comma();
    Serial.print(analogRead(ROLLACCELPIN));
    comma();
    Serial.print(analogRead(PITCHACCELPIN));
    comma();
    Serial.println(analogRead(ZACCELPIN));
    break;
  case 'S': // Send all flight data
    Serial.print(deltaTime);
    comma();
    for (axis = ROLL; axis < LASTAXIS; axis++) {
      Serial.print(gyro.getData(axis));
      comma();
    }
    Serial.print(transmitterCommand[THROTTLE]);
    comma();
    for (axis = ROLL; axis < LASTAXIS; axis++) {
      Serial.print(motorAxisCommand[axis]);
      comma();
    }
    for (motor = FRONT; motor < LASTMOTOR; motor++) {
      Serial.print(motorCommand[motor]);
      comma();
    }
    for (axis = ROLL; axis < LASTAXIS; axis++) {
      Serial.print(accel.getData(axis));
      comma();
    }
     Serial.print(armed, BIN);
    comma();
    if (flightMode == STABLE)
      Serial.println(2000);
    if (flightMode == ACRO)
      Serial.println(1000);
    break;
   case 'T': // Send processed transmitter values
    Serial.print(xmitFactor);
    comma();
    for (axis = ROLL; axis < LASTAXIS; axis++) {
      Serial.print(transmitterCommand[axis]);
      comma();
    }
    for (axis = ROLL; axis < YAW; axis++) {
      Serial.print(levelAdjust[axis]);
      comma();
    }
    Serial.print(motorAxisCommand[ROLL]);
    comma();
    Serial.print(motorAxisCommand[PITCH]);
    comma();
    Serial.println(motorAxisCommand[YAW]);
    break;
  case 'U': // Send smoothed receiver values
    for (channel = ROLL; channel < AUX; channel++) {
      Serial.print(transmitterCommandSmooth[channel]);
      comma();
    }
    Serial.println(transmitterCommandSmooth[AUX]);
    break;
  case 'V': // Send receiver status
    for (channel = ROLL; channel < AUX; channel++) {
      Serial.print(receiverData[channel]);
      comma();
    }
    Serial.println(receiverData[AUX]);
    break;
  case 'X': // Stop sending messages
    break;
  case 'Z': // Send heading
    Serial.print(transmitterCommand[YAW]);
    comma();
    Serial.print(headingHold);
    comma();
    Serial.print(heading);
    comma();
    Serial.println(currentHeading);
    break;
  case '6': // Report remote commands
    for (motor = FRONT; motor < LEFT; motor++) {
      Serial.print(remoteCommand[motor]);
      comma();
    }
    Serial.println(remoteCommand[LEFT]);
    break;
  case '!': // Send flight software version
    Serial.println("1.8");
    queryType = 'X';
    break;
  case 'e': // Send AREF value
    Serial.println(aref);
    queryType = 'X';
    break;
  }
}

// Used to read floating point values from the serial port
float readFloatSerial() {
  byte index = 0;
  byte timeout = 0;
  char data[128] = "";

  do {
    if (Serial.available() == 0) {
      delay(10);
      timeout++;
    }
    else {
      data[index] = Serial.read();
      timeout = 0;
      index++;
    }
  }  while ((data[limitRange(index-1, 0, 128)] != ';') && (timeout < 5) && (index < 128));
  return atof(data);
}


void comma() {
  Serial.print(',');
}

void printInt(int data) {
  byte msb, lsb;
  
  msb = data >> 8;
  lsb = data & 0xff;
  
  Serial.print(msb, BYTE);
  Serial.print(lsb, BYTE);
}