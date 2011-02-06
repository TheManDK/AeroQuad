/*
  AeroQuad v2.1 - January 2011
  www.AeroQuad.com
  Copyright (c) 2011 Ted Carancho.  All rights reserved.
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

// MavLink.pde is responsible for the serial communication for commands and telemetry from the AeroQuad
// This comtains readSerialCommand() which listens for a serial command and it's arguments
// This also contains readSerialTelemetry() which listens for a telemetry request and responds with the requested data

//***************************************************************************************************
//********************************** Serial Commands ************************************************
//***************************************************************************************************
#ifdef MAVLINK
//#include <FastSerial.h>
#include "../mavlink/include/common/mavlink.h"        // Mavlink interface
const int system_type = MAV_QUADROTOR;
const int autopilot_type = MAV_AUTOPILOT_GENERIC;

int system_mode = MAV_MODE_UNINIT;
int system_nav_mode = MAV_NAV_GROUNDED;
int system_status = MAV_STATE_ACTIVE;

long system_dropped_packets = 0;


mavlink_message_t msg; 
uint8_t buf[MAVLINK_MAX_PACKET_LEN];
mavlink_status_t status;



void readSerialPID(unsigned char PIDid) {
  struct PIDdata* pid = &PID[PIDid];
  pid->P = readFloatSerial();
  pid->I = readFloatSerial();
  pid->D = readFloatSerial();
  pid->lastPosition = 0;
  pid->integratedError = 0;
}

void readSerialMavLink() {
  while(Serial.available() > 0) { 
    uint8_t c = Serial.read();
    //try to get a new message 
      if(mavlink_parse_char(MAVLINK_COMM_0, c, &msg, &status)) { 
        // Handle message
        switch(msg.msgid) {
          case MAVLINK_MSG_ID_SET_MODE: {
            system_mode = mavlink_msg_set_mode_get_mode(&msg);
            sendSerialSysStatus();
          }
          break;
          case MAVLINK_MSG_ID_ACTION: {
            uint8_t result = 0;
            
            if (mavlink_msg_action_get_target(&msg) != MAV_SYSTEM_ID || mavlink_msg_action_get_target_component(&msg) != MAV_COMPONENT_ID) return;
              uint8_t action = mavlink_msg_action_get_action(&msg);
              switch(action) {
                MAV_ACTION_MOTORS_START: {
                  armed = ON;
                  result = 1;
                  system_status = MAV_STATE_ACTIVE;
                  sendSerialSysStatus();
                }
                break;
                MAV_ACTION_MOTORS_STOP: {
                  armed = OFF;
                  result = 1;
                  system_status = MAV_STATE_STANDBY;
                  sendSerialSysStatus();
                }
                break;
                MAV_ACTION_CALIBRATE_GYRO: {
                  if (system_status == MAV_STATE_STANDBY)
                  {
                    gyro.calibrate();
                    result = 1;
                  }
                  else
                  {
                  result = 0;
                  }
                }                
                break;                    
              }
             
             mavlink_msg_action_ack_pack(MAV_SYSTEM_ID, MAV_COMPONENT_ID, &msg, action, result);
             uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
             Serial.write(buf, len);
          }
          break;
          case MAVLINK_MSG_ID_MANUAL_CONTROL: {
            if (mavlink_msg_manual_control_get_target(&msg) != MAV_SYSTEM_ID || system_mode != MAV_MODE_MANUAL) return;
            if (mavlink_msg_manual_control_get_roll_manual(&msg))
            {
              receiver.rawData[ROLL] = (int)(((mavlink_msg_manual_control_get_roll(&msg)+0.5)*1000)+1000);
            }
            if (mavlink_msg_manual_control_get_pitch_manual(&msg))
            {
              receiver.rawData[PITCH] = (int)(((mavlink_msg_manual_control_get_pitch(&msg)+0.5)*1000)+1000);
            }
            if (mavlink_msg_manual_control_get_yaw_manual(&msg))
            {
              receiver.rawData[YAW] = (int)(((mavlink_msg_manual_control_get_yaw(&msg)+0.5)*1000)+1000);
            }
            if (mavlink_msg_manual_control_get_thrust_manual(&msg))
            {
              receiver.rawData[THROTTLE] = (int)(((mavlink_msg_manual_control_get_thrust(&msg)+0.5)*1000)+1000);
            }
            
          }
          break;
          case MAVLINK_MSG_ID_PARAM_REQUEST_LIST: {
            int8_t roll_p[15] = "Roll_P";
            int8_t roll_i[15] = "Roll_I";
            int8_t roll_d[15] = "Roll_D";
            sendSerialPID(ROLL, roll_p, roll_i, roll_d, 1, 24);
            
            int8_t pitch_p[15] = "Pitch_P";
            int8_t pitch_i[15] = "Pitch_I";
            int8_t pitch_d[15] = "Pitch_D";
            sendSerialPID(PITCH, pitch_p, pitch_i, pitch_d, 4, 24);
            
            int8_t yaw_p[15] = "Yaw_P";
            int8_t yaw_i[15] = "Yaw_I";
            int8_t yaw_d[15] = "Yaw_D";
            sendSerialPID(YAW, yaw_p, yaw_i, yaw_d, 7, 24);
            
            int8_t heading_p[15] = "Heading_P";
            int8_t heading_i[15] = "Heading_I";
            int8_t heading_d[15] = "Heading_D";
            sendSerialPID(HEADING, heading_p, heading_i, heading_d, 10, 24);
            
            int8_t levelroll_p[15] = "Level Roll_P";
            int8_t levelroll_i[15] = "Level Roll_I";
            int8_t levelroll_d[15] = "Level Roll_D";
            sendSerialPID(LEVELROLL, levelroll_p, levelroll_i, levelroll_d, 13, 24);
                        
            int8_t levelpitch_p[15] = "Level Pitch_P";
            int8_t levelpitch_i[15] = "Level Pitch_I";
            int8_t levelpitch_d[15] = "Level Pitch_D";
            sendSerialPID(LEVELPITCH, levelpitch_p, levelpitch_i, levelpitch_d, 16, 24);
            
            int8_t levelgyroroll_p[15] = "Lvl gyro rol_P";
            int8_t levelgyroroll_i[15] = "Lvl gyro rol_I";
            int8_t levelgyroroll_d[15] = "Lvl gyro rol_D";
            sendSerialPID(LEVELGYROROLL, levelgyroroll_p, levelgyroroll_i, levelgyroroll_d, 19, 24);
            
            int8_t levelgyropitch_p[15] = "Lvl gyro pit_P";
            int8_t levelgyropitch_i[15] = "Lvl gyro pit_I";
            int8_t levelgyropitch_d[15] = "Lvl gyro pit_D";
            sendSerialPID(LEVELGYROPITCH, levelgyropitch_p, levelgyropitch_i, levelgyropitch_d, 22, 24);
          }
          break;
          default:
            //Do nothing
          break;
        }
      } 
      // And get the next one
    } 
    system_dropped_packets += status.packet_rx_drop_count;
}


void readSerialCommand() {
  // Check for serial message
  
  if (Serial.available()) {
    digitalWrite(LEDPIN, LOW);
    queryType = Serial.read();
    switch (queryType) {
    case 'A': // Receive roll and pitch gyro PID
      readSerialPID(ROLL);
      readSerialPID(PITCH);
      minAcro = readFloatSerial();
      break;
    case 'C': // Receive yaw PID
      readSerialPID(YAW);
      readSerialPID(HEADING);
      headingHoldConfig = readFloatSerial();
      heading = 0;
      relativeHeading = 0;
      headingHold = 0;
      break;
    case 'E': // Receive roll and pitch auto level PID
      readSerialPID(LEVELROLL);
      readSerialPID(LEVELPITCH);
      readSerialPID(LEVELGYROROLL);
      readSerialPID(LEVELGYROPITCH);
      windupGuard = readFloatSerial();
      break;
    case 'G': // Receive auto level configuration
      levelLimit = readFloatSerial();
      levelOff = readFloatSerial();
      break;
    case 'I': // Receiver altitude hold PID
#ifdef AltitudeHold
      readSerialPID(ALTITUDE);
      PID[ALTITUDE].windupGuard = readFloatSerial();
      minThrottleAdjust = readFloatSerial();
      maxThrottleAdjust = readFloatSerial();
      altitude.setSmoothFactor(readFloatSerial());
      readSerialPID(ZDAMPENING);
#endif
      break;
    case 'K': // Receive data filtering values
      gyro.setSmoothFactor(readFloatSerial());
      accel.setSmoothFactor(readFloatSerial());
      timeConstant = readFloatSerial();
#if defined(AeroQuad_v1) || defined(AeroQuad_v18)
      flightAngle.initialize();
#endif
      break;
    case 'M': // Receive transmitter smoothing values
      receiver.setXmitFactor(readFloatSerial());
      for(byte channel = ROLL; channel<LASTCHANNEL; channel++) {
        receiver.setSmoothFactor(channel, readFloatSerial());
      }
      break;
    case 'O': // Receive transmitter calibration values
      for(byte channel = ROLL; channel<LASTCHANNEL; channel++) {
        receiver.setTransmitterSlope(channel, readFloatSerial());
        receiver.setTransmitterOffset(channel, readFloatSerial());
      }
      break;
    case 'W': // Write all user configurable values to EEPROM
      writeEEPROM(); // defined in DataStorage.h
      zeroIntegralError();
      break;
    case 'Y': // Initialize EEPROM with default values
      initializeEEPROM(); // defined in DataStorage.h
      gyro.calibrate();
      accel.calibrate();
      zeroIntegralError();
#ifdef HeadingMagHold
      compass.initialize();
#endif
#ifdef AltitudeHold
      altitude.initialize();
#endif
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
      for (byte motor = FRONT; motor < LASTMOTOR; motor++)
        motors.setRemoteCommand(motor, readFloatSerial());
      break;
    case 'a':
      // spare
      break;
    case 'b': // calibrate gyros
      gyro.calibrate();
      break;
    case 'c': // calibrate accels
      accel.calibrate();
#if defined(AeroQuadMega_CHR6DM) || defined(APM_OP_CHR6DM)
      flightAngle.calibrate();
      accel.setOneG(accel.getFlightData(ZAXIS));
#endif
      break;
    case 'd': // send aref
      aref = readFloatSerial();
      break;
    case 'f': // calibrate magnetometer
#ifdef HeadingMagHold
      compass.setMagCal(XAXIS, readFloatSerial(), readFloatSerial());
      compass.setMagCal(YAXIS, readFloatSerial(), readFloatSerial());
      compass.setMagCal(ZAXIS, readFloatSerial(), readFloatSerial());
#endif
      break;
    case '~': //  read Camera values 
      #ifdef Camera
      camera.setMode(readFloatSerial());
      camera.setCenterPitch(readFloatSerial());
      camera.setCenterRoll(readFloatSerial());
      camera.setCenterYaw(readFloatSerial());
      camera.setmCameraPitch(readFloatSerial());
      camera.setmCameraRoll(readFloatSerial());
      camera.setmCameraYaw(readFloatSerial());
      camera.setServoMinPitch(readFloatSerial());
      camera.setServoMinRoll(readFloatSerial());
      camera.setServoMinYaw(readFloatSerial());
      camera.setServoMaxPitch(readFloatSerial());
      camera.setServoMaxRoll(readFloatSerial());
      camera.setServoMaxYaw(readFloatSerial());
      #endif
      break;
    }
    digitalWrite(LEDPIN, HIGH);
  }
}

//***************************************************************************************************
//********************************* Serial Telemetry ************************************************
//***************************************************************************************************

void PrintValueComma(float val)
{
  Serial.print(val);
  comma();
}

void PrintValueComma(char val)
{
  Serial.print(val);
  comma();
}

void PrintValueComma(int val)
{
  Serial.print(val);
  comma();
}

void PrintValueComma(unsigned long val)
{
  Serial.print(val);
  comma();
}

void PrintPID(unsigned char IDPid)
{
  PrintValueComma(PID[IDPid].P);
  PrintValueComma(PID[IDPid].I);
  PrintValueComma(PID[IDPid].D);
}

void sendSerialHeartbeat() {
  mavlink_msg_heartbeat_pack(MAV_SYSTEM_ID, MAV_COMPONENT_ID, &msg, system_type, autopilot_type);
  uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
  Serial.write(buf, len);
}

void sendSerialRawIMU() {
  mavlink_msg_raw_imu_pack(MAV_SYSTEM_ID, MAV_COMPONENT_ID, &msg, 0, accel.getRaw(XAXIS), accel.getRaw(YAXIS), accel.getRaw(ZAXIS), gyro.getRaw(XAXIS), gyro.getRaw(YAXIS), gyro.getRaw(ZAXIS), compass.getRawData(XAXIS), compass.getRawData(YAXIS), compass.getRawData(ZAXIS));
  uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
  Serial.write(buf, len);
}

void sendSerialAttitude() {
  mavlink_msg_attitude_pack(MAV_SYSTEM_ID, MAV_COMPONENT_ID, &msg, 0, flightAngle.getData(ROLL) * 3.14159 / 180, flightAngle.getData(PITCH) * 3.14159 / 180, flightAngle.getData(YAW) * 3.14159 / 180, gyro.rateDegPerSec(XAXIS), gyro.rateDegPerSec(YAXIS), gyro.rateDegPerSec(ZAXIS));
  uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
  Serial.write(buf, len);
}

void sendSerialAltitude() {
  mavlink_msg_set_altitude_pack(MAV_SYSTEM_ID, MAV_COMPONENT_ID, &msg, MAV_COMPONENT_ID, (int)(1234));
  uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
  Serial.write(buf, len);
}

void sendSerialRawPressure() {
  mavlink_msg_raw_pressure_pack(MAV_SYSTEM_ID, MAV_COMPONENT_ID, &msg, MAV_COMPONENT_ID, (int)(1000*altitude.getRawData()), 0,0,0);
  uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
  Serial.write(buf, len);
}

void sendSerialBoot() {
  mavlink_msg_boot_pack(MAV_SYSTEM_ID, MAV_COMPONENT_ID, &msg, VERSION);
  uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
  Serial.write(buf, len);
}

void sendSerialRcRaw() {
  mavlink_msg_rc_channels_raw_pack(MAV_SYSTEM_ID, MAV_COMPONENT_ID, &msg, receiver.rawData[0], receiver.rawData[1], receiver.rawData[2], receiver.rawData[3], receiver.rawData[4], receiver.rawData[5], receiver.rawData[6], receiver.rawData[7], 64);
  uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
  Serial.write(buf, len);
}

void sendSerialPID(int IDPid, int8_t id_p[], int8_t id_i[], int8_t id_d[],int index, int listsize) {
  mavlink_msg_param_value_pack(MAV_SYSTEM_ID, MAV_COMPONENT_ID, &msg, id_p, PID[IDPid].P, index,listsize);
  uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
  Serial.write(buf, len);
  mavlink_msg_param_value_pack(MAV_SYSTEM_ID, MAV_COMPONENT_ID, &msg, id_i, PID[IDPid].I, index+1,listsize);
  len = mavlink_msg_to_send_buffer(buf, &msg);
  Serial.write(buf, len);
  mavlink_msg_param_value_pack(MAV_SYSTEM_ID, MAV_COMPONENT_ID, &msg, id_d, PID[IDPid].D, index+2,listsize);
  len = mavlink_msg_to_send_buffer(buf, &msg);
  Serial.write(buf, len);
}

void sendSerialParamValue(int8_t id[], float value, int index, int listsize) {
  mavlink_msg_param_value_pack(MAV_SYSTEM_ID, MAV_COMPONENT_ID, &msg, id, value, index,listsize);
  uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
  Serial.write(buf, len);
}

void sendSerialRcScaled() {
  mavlink_msg_rc_channels_scaled_pack(MAV_SYSTEM_ID, MAV_COMPONENT_ID, &msg, receiver.scaleToMavLink(0),receiver.scaleToMavLink(1),receiver.scaleToMavLink(2),receiver.scaleToMavLink(3),receiver.scaleToMavLink(4),receiver.scaleToMavLink(5),receiver.scaleToMavLink(6),receiver.scaleToMavLink(7), 128);
  uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
  Serial.write(buf, len);
}

void sendSerialSysStatus() {
  mavlink_msg_sys_status_pack(MAV_SYSTEM_ID, MAV_COMPONENT_ID, &msg, system_mode, system_nav_mode, system_status, (int)(deltaTime/15), (int)(batteryMonitor.getData()*1000), 0, system_dropped_packets);
  uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
  Serial.write(buf, len);
  
  mavlink_msg_named_value_int_pack(MAV_SYSTEM_ID, MAV_COMPONENT_ID, &msg, "mode", system_mode);
  len = mavlink_msg_to_send_buffer(buf, &msg);
  Serial.write(buf, len);
  
  mavlink_msg_named_value_int_pack(MAV_SYSTEM_ID, MAV_COMPONENT_ID, &msg, "nav_mode", system_nav_mode);
  len = mavlink_msg_to_send_buffer(buf, &msg);
  Serial.write(buf, len);
  
  mavlink_msg_named_value_int_pack(MAV_SYSTEM_ID, MAV_COMPONENT_ID, &msg, "status", system_status);
  len = mavlink_msg_to_send_buffer(buf, &msg);
  Serial.write(buf, len);
  
  mavlink_msg_named_value_int_pack(MAV_SYSTEM_ID, MAV_COMPONENT_ID, &msg, "load", (int)(deltaTime/15));
  len = mavlink_msg_to_send_buffer(buf, &msg);
  Serial.write(buf, len);
}

void sendSerialTelemetry() {
  update = 0;
  switch (queryType) {
  case '=': // Reserved debug command to view any variable from Serial Monitor
    Serial.println(minAcro);
    queryType = 'X';
    break;
  case 'D': // Send yaw PID values
    Serial.println(headingHoldConfig, BIN);
    queryType = 'X';
    break;
  case 'F': // Send roll and pitch auto level PID values
    
    Serial.println(windupGuard);
    queryType = 'X';
    break;
  case 'H': // Send auto level configuration values
    //Serial.print(levelLimit);
    //comma();
		PrintValueComma(levelLimit);
    Serial.println(levelOff);
    queryType = 'X';
    break;
  case 'J': // Altitude Hold
#ifdef AltitudeHold
    PrintPID(ALTITUDE);
    PrintValueComma(PID[ALTITUDE].windupGuard);
    PrintValueComma(minThrottleAdjust);
    PrintValueComma(maxThrottleAdjust);
    PrintValueComma(altitude.getSmoothFactor());
    PrintValueComma(PID[ZDAMPENING].P);
    PrintValueComma(PID[ZDAMPENING].I);
    Serial.println(PID[ZDAMPENING].D);
#else
    for(byte i=0; i<9; i++) {
      PrintValueComma(0);
    }
    Serial.println('0');
#endif
    queryType = 'X';
    break;
  case 'L': // Send data filtering values
    PrintValueComma(gyro.getSmoothFactor());
    PrintValueComma(accel.getSmoothFactor());
    Serial.println(timeConstant);
    // comma();
    // Serial.println(flightMode, DEC);
    queryType = 'X';
    break;
  case 'N': // Send transmitter smoothing values
    PrintValueComma(receiver.getXmitFactor());
    for (byte axis = ROLL; axis < AUX; axis++) {
      PrintValueComma(receiver.getSmoothFactor(axis));
    }
    Serial.println(receiver.getSmoothFactor(AUX));
    queryType = 'X';
    break;
  case 'P': // Send transmitter calibration data
    for (byte axis = ROLL; axis < AUX; axis++) {
      PrintValueComma(receiver.getTransmitterSlope(axis));
      PrintValueComma(receiver.getTransmitterOffset(axis));
    }
    PrintValueComma(receiver.getTransmitterSlope(AUX));
    Serial.println(receiver.getTransmitterOffset(AUX));
    queryType = 'X';
    break;
  case 'Q': // Send sensor data
    for (byte axis = ROLL; axis < LASTAXIS; axis++) {
      PrintValueComma(gyro.getData(axis));
    }
    for (byte axis = ROLL; axis < LASTAXIS; axis++) {
      PrintValueComma(accel.getData(axis));
    }
    for (byte axis = ROLL; axis < YAW; axis++) {
      PrintValueComma(levelAdjust[axis]);
    }
    PrintValueComma(flightAngle.getData(ROLL));
    PrintValueComma(flightAngle.getData(PITCH));
    #if defined(HeadingMagHold) || defined(AeroQuadMega_CHR6DM) || defined(APM_OP_CHR6DM)
      PrintValueComma(compass.getAbsoluteHeading());
    #else
      PrintValueComma(0);
    #endif
    #ifdef AltitudeHold
      PrintValueComma(altitude.getData());
    #else
      PrintValueComma(0);
    #endif
    #ifdef BattMonitor
      Serial.print(batteryMonitor.getData());
    #else
      Serial.print(0);
    #endif
    Serial.println();
    break;

  case 'S': // Send all flight data
    PrintValueComma(deltaTime);
    for (byte axis = ROLL; axis < LASTAXIS; axis++) {
      PrintValueComma(gyro.getFlightData(axis));
    }
    #ifdef BattMonitor
      PrintValueComma(batteryMonitor.getData());
    #else
      PrintValueComma(0);
    #endif
    for (byte axis = ROLL; axis < LASTAXIS; axis++) {
      PrintValueComma(motors.getMotorAxisCommand(axis));
    }
    for (byte motor = FRONT; motor < LASTMOTOR; motor++) {
      PrintValueComma(motors.getMotorCommand(motor));
    }
    for (byte axis = ROLL; axis < LASTAXIS; axis++) {
      PrintValueComma(accel.getFlightData(axis));
    }
    Serial.print(armed, BIN);
    comma();
    if (flightMode == STABLE)
      PrintValueComma(2000);
    if (flightMode == ACRO)
      PrintValueComma(1000);
    #if defined(HeadingMagHold) || defined(AeroQuadMega_CHR6DM) || defined(APM_OP_CHR6DM)
      PrintValueComma(compass.getAbsoluteHeading());
    #else
      PrintValueComma(0);
    #endif
    #ifdef AltitudeHold
      PrintValueComma(altitude.getData());
      Serial.print(altitudeHold, DEC);
    #else
      PrintValueComma(0);
      Serial.print('0');
    #endif
    Serial.println();    
    break;
  case 'T': // Send processed transmitter values
    PrintValueComma(receiver.getXmitFactor());
    for (byte axis = ROLL; axis < LASTAXIS; axis++) {
      PrintValueComma(receiver.getData(axis));
    }
    for (byte axis = ROLL; axis < YAW; axis++) {
      PrintValueComma(levelAdjust[axis]);
    }
    PrintValueComma(motors.getMotorAxisCommand(ROLL));
    PrintValueComma(motors.getMotorAxisCommand(PITCH));
    Serial.println(motors.getMotorAxisCommand(YAW));
    break;
  case 'U': // Send smoothed receiver with Transmitter Factor applied values
    for (byte channel = ROLL; channel < AUX; channel++) {
      PrintValueComma(receiver.getData(channel));
    }
    Serial.println(receiver.getData(AUX));
    break;
  case 'V': // Send receiver status
    for (byte channel = ROLL; channel < AUX; channel++) {
      PrintValueComma(receiver.getRaw(channel));
    }
    Serial.println(receiver.getRaw(AUX));
    break;
  case 'X': // Stop sending messages
    break;
  case 'Z': // Send heading
    PrintValueComma(receiver.getData(YAW));
    PrintValueComma(headingHold);
    PrintValueComma(setHeading);
    Serial.println(relativeHeading);
    break;
  case '6': // Report remote commands
    for (byte motor = FRONT; motor < LEFT; motor++) {
      PrintValueComma(motors.getRemoteCommand(motor));
    }
    Serial.println(motors.getRemoteCommand(LEFT));
    break;
  case '!': // Send flight software version
    Serial.println(VERSION, 1);
    queryType = 'X';
    break;
  case '#': // Send software configuration
    // Determine which hardware is used to define max/min sensor values for Configurator plots
#if defined(AeroQuad_v1)
    PrintValueComma(0);
#elif defined(AeroQuadMega_v1)
    PrintValueComma(1);
#elif defined(AeroQuad_v18)
    PrintValueComma(2);
#elif defined(AeroQuadMega_v2)
    PrintValueComma(3);
#elif defined(AeroQuad_Wii)
    PrintValueComma(4);
#elif defined(AeroQuadMega_Wii)
    PrintValueComma(5);
#elif defined(ArduCopter)
    PrintValueComma(6);
#elif defined(Multipilot)
    PrintValueComma(7);
#elif defined(MultipilotI2C)
    PrintValueComma(8);
#elif defined(AeroQuadMega_CHR6DM)
    PrintValueComma(5);
#elif defined(APM_OP_CHR6DM)
    PrintValueComma(6);
#endif
    // Determine which motor flight configuration for Configurator GUI
#if defined(plusConfig)
    Serial.print('0');
#elif defined(XConfig)
    Serial.print('1');
#elif defined(HEXACOAXIAL)
    Serial.print('2');
#elif defined(HEXARADIAL)
    Serial.print('3');
#endif
    Serial.println();
    queryType = 'X';
    break;  
  case 'e': // Send AREF value
    Serial.println(aref);
    queryType = 'X';
    break;
  case 'g': // Send magnetometer cal values
#ifdef HeadingMagHold
    Serial.print(compass.getMagMax(XAXIS), 2);
    comma();
    Serial.print(compass.getMagMin(XAXIS), 2);
    comma();
    Serial.print(compass.getMagMax(YAXIS), 2);
    comma();
    Serial.print(compass.getMagMin(YAXIS), 2);
    comma();
    Serial.print(compass.getMagMax(ZAXIS), 2);
    comma();
    Serial.println(compass.getMagMin(ZAXIS), 2);
#endif
    queryType = 'X';
    break;
  case '`': // Send Camera values 
    #ifdef Camera
    //Serial.print(camera.getMode());
    //comma();
    PrintValueComma(camera.getMode());
    //Serial.print(camera.getCenterPitch());
    //comma();
    PrintValueComma(camera.getCenterPitch());
    //Serial.print(camera.getCenterRoll());
    //comma();
    PrintValueComma(camera.getCenterRoll());
    //Serial.print(camera.getCenterYaw());
    //comma();
    PrintValueComma(camera.getCenterYaw());
    Serial.print(camera.getmCameraPitch() , 2);
    comma();
    Serial.print(camera.getmCameraRoll() , 2);
    comma();
    Serial.print(camera.getmCameraYaw() , 2);
    comma();
    //Serial.print(camera.getServoMinPitch());
    //comma();
    PrintValueComma(camera.getServoMinPitch());
    //Serial.print(camera.getServoMinRoll());
    //comma();
    PrintValueComma(camera.getServoMinRoll());
    //Serial.print(camera.getServoMinYaw());
    //comma();
    PrintValueComma(camera.getServoMinYaw());
    //Serial.print(camera.getServoMaxPitch());
    //comma();
    PrintValueComma(camera.getServoMaxPitch());
    //Serial.print(camera.getServoMaxRoll());
    //comma();
    PrintValueComma(camera.getServoMaxRoll());
    Serial.println(camera.getServoMaxYaw());
    #endif
    break;
  }
}

// Used to read floating point values from the serial port
float readFloatSerial() {
  #define SERIALFLOATSIZE 10
  byte index = 0;
  byte timeout = 0;
  char data[SERIALFLOATSIZE] = "";

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
  }  
  while ((index == 0 || data[index-1] != ';') && (timeout < 5) && (index < sizeof(data)-1));
  data[index] = '\0';
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
#endif
