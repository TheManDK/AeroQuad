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
  mavlink_msg_attitude_pack(MAV_SYSTEM_ID, MAV_COMPONENT_ID, &msg, 0, flightAngle->getData(ROLL) * 3.14159 / 180, flightAngle->getData(PITCH) * 3.14159 / 180, flightAngle->getData(YAW) * 3.14159 / 180, 0, 0, 0);
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
  mavlink_msg_rc_channels_raw_pack(MAV_SYSTEM_ID, MAV_COMPONENT_ID, &msg, receiver.getRaw(0), receiver.getRaw(1), receiver.getRaw(2), receiver.getRaw(3), receiver.getRaw(4), receiver.getRaw(5), receiver.getRaw(6), receiver.getRaw(7), 64);
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
#endif
