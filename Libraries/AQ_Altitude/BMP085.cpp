/*
  AeroQuad v2.2 - Feburary 2011
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
#include <WProgram.h>
#include "BMP085.h"
#include <I2C.h>
#include <AQMath.h>

#define TEMPERATURE 0
#define PRESSURE 1
void BMP085::requestRawPressure(void) {
    I2C::updateRegisterI2C(altitudeAddress, 0xF4, 0x34+(overSamplingSetting<<6));
  }
  
  long BMP085::readRawPressure(void) {
    unsigned char msb, lsb, xlsb;
    I2C::sendByteI2C(altitudeAddress, 0xF6);
    Wire.requestFrom(altitudeAddress, 3); // request three bytes
    while(!Wire.available()); // wait until data available
    msb = Wire.receive();
    while(!Wire.available()); // wait until data available
    lsb = Wire.receive();
    while(!Wire.available()); // wait until data available
    xlsb = Wire.receive();
    return (((long)msb<<16) | ((long)lsb<<8) | ((long)xlsb)) >>(8-overSamplingSetting);
  }

  void BMP085::requestRawTemperature(void) {
    I2C::updateRegisterI2C(altitudeAddress, 0xF4, 0x2E);
  }
  
  unsigned int BMP085::readRawTemperature(void) {
    I2C::sendByteI2C(altitudeAddress, 0xF6);
    return I2C::readWordWaitI2C(altitudeAddress);
  }

  void BMP085::initialize()
  {
	altitudeAddress = 0x77;
    // oversampling setting
    // 0 = ultra low power
    // 1 = standard
    // 2 = high
    // 3 = ultra high resolution
    overSamplingSetting = 3;
    pressure = 0;
    groundPressure = 0;
    temperature = 0;
    
    groundTemperature = 0;
    groundAltitude = 0;
    pressureFactor = 1/5.255;
	
	I2C::sendByteI2C(altitudeAddress, 0xAA);
    ac1 = I2C::readWordWaitI2C(altitudeAddress);
    I2C::sendByteI2C(altitudeAddress, 0xAC);
    ac2 = I2C::readWordWaitI2C(altitudeAddress);
    I2C::sendByteI2C(altitudeAddress, 0xAE);
    ac3 = I2C::readWordWaitI2C(altitudeAddress);
    I2C::sendByteI2C(altitudeAddress, 0xB0);
    ac4 = I2C::readWordWaitI2C(altitudeAddress);
	
	I2C::sendByteI2C(altitudeAddress, 0xB2);
    ac5 = I2C::readWordWaitI2C(altitudeAddress);
    I2C::sendByteI2C(altitudeAddress, 0xB4);
    ac6 = I2C::readWordWaitI2C(altitudeAddress);
    I2C::sendByteI2C(altitudeAddress, 0xB6);
    b1 = I2C::readWordWaitI2C(altitudeAddress);
    I2C::sendByteI2C(altitudeAddress, 0xB8);
    b2 = I2C::readWordWaitI2C(altitudeAddress);
    I2C::sendByteI2C(altitudeAddress, 0xBA);
    mb = I2C::readWordWaitI2C(altitudeAddress);
    I2C::sendByteI2C(altitudeAddress, 0xBC);
    mc = I2C::readWordWaitI2C(altitudeAddress);
    I2C::sendByteI2C(altitudeAddress, 0xBE);
    md = I2C::readWordWaitI2C(altitudeAddress);
    requestRawTemperature(); // setup up next measure() for temperature
    select = TEMPERATURE;
    pressureCount = 0;
    measure();
    delay(5); // delay for temperature
    measure();
    delay(26); // delay for pressure
    measureGround();
    // check if measured ground altitude is valid
    while (abs(getRawData() - getGroundAltitude()) > 10) {
      delay(26);
      measureGround();
    }
    setStartAltitude(getGroundAltitude());
  }

  void BMP085::measure()  
  {
    long x1, x2, x3, b3, b5, b6, p;
    unsigned long b4, b7;
    int32_t tmp;

    // switch between pressure and tempature measurements
    // each loop, since it's slow to measure pressure
    if (select == PRESSURE) {
      rawPressure = readRawPressure();
      if (pressureCount == 1) {
        requestRawTemperature();
        pressureCount = 0;
       select = TEMPERATURE;
      }
      else
        requestRawPressure();
      pressureCount++;
    }
    else { // select must equal TEMPERATURE
      rawTemperature = (long)readRawTemperature();
      requestRawPressure();
      select = PRESSURE;
    }
    
    //calculate true temperature
    x1 = ((long)rawTemperature - ac6) * ac5 >> 15;
    x2 = ((long) mc << 11) / (x1 + md);
    b5 = x1 + x2;
    temperature = ((b5 + 8) >> 4);
  
    //calculate true pressure
    b6 = b5 - 4000;
    x1 = (b2 * (b6 * b6 >> 12)) >> 11; 
    x2 = ac2 * b6 >> 11;
    x3 = x1 + x2;
 
    // Real Bosch formula - b3 = ((((int32_t)ac1 * 4 + x3) << overSamplingSetting) + 2) >> 2;
    // The version below is the same, but takes less program space
    tmp = ac1;
    tmp = (tmp * 4 + x3) << overSamplingSetting;
    b3 = (tmp + 2) >> 2;
 


    x1 = ac3 * b6 >> 13;
    x2 = (b1 * (b6 * b6 >> 12)) >> 16;
    x3 = ((x1 + x2) + 2) >> 2;
    b4 = (ac4 * (uint32_t) (x3 + 32768)) >> 15;
    b7 = ((uint32_t) rawPressure - b3) * (50000 >> overSamplingSetting);
    p = b7 < 0x80000000 ? (b7 << 1) / b4 : (b7 / b4) >> 1;
    
    x1 = (p >> 8) * (p >> 8);
    x1 = (x1 * 3038) >> 16;
    x2 = (-7357 * p) >> 16;
    pressure = (p + ((x1 + x2 + 3791) >> 4));
    
    rawAltitude = 44330 * (1 - pow(pressure/101325.0, pressureFactor)); // returns absolute altitude in meters
    //rawAltitude = (101325.0-pressure)/4096*346;
    //accel.calculateAltitude(); //cumulates onto rawAltitude from fast filtered accel Z reads
    //currentTime = micros();
    altitude = AQMath::filterSmooth(rawAltitude, altitude, smoothFactor);
    //previousTime = currentTime;
  }

