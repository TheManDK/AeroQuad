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

#ifndef BMP085_H
#define BMP085_H

#include "AltitudeProvider.h"
#define TEMPERATURE 0
#define PRESSURE 1

class BMP085 : public AltitudeProvider 
{
private:
  byte overSamplingSetting;
  int ac1, ac2, ac3;
  unsigned int ac4, ac5, ac6;
  int b1, b2, mb, mc, md;
  long pressure;
  long temperature;
  int altitudeAddress;
  long rawPressure, rawTemperature;
  byte select, pressureCount;
  float pressureFactor;
  

public:
  void initialize();

  void measure();
  
  void requestRawPressure(void);
  long readRawPressure(void);
  void requestRawTemperature(void);
  unsigned int readRawTemperature(void);
};
#endif