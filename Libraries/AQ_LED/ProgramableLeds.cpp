

#include <ProgramableLeds.h>

  void ProgramableLeds::initialize(void) {
    byte i;
    // map pins
    LEDport[0] = 43;
    LEDport[1] = 44;
    LEDport[2] = 45;
    LEDport[3] = 46;
    LEDport[4] = 47;
    LEDport[5] = 48;
    LEDport[6] = 49;
    
    for (i=0; i < NR_OF_LEDS; i++) {  
      pinMode(LEDport[i], OUTPUT);
    }
  }

  void ProgramableLeds::on(byte id) {
    digitalWrite(LEDport[id], HIGH);
  }
  
  void ProgramableLeds::off(byte id) {
    digitalWrite(LEDport[id], LOW);
  }
  
  void ProgramableLeds::startProgram(byte no)
  {
	currentState = 0;
	currentProgram = no;
	nextStep();
  }
  void ProgramableLeds::nextStep()
  {
	switch(currentState)
	{
	  case 0:
	  case 1:
		on(0);
		off(1);
	    off(2);
	    off(3);
		currentState++;
	  break;
	  case 2:
	  case 3:
		off(0);
		on(1);
	    off(2);
	    off(3);
		currentState++;
	  break;
	  case 4:
	  case 5:
		off(0);
		off(1);
	    on(2);
	    off(3);
		currentState++;
	  break;
	  case 6:
	  case 7:
		off(0);
		off(1);
	    off(2);
	    on(3);
		currentState++;
	  break;
	  }
	  currentState++;
	  currentState = currentState % 8;
	  
  }
