#ifndef PROGRAMABLELEDS_H
#define PROGRAMABLELEDS_H

#include <WProgram.h>
#include <LedProvider.h>

class ProgramableLeds : public LedProvider { 

private:
  byte currentState;
  byte currentProgram;
public: 
  ProgramableLeds() : LedProvider(){
    // this is the constructor of the object and must have the same name
    // can be used to initialize any of the variables declared above
  }

  // ***********************************************************
  // Define all the virtual functions declared in the main class
  // ***********************************************************
  void initialize(void);

  void on(byte id);
  void off(byte id);
  
  void startProgram(byte no);
  void nextStep();
};
#endif
