#define NR_OF_LEDS 7

// ***********************************************************************
// ************************** Example Class ******************************
// ***********************************************************************
class LedProvider {
public: 
  byte LEDport[NR_OF_LEDS];
  byte state[NR_OF_LEDS];

  // **********************************************************************
  // The following function calls must be defined inside any new subclasses
  // **********************************************************************
  virtual void initialize(void); 
  virtual void on(byte id); 
  virtual void off(byte id);
  virtual void startProgram(byte no);
  virtual void nextStep();
};
