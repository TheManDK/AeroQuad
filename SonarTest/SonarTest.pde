/*
MaxSonar_EZ1 Test
 SummitBid Technologies 2010
 */
const int sonarSensorPin = A1;
const int ledPin = 13;

long value = 0;
int cm = 0;
int inches = 0;

void setup()
{
  Serial.begin(57600);
  pinMode(ledPin, OUTPUT);

}

void loop()
{
  digitalWrite(ledPin, HIGH);
  // Read the value from the sensor pin
  value = analogRead(sonarSensorPin);
  cm = value / 58;    // pulse width is 58 microsecs per cm
  inches = value / 147;     // which is 147 microeconds per inch
  //Serial.print("cm : inches = ");
  //Serial.print(value);
  //Serial.print(':');
  //Serial.println(inches);
  Serial.println(value);
  
  
  digitalWrite(ledPin, LOW);
  delay(20);

}

 
