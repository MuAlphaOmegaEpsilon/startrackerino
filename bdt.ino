#include <EEPROMex.h>
#include <math.h>

/*--- Constants and enums, don't touch these ---*/ 
const float earthRotPeriod = 23.0f * 3600.0f + 56.0f * 60.0f + 4.0f;//Seconds in a solar day
const float deg2Rad = M_PI / 180.0f;                                //To convert from degrees to radiants
const float rad2Deg = 180.0f / M_PI;                                //To convert from radians to degrees
enum RotationMode {TRACKING, SLOW, NORMAL, FAST};                   //The movement mode, everything is linear except for TRACKING
enum MicroStepFactor {ONE, TWO, FOUR, EIGHT, SIXTEEN, THIRTYTWO};   //The factor of microstepping applied by the motor driver to the motor itself
enum EEPROMaction {LOAD, SAVE, INCOHERENT};                         //Used as argument in the dataInEEPROM function
enum MotionState {DISABLED, UP, DOWN};                              //The motion state of the threaded rod
enum BarnDoorTrackerType {saTANGENT, saISOSCELES,                   //The type of structure that the Barn Door Tracker represents, from a single-arm tangent to a double-arm type 4
                          daONE, daTWO, daTHREE, daFOUR};

/*--- Variables about pins, customizable by the user  ---*/
const int arduinoModelDigitalPins = 14;                             //The number of digital pins on this Arduino model

/*--- Digital connections, customizable by the user ---*/
int enablePin = 13;                                                 //Used to switch ON/OFF the motor
int ms0Pin = 12;                                                    //These 3 variables are used together to control the Micro-Stepping of the motor
int ms1Pin = 11;                                                    //
int ms2Pin = 10;                                                    //
int stepPin = 9;                                                    //Makes the motor step when spiking to 1
int directionPin = 8;                                               //Controls the spinning direction of the motor
int blueButtonPin = 7;                                              //The pin associated with the blue button
int greenButtonPin = 6;                                             //The pin associated with the green button
int yellowButtonPin = 5;                                            //The pin associated with the yellow button
int redButtonPin = 4;                                               //The pin associated with the red button
int redLedPin = 3;                                                  //Used to light up the red LED
int resetLedPin = 2;                                                //Used to light up the reset LED

/*--- Variables about the tracker, customizable by the user ---*/
const unsigned long filteringTime = 125;                            //Milliseconds, used for discarding buttons signals if sent consequently during this threshold (e.g. holding a button down)
const float stepsPerRevolution = 200.0f;                            //The number of steps that the stepper motor does for every revolution, WITHOUT taking micro-stepping into account
const float initialRodNutDelta = 2.5f;                              //Centimeters, the threaded rod delta value when the tracker is completely fold, this is set when recalibration occurs
const float rodPitch = 0.15f;                                       //Centimeters, the pitch length of the threaded rod
const float R = 50.0f;                                              //Centimeters, the distance from the main hinges to the nut center
const float B = 35.0f;                                              //Centimeters, this changes based on the Barn Door Tracker type, set accordingly
const float C = 4.0f;                                               //Centimeters, the distance between the camera board hinges and the main hinges
BarnDoorTrackerType bdtType = daTHREE;                              //The kind of structure used to build the barn door tracker
const MicroStepFactor selectedMicroStepping = TWO;

/*--- Tracking variables, don't touch these ---*/
unsigned long lastTimeButtonChanged [arduinoModelDigitalPins];
int lastButtonRead [arduinoModelDigitalPins];
const float Beta = B / C;                                           //Pure number
unsigned long lastLoopTime;                                         //The time in microseconds during the last loop cycle
float microStepFactor;                                              //The factor of micro-stepping actually set
float delayTime;                                                    //Microseconds, used in linear motion
float rodNutDelta;                                                  //Centimeters, the distance measured between the top of the rod and the nut
float timeDelta;                                                    //Seconds, the time represented by the tracker angle (e.g. an angle of 90° represents approx. 6 hours), it is a continuous value so it changes even if the rod is left a little behind
float actualRodNutDeltaPerStep;                                     //Centimeters, the increment in rodNutDelta for every single step done, taking into account micro-stepping
float nextAngle;                                                    //Radians, the angle that will be formed by the tracker if stepping once again
bool isValidNA = false;                                             //Tells if nextAngle is still valid or needs to be updated
bool error = false;                                                 //Determines if an error occurred
bool setupCompleted = false;                                        //Determines if, during the saving of data, the function has been called before or after the first initialization
MotionState motionState;                                            //Tells if the motor is spinning, and in which direction  
RotationMode rotationMode;                                          //Determines the type of movement function that the tracker is using
MicroStepFactor microStep;                                          //Specifies which micro-stepping is used by the DRV8825 to drive the stepper motor



//Given a pin number, reverses its value
void reverseDigitalPin (int pin)
{
  if (digitalRead (pin) == HIGH)
    digitalWrite (pin, LOW);

  else
    digitalWrite (pin, HIGH);
}



//Returns true only once, when the button is pressed down
bool readButton (byte buttonPin)
{
  int reading = digitalRead (buttonPin);
  bool result = false;

  if (reading != lastButtonRead [buttonPin])
  {
    if (reading == HIGH && millis () - lastTimeButtonChanged [buttonPin] > filteringTime)
      result = true;
      
    lastButtonRead [buttonPin] = reading;
    lastTimeButtonChanged [buttonPin] = millis ();
  }
  return result;
}



//Used to recalibrate the tracker when fully closed, this reassignes the base nutDelta
void blueButton ()
{
  recalibrate ();
  digitalWrite (redLedPin, LOW);
  digitalWrite (resetLedPin, HIGH);
  error = true;
}



//Used to gain info about the tracker when the motor is disabled
void greenButton ()
{
    info ();
    lastLoopTime = micros ();
  if (rotationMode != TRACKING)
  {
    setRotationMode (TRACKING);
  }
}



//Used to change from tracking mode to a constant speed mode
void yellowButton ()
{
  setRotationMode (nextEnumField (rotationMode, 4));
}



//Used to change motion, from disabled, to up, to down
void redButton ()
{
  setMotionState (nextEnumField (motionState, 3));
}



//Theta is the angle between the main board and the motor board, set isIsosceles to true in an isosceles configuration
float theta (float deltaRodNut, bool isIsosceles)
{
  if (isIsosceles)
    return 2.0f * asin (deltaRodNut / (2.0f * R));
  else
    return asin (deltaRodNut / R);
}



//Calculates the angle required in order to be synchronized with the sky after a certain time period
float calculateAngleFromTimeDelta (float timeDeltaIncrement = 0.0f)
{
  // 2PI * (time_passed / total_time)
  return 2.0f * M_PI * (timeDelta + timeDeltaIncrement)  / earthRotPeriod;
}



//Calculates the time passed from the angle described by the earth rotation, the tracker or anything else
float calculateTimeDeltaFromAngle (float angle)
{
  // total_time * (angle / 2PI)
  return angle * earthRotPeriod / (2.0f * M_PI);
}



//Calculates the PHI angle (the angle between the main board and the camera board) based on the rodNutDelta variable, eventually incremented
float calculateAngleFromRodNutDelta (float rodNutDeltaIncrement = 0.0f)
{
  float rnd = rodNutDelta + rodNutDeltaIncrement;
  float THETA = (bdtType == saTANGENT) ? theta (rnd, false) : theta (rnd, true);

  switch (bdtType)
  {
    case saTANGENT:
    return THETA;

    case saISOSCELES:
    return THETA;

    case daONE:
    return THETA - asin (sin (THETA) / Beta);

    case daTWO:
    return atan (Beta * sin (THETA) / (Beta * cos (THETA) + 1));

    case daTHREE:
    return atan (Beta * sin (THETA) / (Beta * cos (THETA) - 1));

    case daFOUR:
    return THETA + asin (sin (THETA) / Beta);
  }
}



//Calculates the time delta represented by rodNutDelta variable, eventually incremented
//Remember that the rodNutDelta assumes discrete values, so it won't output continuous timeDelta values!
float calculateTimeDeltaFromRodNutDelta (float rodNutDeltaIncrement = 0.0f)
{
  return calculateTimeDeltaFromAngle (calculateAngleFromRodNutDelta (rodNutDeltaIncrement));
}



//Used for setting a new rotation mode
void setRotationMode (RotationMode rm)
{
  rotationMode = rm;
  isValidNA = false;

  float stepsPerMillimeter = 0.1f / abs(actualRodNutDeltaPerStep);
  
  switch (rm)
  {
  case TRACKING:
    Serial.println ("\nTracking mode");
    lastLoopTime = micros ();
    timeDelta = calculateTimeDeltaFromRodNutDelta ();
    if (motionState == DISABLED)
      digitalWrite(redLedPin, HIGH);
    else
      digitalWrite(redLedPin, 0);
    break; 
    
  case SLOW:
    Serial.println ("\nSlow mode");
    delayTime = 1.0f / (stepsPerMillimeter);        //1mm per second
    digitalWrite(redLedPin, HIGH);
    break;  
    
  case NORMAL:
    Serial.println ("\nNormal mode");
    delayTime = 1.0f / (stepsPerMillimeter * 5.0f); //4mm per second
    digitalWrite(redLedPin, HIGH);
    break;
    
  case FAST:
    Serial.println ("\nFast mode");
    delayTime = 1.0f / (stepsPerMillimeter * 7.5f); //6mm per second
    digitalWrite(redLedPin, HIGH);
    break;
  }
}



//Used for setting a new motion state, choosing from Disabled, Up or Down
void setMotionState (MotionState ms)
{
  motionState = ms;
  isValidNA = false;

  switch (ms)
  {
  case DISABLED:
    Serial.println ("\nMotor DISABLED");
    digitalWrite(enablePin, HIGH);
    digitalWrite(redLedPin, HIGH);
    dataInEEPROM (SAVE);
    break;  
    
  case UP:
    dataInEEPROM (INCOHERENT);
    Serial.println ("\nMotor UP");   
    digitalWrite (directionPin, LOW);
    if (rotationMode != TRACKING)
      digitalWrite(redLedPin, HIGH);
    else
      digitalWrite(redLedPin, LOW);
    digitalWrite(enablePin, LOW);
    actualRodNutDeltaPerStep = abs (actualRodNutDeltaPerStep);  
    lastLoopTime = micros ();
    break;
    
  case DOWN:
    Serial.println ("\nMotor DOWN");
    digitalWrite (directionPin, HIGH); 
    if (rotationMode != TRACKING)
      digitalWrite(redLedPin, HIGH);
    else
      digitalWrite(redLedPin, LOW); 
    digitalWrite(enablePin, LOW); 
    actualRodNutDeltaPerStep = abs (actualRodNutDeltaPerStep) * -1.0f;  
    break;
  }
}



//Used to set a differente micro-stepping to the motor
void setMicroStepFactor (MicroStepFactor msf)
{
  microStep = msf;
  isValidNA = false;

  actualRodNutDeltaPerStep = ((motionState == DOWN) ? -1.0f : 1.0f) * rodPitch / stepsPerRevolution;
  
  switch (msf)
  {
  case ONE:
    Serial.println ("Stepping mode: 1x");
    microStepFactor = 1.0f;
    setMicroSteppingPins (LOW, LOW, LOW);
    break;  
    
  case TWO:
    Serial.println ("Stepping mode: 2x");
    microStepFactor = 2.0f;
    actualRodNutDeltaPerStep /= 2.0f;
    setMicroSteppingPins (HIGH, LOW, LOW);
    break;
    
  case FOUR:
    Serial.println ("Stepping mode: 4x");
    microStepFactor = 4.0f;
    actualRodNutDeltaPerStep /= 4.0f;
    setMicroSteppingPins (LOW, HIGH, LOW);
    break;
    
  case EIGHT:
    Serial.println ("Stepping mode: 8x");
    microStepFactor = 8.0f;
    actualRodNutDeltaPerStep /= 8.0f;
    setMicroSteppingPins (HIGH, HIGH, LOW);
    break;
    
  case SIXTEEN:
    Serial.println ("Stepping mode: 16x");
    microStepFactor = 16.0f;
    actualRodNutDeltaPerStep /= 16.0f;
    setMicroSteppingPins (LOW, LOW, HIGH);
    break;
    
  case THIRTYTWO:
    Serial.println ("Stepping mode: 32x");
    microStepFactor = 32.0f;
    actualRodNutDeltaPerStep /= 32.0f;
    setMicroSteppingPins (HIGH, HIGH, HIGH);
    break;
  }
}



//Used to print informations in the console
void info ()
{
  int hours = (float) millis () / 3600000.0f;
  int minutes = ((float) millis () - (float) hours * 3600000.0f) / 60000.0f;
  int seconds = ((float) millis () - (float) hours * 3600000.0f - (float) minutes * 60000.0f) / 1000.0f;
  Serial.println ("\nINFO\nArduino has been running for " + String (hours) + "h" + String (minutes) + "m" + String (seconds) + "s");
  Serial.println ("Angle from nutDelta: " + String (rad2Deg * calculateAngleFromRodNutDelta ()) + " degrees");
  Serial.println ("Angle from timeDelta: " + String (rad2Deg * calculateAngleFromTimeDelta ()) + " degrees");  
  Serial.println ("Nut extension: " + String (rodNutDelta) + "cm"); 
  Serial.println ("Time represented by angle: " + String ((int) calculateTimeDeltaFromRodNutDelta () / 60) + " minutes and " + String ((int) calculateTimeDeltaFromRodNutDelta() % 60) + " seconds");
  Serial.println ("NextAngle: " + String (rad2Deg * nextAngle));
  String tmp = (nextAngle >= calculateAngleFromTimeDelta ()) ? "bigger " : "smaller ";
  Serial.println ("NextAngle is " + tmp + "than the calculated angle.");
}



//Used to set in a single call the pins used for micro-stepping
void setMicroSteppingPins (int pin0, int pin1, int pin2)
{
  digitalWrite (ms0Pin, pin0);  
  digitalWrite (ms1Pin, pin1); 
  digitalWrite (ms2Pin, pin2); 
}



//Given an enumeration and its field count, it sets the next value circularly
template <typename T> T nextEnumField (T enumer, int enumValues)
{
  if (enumer < enumValues - 1)
   return (T) (enumer + 1);
  else
    return (T) 0;
}



//Used in the loop function to update constantly the timeDelta that the tracker has to chase in order to stay synchronized with the earth rotation
void updateTimeDelta ()
{
  if (motionState == DISABLED)
   return;
  
  unsigned long tmp = (micros () >= lastLoopTime) ? micros () - lastLoopTime : ((unsigned long) 4294967295) - lastLoopTime + micros ();
  
  if (motionState == UP)
    timeDelta += ((float) tmp) / 1000000.0f;
    
  else if (motionState == DOWN)
    timeDelta -= ((float) tmp) / 1000000.0f;

    lastLoopTime = micros ();
}



//Steps the motor in the desired direction, if enabled
void step ()
{
  if (motionState == DISABLED)
    return;
    
  else
    rodNutDelta += actualRodNutDeltaPerStep;
  
  digitalWrite(stepPin, HIGH);
  delayMicroseconds (4);
  digitalWrite(stepPin, LOW);
}



//Resets values to the base ones valid when the tracker is fully closed
void recalibrate ()
{
  rodNutDelta = initialRodNutDelta;   
  timeDelta = calculateTimeDeltaFromRodNutDelta ();
  Serial.println ("Calibration performed, values has been set at the starting position: restart Arduino!");
  dataInEEPROM (SAVE);
}



//Loads or saves data from/in EEPROM
void dataInEEPROM (EEPROMaction save)
{
  int address = 0;

  switch (save)
  {
  case SAVE:
    EEPROM.updateFloat (address, rodNutDelta);                            //Saves the rodNutDelta variable
    address += sizeof (float);
    EEPROM.updateFloat (address, calculateTimeDeltaFromRodNutDelta ());   //Saves the timeDelta represented by the tracker when switched off
    address += sizeof (float);
    EEPROM.updateInt (address, (uint16_t) 0);                             //Saves the coherence of the environment
    address += sizeof (uint16_t);
    if (setupCompleted)
      Serial.println ("DATA SAVED: YOU CAN NOW SHUT DOWN THE TRACKER OR RESUME");
    break;

  case LOAD:
    rodNutDelta = EEPROM.readFloat (address);                             //Loads the rodNutDelta variable
    address += sizeof (float);  
    timeDelta = EEPROM.readFloat (address);                               //Loads the timeDelta variable
    address += sizeof (float); 
    if (EEPROM.readInt (address) == (uint16_t) 1)                         //Checks if the environment was coherent last time it was used, otherwise a recalibration is needed
    {
      digitalWrite (redLedPin, HIGH);
      digitalWrite (resetLedPin, HIGH);
      error = true;  
      Serial.println ("\nWARNING: DATA WASN'T SAVED PROPERLY, RECALIBRATION NEEDED\n");
    }
    address += sizeof (uint16_t);
    break;

  case INCOHERENT:                          
    address += 2 * sizeof (float);
    EEPROM.updateInt (address, (uint16_t) 1);                             //Saves the incoherence of the environment
    address += sizeof (uint16_t);
    break;
  }
}



void setup() 
{
  digitalWrite (enablePin, LOW);
  
  Serial.begin (9600);
  Serial.println ("Starting Stepper Program");
  Serial.println ("\nREMEMBER: ALWAYS ENABLE THE POWER SUPPLY BEFORE STARTING, OTHERWISE A BLUE BUTTON RECALIBRATION WILL BE NEEDED");
  Serial.println ("REMEMBER: ALWAYS DISABLE THE MOTOR WITH THE RED BUTTON BEFORE SHUTTING DOWN ANYTHING ELSE, THIS WILL SAVE DATA\n");

  dataInEEPROM (LOAD);
  
  if (rodNutDelta == NAN || timeDelta == NAN)
  {
    digitalWrite (redLedPin, HIGH);
    digitalWrite (resetLedPin, HIGH);
    error = true;  
    recalibrate ();
    Serial.println ("WARNING: INITIALIZATION NEEDED\nWARNING:AUTO-RECALIBRATION EXECUTED, RESTART ARDUINO");
  }

  if (error)
    return;
  
  Serial.println ("The tracker has a radius of " + String (R) + "cm and a Beta value of " + String (Beta));
  Serial.println ("Resuming with a rod-nut delta of " + String (rodNutDelta) + "cm, an angle of " + String (rad2Deg * calculateAngleFromRodNutDelta ()) + " degrees and a time delta of " + String ((int) timeDelta / 60) + " minutes and " + String ((int) timeDelta % 60) + " seconds");

  setMicroStepFactor (selectedMicroStepping);
  setMotionState (DISABLED);
  setRotationMode (TRACKING);

  lastLoopTime = micros ();
  setupCompleted = true;
}



void loop() 
{
  if (readButton (blueButtonPin))
    blueButton ();
  if (error == true)
    return;
  if (readButton (greenButtonPin))
    greenButton ();
  if (readButton (yellowButtonPin))
    yellowButton ();
  if (readButton (redButtonPin))
    redButton ();
 
  if (motionState == DISABLED)
    return;

  //CONSTANT SPEED MODES
  if (rotationMode != TRACKING) 
  {
    step (); 
    delayMicroseconds ((unsigned long) (delayTime * 1000000.0f));
  }

  //TRACKING MODE
  else                          
  {        
    if (isValidNA == false)
    {
      nextAngle = calculateAngleFromRodNutDelta (actualRodNutDeltaPerStep);
      isValidNA = true; 
    }
    
    updateTimeDelta ();
    
    if ((motionState == UP && nextAngle <= calculateAngleFromTimeDelta ()) || (motionState == DOWN && nextAngle >= calculateAngleFromTimeDelta ()))
    {
      step ();
      nextAngle = calculateAngleFromRodNutDelta (actualRodNutDeltaPerStep);
    }
  }
}
