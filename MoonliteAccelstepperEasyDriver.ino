// Moonlite-compatible stepper controller
//
// Uses AccelStepper (http://www.airspayce.com/mikem/arduino/AccelStepper/)
// Uses AF and the Adafruit v1.2  Shield https://learn.adafruit.com/adafruit--shield
//
// Requires a 10uf - 100uf capacitor between RESET and GND on the  shield; this prevents the
// Arduino from resetting on connect (via DTR going low).  Without the capacitor, this sketch works
// with the stand-alone Moonlite control program (non-ASCOM) but the ASCOM driver does not detect it.
// Adding the capacitor allows the Arduino to respond quickly enough to the ASCOM driver probe
//
// orly.andico@gmail.com, 13 April 2014


#include <AccelStepper.h>
//#include <Stepper.h>
#include <OneWire.h>
#include <DallasTemperature.h>

// maximum speed is 160pps which should be OK for most
// tin can steppers
#define MAXSPEED            2000
#define ACCELERATION        300
#define SPEEDMULT           75
#define STEP_PER_REV        800         //  has 64 steps and has a reduction ratio of 1/:gp64. giving 64 x 64 = 4096 steps!
#define STEP_SIZE           8           // "Macro" steps 2048 / 4 = 512 steps per revolution
#define TEMP_READ_INTERVAL  5000

int in1Pin = 2; 
int in2Pin = 3;

 
AccelStepper stepper(AccelStepper::DRIVER, in1Pin, in2Pin);

// Data wire is plugged into pin 7 on the Arduino
#define ONE_WIRE_BUS 7

// Setup a oneWire instance to communicate with any OneWire devices
OneWire oneWire(ONE_WIRE_BUS);

// Pass our oneWire reference to Dallas Temperature. 
DallasTemperature tempSensor(&oneWire);

// Define a default thermometer device, will be overwritten upon 1-wire device discovery
DeviceAddress thermometer = { 0x28, 0xFF, 0x01, 0xBE, 0x71, 0x15, 0x02, 0xEA };

#define MAXCOMMAND 8
char line[MAXCOMMAND];
int eoc = 0;
int idx = 0;
int isRunning = 0;
int speed = 32;
long millisLastMove = 0;
long millisLastReading = 0;
int temperature = 20;
int tempCompEnabled = 0;

void setup()
{  
  Serial.begin(9600);

  // we ignore the Moonlite speed setting because Accelstepper implements
  // ramping, making variable speeds un-necessary
  //stepper.setSpeed(200);
  stepper.setMaxSpeed(speed * SPEEDMULT);
  stepper.setAcceleration(ACCELERATION);
  stepper.enableOutputs();
  memset(line, 0, MAXCOMMAND);
  millisLastMove = millis();

  // Discover connected temp sensor
  while(oneWire.search(thermometer)); 
  oneWire.reset_search();
  
  // Start up the 1-wire library
  tempSensor.begin();
  
  // set the resolution to 9 bit (0.5C precision)
  tempSensor.setResolution(thermometer, 9);
  millisLastReading = 0;
}


void loop(){
  // run the stepper if there's no pending command and if there are pending movements
  if (!Serial.available())
  {
    motorControl();
  } 
  else 
  {
    readCommandInput();

    readTempSensor();

    if (tempCompEnabled)
    {
      processTempCompensation();
    }
  }

  processCommand();
}


void motorControl(void)
{
  if (isRunning) 
  {
    stepper.run();
    millisLastMove = millis();
  } 
  else 
  {
    // reported on INDI forum that some steppers "stutter" if disableOutputs is done repeatedly
    // over a short interval; hence we only disable the outputs and release the motor some seconds
    // after movement has stopped
    if ((millis() - millisLastMove) > 15000) 
    {
      stepper.disableOutputs();
      //motor.release();
    }
  }

  if (stepper.distanceToGo() == 0) 
  {
    stepper.run();
    isRunning = 0;
  }
}


void readTempSensor(void)
{
  float tempC;
  
  if ((millis() - millisLastReading) >= TEMP_READ_INTERVAL)
  {
    millisLastReading = millis();

    tempSensor.requestTemperatures();
    tempC = tempSensor.getTempC(thermometer);

    // Convert the temp to two's complement 16 bit value with 0.5 granularity
    temperature = (int) (tempC / 0.5);
  }
}


void processTempCompensation(void)
{
  // STUB
}


void readCommandInput(void)
{
  char inChar;
  
  // read the command until the terminating # character
  while (Serial.available() && !eoc) 
  {
    inChar = Serial.read();
    if (inChar != '#' && inChar != ':') {
      line[idx++] = inChar;
      if (idx >= MAXCOMMAND) {
        idx = MAXCOMMAND - 1;
      }
    } 
    else 
    {
      if (inChar == '#') 
      {
        eoc = 1;
      }
    }
  }
}


void processCommand(void)
{
  char cmd[MAXCOMMAND];
  char param[MAXCOMMAND];
  long pos;
  
  // process the command we got
  if (eoc) {
    memset(cmd, 0, MAXCOMMAND);
    memset(param, 0, MAXCOMMAND);

    int len = strlen(line);
    
    if (len >= 2) 
    {
      strncpy(cmd, line, 2);
    }

    if (len > 2) 
    {
      strncpy(param, line + 2, len - 2);
    }

    memset(line, 0, MAXCOMMAND);
    eoc = 0;
    idx = 0;

    // the stand-alone program sends :C# :GB# on startup
    // :C# is a temperature conversion, doesn't require any response

    // LED backlight value, always return "00"
    if (!strcasecmp(cmd, "GB")) 
    {
      Serial.print("00#");
    }

    // home the motor, hard-coded, ignore parameters since we only have one motor
    if (!strcasecmp(cmd, "PH")) 
    { 
      stepper.setCurrentPosition(8000 * STEP_SIZE);
      stepper.moveTo(0);
      isRunning = 1;
    }

    // firmware value, always return "10"
    if (!strcasecmp(cmd, "GV")) 
    {
      Serial.print("10#");
    }

    // get the current motor position
    if (!strcasecmp(cmd, "GP")) 
    {
      pos = stepper.currentPosition() / STEP_SIZE * -1;
      char tempString[6];
      sprintf(tempString, "%04X", pos);
      Serial.print(tempString);
      Serial.print("#");
    }

    // get the new motor position (target)
    if (!strcasecmp(cmd, "GN")) 
    {
      pos = stepper.targetPosition() / STEP_SIZE * -1;
      char tempString[6];
      sprintf(tempString, "%04X", pos);
      Serial.print(tempString);
      Serial.print("#");
    }

    // get the current temperature, hard-coded
    if (!strcasecmp(cmd, "GT")) 
    {
      char tempString[6];
      sprintf(tempString, "%04X", temperature);
      Serial.print(tempString);
      Serial.print("#");
    }

    // get the temperature coefficient, hard-coded
    if (!strcasecmp(cmd, "GC")) 
    {
      Serial.print("02#");
    }

    // get the current motor speed, only values of 02, 04, 08, 10, 20
    if (!strcasecmp(cmd, "GD")) 
    {
      char tempString[6];
      sprintf(tempString, "%02X", speed);
      Serial.print(tempString);
      Serial.print("#");
    }

    // set speed, only acceptable values are 02, 04, 08, 10, 20
    if (!strcasecmp(cmd, "SD")) 
    {
      speed = hexstr2long(param);

      stepper.setSpeed(speed * SPEEDMULT);
      stepper.setMaxSpeed(speed * SPEEDMULT);
    }

    // whether half-step is enabled or not, always return "00"
    if (!strcasecmp(cmd, "GH")) 
    {
      Serial.print("00#");
    }

    // motor is moving - 01 if moving, 00 otherwise
    if (!strcasecmp(cmd, "GI")) {
      if (abs(stepper.distanceToGo()) > 0) 
      {
        Serial.print("01#");
      } 
      else 
      {
        Serial.print("00#");
      }
    }

    // set current motor position
    if (!strcasecmp(cmd, "SP")) 
    {
      pos = hexstr2long(param);
      stepper.setCurrentPosition(pos * STEP_SIZE * -1);
    }

    // set new motor position
    if (!strcasecmp(cmd, "SN")) 
    {
      pos = hexstr2long(param);
      stepper.moveTo(pos * STEP_SIZE * -1);
    }

    // initiate a move
    if (!strcasecmp(cmd, "FG")) 
    {
      isRunning = 1;
      stepper.enableOutputs();
    }

    // stop a move
    if (!strcasecmp(cmd, "FQ")) 
    {
      isRunning = 0;
      stepper.moveTo(stepper.currentPosition() * -1);  // Multiple by -1. Outward focus is negatibe
      stepper.run();
    }

    // enable temperature compensation
    if (!strcasecmp(cmd, "+")) 
    {
      tempCompEnabled = 1;
    }

    // disable temperature compensation
    if (!strcasecmp(cmd, "-")) 
    {
      tempCompEnabled = 0;
    }
  }
}

long hexstr2long(char *line) 
{
  long ret = 0;

  ret = strtol(line, NULL, 16);
  return (ret);
}
