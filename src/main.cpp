#include <Arduino.h>
#include "pins_FYSETC_E4.h"
#include "FastAccelStepper.h"
#include "TMCStepper.h"
#include <math.h>

const float turns_per_day = 1500.0;
const float active_interval = 120.0; // 2 minutes on
const float rest_interval = 360.0;   // 6 minutes off
float intervals_per_day;
float turns_per_interval;
float turns_per_second;
const float phaseAngle = 30.0;
float phaseDelay;
int runSpeed;
int destination;
const int homingSpeed = 2000;

const int doorPin = X_STOP_PIN;

TMC2209Stepper driver_x = TMC2209Stepper(&Serial1, 0.22, 1);
TMC2209Stepper driver_y = TMC2209Stepper(&Serial1, 0.22, 3);
TMC2209Stepper driver_z = TMC2209Stepper(&Serial1, 0.22, 0);

FastAccelStepperEngine engine = FastAccelStepperEngine();
FastAccelStepper *stepper_x = NULL;
FastAccelStepper *stepper_y = NULL;
FastAccelStepper *stepper_z = NULL;

const uint8_t stepper_x_pulsecounter = 1;
const uint8_t stepper_y_pulsecounter = 2;
const uint8_t stepper_z_pulsecounter = 3;

const uint16_t motor_current = 200;
const uint16_t microsteps = 16;
bool forwards = true;

enum WinderState
{
  BOOT = 0,
  IDLE,
  STARTING_1,
  STARTING_2,
  STARTING_3,
  RUNNING,
  HOMING,
  DOOR_OPENED,
  TRAVEL_HOME,
  DOOR_OPEN_PAUSED
};

WinderState winderMachine();
void configureStepper(FastAccelStepper **s, int step_pin, int dir_pin, int en_pin, uint8_t pulsecounter);
void configureDriver(TMC2209Stepper *d, uint16_t current, uint16_t mstep);
void hardwareInit();

void setup()
{
  Serial.begin(115200);
  Serial1.begin(9600, SERIAL_8N1, HARDWARE_SERIAL1_RX, HARDWARE_SERIAL1_TX);
  delay(4000);
  Serial.printf("%.1f turns per day, %.1f seconds spin, %.1f seconds rest\n", turns_per_day, active_interval, rest_interval);
  intervals_per_day = (24.0 * 60.0 * 60.0) / (active_interval + rest_interval);
  turns_per_interval = turns_per_day / intervals_per_day;
  turns_per_interval = ceil(turns_per_interval); // round up to whole revolutions so the watches rest facing upright
  turns_per_second = turns_per_interval / active_interval;
  phaseDelay = (phaseAngle / 360.0) / turns_per_second;
  runSpeed = (int)(turns_per_second * 200.0 * (float)microsteps);
  Serial.printf("%.1f intervals per day, %.2f turns per interval, rounding up. equals %.3f turns per second.\n", intervals_per_day, turns_per_interval, turns_per_second);
  destination = (int16_t)(turns_per_interval * 200.0 * (float)microsteps);
  // hardwareInit();
  pinMode(doorPin, INPUT_PULLUP);
  hardwareInit();
}

void loop()
{
  winderMachine();
  /*
  stepper_x->moveTo(turns_per_interval * 200 * microsteps);
  delay(1000.0 * phaseDelay);
  stepper_y->moveTo(turns_per_interval * 200 * microsteps);
  delay(1000.0 * phaseDelay);
  stepper_z->moveTo(turns_per_interval * 200 * microsteps);
  delay(1000.0 * phaseDelay);
  while (stepper_x->isRunning() || stepper_y->isRunning() || stepper_z->isRunning());
  delay(1000.0 * rest_interval);
  stepper_x->moveTo(0 * 200 * microsteps);
  delay(1000.0 * phaseDelay);
  stepper_y->moveTo(0 * 200 * microsteps);
  delay(1000.0 * phaseDelay);
  stepper_z->moveTo(0 * 200 * microsteps);
  delay(1000.0 * phaseDelay);
  while (stepper_x->isRunning() || stepper_y->isRunning() || stepper_z->isRunning());
  delay(1000.0 * rest_interval);
  */
}

void configureStepper(FastAccelStepper **s, int step_pin, int dir_pin, int en_pin, uint8_t pulsecounter)
{
  *s = engine.stepperConnectToPin(step_pin);
  if (*s)
  {
    FastAccelStepper *sptr = *s;
    sptr->setDirectionPin(dir_pin);
    sptr->setEnablePin(en_pin);
    sptr->setAutoEnable(false);
    sptr->attachToPulseCounter(pulsecounter);
    sptr->clearPulseCounter();
    sptr->setSpeedInHz((uint32_t)(turns_per_second * 200.0 * (float)microsteps));
    sptr->setAcceleration(4000);
    sptr->enableOutputs();
    // sptr->move(1000);
  }
}

void configureDriver(TMC2209Stepper *d, uint16_t current, uint16_t mstep)
{
  d->begin(); // Initiate pins and registeries
  d->defaults();
  d->rms_current(current); // Set stepper current to 600mA. The command is the same as command TMC2130.setCurrent(600, 0.11, 0.5)
  d->microsteps(mstep);
  d->intpol(true);
}

WinderState winderMachine()
{
  static WinderState presentState = BOOT, nextState = BOOT;
  static bool stateEntry = true;
  static unsigned long stateEntryTime = 0;
  
  if (!digitalRead(doorPin))
  {
    if ((presentState != DOOR_OPENED) && (presentState != TRAVEL_HOME) && (presentState != DOOR_OPEN_PAUSED))
    {
      nextState = DOOR_OPENED;
    }
  }
  
  if (presentState != nextState)
  {
    stateEntryTime = millis();
    stateEntry = true;
  }
  presentState = nextState;

  switch (presentState)
  {
  case BOOT:
  {
    if (stateEntry)
      Serial.printf("Entered Boot State, %d\n", millis());
    nextState = STARTING_1;
  }
  break;
  case IDLE:
  {
    if (stateEntry)
      Serial.printf("Entered Idle State, %d\n", millis());
    // move back to starting state after the idle interval
    if (millis() - stateEntryTime > (unsigned long)(rest_interval * 1000.0))
    {
      nextState = STARTING_1;
    }
  }
  break;
  case STARTING_1:
  {
    if (stateEntry)
    {
      Serial.printf("Entered STARTING_1 State, %d\n", millis());
      // set up speeds and begin motor 1
      // stepper_x->setSpeedInHz(runSpeed);
      if (forwards)
        stepper_x->moveTo(destination);
      else
        stepper_x->moveTo(0);
    }
    if (millis() - stateEntryTime > (uint32_t)(phaseDelay * 1000.0))
      nextState = STARTING_2;
    // wait for phase delay time and move on to starting_2
  }
  break;
  case STARTING_2:
  {
    if (stateEntry)
    {
      Serial.printf("Entered STARTING_2 State, %d\n", millis());
      // set up speeds and begin motor 1
      // stepper_y->setSpeedInHz(runSpeed);
      if (forwards)
        stepper_y->moveTo(destination);
      else
        stepper_y->moveTo(0);
    }
    if (millis() - stateEntryTime > (uint32_t)(phaseDelay * 1000.0))
      nextState = STARTING_3;
  }
  break;
  case STARTING_3:
  {
    if (stateEntry)
    {
      Serial.printf("Entered STARTING_3 State, %d\n", millis());
      // set up speeds and begin motor 1
      // stepper_z->setSpeedInHz(runSpeed);
      if (forwards)
        stepper_z->moveTo(destination);
      else
        stepper_z->moveTo(0);
    }
    if (millis() - stateEntryTime > (uint32_t)(phaseDelay * 1000.0))
      nextState = RUNNING;
  }
  break;
  case RUNNING:
  {
    if (stateEntry)
      Serial.printf("Entered RUNNING State, %d\n", millis());
    if (!(stepper_x->isRunning() || stepper_y->isRunning() || stepper_z->isRunning()))
    {
      forwards = !forwards;
      nextState = IDLE;
    }
    // check to see if motors have all completed the move, go to idle if so
  }
  break;
  case HOMING:
  {
    if (stateEntry)
      Serial.printf("Entered HOMING State, %d\n", millis());
    nextState = STARTING_1;
    // probably a whole secondary state machine runs here
  }
  break;
  case DOOR_OPENED:
  {
    if (stateEntry)
      Serial.printf("Entered DOOR_OPENED State, %d\n", millis());
    // if not moving, skip to door_open_paused
    if (!(stepper_x->isRunning() || stepper_y->isRunning() || stepper_z->isRunning()))
      nextState = DOOR_OPEN_PAUSED;
    else
    {
      // if moving, speed the watches up to quickly arrive in upright positio
      stepper_x->setSpeedInHz(homingSpeed);
      stepper_y->setSpeedInHz(homingSpeed);
      stepper_z->setSpeedInHz(homingSpeed);
      int16_t position_x = stepper_x->readPulseCounter();
      int angle_x = position_x % (200 * microsteps);
      if (forwards)
      {
        angle_x = (200 * microsteps) - angle_x;
        stepper_x->moveTo(position_x + angle_x);
      }
      else
      {
        stepper_x->moveTo(position_x - angle_x);
      }

      int16_t position_y = stepper_y->readPulseCounter();
      int angle_y = position_y % (200 * microsteps);
      if (forwards)
      {
        angle_y = (200 * microsteps) - angle_y;
        stepper_y->moveTo(position_y + angle_y);
      }
      else
      {
        stepper_y->moveTo(position_y - angle_y);
      }

      int16_t position_z = stepper_z->readPulseCounter();
      int angle_z = position_z % (200 * microsteps);
      if (forwards)
      {
        angle_z = (200 * microsteps) - angle_z;
        stepper_z->moveTo(position_z + angle_z);
      }
      else
      {
        stepper_z->moveTo(position_z - angle_z);
      }
    }
    nextState = TRAVEL_HOME;

    // calculate the nearest full revolution (upright position) in the direction presently spinning
    // begin the move, and go to travel_home
  }
  break;
  case TRAVEL_HOME:
  {
    if (stateEntry)
      Serial.printf("Entered TRAVEL_HOME State, %d\n", millis());
    // check to see if motors have all stopped and move on to door_open_paused
    if (!(stepper_x->isRunning() || stepper_y->isRunning() || stepper_z->isRunning()))
      nextState = DOOR_OPEN_PAUSED;
  }
  break;
  case DOOR_OPEN_PAUSED:
  {
    if (stateEntry)
    {
      Serial.printf("Entered DOOR_OPEN_PAUSED State, %d\n", millis());
      stepper_x->disableOutputs();
      stepper_y->disableOutputs();
      stepper_z->disableOutputs();
    }
    // check to see if door has been closed, reenable motors, go to homing
    if (digitalRead(doorPin))
    {
      stepper_x->enableOutputs();
      stepper_y->enableOutputs();
      stepper_z->enableOutputs();
      stepper_x->setCurrentPosition(0);
      stepper_x->clearPulseCounter();
      stepper_x->setSpeedInHz(runSpeed);
      stepper_y->setCurrentPosition(0);
      stepper_y->clearPulseCounter();
      stepper_y->setSpeedInHz(runSpeed);
      stepper_z->setCurrentPosition(0);
      stepper_z->clearPulseCounter();
      stepper_z->setSpeedInHz(runSpeed);
      forwards = true;
      nextState = HOMING;
    }
  }
  break;
  }
  stateEntry = false;
  return nextState;
}

void hardwareInit()
{
  configureDriver(&driver_x, motor_current, microsteps); // Initiate pins and registeries
  configureDriver(&driver_y, motor_current, microsteps);
  configureDriver(&driver_z, motor_current, microsteps);
  engine.init();
  configureStepper(&stepper_x, X_STEP_PIN, X_DIR_PIN, X_ENABLE_PIN, stepper_x_pulsecounter);
  configureStepper(&stepper_y, Y_STEP_PIN, Y_DIR_PIN, Y_ENABLE_PIN, stepper_y_pulsecounter);
  configureStepper(&stepper_z, Z_STEP_PIN, Z_DIR_PIN, Z_ENABLE_PIN, stepper_z_pulsecounter);
}