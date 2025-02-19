#include <Arduino.h>
#include "pins_FYSETC_E4.h"
#include "FastAccelStepper.h"
#include "TMCStepper.h"
#include <math.h>

const float turns_per_day = 9000.0;
const float active_interval = 5.0;       //2 minutes on
const float rest_interval = 5.0;         //6 minutes off
float intervals_per_day;
float turns_per_interval;
float turns_per_second;
const float phase = 30.0;
float phaseDelay;


TMC2209Stepper driver_x = TMC2209Stepper(&Serial1, 0.22, 1);
TMC2209Stepper driver_y = TMC2209Stepper(&Serial1, 0.22, 3);
TMC2209Stepper driver_z = TMC2209Stepper(&Serial1, 0.22, 0);

FastAccelStepperEngine engine = FastAccelStepperEngine();
FastAccelStepper *stepper_x = NULL;
FastAccelStepper *stepper_y = NULL;
FastAccelStepper *stepper_z = NULL;

const uint16_t motor_current = 200;
const uint16_t microsteps = 16;
void configureStepper(FastAccelStepper **s, int step_pin, int dir_pin, int en_pin);
void configureDriver(TMC2209Stepper *d, uint16_t current, uint16_t mstep);

void setup()
{
  Serial.begin(115200);
  Serial1.begin(9600, SERIAL_8N1, HARDWARE_SERIAL1_RX, HARDWARE_SERIAL1_TX);
  delay(4000);
  Serial.printf("%.1f turns per day, %.1f seconds spin, %.1f seconds rest\n", turns_per_day, active_interval, rest_interval);
  intervals_per_day = (24.0 * 60.0 * 60.0) / (active_interval + rest_interval);
  turns_per_interval = turns_per_day / intervals_per_day;
  turns_per_interval = ceil(turns_per_interval);    //round up to whole revolutions so the watches rest facing upright
  turns_per_second = turns_per_interval / active_interval;
  phaseDelay = (phase /360.0) / turns_per_second;
  Serial.printf("%.1f intervals per day, %.2f turns per interval, rounding up. equals %.3f turns per second.\n", intervals_per_day, turns_per_interval, turns_per_second);
  configureDriver(&driver_x, motor_current, microsteps); // Initiate pins and registeries
  configureDriver(&driver_y, motor_current, microsteps);
  configureDriver(&driver_z, motor_current, microsteps);
  engine.init();
  configureStepper(&stepper_x, X_STEP_PIN, X_DIR_PIN, X_ENABLE_PIN);
  configureStepper(&stepper_y, Y_STEP_PIN, Y_DIR_PIN, Y_ENABLE_PIN);
  configureStepper(&stepper_z, Z_STEP_PIN, Z_DIR_PIN, Z_ENABLE_PIN);
  
}

void loop()
{
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
}

void configureStepper(FastAccelStepper **s, int step_pin, int dir_pin, int en_pin)
{
  *s = engine.stepperConnectToPin(step_pin);
  if (*s)
  {
    FastAccelStepper *sptr = *s;
    sptr->setDirectionPin(dir_pin);
    sptr->setEnablePin(en_pin);
    sptr->setAutoEnable(false);
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