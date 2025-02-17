#include <Arduino.h>
#include "pins_FYSETC_E4.h"
#include "FastAccelStepper.h"
#include "TMCStepper.h"

TMC2209Stepper driver_x = TMC2209Stepper(&Serial1, 0.22, 1); 
TMC2209Stepper driver_y = TMC2209Stepper(&Serial1, 0.22, 3); 
TMC2209Stepper driver_z = TMC2209Stepper(&Serial1, 0.22, 0); 

FastAccelStepperEngine engine = FastAccelStepperEngine();
FastAccelStepper *stepper_x = NULL;
FastAccelStepper *stepper_y = NULL;
FastAccelStepper *stepper_z = NULL;

const uint16_t motor_current = 100;
const uint16_t microsteps = 16;

void setup() {
  Serial.begin(115200);

  Serial1.begin(9600, SERIAL_8N1, HARDWARE_SERIAL1_RX, HARDWARE_SERIAL1_TX);

  driver_x.begin();             // Initiate pins and registeries
  driver_x.defaults();
  driver_x.rms_current(motor_current);    // Set stepper current to 600mA. The command is the same as command TMC2130.setCurrent(600, 0.11, 0.5)
  driver_x.microsteps(microsteps);
  driver_x.intpol(true);
  

  driver_y.begin();             // Initiate pins and registeries
  driver_x.defaults();
  driver_y.rms_current(motor_current);    // Set stepper current to 600mA. The command is the same as command TMC2130.setCurrent(600, 0.11, 0.5)
  driver_y.microsteps(microsteps);
  driver_y.intpol(true);

  driver_z.begin();             // Initiate pins and registeries
  driver_x.defaults();
  driver_z.rms_current(motor_current);    // Set stepper current to 600mA. The command is the same as command TMC2130.setCurrent(600, 0.11, 0.5)
  driver_z.microsteps(microsteps);
  driver_z.intpol(true);

  engine.init();
  stepper_x = engine.stepperConnectToPin(X_STEP_PIN);
  if (stepper_x) {
    stepper_x->setDirectionPin(X_DIR_PIN);
    stepper_x->setEnablePin(X_ENABLE_PIN);
    stepper_x->setAutoEnable(true);
    stepper_x->setSpeedInUs(100);  // the parameter is us/step !!!
    stepper_x->setAcceleration(100);
    //stepper_x->move(1000);
  }
  stepper_y = engine.stepperConnectToPin(Y_STEP_PIN);
  if (stepper_y) {
    stepper_y->setDirectionPin(Y_DIR_PIN);
    stepper_y->setEnablePin(Y_ENABLE_PIN);
    stepper_y->setAutoEnable(true);
    stepper_y->setSpeedInUs(100);  // the parameter is us/step !!!
    stepper_y->setAcceleration(100);
    //stepper_y->move(1000);
  }
  stepper_z = engine.stepperConnectToPin(Z_STEP_PIN);
  if (stepper_z) {
    stepper_z->setDirectionPin(Z_DIR_PIN);
    stepper_z->setEnablePin(Z_ENABLE_PIN);
    stepper_z->setAutoEnable(true);
    stepper_z->setSpeedInUs(100);  // the parameter is us/step !!!
    stepper_z->setAcceleration(100);
    //stepper_z->move(1000);
  }
}

void loop() {
  stepper_x->moveTo(1*200*microsteps);
  delay(1000);
  stepper_y->moveTo(1*200*microsteps);
  delay(1000);
  stepper_z->moveTo(1*200*microsteps);
  delay(1000);
  while(stepper_x->isRunning() || stepper_y->isRunning() || stepper_z->isRunning()) delay(100);
  stepper_x->moveTo(0*200*microsteps);
  delay(1000);
  stepper_y->moveTo(0*200*microsteps);
  delay(1000);
  stepper_z->moveTo(0*200*microsteps);
  delay(1000);
  while(stepper_x->isRunning() || stepper_y->isRunning() || stepper_z->isRunning()) delay(100);
}
