#include "utils.h"
#include <AccelStepper.h>

volatile unsigned long last_debounce_time_xa = 0;
volatile unsigned long last_debounce_time_xb = 0;
volatile unsigned long last_debounce_time_ya = 0;
volatile unsigned long last_debounce_time_yb = 0;

volatile bool limit_triggered_xa = false;
volatile bool limit_triggered_xb = false;
volatile bool limit_triggered_ya = false;
volatile bool limit_triggered_yb = false;

volatile State current_state = IDLE;

void setupInterrupts()
{
  pinMode(LIMIT_PIN_XA, INPUT_PULLUP);
  pinMode(LIMIT_PIN_XB, INPUT_PULLUP);
  pinMode(LIMIT_PIN_YA, INPUT_PULLUP);
  pinMode(LIMIT_PIN_YB, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(LIMIT_PIN_XA), limitSwitchXA, FALLING);
  attachInterrupt(digitalPinToInterrupt(LIMIT_PIN_XB), limitSwitchXB, FALLING);
  attachInterrupt(digitalPinToInterrupt(LIMIT_PIN_YA), limitSwitchYA, FALLING);
  attachInterrupt(digitalPinToInterrupt(LIMIT_PIN_YB), limitSwitchYB, FALLING);
}


void limitSwitchXA()
{
  unsigned long current_time = millis();
  if (current_time - last_debounce_time_xa > debounce_delay) {
    last_debounce_time_xa = current_time;
    limit_triggered_xa = true;
    Serial.println("Limit switch XA triggered.");
  }
}

void limitSwitchXB()
{
  unsigned long current_time = millis();
  if (current_time - last_debounce_time_xb > debounce_delay) {
    last_debounce_time_xb = current_time;
    limit_triggered_xb = true;
    Serial.println("Limit switch XB triggered.");
  }
}

void limitSwitchYA()
{
  unsigned long current_time = millis();
  if (current_time - last_debounce_time_ya > debounce_delay) {
    last_debounce_time_ya = current_time;
    limit_triggered_ya = true;
    Serial.println("Limit switch YA triggered.");
  }
}

void limitSwitchYB()
{
  unsigned long current_time = millis();
  if (current_time - last_debounce_time_yb > debounce_delay) {
    last_debounce_time_yb = current_time;
    limit_triggered_yb = true;
    Serial.println("Limit switch YB triggered.");
  }
}

void calibrationSingleAxis(AccelStepper & stepper, int limit_pin_a, int limit_pin_b)
{
  int calib_lower_bound = 0; int calib_upper_bound = 0;

  stepper.setMaxSpeed(slow_speed);
  stepper.move(-1000000);
  Serial.println("Moving towards limit A.");

  while (stepper.distanceToGo() != 0) {
    stepper.run();

    if (limit_triggered_xa || limit_triggered_yb) {
      stepper.setCurrentPosition(0); 
      stepBack(stepper, 1);
      limit_triggered_xa = false;
      limit_triggered_yb = false;
      break;
    }
  }
}

void moveStepperSteps(AccelStepper & stepper, long steps, float max_speed = slow_speed) {
  stepper.setMaxSpeed(max_speed);
  stepper.moveTo(steps);
  
  while (stepper.distanceToGo() != 0) {
    stepper.run();
  }
  stepper.setMaxSpeed(slow_speed);
}

// CalibrationResults findCageLimits(
//   AccelStepper & stepper,
//   int limit_pin_a,
//   int limit_pin_b) {

//   bool lower_cage_found = false, upper_cage_found = false;

//   while (!lower_cage_found && !upper_cage_found) {
//       if (limit_triggered_xa || limit_triggered_yb) {
//         stepper.setCurrentPosition(0); // zero out the position
//         stepBack(stepper, 1);
//         limit_triggered_xa = false;
//         limit_triggered_yb = false;
//       }

//     if (Serial.available() > 0) {
//       String command = Serial.readStringUntil('\n');
//       command.trim();
      
//       if (command.equalsIgnoreCase("A") || command.equalsIgnoreCase("W")) {
//         moveStepperSteps(stepper, -25); // Move left

//       } else if (command.equalsIgnoreCase("D") || command.equalsIgnoreCase("S")) {
//         moveStepperSteps(stepper, 25); // Move upper

//       } else if (command.equalsIgnoreCase("L")) {
//         lower_cage_found = true;
//         Serial.println("Lower cage found. # Steps: ");
//         Serial.println(stepper.currentPosition());

//       } else if (command.equalsIgnoreCase("U")) {
//         upper_cage_found = true;
//         Serial.println("Upper cage found. # Steps: ");
//         Serial.println(stepper.currentPosition());
//       }
//   }
//   }
//   }

void stepBack(AccelStepper & stepper, int direction)
{
  stepper.setMaxSpeed(fast_speed);
  stepper.move(direction * 500);
  while (stepper.distanceToGo() != 0) {
    stepper.run();
  }
  stepper.setMaxSpeed(slow_speed);
}

void resetLimitFlags()
{
  limit_triggered_xa = false;
  limit_triggered_xb = false;
  limit_triggered_ya = false;
  limit_triggered_yb = false;
}

Point2D parseMsg(String command)
{
  int spaceIndex = command.indexOf(' ');
  if (spaceIndex == -1) {
    Serial.println("Invalid input. Enter two numbers separated by a space.");
    return; 
  }
  auto xSteps = command.substring(0, spaceIndex).toInt();
  auto ySteps = command.substring(spaceIndex + 1).toInt();

  return {xSteps, ySteps};
}

long * mapSteps(
  long x_desired,
  long y_desired,
  CalibrationResults x_range,
  CalibrationResults y_range)
{
  long xSteps = map(x_desired, -50, 50, x_range.lower_bound, x_range.upper_bound);
  long ySteps = map(y_desired, -50, 50, y_range.lower_bound, y_range.upper_bound);

  xSteps = constrain(xSteps, x_range.lower_bound, x_range.upper_bound);
  ySteps = constrain(ySteps, y_range.lower_bound, y_range.upper_bound);

  static long mapped_steps[2];
  mapped_steps[0] = xSteps;
  mapped_steps[1] = ySteps;

  return mapped_steps;
}

void handleLimitSwitches(AccelStepper & stepper1, AccelStepper & stepper2)
{
    LimitSwitch limitSwitches[] = {
      {limit_triggered_xa, stepper1, 1},
      {limit_triggered_xb, stepper1, -1},
      {limit_triggered_ya, stepper2, -1},
      {limit_triggered_yb, stepper2, 1},
    };

  for (auto & ls : limitSwitches) {
    if (ls.triggered) {
      Serial.println("Limit switch triggered, stepping back & switching to calibration");
      stepBack(ls.stepper, ls.direction);
      resetLimitFlags();
      ls.triggered = false;
      current_state = CALIBRATING;

    }
  }

}
