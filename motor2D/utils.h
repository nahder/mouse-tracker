#ifndef UTILS_H
#define UTILS_H

#include <AccelStepper.h>
#include <MultiStepper.h>

// Structs
struct CalibrationResults
{
  int lower_bound;
  int upper_bound;
};

struct Point2D
{
  int x;
  int y;
};

struct LimitSwitch
{
  volatile bool & triggered;
  AccelStepper & stepper;
  int direction;
};

// Pins
const int LIMIT_PIN_XA = 3;
const int LIMIT_PIN_XB = 2;
const int LIMIT_PIN_YA = 18;
const int LIMIT_PIN_YB = 19;

const int STEP_PIN1 = 12;
const int DIR_PIN1 = 11;
const int STEP_PIN2 = 7;
const int DIR_PIN2 = 6;

// Cage limits

// left switch zero'd, positive is right
const int LEFT_CAGE_LIMIT = 350;
const int RIGHT_CAGE_LIMIT = 4125; //4150 

// top switch zero'd, positive is down
const int TOP_CAGE_LIMIT = 180;
const int BOTTOM_CAGE_LIMIT = 3950;

const int X_CENTER = (LEFT_CAGE_LIMIT + RIGHT_CAGE_LIMIT) / 2;
const int Y_CENTER = (TOP_CAGE_LIMIT + BOTTOM_CAGE_LIMIT) / 2;

// Debounce times
extern volatile unsigned long last_debounce_time_xa;
extern volatile unsigned long last_debounce_time_xb;
extern volatile unsigned long last_debounce_time_ya;
extern volatile unsigned long last_debounce_time_yb;

// Interrupt flags
extern volatile bool limit_triggered_xa;
extern volatile bool limit_triggered_xb;
extern volatile bool limit_triggered_ya;
extern volatile bool limit_triggered_yb;

// Constants
const unsigned long debounce_delay = 700;
const float slow_speed = 300;

const float fast_speed = 7000; // 5000
const long safety_offset = 600;
const float fast_accel = 80000; // 50000

enum State
{
  IDLE = 0,
  CALIBRATING = 1,
  CAGE_LIMIT_FINDING = 2,
  TRACKING = 3,
};

extern volatile State current_state;

void setupInterrupts();
void limitSwitchXA();
void limitSwitchXB();
void limitSwitchYA();
void limitSwitchYB();

void handleLimitSwitches(AccelStepper & stepper1, AccelStepper & stepper2);

void stepBack(AccelStepper & stepper, int direction);

void resetLimitFlags();

Point2D parseMsg(String command);

long * mapSteps(
  long x_desired,
  long y_desired,
  CalibrationResults x_range,
  CalibrationResults y_range);

void calibrationSingleAxis(
  AccelStepper & stepper,
  int limit_pin_a,
  int limit_pin_b);

// CalibrationResults findCageLimits(
//   AccelStepper & stepper,
//   int limit_pin_a,
//   int limit_pin_b);

void moveStepperSteps(AccelStepper & stepper, long steps, float max_speed);

#endif // UTILS_H
