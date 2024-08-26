#include "utils.h"
#include <AccelStepper.h>

AccelStepper stepper1(AccelStepper::DRIVER, STEP_PIN1, DIR_PIN1);
AccelStepper stepper2(AccelStepper::DRIVER, STEP_PIN2, DIR_PIN2);
CalibrationResults x_range;
CalibrationResults y_range;

void setup()
{
  Serial.begin(2000000);
  setupInterrupts();

  stepper1.setAcceleration(fast_accel);
  stepper2.setAcceleration(fast_accel);

  Serial.println("Press 'C' to start the calibration process.");

}

void loop()
{
  switch (current_state) {
    case IDLE: {
        if (Serial.available() > 0) {
          String command = Serial.readStringUntil('\n');
          command.trim();

          if (command.equalsIgnoreCase("C")) {
            resetLimitFlags();
            current_state = CALIBRATING;
          }
          
        //   if (command.equalsIgnoreCase("P")) {
        //     Serial.println("X steps: " + String(stepper1.currentPosition()) + " Y steps: " + String(stepper2.currentPosition()));
          
        // }

        }
        break;
      }
    
    case CALIBRATING: {
      Serial.println("Calibrating...");

      calibrationSingleAxis(stepper1, LIMIT_PIN_XA, LIMIT_PIN_XB);
      calibrationSingleAxis(stepper2, LIMIT_PIN_YA, LIMIT_PIN_YB);

      stepper1.setMaxSpeed(fast_speed);
      stepper2.setMaxSpeed(fast_speed);

      stepper1.moveTo(X_CENTER);
      stepper2.moveTo(Y_CENTER);

        while (stepper1.distanceToGo() != 0 || stepper2.distanceToGo() != 0) {
          stepper1.run();
          stepper2.run();
          Serial.println("Moving to center...");
        }
    
      current_state = TRACKING;
      Serial.println("Entering tracking mode.");
      break;
    }

    case TRACKING: {
        while (current_state == TRACKING) {
          
          stepper1.run();
          stepper2.run();

          handleLimitSwitches(stepper1, stepper2);

          if (Serial.available() > 0) {
            String command = Serial.readStringUntil('\n');
            command.trim();
            auto target = parseMsg(command);

            //clip if out of bounds
            target.x = constrain(target.x, LEFT_CAGE_LIMIT, RIGHT_CAGE_LIMIT);

            //y-axis points down
            target.y = constrain(target.y, TOP_CAGE_LIMIT, BOTTOM_CAGE_LIMIT);

            stepper1.moveTo(target.x);
            stepper2.moveTo(target.y);
          }
        }
        break;
      }

  }
}
