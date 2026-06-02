#include <Arduino.h>
#include <AccelStepper.h>

#define DIR_PIN 8
#define STEP_PIN 7
#define SENSE_PIN 2

#define LIMIT_ON 0

#define DIR_UP -1
#define DIR_DOWN 1

const int MICROSTEP_DIVIDER = 8;
const int STEPS_PER_REV_MOTOR = 200; // 1.8 degrees per full step
const int STEPS_PER_REV_ACTUAL = STEPS_PER_REV_MOTOR * MICROSTEP_DIVIDER;
const int MM_PER_REV = 2;
const int STEPS_PER_MM = STEPS_PER_REV_ACTUAL / MM_PER_REV;

const int MAX_RPM = 120; // Corresponds to 16 mm/s
const int MAX_SPEED = (MAX_RPM / 60.0) * STEPS_PER_REV_ACTUAL;
const int MAX_ACCL = MAX_SPEED;

AccelStepper stepper(AccelStepper::DRIVER, STEP_PIN, DIR_PIN);

const int MOVE_STEP = 0.1;

// Zeros the position of the motor
void zeroPosition();

// Checks if the limit has been triggered
bool limitTriggered();

void setup() {
  Serial.begin(115200);
  Serial.setTimeout(1000);

  pinMode(SENSE_PIN, INPUT);

  stepper.setMaxSpeed(MAX_SPEED);
  stepper.setAcceleration(MAX_ACCL);

  // Check if values divide nicely (errors is movement may occur otherwise)
  if (STEPS_PER_REV_ACTUAL % MM_PER_REV != 0) Serial.println("WARNING - STEPS_PER_MM is only approximate due to integer division!");
  if (STEPS_PER_MM % 100 != 0) Serial.println("WARNING - STEPS_PER_MM is not divisible by 100 which may lead to errors in zeroing the scanner head!");

  //Serial.println("DEBUG - Zeroing scanner assembly");
  //zeroPosition();
  //Serial.println("DEBUG - Zeroing Complete");
}

void loop() {

  // Read line from serial when availible
  if (Serial.available() > 0) {
    String inputStream = Serial.readStringUntil('\n');
    inputStream.trim(); // Removes leading and trailing spaces

    // Convert input stream to char array for strtok
    char inputBuffer[50];
    inputStream.toCharArray(inputBuffer, sizeof(inputBuffer));

    char* cmd = strtok(inputBuffer, " ");
    char* arg = strtok(NULL, " ");

    // Parse Command
    if (strcmp(cmd, "ZERO") == 0) { // Zero laser position

      zeroPosition();
      Serial.println("ZERO_SUCCESS");

    }else if (strcmp(cmd, "MOVE") == 0) { // Move laser position

      if (arg != NULL) {

        bool success = true;
        float arg_val = atof(arg);
        stepper.move(DIR_UP * STEPS_PER_MM * arg_val);
        
        // Run motor until position checking for limit switch
        while (stepper.currentPosition() != stepper.targetPosition()) {
          // Run one step
          stepper.run();
          // Check if limit switch triggered
          if (limitTriggered()) {
            Serial.println("MOVE_FAIL - Limit Triggered");
            success = false;
            break;
          }
        }
        // Print return msg
        if (success) Serial.println("MOVE_SUCCESS");

      }else {
        Serial.println("MOVE_FAIL - No argument passed");
      }

    }else {
      Serial.println("ERROR - Not a valid command");
    }
  }
}

void zeroPosition() {
  
  /*
   * Phase 1: Move DOWN until limit triggered
  */

  // Move to an arbitrary large number of steps DOWN
  stepper.moveTo(DIR_DOWN * 100000);
  while (!limitTriggered()) {
    stepper.run();
  }
  stepper.stop(); // Stop the stepper motor
  stepper.moveTo(stepper.currentPosition()); // Set target to current position

  /*
   * Phase 2: Raise UP until limit not triggered one mm at a time
  */

  while (limitTriggered()) {
    stepper.move(DIR_UP * STEPS_PER_MM * 1); //move 1 mm at a time
    stepper.runToPosition();
  }

  /*
   * Phase 3: Move DOWN until limit triggered one tenth of a mm at a time
  */

  while (!limitTriggered()) {
    stepper.move(DIR_DOWN * STEPS_PER_MM * 0.1); //move 0.1 mm at a time
    stepper.runToPosition();
  }

  /*
   * Phase 4: Move up to be right above limit
  */
  while (limitTriggered()) {
    stepper.move(DIR_UP * STEPS_PER_MM * 0.01);
    stepper.runToPosition();
  }

  /*
   * Set zero position
  */
  stepper.setCurrentPosition(0);

}

bool limitTriggered() {
  return digitalRead(SENSE_PIN) == LIMIT_ON;
}
