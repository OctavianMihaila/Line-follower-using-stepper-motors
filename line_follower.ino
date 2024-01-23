#include <AccelStepper.h>

// Motor 1 pins.
#define DIR_PIN_1 4
#define STEP_PIN_1 5
#define MS1_PIN_1 10
#define MS2_PIN_1 9
#define MS3_PIN_1 8

// Motor 2 pins.
#define DIR_PIN_2 6
#define STEP_PIN_2 7
#define MS1_PIN_2 13
#define MS2_PIN_2 12
#define MS3_PIN_2 11

// IR pins.
#define IR1SENSOR_Pin 2
#define IR2SENSOR_Pin 3
#define IR_OBSTACLE_Pin A4

// Tilt sensor pin.
#define TILTSENSOR_PIN A3

// Power supply pins.
#define START_STOP_BUTTON_PIN A1
#define RELAY_PIN A2
#define DEBUG_FORWARD_PIN A0
#define DEBUG_TURN_PIN A4
#define DEBUG_TILT_SENSOR_PIN A5

#define MOVING_FORWARD 1
#define MOVING_BACKWARD -1
#define TURN_LEFT 2
#define TURN_RIGHT -2
#define ROTATE_LEFT 3
#define ROTATE_RIGHT -3
#define STEPS 1000000
#define REQUIRED_TILTED_DURATION 1000

long ONE_ROTATION_VAL = 3200;
int state = 0; // zero = does nothing.
int previous_state = 0;
int previous_tilt_sensor_value = LOW;
int relayState = LOW; // tracks the current state of the power supply.
int lastStartStopButtonState;
int currentStartStopButtonState;
int tiltStartTime = 0;
int tiltDuration = 0;
bool stopMoving = false;
bool hasMoved = false;
bool oneRotationScheduled = false;
bool isOnRamp = false;
bool is_doing_avoid_maneuver = false;
bool hasRotatedLeft = false;
bool isSpeedIncreased = false;
bool wasOnRamp = false;

AccelStepper stepper1(AccelStepper::DRIVER, STEP_PIN_1, DIR_PIN_1);
AccelStepper stepper2(AccelStepper::DRIVER, STEP_PIN_2, DIR_PIN_2);

void setup() {
  Serial.begin(9600);
  // Set the microstepping mode for Motor 1.
  pinMode(MS1_PIN_1, OUTPUT);
  pinMode(MS2_PIN_1, OUTPUT);
  pinMode(MS3_PIN_1, OUTPUT);
  digitalWrite(MS1_PIN_1, HIGH);
  digitalWrite(MS2_PIN_1, LOW);
  digitalWrite(MS3_PIN_1, HIGH);

  // Set the microstepping mode for Motor 2.
  pinMode(MS1_PIN_2, OUTPUT);
  pinMode(MS2_PIN_2, OUTPUT);
  pinMode(MS3_PIN_2, OUTPUT);
  digitalWrite(MS1_PIN_2, HIGH);
  digitalWrite(MS2_PIN_2, LOW);
  digitalWrite(MS3_PIN_2, HIGH);

  // Set IR pins
  pinMode(IR1SENSOR_Pin, INPUT);
  pinMode(IR2SENSOR_Pin, INPUT);
  pinMode(IR_OBSTACLE_Pin, INPUT);

  // Set the TILT pin.
  pinMode(TILTSENSOR_PIN, INPUT);

  // Set motor power control pins.
  pinMode(START_STOP_BUTTON_PIN, INPUT);
  pinMode(RELAY_PIN, OUTPUT);
  currentStartStopButtonState = digitalRead(START_STOP_BUTTON_PIN);

  // Set debug pins.
  pinMode(DEBUG_FORWARD_PIN, OUTPUT);
  digitalWrite(DEBUG_FORWARD_PIN, LOW);

  pinMode(DEBUG_TURN_PIN, OUTPUT);
  digitalWrite(DEBUG_TURN_PIN, LOW);

  pinMode(DEBUG_TILT_SENSOR_PIN, OUTPUT);
  digitalWrite(DEBUG_TILT_SENSOR_PIN, LOW);

  // Set initial speed and acceleration for Motor 1.
  stepper1.setMaxSpeed(3000.0);
  stepper1.setAcceleration(1500.0);

  // Set initial speed and acceleration for Motor 2.
  stepper2.setMaxSpeed(3000.0);
  stepper2.setAcceleration(1500.0);

  state = MOVING_FORWARD; // Initial state.
}

void loop() {
  int ir1SensorValue = digitalRead(IR1SENSOR_Pin);
  int ir2SensorValue = digitalRead(IR2SENSOR_Pin);
  int tiltSensorValue = digitalRead(TILTSENSOR_PIN);
  int irObstacleSensorValue = digitalRead(IR_OBSTACLE_Pin);

  handle_power_supply_on_off();
  handle_debug_pins();

  state = get_next_movement(ir1SensorValue, ir2SensorValue);
  adjust_speed_when_turning();
  adjust_speed_on_ramps(tiltSensorValue);
  handle_forward_backward_direction_change();

  switch (state) {
    case MOVING_FORWARD:
      move_until_stop(MOVING_FORWARD);
      break;
    case MOVING_BACKWARD:
      move_until_stop(MOVING_BACKWARD);
      break;
    case TURN_LEFT:
      turn_left();
      break;
    case TURN_RIGHT:
      turn_right();
      break;
    default:
      break;
  }
}

void adjust_speed_when_turning() {
  // If turning left, slow down the left motor
  if (previous_state == MOVING_FORWARD && state == TURN_LEFT) {
    stepper1.setSpeed(stepper1.speed() * 0.3);
    stepper1.setMaxSpeed(stepper1.speed() * 0.3);
    stepper1.setAcceleration(0.0);

    stepper2.setMaxSpeed(3000.0);
    stepper2.setAcceleration(1500.0);
    // If turning right, slow down the right motor
  } else if (previous_state == MOVING_FORWARD && state == TURN_RIGHT) {
    stepper2.setSpeed(stepper2.speed() * 0.3);
    stepper2.setMaxSpeed(stepper2.speed() * 0.3);
    stepper2.setAcceleration(0.0);

    stepper1.setMaxSpeed(3000.0);
    stepper1.setAcceleration(1500.0);
    // Restore normal speed and acceleration for forward/backward movements
  } else if ((previous_state == TURN_LEFT || previous_state == TURN_RIGHT)
              && (state == MOVING_FORWARD || state == MOVING_BACKWARD)) {
    stepper1.setMaxSpeed(3000.0);
    stepper1.setAcceleration(1500.0);

    stepper2.setMaxSpeed(3000.0);
    stepper2.setAcceleration(1500.0);
  }

  previous_state = state;
}


void adjust_speed_on_ramps(int currentTiltSensorValue) {
  count_tilted_time(currentTiltSensorValue, previous_tilt_sensor_value);

  previous_tilt_sensor_value = currentTiltSensorValue;
}

void count_tilted_time(int currentTiltSensorValue, int previous_tilt_sensor_value) {
   if (currentTiltSensorValue == previous_tilt_sensor_value) {
    if (tiltStartTime == 0) {
      tiltStartTime = millis();
    } else {
      tiltDuration = millis() - tiltStartTime;
      
      // Used this as workaround for tilt sensor triggers due to vibrations.
      if (tiltDuration >= REQUIRED_TILTED_DURATION) {
        isOnRamp = (currentTiltSensorValue == HIGH) ? true : false;
      }
    }
  } else {
    tiltStartTime = 0;
    tiltDuration = 0;
  }
}

void handle_forward_backward_direction_change() {
  if ((previous_state == MOVING_FORWARD && state == MOVING_BACKWARD) 
      || (previous_state == MOVING_BACKWARD && state == MOVING_FORWARD)) {
    stepper1.stop();
    stepper2.stop();
  }
}

int get_next_movement(int ir1SensorValue, int ir2SensorValue) {
  if (ir1SensorValue == LOW && ir2SensorValue == LOW) {
    return MOVING_FORWARD;
  }

  if (ir1SensorValue == HIGH && ir2SensorValue == LOW) {
    return TURN_LEFT;
  }

  if (ir1SensorValue == LOW && ir2SensorValue == HIGH) {
    return TURN_RIGHT;
  }

  return state;
}

void move_until_stop(int direction) {
    if (stepper1.distanceToGo() == 0 && stepper2.distanceToGo() == 0) {
      switch (direction) {
        case MOVING_FORWARD:
          stepper1.moveTo(stepper1.currentPosition() - STEPS);
          stepper2.moveTo(stepper2.currentPosition() + STEPS);
          break;

        case MOVING_BACKWARD:
          stepper1.moveTo(stepper1.currentPosition() + STEPS);
          stepper2.moveTo(stepper2.currentPosition() - STEPS);
          break;

        default:
          break;
      }
    }

    stepper1.run();
    stepper2.run();
}

void turn_left() {
  stepper1.run();
  stepper2.run();
}

void turn_right() {
  stepper1.run();
  stepper2.run();
}

void handle_power_supply_on_off() {
  lastStartStopButtonState = currentStartStopButtonState;
  currentStartStopButtonState = digitalRead(START_STOP_BUTTON_PIN);
  
  if(lastStartStopButtonState == HIGH && currentStartStopButtonState == LOW) {
    relayState = (relayState == LOW) ? HIGH : LOW;
    digitalWrite(RELAY_PIN, relayState); 
  }
}

void handle_debug_pins() {
  digitalWrite(DEBUG_FORWARD_PIN, LOW);
  digitalWrite(DEBUG_TURN_PIN, LOW);
  digitalWrite(DEBUG_TILT_SENSOR_PIN, LOW);

  if (state == MOVING_FORWARD) {
    digitalWrite(DEBUG_FORWARD_PIN, HIGH);
  }

  if (state == TURN_LEFT || state == TURN_RIGHT) {
    digitalWrite(DEBUG_TURN_PIN, HIGH);
  }

  if (isOnRamp) {
    digitalWrite(DEBUG_TILT_SENSOR_PIN, HIGH);
  }
}