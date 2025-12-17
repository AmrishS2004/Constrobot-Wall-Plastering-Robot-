#include <AccelStepper.h>
#include <Servo.h>

// Pin Definitions
#define STEP_PIN 2
#define DIR_PIN 3
#define ENABLE_PIN 4
#define WHEEL_PWM 6
#define WHEEL_DIR 5
#define SERVO_PIN 9
#define RPWM 10
#define LPWM 11
#define R_EN 12
#define L_EN 13

// Constants
const long MAX_HEIGHT_STEPS = -390000;
const long DELAY_AT_TOP = 5000;
const long DELAY_AT_BOTTOM = 5000;
const int SERVO_BOTTOM_ANGLE = 140;
const int SERVO_TOP_ANGLE = 180;
const int MOTOR_SPEED = 150;
const long TWO_METERS_STEPS = 100000;  // Adjust based on your setup
const long FOUR_METERS_STEPS = 200000;

AccelStepper stepper(AccelStepper::DRIVER, STEP_PIN, DIR_PIN);
Servo myServo;

enum State {
  IDLE,
  MOVING_UP,
  WAITING_AT_TOP,
  MOVING_DOWN,
  WAITING_FOR_MARK,
  MARKED_DELAY,
  MOVING_UP_4M,
  WAITING_AT_4M,
  RETURN_TO_MARK,
  WAITING_TO_RESUME,
  GO_TO_GROUND,
  FINAL_DELAY
};

State state = IDLE;
unsigned long waitStartTime = 0;
long markedPosition = 0;
bool markReceived = false;
bool startReceived = false;

void setup() {
  Serial.begin(9600);

  // Stepper
  pinMode(ENABLE_PIN, OUTPUT);
  digitalWrite(ENABLE_PIN, LOW);
  stepper.setMaxSpeed(100000);
  stepper.setAcceleration(5000);

  // Wheel motor
  pinMode(WHEEL_DIR, OUTPUT);
  pinMode(WHEEL_PWM, OUTPUT);

  // BTS7960
  pinMode(RPWM, OUTPUT);
  pinMode(LPWM, OUTPUT);
  pinMode(R_EN, OUTPUT);
  pinMode(L_EN, OUTPUT);
  digitalWrite(R_EN, HIGH);
  digitalWrite(L_EN, HIGH);

  // Servo
  myServo.attach(SERVO_PIN);
  myServo.write(SERVO_BOTTOM_ANGLE);
}

void loop() {
  // Check for serial command from Jetson
  if (Serial.available()) {
    String command = Serial.readStringUntil('\n');
    command.trim();

    if (command == "START" && !startReceived) {
      startReceived = true;
      state = MOVING_UP;
      stepper.moveTo(MAX_HEIGHT_STEPS);
      startBTS7960Motor();
      myServo.write(SERVO_BOTTOM_ANGLE);
      Serial.println("Cycle Started");
    }

    if (command == "MARK") {
      markReceived = true;
      markedPosition = stepper.currentPosition();
      state = MARKED_DELAY;
      waitStartTime = millis();
      Serial.println("Mark Received");
    }
  }

  switch (state) {
    case MOVING_UP:
      if (stepper.distanceToGo() == 0) {
        stopBTS7960Motor();
        myServo.write(SERVO_TOP_ANGLE);
        waitStartTime = millis();
        state = WAITING_AT_TOP;
        Serial.println("Top Reached");
      }
      break;

    case WAITING_AT_TOP:
      if (millis() - waitStartTime >= DELAY_AT_TOP) {
        stepper.moveTo(0);
        state = MOVING_DOWN;
        Serial.println("DOWN_START");
      }
      break;

    case MOVING_DOWN:
      Serial.println("DOWN_START");  // Signal Jetson to start detection
      state = WAITING_FOR_MARK;
      break;

    case WAITING_FOR_MARK:
      // Wait for Jetson to send "MARK"
      break;

    case MARKED_DELAY:
      if (millis() - waitStartTime >= 2000) {
        stepper.moveTo(markedPosition - TWO_METERS_STEPS);
        startBTS7960Motor();
        myServo.write(SERVO_BOTTOM_ANGLE);
        state = MOVING_UP_4M;
        Serial.println("Moving Up 4M");
      }
      break;

    case MOVING_UP_4M:
      if (stepper.distanceToGo() == 0) {
        waitStartTime = millis();
        myServo.write(SERVO_TOP_ANGLE);
        stopBTS7960Motor();
        state = WAITING_AT_4M;
        Serial.println("At 4M Above Mark");
      }
      break;

    case WAITING_AT_4M:
      if (millis() - waitStartTime >= 2000) {
        stepper.moveTo(markedPosition);
        state = RETURN_TO_MARK;
      }
      break;

    case RETURN_TO_MARK:
      if (stepper.distanceToGo() == 0) {
        waitStartTime = millis();
        Serial.println("RESUME_DETECT");
        state = WAITING_TO_RESUME;
      }
      break;

    case WAITING_TO_RESUME:
      if (millis() - waitStartTime >= 2000) {
        stepper.moveTo(0);
        state = GO_TO_GROUND;
      }
      break;

    case GO_TO_GROUND:
      if (stepper.distanceToGo() == 0) {
        Serial.println("DETECT_STOP");
        waitStartTime = millis();
        runWheelMotor();
        state = FINAL_DELAY;
      }
      break;

    case FINAL_DELAY:
      if (millis() - waitStartTime >= 2000) {
        stopWheelMotor();
      }
      if (millis() - waitStartTime >= DELAY_AT_BOTTOM) {
        stepper.moveTo(MAX_HEIGHT_STEPS);
        myServo.write(SERVO_BOTTOM_ANGLE);
        startBTS7960Motor();
        state = MOVING_UP;
        Serial.println("Restarting Cycle");
      }
      break;

    default:
      break;
  }

  stepper.run();
}

// Motor functions
void startBTS7960Motor() {
  analogWrite(RPWM, 0);
  analogWrite(LPWM, 90);
  Serial.println("BTS7960 Motor ON");
}

void stopBTS7960Motor() {
  analogWrite(RPWM, 0);
  analogWrite(LPWM, 0);
  Serial.println("BTS7960 Motor OFF");
}

void runWheelMotor() {
  digitalWrite(WHEEL_DIR, HIGH);
  analogWrite(WHEEL_PWM, MOTOR_SPEED);
}

void stopWheelMotor() {
  analogWrite(WHEEL_PWM, 0);
}

