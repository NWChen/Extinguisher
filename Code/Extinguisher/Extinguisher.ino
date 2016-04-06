#include <Servo.h>

// Motor control constants
#define MOTORLATCH 12
#define MOTORCLK 4
#define MOTORENABLE 7
#define MOTORDATA 8
#define LEFT_MOTOR 2
#define RIGHT_MOTOR 3
#define MOTOR1_A 1
#define MOTOR1_B 4
#define MOTOR2_A 2
#define MOTOR2_B 3
#define MOTOR3_A 5 // 7
#define MOTOR3_B 7 // 0
#define MOTOR4_A 13 // 2
#define MOTOR4_B 14 // 5
#define MOTOR1_PWM 11
#define MOTOR2_PWM 3
#define MOTOR3_PWM 6
#define MOTOR4_PWM 5
#define FORWARD 1
#define BACKWARD 2
#define BRAKE 3
#define RELEASE 4
#define SPEED 255
#define HALF_SPEED 150
#define MOTION_DELAY 10

// Servo control constants
#define SPONGE_UP 90
#define SPONGE_DOWN 160
#define SERVO_PWM 9

// Sensor constants
#define IR_LEFT 3
#define IR_MIDDLE 4
#define IR_RIGHT 5
#define IR_FLAME 2
#define LIMIT_SWITCH 1
#define THRESHOLD_BLACK 100
#define THRESHOLD_FLAME 970
#define THRESHOLD_SWITCH 600

Servo servo;
float maxTurnTime = 0;

/*
 * ****************************************************
 * Setup and loop functions for execution
 * ****************************************************
 */
void setup()
{
  Serial.begin(9600);
  servo.attach(SERVO_PWM);
  servo.write(SPONGE_UP);
}

void loop() {

  // Line following subroutine
  // |  x  | Robot in the center of the line
  if (!isLeftOn() && isMiddleOn() && !isRightOn()) {
    Serial.println("|  x  |");
    moveForward(SPEED);
    delay(MOTION_DELAY);
    releaseAllMotors();
  }

  // |x x  | or |x    |Robot to the right of the line
  else if ((isLeftOn() || isMiddleOn()) && !isRightOn()) {
    Serial.println("|x x  | or |x    |");
    halfTurnLeft(SPEED);
    delay(MOTION_DELAY);
    releaseAllMotors();
  }

  // |  x x| or |    x| Robot to the left of the line
  else if ((isRightOn() || isMiddleOn()) && !isLeftOn()) {
    Serial.println("|  x x| or |    x|");
    halfTurnRight(SPEED);
    delay(MOTION_DELAY);
    releaseAllMotors();
  }

  // |x x x| Robot at intersection
  else if (isLeftOn() && isMiddleOn() && isRightOn()) {
    Serial.println("|x x x|");
    
    // Variables to determine the state of the intersection.
    boolean forwardLineExists = false;
    boolean rightLineExists = false;
    boolean leftLineExists = false;
    
    // Move forward until the line sensor clears the forward edge of the intersection.
    // This ensures the line sensor never mistakes the intersection for another line.
    while(isLeftOn() && isMiddleOn() && isRightOn()) {
      Serial.println("CLEARING INTERSECTION...");
      moveForward(SPEED);
      delay(MOTION_DELAY);
    }
    
    // Proceed forward slightly, just to give us extra clearance from the forward edge of the intersection.
    // Then determine whether a forward line exists.
    moveForward(SPEED);
    delay(250); // Change this to a value dependent on maxTurnTime
    if(isLeftOn() || isMiddleOn() || isRightOn())
      forwardLineExists = true;
    
    // Determine whether a right line exists.
    // Turn as close to a right angle as we can, making measurements along the way.
    float startTime = millis();
    while(!hasExceededRightAngle(millis() - startTime)) {
      Serial.println("SEEKING RIGHT LINE...");
      turnRight(SPEED);
      delay(MOTION_DELAY);
      
      // Check if we've detected a line.
      // Ensure we're not just detecting the line that we started on.
      if(isMiddleOn() && (millis()-startTime > maxTurnTime/5.0))
        rightLineExists = true;
    }
      
    // If a right line exists, we should take it,
    // so check whether we found a right line or not.
  }

  // |     | Robot has wandered off completely
  else {
    Serial.println("NO LINE");
    moveForward(SPEED);
    delay(MOTION_DELAY);
    releaseAllMotors();
  }

  /*
  // IR sensor has detected a flame
  if(isFlameDetected() || isSwitchHit()) {
    Serial.println("FLAME DETECTED");
    deploy();
  }
  */
  // Limit switch has been struck
}

/*
 * ****************************************************
 * Functional abstractions for robot motion and sensing
 * ****************************************************
 */

void moveForward(int speed) {
  motor(LEFT_MOTOR, BACKWARD, speed);
  motor(RIGHT_MOTOR, BACKWARD, speed);
}

void moveBackward(int speed) {
  motor(LEFT_MOTOR, FORWARD, speed);
  motor(RIGHT_MOTOR, FORWARD, speed);
}

void turnLeft(int speed) {
  motor(LEFT_MOTOR, FORWARD, speed);
  motor(RIGHT_MOTOR, BACKWARD, speed);
}

void turnRight(int speed) {
  motor(LEFT_MOTOR, BACKWARD, speed);
  motor(RIGHT_MOTOR, FORWARD, speed);
}

void halfTurnLeft(int speed) {
  motor(RIGHT_MOTOR, BACKWARD, speed);
}

void halfTurnRight(int speed) {
  motor(LEFT_MOTOR, BACKWARD, speed);
}

boolean isSwitchPressed() {
  if (analogRead(LIMIT_SWITCH) > 1000)
    return true;
  return false;
}

boolean hasExceededRightAngle(float time) {
  if(time > maxTurnTime)
    return true;
  return false;
}

void releaseAllMotors() {
  motor(LEFT_MOTOR, RELEASE, 0);
  motor(RIGHT_MOTOR, RELEASE, 0);
}

void deploy() {
  servo.write(SPONGE_UP);
  delay(100);

  // Profile a motion to prevent stripping the servo from the momentum of the smothering linkage
  for (int i = 0; i < (SPONGE_DOWN - SPONGE_UP); i++) {
    servo.write(i + SPONGE_UP);
    delay(i / 2);
  }

  delay(100);
  servo.write(SPONGE_UP);
  delay(500);
}

boolean isLeftOn() {
  return analogRead(IR_LEFT) > THRESHOLD_BLACK;
}

boolean isMiddleOn() {
  return analogRead(IR_MIDDLE) > THRESHOLD_BLACK;
}

boolean isRightOn() {
  return analogRead(IR_RIGHT) > THRESHOLD_BLACK;
}

boolean isFlameDetected() {
  return analogRead(IR_FLAME) < THRESHOLD_FLAME;
}

/*
 * ****************************************************
 * Predefined low-level control code
 * ****************************************************
 */
void motor(int nMotor, int command, int speed)
{
  int motorA, motorB;

  if (nMotor >= 1 && nMotor <= 4)
  {
    switch (nMotor)
    {
      case 1:
        motorA   = MOTOR1_A;
        motorB   = MOTOR1_B;
        break;
      case 2:
        motorA   = MOTOR2_A;
        motorB   = MOTOR2_B;
        break;
      case 3:
        motorA   = MOTOR3_A;
        motorB   = MOTOR3_B;
        break;
      case 4:
        motorA   = MOTOR4_A;
        motorB   = MOTOR4_B;
        break;
      default:
        break;
    }

    switch (command)
    {
      case FORWARD:
        motor_output (motorA, HIGH, speed);
        motor_output (motorB, LOW, -1);     // -1: no PWM set
        break;
      case BACKWARD:
        motor_output (motorA, LOW, speed);
        motor_output (motorB, HIGH, -1);    // -1: no PWM set
        break;
      case BRAKE:
        // The AdaFruit library didn't implement a brake.
        // The L293D motor driver ic doesn't have a good
        // brake anyway.
        // It uses transistors inside, and not mosfets.
        // Some use a software break, by using a short
        // reverse voltage.
        // This brake will try to brake, by enabling
        // the output and by pulling both outputs to ground.
        // But it isn't a good break.
        motor_output (motorA, LOW, 255); // 255: fully on.
        motor_output (motorB, LOW, -1);  // -1: no PWM set
        break;
      case RELEASE:
        motor_output (motorA, LOW, 0);  // 0: output floating.
        motor_output (motorB, LOW, -1); // -1: no PWM set
        break;
      default:
        break;
    }
  }
}


// ---------------------------------
// motor_output
//
// The function motor_ouput uses the motor driver to
// drive normal outputs like lights, relays, solenoids,
// DC motors (but not in reverse).
//
// It is also used as an internal helper function
// for the motor() function.
//
// The high_low variable should be set 'HIGH'
// to drive lights, etc.
// It can be set 'LOW', to switch it off,
// but also a 'speed' of 0 will switch it off.
//
// The 'speed' sets the PWM for 0...255, and is for
// both pins of the motor output.
//   For example, if motor 3 side 'A' is used to for a
//   dimmed light at 50% (speed is 128), also the
//   motor 3 side 'B' output will be dimmed for 50%.
// Set to 0 for completelty off (high impedance).
// Set to 255 for fully on.
// Special settings for the PWM speed:
//    Set to -1 for not setting the PWM at all.
//
void motor_output (int output, int high_low, int speed)
{
  int motorPWM;

  switch (output)
  {
    case MOTOR1_A:
    case MOTOR1_B:
      motorPWM = MOTOR1_PWM;
      break;
    case MOTOR2_A:
    case MOTOR2_B:
      motorPWM = MOTOR2_PWM;
      break;
    case MOTOR3_A:
    case MOTOR3_B:
      motorPWM = MOTOR3_PWM;
      break;
    case MOTOR4_A:
    case MOTOR4_B:
      motorPWM = MOTOR4_PWM;
      break;
    default:
      // Use speed as error flag, -3333 = invalid output.
      speed = -3333;
      break;
  }

  if (speed != -3333)
  {
    // Set the direction with the shift register
    // on the MotorShield, even if the speed = -1.
    // In that case the direction will be set, but
    // not the PWM.
    shiftWrite(output, high_low);

    // set PWM only if it is valid
    if (speed >= 0 && speed <= 255)
    {
      analogWrite(motorPWM, speed);
    }
  }
}


// ---------------------------------
// shiftWrite
//
// The parameters are just like digitalWrite().
//
// The output is the pin 0...7 (the pin behind
// the shift register).
// The second parameter is HIGH or LOW.
//
// There is no initialization function.
// Initialization is automatically done at the first
// time it is used.
//
void shiftWrite(int output, int high_low)
{
  static int latch_copy;
  static int shift_register_initialized = false;

  // Do the initialization on the fly,
  // at the first time it is used.
  if (!shift_register_initialized)
  {
    // Set pins for shift register to output
    pinMode(MOTORLATCH, OUTPUT);
    pinMode(MOTORENABLE, OUTPUT);
    pinMode(MOTORDATA, OUTPUT);
    pinMode(MOTORCLK, OUTPUT);

    // Set pins for shift register to default value (low);
    digitalWrite(MOTORDATA, LOW);
    digitalWrite(MOTORLATCH, LOW);
    digitalWrite(MOTORCLK, LOW);
    // Enable the shift register, set Enable pin Low.
    digitalWrite(MOTORENABLE, LOW);

    // start with all outputs (of the shift register) low
    latch_copy = 0;

    shift_register_initialized = true;
  }

  // The defines HIGH and LOW are 1 and 0.
  // So this is valid.
  bitWrite(latch_copy, output, high_low);

  // Use the default Arduino 'shiftOut()' function to
  // shift the bits with the MOTORCLK as clock pulse.
  // The 74HC595 shiftregister wants the MSB first.
  // After that, generate a latch pulse with MOTORLATCH.
  shiftOut(MOTORDATA, MOTORCLK, MSBFIRST, latch_copy);
  delayMicroseconds(5);    // For safety, not really needed.
  digitalWrite(MOTORLATCH, HIGH);
  delayMicroseconds(5);    // For safety, not really needed.
  digitalWrite(MOTORLATCH, LOW);
}

