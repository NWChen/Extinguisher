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
#define SPONGE_DOWN 20
#define SERVO_PWM 9

// Sensor constants
#define IR_LEFT 3
#define IR_MIDDLE 4
#define IR_RIGHT 5
#define IR_FLAME 2
#define LIMIT_SWITCH 1
#define THRESHOLD_BLACK 100
#define THRESHOLD_FLAME 50
#define THRESHOLD_SWITCH 600
#define MAX_READINGS 100

int flameReadings[MAX_READINGS];
int currentReading = 0;
Servo servo;
String lastPrintedString = "";

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
  
  // Wait for user input to begin moving.
  while(!isSwitchPressed()) {
    print("PRESS SWITCH TO START.");
  }
}

void loop() {
  // Line following subroutine
  // We follow the right edge of the line, which is assumed to have thickness wider
  // than the range of the IR transmitter-receiver pair,
  // so as to implicitly handle intersections of varying degree.
  
  // Update flame readings once every loop
  flameReadings[currentReading] = getFlame();
  currentReading++;
  if(currentReading >= 100)
    currentReading = 0;
  
  // Seek a flame.
  if(getFlame() > THRESHOLD_FLAME) {
    print("POTENTIAL FLAME DETECTED.");
    if(seekFlame()) {
      approachFlame();
      deploy();
    }
  }
  
  // |? x  | Robot to the right of the right line edge
  if(isMiddleOn() && !isRightOn()) {
    print("|? x  |");
    moveForward(SPEED);
    delay(MOTION_DELAY);
    releaseAllMotors();
  }

  // |? ? x| Robot to the left of the right line edge
  else if(isRightOn()) {
    print("|? ? x|");
    halfTurnRight(SPEED);
    delay(MOTION_DELAY);
    releaseAllMotors();
  }

  // |x    | Robot far to the right of the right line edge
  else if(isLeftOn() && !isMiddleOn() && !isRightOn()) {
    print("|x    |");
    halfTurnLeft(SPEED);
    delay(MOTION_DELAY);
    releaseAllMotors();
  }
  
  // |     | Robot has completely wandered off the line
  else if(!isLeftOn() && !isMiddleOn() && !isRightOn()) {
    print("|     |");
    turnLeft(SPEED);
    delay(MOTION_DELAY);
    releaseAllMotors();
  }
}

/*
 * ****************************************************
 * Functional abstractions for robot motion
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

void releaseAllMotors() {
  motor(LEFT_MOTOR, RELEASE, 0);
  motor(RIGHT_MOTOR, RELEASE, 0);
}

void deploy() {
  // Reset the smothering mechanism to its upright position
  const int TIMEOUT = 5;
  if(servo.read() < SPONGE_UP)
    for(int i=servo.read(); i<SPONGE_UP; i++) {
      servo.write(i);
      delay(TIMEOUT);
    }
  else
    for(int i=servo.read(); i>SPONGE_UP; i++) {
      servo.write(i);
      delay(TIMEOUT);
    } 
  
  // Smoothly deploy the smothering mechanism
  // Accelerate as we approach the midpoint of the range of motion
  for(int i=servo.read(); i>SPONGE_DOWN+((SPONGE_UP-SPONGE_DOWN)/2); i--) {
    servo.write(pow(i,1.5));
    delay(TIMEOUT);
  }
  // Decelerate as we approach the end of the range of motion
  for(int i=servo.read(); i>SPONGE_DOWN; i--) {
    servo.write(pow(i,1.5));
    delay(TIMEOUT);
  }
  
  // Reset the smothering mechanism to its upright position
  for(int i=servo.read(); i<SPONGE_UP; i++) {
    servo.write(i);
    delay(TIMEOUT);
  }
}

/*
 * *******************************************************
 * Functional abstractions for robot sensing and debugging
 * *******************************************************
 */
boolean isSwitchPressed() {
  if (analogRead(LIMIT_SWITCH) > 1000)
    return true;
  return false;
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

// Get the value of the IR flame sensor.
// Map this value to a smaller range to prevent integer overflow when computing a moving average.
// Higher value corresponds to a higher flame intensity.
int getFlame() {
  return map(analogRead(IR_FLAME), 0, 1023, 100, 0);
}

// Get the average of the last 100 samples.
int getAverageFlame() {
  int sum = 0;
  for(int i=0; i<MAX_READINGS; i++)
    sum += flameReadings[i];
  return sum/MAX_READINGS;
}

// Turn towards maximum flame readings.
// Returns true if the algorithm can converge on a flame; false otherwise.
boolean seekFlame() {
  const int TURN_DELAY = 1000;
  const int TURN_SPEED = 150;
  
  // Scan right for a flame
  int maxFlameRight = 0;
  int startTime = millis();
  while((millis()-startTime) < TURN_DELAY) {
    print("SCANNING RIGHT FOR FLAME.");
    turnRight(TURN_SPEED);
    delay(MOTION_DELAY);
    releaseAllMotors();
    if(getFlame() > maxFlameRight)
      maxFlameRight = getFlame();
  }
  
  // Return to center position
  startTime = millis();
  while((millis()-startTime) < TURN_DELAY) {
    print("RETURNING TO CENTER.");
    turnLeft(TURN_SPEED);
    delay(MOTION_DELAY);
    releaseAllMotors();
  }
  
  // Scan left for a flame
  int maxFlameLeft = 0;
  startTime = millis();
  while((millis()-startTime) < TURN_DELAY) {
    print("SCANNING LEFT FOR FLAME.");
    turnLeft(TURN_SPEED);
    delay(MOTION_DELAY);
    releaseAllMotors();
    if(getFlame() > maxFlameLeft)
      maxFlameLeft = getFlame();
  }
  
  // Determine whether a flame actually exists
  if((maxFlameLeft < 30) && (maxFlameRight < 30)) {
    print("DID NOT CONVERGE ON A FLAME.");
    return false;
  }
  
  // Evaluate scan results
  if(maxFlameRight > maxFlameLeft) 
    // Turn towards the flame if it is towards the right.
    while(getFlame() < (maxFlameRight-5)) {
      print("PURSUING RIGHT FLAME.");
      turnRight(TURN_SPEED);
      delay(MOTION_DELAY);
      releaseAllMotors();
    }
  else 
    // Turn towards the flame if it is towards the left.
    while(getFlame() < (maxFlameLeft-5)) {
      print("PURSUING LEFT FLAME.");
      turnLeft(TURN_SPEED);
      delay(MOTION_DELAY);
      releaseAllMotors();
    }
  return true;
}

// Approach a flame by getting as close as possible to it.
void approachFlame() {
  while(!isSwitchPressed()) {
    print("APPROACHING FLAME.");
    moveForward(150);
    delay(MOTION_DELAY);
    releaseAllMotors();
  }
}

void print(String s) {
  if(s!=lastPrintedString) {
    Serial.println(s);
    s = lastPrintedString;
  }
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

