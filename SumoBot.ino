/**
  National Robotics Comptetion 2023
  NAME: Sumo Bot 
  FUNCTION: Sumo Bot go brrrr. (RUSHED)

  @software: Gappi, Jeric Marcel L.
  @hardware: Nino, Dominik O.
  @version: 2 2/12/2023

  This version was rushed and should be refactor based on arena.
*/

#include <IRremote.h>
// LEFT SIDE MOTORS
#define ENA_BACK_L 8 // Enable Pin for Speed -> BACK LEFT MOTOR
#define IN1_BACK_L 9 // BW Motor Movement
#define IN2_BACK_L 10 // FW Motor Movement

#define ENB_FRONT_L 13 // Enable Pin for Speed -> FRONT LEFT MOTOR
#define IN4_FRONT_L 12 // FW Motor Movement
#define IN3_FRONT_L 11 // BW Motor Movement

// RIGHT SIDE MOTORS
#define ENA_FRONT_R 7// Enable Pin for Speed -> FRONT RIGHT MOTOR
#define IN1_FRONT_R 6 // BW Motor Movement
#define IN2_FRONT_R 5 // FW Motor Movement

#define ENB_BACK_R 2 // Enable Pin for Speed -> BACK RIGHT MOTOR
#define IN4_BACK_R 3 // FW Motor Movement
#define IN3_BACK_R 4 // BW Motor Movement

#define IR_FRONT_L A0
#define IR_FRONT_R A1
//#define IR_BACK_L A2
#define IR_BACK A3
#define TRIG_FRONT A4
#define ECHO_FRONT A5
#define TRIG_BACK A6
#define ECHO_BACK A7
#define TRIG_LEFT A8
#define ECHO_LEFT A9
#define TRIG_RIGHT A10
#define ECHO_RIGHT A11
#define IR_RECEIVER 14

#define MAX_DISTANCE 15

int irArray[3] = {0, 0, 0};
int irArrayPins[3] = {IR_FRONT_L, IR_FRONT_R, IR_BACK};
int ultrasonicData[4] = {0, 0, 0, 0};
int ultrasonicPins[4][2] = {{ECHO_FRONT, TRIG_FRONT}, {ECHO_LEFT, TRIG_LEFT}, {ECHO_RIGHT, TRIG_RIGHT}, {ECHO_BACK, TRIG_BACK}};
long inches;

IRrecv IR(A12);

bool is_on = false;

void setup() {
  IR.enableIRIn();
  Serial.begin(9600);
  // Pin declaration of L298N Motor Driver
  for(uint8_t i = 2; i <= 13; i++) {
    pinMode(i, OUTPUT);
  }
  // ECHO PINS
  for(uint8_t i = 0; i < 4; i++) {
    pinMode(ultrasonicPins[i][0], INPUT);
  }
  // TRIG PINS
  for(uint8_t i = 0; i < 4; i++) {
    pinMode(ultrasonicPins[i][1], OUTPUT);
  }
  // Pin Declaration of IRs
  int a = A3;
  for(uint8_t i = 0; i < 3; i++) {
    pinMode(irArrayPins[i], INPUT);
  }
  pinMode(52, OUTPUT);
  pinMode(IR_RECEIVER, INPUT);
  // Sets the IN pin of L298N to LOW.
  digitalWrite(IN1_BACK_L, LOW);
  digitalWrite(IN2_BACK_L, LOW);
  digitalWrite(IN3_FRONT_L, LOW);
  digitalWrite(IN4_FRONT_L, LOW);

  digitalWrite(IN1_FRONT_R, LOW);
  digitalWrite(IN2_FRONT_R, LOW);
  digitalWrite(IN3_BACK_R, LOW);
  digitalWrite(IN4_BACK_R, LOW);

  while(!is_on)
    checkRemote();
}

void loop() {
  checkRemote();
  updateDistance();
  checkIR();
  checkRemote();
  if(ultrasonicData[0] >= 0 && ultrasonicData[0] <= MAX_DISTANCE && irArray[0] == 0 && irArray[1] == 0 && irArray[0] == 0 && irArray[1] == 0) {
    attack();
    checkIR();
    checkRemote();
  }
  else {
    checkIR();
    if(irArray[0] == 0 && irArray[1] == 0 && irArray[2] == 0) {
      search_left();
    }
    
    updateDistance();
    decision();
  }
}

// Checks the IR Remote for input.
void checkRemote() {
  if(IR.decode()) {
    if(IR.decodedIRData.decodedRawData == 0xBC43FF00 || IR.decodedIRData.decodedRawData == 0xBF40FF00 || IR.decodedIRData.decodedRawData == 0xBB44FF00) {
      digitalWrite(52, HIGH);
      is_on = true;
    }
    else if(IR.decodedIRData.decodedRawData == 0xF807FF00 || IR.decodedIRData.decodedRawData == 0xEA15FF00 || IR.decodedIRData.decodedRawData == 0xF609FF00) {
      is_on = false;
      digitalWrite(52, LOW);
      asm volatile ("jmp 0");
    }
    IR.resume();
  }
}

// Creates decision based on distance and IR. This should be paired with checkIR() function.
void decision() {
  checkRemote();
  if(ultrasonicData[1] >= 0 && ultrasonicData[1] <= MAX_DISTANCE && irArray[0] == 0 && irArray[1] == 0 && irArray[2] == 0) { // left
      left(180, 180, 180, 180);
      delay(150);
    }
    else if(ultrasonicData[2] >= 0 && ultrasonicData[2] <= MAX_DISTANCE && irArray[1] == 0 && irArray[2] == 0) { // right
      right(180, 180, 180, 180);
      delay(150);
    }
    else if(ultrasonicData[2] >= 0 && ultrasonicData[2] <= MAX_DISTANCE && irArray[0] == 0 && irArray[1] == 0 && irArray[2] == 0) { // back
      right(180, 180, 180, 180);
      delay(300);
  }
}

// A predetermined move attack.
void attack() {
  perpendicular();
  forward(180, 180, 180, 180);
  delay(320);
  stop();
  delay(20);
  perpendicular_r();
  updateDistance();
  while(ultrasonicData[0] >= 0 && ultrasonicData[0] <= MAX_DISTANCE) {
    checkRemote();
    for(int i = 0; i < 3; i++) {
      irArray[i] = digitalRead(irArrayPins[i]);
    }
    
    if(ultrasonicData[0] >= 0 && ultrasonicData[0] <= MAX_DISTANCE && irArray[0] == 0 && irArray[1] == 0) {
      forward(250, 250, 250, 250);
    }
    else {
      updateDistance();
      decision();
      checkIR();
      break;
    }
    updateDistance();
    decision();
    checkIR();
    right(180, 180, 180, 180);
    delay(60);
    stop();
  }
  checkIR();
  stop();
  delay(10);
}

// Search for opponent. Rotates left.
void search_left() {
  forward(180, 180, 180, 180);
  delay(100);
  left(180, 180, 180, 180);
  delay(110);
  stop();
  updateDistance();
  delay(20);
}

// Search for opponent. Rotates right.
void search_right() {
  forward(180, 180, 180, 180);
  delay(100);
  right(180, 180, 180, 180);
  delay(150);
  stop();
  updateDistance();
  delay(20);
}

// Tokyo drift left
void perpendicular() {
  forward(180, 180, 180, 180);
  delay(47);
  left(180, 180, 180, 180);
  delay(147);
  stop();
  delay(20);
}

// Tokyo drift right
void perpendicular_r() {
  forward(180, 180, 180, 180);
  delay(45);
  right(180, 180, 180, 180);
  delay(142);
  stop();
  delay(20);
}

void reverseRotate_r(int d) {
  reverse(200, 200, 200, 200);
  delay(45);
  right_r(250, 250, 250, 250);
  delay(d);
}

// Updates the IR Sensors.
void checkIR() {
  stop();
  delay(10);
  for(int i = 0; i < 3; i++) {
    irArray[i] = digitalRead(irArrayPins[i]);
  }
  
  if((irArray[0] == 1 || irArray[1] == 1) && irArray[2] == 0) {
    stop();
    reverse(180, 180, 180, 180);
    delay(200);
    right(180, 180, 180, 180);
    delay(300);
  }/**
  else if(irArray[0] == 1 && irArray[1] == 0 && irArray[2] == 0) {
    reverse(180, 180, 180, 180);
    delay(150);
  }
  else if(irArray[0] == 0 && irArray[1] == 1 && irArray[2] == 0) {
    reverse(180, 180, 180, 180);
    delay(150);
  }*/
  else if(irArray[0] == 0 && irArray[1] == 0 && irArray[2] == 1) {
    forward(200, 200, 200, 200); // change to reverse rotate.
    delay(150);
  }
  else if((irArray[0] == 1 || irArray[1] == 1) && irArray[2] == 1) {
    reverseRotate_r(150);
  }
  else if(irArray[0] == 1 && irArray[1] == 1 && irArray[2] == 1) {
    stop();
    delay(180);
  }
}

// Updates the distance calculated by the Ultrasonic sensor.
void updateDistance() {
  long duration;
  for(uint8_t i = 0; i < 4; i++) {
    duration = pingPulse(ultrasonicPins[i][0], ultrasonicPins[i][1]);
    inches = microsecondsToInches(duration);
    ultrasonicData[i] = inches;
    delay(50);
  }
}

/**
  Moves the robot forward with the desired speed.

  @param fl_spd speed of the front left motor
  @param fr_spd speed of the front right motor
  @param bl_spd speed of the back left motor
  @param br_spd speed of the back right motor
*/
void forward(int fl_spd, int fr_spd, int bl_spd, int br_spd) {
  //Serial.println("FORWARD");
  analogWrite(ENB_FRONT_L, fl_spd);
  analogWrite(ENA_FRONT_R, fr_spd);
  analogWrite(ENA_BACK_L, bl_spd);
  analogWrite(ENB_BACK_R, br_spd);

  digitalWrite(IN1_BACK_L, HIGH);
  digitalWrite(IN2_BACK_L, LOW);

  digitalWrite(IN3_FRONT_L, LOW);
  digitalWrite(IN4_FRONT_L, HIGH);

  digitalWrite(IN1_FRONT_R, LOW);
  digitalWrite(IN2_FRONT_R, HIGH);

  digitalWrite(IN3_BACK_R, HIGH);
  digitalWrite(IN4_BACK_R, LOW);
}

/**
  Moves the robot in reverse with the desired speed.

  @param fl_spd speed of the front left motor
  @param fr_spd speed of the front right motor
  @param bl_spd speed of the back left motor
  @param br_spd speed of the back right motor
*/
void reverse(int fl_spd, int fr_spd, int bl_spd, int br_spd) {
  analogWrite(ENB_FRONT_L, fl_spd);
  analogWrite(ENA_FRONT_R, fr_spd);
  analogWrite(ENA_BACK_L, bl_spd);
  analogWrite(ENB_BACK_R, br_spd);

  digitalWrite(IN1_BACK_L, LOW);
  digitalWrite(IN2_BACK_L, HIGH);

  digitalWrite(IN3_FRONT_L, HIGH);
  digitalWrite(IN4_FRONT_L, LOW);

  digitalWrite(IN1_FRONT_R, HIGH);
  digitalWrite(IN2_FRONT_R, LOW);

  digitalWrite(IN3_BACK_R, LOW);
  digitalWrite(IN4_BACK_R, HIGH);
}

// Stops the motor.
void stop(){
  analogWrite(ENB_FRONT_L, 0);
  analogWrite(ENA_FRONT_R, 0);
  analogWrite(ENA_BACK_L, 0);
  analogWrite(ENB_BACK_R, 0);

  digitalWrite(IN1_BACK_L, LOW);
  digitalWrite(IN2_BACK_L, LOW);
  digitalWrite(IN3_FRONT_L, LOW);
  digitalWrite(IN4_FRONT_L, LOW);

  digitalWrite(IN1_FRONT_R, LOW);
  digitalWrite(IN2_FRONT_R, LOW);
  digitalWrite(IN3_BACK_R, LOW);
  digitalWrite(IN4_BACK_R, LOW);
}

/**
  Moves the robot to the left with the desired speed.

  @param fl_spd speed of the front left motor
  @param fr_spd speed of the front right motor
  @param bl_spd speed of the back left motor
  @param br_spd speed of the back right motor
*/
void left(int fl_spd, int fr_spd, int bl_spd, int br_spd) {
  analogWrite(ENB_FRONT_L, fl_spd);
  analogWrite(ENA_FRONT_R, fr_spd);
  analogWrite(ENA_BACK_L, bl_spd);
  analogWrite(ENB_BACK_R, br_spd);

  digitalWrite(IN1_BACK_L, LOW);
  digitalWrite(IN2_BACK_L, HIGH);

  digitalWrite(IN3_FRONT_L, HIGH);
  digitalWrite(IN4_FRONT_L, LOW);

  digitalWrite(IN1_FRONT_R, LOW);
  digitalWrite(IN2_FRONT_R, HIGH);

  digitalWrite(IN3_BACK_R, HIGH);
  digitalWrite(IN4_BACK_R, LOW);
}

/**
  Moves the robot to the right with the desired speed.

  @param fl_spd speed of the front left motor
  @param fr_spd speed of the front right motor
  @param bl_spd speed of the back left motor
  @param br_spd speed of the back right motor
*/
void right(int fl_spd, int fr_spd, int bl_spd, int br_spd) {
  analogWrite(ENB_FRONT_L, fl_spd);
  analogWrite(ENA_FRONT_R, fr_spd);
  analogWrite(ENA_BACK_L, bl_spd);
  analogWrite(ENB_BACK_R, br_spd);

  digitalWrite(IN1_BACK_L, HIGH);
  digitalWrite(IN2_BACK_L, LOW);

  digitalWrite(IN3_FRONT_L, LOW);
  digitalWrite(IN4_FRONT_L, HIGH);

  digitalWrite(IN1_FRONT_R, HIGH);
  digitalWrite(IN2_FRONT_R, LOW);

  digitalWrite(IN3_BACK_R, LOW);
  digitalWrite(IN4_BACK_R, HIGH);
}

/**
  Moves the robot to the right (Reverse) with the desired speed.

  @param fl_spd speed of the front left motor
  @param fr_spd speed of the front right motor
  @param bl_spd speed of the back left motor
  @param br_spd speed of the back right motor
*/
void right_r(int fl_spd, int fr_spd, int bl_spd, int br_spd) {
  analogWrite(ENB_FRONT_L, fl_spd);
  analogWrite(ENA_FRONT_R, fr_spd);
  analogWrite(ENA_BACK_L, bl_spd);
  analogWrite(ENB_BACK_R, br_spd);

  digitalWrite(IN1_BACK_L, LOW);
  digitalWrite(IN2_BACK_L, HIGH);

  digitalWrite(IN3_FRONT_L, HIGH);
  digitalWrite(IN4_FRONT_L, LOW);

  digitalWrite(IN1_FRONT_R, LOW);
  digitalWrite(IN2_FRONT_R, HIGH);

  digitalWrite(IN3_BACK_R, HIGH);
  digitalWrite(IN4_BACK_R, LOW);
}

/**
  Get the duration (in microseconds) of the pulses triggered by Trig Pin and produced by Echo Pin.

  @param ECHO_PIN the echo pin of the current Ultrasonic Sensor
  @param TRIG_PIN the trig pin of the current Ultrasonic Sensor
  @return the length of the pulse in microseconds (bcs it is proportional to the time it was detected)
*/
long pingPulse(int ECHO_PIN, int TRIG_PIN) {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10); // as per datasheet recommendations
  digitalWrite(TRIG_PIN, LOW);

  return pulseIn(ECHO_PIN, HIGH);
}

/**
  Converts Microseconds to Inches

  @param microseconds the microseconds to be converted
  @return inches (int)
*/
long microsecondsToInches(long microseconds) {
  return microseconds / 74 / 2;
}