#include <QTRSensors.h>
#include <SharpIR.h>

#define NUM_SENSORS             8  // number of sensors used
#define NUM_SAMPLES_PER_SENSOR  4  // average 4 analog samples per sensor reading
#define EMITTER_PIN  QTR_NO_EMITTER_PIN // emitter is controlled by digital pin 2

//ultrasonic sensor
#define trig_front 6
#define echo_front 22

#define trig_back 7
#define echo_back 24

//Sharp IR
#define ir_right A11
#define ir_mid   A10
#define ir_left  A9

// IR model
#define model1 1080
#define model2 1080
#define model3 1080

// left motors
int  INA_leftfront = 3;
int INB_leftfront = 38;
int EN1_leftfront = 40;

int INA_leftback = 2;
int INB_leftback = 44;
int EN2_leftback = 42;

//right motors
int  INA_rightfront = 5;
int INB_rightfront = 41;
int EN1_rightfront = 39;

int INA_rightback = 4;
int INB_rightback = 45;
int EN2_rightback = 43;

//encoder pins
int encoderPin1_left = 36;
int encoderPin2_left = 34;
int encoderPin2Value_left = 0;
int Position_left = 0; // variable for position

int encoderPin1_right = 37;
int encoderPin2_right = 35;
int encoderPin2Value_right = 0;
int Position_right = 0; // variable for position

int initial_motor_speed = 120;

//pins for color sensors
#define S0 51
#define S1 53
#define S2 50
#define S3 48
#define sensorOut 52

int redMax = 0;
int redMin = 0;
int greenMax = 0;
int greenMin = 0;
int blueMax = 0;
int blueMin = 0;

int threshold = 100;

// sensors 0 through 5 are connected to analog inputs 0 through 5, respectively
QTRSensorsAnalog qtra((unsigned char[]) {
  0, 1, 2, 3, 4, 5, 6, 7
},
NUM_SENSORS, NUM_SAMPLES_PER_SENSOR, EMITTER_PIN);
unsigned int sensorValues[NUM_SENSORS];


void setup() {
  Serial.begin(9600);
  //define pinmode of motor pins
  pinMode(INA_leftfront, OUTPUT);
  pinMode(INB_leftfront, OUTPUT);
  pinMode(EN1_leftfront, OUTPUT);

  pinMode(INA_leftback, OUTPUT);
  pinMode(INB_leftback, OUTPUT);
  pinMode(EN2_leftback, OUTPUT);

  pinMode(INA_rightfront, OUTPUT);
  pinMode(INB_rightfront, OUTPUT);
  pinMode(EN1_rightfront, OUTPUT);

  pinMode(INA_rightback, OUTPUT);
  pinMode(INB_rightback, OUTPUT);
  pinMode(EN2_rightback, OUTPUT);

  //setup encoders
  pinMode(encoderPin1_left, INPUT);
  pinMode(encoderPin2_left, INPUT);
  attachInterrupt(digitalPinToInterrupt(encoderPin1_left), checkEncoder_left, RISING);

  pinMode(encoderPin1_right, INPUT);
  pinMode(encoderPin2_right, INPUT);
  attachInterrupt(digitalPinToInterrupt(encoderPin1_right), checkEncoder_right, RISING);


  //define pinMode for color sensor
  pinMode(S0, OUTPUT);
  pinMode(S1, OUTPUT);
  pinMode(S2, OUTPUT);
  pinMode(S3, OUTPUT);
  pinMode(sensorOut, INPUT);
  /// Setting frequency-scaling to 20%
  digitalWrite(S0, HIGH);
  digitalWrite(S1, LOW);

  pinMode(13, OUTPUT);
  digitalWrite(13, HIGH);    // turn on Arduino's LED to indicate we are in calibration mode
  for (int i = 0; i < 400; i++)  // make the calibration take about 10 seconds
  {
    qtra.calibrate();       // reads all sensors 10 times at 2.5 ms per six sensors (i.e. ~25 ms per call)
  }
  digitalWrite(13, LOW);     // turn off Arduino's LED to indicate we are through with calibration

  forward();
  analogWrite(INA_leftfront, 155); //Left Motor Speed
  analogWrite(INA_leftback, 155); //Right Motor Speed
  analogWrite(INA_rightfront, 155); //Left Motor Speed
  analogWrite(INA_rightback, 200);
  delay(300);


  //setup for ultrasonic
  pinMode(trig_front, OUTPUT);
  pinMode(trig_back, OUTPUT);

  pinMode(echo_front, INPUT);
  pinMode(echo_back, INPUT);
}

void loop() {
  // put your main code here, to run repeatedly:
  //followCurve();
  wall_following();
  /* unsigned int position = qtra.readLine(sensorValues);
    int Error = position - 2500;
    for (unsigned char i = 0; i < NUM_SENSORS; i++)
    { Serial.print(sensorValues[i]);
     Serial.print('\t');
    }
  */

}


//////////////////////////////////
/*
  void mazeSolver() {
  while (1) {
    char color = readColor();
    if (color == "R") {
      stopbot();
      return;
    }
    else {
      unsigned int position = qtra.readLine(sensorValues);
      int junction = findJunction();
      if (sensorValues[0] > threshold) { // condition for straight line
        followStraight();
      }
      else if (junction == 0) {
        followStraight();
        delay(50);
        //turnLeft();
        delay(10);
      }
      else if (junction == 1) {
        followStraight();
      }
      else if (junction == 2) {
        //turnRight();
        delay(10);
      }
      else if (junction == 3) {
      //  turnU();
        delay(10);
      }
    }
  }
  }
*/
//////////////////////////////////////////
/*
  int findJunction() {
  unsigned int position = qtra.readLine(sensorValues);
  if () { // condition for L junction
    return 0; //trun left
  }
  if () { // condition for T junction
    return 0;  //turn left
  }
  if () { // conditon for  R junction
    followStraight();
    delay(50);
    if () { // condition for T right junction
      return 1; //go straight
    }
    if () { //  condition black plane
      return 2;; // turn right
    }
  }
  if () { // condition for 180 turn
    return 3; // turn 180
  }
  }
*/
///////////////////////////////////////////
void followStraight() {
  unsigned int position = qtra.readLine(sensorValues);
  int Error = position - 3500;
  float PID_value_ir = IR_pid(Error);
  motor_control(PID_value_ir);
}

////////////////////////////////////////////
//function for first curve in task A
void followCurve() {
  while (1) {
    char color = readColor();
    if (color == "B") {
      stopbot();
      return;
    }
    else {
      unsigned int position = qtra.readLine(sensorValues);
      int Error = position - 3500;
      float PID_value_ir = IR_pid(Error);
      motor_control(PID_value_ir);
    }
  }
}

///////////////////////////////////////////////
float Kp_ir = 0.02;
float Ki_ir = 0;
float Kd_ir = 1;

float P_ir = 0, I_ir = 0, D_ir = 0, PID_value_ir = 0;
float previous_error_ir = 0, previous_I_ir = 0;

//PID function
float IR_pid(float error_ir)
{
  P_ir = error_ir;
  I_ir = I_ir + previous_I_ir;
  D_ir = error_ir - previous_error_ir;

  PID_value_ir = (Kp_ir * P_ir) + (Ki_ir * I_ir) + (Kd_ir * D_ir);

  previous_I_ir = I_ir;
  previous_error_ir = error_ir;
  return PID_value_ir;
}


//Motor control function
void motor_control(float PID_value_ir)
{
  // Calculating the effective motor speed:
  int left_motor_speed = initial_motor_speed + PID_value_ir;
  int right_motor_speed = initial_motor_speed -  PID_value_ir;

  // The motor speed should not exceed the max PWM value
  left_motor_speed = constrain(left_motor_speed, 0, 255);
  right_motor_speed = constrain(right_motor_speed, 0, 255);
  /*
    Serial.print(left_motor_speed);
    Serial.print("\t");
    Serial.println(right_motor_speed);
  */
  analogWrite(INA_leftfront, left_motor_speed * 0.95); //Left Motor Speed
  analogWrite(INA_leftback, left_motor_speed); //Right Motor Speed
  analogWrite(INA_rightfront, right_motor_speed); //Left Motor Speed
  analogWrite(INA_rightback, right_motor_speed * 1.1); //Right Motor Speed

  //following lines of code are to make the bot move forward
  forward();
}

//////////////////////////////////////////////
char readColor() {
  // Setting "red" read
  digitalWrite(S2, LOW);
  digitalWrite(S3, LOW);
  // Reading the output frequency
  int frequency = pulseIn(sensorOut, LOW);
  int R = frequency;
  // Printing the value on the serial monitor
  /*Serial.print("R= ");//printing name
    Serial.print(frequency);//printing RED color frequency
    Serial.print("  ");
    delay(50);*/

  // Setting "Green" reading
  digitalWrite(S2, HIGH);
  digitalWrite(S3, HIGH);
  // Reading the output frequency
  frequency = pulseIn(sensorOut, LOW);
  int G = frequency;
  // Printing the value on the serial monitor
  /*Serial.print("G= ");//printing name
    Serial.print(frequency);//printing RED color frequency
    Serial.print("  ");
    delay(50);*/

  //Setting "Blue" read
  digitalWrite(S2, LOW);
  digitalWrite(S3, HIGH);
  // Reading the output frequency
  frequency = pulseIn(sensorOut, LOW);
  int B = frequency;
  // Printing the value on the serial monitor
  /*Serial.print("B= ");//printing name
    Serial.print(frequency);//printing RED color frequency
    Serial.println("  ");
    delay(50);*/

  if (R<redMax & R>redMin & G<greenMax & G>greenMin) {
    Serial.println("R");
    return "R";
  }

  if (B<blueMax & B>blueMin & G<greenMax & G>greenMin) {
    Serial.println("G");
    return "G";
  }

  if (R<redMax & R>redMin & B<blueMax & B>blueMin) {
    Serial.println("B");
    return "B";
  }
}

/////////////////////////////////////////////////////
void forward() {
  digitalWrite(INB_leftfront, LOW);
  digitalWrite(EN1_leftfront, HIGH);

  digitalWrite(INB_leftback, LOW);
  digitalWrite(EN2_leftback, HIGH);

  digitalWrite(INB_rightfront, HIGH);
  digitalWrite(EN1_rightfront, HIGH);

  digitalWrite(INB_rightback, LOW);
  digitalWrite(EN2_rightback, HIGH);

}
int turnSpeed = 60;
void reverse() {
  digitalWrite(INB_leftfront, HIGH);
  digitalWrite(EN1_leftfront, HIGH);

  digitalWrite(INB_leftback, HIGH);
  digitalWrite(EN2_leftback, HIGH);

  digitalWrite(INB_rightfront, LOW);
  digitalWrite(EN1_rightfront, HIGH);

  digitalWrite(INB_rightback, HIGH);
  digitalWrite(EN2_rightback, HIGH);
}

void right() {
  digitalWrite(INB_leftfront, HIGH);
  digitalWrite(EN1_leftfront, HIGH);

  digitalWrite(INB_leftback, LOW);
  digitalWrite(EN2_leftback, HIGH);

  digitalWrite(INB_rightfront, LOW);
  digitalWrite(EN1_rightfront, HIGH);

  digitalWrite(INB_rightback, LOW);
  digitalWrite(EN2_rightback, HIGH);
}

void left() {
  digitalWrite(INB_leftfront, LOW);
  digitalWrite(EN1_leftfront, HIGH);

  digitalWrite(INB_leftback, HIGH);
  digitalWrite(EN2_leftback, HIGH);

  digitalWrite(INB_rightfront, HIGH);
  digitalWrite(EN1_rightfront, HIGH);

  digitalWrite(INB_rightback, HIGH);
  digitalWrite(EN2_rightback, HIGH);
}



void turnleft_90() {
  digitalWrite(INB_leftfront, LOW);
  digitalWrite(EN1_leftfront, HIGH);

  digitalWrite(INB_leftback, LOW);
  digitalWrite(EN2_leftback, HIGH);

  digitalWrite(INB_rightfront, HIGH);
  digitalWrite(EN1_rightfront, HIGH);

  digitalWrite(INB_rightback, HIGH);
  digitalWrite(EN2_rightback, HIGH);

  while (Position_left < 70 || Position_right < 70) {
    analogWrite(INA_leftfront, turnSpeed);
    analogWrite(INA_leftback, turnSpeed);
    analogWrite(INA_rightfront, turnSpeed);
    analogWrite(INA_rightfront, turnSpeed);
  }
  while (Position_left < 90 || Position_right < 90) {
    analogWrite(INA_leftfront, turnSpeed - 30); // turnSpeed-30 mean slow speed after 70 degrees
    analogWrite(INA_leftback, turnSpeed - 30);
    analogWrite(INA_rightfront, turnSpeed - 30);
    analogWrite(INA_rightfront, turnSpeed - 30);
  }
  stopbot();
  delay(500);
}

void stopbot() {
  digitalWrite(EN1_leftfront, LOW);
  digitalWrite(EN2_leftback, LOW);
  digitalWrite(EN1_rightfront, LOW);
  digitalWrite(EN2_rightback, LOW);
}

void checkEncoder_left()
{
  encoderPin2Value_left = digitalRead(encoderPin2_left);
  if (encoderPin2Value_left == 1)
  {
    Position_left++;
  }
  else
  {
    Position_left--;
  }
}

void checkEncoder_right()
{
  encoderPin2Value_right = digitalRead(encoderPin2_right);

  if (encoderPin2Value_right == 1)
  {
    Position_right++;
  }
  else
  {
    Position_right--;
  }
}

void wall_following() {
  float wall_threshold = 10.00;
  while (true) {

    digitalWrite(trig_front, LOW);
    delay(2);
    digitalWrite(trig_front, HIGH);
    delay(10);
    digitalWrite(trig_front, LOW);
    long duration_front = pulseIn(echo_front, HIGH);

    digitalWrite(trig_back, LOW);
    delay(2);
    digitalWrite(trig_back, HIGH);
    delay(10);
    digitalWrite(trig_back, LOW);
    long duration_back = pulseIn(echo_back, HIGH);

    /*   float right_ir_value = analogRead(ir_right);
       float mid_ir_value = analogRead(ir_mid);
       float left_ir_value = analogRead(ir_left);

       float right_ir_distance = 27.86 / (right_ir_value - 0.16);
       float mid_ir_distance = 27.86 / (mid_ir_value - 0.16);
       float left_ir_distance = 27.86 / (left_ir_value - 0.16);*/


    float distance_front = duration_front * 0.034 / 2;
    float distance_back = duration_back * 0.034 / 2;

    float front_error = distance_front - wall_threshold;
    float back_error = distance_back - wall_threshold;

    //  Serial.print(front_error);

    float front_pid = wall_pid(front_error);
    float back_pid = wall_pid(back_error);
    /*  Serial.print("\t");
       Serial.print(front_pid);
       Serial.print("\t");*/
    motor_control(front_pid);


    /*
        Serial.print("Distance_front: ");
        Serial.print(distance_front);
        Serial.print(" cm     ");*/
    /*
          Serial.print("Distance_back: ");
          Serial.print(distance_back);
          Serial.println(" cm");


          /*
          Serial.print("IR_right: ");
          Serial.print(right_ir_distance);
          Serial.print(" cm     ");

          Serial.print("IR_mid: ");
          Serial.print(mid_ir_distance);
          Serial.print(" cm     ");

          Serial.print("IR_left: ");
          Serial.print(left_ir_distance);
          Serial.print(" cm     ");*/

    delay(2);
  }
}

float Kp_wall = 12;
float Ki_wall = 0;
float Kd_wall = 4;

float P_wall = 0, I_wall = 0, D_wall = 0, PID_value_wall = 0;
float previous_error_wall = 0, previous_I_wall = 0;

//PID function
float wall_pid(float error_wall) {

  P_wall = error_wall;
  I_wall = I_wall + previous_I_wall;
  D_wall = error_wall - previous_error_wall;

  PID_value_wall = (Kp_wall * P_wall) + (Ki_wall * I_wall) + (Kd_wall * D_wall);

  previous_I_wall = I_wall;
  previous_error_wall = error_wall;

  return PID_value_wall;
}
