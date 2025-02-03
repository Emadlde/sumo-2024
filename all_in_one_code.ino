int button_state1;
int button_state2;



// Define pin numbers
#include <QTRSensors.h>
#include <SoftwareSerial.h>

const int YELLOW_IR_LEFT = 44;
const int YELLOW_IR_MID = 48;
const int YELLOW_IR_RIGHT = 40;

int RED_SENSOR = 26;
int RED_SENSOR_ARRAY = A0;
int Value_for_red_sensor = 0;
int Value_RED_SENSOR_ARRAY = 0;
// Motor speeds
int right_motor_speed;
int left_motor_speed;

int low_speed = 120;
int mid_speed = 150;
int high_speed = 175;
int very_high_speed = 200;

int right_backward = 3;
int right_forward = 2;

int left_backward = 7;
int left_forward = 6;




int trig = 36;
int echo = 34;

int speedA = 145;
long long timing;
long long startingTime;


#define NUM_SENSORS 8  // number of sensors used

QTRSensors qtra;

const uint8_t SensorCount = 8;
uint16_t sensorValues[SensorCount];

float Kp = 0.08;
//change the value by trial-and-error (ex: 0.07).
float Ki = 0.0001;
//change the value by trial-and-error (ex: 0.0008).
float Kd = 0.4;
//change the value by trial-and-error (ex: 0.6).
int P;
int I;
int D;

#define LEFT_MOTOR_DEFAULT_SPEED 200
#define RIGHT_MOTOR_DEFAULT_SPEED 220  //motor 2 is the right one
#define LEFT_MOTOR_MAX_SPEED 220
#define RIGHT_MOTOR_MAX_SPEED 240

int lastError = 0;
int last_proportional = 0;
int integral = 0;
int move = 1;  ///////////put one to make the  robot move ///////////READ MEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEE/////
long sum = 0;

int i = 0;
int c = 0;
long bestsum = 1000000000;
int n = 1;
SoftwareSerial mySerial(18, 19);  // where is the RX connected to , where is the TXconnected to

int arr[8] = { 0, 0, 0, 0, 0, 0, 0, 0 };



void setup() {
  pinMode(A8, INPUT);
  pinMode(A9, INPUT);


  pinMode(YELLOW_IR_LEFT, INPUT);
  pinMode(YELLOW_IR_MID, INPUT);
  pinMode(YELLOW_IR_RIGHT, INPUT);


  pinMode(RED_SENSOR_ARRAY, INPUT);
  pinMode(RED_SENSOR, INPUT);

  pinMode(right_forward, OUTPUT);
  pinMode(right_backward, OUTPUT);
  pinMode(left_backward, OUTPUT);
  pinMode(left_forward, OUTPUT);
  TCCR4B = TCCR4B & B11111000 | B00000010;  //  3921.16 Hz
  TCCR3B = TCCR3B & B11111000 | B00000010;  //  3921.16 Hz

  pinMode(10, INPUT_PULLUP);
  pinMode(11, INPUT_PULLUP);
  pinMode(trig, OUTPUT);
  pinMode(echo, INPUT);

  qtra.setTypeAnalog();
  qtra.setSensorPins((const uint8_t[]){
                       A7, A6, A5, A4, A3, A2, A1, A0 },
                     SensorCount);

  pinMode(RED_SENSOR_ARRAY, INPUT);
  pinMode(RED_SENSOR, INPUT);

  button_state1 = digitalRead(11);
  button_state2 = digitalRead(10);
  delay(4300);
}


void loop() {

  //111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111
  if (button_state1 == 0 & button_state2 == 0) {
    //side_hit_right

    //rotate to the right abit

    analogWrite(right_forward, 0);
    analogWrite(left_forward, 150);
    analogWrite(right_backward, 150);
    analogWrite(left_backward, 0);
    delay(60);
    start_without_jumping();

    //when you see a white line rotate to the other side
    while (1) {
      Value_for_red_sensor = digitalRead(RED_SENSOR);
      Value_RED_SENSOR_ARRAY = digitalRead(RED_SENSOR_ARRAY);
      if (Value_for_red_sensor == 0 || Value_RED_SENSOR_ARRAY == 0) {
        analogWrite(right_forward, 255);
        analogWrite(left_forward, 0);
        analogWrite(right_backward, 0);
        analogWrite(left_backward, 255);
        delay(250);
        break;
      } else {
        analogWrite(right_forward, 200);
        analogWrite(left_forward, 200);
        analogWrite(right_backward, 0);
        analogWrite(left_backward, 0);
      }
    }
    detection();
  }





  //22222222222222222222222222222222222222222222222222222222222222222222222222222222222222222222222222222222222222222222222222222222222222222222222222222222222222222222222



  if (button_state1 == 1 & button_state2 == 1) {
    //side_hit_left

    //rotate to the left abit

    analogWrite(right_forward, 150);
    analogWrite(left_forward, 0);
    analogWrite(right_backward, 0);
    analogWrite(left_backward, 150);
    delay(75);
    start_without_jumping();

    //when you see a white line rotate to the other side
    while (1) {
      Value_for_red_sensor = digitalRead(RED_SENSOR);
      Value_RED_SENSOR_ARRAY = digitalRead(RED_SENSOR_ARRAY);
      if (Value_for_red_sensor == 0 || Value_RED_SENSOR_ARRAY == 0) {
        analogWrite(right_forward, 0);
        analogWrite(left_forward, 0);
        analogWrite(right_backward, 255);
        analogWrite(left_backward, 0);
        delay(160);
        break;
      } else {
        analogWrite(right_forward, 200);
        analogWrite(left_forward, 200);
        analogWrite(right_backward, 0);
        analogWrite(left_backward, 0);
      }
    }
    detection();
  }



  // 333333333333333333333333333333333333333333333333333333333333333333333333333333333333333333333333333333333333333333333333333333333333333333333333333333333333

  if (button_state1 == 0 & button_state2 == 1) {
    //detect forward
    detection();
  }






  //44444444444444444444444444444444444444444444444444444444444444444444444444444444444444444444444444444444444444444444444444444444444444444444444444444444444444444444444444


  if (button_state1 == 1 & button_state2 == 0) {
    while (sensorValues[7] > 700) {
      qtra.read(sensorValues);

      analogWrite(right_forward, 0);
      analogWrite(left_forward, 200);
      analogWrite(right_backward, 200);
      analogWrite(left_backward, 0);
    }
    analogWrite(right_forward, 0);
    analogWrite(left_forward, 0);
    analogWrite(right_backward, 0);
    analogWrite(left_backward, 0);
    white_line();
  }
}






void white_line() {
  while (1) {

    long startTime = millis();
    long loopDuration = 10000;  // 10 seconds

    while (millis() - startTime < loopDuration) {
      sum += run();
    }

    Serial.print(" ki is: ");
    Serial.println(Ki, 4);

    Serial.print("with the sum of it is: ");
    Serial.println(sum);

    if (sum < bestsum) {
      bestsum = sum;
      Serial.println("AND ITS THE BEST UNTILL NOW !!!");
    }

    Serial.println("--------------------------------");
    sum = 0;
    Ki += 0.000;
  }
}
void detection() {
  while (1) {

    detect_white();
    int distance = getDistance(trig, echo);
    int ir_left = digitalRead(YELLOW_IR_LEFT);
    // int ir_mid = digitalRead(YELLOW_IR_MID);
    int ir_right = digitalRead(YELLOW_IR_RIGHT);
    // Determine direction and set motor speeds based on IR readings
    if (distance > 11) {
      startingTime = millis();
    }
    if (distance <= 11) {
      timing = millis() - startingTime;
      speedA = 145 + (timing);
      if (speedA > 255) {
        speedA = 255;
      }
      attack2(speedA);
    }

    else if (distance <= 60 && ((ir_left == 0 && ir_right == 0) || (ir_left == 1 && ir_right == 1))) {
      forward();
      speedA = 145;

    } else if (ir_left == 0 && distance >= 60 && ir_right == 1) {
      left();
      speedA = 145;

    } else if (ir_left == 0 && distance <= 60 && ir_right == 1) {
      left_alot();
      speedA = 145;

    } else if (ir_left == 1 && distance <= 60 && ir_right == 0) {
      right();
      speedA = 145;
    } else if (ir_left == 1 && distance >= 60 && ir_right == 0) {
      right_alot();
      speedA = 145;

    } else {
      forward();
      speedA = 145;
    }
    // Add code for moving motors here
  }
}


void start_without_jumping() {

  analogWrite(right_forward, 40);
  analogWrite(left_forward, 40);
  analogWrite(right_backward, 0);
  analogWrite(left_backward, 0);
  delay(25);  //15

  analogWrite(right_forward, 115);
  analogWrite(left_forward, 115);
  analogWrite(right_backward, 0);
  analogWrite(left_backward, 0);
  delay(35);

  analogWrite(right_forward, 170);
  analogWrite(left_forward, 170);
  analogWrite(right_backward, 0);
  analogWrite(left_backward, 0);
}


void spin() {
  analogWrite(right_forward, mid_speed + 10);
  analogWrite(left_forward, 0);
  analogWrite(right_backward, 0);
  analogWrite(left_backward, mid_speed);
}
void forward() {
  analogWrite(right_forward, mid_speed + 10);
  analogWrite(left_forward, mid_speed);
  analogWrite(right_backward, 0);
  analogWrite(left_backward, 0);
}

void left() {
  analogWrite(right_forward, high_speed);
  analogWrite(left_forward, low_speed);
  analogWrite(right_backward, 0);
  analogWrite(left_backward, 0);
}

void left_alot() {
  analogWrite(right_forward, very_high_speed);
  analogWrite(left_forward, low_speed - 10);
  analogWrite(right_backward, 0);
  analogWrite(left_backward, 0);
}

void right() {
  analogWrite(right_forward, low_speed);
  analogWrite(left_forward, high_speed);
  analogWrite(right_backward, 0);
  analogWrite(left_backward, 0);
}

void right_alot() {
  analogWrite(right_forward, low_speed);
  analogWrite(left_forward, very_high_speed);
  analogWrite(right_backward, 0);
  analogWrite(left_backward, 0);
}
void stop() {
  analogWrite(right_forward, 0);
  analogWrite(left_forward, 0);
  analogWrite(right_backward, 0);
  analogWrite(left_backward, 0);
}

void detect_white() {
  Value_for_red_sensor = digitalRead(RED_SENSOR);
  Value_RED_SENSOR_ARRAY = digitalRead(RED_SENSOR_ARRAY);

  Serial.println(Value_RED_SENSOR_ARRAY);

  if (Value_RED_SENSOR_ARRAY == 0) {
    while (Value_RED_SENSOR_ARRAY == 0 || Value_for_red_sensor == 0) {
      Value_for_red_sensor = digitalRead(RED_SENSOR);
      Value_RED_SENSOR_ARRAY = digitalRead(RED_SENSOR_ARRAY);
      analogWrite(right_forward, 0);  //change morotr pins
      analogWrite(left_forward, 0);
      analogWrite(right_backward, 255);
      analogWrite(left_backward, 255);
    }
    analogWrite(right_forward, 255);
    analogWrite(left_forward, 0);
    analogWrite(right_backward, 0);
    analogWrite(left_backward, 255);
    delay(120);  ////to edit
    start_without_jumping();
  } else if (Value_for_red_sensor == 0) {
    while (Value_for_red_sensor == 0 || Value_RED_SENSOR_ARRAY == 0) {
      Value_for_red_sensor = digitalRead(RED_SENSOR);
      Value_RED_SENSOR_ARRAY = digitalRead(RED_SENSOR_ARRAY);
      analogWrite(right_forward, 0);  //change morotr pins
      analogWrite(left_forward, 0);
      analogWrite(right_backward, 255);
      analogWrite(left_backward, 255);
    }
    analogWrite(right_forward, 0);
    analogWrite(left_forward, 250);
    analogWrite(right_backward, 250);
    analogWrite(left_backward, 0);
    delay(100);  ////to edit
    start_without_jumping();
  }
}

int run() {



  ///////////////////////////////////////////////////////////////////////READ AND PUT IN sensorValues/////////////////////////////////////////////////////////////////////////////

  //////SWITCH DATA FROM ANALOG TO DIGITAL AND PUT IT IN ARR//////

  qtra.read(sensorValues);
  //for(int i=0;i<=7;i++)
  //{
  //Serial.print(sensorValues[i]);
  //Serial.print(" ");

  //}
  //Serial.println();
  ///////////////////////////////////////////////////////////////////////PID/////////////////////////////////////////////////////////////////////////////
  int position = qtra.readLineWhite(sensorValues);
  int error = position - 7000;
  ////////////////////////TO DO TO DO TO DO TO DO TO DO TO DO TO DO TO DO TO DO TO DO TO DO TO DO TO DO TO DO TO DO TO DO///////////////////////////////////////////////////////
  // find the perfrct pose ising this equation
  //    0*value0 + 1000*value1 + 2000*value2 + ...
  //   --------------------------------------------
  //         value0  +  value1  +  value2 + ...

  P = error;
  I = I + error;
  D = error - lastError;
  lastError = error;
  int motorspeed = P * Kp + I * Ki + D * Kd;

  lastError = error;
  int leftMotorSpeed = LEFT_MOTOR_DEFAULT_SPEED + motorspeed;
  int rightMotorSpeed = RIGHT_MOTOR_DEFAULT_SPEED - motorspeed;


  if (move) {
    set_motors(leftMotorSpeed, rightMotorSpeed);
  }

  return abs(error);
}

void set_motors(int motor4speed, int motor2speed) {
  if (motor4speed > LEFT_MOTOR_MAX_SPEED) motor4speed = LEFT_MOTOR_MAX_SPEED;    // limit top speed
  if (motor2speed > RIGHT_MOTOR_MAX_SPEED) motor2speed = RIGHT_MOTOR_MAX_SPEED;  // limit top speed
  if (motor4speed < 0) motor4speed = 0;                                          // keep motor above 0
  if (motor2speed < 0) motor2speed = 0;                                          // keep motor speed above 0
  analogWrite(right_forward, motor2speed);
  analogWrite(left_forward, motor4speed);
}
void attack2(int x) {
  analogWrite(right_forward, x);
  analogWrite(left_forward, x - 20);
  analogWrite(right_backward, 0);
  analogWrite(left_backward, 0);
}

long getDistance(int trigPin, int echoPin) {
  // Send a 10us pulse to trigger the ultrasonic sensor
  digitalWrite(trigPin, LOW);
  delayMicroseconds(10);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(2);
  digitalWrite(trigPin, LOW);

  // Read the time it takes for the pulse to return
  long duration = pulseIn(echoPin, HIGH);  // Timeout after 30ms to avoid blocking

  // Calculate the distance in cm
  long distance = duration * 0.034 / 2;
  delay(5);
  return distance;
}

void ruler() {
  int upperLDR = analogRead(A8);
  int lowerLDR = analogRead(A9);
  Serial.print(upperLDR);
  Serial.print(" ");
  Serial.println(lowerLDR);
  if (lowerLDR > 150) {
    while (lowerLDR > 150) {
      upperLDR = analogRead(A8);
      lowerLDR = analogRead(A9);
      analogWrite(right_backward, 0);
      analogWrite(left_backward, 250);
      analogWrite(right_forward, 250);
      analogWrite(left_forward, 0);

      /*  analogWrite(right_backward, 60);
        analogWrite(left_backward, 250);
        analogWrite(right_forward, 0);
        analogWrite(left_forward, 0);*/
    }
    analogWrite(right_backward, 250);
    analogWrite(left_backward, 250);
    analogWrite(right_forward, 0);
    analogWrite(left_forward, 0);
    delay(400);
  }
}