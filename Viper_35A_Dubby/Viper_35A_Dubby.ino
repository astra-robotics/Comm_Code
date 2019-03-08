#include <EnableInterrupt.h>
#include <Servo.h>

#define SERIAL_PORT_SPEED 57600
#define RC_NUM_CHANNELS  4

#define RC_CH1  0
#define RC_CH2  1
#define RC_CH3  2
#define RC_CH4  3

#define RC_CH1_INPUT  A0
#define RC_CH2_INPUT  A1
#define RC_CH3_INPUT  A2
#define RC_CH4_INPUT  A3

#define DRIVE1 6
#define DRIVE2 9
#define DRIVE3 5
#define DRIVE4 10

#define CH1_H 1980
#define CH1_L 1184

#define CH2_H 1744
#define CH2_L 1066

#define CH3_H 1735
#define CH3_L 1090

uint16_t rc_values[RC_NUM_CHANNELS];
uint32_t rc_start[RC_NUM_CHANNELS];
volatile uint16_t rc_shared[RC_NUM_CHANNELS];

struct MotorDrive {
  float FL, FR, RL, RR;
};

Servo driveServo1;
Servo driveServo2;
Servo driveServo3;
Servo driveServo4;

float rc_n[4];
int pwmf=255;
int pwm=127;
int i;

void rc_read_values() {
  noInterrupts();
  memcpy(rc_values, (const void *)rc_shared, sizeof(rc_shared));
  interrupts();
}

void calc_input(uint8_t channel, uint8_t input_pin) {
  if (digitalRead(input_pin) == HIGH) {
    rc_start[channel] = micros();
  } else {
    uint16_t rc_compare = (uint16_t)(micros() - rc_start[channel]);
    rc_shared[channel] = rc_compare;
  }
}

void calc_ch1() { calc_input(RC_CH1, RC_CH1_INPUT); }
void calc_ch2() { calc_input(RC_CH2, RC_CH2_INPUT); }
void calc_ch3() { calc_input(RC_CH3, RC_CH3_INPUT); }
void calc_ch4() { calc_input(RC_CH4, RC_CH4_INPUT); }

MotorDrive multiply(float val, MotorDrive d) {
  d.FL = d.FL * val;
  d.FR = d.FR * val;
  d.RL = d.RL * val;
  d.RR = d.RR * val;

  return d;
}

MotorDrive addd(MotorDrive d1, MotorDrive d2) {
  MotorDrive md;
  
  md.FL = d1.FL + d2.FL;
  md.FR = d1.FR + d2.FR;
  md.RL = d1.RL + d2.RL;
  md.RR = d1.RR + d2.RR;

  return md;
}

bool checkValue(float value) {
  if (value >= -1 && value <= 1) {
    return true;
  }
  return false;
}

float viper_val(float val) {

  val = 1500 + (val * 500);
  
  if (val > 1450 && val < 1550) val = 1500;
  
  return val;
}

MotorDrive motor_control(float throttle, float steer) {

  MotorDrive output;
  
  MotorDrive FORWARD = { 1, 0.9, 0.92, 0.9} ;
  MotorDrive LEFT = { -1, 1, -1, 1 };
  MotorDrive RIGHT = { 1, -1, 1, -1 };

  if (!checkValue(throttle)) {
    Serial.print("Wrong throttle value");
    output.FL = 1500;
    output.FR = 1500;
    output.RL = 1500;
    output.RR = 1500;
    return output;
  }

  if (!checkValue(steer)) {
    Serial.print("Wrong steer value");
    output.FL = 1500;
    output.FR = 1500;
    output.RL = 1500;
    output.RR = 1500;
    return output;
  }

  if (steer < 0) {
    output = addd(multiply(throttle, FORWARD) , multiply(-steer, LEFT));
  }
  else {
    output = addd(multiply(throttle, FORWARD) , multiply(steer, RIGHT));
  }

  output.FL = viper_val(output.FL);
  output.FR = viper_val(output.FR);
  output.RL = viper_val(output.RL);
  output.RR = viper_val(output.RR);
//
//  Serial.println(output.FL);
//  Serial.println(output.FR);
//  Serial.println(output.RL);
//  Serial.println(output.RR);


  return output;
}

void setup() {
  Serial.begin(SERIAL_PORT_SPEED);

  pinMode(RC_CH1_INPUT, INPUT);
  pinMode(RC_CH2_INPUT, INPUT);
  pinMode(RC_CH3_INPUT, INPUT);
  pinMode(RC_CH4_INPUT, INPUT);
  driveServo1.attach(DRIVE1,1000,2000);
  driveServo2.attach(DRIVE2,1000,2000);
  driveServo3.attach(DRIVE3,1000,2000);
  driveServo4.attach(DRIVE4,1000,2000);

  enableInterrupt(RC_CH1_INPUT, calc_ch1, CHANGE);
  enableInterrupt(RC_CH2_INPUT, calc_ch2, CHANGE);
  enableInterrupt(RC_CH3_INPUT, calc_ch3, CHANGE);
  enableInterrupt(RC_CH4_INPUT, calc_ch4, CHANGE);
}


void loop() {
  rc_read_values();

  Serial.print("\n\n");
  Serial.print("CH1:"); Serial.print(rc_values[RC_CH1]); Serial.print("\t");
  Serial.print("CH2:"); Serial.print(rc_values[RC_CH2]); Serial.print("\t");
  Serial.print("CH3:"); Serial.print(rc_values[RC_CH3]); Serial.print("\t");
  Serial.print("CH4:"); Serial.println(rc_values[RC_CH4]); Serial.print("\t");

  float scale = ((float)rc_values[RC_CH2] - CH2_L) / (CH2_H - CH2_L);

  float CH3_M = CH3_L + (CH3_H - CH3_L)/2;
  float CH1_M = CH1_L + (CH1_H - CH1_L)/2;

  
  float throttle = ((float)rc_values[RC_CH3] - CH3_M) * 2 * scale / (CH3_H - CH3_L);
  float steer = -((float)rc_values[RC_CH1] - CH1_M) * 2 * scale / (CH1_H - CH1_L);

//  Serial.print("\n\nSteer:"); Serial.println(steer);
//  Serial.println(scale);

  MotorDrive d = motor_control(throttle, steer);

  Serial.print("\n");
  Serial.print("M1:"); Serial.print(d.FL); Serial.print("\t");
  Serial.print("M2:"); Serial.print(d.FR); Serial.print("\t");
  Serial.print("M3:"); Serial.print(d.RL); Serial.print("\t");
  Serial.print("M4:"); Serial.print(d.RR); Serial.print("\t");

  driveServo1.writeMicroseconds(d.FL);
  driveServo2.writeMicroseconds(d.FR);
  driveServo3.writeMicroseconds(d.RL);
  driveServo4.writeMicroseconds(d.RR);

  delay(100);
//  commondrive(rc_values[RC_CH2],rc_values[RC_CH1],rc_values[RC_CH3]);
}
