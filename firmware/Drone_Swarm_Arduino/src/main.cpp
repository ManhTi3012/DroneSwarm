//#include <Arduino.h>

/// telemetry stuff
#include "esp_now_comm.h"
#include "telemetry.h"
/// sensors
#include "Wire.h"
#include "BNO055_support.h"
/// algorythm
#include "constant.h"
#include "PID.h"
#include "motor_control.h"

#define IDLE_THRUST 0

PID pitch_pid;
PID roll_pid;
Motor motor_fl, motor_fr, motor_bl, motor_br;
int pwm_FL, pwm_FR, pwm_BL, pwm_BR, thrust;
double pitch_tilt = 0.00, roll_tilt = 0.00;

///  imu data
double gyro_x,gyro_y,gyro_z;
double accel_x,accel_y,accel_z;
struct bno055_t myBNO;
struct bno055_euler myEulerData;
struct bno055_accel accelData;
struct bno055_gyro gyroData;

/// low pass filter
double pitch, roll;


EspNowComm telemetry; 

const uint8_t receiver_mac[] = { 0xE4, 0x65, 0xB8, 0x51, 0xDE, 0x9C };
// joy pad 0xA8, 0x42, 0xE3, 0xC6, 0x38, 0xF4 
// controller  0xE4, 0x65, 0xB8, 0x51, 0xDE, 0x9C 


unsigned long long now = 0;
unsigned long long dirLoopTimer = 0;
unsigned long long pidLoopTimer = 0;
unsigned long long lastReceiveTime = 0;
unsigned long long microsNow = 0;

unsigned long loopPeriod = 5; // ms
unsigned long lastLoop = millis();

TaskHandle_t Task1;

void Task1code( void * parameter);
void getImuData();

void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) {
  lastReceiveTime = millis();
  memcpy(&drone_command_, incomingData, sizeof(drone_command_));
  thrust = drone_command_.rc_input[0];
  pitch_tilt = map(drone_command_.rc_input[1], 0, 4096, -20.00, 20.00);
  roll_tilt = map(drone_command_.rc_input[2], 0, 4096, -20.00, 20.00);
}

void setup() {
  pitch_pid.setGains(5.0, 0.0, 0.8);
  pitch_pid.setOutputLimits(-800, 800);

  roll_pid.setGains(5.0, 0.0,  0.8);
  roll_pid.setOutputLimits(-800, 800);

  telemetry.init();
  telemetry.add_peer(receiver_mac);

  pinMode(LED1,OUTPUT);
  pinMode(LED2,OUTPUT);
  pinMode(LED3,OUTPUT);

  pinMode(BATTERY_PIN,INPUT);

  motor_fl.init(0,MOTOR_FL,PWM_FREQ,PWM_RES);
  motor_fr.init(1,MOTOR_FR,PWM_FREQ,PWM_RES);
  motor_bl.init(2,MOTOR_BL,PWM_FREQ,PWM_RES);
  motor_br.init(3,MOTOR_BR,PWM_FREQ,PWM_RES);

  Wire.begin();
  Wire.setClock(400000);

  Serial.begin(115200);

  digitalWrite(LED1,HIGH);
  delay(2000);
  digitalWrite(LED2,HIGH);

  BNO_Init(&myBNO); //Assigning the structure to hold information about the device

  //Configuration to NDoF mode
  bno055_set_operation_mode(OPERATION_MODE_NDOF);

  delay(1);

  digitalWrite(LED3,HIGH);
  
  esp_now_register_recv_cb(esp_now_recv_cb_t(OnDataRecv));

  xTaskCreatePinnedToCore(
      Task1code, /* Function to implement the task */
      "Task1", /* Name of the task */
      10000,  /* Stack size in words */
      NULL,  /* Task input parameter */
      0,  /* Priority of the task */
      &Task1,  /* Task handle. */
      0); /* Core where the task should run */
  delay(1000);
  digitalWrite(LED2,LOW);
  digitalWrite(LED3,LOW);

}


void getImuData(){
  bno055_read_euler_hrp(&myEulerData);	
  bno055_read_accel_xyz(&accelData);
  bno055_read_gyro_xyz(&gyroData);
  gyro_x = double(gyroData.x) / 16.00;
  gyro_y = double(gyroData.y) / 16.00;
  gyro_z = double(gyroData.z) / 16.00;
  accel_x= double(accelData.x)/ 16.00;
  accel_y= double(accelData.y)/ 16.00;
  accel_z= double(accelData.z)/ 16.00;

}

void Task1code( void * parameter) {
  for(;;) {
    update_data();
    telemetry.send_data(receiver_mac, drone_status_);
    //Serial.println(drone_status_.battery);
    delay(20);
    // Serial.print("task1() running on core ");
    // Serial.println(xPortGetCoreID());
    // Serial.print("Pitch: ");
    // Serial.print(kalAngleX);
    // Serial.print("Roll: ");
    // Serial.println(kalAngleY);
    // delay(5);
  }
}

void loop() {
  unsigned long long current = millis();
  if (current - dirLoopTimer >= 10) {
    getImuData();   
    roll = double(myEulerData.r) / 16.00;
    pitch = double(myEulerData.p) / 16.00;
    dirLoopTimer = millis();
  }

  if(current - pidLoopTimer >= 20){
    double dt = 0.02; //(double)(micros() - microsNow) / 1000000; // Calculate delta time
    //roll and pitch are swapped
    float pitch_pwm = pitch_pid.calculate(pitch_tilt, roll, dt);
    float roll_pwm = roll_pid.calculate(roll_tilt, pitch, dt);

    pwm_FL = constrain(thrust - pitch_pwm + roll_pwm + IDLE_THRUST, 0, 1023);
    pwm_FR = constrain(thrust - pitch_pwm - roll_pwm + IDLE_THRUST, 0, 1023);
    pwm_BL = constrain(thrust + pitch_pwm + roll_pwm + IDLE_THRUST, 0, 1023);
    pwm_BR = constrain(thrust + pitch_pwm - roll_pwm + IDLE_THRUST, 0, 1023);

    // if(current - lastReceiveTime > 2000){
    //   pwm_FL = 0;
    //   pwm_FR = 0;
    //   pwm_BL = 0;
    //   pwm_BR = 0;
    // }
    motor_fl.setOutput(pwm_FL);
    motor_fr.setOutput(pwm_FR);
    motor_bl.setOutput(pwm_BL);
    motor_br.setOutput(pwm_BR);

    pidLoopTimer = millis();
  }
}