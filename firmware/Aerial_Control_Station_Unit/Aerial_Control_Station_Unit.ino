#include "esp_now_comm.h"

unsigned long long lastReceiveTime = 0;
unsigned long long lastSentTime = 0;

unsigned long long lastDroneCommandTime = 0;

bool isStatusSent = false;
bool isStatusSentOld = false;

const uint8_t drone_mac[] = {0xf8, 0xb3, 0xb7, 0x34, 0xc6, 0x04};
// 1st drone 0xf8, 0xb3, 0xb7, 0x34, 0xb8, 0x88
// 2nd drone 0xf8, 0xb3, 0xb7, 0x34, 0xc6, 0x04
EspNowComm telemetry;

bool connectionState = false;
bool connectionStateOld = false;
// Structure example to receive data
// Must match the sender structure
typedef enum {
    MODE_MANUAL,
    MODE_STABILIZE,
    MODE_ALT_HOLD,
    MODE_AUTO
} FlightMode;

struct DroneStatus{
    float gyro[3];          // raw x, y, z
    float accel[3];         // raw x, y, z
    float direction[2];     // filtered pitch/roll
    uint16_t battery;       // 0-4095
    uint16_t motor_pwm[4];  // 0-1023
    FlightMode flight_mode; // current flight mode
    bool failsafe;          // true if in failsafe
} drone_status_;

struct DroneCommand{
    FlightMode flight_mode;// set flight mode
    int16_t rc_input[4];  // throttle, yaw, pitch, roll        
} drone_command_;

float pitch;
float roll;

// callback function that will be executed when data is received
void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) {
  lastReceiveTime = millis();
  memcpy(&drone_status_, incomingData, sizeof(drone_status_));

  Serial.print("PWM:");
  Serial.print(drone_status_.motor_pwm[0]);
  Serial.print(',');
  Serial.print(drone_status_.motor_pwm[1]);
  Serial.print(',');
  Serial.print(drone_status_.motor_pwm[2]);
  Serial.print(',');
  Serial.println(drone_status_.motor_pwm[3]);

  Serial.print("BAT:");
  Serial.println(drone_status_.battery);

  Serial.print("ACC:");
  Serial.print(drone_status_.accel[0]);
  Serial.print(',');
  Serial.print(drone_status_.accel[1]);
  Serial.print(',');
  Serial.println(drone_status_.accel[2]);

  Serial.print("GYR:");
  Serial.print(drone_status_.gyro[0]);
  Serial.print(',');
  Serial.print(drone_status_.gyro[1]);
  Serial.print(',');
  Serial.println(drone_status_.gyro[2]);

  Serial.print("DIR:");
  Serial.print(drone_status_.direction[0]);
  Serial.print(',');
  Serial.println(drone_status_.direction[1]); 

  Serial.print("THR:");
  Serial.println(constrain(map(analogRead(32), 2000, 4096, 0, 1023), 0, 1023));
}
 
void setup() {
  // Initialize Serial Monitor
  Serial.begin(115200);
  
  // Set device as a Wi-Fi Station
  telemetry.init();
  telemetry.add_peer(drone_mac);

  pinMode(32,INPUT);

  // Init ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }
  
  // Once ESPNow is successfully Init, we will register for recv CB to
  // get recv packer info
  esp_now_register_recv_cb(esp_now_recv_cb_t(OnDataRecv));
}
 
void loop() {
  if(millis() - lastReceiveTime > 200){
    connectionState = false;
  }
  else{
    connectionState = true;
  }

  if(connectionState != connectionStateOld){
    if(connectionState){
      Serial.println("OK:");
    }
    else{
      Serial.println("NO:");
    }
    connectionStateOld = connectionState;
  }

  if(millis() - lastDroneCommandTime >= 10){
    
    drone_command_.rc_input[0] = constrain(map(analogRead(32), 2000, 4096, 0, 1023), 0, 1023);
    drone_command_.rc_input[1] = analogRead(34);
    drone_command_.rc_input[2] = analogRead(35);
    telemetry.send_data(drone_mac, drone_command_);
    lastDroneCommandTime = millis();
  }

}