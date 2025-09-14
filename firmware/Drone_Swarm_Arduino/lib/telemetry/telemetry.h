/*
 * telemetry.h
 *
 *  Created on: May 30, 2025
 *      Author: Ti Manh
 */

#pragma once

#include <Arduino.h>

#define BATTERY_PIN 34

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
    FlightMode flight_mode; // set flight mode
    int16_t rc_input[4];    // throttle, yaw, pitch, roll        
} drone_command_;

extern int pwm_FL;
extern int pwm_FR;
extern int pwm_BL;
extern int pwm_BR;

extern double gyro_x,gyro_y,gyro_z;
extern double accel_x,accel_y,accel_z;
extern double pitch, roll;

void update_data(){
    drone_status_.battery = analogRead(BATTERY_PIN);
    drone_status_.motor_pwm[0] = pwm_FL;
    drone_status_.motor_pwm[1] = pwm_FR;
    drone_status_.motor_pwm[2] = pwm_BL;
    drone_status_.motor_pwm[3] = pwm_BR;

    drone_status_.accel[0] = accel_x;
    drone_status_.accel[1] = accel_y;
    drone_status_.accel[2] = accel_z;

    drone_status_.gyro[0] = gyro_x;    
    drone_status_.gyro[1] = gyro_y;    
    drone_status_.gyro[2] = gyro_z;    

    drone_status_.direction[0] = roll;
    drone_status_.direction[1] = pitch;

}