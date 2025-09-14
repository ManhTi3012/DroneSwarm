/*
 * motor_control.cc
 *
 *  Created on: May 29, 2025
 *      Author: Ti Manh
 */

#include "motor_control.h"
#include <esp32-hal-ledc.h>

void Motor::init(int channel, int pin, int freq, int res){
    ledcSetup(channel, freq, res);
    ledcAttachPin(pin, channel);   
    channel_ = channel;
    pwm_res_ = res;
}

void Motor::setRampSpeed(int ramp_speed){
    ramp_speed_ = ramp_speed;
}

float Motor::RampPwm(float current_pwm_, float target_pwm_, float dt){
	float delta = (float) ramp_speed_ * dt;
    float output;
	if(current_pwm_ < target_pwm_){
		if(current_pwm_ + delta > target_pwm_){output = target_pwm_;}
		else{output = current_pwm_ + delta;}
	}
	else if (current_pwm_ > target_pwm_){
		if(current_pwm_ - delta < target_pwm_){output = target_pwm_ ;}
		else{output = current_pwm_ - delta;}
	}
    return output;
}

void Motor::setOutput(int pwm){
    current_pwm = pwm;
    ledcWrite(channel_,current_pwm);
}

void Motor::setOutput(int pwm, float dt){
    current_pwm = RampPwm(current_pwm, pwm, dt);
    ledcWrite(channel_,current_pwm);
}
