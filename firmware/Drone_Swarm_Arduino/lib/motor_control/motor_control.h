/*
 * motor_control.h
 *
 *  Created on: May 29, 2025
 *      Author: Ti Manh
 */

#pragma once
#include <cstdint>

class Motor{
public:
	// Khởi tạo đối tượng
	void init(int channel, int pin, int freq, int res);
	// Thiết lập tốc độ tăng độ rộng của PWM
	void setRampSpeed(int ramp_speed);
	// Thiết lập đầu ra (không dùng Ramp)
	void setOutput(int pwm);
	// Thiết lập đầu ra (sử dụng Ramp)
	void setOutput(int pwm, float dt);

private:
	unsigned int channel_;
	unsigned int pwm_res_;
	int ramp_speed_ = 8000;
	int current_pwm = 0;
	// Tính toán giá trị xung (khi dùng ở chế độ Ramp)
	float RampPwm(float current_pwm_, float target_pwm_, float dt);
};
