/*
 * PID.h
 *
 *  Created on: May 26, 2025
 *      Author: Ti Manh
 */

#pragma once

class PID {
public:
    // Tính toán giá trị PID trả về
    double calculate(double setpoint, double actual, double dt);
    // Thiết lập các thông số P, I, D
    void setGains(double set_Kp, double set_Ki, double set_Kd);
    // Thiết lập giới hạn của PID
    void setOutputLimits(double lowerLimit, double upperLimit);

private:
    double Kp, Ki, Kd;
    double integral_ = 0;
    double prev_error_ = 0;
    double prev_actual = 0;
    double output_lim_[2] = {-1e6, 1e6}; // Default: no limits
};
