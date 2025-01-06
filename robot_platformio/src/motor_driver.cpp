#include <Arduino.h>
#include "motor_driver.h"

namespace diffrobot {
    MotorDriver::MotorDriver(const int& pwm,
                             const int& forward,
                             const int&backward): pwm_pin_{pwm},
                                                  forward_pin_{forward},
                                                  backward_pin_{backward} {}
    void MotorDriver::begin() {
        pinMode(pwm_pin_, OUTPUT);
        pinMode(forward_pin_, OUTPUT);
        pinMode(backward_pin_, OUTPUT);
    }

    void MotorDriver::set_speed(int speed) {
        bool forward = true;
        
        if (speed > kMaxSpeed) {
            speed = kMaxSpeed;
        }

        if (speed < kMinSpeed) {
            speed = -speed;
            forward = false;
        }

        if (forward) {
            digitalWrite(backward_pin_, HIGH);
            digitalWrite(forward_pin_, LOW);
        } else {
            digitalWrite(backward_pin_, LOW);
            digitalWrite(forward_pin_, HIGH);
        }
        
        analogWrite(pwm_pin_, speed);
    }
}