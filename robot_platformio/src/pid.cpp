#include "pid.h"

namespace diffrobot {
    void Pid::reset(int encoder_count) {
        setpoint_ = 0;
        integral_term_ = 0;
        last_enconder_count_ = encoder_count;
        last_input_ = 0;
        last_output_ = 0;
    }

    void Pid::enable() { enabled_ = true; }

    bool Pid::enabled() { return enabled_; }

    void Pid::disable() { enabled_ = false; }

    void Pid::compute( int encoder_count, int& computed_output) {
        if (!enabled_) {
            if(last_input_ != 0) {
                reset(encoder_count);
            }
            return;
        }

        int input = encoder_count - last_enconder_count_;
        long error = setpoint_ - input;

        long output = (kp_ * error - kd_ * (input - last_input_) + integral_term_) /ko_;
        output += last_output_;

        if (output >= output_max_) {
            output = output_max_;
        } else if (output <= output_min_) {
            output = output_min_;
        } else {
            integral_term_ += ki_ * error;
        }

        computed_output = output;

        last_enconder_count_ = encoder_count;
        last_input_ = input;
        last_output_ = output;
    }

    void Pid::set_setpoint(int setpoint) { setpoint_ = setpoint; }

    void Pid::set_tunings(int kp, int kd, int ki, int ko) {
        kp_ = kp;
        kd_ = kd;
        ki_ = ki;
        ko_ = ko;
    }
}