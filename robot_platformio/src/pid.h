#pragma once
#include <Arduino.h>

namespace diffrobot {
    class Pid
    {
    private:
        int kp_{0};
        int kd_{0};
        int ki_{0};
        int ko_{0};
        int output_min_{0};
        int output_max_{0};
        bool enabled_{false};
        int setpoint_{0};
        int integral_term_{0};
        long last_enconder_count_{0};
        int last_input_{0};
        long last_output_{0};
    public:
        Pid(int kp, int kd, int ki, int ko, int output_min, int output_max)
            :kp_{kp}, kd_{kd}, ki_{ki}, ko_{ko}, output_min_{output_min}, output_max_{output_max} {}
        
        void reset(int encoder_count);

        void enable();

        bool enabled();

        void disable();

        void compute(int encoder_count, int& computed_output);

        void set_setpoint(int setpoint);

        void set_tunings(int kp, int kd, int ki, int ko);
    };
}