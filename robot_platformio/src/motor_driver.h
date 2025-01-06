#pragma once 

namespace diffrobot {

    class MotorDriver
    {
    private:
        static constexpr int kMinSpeed{0};
        static constexpr int kMaxSpeed{255};
        int pwm_pin_{0};
        int forward_pin_{0};
        int backward_pin_{0};
    public:
        MotorDriver(const int& pwm, const int& forward, const int&backward);
        void begin();
        void set_speed(int speed);
    };
}