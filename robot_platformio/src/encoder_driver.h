#pragma once
#include <Arduino.h>

namespace diffrobot {

    class EncoderDriver
    {
    private:
        int encA_{0};
        int encB_{0};
        volatile int pos_i{0};
        volatile byte ant_{0};
        volatile byte act_{0};
    public:
        EncoderDriver(const int& encA, const int& encB);

        void begin();
        void IRAM_ATTR readEncoder();
        int read();
    };   
}