#include "encoder_driver.h"

namespace diffrobot {

    EncoderDriver::EncoderDriver(const int& encA, const int& encB): encA_{encA},
                                                                    encB_{encB} {}
    
    void EncoderDriver::begin() {
        pinMode(encA_, INPUT);
        pinMode(encB_, INPUT);
    }

    void IRAM_ATTR EncoderDriver::readEncoder() {
        ant_ = act_;

        if(digitalRead(encA_)) bitSet(act_, 1); else bitClear(act_, 1);
        if(digitalRead(encB_)) bitSet(act_, 0); else bitClear(act_, 0);

        if(ant_ == 2 && act_ == 0) pos_i--;
        if(ant_ == 0 && act_ == 1) pos_i--;
        if(ant_ == 3 && act_ == 2) pos_i--;
        if(ant_ == 1 && act_ == 3) pos_i--;

        if(ant_ == 1 && act_ == 0) pos_i++;
        if(ant_ == 3 && act_ == 1) pos_i++;
        if(ant_ == 0 && act_ == 2) pos_i++;
        if(ant_ == 2 && act_ == 3) pos_i++;
    }

    int EncoderDriver::read() {
        int pos = 0;
        noInterrupts();
        pos = pos_i;
        interrupts();
        return pos;
    }
}