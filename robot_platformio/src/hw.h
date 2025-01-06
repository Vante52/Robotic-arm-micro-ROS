namespace diffrobot {

struct Hw
{
    // Left encoder channel A pin
    static const unsigned int kLeftEncoderChannelAGpioPin{15};
    // Left encoder channel B pin
    static const unsigned int kLeftEncoderChannelBGpioPin{2};

    // Right encoder channel A pin
    static const unsigned int kRightEncoderChannelAGpioPin{12};
    // Rigth encoder channel B pin
    static const unsigned int kRightEncoderChannelBGpioPin{14};

    // Left motor driver backward pin
    static const unsigned int kLeftMotorBackwardGpioPin{19};
    // Left motor driver fordward pin
    static const unsigned int kLeftMotorForwardGpioPin{18};

    // Right motor driver backward pin
    static const unsigned int kRightMotorBackwardGpioPin{22};
    // Right motor driver fordward pin
    static const unsigned int kRightMotorForwardGpioPin{23};

    // Enable input for left motor pin
    static const unsigned int kLeftMotorEnableGpioPin{32};
    // Enable input for right motor pin
    static const unsigned int kRightMotorEnableGpioPin{33};
};

}