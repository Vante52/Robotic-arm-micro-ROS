#pragma once

namespace diffrobot {

struct Constants
{
    static const int kPidRate{30};
    const double kPidPeriod{1000 / kPidRate};
    static const int kPidKp{30};
    static const int kPidKd{10};
    static const int kPidKi{0};
    static const int kPidKo{10};
    static const int kPwmMin{0};
    static const int kPwmMax{255};
};

}