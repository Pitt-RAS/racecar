#include "PWM.h"
#include <math.h>

PWM::PWM(int pin) {
    pwm_.attach(pin);
    ConfigLimit(1);
    ConfigOffset(0);
    Set(0);
}

void PWM::ConfigLimit(double limit) {
    limit_ = limit;
}

void PWM::ConfigOffset(double offset) {
    offset_ = offset;
}

void PWM::Set(double speed) {
    speed += offset_;
    if ( fabs(speed) > 1 )
        speed = copysign(1, speed);
    speed *= limit_;
    speed *= 500;
    speed += 1500;

    pwm_.writeMicroseconds((int)speed);
}

