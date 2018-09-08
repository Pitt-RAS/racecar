#include "PWM.h"
#include <math.h>

PWM::PWM(int pin) {
    pwm_.attach(pin);
    Set(0);
    ConfigLimit(1);
}

void PWM::ConfigLimit(double limit) {
    limit_ = limit;
}

void PWM::Set(double speed) {
    if ( fabs(speed) > 1 )
        speed = copysign(1, speed);
    speed *= limit_;
    speed *= 500;
    speed += 1500;

    pwm_.writeMicroseconds((int)speed);
}

