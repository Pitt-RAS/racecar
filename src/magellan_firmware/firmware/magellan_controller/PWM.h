#ifndef PWM_H
#define PWM_H

#include <Servo.h>

class PWM {
public:
    PWM(int pin);
    void Set(double speed);
    void ConfigLimit(double limit);
    void ConfigOffset(double offset);
private:
    Servo pwm_;
    double limit_;
    double offset_;
};

#endif
