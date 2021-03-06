#include "EncoderPublisher.h"
#include <Arduino.h>
#include "config.h"

static bool encoder_isr_init = false;

enum State {A,B,C};
volatile enum State encoder_state;
volatile static long int encoder_total = 0;
static long int encoder_last_total = 0;
static int encoder_delta = 0;

// forwards would go A C B A C B ...
// backwards would go A B C A B C ...
static void isr_A() {
    // if last phase triggered was B, increment; else decrement (because we went backwards)
    if (encoder_state == B) {
        encoder_total++;
    }
    else{
        encoder_total--;
    }

    encoder_state = A;
}

static void isr_B() {
    // if last phase triggered was C, increment; else decrement (backwards)
    if (encoder_state == C) {
        encoder_total++;
    }
    else{
        encoder_total--;
    }

    encoder_state = B;
}

static void isr_C() {
    // if last phase triggered was A, increment; else decrement (backwards)
    if (encoder_state == A) {
        encoder_total++;
    }
    else{
        encoder_total--;
    }

    encoder_state = C;
}

static void SetupEncoderISR() {
    if ( encoder_isr_init )
        return;
    encoder_isr_init = true;

    pinMode(ENCODER_A, INPUT_PULLUP);
    pinMode(ENCODER_B, INPUT_PULLUP);
    pinMode(ENCODER_C, INPUT_PULLUP);

    attachInterrupt(digitalPinToInterrupt(ENCODER_A), isr_A, CHANGE);
    attachInterrupt(digitalPinToInterrupt(ENCODER_B), isr_B, CHANGE);
    attachInterrupt(digitalPinToInterrupt(ENCODER_C), isr_C, CHANGE);
}

EncoderPublisher::EncoderPublisher(ros::NodeHandle& nh) :
        nh_(nh),
        velocity_publisher_("/platform/velocity", &twist_msg_),
        encoder_total_publisher_("/platform/encoder_total", &encoder_total_msg_),
        update_rate_(ENCODER_UPDATE_HZ),
        update_rate_debug_(DEBUG_HZ) {
    SetupEncoderISR();

    nh.advertise(velocity_publisher_);
    nh.advertise(encoder_total_publisher_);

    twist_msg_.header.frame_id = "base_link";
    twist_msg_.twist.covariance[0] = kVelocityVariance;
}

void EncoderPublisher::Update() {
    if ( update_rate_.NeedsRun() ) {
        // Update encoder counts
        cli();                                              // stop isrs

        encoder_delta = encoder_total - encoder_last_total;
        encoder_last_total = encoder_total;

        sei();                                              // start isrs

        double velocity = compute_velocity(encoder_delta);  // velocity = m/s
        twist_msg_.header.stamp = nh_.now();
        twist_msg_.twist.twist.linear.x = velocity;

        velocity_publisher_.publish(&twist_msg_);
    }

    if ( update_rate_debug_.NeedsRun() ) {
        encoder_total_msg_.left = encoder_total;
        encoder_total_msg_.right = -1;
        encoder_total_publisher_.publish(&encoder_total_msg_);
    }
}

// pass encoder_total
double EncoderPublisher::compute_distance(double steps) {
    steps = steps / STEPS_PER_REV;                  // 6 steps per revolution (motor)
    steps = steps / GEAR_RATIO;                     // gear ratio (87 teeth on big gear / 18 teeth on small gear)
    steps = steps / BIG_GEAR_TO_WHEEL_RATIO;        // 3 big gear revolutions per 1 wheel revolution
    // at this point steps = wheel revolutions in timeframe

    steps = steps * (PI * WHEEL_DIAMETER_METERS);   // total meters driven
    return steps;
}

// pass encoder_delta
double EncoderPublisher::compute_velocity(double delta) {
    delta = delta / STEPS_PER_REV;                   // 6 delta per revolution (motor)
    delta = delta / GEAR_RATIO;                      // gear ratio (87 teeth on big gear / 18 teeth on small gear)
    delta = delta / BIG_GEAR_TO_WHEEL_RATIO;         // 3 big gear revolutions per 1 wheel revolution
    // at this point delta = wheel revolutions in timeframe

    delta = delta * (PI * WHEEL_DIAMETER_METERS);    // distance driven from (t2 - t1)
    delta = delta * ENCODER_UPDATE_HZ;               // delta = meters/second
    return delta;
}
