#include "mbed.h"

#include "QEI.h"
#include "motor.h"

// External references to objects declared in main.cpp
extern Motor motor_left;
extern Motor motor_right;
extern QEI encoder_left;
extern QEI encoder_right;
extern DigitalOut driver_board_en;

// Constants used for calculation
const float WHEEL_DIAMETER = 0.083f; // 82mm in meters
const float PI = 3.14159f;
const int TOTAL_PULSES_PER_REV = 512; // revolution of wheel

const float TS = 0.05f; // 50ms sampling period
const float DISTANCE_PER_TICK = (PI * WHEEL_DIAMETER) / TOTAL_PULSES_PER_REV;


// Motor Class Constructor
Motor::Motor(PinName pwm_pin, PinName dir_pin, PinName bip_pin, QEI& encoder)
    : _pwm(pwm_pin), _dir(dir_pin), _bip(bip_pin), _encoder(encoder), _alpha(0.8f) {
    _pwm.period_us(50);
    _pwm.write(1.0f);
    _bip = 0;
    _prev_ticks = 0;
    _filtered_v = 0.0f;
    _last_duty = 0.0f;
}

void Motor::set_duty(float duty) {
    if (duty > 1.0f) duty = 1.0f;       // Guardrails: Constrain input to [-1, 1]
        if (duty < -1.0f) duty = -1.0f;

        _last_duty = duty; // Store for feedback logic

        if (duty >= 0) {        // Determine Direction based on sign
            _dir = 0;
        } else {
            _dir = 1;
        }

        // Set Speed (Magnitude) with Inversion Logic
        // get abs. value because PWM duty cycle must be positive
        float speed = std::fabs(duty);
        _pwm.write(1.0f - speed);
}

float Motor::get_duty() { return _last_duty; }


// This must be called at a FIXED interval (e.g., every 50ms)
void Motor::update_velocity(float dt, float distance_per_tick) {
    int current_ticks = _encoder.getPulses();
    int delta_ticks = current_ticks - _prev_ticks;
    _prev_ticks = current_ticks;

    // Calculate raw velocity: v = ds / dt
    float raw_v = (delta_ticks * distance_per_tick) / dt;

    // Apply Low-Pass Filter: y[n] = a*y[n-1] + (1-a)*x[n]
    _filtered_v = (_alpha * _filtered_v) + ((1.0f - _alpha) * raw_v);
}

float Motor::get_velocity() { return _filtered_v; }




//  -----     Navigation Functions    -----

float measure_distance(int pulses) {
    // Get the current pulse count
    // Distance per single tick * Number of ticks
    return (float)pulses * DISTANCE_PER_TICK;
}


void velocity_control(float target_vleft, float target_vright) {
    float Kp = 0.05f;

    // 1. Calculate Error (Set-point - Feedback)
    float error_l = target_vleft - motor_left.get_velocity();
    float error_r = target_vright - motor_right.get_velocity();

    // 2. Proportional Control Logic
    // We adjust the CURRENT duty cycle by the error scaled by Kp
    float new_duty_l = motor_left.get_duty() + (error_l * Kp);
    float new_duty_r = motor_right.get_duty() + (error_r * Kp);

    // 3. Apply the new calculated effort
    motor_left.set_duty(new_duty_l);
    motor_right.set_duty(new_duty_r);
}


void stop_motors(){
    motor_left.set_duty(0.0f); 
    motor_right.set_duty(0.0f);

}


void move_rotate(float degrees, float speed){
    const float L = 0.165f; // Track width in meters

    // 1. Convert degrees to radians and calculate target arc length
    float radians = std::fabs(degrees) * (PI / 180.0f);
    float target_arc = radians * (L / 2.0f);

    // 2. Setup hardware
    encoder_left.reset();
    encoder_right.reset();
    driver_board_en = 1;

    float current_dist = 0;

    while ((current_dist) < target_arc) {

        // 3. Set directions based on sign of 'degrees'
        if (degrees > 0) {
            velocity_control(speed, -speed);    // (Clockwise)
        } else {
            velocity_control(-speed, speed);    // (Anti-clockwise)
        }


        float dist_l = std::fabs(measure_distance(encoder_left.getPulses()));
        float dist_r = std::fabs(measure_distance(encoder_right.getPulses()));
        current_dist = (dist_l + dist_r) / 2.0f;    // Average both wheels

        wait(TS);
    }

    stop_motors();
}


void move_straight(float target_distance, float speed) {
    
    encoder_left.reset();   // 1. Reset encoders to establish start position (r = 0)
    encoder_right.reset();
    
    driver_board_en = 1;    // 2. Enable board
    float current_dist = 0;

    // 3. Monitor position (r) until target is reached
    // std::fabs handles cases where target or speed are negative
    while ((current_dist) < std::fabs(target_distance)) {

        velocity_control(speed, speed);

        // Update the average distance inside the loop
        float dist_l = std::fabs(measure_distance(encoder_left.getPulses()));
        float dist_r = std::fabs(measure_distance(encoder_right.getPulses()));
        current_dist = (dist_l + dist_r) / 2.0f;    // Average both wheels

        wait(TS); 
    }

    // 4. Target reached: Hard Stop
    stop_motors();
}

