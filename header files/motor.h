#pragma once  // to prevent double inclusion

#include "mbed.h"
#include "QEI.h"


class Motor {
private:
    PwmOut _pwm;
    DigitalOut _dir;
    DigitalOut _bip;
    QEI&       _encoder; // Reference to the hardware encoder

    // Velocity state variables
    int _prev_ticks;
    float _current_v;
    float _filtered_v;

    float _last_duty; // Track the current effort
    
    // Filter coefficient (0.0 to 1.0)
    // 0.8 means 80% old value, 20% new value. Higher = smoother but slower response.
    const float _alpha;

public:
    // Constructor: Uses initializer list to map pins to hardware objects
    Motor(PinName pwm_pin, PinName dir_pin, PinName bip_pin, QEI& encoder);

    // Encapsulated method to set duty cycle
    void set_duty(float duty);

    float get_duty();

    // This must be called at a FIXED interval (e.g., every 50ms)
    void update_velocity(float dt, float distance_per_tick);

    float get_velocity();

};

// Function Prototypes for Navigation
float measure_distance(int pulses);

void stop_motors();
void move_straight(float target_distance, float speed);
void move_rotate(float degrees, float speed);
void move_rotateMANUAL(float degrees, float speed);

void velocity_control(float target_vleft, float target_vright);

void follow_line(float base_speed, float kp, float kd);




