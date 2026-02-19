#include "mbed.h"                         
#include "C12832.h"
#include "QEI.h"
#include "pin_assignment.h"
#include <cmath> // Required for fabs()

C12832 lcd(D11, D13, D12, D7, D10);

class Motor {
private:
    PwmOut _pwm;
    DigitalOut _dir;
    DigitalOut _bip;

public:
    // Constructor: Uses initializer list to map pins to hardware objects
    Motor(PinName pwm_pin, PinName dir_pin, PinName bip_pin) 
        : _pwm(pwm_pin), _dir(dir_pin), _bip(bip_pin) 
    {
        _pwm.period_us(50); // Set 20kHz frequency here once
        _pwm.write(1.0f);   // Default to OFF (Inverted logic)
        _bip = 0; // Defaulting to Unipolar mode
    }

    // Encapsulated method to set speed
    void set_speed(float duty) {
        if (duty > 1.0f) duty = 1.0f;       // Guardrails: Constrain input to [-1, 1]
        if (duty < -1.0f) duty = -1.0f;

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

};


// --- OBJECTS  DECLARATION---
Motor motor_left(MOTORL_PWM_PIN, MOTORL_DIRECTION_PIN, MOTORL_BIPOLAR_PIN);
Motor motor_right(MOTORR_PWM_PIN , MOTORR_DIRECTION_PIN, MOTORR_BIPOLAR_PIN);

DigitalOut driver_board_en(DRIVER_ENABLE_PIN);
Serial pc(USBTX, USBRX, 115200);

// QEI(PinName channelA, PinName channelB, PinName index, int pulsesPerRev)
QEI encoder_left(MOTORL_CHA_PIN, MOTORL_CHB_PIN, NC, 64); 
QEI encoder_right(MOTORR_CHA_PIN, MOTORR_CHB_PIN, NC, 64);


void stop_motors(){
    motor_left.set_speed(0.0f); 
    motor_right.set_speed(0.0f);

}

// --- DIMENSIONAL ANALYSIS CONSTANTS ---
const float WHEEL_DIAMETER = 0.083f; // 82mm in meters
const float GEAR_RATIO = 15.0f;      // Adjust based on your specific buggy
const float PI = 3.14159f;
const int ENCODER_RESOLUTION = 64 * 4; // PPR * X4 Encoding
const int TOTAL_PULSES_PER_REV = 512;

/**
 * @brief Converts raw encoder pulses to meters
 * @param pulses The raw integer from encoder.getPulses()
 * @return Distance in meters (float)
 */
float measure_distance(int pulses) {
    // Total Ticks for one full wheel revolution
    float ticks_per_rev = TOTAL_PULSES_PER_REV;
    
    // Distance per single tick
    float distance_per_tick = (PI * WHEEL_DIAMETER) / ticks_per_rev;
    
    return (float)pulses * distance_per_tick;
}


void move_rotate(float degrees, float speed){
    const float L = 0.165f; // Track width in meters
    const float PI = 3.14159f;

    // 1. Convert degrees to radians and calculate target arc length
    float radians = std::fabs(degrees) * (PI / 180.0f);
    float target_arc = radians * (L / 2.0f);

    // 2. Setup hardware
    encoder_left.reset();
    encoder_right.reset();
    driver_board_en = 1;

    // 3. Set directions based on sign of 'degrees'
    if (degrees > 0) {
        motor_left.set_speed(-speed);     // Forward
        motor_right.set_speed(speed);   // Backward (Clockwise)
    } else {
        motor_left.set_speed(speed);    // Backward
        motor_right.set_speed(-speed);     // Forward (Anti-clockwise)
    }

    while (std::fabs(measure_distance(encoder_left.getPulses())) < target_arc) {
        wait(0.02);
    }

    stop_motors();
    driver_board_en = 0;
}


void move_straight(float target_distance, float speed) {
    
    encoder_left.reset();   // 1. Reset encoders to establish start position (r = 0)
    encoder_right.reset();
    
    driver_board_en = 1;    // 2. Enable board and set requested speed
    motor_left.set_speed(speed);
    motor_right.set_speed(speed);

    // 3. Monitor position (r) until target is reached
    // std::fabs handles cases where target or speed are negative
    while (std::fabs(measure_distance(encoder_left.getPulses())) < std::fabs(target_distance)) {
        
        wait(0.02); 
    }

    // 4. Target reached: Hard Stop
    stop_motors();
    driver_board_en = 0; 
}



int main(){

    driver_board_en = 0; // Board disabled
    encoder_left.reset();
    encoder_right.reset();

    wait(10);
    while(1){
      // --- 4. EXECUTION SEQUENCE ---
        driver_board_en = 1; // Enable the H-Bridge

        
        // Get the current pulse count
        int counts_l = encoder_left.getPulses();
        int counts_r = encoder_right.getPulses();
        // Convert to meters
        float dist_l = measure_distance(counts_l);
        float dist_r = measure_distance(counts_r);

        // Print to terminal: \r\n moves the cursor to a new line
        pc.printf("L: %d | R: %d | DistL: %.3fm | DistR: %.3fm\r\n", counts_l, counts_r, dist_l, dist_r);
        
        wait(0.1);

    }
}