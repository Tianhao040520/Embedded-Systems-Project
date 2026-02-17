#include "mbed.h"                         //Imports mbed libraries
#include "C12832.h"
#include "QEI.h"
#include "pin_assignment.h"

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
        _pwm.write(0.0f);   // Ensure it starts at 0%
        _bip = 0; // Defaulting to Unipolar mode
    }

    // Encapsulated method to set speed
    void set_speed(float duty) {
        if (duty > 1.0f) duty = 1.0f;
        if (duty < 0.0f) duty = 0.0f;
        _pwm.write(duty);
    }

    void set_direction(bool state) {
        _dir = state;
    }
};





// --- 2. HARDWARE OBJECTS ---
Motor motor_left(MOTORL_PWM_PIN, MOTORL_DIRECTION_PIN, MOTORL_BIPOLAR_PIN);
Motor motor_right(MOTORR_PWM_PIN , MOTORR_DIRECTION_PIN, MOTORR_BIPOLAR_PIN);

DigitalOut driver_board_en(DRIVER_ENABLE_PIN);

int main(){

    // Initial Safety State
    driver_board_en = 0; // Board disabled
    
    while(1){
      // --- 4. EXECUTION SEQUENCE ---
            driver_board_en = 1; // Enable the H-Bridge
            
            // Move Forward at 30% (Use your class methods!)
        motor_left.set_direction(1);
        motor_right.set_direction(1);
        motor_left.set_speed(0.3f); 
        motor_right.set_speed(0.3f);
        wait(5);

        motor_left.set_direction(1);
        motor_right.set_direction(1);
        motor_left.set_speed(0.7f); 
        motor_right.set_speed(0.7f);
        wait(5);

        // Stop
        motor_left.set_speed(1.0f);
        motor_right.set_speed(1.0f);
        wait(5);

        motor_left.set_direction(0);
        motor_right.set_direction(0);
        motor_left.set_speed(0.3f); 
        motor_right.set_speed(0.3f);
        wait(5);

        motor_left.set_direction(0);
        motor_right.set_direction(0);
        motor_left.set_speed(0.7f); 
        motor_right.set_speed(0.7f);
        wait(5);

    }


}
