#include "mbed.h"                         
#include "C12832.h"
#include "QEI.h"
#include "pin_assignment.h"

C12832 lcd(D11, D13, D12, D7, D10);



class Motor
{
private:
    PwmOut _pwm;        // Speed signal
    DigitalOut _dir;    // Direction signal
    float current_speed; // Speed value (0.0 to 1.0)
    bool dir_forward;   // Direction state

public:
    /**
     * @brief Constructor for the Motor class
     * @param pwm_pin Pin for PWM speed control
     * @param dir_pin Pin for Direction control
     */
    Motor(PinName pwm_pin, PinName dir_pin);


    /**
     * @brief Set the motor speed and direction
     * @param speed Float from -1.0 (Full Reverse) to 1.0 (Full Forward)
     */
    void set_speed(float speed); 

    /**
     * @brief Emergency stop - cuts power immediately
     */
    void stop();

    // Getters for telemetry
    float get_speed() const { return current_speed; }

};

Motor::Motor(PinName pwm_pin, PinName dir_pin) : _pwm(pwm_pin), _dir(dir_pin) {
    // 20kHz is standard to avoid audible noise
    _pwm.period_us(50); 
    stop();
}

void Motor::set_speed(float speed) {
    // 1. Constrain speed to valid range [-1.0, 1.0]
    if (speed > 1.0f) speed = 1.0f;
    if (speed < -1.0f) speed = -1.0f;

    // 2. Determine Direction
    if (speed >= 0) {
        _dir = 1; // Forward logic
        current_speed = speed;
    } else {
        _dir = 0; // Reverse logic
        current_speed = -speed; // PWM duty cycle must be positive
    }

    // 3. Update Hardware
    _pwm.write(current_speed);
}

void Motor::stop() {
    _pwm.write(0.0f);
    current_speed = 0.0f;
}


class MotorDriverBoard
{
private:
protected:
public:
    
};



// --- 2. HARDWARE OBJECTS ---
DigitalOut driver_board_en(DRIVER_ENABLE_PIN);

Motor left_motor(MOTORL_PWM_PIN, MOTORL_DIRECTION_PIN);
Motor right_motor(MOTORR_PWM_PIN, MOTORR_DIRECTION_PIN);


int main(){

    // Initial Safety State
    driver_board_en = 0; // Board disabled
    left_motor.set_speed(0.0f);  // stop
    right_motor.set_speed(0.0f); 
    
    while(1){
      
        driver_board_en = 1; // Enable the H-Bridge
            
        left_motor.set_speed(0.6f);  // Forward 
        right_motor.set_speed(0.6f); 
        wait(10);
        
        left_motor.stop();
        right_motor.stop();
        wait(3);
            
        left_motor.set_speed(-0.6f);  // Reverse 
        right_motor.set_speed(-0.6f); 
        wait(10);

        left_motor.stop();
        right_motor.stop();
        wait(3);
    }


}

