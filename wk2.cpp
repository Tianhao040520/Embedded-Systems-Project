#include "mbed.h"                         //Imports mbed libraries
#include "C12832.h"
#include "pin_assignment.h"

C12832 lcd(D11, D13, D12, D7, D10);



class Motor
{
private:
public:
};



class MotorDriverBoard
{
private:
protected:
public:
    
};



// --- 2. HARDWARE OBJECTS ---
PwmOut motor_l(MOTORL_PWM_PIN);         //  refer pins in header file
DigitalOut dir_l(MOTORL_DIRECTION_PIN);
PwmOut motor_r(MOTORR_PWM_PIN);
DigitalOut dir_r(MOTORR_DIRECTION_PIN);
DigitalOut driver_board_en(DRIVER_ENABLE_PIN);

int main(){
     // --- 3. CONFIGURATION ---
    // Set frequency to 20kHz (50us period) to avoid audible whining
    motor_l.period_us(50);
    motor_r.period_us(50);

    // Initial Safety State
    driver_board_en = 0; // Board disabled
    motor_l.write(0.0f); // Speed 0%
    motor_r.write(0.0f);
    
    while(1){
      // --- 4. EXECUTION SEQUENCE ---
            driver_board_en = 1; // Enable the H-Bridge
            
            // Move Forward at 30% Duty Cycle
            dir_l = 1; 
            dir_r = 1;
            motor_l.write(0.6f); 
            motor_r.write(0.6f);
            



    }


}

