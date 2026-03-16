#include "mbed.h"                         
#include "C12832.h"
#include "QEI.h"
#include "pin_assignment.h"
#include "motor.h"
#include <cmath> // Required for fabs()

C12832 lcd(D11, D13, D12, D7, D10);

// --- OBJECTS  DECLARATION---
DigitalOut driver_board_en(DRIVER_ENABLE_PIN);
Serial pc(USBTX, USBRX, 115200);

QEI encoder_left(MOTORL_CHA_PIN, MOTORL_CHB_PIN, NC, 64);   // QEI(PinName channelA, PinName channelB, PinName index, int pulsesPerRev)
QEI encoder_right(MOTORR_CHA_PIN, MOTORR_CHB_PIN, NC, 64);

Motor motor_left(MOTORL_PWM_PIN, MOTORL_DIRECTION_PIN, MOTORL_BIPOLAR_PIN, encoder_left);
Motor motor_right(MOTORR_PWM_PIN , MOTORR_DIRECTION_PIN, MOTORR_BIPOLAR_PIN, encoder_right);



// --- CONSTANTS ---
const float WHEEL_DIAMETER = 0.083f; // 82mm in meters
const float PI = 3.14159f;
const int TOTAL_PULSES_PER_REV = 512; // rev of wheel

const float TS = 0.05f; // 50ms sampling period
const float DISTANCE_PER_TICK = (PI * WHEEL_DIAMETER) / TOTAL_PULSES_PER_REV;

Ticker motor_ticker; 

////* ISR FUNCTIONS *////


void motor_update_isr() {       // This function will be called automatically every 50ms
    motor_left.update_velocity(TS, DISTANCE_PER_TICK);
    motor_right.update_velocity(TS, DISTANCE_PER_TICK);
}



////* FUNCTIONS *////



//-----------------------------     MAIN    -----------------------------

int main(){

    driver_board_en = 0; // Board disabled
    encoder_left.reset();
    encoder_right.reset();

    motor_ticker.attach(&motor_update_isr, TS);

    pc.printf("System Booting... Wait 3s\r\n");
    wait(3);


    while(1){
      
        driver_board_en = 1; // Enable the H-Bridge


        // Use the "Set Point" logic
        float v_left = motor_left.get_velocity();
        float v_right = motor_right.get_velocity();

        pc.printf("V_L: %.2f m/s | V_R: %.2f m/s\r\n", v_left, v_right);


        velocity_control(0.5f, 0.5f);


        wait(TS);

    }
}
