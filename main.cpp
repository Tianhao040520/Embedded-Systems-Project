#include "mbed.h"                         
#include "C12832.h"
#include "QEI.h"
#include "pin_assignment.h"
#include "motor.h"
#include "ble_turnaround.h"
#include <cmath> // Required for fabs()

C12832 lcd(D11, D13, D12, D7, D10);

// --- OBJECTS  DECLARATION---
DigitalOut driver_board_en(DRIVER_ENABLE_PIN);
Serial pc(USBTX, USBRX, 115200);
Serial hm10(BT_TX_PIN, BT_RX_PIN, 9600);

QEI encoder_left(MOTORL_CHA_PIN, MOTORL_CHB_PIN, NC, 64);   // QEI(PinName channelA, PinName channelB, PinName index, int pulsesPerRev)
QEI encoder_right(MOTORR_CHA_PIN, MOTORR_CHB_PIN, NC, 64);

Motor motor_left(MOTORL_PWM_PIN, MOTORL_DIRECTION_PIN, MOTORL_BIPOLAR_PIN, encoder_left);
Motor motor_right(MOTORR_PWM_PIN , MOTORR_DIRECTION_PIN, MOTORR_BIPOLAR_PIN, encoder_right);

// Array of AnalogIn objects to read the sensors.
AnalogIn a0(SENSOR0_OUT_PIN), a1(SENSOR1_OUT_PIN), a2(SENSOR2_OUT_PIN), a3(SENSOR3_OUT_PIN), a4(SENSOR4_OUT_PIN), a5(SENSOR5_OUT_PIN);

 // Array of DigitalOut objects to control the LEDs.
DigitalOut d0(SENSOR0_IN_PIN, PullUp), d1(SENSOR1_IN_PIN, PullUp), d2(SENSOR2_IN_PIN, PullUp), d3(SENSOR3_IN_PIN, PullUp) ;
DigitalOut d4(SENSOR4_IN_PIN, PullUp), d5(SENSOR5_IN_PIN, PullUp);



// --- CONSTANTS ---
const float WHEEL_DIAMETER = 0.083f; // 82mm in meters
const float PI = 3.14159f;
const int TOTAL_PULSES_PER_REV = 512; // rev of wheel

const float TS = 0.03f; // 50ms sampling period
const float DISTANCE_PER_TICK = (PI * WHEEL_DIAMETER) / TOTAL_PULSES_PER_REV;

Ticker motor_ticker; 

////* ISR FUNCTIONS *////


void motor_update_isr() {       // This function will be called automatically every 50ms
    motor_left.update_velocity(TS, DISTANCE_PER_TICK);
    motor_right.update_velocity(TS, DISTANCE_PER_TICK);
}



////* FUNCTIONS *////

float calculate_line_error() {
   
    float s0 = (a0.read()) * 100.0f;
    float s1 = (a1.read()) * 100.0f;
    float s2 = (a2.read()) * 100.0f;
    float s3 = (a3.read()) * 100.0f;
    float s4 = (a4.read()) * 100.0f;
    float s5 = (a5.read()) * 100.0f;

    // Define Weights: Sensors on the left are negative, right are positive
    float weights[] = {-4.0f, -2.0f, -1.0f, 1.0f, 2.0f, 4.0f};

    // Calculate Weighted Sum and Total Intensity
    float numerator = (s0 * weights[0]) + (s1 * weights[1]) + (s2 * weights[2]) + 
                      (s3 * weights[3]) + (s4 * weights[4]) + (s5 * weights[5]);
                      
    float denominator = s0 + s1 + s2 + s3 + s4 + s5;

    // Safety: If the total 'blackness' is too low, no line is seen
    if (denominator < 50.0f) { 
        return 0.0f;}

    return numerator / denominator;
}


void follow_line(float base_speed, float kp, float kd) {
    static float last_line_error = 0; // Remembers value for next call
    
    // Get the current position from the sensor array
    float current_error = calculate_line_error();

    // PD Steering Math
    float derivative = (current_error - last_line_error) / TS;
    float steering_adj = (current_error * kp) + (derivative * kd);
    
    // Update last error for the next cycle
    last_line_error = current_error;

    // Differential Steering Calculation
    float target_v_left  = base_speed - steering_adj;
    float target_v_right = base_speed + steering_adj;

    // 4. Pass targets to the underlying velocity controller
    velocity_control(target_v_left, target_v_right);
}

void check_ble_turnaround(float turn_speed) {
    static bool initialised = false;

    if (!initialised) {
        hm10.baud(9600);
        pc.baud(115200);
        initialised = true;
        pc.printf("BLE turnaround listener ready.\r\n");
        pc.printf("Send T = clockwise 180, U = anticlockwise 180, S = stop\r\n");
    }

    if (hm10.readable()) {
        char c = hm10.getc();

        if (c == 'T') {
            pc.printf("Received T -> clockwise 180 turnaround\r\n");
            hm10.putc('T');
            stop_motors();
            move_rotate(180.0f, turn_speed);
        }
        else if (c == 'U') {
            pc.printf("Received U -> anticlockwise 180 turnaround\r\n");
            hm10.putc('U');
            stop_motors();
            move_rotate(-180.0f, turn_speed);
        }
        else if (c == 'S') {
            pc.printf("Received S -> stop\r\n");
            hm10.putc('S');
            stop_motors();
        }
        else {
            pc.printf("Unknown BLE command: %c\r\n", c);
        }
    }
}


//-----------------------------     MAIN    -----------------------------

int main(){

    driver_board_en = 0; // Board disabled
    encoder_left.reset();
    encoder_right.reset();

    motor_ticker.attach(&motor_update_isr, TS);

    float last_line_error = 0;

    pc.printf("System Booting... Wait 3s\r\n");
    wait(3);


    while(1){
      
        driver_board_en = 1; // Enable the H-Bridge

        d0 = 1; d1 = 1; d2 = 1; d3 = 1; d4 = 1; d5 = 1; // Turn on LED

        float current_error = calculate_line_error();

        
        follow_line(0.4f, 0.6f, 0.15f);
        // Target, Kp, Kd
        //  0.3   0.6    0.1


        float v_left = motor_left.get_velocity();
        float v_right = motor_right.get_velocity();

        pc.printf("Err: %.2f | V_L: %.2f m/s | V_R: %.2f m/s\r\n", current_error, v_left, v_right);

        check_ble_turnaround(0.25f);

        // d0 = 0; d1 = 0; d2 = 0; d3 = 0; d4 = 0; d5 = 0;

        wait(TS);

    }
}
