#include "mbed.h"

// Sensor Analog Input Pins
#define SENSOR0_OUT_PIN         PC_0
#define SENSOR1_OUT_PIN         PC_1
#define SENSOR2_OUT_PIN         PB_0
#define SENSOR3_OUT_PIN         PA_4
#define SENSOR4_OUT_PIN         PC_3
#define SENSOR5_OUT_PIN         PC_2

// Sensor Digital Output Control Pin
#define SENSOR0_IN_PIN         PC_2
#define SENSOR1_IN_PIN         PC_3 
#define SENSOR2_IN_PIN         PA_4 
#define SENSOR3_IN_PIN         PB_0 
#define SENSOR4_IN_PIN         PC_1 
#define SENSOR5_IN_PIN         PC_0

// Initialize Serial for terminal output
Serial pc(USBTX, USBRX, 115200);

// Array of AnalogIn objects to read the sensors.
AnalogIn a0(SENSOR0_OUT_PIN), a1(SENSOR1_OUT_PIN), a2(SENSOR2_OUT_PIN), a3(SENSOR3_OUT_PIN), a4(SENSOR4_OUT_PIN), a5(SENSOR5_OUT_PIN);

 // Array of DigitalOut objects to control the LEDs.
DigitalOut d0(D3, PullUp);
DigitalOut d1(D4, PullUp);
DigitalOut d2(D5, PullUp);
DigitalOut d3(D6, PullUp);
DigitalOut d4(D7, PullUp);
DigitalOut d5(D8, PullUp);

float calculate_line_error() {
    // 1. Read all 6 sensors (12-bit)
    float s0 = (float)(a0.read_u16() >> 4);
    float s1 = (float)(a1.read_u16() >> 4);
    float s2 = (float)(a2.read_u16() >> 4);
    float s3 = (float)(a3.read_u16() >> 4);
    float s4 = (float)(a4.read_u16() >> 4);
    float s5 = (float)(a5.read_u16() >> 4);

    // 2. Define Weights: Sensors on the left are negative, right are positive
    float weight0 = -3.0f;
    float weight1 = -2.0f;
    float weight2 = -1.0f;
    float weight3 = 1.0f;
    float weight4 = 2.0f;
    float weight5 = 3.0f;

    // 3. Calculate Weighted Sum and Total Intensity
    float numerator = (s0 * weight0) + (s1 * weight1) + (s2 * weight2) + 
                      (s3 * weight3) + (s4 * weight4) + (s5 * weight5);
                      
    float denominator = s0 + s1 + s2 + s3 + s4 + s5;

    // 4. Safety: Prevent division by zero if the buggy is lifted or off-track
    if (denominator < 100.0f) {
        return 0.0f; 
    }

    return numerator / denominator;
}

int main()
{
    pc.printf("\n=== NUCLEO-F401RE TCRT5000 Sensor Array Test ===\n");
    pc.printf("Reading Analog (0-4095) states...\n");

    // Enable all IR LEDs via Darlington Array
    d0 = 1; d1 = 1; d2 = 1; d3 = 1; d4 = 1; d5 = 1;
    
    wait(0.01);

    while (1) {
        // Read 16-bit values and shift to 12-bit (0~4095)
        int a0v = a0.read_u16() >> 4; 
        int a1v = a1.read_u16() >> 4;
        int a2v = a2.read_u16() >> 4;
        int a3v = a3.read_u16() >> 4;
        int a4v = a4.read_u16() >> 4;
        int a5v = a5.read_u16() >> 4;

        float val= calculate_line_error();

        
        const char* dir = "Centered";
        if (val < -0.01f) dir = "Right"; // Only report Right if error is significant
        else if (val > 0.01f) dir = "Left"; // Only report Left if error is significant


        pc.printf("Analog: %4d %4d %4d %4d %4d %4d | Error: %7.3f | Dir: %s\r\n",
                   a0v, a1v, a2v, a3v, a4v, a5v, val, dir);

        
        wait(0.1); 
    }
}
