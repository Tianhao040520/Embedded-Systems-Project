#include "mbed.h"

// Sensor Analog Input Pins
#define SENSOR0_OUT_PIN         A0
#define SENSOR1_OUT_PIN         A1
#define SENSOR2_OUT_PIN         A2
#define SENSOR3_OUT_PIN         A3
#define SENSOR4_OUT_PIN         A4
#define SENSOR5_OUT_PIN         A5

// Sensor Digital Output Control Pin
#define SENSOR0_IN_PIN         D3
#define SENSOR1_IN_PIN         D5 
#define SENSOR2_IN_PIN         D6 
#define SENSOR3_IN_PIN         D9 
#define SENSOR4_IN_PIN         D10 
#define SENSOR5_IN_PIN         D11

// Initialize Serial for terminal output
Serial pc(USBTX, USBRX, 115200);

// Array of AnalogIn objects to read the sensors.
AnalogIn a0(SENSOR0_OUT_PIN), a1(SENSOR1_OUT_PIN), a2(SENSOR2_OUT_PIN), a3(SENSOR3_OUT_PIN), a4(SENSOR4_OUT_PIN), a5(SENSOR5_OUT_PIN);

 // Array of DigitalOut objects to control the LEDs.
DigitalOut d0(SENSOR0_IN_PIN, PullUp), d1(SENSOR1_IN_PIN, PullUp), d2(SENSOR2_IN_PIN, PullUp), d3(SENSOR3_IN_PIN, PullUp) ;
DigitalOut d4(SENSOR4_IN_PIN, PullUp), d5(SENSOR5_IN_PIN, PullUp);


float calculate_line_error() {
   // Invert the readings: (1.0 - read()) * 100
    // Now Black Line (~0.1) becomes ~90, and White Floor (~0.9) becomes ~10.
    float s0 = (1.0f - a0.read()) * 100.0f;
    float s1 = (1.0f - a1.read()) * 100.0f;
    float s2 = (1.0f - a2.read()) * 100.0f;
    float s3 = (1.0f - a3.read()) * 100.0f;
    float s4 = (1.0f - a4.read()) * 100.0f;
    float s5 = (1.0f - a5.read()) * 100.0f;

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

    // Safety: If the total 'blackness' is too low, no line is seen
    if (denominator < 50.0f) { 
        return 0.0f;}

    return numerator / denominator;
}

int main()
{
    pc.printf("\n=== NUCLEO-F401RE TCRT5000 Sensor Array Test ===\n");
    pc.printf("Reading Analog (0-4095) states...\n");

    // Disable all IR LEDs via Darlington Array
    d0 = 0; d1 = 0; d2 = 0; d3 = 0; d4 = 0; d5 = 0;
    
    wait(0.01);

    while (1) {
        d0 = 1; d1 = 1; d2 = 1; d3 = 1; d4 = 1; d5 = 1;

        wait_ms(1); 
        
        int a0v = a0.read()* 100.0f ; 
        int a1v = a1.read()* 100.0f ;
        int a2v = a2.read()* 100.0f ;
        int a3v = a3.read()* 100.0f ;
        int a4v = a4.read()* 100.0f ;
        int a5v = a5.read()* 100.0f ;

        float val= calculate_line_error();

        // d0 = 0; d1 = 0; d2 = 0; d3 = 0; d4 = 0; d5 = 0;

        
        const char* dir = "Centered";
        if (val < -0.01f) dir = "Right"; // Only report Right if error is significant
        else if (val > 0.01f) dir = "Left"; // Only report Left if error is significant 


        pc.printf("Analog: %2d %2d %2d %2d %2d %2d | Error: %7.3f | Dir: %s\r\n",
                   a0v, a1v, a2v, a3v, a4v, a5v, val, dir);

        
        wait(0.1); 
    }
}
