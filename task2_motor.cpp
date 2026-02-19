#include "mbed.h"
#include "pin_assignment.h"
#include "Potentiometer.h"
#include "tasks.h"

// Potentiometers (confirmed: A0/A1, VDD = 3.3V)
static Potentiometer potL(A1, 3.3f);
static Potentiometer potR(A0, 3.3f);

// PWM outputs to motor driver board inputs
static PwmOut pwmL(MOTORL_PWM_PIN);
static PwmOut pwmR(MOTORR_PWM_PIN);

// Motor driver control pins
static DigitalOut driver_en(DRIVER_ENABLE_PIN);
static DigitalOut dirL(MOTORL_DIRECTION_PIN);
static DigitalOut dirR(MOTORR_DIRECTION_PIN);
static DigitalOut bipL(MOTORL_BIPOLAR_PIN);
static DigitalOut bipR(MOTORR_BIPOLAR_PIN);

static float clamp01(float x) {
    if (x < 0.0f) return 0.0f;
    if (x > 1.0f) return 1.0f;
    return x;
}

// Apply a small deadband around 0.5 to prevent jitter near "stop"
static float apply_deadband(float x, float center, float width) {
    if (x > center - width && x < center + width) return center;
    return x;
}

void task2_motor() {

    // Switching frequency justification: 20 kHz is above audible range and gives smooth control
    pwmL.period_us(50);   // 20 kHz
    pwmR.period_us(50);

    // Task 2 requirement: motors set to BIPOLAR mode
    bipL = 1;
    bipR = 1;

    // In locked anti-phase (bipolar) mode, direction can be controlled by duty around 50%.
    // Some boards still require DIR to be defined; set it to a known state.
    dirL = 0;
    dirR = 0;

    // Enable H-bridge
    driver_en = 1;

    // Start stopped: bipolar "stop" = 50% effective duty (low-active inversion doesn't change 50%)
    pwmL.write(0.5f);
    pwmR.write(0.5f);

    while (true) {
        // Read pots: 0..1
        float pL = clamp01(potL.amplitudeNorm());
        float pR = clamp01(potR.amplitudeNorm());

        // Bipolar stop at 0.5 (apply deadband to reduce tiny movements/noise)
        pL = apply_deadband(pL, 0.5f, 0.03f);
        pR = apply_deadband(pR, 0.5f, 0.03f);

        // Effective bipolar duty = pot value (0..1):
        // 0.5 -> stop, <0.5 reverse, >0.5 forward
        float dutyEffL = pL;
        float dutyEffR = pR;

        // Low-active PWM input: write inverted duty
        float dutyWriteL = 1.0f - dutyEffL;
        float dutyWriteR = 1.0f - dutyEffR;

        pwmL.write(dutyWriteL);
        pwmR.write(dutyWriteR);

        wait_ms(20);
    }
}
