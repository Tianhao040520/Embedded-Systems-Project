#include "mbed.h"
#include "pin_assignment.h"
#include "Potentiometer.h"
#include "tasks.h"

// Potentiometers (connected to A1 and A0, VDD = 3.3V)
static Potentiometer potL(A1, 3.3f);
static Potentiometer potR(A0, 3.3f);

// Two PWM logic outputs to the motor driver input test-points
static PwmOut pwmL(MOTORL_PWM_PIN);
static PwmOut pwmR(MOTORR_PWM_PIN);

// Motor driver board control pins (kept in a safe state for Task 1)
static DigitalOut driver_en(DRIVER_ENABLE_PIN);
static DigitalOut dirL(MOTORL_DIRECTION_PIN);
static DigitalOut dirR(MOTORR_DIRECTION_PIN);
static DigitalOut bipL(MOTORL_BIPOLAR_PIN);
static DigitalOut bipR(MOTORR_BIPOLAR_PIN);

// Clamp input value to the range [0.0, 1.0]
static float clamp01(float x) {
    if (x < 0.0f) return 0.0f;
    if (x > 1.0f) return 1.0f;
    return x;
}

void task1_pwm() {
    // Set PWM switching frequency to 20 kHz (period = 50 us)
    pwmL.period_us(50);
    pwmR.period_us(50);

    // Put the driver board in a known, safe configuration.
    // Task 1 does NOT require the wheels to spin, only PWM waveforms on the scope.
    driver_en = 0;   // Keep H-bridge disabled (safe)
    bipL = 0;        // Unipolar mode (simple for PWM demonstration)
    bipR = 0;
    dirL = 0;        // Fixed direction (doesn't matter while disabled)
    dirR = 0;

    // Initialise PWM outputs to "motor off" for low-active PWM inputs
    // Low-active: write(1.0f) => always HIGH => 0% effective (LOW) drive
    pwmL.write(1.0f);
    pwmR.write(1.0f);

    while (true) {
        // Read normalised potentiometer values (0.0 to 1.0)
        float p1 = clamp01(potL.amplitudeNorm());
        float p2 = clamp01(potR.amplitudeNorm());

        // Low-active PWM mapping:
        // Effective drive is proportional to LOW time, so invert the duty cycle.
        pwmL.write(1.0f - p1);
        pwmR.write(1.0f - p2);

        // Slow update rate slightly to reduce jitter on duty measurement
        wait_ms(20);
    }
}
