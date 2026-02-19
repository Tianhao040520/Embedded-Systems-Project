#include "mbed.h"
#include "tasks.h"

// Switch different task from here
#define CURRENT_TASK 2

int main() {
#if   (CURRENT_TASK == 1)
    task1_pwm();
#elif (CURRENT_TASK == 2)
    task2_motor();
#elif (CURRENT_TASK == 3)
    task3_encoder();
#elif (CURRENT_TASK == 6)
    task6_square();
#else
#error "Invalid CURRENT_TASK value. Use 1,2,3,6."
#endif

   
    while (true) { wait(1.0f); }
}
