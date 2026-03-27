#pragma once


// Debug LED pin
#define LED_PIN                 LED1

// Motor 1 Pins
#define MOTORL_BIPOLAR_PIN      PC_9
#define MOTORL_DIRECTION_PIN    PB_8
#define MOTORL_PWM_PIN          PB_9

// Motor 2 Pins
#define MOTORR_BIPOLAR_PIN      PB_2
#define MOTORR_DIRECTION_PIN    PB_1
#define MOTORR_PWM_PIN          PB_15

// Other Motor Driver Board Pins
#define DRIVER_ENABLE_PIN       PC_4
#define DRIVER_MONITOR_PIN      PB_4

// Motor Encoder Channels Pins
#define MOTORL_CHA_PIN          PC_8
#define MOTORL_CHB_PIN          PC_6
#define MOTORR_CHA_PIN          PB_14
#define MOTORR_CHB_PIN          PB_13  // switch A and B pins potentially!!!

// Bluetooth Pins
#define BT_TX_PIN               PA_11
#define BT_RX_PIN               PA_12

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
