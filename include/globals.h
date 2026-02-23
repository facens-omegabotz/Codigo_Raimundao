#ifndef HEADERS_GLOBALS_H_
#define HEADERS_GLOBALS_H_

#define STACK_DEPTH 10000
#define TASK_PRIORITY 1

#define EVENT_SENSOR1 (1<<0)
#define EVENT_SENSOR2 (1<<1)
#define EVENT_SENSOR3 (1<<2)
#define EVENT_SENSOR4 (1<<3)
#define EVENT_QRE_LEFT (1<<4)
#define EVENT_QRE_RIGHT (1<<5)

#define SENSOR1 32
#define SENSOR2 33
#define SENSOR3 25
#define SENSOR4 27

#define NUM_SENSORS 2
#define NUM_SAMPLES_PER_SENSOR 4

#define QTR1 36
#define QTR2 39

#define IR 17

#define LED1 18
#define LED2 19

#define PWMA 4
#define PWMB 21
#define AIN1 16
#define AIN2 22
#define BIN1 23
#define BIN2 5

#define PWM_FREQ 1000
#define PWM_RES 8
#define PWM_CH1 1
#define PWM_CH2 2

#define WOODPECKER_PULSES 5

#define DEBUG_MODE 0 // undefine for no debug

#endif