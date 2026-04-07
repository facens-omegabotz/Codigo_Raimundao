#ifndef HEADERS_GLOBALS_H_
  
  #define HEADERS_GLOBALS_H_

  #define STACK_DEPTH 10000
  #define TASK_PRIORITY 1

  #define EVENT_BIT_SENSOR_1 (1<<0)
  #define EVENT_BIT_SENSOR_2 (1<<1)
  #define EVENT_BIT_SENSOR_3 (1<<2)
  #define EVENT_BIT_SENSOR_4 (1<<3)
  #define EVENT_BIT_SENSOR_5 (1<<4)

  #define ALL_IR_EVENT_BITS     EVENT_BIT_SENSOR_1 | EVENT_BIT_SENSOR_2 | EVENT_BIT_SENSOR_3 | EVENT_BIT_SENSOR_4 | EVENT_BIT_SENSOR_5
  #define NO_LETMOST_BIT        EVENT_BIT_SENSOR_2 | EVENT_BIT_SENSOR_3 | EVENT_BIT_SENSOR_4 | EVENT_BIT_SENSOR_5
  #define RIGHT_SENSOR_BITS     EVENT_BIT_SENSOR_4 | EVENT_BIT_SENSOR_5
  #define LEFT_SENSOR_BITS      EVENT_BIT_SENSOR_1 | EVENT_BIT_SENSOR_2
  #define NO_RIGHTMOST_BIT      EVENT_BIT_SENSOR_1 | EVENT_BIT_SENSOR_2 | EVENT_BIT_SENSOR_3 | EVENT_BIT_SENSOR_4
  #define FORWARD_BIT           EVENT_BIT_SENSOR_3

  #define EVENT_QRE_LEFT (1<<5)
  #define EVENT_QRE_RIGHT (1<<6)

  #define IR_SENSOR_1 25u
  #define IR_SENSOR_2 27u
  #define IR_SENSOR_3 26u
  #define IR_SENSOR_4 33u
  #define IR_SENSOR_5 32u

  #define NUM_SENSORS 2u
  #define NUM_SAMPLES_PER_SENSOR 4u

  #define QTR_1 35u
  #define QTR_2 34u

  #define IR 13u

  #define SERVO_1 4u
  #define SERVO_2 16u

  #define PWMA 18u
  #define PWMB 17u
  #define AIN1 19u
  #define AIN2 21u
  #define BIN1 22u
  #define BIN2 23u

  #define PWM_FREQ 1000
  #define PWM_RES 8
  #define PWM_CH1 1
  #define PWM_CH2 2

  #define WOODPECKER_PULSES 5

  #define DEBUG_MODE 0 // undefine for no debug

#endif