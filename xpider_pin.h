#ifndef XPIDER_PIN_H_
#define XPIDER_PIN_H_

/* IO reference value */
#define IO_REFERENCE 5.00f

/* Minimal battery voltage the robot can work properly */
#define BATTERY_THRESHOLD 3.0f

#define CAMERA_MIN_ANGLE 15
#define CAMERA_MAX_ANGLE 65

/*
 * Extrem value offset, to prevent seize on the extrem position 
 */
#define CAMERA_VALUE_OFFSET 80

#define CAMERA_ENCODER_MIN_ERROR 3

/*
 * Atmega328(Arduino)
 */

/* Step counter */
#define STEP_HALL_INT 2

/* Rotate */
#define MOTOR_R_1 3
#define MOTOR_R_2 5

/* MPU6050 DMP data ready (unused) */
#define MPU6050_INT 4

/* Walk */
#define MOTOR_F_1 9
#define MOTOR_F_2 6

/* Front leds bus */
#define LED_FRONT 7

/* Camera */
#define CAMERA_1 10
#define CAMERA_2 11

/* Rotate counter */
#define ROTATE_HALL 12

/* Camera ready */
#define CAMERA_READY A0

/* IR distance */
#define IR_DIST A2

/* MPU6050 I2C */
#define MPU6050_SDA A4
#define MPU6050_SCL A5

/* Battery voltage */
#define BATTERY_VOLT A6

/* Camera encoder */
#define CAMERA_ENCODER A7

/* 
 * Unused pin
 * D: 8, 13
 * A: 1, 3
 */

/*
 * Intel Curie
 */

/* Inside serial pin */
#define INSIDE_TX 2
#define INSIDE_RX 3

/* Mic */
#define SPEAKER 5

/* Rear led */
#define LED_REAR 13

/* SPI flash chip select */
#define FLASH_CHIP_SELECT 21

/* Sound sensor */
#define SOUND_SENSOR A2

#endif // XPIDER_PIN_H_
