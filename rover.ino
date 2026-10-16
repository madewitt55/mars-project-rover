/**
 * @file rover.ino
 * @brief Firmware for controlling a Bluetooth-compatible rover
 *
 * @details Designed to run on a compatible microcontroller in combination with a Bluetooth antenna.
 *
 * @note Rover firmware will reject and disconnect from any unrecognized BT connections. The MAC
         address of the connecting device's BT antenna must be defined as `BEACON_BT_MAC`. Any
         connected serial monitor should run at 9600 baud. On boot, setup() waits 3 seconds to let
         the serial monitor attach before initialization output is printed.
 */

#include "Logger.h" // Local logging helper class
#include "Bluetooth.h" // Local Bluetooth SPP connection and command helpers

//---------- L298N Dual H-Bridge Motor Driver ----------
// When using a power supply greater than 12V, 5V regulator jumper MUST be installed
// With jumper installed, a separate 5V power source must be supplied to power the driver's logic
// H-Bridge direction control pins
#define IN1 13 // Left motor, when driven HIGH, current enters motor through OUT1 and returns through OUT2
#define IN2 14 // Left motor, when driven HIGH, current enters motor through OUT2 and returns through OUT1
#define IN3 25 // Right motor, when driven HIGH, current enters motor through OUT3 and returns through OUT4
#define IN4 26 // Right motor, when driven HIGH, current enters motor through OUT1 and returns through OUT2
// PWM speed control pins
#define ENA 32 // Enable A (left motor), PWM output value determines speed of motor
#define ENB 33 // Enable B (right motor), PWM output value determines speed of motor

enum MotorDirection { FORWARD, BACKWARD, STOP };

// Initialize logger on default serial
// Pass `SerialBT` to log to BEACON
Logger Logger(Serial);

void setup() {
    Serial.begin(9600); // Baud rate 9600 Bd
    delay(3000); // Allow serial monitor time to open

    beginBluetooth(); // Registers the BT callback and listens for the beacon's connection

    // Motor driver pin setup, all OUTPUT, all LOW and 0 by default
    pinMode(IN1, OUTPUT);
    pinMode(IN2, OUTPUT);
    pinMode(IN3, OUTPUT);
    pinMode(IN4, OUTPUT);
    pinMode(ENA, OUTPUT);
    pinMode(ENB, OUTPUT);
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, LOW);
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, LOW);
    analogWrite(ENA, 0);
    analogWrite(ENB, 0);
}

void loop() {
    // if (bt_connected && SerialBT.available()) {
    //     char cmd[BT_TOKEN_MAX_LEN];
    //     readBluetoothCommand(cmd, sizeof(cmd));
    //     Logger.debug(cmd);
    // }
    setLeftMotor(255, FORWARD);
    setRightMotor(255, FORWARD);
    delay(1000);
    setLeftMotor(0, STOP);
    setRightMotor(0, STOP);
    delay(1000);
    setLeftMotor(30, BACKWARD);
    setRightMotor(30, BACKWARD);
    delay(1000);
}

/**
 * @brief Sets the PWM speed and direction of the left motor
 *
 * @param pwm PWM duty cycle for the motor's speed (0-255), ignored when `direction` is `STOP`
 * @param direction Direction for the motor to drive, `FORWARD`, `BACKWARD`, or `STOP`
 */
void setLeftMotor(uint8_t pwm, MotorDirection direction) {
    digitalWrite(IN1, direction == FORWARD ? HIGH : LOW);
    digitalWrite(IN2, direction == BACKWARD ? HIGH : LOW);
    analogWrite(ENA, direction == STOP ? 0 : pwm);
}

/**
 * @brief Sets the PWM speed and direction of the right motor
 *
 * @param pwm PWM duty cycle for the motor's speed (0-255), ignored when `direction` is `STOP`
 * @param direction Direction for the motor to drive, `FORWARD`, `BACKWARD`, or `STOP`
 */
void setRightMotor(uint8_t pwm, MotorDirection direction) {
    digitalWrite(IN3, direction == FORWARD ? HIGH : LOW);
    digitalWrite(IN4, direction == BACKWARD ? HIGH : LOW);
    analogWrite(ENB, direction == STOP ? 0 : pwm);
}

