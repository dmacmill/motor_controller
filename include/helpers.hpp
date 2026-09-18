#pragma once

#include <Arduino.h>
#include <cmath>

#include "pins.hpp"

/// dutyCycles <-> velocity converters

/*worked for the old L298N*/
// 
// double m1_duty_cycle_to_vel(int duty_cycle) {
//     int d = duty_cycle > 255 ? 255 : duty_cycle;
//     return 0.0071428571*d - 1.0928571345;
// }

// double m2_duty_cycle_to_vel(int duty_cycle) {
//     int d = duty_cycle > 255 ? 255 : duty_cycle;
//     return 0.0055*d - 0.74;
// }

// // Used in control code (worked for the old L298N)
// int m1_vel_to_duty_cycle(double vel) {
//     vel = abs(vel);
//     int d = (int)std::round(140.0*vel + 153.0);
//     return d > 255 ? 255 : d;
// }

// int m2_vel_to_duty_cycle(double vel) {
//     vel = abs(vel);
//     int d = (int)std::round(181.818182*vel + 134.545454);
//     return d > 255 ? 255 : d;
//}


/*below works for TB6612FNG*/
int m1_vel_to_duty_cycle(double vel) {
    vel = abs(vel);
    int d = (int)std::round(160*vel + 124);
    return d > 255 ? 255 : d;
}

int m2_vel_to_duty_cycle(double vel) {
    vel = abs(vel);
    int d = (int)std::round(230*vel + 91);
    return d > 255 ? 255 : d;
}

/// motor helpers

void motors_forward()
{
    digitalWrite(MOTOR1_PIN1, LOW);
    digitalWrite(MOTOR1_PIN2, HIGH);
    digitalWrite(MOTOR2_PIN1, HIGH);
    digitalWrite(MOTOR2_PIN2, LOW);
}

void m1_forward()
{
    digitalWrite(MOTOR1_PIN1, LOW);
    digitalWrite(MOTOR1_PIN2, HIGH);
}


void m2_forward()
{
    digitalWrite(MOTOR2_PIN1, HIGH);
    digitalWrite(MOTOR2_PIN2, LOW);
}

void motors_backward()
{
    digitalWrite(MOTOR1_PIN1, HIGH);
    digitalWrite(MOTOR1_PIN2, LOW);
    digitalWrite(MOTOR2_PIN1, LOW);
    digitalWrite(MOTOR2_PIN2, HIGH);
}

void m1_backward()
{
    digitalWrite(MOTOR1_PIN1, HIGH);
    digitalWrite(MOTOR1_PIN2, LOW);
}


void m2_backward()
{
    digitalWrite(MOTOR2_PIN1, LOW);
    digitalWrite(MOTOR2_PIN2, HIGH);
}

void motors_stop()
{
    digitalWrite(MOTOR1_PIN1, LOW);
    digitalWrite(MOTOR1_PIN2, LOW);
    digitalWrite(MOTOR2_PIN1, LOW);
    digitalWrite(MOTOR2_PIN2, LOW);
}