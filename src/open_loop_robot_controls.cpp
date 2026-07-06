// #include <Arduino.h>

// #define MOTOR1_PIN1 5
// #define MOTOR1_PIN2 17
// #define ENABLE1_PIN 16
// #define MOTOR2_PIN1 26
// #define MOTOR2_PIN2 27
// #define ENABLE2_PIN 14

// // PWM properties
// const int freq = 30000;
// const int pwmChannel = 0;
// const int resolution = 8;
// int dutyCycle = 160;

// void setup() {
//   pinMode(MOTOR1_PIN1, OUTPUT);
//   pinMode(MOTOR1_PIN2, OUTPUT);
//   pinMode(ENABLE1_PIN, OUTPUT);

//   pinMode(MOTOR2_PIN1, OUTPUT);
//   pinMode(MOTOR2_PIN2, OUTPUT);
//   pinMode(ENABLE2_PIN, OUTPUT);

//   // configue LEDC PWM
//   ledcAttachChannel(ENABLE1_PIN, freq, resolution, pwmChannel);
//   ledcAttachChannel(ENABLE2_PIN, freq, resolution, pwmChannel);

//   Serial.begin(9600);

//   Serial.print("Testing DC Motor...");
// }

// void motors_forward() {
//     digitalWrite(MOTOR1_PIN1, LOW);
//     digitalWrite(MOTOR1_PIN2, HIGH);
//     digitalWrite(MOTOR2_PIN1, HIGH);
//     digitalWrite(MOTOR2_PIN2, LOW);
// }

// void motors_backward() {
//     digitalWrite(MOTOR1_PIN1, HIGH);
//     digitalWrite(MOTOR1_PIN2, LOW);
//     digitalWrite(MOTOR2_PIN1, LOW);
//     digitalWrite(MOTOR2_PIN2, HIGH);
// }

// void motors_stop() {
//     digitalWrite(MOTOR1_PIN1, LOW);
//     digitalWrite(MOTOR1_PIN2, LOW);
//     digitalWrite(MOTOR2_PIN1, LOW);
//     digitalWrite(MOTOR2_PIN2, LOW);
// }

// void loop() {
//     ledcWrite(ENABLE1_PIN, dutyCycle);
//     ledcWrite(ENABLE2_PIN, dutyCycle);

//     Serial.println("Forwards");
//     motors_forward();
//     delay(1000);
    
//     Serial.println("Stopped");
//     motors_stop();
//     delay(500);

// }
