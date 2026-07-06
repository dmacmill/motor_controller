// #include <Arduino.h>

// #define MOTOR1_PIN1 27
// #define MOTOR1_PIN2 26
// #define ENABLE1_PIN 14

// // PWM properties
// const int freq = 30000;
// const int pwmChannel = 0;
// const int resolution = 8;
// int dutyCycle = 140;

// void setup() {
//   pinMode(MOTOR1_PIN1, OUTPUT);
//   pinMode(MOTOR1_PIN2, OUTPUT);
//   pinMode(ENABLE1_PIN, OUTPUT);

//   // configue LEDC PWM
//   ledcAttachChannel(ENABLE1_PIN, freq, resolution, pwmChannel);

//   Serial.begin(9600);

//   Serial.print("Testing DC Motor...");
// }

// void loop() {
//   // todo: the first loop doesn't seem to want to move forward. figure out why
//   Serial.println("Moving Forward");
//   digitalWrite(MOTOR1_PIN1, LOW);
//   digitalWrite(MOTOR1_PIN2, HIGH);
//   delay(1000);

//   Serial.println("Motor stopped");
//   digitalWrite(MOTOR1_PIN1, LOW);
//   digitalWrite(MOTOR1_PIN2, LOW);
//   delay(1000);

//   Serial.println("Moving Backwards");
//   digitalWrite(MOTOR1_PIN1, HIGH);
//   digitalWrite(MOTOR1_PIN2, LOW); 
//   delay(1000);

//   Serial.println("Motor stopped");
//   digitalWrite(MOTOR1_PIN1, LOW);
//   digitalWrite(MOTOR1_PIN2, LOW);
//   delay(1000);

//   //Move DC motor forward with increasing speed
//   digitalWrite(MOTOR1_PIN1, LOW);
//   digitalWrite(MOTOR1_PIN2, HIGH);
//   while (dutyCycle <= 255){
//     ledcWrite(ENABLE1_PIN, dutyCycle);   
//     Serial.print("Forward with duty cycle: ");
//     Serial.println(dutyCycle);
//     dutyCycle = dutyCycle + 5;
//     delay(500);
//   }
//   dutyCycle = 140;

//   Serial.println("Motor stopped");
//   digitalWrite(MOTOR1_PIN1, LOW);
//   digitalWrite(MOTOR1_PIN2, LOW);
//   delay(1000);
// }
