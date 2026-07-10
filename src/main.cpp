#include <Arduino.h>

#define MOTOR1_PIN1 5
#define MOTOR1_PIN2 17
#define ENABLE1_PIN 16
#define ENCPULSEA1_PIN 19
#define ENCPULSEB1_PIN 18

#define MOTOR2_PIN1 26
#define MOTOR2_PIN2 27
#define ENABLE2_PIN 14
#define ENCPULSEA2_PIN 35
#define ENCPULSEB2_PIN 32

static unsigned long lastControl = 0; // last control loop timestamp
static unsigned long lastPrint = 0; 

// PWM properties
const int freq = 30000;
const int pwmChannel = 0;
const int resolution = 8;
int dutyCycle = 200;

/*
Duty cycles:
160 => 0.13 m/s (estimate) => 0.133 m/s (actual)
180 => 0.17 m/s (estimate) => 0.25 m/s (actual)
200 => 0.11 m/s (estimate) => 0.33 m/s (actual)
*/

// Encoder state
struct EncoderSnapshot {
    volatile int count = 0;
    int prev_count = 0;

    double position = 0;
    double velocity = 0;
    double omega = 0;
};

EncoderSnapshot m1;
EncoderSnapshot m2;

// Wheel properties
const double WHEEL_DIAMETER = 0.06;                             // meters
const double WHEEL_CIRCUMFRENCE = WHEEL_DIAMETER * 3.141592653; // meters
const double METERS_PER_COUNT = WHEEL_CIRCUMFRENCE / 1080;      // meters / count
double distance_traveled_1 = 0.0;
double distance_traveled_2 = 0.0;

// rising edge of the phase A signal (pos/vel) of the 1 motor
void HandleEncoderInterrupt1()
{
    digitalRead(ENCPULSEB1_PIN) == LOW ? ++m1.count : --m1.count;
}

// rising edge of the phase A signal (pos/vel) of the 2 motor
void HandleEncoderInterrupt2()
{
    // forwards = high because the motor is flipped around
    digitalRead(ENCPULSEB2_PIN) == HIGH ? ++m2.count : --m2.count;
}

void setup()
{
    pinMode(MOTOR1_PIN1, OUTPUT);
    pinMode(MOTOR1_PIN2, OUTPUT);
    pinMode(ENABLE1_PIN, OUTPUT);
    pinMode(ENCPULSEA1_PIN, INPUT_PULLUP);
    pinMode(ENCPULSEA2_PIN, INPUT_PULLUP);

    pinMode(MOTOR2_PIN1, OUTPUT);
    pinMode(MOTOR2_PIN2, OUTPUT);
    pinMode(ENABLE2_PIN, OUTPUT);
    pinMode(ENCPULSEB1_PIN, INPUT_PULLUP);
    pinMode(ENCPULSEB2_PIN, INPUT_PULLUP);

    // Attach intterupts to pullup resistor pins
    attachInterrupt(digitalPinToInterrupt(ENCPULSEA1_PIN), HandleEncoderInterrupt1, RISING);
    attachInterrupt(digitalPinToInterrupt(ENCPULSEA2_PIN), HandleEncoderInterrupt2, RISING);

    // configue LEDC PWM
    ledcAttachChannel(ENABLE1_PIN, freq, resolution, pwmChannel);
    ledcAttachChannel(ENABLE2_PIN, freq, resolution, pwmChannel);

    // Begin at 115200 baud
    Serial.begin(115200);
    delay(2000);
    Serial.println("Testing DC Motor...");
}

void motors_forward()
{
    digitalWrite(MOTOR1_PIN1, LOW);
    digitalWrite(MOTOR1_PIN2, HIGH);
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

void motors_stop()
{
    digitalWrite(MOTOR1_PIN1, LOW);
    digitalWrite(MOTOR1_PIN2, LOW);
    digitalWrite(MOTOR2_PIN1, LOW);
    digitalWrite(MOTOR2_PIN2, LOW);
}

void loop()
{
    ledcWrite(ENABLE1_PIN, dutyCycle);
    ledcWrite(ENABLE2_PIN, dutyCycle);

    bool moving = true;

    // Get timestamps and state of motor encoders
    unsigned int now = micros();

    // Begin control loop, run every 10 ms
    if (now - lastControl >= 10000) {
        double dt = (now - lastControl) * 1e-6;
        lastControl = now;

        noInterrupts();
        int m1_count = m1.count;
        int m2_count = m2.count;
        interrupts();

        if (m1_count != m1.prev_count || m2_count != m2.prev_count) {
            double rot1 = m1_count * 360.0 / (12 * 90); // 90 is encoder motor ratio. 12 is encoder motor ppr.
            double prev_rot1 = m1.prev_count * 360.0 / (12 * 90);

            // use the counts and not the degrees for distance. 360 deg  = 1080 count
            int delta = m1_count - m1.prev_count;
            double distance_1 = delta * METERS_PER_COUNT;
            distance_traveled_1 += distance_1;
            m1.position = distance_traveled_1; // unnecessary, but nice sanity check

            // and the instantaneous velocity
            // double lin_vel = distance_1 / (duration1 * 1e-6);
            double lin_vel = distance_1 / dt;
            m1.velocity = m1.velocity * 0.8
                        + lin_vel     * 0.2;   // low-pass filter for random jolt avoidance

            // and for the angular velocity (of this wheel instant)
            // double omega = delta * (2 * PI / 1080) / (duration1 * 1e-6);
            double omega = delta * (2 * PI / 1080) / dt;

            Serial.print("enable1: ");
            Serial.print(std::fmod(rot1, 360.0));
            Serial.print("°  \t");
            Serial.print("count=");
            Serial.print(m1_count);
            Serial.print(" delta=");
            Serial.print(delta);
            Serial.print(" dt: ");
            Serial.print(dt, 6);
            Serial.print(" secs");
            Serial.print("\t lin_vel=");
            Serial.print(lin_vel);
            Serial.print(" velocity=");
            Serial.print(m1.velocity);
            Serial.print(" omega=");
            Serial.print(omega);
            Serial.print("\t so far: ");
            Serial.print(distance_traveled_1, 4);
            Serial.println("");


            // m2
            // float rot2 = m2_count * 360.0 / (12 * 90);
            // float prev_rot2 = pm2_count * 360.0 / (12 * 90);

            // double distance_2 = (rot2 - prev_rot2) * (WHEEL_CIRCUMFRENCE / 360.0);

            // Serial.print("\t|| enable2: ");
            // Serial.print(std::fmod(rot2, 360.0));
            // Serial.print("°  \t");
            // Serial.println(m2_count);

            m1.prev_count = m1_count;    
            m2.prev_count = m2_count;
        }
    }


    if (!moving) {
        motors_stop();
    }
    else {
        motors_forward();
    }
}