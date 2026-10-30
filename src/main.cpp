#include <Arduino.h>

#include "soc/gpio_struct.h"

#include "pins.hpp"
#include "helpers.hpp"
#include "ros_entities.hpp"

static unsigned long lastAgentRead   = 0; // last microros agent check timestamp
static unsigned long lastControlRead = 0; // last motor control timestamp
static unsigned long lastEncoderRead = 0; // last encoder read timestamp
// static unsigned long lastPrint = 0;

// PWM properties
const int freq = 30000;
const int pwmChannel = 0;
const int resolution = 8;
// static int dutyCycle = 0; // moved to control loop section


// Encoder state
struct EncoderSnapshot {
    volatile int count = 0; // todo: integer overflow/underflow, what do
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

// Controller properties
static volatile double linearVelocity = 0.0;
static volatile double angularVelocity = 0;
static double errprev1 = 0;
static double errint1 = 0;
static double errprev2 = 0;
static double errint2 = 0;

// Interrupt values
volatile bool moving = true;
volatile bool pressed = false;
volatile unsigned long lastTimeBtnPressed = 0;

// ROS Callbacks
// Since callbacks have no this pointer, we do this thing

std::unique_ptr<RosEntities> ros_entities = nullptr;

void cmd_vel_callback(const void * msgin) {
    if (msgin == nullptr) return;

    const auto * twist_msg = static_cast<const geometry_msgs__msg__Twist *>(msgin);
    linearVelocity = twist_msg->linear.x;
    angularVelocity = twist_msg->angular.z;
    // digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
}
void diagnostics_callback(rcl_timer_t * timer, int64_t last_call_time) {
    (void)timer;
    (void)last_call_time;

    if (ros_entities == nullptr) {
        while (1) {
            digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
            delay(5000); // 5 second / long blink = not able to send diagnostics somehow
        }
    }

    digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
    rosidl_runtime_c__String__assign(
        &(ros_entities->diagnostics_msg.data), 
        "motor controller running");

    RCSOFTCHECK(rcl_publish(
        &(ros_entities->diagnostics_publisher),
        &(ros_entities->diagnostics_msg),
        nullptr));
}

// Interrupts
// rising edge of the phase A signal (pos/vel) of the 1 motor
void IRAM_ATTR HandleEncoderInterrupt1()
{
    digitalRead(ENCPULSEB1_PIN) == LOW ? ++m1.count : --m1.count;
    // gpio_get_level((gpio_num_t)ENCPULSEB1_PIN) == LOW ? ++m1.count : --m1.count;
    // (GPIO.in & (1UL << ENCPULSEB1_PIN)) == LOW ? ++m1.count : --m1.count;
}

// rising edge of the phase A signal (pos/vel) of the 2 motor
void IRAM_ATTR HandleEncoderInterrupt2()
{
    // forwards = high because the motor is flipped around
    digitalRead(ENCPULSEB2_PIN) == HIGH ? ++m2.count : --m2.count;
}

void IRAM_ATTR HandleBtnInterrupt() 
{
    unsigned long now = millis();
    if (now - lastTimeBtnPressed > 100) {
        // Serial.println("pushed button!!!");
        moving = !moving;
        pressed = true;
    }
    lastTimeBtnPressed = now;
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
    pinMode(BTN_PIN, INPUT_PULLUP);
    pinMode(POTEN_PIN, INPUT);
    pinMode(LED_BUILTIN, OUTPUT);

    // Attach intterupts to pullup resistor pins
    attachInterrupt(digitalPinToInterrupt(ENCPULSEA1_PIN), HandleEncoderInterrupt1, RISING);
    attachInterrupt(digitalPinToInterrupt(ENCPULSEA2_PIN), HandleEncoderInterrupt2, RISING);
    attachInterrupt(digitalPinToInterrupt(BTN_PIN), HandleBtnInterrupt, RISING); // debug button

    // configure debug potentiometer
    analogSetAttenuation(ADC_11db); // 3.3V debug potentiometer

    // configue LEDC PWM
    ledcAttachChannel(ENABLE1_PIN, freq, resolution, pwmChannel);
    ledcAttachChannel(ENABLE2_PIN, freq, resolution, pwmChannel);


    // Using Serial as our way to publish ROS2 messages
    Serial.begin(115200);
    
    // configure microros hardware
    set_microros_serial_transports(Serial);
    
    delay(2000);
    
    // Serial.println("Testing DC Motor...");
}



void loop()
{
    unsigned long now = micros();

    // Checking microros2 agent loop
    checkAgentAndSpin(ros_entities); // also reconnects to microros

    // Begin motor control loop, run every 20 ms (50 hz)
    if (now - lastControlRead >= 20000) {
        double dt = (now - lastControlRead) * 1e-6;
        lastControlRead = now;
        int dutyCycle1 = 0;
        int dutyCycle2 = 0;
        bool pid = true;
        
        if (!moving ) {
            motors_stop();
        }
        else{
            if (pid) {
                // PID control Motor 1 (left wheel)
                double targetVelocity1 = linearVelocity; // todo: change this to use inv. kinematics 
                double err1 = targetVelocity1 - m1.velocity;

                double kp_1 = 300.0;
                double ki_1 = 150.0; // todo: p is ok, but try higher i
                double kd_1 = 10.0;

                double dedt1 = (err1-errprev1)/dt;
                errint1 += err1*dt;

                double u1 = kp_1*err1  + ki_1*errint1 + kd_1*dedt1;

                // double newTargetVel1 = targetVelocity1 + u1;
                dutyCycle1 = (int)constrain(u1, 0, 255);

                if (targetVelocity1 > 0.0)
                    m1_forward();
                else
                    m1_backward();

                // PID control Motor 2 (right wheel)
                double targetVelocity2 = linearVelocity; // todo: change this to use inv. kinematics
                double err2 = targetVelocity2 - m2.velocity;

                double kp_2 = 300.0;
                double ki_2 = 150.0;
                double kd_2 = 10.0;

                double dedt2 = (err2-errprev2)/dt;
                errint2 += err2*dt;

                double u2 = kp_2*err2 + ki_2*errint2 + kd_2*dedt2;
                
                //double newTargetVel2 = targetVelocity2 + u2;
                dutyCycle2 = (int)constrain(u2, 0, 255);

                if (targetVelocity2 > 0.0)
                    m2_forward();
                else
                    m2_backward();


                // csv format for graphs
                // target1,actual1,target2,actual2,u1,duty1,u2,duty2
                
                // m1/m2 vels
                // Serial.print("pid: ");
                // Serial.print(targetVelocity1);
                // Serial.print(",");
                // Serial.print(m1.velocity);
                // Serial.print(",");

                // Serial.print(targetVelocity2);
                // Serial.print(",");
                // Serial.print(m2.velocity);
                // Serial.print(",");

                // // m1/m2 u vals and duty cycles
                // Serial.print(u1);
                // Serial.print(",");
                // Serial.print(dutyCycle1);            
                // Serial.print(",");
            
                // Serial.print(u2);
                // Serial.print(",");
                // Serial.print(dutyCycle2);
                // Serial.print(",");
                // Serial.println("");
            }
            else {
                dutyCycle1 = 100; //m1_vel_to_duty_cycle(linearVelocity);
                if (linearVelocity > 0.0)
                    m1_forward();
                else
                    m1_backward();

                dutyCycle2 = 100; //m2_vel_to_duty_cycle(linearVelocity);
                if (linearVelocity > 0.0)
                    m2_forward();
                else
                    m2_backward();

                // Serial.print("nopid: ");
                // Serial.print(linearVelocity);
                // Serial.print(",");
                // Serial.print(m1.velocity);
                // Serial.print(",");
                // Serial.print(dutyCycle1);
                // Serial.print(",");

                // Serial.print(linearVelocity);
                // Serial.print(",");
                // Serial.print(m2.velocity);
                // Serial.print(",");
                // Serial.print(dutyCycle2);
                // Serial.print(",");
                // Serial.println();
            }
        }
        
        
        ledcWrite(ENABLE1_PIN, dutyCycle1);
        ledcWrite(ENABLE2_PIN, dutyCycle2);
    }

    now = micros();
    
    ///////////////////////////////////////////////////////

    // Begin encoder read loop, run every 10 ms (100 hz)
    if (now - lastEncoderRead >= 10000) {
        double dt = (now - lastEncoderRead) * 1e-6;
        lastEncoderRead = now;

        noInterrupts();
        int m1_count = m1.count;
        int m2_count = m2.count;
        interrupts();

        if (m1_count != m1.prev_count || m2_count != m2.prev_count) {
            double rot1 = m1_count * 360.0 / (12 * 90); // 90 is encoder motor ratio. 12 is encoder motor ppr.
            double prev_rot1 = m1.prev_count * 360.0 / (12 * 90);

            // use the counts and not the degrees for distance. 360 deg  = 1080 count
            int delta1 = m1_count - m1.prev_count;
            double distance_1 = delta1 * METERS_PER_COUNT;
            m1.position += distance_1; // unnecessary, but nice sanity check

            // and the instantaneous velocity
            double lin_vel1 = distance_1 / dt;
            m1.velocity = m1.velocity * 0.8
                        + lin_vel1    * 0.2;   // low-pass filter for random jolt avoidance

            // and for the angular velocity (of this wheel instant)
            m1.omega = delta1 * (2 * PI / 1080) / dt;

            // // Serial.print("enable1: ");
            // // Serial.print(std::fmod(rot1, 360.0));
            // // Serial.print("°  \t");
            // Serial.print("count=");
            // Serial.print(m1_count);
            // Serial.print(" delta=");
            // Serial.print(delta1);
            // // Serial.print(" dt: ");
            // // Serial.print(dt, 6);
            // // Serial.print(" secs");
            // Serial.print("\t lin_vel=");
            // Serial.print(lin_vel1);
            // Serial.print(" velocity=");
            // Serial.print(m1.velocity);
            // Serial.print(" omega=");
            // Serial.print(m1.omega);
            // Serial.print("\t so far: ");
            // Serial.print(m1.position, 4);
            // Serial.println("");

            ///////////////////////////////////////////
            // m2
            double rot2 = m2_count * 360.0 / (12 * 90);
            double prev_rot2 = m1.prev_count * 360.0 / (12 * 90);
            // position
            int delta2 = m2_count - m2.prev_count;
            double distance_2 = delta2 * METERS_PER_COUNT;
            m2.position += distance_2;
            // linear velocity
            double lin_vel2 = distance_2 / dt;
            m2.velocity = m2.velocity * 0.8 +
                        + lin_vel2    * 0.2;
            // angular velocity (sorta)
            m2.omega = delta2 * (2 * PI / 1080) / dt;

            ///////////////////////////////////////////
            // reset prev counts
            m1.prev_count = m1_count;    
            m2.prev_count = m2_count;
        }

        pressed = false;
    }
}