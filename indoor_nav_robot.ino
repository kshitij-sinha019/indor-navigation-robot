#include "ultrasonic.h"
#include "motor.h"

UltrasonicSensor sensor(7, 8);
MotorController motors(5, 6, 9, 10);

void setup() {
    Serial.begin(9600);
    sensor.begin();
    motors.begin();
}

void loop() {
    int distance = sensor.getDistance();
    Serial.print("Distance: ");
    Serial.println(distance);
    
    if (distance < 20) {
        motors.stop();
        delay(500);
        motors.turnRight();
        delay(700);
    } else {
        motors.forward();
    }
    delay(100);
}

#ifndef ULTRASONIC_H
#define ULTRASONIC_H

#include <Arduino.h>

class UltrasonicSensor {
public:
    UltrasonicSensor(int trigPin, int echoPin);
    void begin();
    int getDistance();

private:
    int trigPin, echoPin;
};

#endif

#include "ultrasonic.h"

UltrasonicSensor::UltrasonicSensor(int trigPin, int echoPin) {
    this->trigPin = trigPin;
    this->echoPin = echoPin;
}

void UltrasonicSensor::begin() {
    pinMode(trigPin, OUTPUT);
    pinMode(echoPin, INPUT);
}

int UltrasonicSensor::getDistance() {
    digitalWrite(trigPin, LOW);
    delayMicroseconds(2);
    digitalWrite(trigPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigPin, LOW);
    long duration = pulseIn(echoPin, HIGH);
    return duration * 0.034 / 2;
}

#ifndef MOTOR_H
#define MOTOR_H

#include <Arduino.h>

class MotorController {
public:
    MotorController(int left1, int left2, int right1, int right2);
    void begin();
    void forward();
    void stop();
    void turnRight();

private:
    int left1, left2, right1, right2;
};

#endif

#include "motor.h"

MotorController::MotorController(int left1, int left2, int right1, int right2) {
    this->left1 = left1;
    this->left2 = left2;
    this->right1 = right1;
    this->right2 = right2;
}

void MotorController::begin() {
    pinMode(left1, OUTPUT);
    pinMode(left2, OUTPUT);
    pinMode(right1, OUTPUT);
    pinMode(right2, OUTPUT);
}

void MotorController::forward() {
    digitalWrite(left1, HIGH);
    digitalWrite(left2, LOW);
    digitalWrite(right1, HIGH);
    digitalWrite(right2, LOW);
}

void MotorController::stop() {
    digitalWrite(left1, LOW);
    digitalWrite(left2, LOW);
    digitalWrite(right1, LOW);
    digitalWrite(right2, LOW);
}

void MotorController::turnRight() {
    digitalWrite(left1, HIGH);
    digitalWrite(left2, LOW);
    digitalWrite(right1, LOW);
    digitalWrite(right2, HIGH);
}
