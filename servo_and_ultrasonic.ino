
#include "Servo.h"

// Create servo object
Servo myservo;

// Define pins
#define servoPin 9
#define echoPin 2
#define trigPin 3

long duration;
int distance;

int angle = 0;
int direction = 1;  // 1 = forward, -1 = backward

void setup() {
  myservo.attach(servoPin);

  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);

  Serial.begin(9600);
}

void loop() {
  // Move servo to current angle
  myservo.write(angle);

  // Trigger ultrasonic sensor
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);

  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  // Read echo with timeout
  duration = pulseIn(echoPin, HIGH, 30000);

  // Calculate and display distance
  Serial.print("Angle: ");
  Serial.print(angle);
  Serial.print(" degrees | Distance: ");

  if (duration > 0) {
    distance = duration * 0.034 / 2;
    Serial.print(distance);
    Serial.println(" cm");
  } else {
    Serial.println("No echo");
  }

  // Move servo to the next angle
  angle += direction;

  // Reverse direction at each endpoint
  if (angle >= 180) {
    angle = 180;
    direction = -1;
  } else if (angle <= 0) {
    angle = 0;
    direction = 1;
  }

  // Control sweep speed
  delay(20);
}
