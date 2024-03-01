#include <Servo.h>//ultrasonic sensor connections
int trig=A2;
int echo=A3;
int dt=10;

// Motor A connections
int enA = 8;
int in1 = 10;
int in2 = 9;
// Motor B connections
int enB = 4;
int in3 = 6;
int in4 = 5;


//int distance,duration;
void setup() {
  // put your setup code here, to run once:
    pinMode(trig,OUTPUT);
    pinMode(echo,INPUT);
    Serial.begin(9600);
    pinMode(A0,HIGH);

  
  	// Set all the motor control pins to outputs
	pinMode(enA, OUTPUT);
	pinMode(enB, OUTPUT);
	pinMode(in1, OUTPUT);
	pinMode(in2, OUTPUT);
	pinMode(in3, OUTPUT);
	pinMode(in4, OUTPUT);
	
	// Turn off motors - Initial state
	digitalWrite(in1, LOW);
	digitalWrite(in2, LOW);
	digitalWrite(in3, LOW);
	digitalWrite(in4, LOW);
}

//This code is written to calculate the DISTANCE using ULTRASONIC SENSOR

int calc_dis()
{
    int duration,distance;
    digitalWrite(trig,HIGH);
    delay(dt);
    digitalWrite(trig,LOW);
    duration=pulseIn(echo,HIGH);
    distance = (duration/2) / 29.1;
    return distance;
}

void loop() {
	directionControl();
	delay(1000);
	speedControl();
	delay(1000);
}

// This function lets you control spinning direction of motors
void directionControl() {
	// Set motors to maximum speed
	// For PWM maximum possible values are 0 to 255
	analogWrite(enA, 255);
	analogWrite(enB, 255);

	// Turn on motor A & B
	digitalWrite(in1, HIGH);
	digitalWrite(in2, LOW);
	digitalWrite(in3, HIGH);
	digitalWrite(in4, LOW);
	delay(2000);
	
	// Now change motor directions
	digitalWrite(in1, LOW);
	digitalWrite(in2, HIGH);
	digitalWrite(in3, LOW);
	digitalWrite(in4, HIGH);
	delay(2000);
	
	// Turn off motors
	digitalWrite(in1, LOW);
	digitalWrite(in2, LOW);
	digitalWrite(in3, LOW);
	digitalWrite(in4, LOW);
}

// This function lets you control speed of the motors
void speedControl() {
	// Turn on motors
	digitalWrite(in1, LOW);
	digitalWrite(in2, HIGH);
	digitalWrite(in3, LOW);
	digitalWrite(in4, HIGH);
	
	// Accelerate from zero to maximum speed
	for (int i = 0; i < 256; i++) {
		analogWrite(enA, i);
		analogWrite(enB, i);
		delay(20);
	}
	
	// Decelerate from maximum speed to zero
	for (int i = 255; i >= 0; --i) {
		analogWrite(enA, i);
		analogWrite(enB, i);
		delay(20);
	}
	
	// Now turn off motors
	digitalWrite(in1, LOW);
	digitalWrite(in2, LOW);
	digitalWrite(in3, LOW);
	digitalWrite(in4, LOW);
    }
// Define the pin for the servo
#define SERVO_PIN=7

// Create a servo object
Servo servo;

int angle = 0;
int angleIncrement = 1;
unsigned long previousMillis = 0;
unsigned long interval = 1000; // Interval in milliseconds


// Main program
int main() {
  setup();{// Call the setup function
    // Attach the servo to the pin
  servo.attach(7);
}

  while (true) {
    unsigned long currentMillis = millis();
    if (currentMillis - previousMillis >= interval) {
      // Save the last time you blinked the LED
      previousMillis = currentMillis;

      // Move the servo
      servo.write(angle);
      angle += angleIncrement;
      
      // Reverse direction when reaching the limits
      if (angle <= 0 || angle >= 180) {
        angleIncrement *= -1;
      }
    }
  }

  return 0;
}
