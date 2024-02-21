/*  
	Arduino with PIR motion sensor
	For complete project details, visit: http://RandomNerdTutorials.com/pirsensor
	Modified by Rui Santos based on PIR sensor by Limor Fried
 */
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>


//Animal pins:
#define FLUFFY 0
#define SWEATER 2
#define POLAR 4
#define DINO 6
#define PUPPY 8
#define EMPTY 10

// State variables
#define MOVING	LOW
#define STOPPED HIGH
volatile int state = STOPPED;
volatile int edgeDetected = -1; // 0 = none, 1 = rising, -1 = falling. init'd to -1 so that it'll actually start moving

int pins[] = {FLUFFY, SWEATER, POLAR, DINO, PUPPY, EMPTY};
int angles[] = {5, 5, 5, 5, 5, 5};
int numberAnimals = sizeof(pins)/sizeof(int);

int sensor = 2;              // the pin that the sensor is attached to
int val = 0;                 // variable to store the sensor status (value)

// called this way, it uses the default address 0x40
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

// Depending on your servo make, the pulse width min and max may vary, you 
// want these to be as small/large as possible without hitting the hard stop
// for max range. You'll have to tweak them as necessary to match the servos you
// have!
#define SERVOMIN  150 // This is the 'minimum' pulse length count (out of 4096)
#define SERVOMAX  600 // This is the 'maximum' pulse length count (out of 4096)
#define USMIN  600 // This is the rounded 'minimum' microsecond length based on the minimum pulse of 150
#define USMAX  2400 // This is the rounded 'maximum' microsecond length based on the maximum pulse of 600
#define SERVO_FREQ 50 // Analog servos run at ~50 Hz updates

// Define the servo parameters
#define SERVO_FORWARD_PULSE_WIDTH  410
#define SERVO_STOP_PULSE_WIDTH     0
#define SERVO_BACKWARD_PULSE_WIDTH 205

// Define servo rotation parameters
#define STEP_DELAY    15  // Milliseconds per degree, approximate
#define NUM_ROTATIONS 3   // How many times an animal moves back and forth
#define STOP_DELAY    200 // How long to wait on stopped segments

// our servo # counter
uint8_t servonum = 0;

void setup() {
	//pinMode(led, OUTPUT);      // initalize LED as an output
	pinMode(sensor, INPUT);    // initialize sensor as an input
	Serial.begin(9600);        // initialize serial
	Serial.println("Hi queen");

	pwm.begin();

	/*
	 * In theory the internal oscillator (clock) is 25MHz but it really isn't
	 * that precise. You can 'calibrate' this by tweaking this number until
	 * you get the PWM update frequency you're expecting!
	 * The int.osc. for the PCA9685 chip is a range between about 23-27MHz and
	 * is used for calculating things like writeMicroseconds()
	 * Analog servos run at ~50 Hz updates, It is importaint to use an
	 * oscilloscope in setting the int.osc frequency for the I2C PCA9685 chip.
	 * 1) Attach the oscilloscope to one of the PWM signal pins and ground on
	 *    the I2C PCA9685 chip you are setting the value for.
	 * 2) Adjust setOscillatorFrequency() until the PWM update frequency is the
	 *    expected value (50Hz for most ESCs)
	 * Setting the value here is specific to each individual I2C PCA9685 chip and
	 * affects the calculations for the PWM update frequency. 
	 * Failure to correctly set the int.osc value will cause unexpected PWM results
	 */
	pwm.setOscillatorFrequency(27000000);
	pwm.setPWMFreq(SERVO_FREQ);  // Analog servos run at ~50 Hz updates

	// Register pin change interrupt for motion sensor
	attachInterrupt (digitalPinToInterrupt (sensor), motionISR, CHANGE);  // attach interrupt handler, both edges

	delay(10);
}

void moveMe(int animal, int angle){
	Serial.print("Moving animal ");
	Serial.println(animal);
	int delayTime = STEP_DELAY * angle;
	for(int i=0; i<NUM_ROTATIONS; i++){
		pwm.setPWM(animal, 0, SERVO_FORWARD_PULSE_WIDTH); //full speed forward
		delay(delayTime);
		pwm.setPWM(animal, 0, SERVO_STOP_PULSE_WIDTH); //stops
		if(edgeDetected == 1){
			return;
		}
		delay(STOP_DELAY);
		pwm.setPWM(animal, 0, SERVO_BACKWARD_PULSE_WIDTH); //full speed backward
		delay(delayTime);
		pwm.setPWM(animal, 0, SERVO_STOP_PULSE_WIDTH); //stops
		if(edgeDetected == 1){
			return;
		}
		delay(STOP_DELAY);
	}
}

void stopAll(){
	for(int i = 0; i < numberAnimals; i++){
		pwm.setPWM(pins[i], 0, 0); 
	}
}

// You can use this function if you'd like to set the pulse length in seconds
// e.g. setServoPulse(0, 0.001) is a ~1 millisecond pulse width. It's not precise!
// not used??
void setServoPulse(uint8_t n, double pulse) {
	double pulselength;

	pulselength = 1000000;   // 1,000,000 us per second
	pulselength /= SERVO_FREQ;   // Analog servos run at ~60 Hz updates
	Serial.print(pulselength); Serial.println(" us per period"); 
	pulselength /= 4096;  // 12 bits of resolution
	Serial.print(pulselength); Serial.println(" us per bit"); 
	pulse *= 1000000;  // convert input seconds to us
	pulse /= pulselength;
	Serial.println(pulse);
	pwm.setPWM(n, 0, pulse);
}

void servoTestSequence(){
	// Drive each servo one at a time using setPWM()
	Serial.println(servonum);
	for (uint16_t pulselen = SERVOMIN; pulselen < SERVOMAX; pulselen++) {
		pwm.setPWM(servonum, 0, pulselen);
	}

	delay(500);
	for (uint16_t pulselen = SERVOMAX; pulselen > SERVOMIN; pulselen--) {
		pwm.setPWM(servonum, 0, pulselen);
	}

	delay(500);

	servonum++;
	if (servonum > 10) servonum = 0; // Testing the first 10 servo channels
}
#define DELAY_MS 5000
int currentTime = 0;
int triggerTime = 0;

void motionISR(){
	if(digitalRead(sensor) == HIGH){
		Serial.println("Motion detected!");
		edgeDetected = 1;
		Serial.println("Stopping all animals.");
		// stopAll(); // moved out of interrupt context
	}
	else{
		Serial.println("Motion stopped.");
		// state = MOVING; // not yet
		edgeDetected = -1;
		triggerTime = millis();
	}
}

// int oldVal = HIGH;
// bool stopped = HIGH;
void loop(){
	currentTime = millis();
	int dTime = currentTime - triggerTime;

	if(edgeDetected == 1){
		stopAll();
		state = STOPPED;
		edgeDetected = 0;
	}
	if(edgeDetected == -1 && dTime > DELAY_MS){
		state = MOVING;
		edgeDetected = 0;
	}

	if(state == MOVING){
		// Choose which animal to wiggle
		int index = random(0, numberAnimals-1);
		Serial.print("Moving animal");
		Serial.print(index);
		Serial.print(" on pin ");
		Serial.print(pins[index]);
		Serial.print(" by ");
		Serial.print(angles[index]);
		Serial.println("degrees");
		//moveMe(pins[index], angles[index]);
		moveMe(10, 5);
	}
	else{ // ie, motion is detected
		  // only trigger on edges - not appropriate in this scope, moved to edge detection area
		/*
		   Serial.println("Stopping all animals.");
		   stopAll();
		 */
	}
}
