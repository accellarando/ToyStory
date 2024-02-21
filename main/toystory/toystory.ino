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
 
 int pins[] = {FLUFFY, SWEATER, POLAR, DINO, PUPPY, EMPTY};
 int angles[] = {5, 5, 5, 5, 5, 5};
 int numberAnimals = sizeof(pins)/sizeof(int);
 
//int led = 13;                // the pin that the LED is atteched to
int sensor = 2;              // the pin that the sensor is atteched to
int state = HIGH;             // by default, no motion detected
int val = 0;                 // variable to store the sensor status (value)

// called this way, it uses the default address 0x40
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();
// you can also call it with a different address you want
//Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x41);
// you can also call it with a different address and I2C interface
//Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x40, Wire);

// Depending on your servo make, the pulse width min and max may vary, you 
// want these to be as small/large as possible without hitting the hard stop
// for max range. You'll have to tweak them as necessary to match the servos you
// have!
#define SERVOMIN  150 // This is the 'minimum' pulse length count (out of 4096)
#define SERVOMAX  600 // This is the 'maximum' pulse length count (out of 4096)
#define USMIN  600 // This is the rounded 'minimum' microsecond length based on the minimum pulse of 150
#define USMAX  2400 // This is the rounded 'maximum' microsecond length based on the maximum pulse of 600
#define SERVO_FREQ 50 // Analog servos run at ~50 Hz updates

// our servo # counter
uint8_t servonum = 0;

void setup() {
  //pinMode(led, OUTPUT);      // initalize LED as an output
  pinMode(sensor, INPUT);    // initialize sensor as an input
  Serial.begin(9600);        // initialize serial
  Serial.println("8 channel Servo test!");

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

  delay(10);
}

void moveMe(int animalPinNumber, int angle ){
  
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
int oldVal = HIGH;
void loop(){
  delay(10); //calm down
  val = digitalRead(sensor);   // read sensor value
  
  currentTime = millis();
  
  // on rising edge, note current time and wait DELAY_MS before triggering more movement.
  // on falling edge of val, stop all motion. (active low)

  if(val == HIGH && oldVal == LOW){ //posedge
    triggerTime = millis();
  }
  if(val == LOW && oldVal == HIGH){ //negedge
    Serial.println("No motion!");
    state = HIGH;
  }
  oldVal = val;
  int dTime = currentTime - triggerTime;
  if(dTime > DELAY_MS)
    state = LOW;
    
  //Servo shit
  //servoTestSequence();
  if(state == LOW){
    // Choose which animal to wiggle
    int index = random(0, numberAnimals-1);
    Serial.println("Moving animal %d on pin %d by %d degrees", index, pins[index], angles[index]);
    moveMe(pins[index], angles[index]);
  }
  else{ // ie, motion is detected
    Serial.println("Stopping all animals.");
    stopAll();
  }
}
