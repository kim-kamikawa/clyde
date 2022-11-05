#include "mbed.h"
#include "HCSR04.h"
#include "QEI.h"
#include "FXOS8700Q.h"
#include "PID.h"
#include <math.h>

#define PI 3.14159265358979323846f

Serial pc(USBTX, USBRX);
DigitalOut redLED(LED_RED);

// Configured for the FRDM-K64F with onboard FXOS8700CQ
I2C i2c(PTE25, PTE24);
FXOS8700QAccelerometer acc(i2c, FXOS8700CQ_SLAVE_ADDR1);
FXOS8700QMagnetometer mag(i2c, FXOS8700CQ_SLAVE_ADDR1);
motion_data_units_t acc_data, mag_data;

double heading = 0;                                 
double currentHeading = 0;
enum headingDirection {TURN_LEFT, TURN_RIGHT};
enum drivingDirection {DRIVING_FORWARD, DRIVING_BACKWARD} direction;

// Left motor
PwmOut enA(PTC2);                                       // D6
DigitalOut in1(PTC3);                                   // D7
DigitalOut in2(PTC12);                                  // D8
float leftSpeed = 0.2;                                  // Duty-cycle as percentage: 1.0 to 0.0
float leftTurnRate = 0.6;                               // speed for swing turns

// Right motor
DigitalOut in3(PTC4);                                   // D9
DigitalOut in4(PTD0);                                   // D10
PwmOut enB(PTD2);                                       // D11
float rightSpeed = 0.2;                                 // Duty-cycle as percentage: 1.0 to 0.0
float rightTurnRate = 0.6;                              // speed for swing turns

float fullSpeed = 1.0;
enum motorPosition {LEFT, RIGHT};
enum motorDirection {FORWARD, REVERSE};

// Encoder
int PPR = 341;                                          // pulses per revolution
int encoderPPR = 4*PPR;                                 // encoder pulses per revolution
float wheelCircumference = PI*65;                       // diameter=65mm
QEI leftMotor(PTB23, PTA2, NC, PPR, QEI::X4_ENCODING);  // A(D4) B(D6) 
QEI rightMotor(PTD3, PTD1, NC, PPR, QEI::X4_ENCODING);  // A(D12) B(D13)

// HC-SR04
HCSR04 sensor(PTC16, PTC17); //Echo(D0) Trigger(D1)
float sampleTime = 0.5;
Ticker ticker;


void calc() {
    sensor.startMeasurement();
}

void Motor(motorPosition position, motorDirection direction)
{
    if (position == LEFT) 
        if (direction == FORWARD) {
            // CCW
            in1 = 1;
            in2 = 0;
        } 
        else {
            // CW
            in1 = 0;
            in2 = 1;
        }
    else // RIGHT
        if (direction == FORWARD) {
            // CW
            in3 = 0;
            in4 = 1;
        } 
        else {
            // CCW
            in3 = 1;
            in4 = 0;
        }
}

void Stop()
{
    // left motor off
    in1 = 0;
    in2 = 0;
    // right motor off
    in3 = 0;
    in4 = 0;
}

void GoStraight(drivingDirection direction)
{
    if (direction == DRIVING_FORWARD) {
        Motor(LEFT, FORWARD);
        enA = leftSpeed;
        Motor(RIGHT, FORWARD);
        enB = rightSpeed;
    }
    else {
        Motor(LEFT, REVERSE);
        enA = leftSpeed;
        Motor(RIGHT, REVERSE);
        enB = rightSpeed;
    }
}

void GoPointLeft(drivingDirection direction)
{
    if (direction == DRIVING_FORWARD) {
        Motor(LEFT, REVERSE);
        enA = leftSpeed;
        Motor(RIGHT, FORWARD);
        enB = rightSpeed;
    }
    else {
        Motor(LEFT, FORWARD);
        enA = leftSpeed;
        Motor(RIGHT, REVERSE);
        enB = rightSpeed;
    }
}

void GoPointRight(drivingDirection direction)
{
    if (direction == DRIVING_FORWARD) {
        Motor(LEFT, FORWARD);
        enA = leftSpeed;
        Motor(RIGHT, REVERSE);
        enB = rightSpeed;
    }
    else {
        Motor(LEFT, REVERSE);
        enA = leftSpeed;
        Motor(RIGHT, FORWARD);
        enB = rightSpeed;
    }
}

void GoSwingLeft(drivingDirection direction)
{
    if (direction == DRIVING_FORWARD) {
        Motor(LEFT, FORWARD);
        enA = leftTurnRate;
        Motor(RIGHT, FORWARD);
        enB = rightSpeed;
    }
    else {
        Motor(LEFT, REVERSE);
        enA = leftTurnRate;
        Motor(RIGHT, REVERSE);
        enB = rightSpeed;
    }
}

void GoSwingRight(drivingDirection direction)
{
    if (direction == DRIVING_FORWARD) {
        Motor(LEFT, FORWARD);
        enA = leftSpeed;
        Motor(RIGHT, FORWARD);
        enB = rightTurnRate;
    }
    else {
        Motor(LEFT, REVERSE);
        enA = leftSpeed;
        Motor(RIGHT, REVERSE);
        enB = rightTurnRate;
    }
}

bool IsDriving()
{
    /*
    // using encoders
    leftMotor.reset();
    rightMotor.reset();
    wait_ms(1);
    if ((leftMotor.getPulses() == 0) && (rightMotor.getPulses() == 0))
        return false;
    return true;
    */
    return true;
}

void motorTest(void)
{
    Stop();
    leftSpeed=1;
    Motor(LEFT, FORWARD);
    enA = leftSpeed;
    wait(10);
    //rightSpeed = 1.0;
    //Stop();
    //Motor(RIGHT, FORWARD);
    //enB = rightSpeed;
    //wait(10);
    Stop();
}

void noEncoderTest(void) 
{   
    // surface = laminate

    leftSpeed = 0.5;
    rightSpeed = 0.5;
    leftTurnRate = 0.2;
    rightTurnRate = 0.2;

    wait(10);

    // forward and backward same "distance"
    Stop();
    GoStraight(DRIVING_FORWARD);
    wait(1);
    Stop();
    wait(1.5);
    GoStraight(DRIVING_BACKWARD);
    wait(1);
    Stop();
    wait(1.5);
    
    // point turn in both directions
    GoPointRight(DRIVING_FORWARD);
    wait(1);
    Stop();
    wait(1.5);
    GoPointLeft(DRIVING_FORWARD);
    wait(1);
    Stop();
    wait(1.5);

    // arch turn in both directions
    GoSwingRight(DRIVING_FORWARD);
    wait(3.3);
    Stop();
    wait(1.5);
    GoSwingLeft(DRIVING_FORWARD);
    wait(3.3);
    Stop();
    wait(1.5);

    // do a square of about 30 cm side using right turns then doing left turns
    rightTurnRate = 0.05;
    leftTurnRate = 0.05;
    GoStraight(DRIVING_FORWARD);
    wait(0.4);
    GoSwingRight(DRIVING_FORWARD);
    wait(0.5);
    GoStraight(DRIVING_FORWARD);
    wait(0.4);
    GoSwingRight(DRIVING_FORWARD);
    wait(0.5);
    GoStraight(DRIVING_FORWARD);
    wait(0.4);
    GoSwingRight(DRIVING_FORWARD);
    wait(0.5);
    GoStraight(DRIVING_FORWARD);
    wait(0.4);
    GoSwingLeft(DRIVING_FORWARD);
    wait(0.51);
    GoStraight(DRIVING_FORWARD);
    wait(0.4);
    GoSwingLeft(DRIVING_FORWARD);
    wait(0.51);
    GoStraight(DRIVING_FORWARD);
    wait(0.4);
    GoSwingLeft(DRIVING_FORWARD);
    wait(0.51);
    GoStraight(DRIVING_FORWARD);
    wait(0.4);
    GoSwingLeft(DRIVING_FORWARD);
    wait(0.51);
    GoStraight(DRIVING_FORWARD);
    wait(0.4);
    Stop();
            
    /*        
    // UCSD carpet, speed unknown
    Stop();
    GoStraight(DRIVING_FORWARD);
    wait(0.4);
    GoSwingRight(DRIVING_FORWARD);
    wait(0.6);
    GoStraight(DRIVING_FORWARD);
    wait(0.4);
    GoSwingRight(DRIVING_FORWARD);
    wait(0.6);
    GoStraight(DRIVING_FORWARD);
    wait(0.4);
    GoSwingRight(DRIVING_FORWARD);
    wait(0.6);
    GoStraight(DRIVING_FORWARD);
    wait(0.4);
    Stop();
    */
}

double GetHeading()
{
    // data in units
    mag.getAxis(mag_data);
    //printf("MAG: X=%4.1ff Y=%4.1ff Z=%4.1ff\n", acc_data.x, acc_data.y, acc_data.z, mag_data.x, mag_data.y, mag_data.z);

    // convert magnetometer XY data to degrees (0-359)
    double degrees = atan2((double)mag_data.y, (double)mag_data.x)*180.0/PI;
    if (degrees < 0)
        degrees += 360;
    
    printf("%f\n", degrees);
    return degrees;
}

headingDirection GetDirectionChange()
{
    double difference = heading - currentHeading;
    if (difference > 180)
        difference = difference - 360;
    else if (difference < -180)
        difference = 360 + difference;
    printf("new diff: %f => ", difference);
    if (difference > 0)
        return TURN_RIGHT;
    return TURN_LEFT;        
}

void CheckHeading()
{
    currentHeading = GetHeading();
    /*
    printf("heading: %f, currentHeading: %f, diff: %f ", heading, currentHeading, heading-currentHeading);

    if (currentHeading == heading) {
        GoStraight(direction);
        printf("straight\n\n");
    }
    else {
        // adjust motors
        if (GetDirectionChange() == TURN_RIGHT) {
            GoSwingRight(direction);
            printf("swing right\n\n");
        }
        else {
            GoSwingLeft(direction);
            printf("swing left\n\n");
        }
    }
    */
}

void driveStraightIMU()
{
    Timer timer;
    
    // motors off
    Stop();
    
    // initiate IMU
    acc.enable();
    mag.enable();
    
    // start driving
    leftSpeed = 0.2;
    rightSpeed = 0.2;
    leftTurnRate = .18;
    rightTurnRate = .18;
    //GoStraight(DRIVING_FORWARD);
    
    // wait a bit
    wait_ms(200);
    
    // get initial heading
    heading = GetHeading();

    timer.start();
    //while (timer.read() < 5) {
    while (true) {
        CheckHeading();
        wait(.5);
    }
    
    Stop();
}

void measureDistance()
{
    // measure the distance for an object using the ultrasound sensor

    // initiate ultrasonic sensor
    sensor.setRanges(2, 400);
    ticker.attach(&calc, sampleTime);
    pc.printf("Minimum sensor range = %g cm\n\rMaximum sensor range = %g cm\n\r", sensor.getMinRange(), sensor.getMaxRange());
    
    while (true) {

        while(!sensor.isNewDataReady()) {
            // wait for new data
        }
        float sensorDistance = sensor.getDistance_cm();
        printf("sensorDistance: %f\n", sensorDistance);
        
        // wait a bit
        wait(0.5f);
    }
}

void keepAway()
{
    // keep the distance to flat object place in front of the robot
    
    Timer timer;
    float sensorDistance;
    float distance = 10.0;
    float leftFullSpeed = 0.5;
    float rightFullSpeed = 0.5;
    
    // motors off
    Stop();
    
    wait(10);
    
    // initiate ultrasonic sensor
    sensor.setRanges(2, 400);
    ticker.attach(&calc, sampleTime);
     
    timer.start();
    while (timer.read() < 60) {

        while(!sensor.isNewDataReady()) {
            // wait for new data
        }
        sensorDistance = sensor.getDistance_cm();
        if (sensorDistance < distance) 
            Stop();
        else {
            if (sensorDistance < distance+30) { 
                leftSpeed = 0.10;  //minimum for robot to move
                rightSpeed = 0.10;
            }
            else if (sensorDistance < distance+60) {
                leftSpeed = leftFullSpeed * 0.25f;
                rightSpeed = rightFullSpeed * 0.25f;
            }
            else if (sensorDistance < distance+90) {
                leftSpeed = leftFullSpeed * 0.30f;
                rightSpeed = rightFullSpeed * 0.30f;
            }
            else if (sensorDistance < distance+120) {
                leftSpeed = leftFullSpeed * 0.40f;
                rightSpeed = rightFullSpeed * 0.40f;
            }
            else if (sensorDistance < distance+150) {
                leftSpeed = leftFullSpeed * 0.50f;
                rightSpeed = rightFullSpeed * 0.50f;
            }
            else {
                leftSpeed = leftFullSpeed;
                rightSpeed = rightFullSpeed;
            }
            GoStraight(DRIVING_FORWARD);
        }
        redLED = !redLED;
    }
    Stop();
}

int calcEncoderPulses(float distance)
{
    //distance in mm
    return (int)(distance/wheelCircumference*encoderPPR);
}

#define RATE 0.1
 
//Kc, Ti, Td, interval
PID leftPID(1.0, 0.0, 0.0, RATE);
PID rightPID(1.0, 0.0, 0.0, RATE); 

void encoderPIDTest(void) 
{   
    // surface = laminate
    Stop();
    leftSpeed = 0.5;
    rightSpeed = 0.5;
    leftTurnRate = 0.2;
    rightTurnRate = 0.2;
    rightMotor.reset();
    leftMotor.reset();
    
    GoStraight(DRIVING_FORWARD);
           
    //encoder input from 0 to 5000 pulses per second 
    leftPID.setInputLimits(0, 5000);
    rightPID.setInputLimits(0, 5000);
    //Pwm output from 0.0 to 0.5
    leftPID.setOutputLimits(0.0, 0.5);
    rightPID.setOutputLimits(0.0, 0.5);

    //leftPID.setMode(AUTO);
    //rightPID.setMode(AUTO);
    //We want the process variable to be 1.7V
    leftPID.setSetPoint(1.7);
 
  while(1){
    //Update the process variable.
    //leftPID.setProcessValue(pv.read());
    //Set the new output.
    //leftPID = controller.getRealOutput();
    //Wait for another loop calculation.
    wait(RATE);
  }    
}

void driveStraightEncoders()
{
    // left=master or right=master
    // no encoders - goes left
    // encoders - goes left more
    // left=master: right speed goes up
    // right=master: left speed goes down
    
    Stop();
    wait(5);
    //enA.period(0.01);
    //enB.period(0.01);
    leftSpeed = 0.0;
    rightSpeed = 0.0;
    float error = 0.0;
    //constant of proportionality
    double kp = 0.0005; 
    //double kp=0;
    // start timer
    Timer timer;
    timer.start();
    // start driving
    float leftEndSpeed = 0.15;
    float rightEndSpeed = 0.15;
/*    // one second acceleration ramp
    for(float time=0.1; time <= 1.0; time+=0.1)
    {
        leftSpeed=sin(PI/2*time)*leftEndSpeed;
        rightSpeed=sin(PI/2*time)*rightEndSpeed;
        GoStraight(DRIVING_FORWARD);
        wait(0.1);
    }*/
    // half-second acceleration ramp
    for(float time=0.1f; time <= 0.5f; time+=0.1f)
    {
        leftSpeed=sin(PI*time)*leftEndSpeed;
        rightSpeed=sin(PI*time)*rightEndSpeed;
        GoStraight(DRIVING_FORWARD);
        wait(0.1);
    }
    // reset encoders
    leftMotor.reset();
    rightMotor.reset();
    while (timer.read() < 5)
    {
        float left = abs(leftMotor.getPulses());
        float right = rightMotor.getPulses();
        
        // left=master
        //printf("left=master  ");
        //error = left-right;
        //rightSpeed += error*kp;
        
        // right=master
        printf("right=master  ");
        error = right-left;
        //leftSpeed += error*kp;
        if (error != 0.0f)
            kp = leftSpeed*(error/left);
        leftSpeed += kp;
        if (leftSpeed < 0.1f)
            leftSpeed = 0.1f;
        if (leftSpeed > 0.2f)
            leftSpeed = 0.2f;
        
        // no speed change
        //printf("no speed change  ");
        //error = left-right;
        
        printf("leftSpeed=%f   rightSpeed=%f   %f  %f  %3f %f\n",leftSpeed, rightSpeed, left, right, error, kp);
        GoStraight(DRIVING_FORWARD);
        leftMotor.reset();
        rightMotor.reset();
        wait_ms(100);
    }
    Stop();
}

int main(void)
{
    redLED = 0;
    //motorTest();
    //noEncoderTest();
    //driveStraightIMU(); //magnetometer vs motors
    driveStraightEncoders(); 
    //measureDistance();
    //keepAway(); not straight, deceleration ramp needs work
    //encoderPIDTest(); 
    redLED = 1;
}
