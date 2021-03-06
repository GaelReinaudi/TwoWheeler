
#include <Adafruit_MotorShield.h>
#include "utility/Adafruit_PWMServoDriver.h"
#include <PID_v1.h>

#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include<Wire.h>

// Create the motor shield object with the default I2C address
Adafruit_MotorShield AFMS = Adafruit_MotorShield(); 
// Select which 'port' M1, M2, M3 or M4. In this case, M1
Adafruit_DCMotor *M1 = AFMS.getMotor(1);
// You can also make another motor on port M2
Adafruit_DCMotor *M2 = AFMS.getMotor(2);

MPU6050 mpu;
#define LED_PIN 13 // (Arduino is 13, Teensy is 11, Teensy++ is 6)
bool blinkState = false;

// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorFloat gravity;    // [x, y, z]            gravity vector
bool overAngle = false; // true if the angle is very much to big and we should stop the motors

//Define Variables we'll be connecting to
double Input = 0.0, Output = 0.0;
double InitialSetpointGz = 15.0;
double Setpoint = InitialSetpointGz;
//Specify the links and initial tuning parameters
PID myPID(&Input, &Output, &Setpoint,3,50,0.35, DIRECT);

double voltage = 8.0; // V
double scaleOutput = 12.0 / voltage;

// ================================================================
// ===               INTERRUPT DETECTION ROUTINE                ===
// ================================================================
volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
    mpuInterrupt = true;
}

// ================================================================
// ===                      INITIAL SETUP                       ===
// ================================================================
void setup(){
    Serial.begin(115200);
    while (!Serial); // wait for Leonardo enumeration, others continue immediately

    Serial.println("Adafruit Motorshield v2 - DC Motor test!");
    AFMS.begin();  // create with the default frequency 1.6KHz
    //AFMS.begin(1000);  // OR with a different frequency, say 1KHz

    Wire.begin();
    TWBR = 24; // 400kHz I2C clock (200kHz if CPU is 8MHz)
  
    Serial.println(F("Initializing I2C devices..."));

    mpu.initialize();

    // verify connection
    Serial.println(F("Testing device connections..."));
    Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

    // load and configure the DMP
    Serial.println(F("Initializing DMP..."));
    devStatus = mpu.dmpInitialize();

    // supply your own gyro offsets here, scaled for min sensitivity
    //mpu.setXGyroOffset(220);
    //mpu.setYGyroOffset(76);
    //mpu.setZGyroOffset(-85);
    //mpu.setZAccelOffset(1788); // 1688 factory default for my test chip

    mpu.setSleepEnabled(false);

    // make sure it worked (returns 0 if so)
    if (devStatus == 0) {
        // turn on the DMP, now that it's ready
        Serial.println(F("Enabling DMP..."));
        mpu.setDMPEnabled(true);

        // enable Arduino interrupt detection
        Serial.println(F("Enabling interrupt detection (Arduino external interrupt 0)..."));
        attachInterrupt(0, dmpDataReady, RISING);
        mpuIntStatus = mpu.getIntStatus();

        // set our DMP Ready flag so the main loop() function knows it's okay to use it
        Serial.println(F("DMP ready! Waiting for first interrupt..."));
        dmpReady = true;

        // get expected DMP packet size for later comparison
        packetSize = mpu.dmpGetFIFOPacketSize();
    } else {
        // ERROR!
        // 1 = initial memory load failed
        // 2 = DMP configuration updates failed
        // (if it's going to break, usually the code will be 1)
        Serial.print(F("DMP Initialization failed (code "));
        Serial.print(devStatus);
        Serial.println(F(")"));
    }

    // configure LED for output
    pinMode(LED_PIN, OUTPUT);
  
  //initialize the variables we're linked to
  myPID.SetOutputLimits(-150, 150);								//the arduino pwm limits
  //turn the PID on
  myPID.SetMode(AUTOMATIC);
}

// ================================================================
// ===                    MAIN PROGRAM LOOP                     ===
// ================================================================
void loop()
{
    // if programming failed, don't try to do anything
    if (!dmpReady)
        return;
    bool donePDI = false;
    // wait for MPU interrupt or extra packet(s) available
    while (!mpuInterrupt && fifoCount < packetSize) {
        // other program behavior stuff here
        // .
        // .
        // .
        // if you are really paranoid you can frequently test in between other
        // stuff to see if mpuInterrupt is true, and if so, "break;" from the
        // while() loop to immediately process the MPU data
        // .
        if(!donePDI) {
            iteratePID();
            donePDI = true;
        }
    }

    // reset interrupt flag and get INT_STATUS byte
    mpuInterrupt = false;
    mpuIntStatus = mpu.getIntStatus();

    // get current FIFO count
    fifoCount = mpu.getFIFOCount();

    // check for overflow (this should never happen unless our code is too inefficient)
    if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
        // reset so we can continue cleanly
        mpu.resetFIFO();
        Serial.println(F("FIFO overflow!"));

    // otherwise, check for DMP data ready interrupt (this should happen frequently)
    } 
    else if (mpuIntStatus & 0x02) {
        // wait for correct available data length, should be a VERY short wait
        while (fifoCount < packetSize) {
          fifoCount = mpu.getFIFOCount();
        }

        while (fifoCount >= 2 * packetSize) {
            // read a packet from FIFO
            mpu.getFIFOBytes(fifoBuffer, packetSize);
            fifoCount = mpu.getFIFOCount();
        }
        // read a packet from FIFO
        mpu.getFIFOBytes(fifoBuffer, packetSize);
        
        // track FIFO count here in case there is > 1 packet available
        // (this lets us immediately read more without waiting for an interrupt)
        fifoCount -= packetSize;

        mpu.dmpGetQuaternion(&q, fifoBuffer);
        mpu.dmpGetGravity(&gravity, &q);

        // too much angle ?
        if (abs(InitialSetpointGz - gravity.z*100.0) > 60) {
            overAngle = true;
        }
        // hysteresis to reanable the balacing when we are very close to vertical
        if (overAngle && abs(InitialSetpointGz - gravity.z*100.0) < 5) {
            overAngle = false;
        }
        
        static unsigned char c = 0;
        c += 32;
        if(c == 0) {
            // blink LED to indicate activity
            blinkState = !blinkState;
            digitalWrite(LED_PIN, blinkState);
        }
    }
}

void iteratePID()
{
    Serial.print("\tgravity\t");
    Serial.print(gravity.x);
    Serial.print("\t");
    Serial.print(gravity.y);
    Serial.print("\t");
    Serial.print(gravity.z);
    
    if (overAngle) {
        // makes the setpoint to come back to InitialSetpointGz
        myPID.SetMode(MANUAL);
        Input = 0;
        Setpoint = 0;
        Output = 0.0;
    }
    else {
        myPID.SetMode(AUTOMATIC);
        Input = InitialSetpointGz - gravity.z*100.0;
        // adjust setpoint to aim at motors off
        if (abs(Output) > 20)
            Setpoint += Output * 0.00001;
    }
    myPID.Compute();
    int v = Output * scaleOutput;
    if (overAngle) {
        setMotorSpeed(0, 0);
    }
    else {
        setMotorSpeed(v, v);
    }
    
    Serial.print("\t\t Setpoint ");
    Serial.print(Setpoint);
    Serial.print("\t In ");
    Serial.print(Input);
    Serial.print("\t Out ");
    Serial.print(Output);
    Serial.println("");
}

void setMotorSpeed(int16_t v1, int16_t v2)
{
    v1 = max(-255, min(255, v1));
    v2 = max(-255, min(255, v2));
    if(v1 > 0) {
        M1->run(FORWARD);
    }
    else if(v1 < 0) {
        M1->run(BACKWARD);
        v1 = -v1;
    }
    else {
        M1->run(RELEASE);
    }    
    if(v2 > 0) {
        M2->run(FORWARD);
    }
    else if(v2 < 0) {
        M2->run(BACKWARD);
        v2 = -v2;
    }
    else {
        M2->run(RELEASE);
    }
    M1->setSpeed(v1);  
    M2->setSpeed(v2);  
}
