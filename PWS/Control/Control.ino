#include <Servo.h>
// I2Cdev and MPU6050 must be installed as libraries, or else the .cpp/.h files
// for both classes must be in the include path of your project
#include "I2Cdev.h"
#include "StopWatch.h"

#include "MPU6050_6Axis_MotionApps20.h"
//#include "MPU6050.h" // not necessary if using MotionApps include file

// Arduino Wire library is required if I2Cdev I2CDEV_ARDUINO_WIRE implementation
// is used in I2Cdev.h
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif

// class default I2C address is 0x68
// specific I2C addresses may be passed as a parameter here
// AD0 low = 0x68 (default for SparkFun breakout and InvenSense evaluation board)
// AD0 high = 0x*
MPU6050 mpu;
//MPU6050 mpu(0x*); // <-- use for AD0 high

/* =========================================================================
   NOTE: In addition to connection 3.3v, GND, SDA, and SCL, this sketch
   depends on the MPU-6050's INT pin being connected to the Arduino's
   external interrupt #0 pin. On the Arduino Uno and Mega 2560, this is
   digital I/O pin 2.
 * ========================================================================= */


// uncomment "OUTPUT_READABLE_QUATERNION" if you want to see the actual
// quaternion components in a [w, x, y, z] format (not best for parsing
// on a remote host such as Processing or something though)
//#define OUTPUT_READABLE_QUATERNION

// uncomment "OUTPUT_READABLE_EULER" if you want to see Euler angles
// (in degrees) calculated from the quaternions coming from the FIFO.
// Note that Euler angles suffer from gimbal lock (for more info, see
// http://en.wikipedia.org/wiki/Gimbal_lock)
//#define OUTPUT_READABLE_EULER

// uncomment "OUTPUT_READABLE_YAWPITCHROLL" if you want to see the yaw/
// pitch/roll angles (in degrees) calculated from the quaternions coming
// from the FIFO. Note this also requires gravity vector calculations.
// Also note that yaw/pitch/roll angles suffer from gimbal lock (for
// more info, see: http://en.wikipedia.org/wiki/Gimbal_lock)
#define OUTPUT_READABLE_YAWPITCHROLL

// uncomment "OUTPUT_READABLE_REALACCEL" if you want to see acceleration
// components with gravity removed. This acceleration reference frame is
// not compensated for orientation, so +X is always +X according to the
// sensor, just without the effects of gravity. If you want acceleration
// compensated for orientation, us OUTPUT_READABLE_WORLDACCEL instead.
//#define OUTPUT_READABLE_REALACCEL

// uncomment "OUTPUT_READABLE_WORLDACCEL" if you want to see acceleration
// components with gravity removed and adjusted for the world frame of
// reference (yaw is relative to initial orientation, since no magnetometer
// is present in this case). Could be quite handy in some cases.
//#define OUTPUT_READABLE_WORLDACCEL

// uncomment "OUTPUT_TEAPOT" if you want output that matches the
// format used for the InvenSense teapot demo
//#define OUTPUT_TEAPOT



#define LED_PIN 13 // (Arduino is 13, Teensy is 11, Teensy++ is 6)
//bool blinkState = false;

//#define RESET_PIN 12

// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

//speed vars
StopWatch SW;
float yprBegin[3];
float yprEnd[3];
int SWphase=0;
float yprSpeed[3];


//Final yaw/pitch/roll vars
float yprFinal[3] = {0.0, 0.0, 0.0};
float yprLast[3] = {0.0, 0.0, 0.0};
float yprDif[3] = {0.0, 0.0, 0.0};
float yprOffset[3] = {0.0, 0.0, 0.0};

byte difCount = 0;
bool initDone = false;
//bool yprSetOffset = false;

float offset[3] = {0.0, 0.0, 0.0};
float halfOffset[3] = {0.0, 0.0, 0.0};

// packet structure for InvenSense teapot demo
//uint8_t teapotPacket[14] = { '$', 0x02, 0,0, 0,0, 0,0, 0,0, 0x00, 0x00, '\r', '\n' };

//VARIABLES FOR ENGINES
Servo firstMotor, secondMotor, thirdMotor, fourthMotor;
int value = 0;
int motor1Power, motor2Power, motor3Power, motor4Power = 0;

int throttle = 15;

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

void initMotors() {
  //Motor names as inscribed on drone
  firstMotor.attach(9);
  secondMotor.attach(6);
  thirdMotor.attach(10);
  fourthMotor.attach(11);

  //Start by sending the lowest value to the motors
  firstMotor.writeMicroseconds(1200);
  secondMotor.writeMicroseconds(1200);
  thirdMotor.writeMicroseconds(1200);
  fourthMotor.writeMicroseconds(1200);
  Serial.println("Start ESC and wait 2 secs, then press a LETTER !!NOT NUMBERS!! and enter");

  //Be sure to send a !!LETTER!! and an enter
  //This unlocks the throttle and allows you to start the motors
  //while (Serial.available() && Serial.read()); // empty buffer
  //while (!Serial.available());                 // wait for data
  //while (Serial.available() && Serial.read()); // empty buffer again
  Serial.println("Motors configured");
}

void initGyro() {
    // join I2C bus (I2Cdev library doesn't do this automatically)
    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
        TWBR = 24; // 400kHz I2C clock (200kHz if CPU is 8MHz)
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
    #endif

    // initialize serial communication
    // (115200 chosen because it is required for Teapot Demo output, but it's
    // really up to you depending on your project)
    //while (!Serial); // wait for Leonardo enumeration, others continue immediately
    // NOTE: 8MHz or slower host processors, like the Teensy @ 3.3v or Ardunio
    // Pro Mini running at 3.3v, cannot handle this baud rate reliably due to
    // the baud timing being too misaligned with processor ticks. You must use
    // 38400 or slower in these cases, or use some kind of external separate
    // crystal solution for the UART timer.

    // initialize device
    Serial.println(F("Initializing I2C devices..."));
    mpu.initialize();

    // verify connection
    Serial.println(F("Testing device connections..."));
    Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));
    /*
    // wait for ready
    Serial.println(F("\nSend any character to begin DMP programming and demo: "));
    while (Serial.available() && Serial.read()); // empty buffer
    while (!Serial.available());                 // wait for data
    while (Serial.available() && Serial.read()); // empty buffer again
    */
    // load and configure the DMP
    Serial.println(F("Initializing DMP..."));
    devStatus = mpu.dmpInitialize();

    // supply your own gyro offsets here, scaled for min sensitivity
    mpu.setXGyroOffset(220);
    mpu.setYGyroOffset(76);
    mpu.setZGyroOffset(-85);
    mpu.setZAccelOffset(1788); // 1688 factory default for my test chip

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
    
    Serial.println("Gyro initialized");
}

bool calibrateGyro() {
  // if programming failed, don't try to do anything
    if (!dmpReady) {
      return false;
    }

    // wait for MPU interrupt or extra packet(s) available
    while (!mpuInterrupt && fifoCount < packetSize) {
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
    } else if (mpuIntStatus & 0x02) {
        // wait for correct available data length, should be a VERY short wait
        while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();

        // read a packet from FIFO
        mpu.getFIFOBytes(fifoBuffer, packetSize);
        
        // track FIFO count here in case there is > 1 packet available
        // (this lets us immediately read more without waiting for an interrupt)
        fifoCount -= packetSize;
        
        yprLast[0] = yprFinal[0];
        yprLast[1] = yprFinal[1];
        yprLast[2] = yprFinal[2];

        yprFinal[0] = ypr[0] * 180/M_PI;
        yprFinal[1] = ypr[1] * 180/M_PI;
        yprFinal[2] = ypr[2] * 180/M_PI;          
          
        yprDif[0] = yprFinal[0] - yprLast[0];
        yprDif[1] = yprFinal[1] - yprLast[1];
        yprDif[2] = yprFinal[2] - yprLast[2];
          
        if((round(yprDif[0]*100) >= -1 && round(yprDif[0]*100) <= 1) &&
          (round(yprDif[1]*100) >= -1 && round(yprDif[1]*100) <= 1) &&
          (round(yprDif[2]*100) >= -1 && round(yprDif[2]*100) <= 1) && !initDone) {
            difCount++;
        } else {
          difCount = 0;
        }
            
        if(difCount >= 100) {
          initDone = true;
        } 
        
        if (initDone) {
          yprOffset[0] = yprFinal[0];
          yprOffset[1] = yprFinal[1];
          yprOffset[2] = yprFinal[2];
        }
        
        #ifdef OUTPUT_READABLE_YAWPITCHROLL
            // display Euler angles in degrees
            mpu.dmpGetQuaternion(&q, fifoBuffer);
            mpu.dmpGetGravity(&gravity, &q);
            mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
            Serial.print("ypr\t");
            Serial.print(yprFinal[0]);
            Serial.print("\t");
            Serial.print(yprFinal[1]);
            Serial.print("\t");
            Serial.print(yprFinal[2]);
            Serial.print("\t");
            Serial.print(yprDif[2]);
            Serial.print("\t");
            Serial.print(difCount);
            Serial.print("\t");
            Serial.println(initDone);
        #endif

        return initDone;
    }
}

void setup() {
  Serial.begin(115200);
  initGyro(); //Initiate the gyroscope
  while(!initDone) {
    calibrateGyro(); //Calibrate the gyro as long as the gyro is not calibrated
  }
  digitalWrite(LED_PIN, HIGH); //Tell the user that the gyro is calibrated and that the drone is about to fly by lighting up a pin
  Serial.println("Gyro calibrated");
  initMotors(); //Calibrate the motors
  delay(1000); //Delay 1 second so that it is safe to fly
}

// ================================================================
// ===                    MAIN PROGRAM LOOP                     ===
// ================================================================

void getYPR() {
  // if programming failed, don't try to do anything
    if (!dmpReady) return;

    // wait for MPU interrupt or extra packet(s) available
    while (!mpuInterrupt && fifoCount < packetSize) {
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
    } else if (mpuIntStatus & 0x02) {
        // wait for correct available data length, should be a VERY short wait 
        while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();

        // read a packet from FIFO
        mpu.getFIFOBytes(fifoBuffer, packetSize);
        
        // track FIFO count here in case there is > 1 packet available
        // (this lets us immediately read more without waiting for an interrupt)
        fifoCount -= packetSize;
        
        //yprLast[0] = yprFinal[0];
        //yprLast[1] = yprFinal[1];
        //yprLast[2] = yprFinal[2];

        yprFinal[0] = ypr[0] * 180/M_PI - yprOffset[0];
        yprFinal[1] = ypr[1] * 180/M_PI - yprOffset[1];
        yprFinal[2] = ypr[2] * 180/M_PI - yprOffset[2];          

        //yprDif[0] = yprFinal[0] - yprLast[0];
        //yprDif[1] = yprFinal[1] - yprLast[1];
        //yprDif[2] = yprFinal[2] - yprLast[2];

        //if(abs(yprDif[0]) > 30 || abs(yprDif[1]) > 30 || abs(yprDif[2])) {
        //  mpu.resetFIFO();
        //}
        /*  
          if((round(yprDif[0]*100) >= -1 && round(yprDif[0]*100) <= 1) &&
            (round(yprDif[1]*100) >= -1 && round(yprDif[1]*100) <= 1) &&
            (round(yprDif[2]*100) >= -1 && round(yprDif[2]*100) <= 1) && !initDone) {
              difCount++;
            } else {
              difCount = 0;
            }
            
          if(difCount >= 100) {
            initDone = true;
          } 
        } else if (!yprSetOffset) {
          yprOffset[0] = yprFinal[0];
          yprOffset[1] = yprFinal[1];
          yprOffset[2] = yprFinal[2];
          yprSetOffset = true;
        }
        */
        #ifdef OUTPUT_READABLE_YAWPITCHROLL
            // display Euler angles in degrees
            mpu.dmpGetQuaternion(&q, fifoBuffer);
            mpu.dmpGetGravity(&gravity, &q);
            mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
            Serial.print("ypr\t");
            Serial.print(yprFinal[0]);
            Serial.print("\t");
            Serial.print(yprFinal[1]);
            Serial.print("\t");
            Serial.println(yprFinal[2]);
        #endif
    }
}

void yprSpeed(){
  
if(SWphase==0){
    
  yprBegin[0] = yprFinal[0];
  yprBegin[1] = yprFinal[1];
  yprBegin[2] = yprFinal[2];
  
  SW.start();
  }
  
  else if(SWphase==10){
  yprEnd[0] = yprFinal[0];
  yprEnd[1] = yprFinal[1]; 
  yprEnd[2] = yprFinal[2]; 

  SW.stop();
  yprSpeed[0] = (yprEnd[0]-yprBegin[0])/(SW.elapsed/1000);
  yprSpeed[1] = (yprEnd[1]-yprBegin[0])/(SW.elapsed/1000);
  yprSpeed[2] = (yprEnd[2]-yprBegin[0])/(SW.elapsed/1000);
  SW.reset();
  SWphase=0;
  }
  else{
  SWphase++;
  }

}

void controlEngines() {
  throttle = 10;
  /*
  for(byte i = 0; i < 3; i++) {
    if(abs(yprFinal[i]) > 3 && halfOffset[i] == 0.00) {
      halfOffset[i] = yprFinal[i] * (float) 0.50;
      offset[i] = 5;//halfOffset[i] * (float) 3.00;
    }
    //if(yprFinal[i] - halfOffset[i] >= -0.10 && yprFinal[i] - halfOffset[i] <= 0.10) {
    //  offset[i] = -offset[i];
    //}
    if(yprFinal[i] >= -0.10 && yprFinal[i] <= 0.10) {
      offset[i] = 0.00;
      halfOffset[i] = 0.00;
    }
    Serial.print("\t");
    Serial.print(offset[i]);
  }
  Serial.println();
  */
  /*
  if(abs(yprFinal[1]) > 3 && abs(halfOffset[1]) == 0.0) {
    halfOffset[1] = yprFinal[1]/2;
    offset[1] = (halfOffset[1]/3) * 2;
  }
  if(yprFinal[1] - halfOffset[1] >= -0.10 && yprFinal[1] - halfOffset[1] <= 0.10) {
    offset[1] = -offset[1];
  }
  if(yprFinal[1] >= -0.10 && yprFinal[1] <= 0.10) {
    offset[1] = 0;
    halfOffset[1] = 0;
  }

  if(abs(yprFinal[2]) > 3 && abs(halfOffset[2]) == 0.0) {
    halfOffset[2] = yprFinal[2]/2;
    offset[2] = (halfOffset[2]/3) * 2;
  }
  if(yprFinal[2] - halfOffset[2] >= -0.10 && yprFinal[2] - halfOffset[2] <= 0.10) {
    offset[2] = -offset[2];
  }
  if(yprFinal[2] >= -0.10 && yprFinal[2] <= 0.10) {
    offset[2] = 0;
    halfOffset[2] = 0;
  }
  */
  
  offset[0] = (yprFinal[0] / (float) 3) * (float) 2;
  offset[1] = (yprFinal[1] / (float) 3) * (float) 2;
  offset[2] = (yprFinal[2] / (float) 3) * (float) 2;

  //Power     = throttle +/- offsetPitch +/- offsetRoll +/- offsetYaw
  motor1Power = throttle - offset[1] - offset[2] - offset[0];
  motor2Power = throttle - offset[1] + offset[2] + offset[0];
  motor3Power = throttle + offset[1] - offset[2] + offset[0];
  motor4Power = throttle + offset[1] + offset[2] - offset[0];
  
  /*
  if(Serial.available()) {
    value = Serial.parseInt();
  }
  
  //If value == -1, shut down engines
  if(value == -1)
    motor1Power = motor2Power = motor3Power = motor4Power = 1200;
  else { 
  */
  //Otherwise map the percentage PWM values for engines
  motor1Power = map(motor1Power, 0, 100, 1225, 2100);
  motor2Power = map(motor2Power, 0, 100, 1229, 2100);
  motor3Power = map(motor3Power, 0, 100, 1225, 2100);
  motor4Power = map(motor4Power, 0, 100, 1239, 2100);
  //}
  
  //Send values to engines
  firstMotor.writeMicroseconds(motor1Power);
  secondMotor.writeMicroseconds(motor2Power);
  thirdMotor.writeMicroseconds(motor3Power);
  fourthMotor.writeMicroseconds(motor4Power);
}

void loop() {

  getYPR(); //Get data from the gyroscope
  
  yprSpeed(); //Calculate the speed of change in orientation
  
  controlEngines(); //Control the engines accordingly to this data
}

//RESET THE ARDUINO
void(* resetFunc) (void) = 0;

//RESET ARDUINO BY PRESSING R
void serialEvent() {
  while(Serial.available()) {
    if((char) Serial.read() == 'r') {
      //digitalWrite(RESET_PIN, LOW);
    }
  }
}
