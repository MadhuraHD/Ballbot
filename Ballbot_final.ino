#include "math.h"
#include "PID_v1.h"
#include "FrequencyTimer2.h"
#include "TimerOne.h"
#include "TimerThree.h"

#define ROBOT_HEIGHT 16.73 //inches
#define WHEEL_RADIUS 1.9685
#define STEPS_PER_ROTATION 1600 //steps
#define MAX_SPEED 30 //inches per second
#define MAX_FREQ 50 //component steps per second
#define ACCEL 10 //steps per second

int sampleRate =  10000; //microseconds
bool printFlag = false;

#define DIR1 2
#define STEP1 3
#define DIR2 4
#define STEP2 5
#define DIR3 6
#define STEP3 7

//Three PID controllers; one for each axis of rotation
//Define Variables we'll be connecting to
double CurrentAngleX, OutputSpeedX, DesiredAngleX = 0;
double CurrentAngleY, OutputSpeedY, DesiredAngleY = 0;
double CurrentAngleZ, OutputSpeedZ, DesiredAngleZ = 0;
double Kp_x = 11, Kp_y = 11, Kp_z = 11;
double Ki_x = 0.35, Ki_y = 0.35, Ki_z = 0.35;
double Kd_x = 4.5, Kd_y = 4.5, Kd_z = 4.5;

//Specify the links and initial tuning parameters
PID xPID(&CurrentAngleX, &OutputSpeedX, &DesiredAngleX, Kp_x, Ki_x, Kd_x, DIRECT);
PID yPID(&CurrentAngleY, &OutputSpeedY, &DesiredAngleY, Kp_y, Ki_y, Kd_y, DIRECT);
PID zPID(&CurrentAngleZ, &OutputSpeedZ, &DesiredAngleZ, Kp_z, Ki_z, Kd_z, DIRECT);

#define AHRS true         // Set to false for basic data read
#define SerialDebug true  // Set to true to get Serial output for debugging

// -----------------------------------------------------------------------------
// Need this to compile for some reason... No idea why. Think it has something
// to do with the way the Teensy loader works
extern "C" {
  int _getpid() {
    return -1;
  }
  int _kill(int pid, int sig) {
    return -1;
  }
  int _write() {
    return -1;
  }
}
// -----------------------------------------------------------------------------

// Pin definitions
int Pin = 12;  // These can be changed, 2 and 3 are the Arduinos ext int pins
int RPM = 0;

double speed1, speed2, speed3;
double timer = 0, timerstart, timerend;
double yaw, pitch, roll;
int currentSpeed1, currentSpeed2, currentSpeed3;

#include "I2Cdev.h"

#include "MPU6050_6Axis_MotionApps_V6_12.h"
//#include "MPU6050.h" // not necessary if using MotionApps include file

#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
#include "Wire.h"
#endif

// class default I2C address is 0x68
// specific I2C addresses may be passed as a parameter here
// AD0 low = 0x68 (default for SparkFun breakout and InvenSense evaluation board)
// AD0 high = 0x69
MPU6050 mpu;
//MPU6050 mpu(0x69); // <-- use for AD0 high

#define OUTPUT_READABLE_YAWPITCHROLL
#define OUTPUT_READABLE_REALACCEL
#define OUTPUT_READABLE_WORLDACCEL

#define INTERRUPT_PIN 2  // use pin 2 on Arduino Uno & most boards
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
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 gy;         // [x, y, z]            gyro sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

// packet structure for InvenSense teapot demo
uint8_t teapotPacket[14] = { '$', 0x02, 0, 0, 0, 0, 0, 0, 0, 0, 0x00, 0x00, '\r', '\n' };



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

void setupIMU() {
  // join I2C bus (I2Cdev library doesn't do this automatically)
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
  Wire.begin();
  Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties
#elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
  Fastwire::setup(400, true);
#endif

  // initialize serial communication
  Serial.begin(115200);
  while (!Serial); // wait for Leonardo enumeration, others continue immediately

  // initialize device
//  Serial.println(F("Initializing I2C devices..."));
  mpu.initialize();
  pinMode(INTERRUPT_PIN, INPUT);

  // verify connection
  //Serial.println(F("Testing device connections..."));
  Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

  // load and configure the DMP
  //Serial.println(F("Initializing DMP..."));
  devStatus = mpu.dmpInitialize();

  // supply your own gyro offsets here, scaled for min sensitivity
  mpu.setXGyroOffset(51);
  mpu.setYGyroOffset(76);
  mpu.setZGyroOffset(-85);
  mpu.setXAccelOffset(1150);  //----
  mpu.setYAccelOffset(-50);   //----
  mpu.setZAccelOffset(1788);  //1788
  // make sure it worked (returns 0 if so)
  if (devStatus == 0) {
    // Calibration Time: generate offsets and calibrate our MPU6050
    mpu.CalibrateAccel(6);
    mpu.CalibrateGyro(6);
//    Serial.println();
    mpu.PrintActiveOffsets();
    // turn on the DMP, now that it's ready
    //Serial.println(F("Enabling DMP..."));
    mpu.setDMPEnabled(true);

    // enable Arduino interrupt detection
    //Serial.print(F("Enabling interrupt detection (Arduino external interrupt "));
    //Serial.print(digitalPinToInterrupt(INTERRUPT_PIN));
    //Serial.println(F(")..."));
    attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
    mpuIntStatus = mpu.getIntStatus();

    // set our DMP Ready flag so the main loop() function knows it's okay to use it
    //Serial.println(F("DMP ready! Waiting for first interrupt..."));
    dmpReady = true;

    // get expected DMP packet size for later comparison
    packetSize = mpu.dmpGetFIFOPacketSize();
  } else {
     ERROR!
    //Serial.print(F("DMP Initialization failed ");
//    Serial.println(F(")"));
  }

  // configure LED for output
  pinMode(LED_PIN, OUTPUT);
}


// ================================================================
// ===                    MAIN PROGRAM LOOP                     ===
// ================================================================

void readIMU() {
  // if programming failed, don't try to do anything
  if (!dmpReady) return;
  // read a packet from FIFO
  if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) { // Get the Latest packet

#ifdef OUTPUT_READABLE_QUATERNION
    // display quaternion values in easy matrix form: w x y z
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    //    Serial.print("quat\t");
    //    Serial.print(q.w);
    //    Serial.print("\t");
    //    Serial.print(q.x);
    //    Serial.print("\t");
    //    Serial.print(q.y);
    //    Serial.print("\t");
    //    Serial.println(q.z);
#endif

#ifdef OUTPUT_READABLE_EULER
    // display Euler angles in degrees
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetEuler(euler, &q);
    //    Serial.print("euler\t");
    //    Serial.print(euler[0] * 180 / M_PI);
    //    Serial.print("\t");
    //    Serial.print(euler[1] * 180 / M_PI);
    //    Serial.print("\t");
    //    Serial.println(euler[2] * 180 / M_PI);
#endif

#ifdef OUTPUT_READABLE_YAWPITCHROLL
    // display Euler angles in degrees
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
    //    Serial.print("ypr\t");
    //    Serial.print(ypr[0] * 180 / M_PI);
    //    Serial.print("\t");
    //    Serial.print(ypr[1] * 180 / M_PI);
    //    Serial.print("\t");
    //    Serial.print(ypr[2] * 180 / M_PI);
    Serial.println();

#endif

#ifdef OUTPUT_READABLE_YAWPITCHROLL
    // display Euler angles in degrees
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);

    yaw = ypr[0] * 180 / M_PI;
    pitch = ypr[1] * 180 / M_PI;
    roll = ypr[2] * 180 / M_PI;
    
                Serial.print("Yaw: ");
                Serial.print(yaw);
                Serial.print(" Pitch: ");
                Serial.print(pitch);
                Serial.print(" Roll: ");
                Serial.print(roll);
                
#endif


#ifdef OUTPUT_READABLE_REALACCEL
    // display real acceleration, adjusted to remove gravity
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetAccel(&aa, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
    //    Serial.print("areal\t");
    //    Serial.print(aaReal.x);
    //    Serial.print("\t");
    //    Serial.print(aaReal.y);
    //    Serial.print("\t");
    //    Serial.println(aaReal.z);
#endif

#ifdef OUTPUT_READABLE_WORLDACCEL
    // display initial world-frame acceleration, adjusted to remove gravity
    // and rotated based on known orientation from quaternion
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetAccel(&aa, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
    mpu.dmpGetLinearAccelInWorld(&aaWorld, &aaReal, &q);
    //    Serial.print("aworld\t");
    //    Serial.print(aaWorld.x);
    //    Serial.print("\t");
    //    Serial.print(aaWorld.y);
    //    Serial.print("\t");
    //    Serial.println(aaWorld.z);
#endif

#ifdef OUTPUT_TEAPOT
    // display quaternion values in InvenSense Teapot demo format:
    teapotPacket[2] = fifoBuffer[0];
    teapotPacket[3] = fifoBuffer[1];
    teapotPacket[4] = fifoBuffer[4];
    teapotPacket[5] = fifoBuffer[5];
    teapotPacket[6] = fifoBuffer[8];
    teapotPacket[7] = fifoBuffer[9];
    teapotPacket[8] = fifoBuffer[12];
    teapotPacket[9] = fifoBuffer[13];
    Serial.write(teapotPacket, 14);
    teapotPacket[11]++; // packetCount, loops at 0xFF on purpose
#endif

    // blink LED to indicate activity
    blinkState = !blinkState;
    digitalWrite(LED_PIN, blinkState);
  }
}

void setupStepper() { // Set a PWM signal to the step pins with 50% duty cycle
  pinMode(DIR1, OUTPUT);          // sets the digital pin 13 as output
  pinMode(DIR2, OUTPUT);          // sets the digital pin 13 as output
  pinMode(DIR3, OUTPUT);          // sets the digital pin 13 as output

  analogWrite(STEP1, 10);
  analogWrite(STEP2, 10);
  analogWrite(STEP3, 10);
}

void updateMotors(long stepHz1, long stepHz2, long stepHz3) {
  if (stepHz1 < 0) {
    digitalWrite(DIR1, LOW);
  }
  else {
    digitalWrite(DIR1, HIGH);
  }
  analogWrite(STEP1, abs(stepHz1)); // pins 3 also change


  if (stepHz2 < 0) {
    digitalWrite(DIR2, LOW);
  }
  else {
    digitalWrite(DIR2, HIGH);
  }
  analogWrite(STEP2, abs(stepHz2)); // pins 3 also change


  if (stepHz3 < 0) {
    digitalWrite(DIR3, LOW);
  }
  else {
    digitalWrite(DIR3, HIGH);
  }
  analogWrite(STEP3, abs(stepHz3)); // pins 3 also change
}

void speedCalculations() {
  // Convert PID output to inches per second
  int speedX = map(OutputSpeedX, 0, 255, 0, MAX_SPEED);
  int speedY = map(OutputSpeedY, 0, 255, 0, MAX_SPEED);
  int speedZ = map(OutputSpeedZ, 0, 255, 0, MAX_SPEED);
  //  Serial.print("Speed-x: "); Serial.println(speedX);
  //  Serial.print("Speed-y: "); Serial.println(speedY);
  //  Serial.print("Speed-z: "); Serial.println(speedZ);
  if (pitch > 0) {
    speedX = -speedX;
  }
  if (roll > 0) {
    speedY = -speedY;
  }
//  if (yaw > 0) {
//    speedZ = -speedZ;
//  }

{
  //Serial.println(targetvel2);
  speed3= -(0.333)*(speedZ + (1.414*((speedX*cos(yaw))-(speedY*sin(yaw)))));
  speed2= -(0.333)*(speedZ+ ((1/1.414)*((sin(yaw)*((-1.732*speedX)+speedY))-(cos(yaw)*(speedX+(1.732*speedY))))));
  speed1= -(0.333)*(speedZ+ ((1/1.414)*((sin(yaw)*((1.732*speedX)+speedY))+(cos(yaw)*(-speedX+(1.732*speedY))))));
  }
}

void setupPID() {
  xPID.SetMode(AUTOMATIC);
  yPID.SetMode(AUTOMATIC);
  zPID.SetMode(AUTOMATIC);
}

void setup() {
  setupIMU();
  setupPID();
  setupStepper();
}

void loop() {

  readIMU();
  CurrentAngleX = (double) - abs(pitch);
  CurrentAngleY = (double) - abs(roll);
  // CurrentAngleZ = (double) - abs(yaw);

  timer = micros() - timerstart;
  if (timer >= sampleRate) {
    xPID.Compute();
    yPID.Compute();
    zPID.Compute();
    speedCalculations();
    updateMotors(speed1, speed2, speed3);
    timerstart = micros();

    Serial.print("X-angle: "); Serial.print(CurrentAngleX);
    Serial.print(" X-speed: "); Serial.print(OutputSpeedX);
    Serial.print(" Y-angle: "); Serial.print(CurrentAngleY);
    Serial.print(" Y-speed: "); Serial.print(OutputSpeedY);
    Serial.print(" Z-angle: "); Serial.print(CurrentAngleZ);
    Serial.print(" Z-speed: "); Serial.print(OutputSpeedZ);

    Serial.print(" Speed 1: "); Serial.print(speed1);
    Serial.print(" Speed 2: "); Serial.print(speed2);
    Serial.print(" Speed 3: "); Serial.print(speed3);
    Serial.print("\n");
  // Serial.print("Timer: "); Serial.print(timer);
  }
}
