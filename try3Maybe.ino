#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include "LedControl.h"
MPU6050 mpu;
LedControl lc = LedControl(12, 11, 10, 1); // Pins: DIN,CLK,CS, # of Display connected

// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer
float euler[3];         // [psi, theta, phi]    Euler angle container
Quaternion q;
int oppositeSignReadingsInARow = 0;
int frameChangeCount = 50;
int frameChangeCountThreshold = 50;
int frameCount = 0;
int negFrameCount = 25;
int threshold = 50;
bool isNegative = false;
volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
  mpuInterrupt = true;
}
byte frame1[] = {B11111111, B11111111, B00000000, B00000000, B00000000, B00000000, B00000000, B00000000}; //
byte frame2[] = {B11111111, B01111111, B00000001, B00000000, B00000000, B00000000, B00000000, B00000000}; //
byte frame3[] = {B11111111, B00111111, B00000011, B00000000, B00000000, B00000000, B00000000, B00000000}; //
byte frame4[] = {B11111111, B00011111, B00000111, B00000000, B00000000, B00000000, B00000000, B00000000}; //
byte frame5[] = {B01111111, B00011111, B00001111, B00000000, B00000000, B00000000, B00000000, B00000000};
byte frame6[] = {B0011111, B00011111, B00001111, B00000001, B00000000, B00000000, B00000000, B00000000};
byte frame7[] = {B00011111, B00011111, B00001111, B00000011, B0000000, B00000000, B00000000, B00000000};
byte frame8[] = {B00011111, B00011111, B00000111, B00000011, B0000001, B00000000, B00000000, B00000000};
byte frame9[] = {B00011111, B00001111, B00000111, B00000011, B0000001, B00000001, B00000000, B00000000};
byte frame10[] = {B00001111, B00001111, B00000111, B00000011, B0000001, B00000001, B00000001, B00000000};
byte frame11[] = {B00001111, B00000111, B00000111, B00000011, B0000001, B00000001, B00000001, B00000001};
byte frame12[] = {B00000111, B00000111, B00000111, B0000011, B0000001, B00000001, B00000001, B00000011};
byte frame13[] = {B00000111, B00000111, B00000111, B00000001, B0000001, B00000001, B00000011, B00000011};
byte frame14[] = {B00000111, B00000111, B00000011, B00000001, B0000001, B00000001, B00000011, B00000111};
byte frame15[] = {B00000111, B00000011, B00000001, B00000001, B0000001, B00000011, B00000111, B00000111};
byte frame16[] = {B00000011, B00000011, B00000001, B00000001, B0000001, B00000111, B00000111, B00000111};
byte frame17[] = {B00000011, B00000001, B00000001, B00000001, B0000011, B00000111, B00000111, B00000111};
byte frame18[] = {B00000001, B00000001, B00000001, B00000001, B0000011, B00000111, B00001111, B00001111};
byte frame19[] = {B0000000, B00000001, B00000001, B00000001, B0000011, B00000111, B00001111, B00001111};
byte frame20[] = {B0000000, B00000000, B00000001, B00000001, B0000011, B00000111, B00001111, B00011111};
byte frame21[] = {B0000000, B00000000, B00000000, B00000001, B0000011, B00000111, B00011111, B00011111};
byte frame22[] = {B0000000, B00000000, B00000000, B00000000, B0000011, B00001111, B00011111, B00011111};
byte frame23[] = {B0000000, B00000000, B00000000, B00000000, B0000001, B00001111, B00011111, B00111111};
byte frame24[] = {B0000000, B00000000, B00000000, B00000000, B0000000, B00001111, B00011111, B01111111};
byte frame25[] = {B0000000, B00000000, B00000000, B00000000, B0000000, B00000111, B00011111, B11111111}; //
byte frame26[] = {B0000000, B00000000, B00000000, B00000000, B0000000, B00000011, B00111111, B11111111}; //
byte frame27[] = {B0000000, B00000000, B00000000, B00000000, B0000000, B00000001, B01111111, B11111111}; //
byte frame28[] = {B0000000, B00000000, B00000000, B00000000, B0000000, B00000000, B11111111, B11111111}; //
class setFrame {
    //  Take values in Arrays and Display them
  public:
    void framea()
    {
      for (int i = 0; i < 8; i++)
      {
        lc.setRow(0, i, frame1[i]);
      }
    }

    void frameb()
    {
      for (int i = 0; i < 8; i++)
      {
        lc.setRow(0, i, frame2[i]);
      }
    }

    void framec()
    {
      for (int i = 0; i < 8; i++)
      {
        lc.setRow(0, i, frame3[i]);
      }
    }
    void framed()
    {
      for (int i = 0; i < 8; i++)
      {
        lc.setRow(0, i, frame4[i]);
      }
    }
    void framee()
    {
      for (int i = 0; i < 8; i++)
      {
        lc.setRow(0, i, frame5[i]);
      }
    }
    void framef()
    {
      for (int i = 0; i < 8; i++)
      {
        lc.setRow(0, i, frame6[i]);
      }
    }
    void frameg()
    {
      for (int i = 0; i < 8; i++)
      {
        lc.setRow(0, i, frame7[i]);
      }
    }
    void frameh()
    {
      for (int i = 0; i < 8; i++)
      {
        lc.setRow(0, i, frame8[i]);
      }
    }
    void framei()
    {
      for (int i = 0; i < 8; i++)
      {
        lc.setRow(0, i, frame9[i]);
      }
    }
    void framej()
    {
      for (int i = 0; i < 8; i++)
      {
        lc.setRow(0, i, frame10[i]);
      }
    }
    void framek()
    {
      for (int i = 0; i < 8; i++)
      {
        lc.setRow(0, i, frame11[i]);
      }
    }
    void framel()
    {
      for (int i = 0; i < 8; i++)
      {
        lc.setRow(0, i, frame12[i]);
      }
    }
    void framem()
    {
      for (int i = 0; i < 8; i++)
      {
        lc.setRow(0, i, frame13[i]);
      }
    }
    void framen()
    {
      for (int i = 0; i < 8; i++)
      {
        lc.setRow(0, i, frame14[i]);
      }
    }
    void frameo()
    {
      for (int i = 0; i < 8; i++)
      {
        lc.setRow(0, i, frame15[i]);
      }
    }
    void framep()
    {
      for (int i = 0; i < 8; i++)
      {
        lc.setRow(0, i, frame16[i]);
      }
    }
    void frameq()
    {
      for (int i = 0; i < 8; i++)
      {
        lc.setRow(0, i, frame17[i]);
      }
    }
    void framer()
    {
      for (int i = 0; i < 8; i++)
      {
        lc.setRow(0, i, frame18[i]);
      }
    }
    void frames()
    {
      for (int i = 0; i < 8; i++)
      {
        lc.setRow(0, i, frame19[i]);
      }
    }
    void framet()
    {
      for (int i = 0; i < 8; i++)
      {
        lc.setRow(0, i, frame20[i]);
      }
    }
    void frameu()
    {
      for (int i = 0; i < 8; i++)
      {
        lc.setRow(0, i, frame21[i]);
      }
    }
    void framev()
    {
      for (int i = 0; i < 8; i++)
      {
        lc.setRow(0, i, frame22[i]);
      }
    }
    void framew()
    {
      for (int i = 0; i < 8; i++)
      {
        lc.setRow(0, i, frame23[i]);
      }
    }
    void framex()
    {
      for (int i = 0; i < 8; i++)
      {
        lc.setRow(0, i, frame24[i]);
      }
    }
    void framey()
    {
      for (int i = 0; i < 8; i++)
      {
        lc.setRow(0, i, frame25[i]);
      }
    }
    void framez()
    {
      for (int i = 0; i < 8; i++)
      {
        lc.setRow(0, i, frame26[i]);
      }
    }
    void frameYAY()
    {
      for (int i = 0; i < 8; i++)
      {
        lc.setRow(0, i, frame27[i]);
      }
    }
    void framedone()
    {
      for (int i = 0; i < 8; i++)
      {
        lc.setRow(0, i, frame28[i]);
      }
    }
    // typedef for class function
    typedef void (setFrame::*GeneralFunction) ();

    static const GeneralFunction uploadFrames [28];
}; // end of class
//points to all of the functions
const setFrame::GeneralFunction setFrame::uploadFrames [28] =
{
  &setFrame::framea,
  &setFrame::frameb,
  &setFrame::framec,
  &setFrame::framed,
  &setFrame::framee,
  &setFrame::framef,
  &setFrame::frameg,
  &setFrame::frameh,
  &setFrame::framei,
  &setFrame::framej,
  &setFrame::framek,
  &setFrame::framel,
  &setFrame::framem,
  &setFrame::framen,
  &setFrame::frameo,
  &setFrame::framep,
  &setFrame::frameq,
  &setFrame::framer,
  &setFrame::frames,
  &setFrame::framet,
  &setFrame::framew,
  &setFrame::framex,
  &setFrame::framey,
  &setFrame::framez,
  &setFrame::frameYAY,
  &setFrame::framedone,

};

void setup() {
  //LED schtuff
  lc.shutdown(0, false); // Wake up displays
  lc.setIntensity(0, 3); // Set intensity levels
  lc.clearDisplay(0);  // Clear Displays
  Wire.begin();

  // initialize serial communication
  // (115200 chosen because it is required for Teapot Demo output, but it's
  // really up to you depending on your project)
//  Serial.begin(9600);
//  while (!Serial); // wait for Leonardo enumeration, others continue immediately

  // NOTE: 8MHz or slower host processors, like the Teensy @ 3.3v or Ardunio
  // Pro Mini running at 3.3v, cannot handle this baud rate reliably due to
  // the baud timing being too misaligned with processor ticks. You must use
  // 38400 or slower in these cases, or use some kind of external separate
  // crystal solution for the UART timer.

  // initialize device
//  Serial.println(F("Initializing I2C devices..."));
  mpu.initialize();

  // verify connection
//  Serial.println(F("Testing device connections..."));
//  Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

  // wait for ready
//  Serial.println(F("\nSend any character to begin DMP programming and demo: "));
//  while (Serial.available() && Serial.read()); // empty buffer
//  while (!Serial.available());                 // wait for data
//  while (Serial.available() && Serial.read()); // empty buffer again

  // load and configure the DMP
//  Serial.println(F("Initializing DMP..."));
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
}



// ================================================================
// ===                    MAIN PROGRAM LOOP                     ===
// ================================================================

void loop() {
  // if programming failed, don't try to do anything
  if (!dmpReady) {
    //    Serial.println("I think the program might of failed here?");
    return;
  }

  // wait for MPU interrupt or extra packet(s) available
  while (!mpuInterrupt && fifoCount < packetSize) {
    // other program behavior stuff here
    //    Serial.println("WHAT");
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
    //    Serial.println(F("FIFO overflow!"));

    // otherwise, check for DMP data ready interrupt (this should happen frequently)
  } else if (mpuIntStatus & 0x02) {
    // wait for correct available data length, should be a VERY short wait
    while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();

    // read a packet from FIFO
    mpu.getFIFOBytes(fifoBuffer, packetSize);

    // track FIFO count here in case there is > 1 packet available
    // (this lets us immediately read more without waiting for an interrupt)
    fifoCount -= packetSize;
    // display Euler angles in degrees
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetEuler(euler, &q);
    //initialize to access frames
    setFrame setFrames;
    float psi = euler[0] * 180 / M_PI;
    float theta = euler[1] * 180 / M_PI;
    float phi = euler[2] * 180 / M_PI;
    if (isNegative && phi > 0  && phi != 180) {
      oppositeSignReadingsInARow ++;
    }
    else if (!isNegative && phi < 0 && phi != -180) {
      oppositeSignReadingsInARow ++;
    }
    if (oppositeSignReadingsInARow >= threshold) {
      isNegative = !isNegative;
      oppositeSignReadingsInARow = 0;
      //reset Frame Counts
      //      Serial.println("FRAME COUNTS RESET");
      frameCount = 0;
      negFrameCount = 25;
      frameChangeCount = 0;
    }
    if (frameChangeCount == frameChangeCountThreshold) {
      //debugging
      if (frameCount == 26) {
        frameCount = 0;
      }
      if (negFrameCount == 0) {
        negFrameCount = 25;
      }
      if (!isNegative && frameCount < 26) {
        //        Serial.println(frameCount);
        setFrame::GeneralFunction f = setFrame::uploadFrames [frameCount];
        // call the function
        (setFrames.*f) ();
        frameCount ++;
      }
      else if (isNegative && negFrameCount >= 0) {
        //        Serial.println("switch WTFFF");
        setFrame::GeneralFunction f = setFrame::uploadFrames [negFrameCount];
        // call the function
        (setFrames.*f) ();
        negFrameCount --;
      }
      frameChangeCount = 0;
      //      Serial.println(frameChangeCount);
    }
    frameChangeCount++;
    //    Serial.print("euler\t");
    //    Serial.print(psi);
    //    Serial.print("\t");
    //    Serial.print(theta);
    //    Serial.print("\t");
    //    Serial.println(phi);
  }
}
