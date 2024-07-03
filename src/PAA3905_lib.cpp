
#include "PAA3905_lib.h"

#define DEBUG

#ifdef DEBUG
#ifndef DEBUG_H
#define DEBUG_H
  #include <stdio.h> 
  #include <stdarg.h>

  #define DBGI Serial.begin(115200)
  #define BRK {char bugSbuf[128]; \
    sprintf(bugSbuf,"Break @%s:%d",__FILE__,__LINE__); \
    Serial.println((String)bugSbuf); \
    while (0==Serial.available()); \
    while (Serial.available()) Serial.read(); \
    Serial.println("Go!!");}
  
  #define DBG(...) {char bugSbuf[250];sprintf(bugSbuf,__VA_ARGS__);Serial.println((String)bugSbuf);}
#endif  // DEBUG_H 
#endif  // DEBUG

#define dly1u delayMicroseconds(1)
#define dly4u delayMicroseconds(4)

void PAA3905::tr8b(uint8_t data){        // transfer 8bits
    int i;
    uint8_t bitmap = 0b10000000;
    for(i=0;i<8;i++){
      if (bitmap & data) mosi1;
      else mosi0;
      dly1u; 
      scp1;
      dly1u; 
      scp0;
      bitmap=bitmap>>1;
    }
}

uint8_t PAA3905::rd8b(){                // read 8bits
    uint8_t data;
    data=0b00000000;
    uint8_t bitmap = 0b10000000;
    for(int i=0;i<8;i++){
        dly1u; 
        scp1;
        if (miso) data = data | bitmap;
        dly1u; 
        scp0;
        bitmap=bitmap>>1;
    }
    return data;
}

PAA3905::PAA3905() {
    enhancement=0;
    bEnableJitterSuppression = true;
    nCSpin = PIN_NCS;                   // initial SPI default pin number
    sCLKpin = PIN_CLK;
    mISOpin = PIN_MISO;
    mOSIpin = PIN_MOSI;
}

/****************************************************************************************
  Section 6.4, 6.5  : Write and Read operations and required timing in between
*****************************************************************************************/

// Register write function
void PAA3905::regWrite(uint8_t address, uint8_t value) {
    ss0;
    delayMicroseconds(TIME_us_TNCS_SCLK);
    tr8b(address | 0x80);       // send the address
    delayMicroseconds(10);
    tr8b(value);                // send the value
    delayMicroseconds(TIME_us_TSCLK_NCS_W);
    ss1;
}

void PAA3905::regWrite(uint8_t address, uint8_t value, int delayMicroS) {
    regWrite(address,value);
    delayMicroseconds(delayMicroS);
}

// Register Read function
uint8_t PAA3905::regRead(uint8_t address) {

    uint8_t result = 0;                      // result to return

    ss0;
    delayMicroseconds(TIME_us_TNCS_SCLK);
    tr8b(address & 0x7f);                         // send address of read
    delayMicroseconds(TIME_us_TSRAD);
    result=rd8b();
    delayMicroseconds(TIME_us_TSCLK_NCS_R);
    ss1;   
    return result;   
}

uint8_t PAA3905::regRead(unsigned char address, int delayMicroS){
    uint8_t v;
    v = regRead(address);
    delayMicroseconds(delayMicroS);
    return v;
}

int PAA3905::begin(int mode,int csp,int clkp,int mip,int mop){
    enhancement=mode;
    nCSpin=csp;
    sCLKpin=clkp;
    mISOpin=mip;
    mOSIpin=mop;
    return begin();
}

int PAA3905::begin(int mode){
    enhancement=mode;
    return begin();
}

int PAA3905::begin(){
    pinMode(nCSpin,OUTPUT);
    ss1;
    pinMode(sCLKpin,OUTPUT);
    scp0;
    pinMode(mOSIpin,OUTPUT);
    mosi0;
    pinMode(mISOpin,INPUT);
    return pwrOnReset();
}

/*********************************
  Section 7.1.1 Initial Flow
*********************************/
// return 0 is power on reset ok, return non-zero is fail
int PAA3905::pwrOnReset()
{
   // Power up sequence
   // Drive NCS high
   ss1;
   delay(1);
   // Drive NCS low
   ss0;
   // Read product ID to verify
   ProductID = regReadT(0x00);
   if (ProductID != PROD_PAA3905)
   {
      // Wrong Product ID, error handling ...
      return 1;
   }
   // Write 0x5a to Power_Up_Reset register (0x3a)
   regWrite(0x3a, 0x5a);
   delay(1);  // delay 1ms
   // Read register 0x02 to 0x06
   for (int i = 0; i < 5; i++)
   {
        regReadT(0x02+i);
   }
   // Init performance optimization registers as per section 7.1.2
   init();
   return 0;
}

/***************************************************
  Section 7.1.2 Initialization Register Setting
***************************************************/
void PAA3905::init()
{
    regWrite(0x3a, 0x5a);
    if (enhancement == 0){
        regWriteT(0x7F, 0x00);
        regWriteT(0x51, 0xFF);
        regWriteT(0x4E, 0x2A);
        regWriteT(0x66, 0x3E);
        regWriteT(0x7F, 0x14);
        regWriteT(0x7E, 0x71);
        regWriteT(0x55, 0x00);
        regWriteT(0x59, 0x00);
        regWriteT(0x6F, 0x2C);
        regWriteT(0x7F, 0x05);
        regWriteT(0x4D, 0xAC);
        regWriteT(0x4E, 0x32);
        regWriteT(0x7F, 0x09);
        regWriteT(0x5C, 0xAF);
        regWriteT(0x5F, 0xAF);
        regWriteT(0x70, 0x08);
        regWriteT(0x71, 0x04);
        regWriteT(0x72, 0x06);
        regWriteT(0x74, 0x3C);
        regWriteT(0x75, 0x28);
        regWriteT(0x76, 0x20);
        regWriteT(0x4E, 0xBF);
        regWriteT(0x7F, 0x03);
        regWriteT(0x64, 0x14);
        regWriteT(0x65, 0x0A);
        regWriteT(0x66, 0x10);
        regWriteT(0x55, 0x3C);
        regWriteT(0x56, 0x28);
        regWriteT(0x57, 0x20);
        regWriteT(0x4A, 0x2D);
        regWriteT(0x4B, 0x2D);
        regWriteT(0x4E, 0x4B);
        regWriteT(0x69, 0xFA);
        regWriteT(0x7F, 0x05);
        regWriteT(0x69, 0x1F);
        regWriteT(0x47, 0x1F);
        regWriteT(0x48, 0x0C);
        regWriteT(0x5A, 0x20);
        regWriteT(0x75, 0x0F);
        regWriteT(0x4A, 0x0F);
        regWriteT(0x42, 0x02);
        regWriteT(0x45, 0x03);
        regWriteT(0x65, 0x00);
        regWriteT(0x67, 0x76);
        regWriteT(0x68, 0x76);
        regWriteT(0x6A, 0xC5);
        regWriteT(0x43, 0x00);
        regWriteT(0x7F, 0x06);
        regWriteT(0x4A, 0x18);
        regWriteT(0x4B, 0x0C);
        regWriteT(0x4C, 0x0C);
        regWriteT(0x4D, 0x0C);
        regWriteT(0x46, 0x0A);
        regWriteT(0x59, 0xCD);
        regWriteT(0x7F, 0x0A);
        regWriteT(0x4A, 0x2A);
        regWriteT(0x48, 0x96);
        regWriteT(0x52, 0xB4);
        regWriteT(0x7F, 0x00);
        regWriteT(0x5B, 0xA0);
   }else{
        regWriteT(0x7F, 0x00);
        regWriteT(0x51, 0xFF);
        regWriteT(0x4E, 0x2A);
        regWriteT(0x66, 0x26);
        regWriteT(0x7F, 0x14);
        regWriteT(0x7E, 0x71);
        regWriteT(0x55, 0x00);
        regWriteT(0x59, 0x00);
        regWriteT(0x6F, 0x2C);
        regWriteT(0x7F, 0x05);
        regWriteT(0x4D, 0xAC);
        regWriteT(0x4E, 0x65);
        regWriteT(0x7F, 0x09);
        regWriteT(0x5C, 0xAF);
        regWriteT(0x5F, 0xAF);
        regWriteT(0x70, 0x00);
        regWriteT(0x71, 0x00);
        regWriteT(0x72, 0x00);
        regWriteT(0x74, 0x14);
        regWriteT(0x75, 0x14);
        regWriteT(0x76, 0x06);
        regWriteT(0x4E, 0x8F);
        regWriteT(0x7F, 0x03);
        regWriteT(0x64, 0x00);
        regWriteT(0x65, 0x00);
        regWriteT(0x66, 0x00);
        regWriteT(0x55, 0x14);
        regWriteT(0x56, 0x14);
        regWriteT(0x57, 0x06);
        regWriteT(0x4A, 0x20);
        regWriteT(0x4B, 0x20);
        regWriteT(0x4E, 0x32);
        regWriteT(0x69, 0xFE);
        regWriteT(0x7F, 0x05);
        regWriteT(0x69, 0x14);
        regWriteT(0x47, 0x14);
        regWriteT(0x48, 0x1c);
        regWriteT(0x5A, 0x20);
        regWriteT(0x75, 0xE5);
        regWriteT(0x4A, 0x05);
        regWriteT(0x42, 0x04);
        regWriteT(0x45, 0x03);
        regWriteT(0x65, 0x00);
        regWriteT(0x67, 0x50);
        regWriteT(0x68, 0x50);
        regWriteT(0x6A, 0xC5);
        regWriteT(0x43, 0x00);
        regWriteT(0x7F, 0x06);
        regWriteT(0x4A, 0x1E);
        regWriteT(0x4B, 0x1E);
        regWriteT(0x4C, 0x34);
        regWriteT(0x4D, 0x34);
        regWriteT(0x46, 0x32);
        regWriteT(0x59, 0x0D);
        regWriteT(0x7F, 0x0A);
        regWriteT(0x4A, 0x2A);
        regWriteT(0x48, 0x96);
        regWriteT(0x52, 0xB4);
        regWriteT(0x7F, 0x00);
        regWriteT(0x5B, 0xA0);
   };
}



/************************************************************
  Section 7.2.1 Automatic Switching of Operation Modes
*************************************************************/
void PAA3905::opModes_012()
{
    regWriteT(0x7F, 0x08);
    regWriteT(0x68, 0x02);
    regWriteT(0x7F, 0x00);
}

void PAA3905::opModes_01()
{
    regWriteT(0x7F, 0x08);
    regWriteT(0x68, 0x01);
    regWriteT(0x7F, 0x00);
}

/******************************
  Section 7.3.1 Burst Read
*******************************/
int PAA3905::readMotion(motionDataT *motionD)
{
    int i = 0, j = 0;
    char *buff;
    buff = (char *)motionD;
    
    // Lower NCS and wait for Tncs-sclk
    ss0;
    delayMicroseconds(TIME_us_TNCS_SCLK);
    tr8b(0x16);     // Transfer the motion burst address
    mosi1;          // Held the MOSI high
    delayMicroseconds(TIME_us_TSRAD);    // Delay Tsrad
    // Read all motion data
    for (i = 0; i < MOTION_BURST_SIZE; i++)     // burst read motion data
    {
        *buff = rd8b();
        buff++;
    }
    // Lower MOSI line
    mosi0;
    // Raise NCS and hold for Tbexit
    ss1;
    delayMicroseconds(TIME_us_TBEXIT);

    uint8_t mFlag;
    mFlag = ((0x80 & (motionD->motion))==0x80)? 1:0;
    if (mFlag==0) return 0;
    
    // Construct the 3 bytes long shutter value into 32 bits value
    shutter.B[0] = motionD->shutter_l;
    shutter.B[1] = motionD->shutter_h;
    shutter.B[2] = motionD->shutter_hh;
    shutter.B[3] = 0;

    squal = motionD->squal;

    // Get the Brightness operation mode from Observation register bits[7:6]
    opMode = (motionD->observation >> 6) & 0x03;

    deltaX.B[0] = motionD->deltaX_L;
    deltaX.B[1] = motionD->deltaX_H;
    deltaY.B[0] = motionD->deltaY_L;
    deltaY.B[1] = motionD->deltaY_H;  

    // Check Invalid motion data condition, as per section 7.4.3
    if (opMode == opBRIGHT)
    {
        i = 0x19;
        j = 0x00ff80;
    }
    else if (opMode == opBRIGHT)
    {
        i = 0x46;
        j = 0x00ff80;
    }
    else  // SUPER_LOW_LIGHT
    {
        i = 0x55;
        j = 0x025998;
    };

    if (bEnableJitterSuppression)
    {   
        // Squal less than i        and    Shutter >= j
        if ((squal < i) && (shutter.i >= j))
        {
            deltaX.i=0;
            deltaY.i=0;
            motionD->deltaX_L=0;
            motionD->deltaX_H=0;
            motionD->deltaY_L=0;
            motionD->deltaY_H=0;
            return 0;
        }
    }
    return opMode+1;
}

int PAA3905::readMotion(int *dX,int *dY){
    uint8_t mFlag;
    mFlag = ((0x80 & regReadT(0x02))==0x80)? 1:0;
    if (mFlag==0) return 0;
    observation=regReadT(0x15);
    deltaX.B[0]=regReadT(0x03);
    deltaX.B[1]=regReadT(0x04);
    deltaY.B[0]=regReadT(0x05);
    deltaY.B[1]=regReadT(0x06);
    squal=regReadT(0x07);
    // Construct the 3 bytes long shutter value into 32 bits value
    shutter.B[0] = regReadT(0x0b);
    shutter.B[1] = regReadT(0x0c);
    shutter.B[2] = regReadT(0x0d);
    shutter.B[3] = 0;
    opMode = (observation >> 6) & 0x03;

    // Check Invalid motion data condition, as per section 7.4.3
    uint8_t si=0;
    uint32_t sj=0;
    if (opMode == opBRIGHT)
    {
        si = 0x19;
        sj = 0x00ff80;
    }
    else if (opMode == opBRIGHT)
    {
        si = 0x46;
        sj = 0x00ff80;
    }
    else  // SUPER_LOW_LIGHT
    {
        si = 0x55;
        sj = 0x025998;
    };

    if (bEnableJitterSuppression)
    {   
        // Squal less than i and Shutter >= j
        if ((squal < si) && (shutter.ui >= sj))
        {
            *dX=deltaX.i=0;
            *dY=deltaY.i=0;
            return 0;
        }
    }  
    *dX=deltaX.i;   
    *dY=deltaY.i; 
    return opMode+1;
}

void PAA3905::getSqualShutter(uint8_t *sql, uint32_t *sht){
    *sql= squal;
    *sht= shutter.ui;
}

int PAA3905::getDeltaX(){
    return deltaX.i;
}

int PAA3905::getDeltaY(){
    return deltaY.i;
}

/*********************************
  Section 7.4.5 Raw Data Output
**********************************/
// Step 1 to enter frame capture mode

void PAA3905::frameCaptureSetup()
{  
    regWriteT(0x7f, 0x07);
    regWriteT(0x4d, 0x11);
    regWriteT(0x4c, 0x00);
    regWriteT(0x7f, 0x08);
    regWriteT(0x6a, 0x38);
    regWriteT(0x7f, 0x00);
    regWriteT(0x55, 0x04);
    regWriteT(0x40, 0x80);
    regWriteT(0x4d, 0x11);
}

// Step 2 to 6 of reading the whole array of raw data
#define READ_MAX_CYCLE  2000    // Maximum cycles of read before error
int  PAA3905::frameCapture(char *pixels)
{
    int i, j;
    int retry = 0;
    char *framData;
    framData=pixels;

    // Note during frame capture, stop motion temporary
    // Start the frame capture
    regWriteT(0x7f, 0x00);
    regWriteT(0x67, 0x25);
    regWriteT(0x55, 0x20);
    regWriteT(0x7f, 0x13);
    regWriteT(0x42, 0x01);
    regWriteT(0x7f, 0x00);
    regWriteT(0x0f, 0x11);
    regWriteT(0x0f, 0x13);
    regWriteT(0x0f, 0x11);
    // Read register 0x10 until bit 0 = '1'
    retry = 0;
    while (retry < READ_MAX_CYCLE)
    {
        if (IS_BIT_SET(regRead(0x10), BIT0))
        {
            break;
        }
        delayMicroseconds(TIME_us_TSRW_TSRR);
        retry++;
    }
    if (retry >= READ_MAX_CYCLE) 
    {
        // Error reading back 0x10
        return 0x10;
    };
  
    // Read row by row
    for (i = 0; i < ARRAY_ROW; i++)
    {
    // Read col by col for each row
        for (j = 0; j < ARRAY_COL; j++)
        {
            *framData = regReadT(0x13);
            framData++;
        }
    }
    return 0;
}

// Step 8 to exit frame capture mode
void PAA3905::frameCaptureExit()
{  
    regWriteT(0x7f, 0x00);
    regWriteT(0x4d, 0x11);
    regWriteT(0x40, 0x80);
    regWriteT(0x55, 0x80);

    regWriteT(0x7f, 0x08);
    regWriteT(0x6a, 0x18);

    regWriteT(0x7f, 0x07);
    regWriteT(0x41, 0x0d);
    regWriteT(0x4c, 0x80);
    regWriteT(0x7f, 0x00);
}

// Arduino timer interrupt service,setup so to be called every 1 ms
void PAA3905::timer1msTrigger() 
{
    if (t1_data > 0)
    {
        t1_data--;
        if(t1_data == 0)
        {
            endFrame();
        }
    }
}

// Step 2 & 3 Stop the normal operation of current frame and setup timer
// Should be call in Power_ON_Reset(), after InitRegisters()
void PAA3905::initFrameSync()
{
    regWriteT(0x7f, 0x07);
    regWriteT(0x41, 0x82);
    regWriteT(0x7f, 0x00);
    delay(100); // delay 100ms
    startFrame();
}

// Step 5 Start of frame and reading motion
void PAA3905::startFrame()
{
    // Read AMS mode from register 0x15
    unsigned char val = (regReadT(0x15)) >> 6;

    regWriteT(0x1f, 0x00);
  
    // Setup and start timer (codes as per in Arduino demo kit)
    t1_data = sync_interval_mode[val];    // interval is based on the current AMS mode

    // Read all devices motion after this, to within 300us
    // ReadMotion(device 1);
    // ReadMotion(device 2);
    // ReadMotion(device 3);
}

// Step 6 & 7 End of timer check and repeat for new frame
void PAA3905::endFrame()
{
    // To continue
    startFrame();
    // if not, just dont start the timer and reset the chip
}

void PAA3905::wakeUp()
{
    ss1;
    delay(1);
    regWrite(0x3b,0xc7);
    delay(1);
    regWrite(0x3b,0x00);
    delay(1);
    regReadT(0x02);
    regReadT(0x03);
    regReadT(0x04);
    regReadT(0x05);
    regReadT(0x06);
}

void PAA3905::shutDown()
{
    regWrite(0x3b,0xb6);
    delay(1);
}
