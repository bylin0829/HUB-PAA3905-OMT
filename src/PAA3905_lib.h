// PAA3905 arduino library header file v0.2
// programmer : Tom, Yen 

#ifndef _PAA3905_lib_H_
#define _PAA3905_lib_H_

#include <arduino.h>

#define hub5168p_master
// #define UNO_master

#ifdef hub5168p_master
    #define PIN_NCS 9   
    #define PIN_CLK 10
    #define PIN_MISO 11
    #define PIN_MOSI 12
#endif

#ifdef UNO_master
    #define PIN_NCS 10 // output pin at Arduino UNO that control the NCS pin 
    #define PIN_CLK 13
    #define PIN_MISO 12
    #define PIN_MOSI 11
#endif

// Step 2 to 6 of reading the whole array of raw data
#define READ_MAX_CYCLE  2000    // Maximum cycles of read before error

// Time delay for various operations
#define TIME_us_TSWW          11 //  - actually 10.5us, but Arduino delay function only accept round number
#define TIME_us_TSWR           6
#define TIME_us_TSRW_TSRR      2 //  - actually 1.5us, but Arduino delay function only accept round number
#define TIME_us_TSRAD          2
#define TIME_us_TSCLK_NCS_W    2
#define TIME_us_TBEXIT         1    // should be 500ns
#define TIME_us_TNCS_SCLK      1    // should be 120ns
#define TIME_us_TSCLK_NCS_R    1    // should be 120ns

// Bit Define
#define BIT0 (1 << 0)
#define BIT1 (1 << 1)
#define BIT2 (1 << 2)
#define BIT3 (1 << 3)
#define BIT4 (1 << 4)
#define BIT5 (1 << 5)
#define BIT6 (1 << 6)
#define BIT7 (1 << 7)
// Helper macro
#define BIT_CLEAR(REG,MASK)        (REG &= ~MASK)
#define BIT_SET(REG,MASK)          (REG |=  MASK)
#define IS_BIT_CLEAR(REG,MASK)    ((REG & MASK)==0x00)
#define IS_BIT_SET(REG,MASK)      ((REG & MASK)==MASK)

// Motion read size
#define MOTION_BURST_SIZE   14
#define MOTION_REPORT_SIZE  MOTION_BURST_SIZE

// Array define for frame capture
#define ARRAY_COL        35
#define ARRAY_ROW        35
#define ARRAY_SIZE       (ARRAY_COL*ARRAY_ROW)

// Product ID Define
#define PROD_PAA3905    0xA2

// Union for 16-bit value
typedef union 
{
    uint8_t B[2];
    uint16_t ui;
    int16_t i;
} n16bitT;

typedef union 
{
    uint8_t B[4];
    uint32_t ui;
    int32_t i;
} n32bitT;


// Structure for motion data
typedef struct       // Burst reading from register 0x50
{
    uint8_t motion;        // BYTE 0
    uint8_t observation;   // BYTE 1
    uint8_t deltaX_L;      // BYTE 2
    uint8_t deltaX_H;      // BYTE 3
    uint8_t deltaY_L;      // BYTE 4
    uint8_t deltaY_H;      // BYTE 5
    uint8_t csd_type;      // BYTE 6
    uint8_t squal;         // BYTE 7
    uint8_t rd_sum;        // BYTE 8
    uint8_t rd_max;        // BYTE 9
    uint8_t rd_min;        // BYTE 10
    uint8_t shutter_hh;    // BYTE 11
    uint8_t shutter_h;     // BYTE 12
    uint8_t shutter_l;     // BYTE 13
} motionDataT;

enum OperationModes
{
    BRIGHT,
    LOW_LIGHT,
    SUPER_LOW_LIGHT,
};

#define opBRIGHT 0
#define opLOW_LIGHT 1
#define opSUPER_LOW_LIGHT 2

#define MODE01_FRAMEPERIOD       ( 9)     // 9.0ms
#define MODE2_FRAMEPERIOD        (22)     // 20ms
#define ss0 digitalWrite(nCSpin,LOW)
#define ss1 digitalWrite(nCSpin, HIGH)
#define scp0 digitalWrite(sCLKpin,LOW)
#define scp1 digitalWrite(sCLKpin,HIGH)
#define miso digitalRead(mISOpin)
#define mosi0 digitalWrite(mOSIpin,LOW)
#define mosi1 digitalWrite(mOSIpin,HIGH)

class PAA3905{
    private:
        uint8_t ProductID;                      // Chip Product ID value
        int enhancement=0;
        bool bEnableJitterSuppression = true;   // Enable jitter suppression
        n32bitT shutter;                        // Shutter value in 32 bits format   
        n16bitT deltaX,deltaY;
        uint8_t observation;
        uint8_t squal; 
        uint8_t opMode;                         // To store the current operation mode
        unsigned int t1_data = 0;
        unsigned char sync_interval_mode[3] = { MODE01_FRAMEPERIOD, MODE01_FRAMEPERIOD, MODE2_FRAMEPERIOD }; // individual sync interval for modes
        int nCSpin,sCLKpin,mISOpin,mOSIpin;     // spi pins for PAA3905
        void tr8b(uint8_t data);
        uint8_t rd8b();
        void init();
        void regWrite(uint8_t address, uint8_t value);                          // Register write function
        void regWrite(uint8_t address, uint8_t value,int delayMicroS);          // Register write function with delay microseconds
        void regWriteT(uint8_t address, uint8_t value) { regWrite(address,value,TIME_us_TSWW); };   // Function that perform register write and wait for Tsww microseconds
        uint8_t regRead(uint8_t address);                                       // Register read function
        uint8_t regRead(uint8_t address,int delayMicroS);                       // Register read function with delay microseconds
        uint8_t regReadT(uint8_t address) { return regRead(address,TIME_us_TSRW_TSRR);}; // Function that perform register read and wait for Tsrw/Tsrr microseconds
        int pwrOnReset();                           // Power on reset & initial setup registers
    public:
        PAA3905();
        ~PAA3905(){};  
        int begin(int mode,int csp,int clkp,int mip,int mop);
        int begin(int csp,int clkp,int mip,int mop);
        int begin(int mode);
        int begin();

        int readMotion(int *dX,int *dY);            // single read mode, return "opMode+1" is successed, Don't read anything is return 0
        int readMotion(motionDataT *motionX);       // brust read mode, return "opMode+1" is successed, Don't read anything is return 0
        int getDeltaX();                            // return delta X
        int getDeltaY();                            // return delta Y
        void getSqualShutter(uint8_t *sql, uint32_t *sht); 
        
        void shutDown();                            // shutdown chip
        void wakeUp();                              // wake up chip
        
        void opModes_012();                         // Automatic Switching of (0,1,2) Operation Modes
        void opModes_01();                          // Automatic Switching of (0,1) Operation Modes
        
        void frameCaptureSetup();
        int  frameCapture(char *pixels);
        void frameCaptureExit();
        void initFrameSync();
        void startFrame();
        void endFrame();
        void timer1msTrigger();                     // 1ms trigger
};

#endif 
