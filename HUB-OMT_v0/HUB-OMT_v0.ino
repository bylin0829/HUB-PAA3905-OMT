
// HUB-OMT & HUB5168+ arduino demo program
// design by Tom,Yen 2024/05/16

#define DEBUG

#ifdef DEBUG
#ifndef DEBUG_H
#define DEBUG_H
#include <stdio.h>
#include <stdarg.h>

#define DBGI Serial.begin(115200)
#define BRK                                               \
  {                                                       \
    char bugSbuf[128];                                    \
    sprintf(bugSbuf, "Break @%s:%d", __FILE__, __LINE__); \
    Serial.println((String)bugSbuf);                      \
    while (0 == Serial.available())                       \
      ;                                                   \
    while (Serial.available())                            \
      Serial.read();                                      \
    Serial.println("Go!!");                               \
  }

#define DBG(...)                     \
  {                                  \
    char bugSbuf[250];               \
    sprintf(bugSbuf, __VA_ARGS__);   \
    Serial.println((String)bugSbuf); \
  }
#endif // DEBUG_H
#endif // DEBUG

// #define BRUST_READ

#include "Wire.h"
#include <MPU6050_light.h> // MPU6050 G-sensor 程式庫

// MPU6050 mpu(Wire); // 使用I2C MPU-6050物件

#include "src/hub5168p.h"
#include "src/PAA3905_lib.h" // 使用PAA3905程式庫

long timer = 0;

PAA3905 paa;

void setup()
{
  Serial.begin(115200);
  Wire.begin();
  // byte status = mpu.begin(); // 啓始MPU6050
  // DBG("MPU6050 status: %d", status);
  // while (status != 0)
  // {
  // } // stop everything if could not connect to MPU6050
  DBG("Calculating offsets, do not move MPU6050");
  delay(1000);
  // mpu.calcOffsets(true, true); // gyro and accelero
  DBG("Done!");
  DBG("PAA Error %d", paa.begin()); // 啓始PAA3905
  Serial.println("OMT example");
}

void loop()
{
  // mpu.update();

  if (millis() - timer > 100)
  {                             // print data every 0.1 second
    int dx = 0, dy = 0, op = 0; // op 0: nothing, 1: high light 2: low light 3:very low light
    uint8_t sql = 0;
    uint32_t sht = 0;

#ifndef BRUST_READ
    if (op = paa.readMotion(&dx, &dy))
    { // 讀取PAA3905資料
      paa.getSqualShutter(&sql, &sht);
    }
#else
    motionDataT mD;

    if (op = paa.readMotion(&mD))
    {
      paa.getSqualShutter(&sql, &sht);
      dx = paa.getDeltaX();
      dy = paa.getDeltaY();
    }
#endif
    // 輸出HUB-OMT資料及讀取MPU6050
    /*
    DBG("OP=%d Sql=%d Sht=%d Dx=%d Dy=%d Temp=%f angleX=%f angleY=%f angleZ=%f accX=%f accY=%f accZ=%f",
        op, sql, sht, dx, dy, mpu.getTemp(),
        mpu.getAngleX(), mpu.getAngleY(), mpu.getAngleZ(),
        mpu.getAccX(), mpu.getAccY(), mpu.getAccZ());
        */
    DBG("Dx=%d Dy=%d OP=%d Sql=%d Sht=%d", dx, dy, op, sql, sht);
    timer = millis();
  }
}
