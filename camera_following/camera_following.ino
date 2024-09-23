// PAA3905 & HUB5168+ Demo program
// design by Danny,Lin 2024/09/22

#define DEBUG

#ifdef DEBUG
#ifndef DEBUG_H
#define DEBUG_H
#include <stdio.h>
#include <stdarg.h>

#define BRK                                                   \
    {                                                         \
        char bugSbuf[128];                                    \
        sprintf(bugSbuf, "Break @%s:%d", __FILE__, __LINE__); \
        Serial.println((String)bugSbuf);                      \
        while (0 == Serial.available())                       \
            ;                                                 \
        while (Serial.available())                            \
            Serial.read();                                    \
        Serial.println("Go!!");                               \
    }

#define DBG(...)                         \
    {                                    \
        char bugSbuf[250];               \
        sprintf(bugSbuf, __VA_ARGS__);   \
        Serial.println((String)bugSbuf); \
    }
#endif // DEBUG_H
#endif // DEBUG

// #define BRUST_READ

#include <MPU6050_light.h> // MPU6050 G-sensor 程式庫
#include <AmebaServo.h>
#include "Wire.h"
#include "src/hub5168p.h"
#include "src/PAA3905_lib.h" // 使用PAA3905程式庫

// MACRO
// distance map dy pixel per cm
#define dy2CmConverter(d, dy) ((d > 8.0) ? (-0.0007 * d * d * d + 0.1043 * d * d - 5.3869 * d + 96.122) : (0.0))

// function prototype
float ultrasonicUpdate(void);

// class instance
PAA3905 paa;
AmebaServo myservo;

// pinout
const int myservo_PWM_pin = 3;
const int trigger_pin = 8;
const int echo_pin = 7;

// variable
float servoPos = 90.0;
long omtTimer = 0, servoTimer = 0;
float distanceCurrent = 0.0, distanceOld = 0.0;

void setup()
{
    Serial.begin(115200);
    Wire.begin();
    pinMode(trigger_pin, OUTPUT);
    pinMode(echo_pin, INPUT);
    myservo.attach(myservo_PWM_pin);

    delay(1000);
    myservo.write(90);
    DBG("Please put target in front of device.");
    delay(3000);

    DBG("Done!");
    DBG("PAA Error Code %d", paa.begin());
    Serial.println("Start");
}

void loop()
{
    if (millis() - omtTimer > 50)
    {                               // print data every 0.1 second
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
        distanceCurrent = ultrasonicUpdate();
        if (dy == 0 || distanceCurrent < 8.0)
            return;

        if (abs(distanceCurrent - distanceOld) < 30.0)
        {
            float dyPixelRatio, dyPixel2Cm;
            // pixel to cm
            if (distanceCurrent > 50.0)
            {
                dyPixelRatio = dy2CmConverter(50.0, abs(dy));
            }
            else
            {
                dyPixelRatio = dy2CmConverter(distanceCurrent, abs(dy));
            }
            dyPixel2Cm = abs(dy) / dyPixelRatio;
            float sign = (dy > 0.0) ? 1.0 : -1.0;
            float radTan = tan(dyPixel2Cm * sign / distanceCurrent);
            float deg = radTan * 57.29;
            servoPos += deg;
            servoPos = (servoPos > 130) ? 130 : ((servoPos < 50) ? 50 : servoPos);
            DBG("Dy=%2d dist=%.1f dist_old:%.1f pixelRatio:%.2f pixel2cm:%.2f deg:%.2f radTan:%.3f servoPos:%.1f", dy, distanceCurrent, distanceOld, dyPixelRatio, dyPixel2Cm, deg, radTan, servoPos);
            distanceOld = distanceCurrent; // update value
        }

        if (millis() - servoTimer > 500)
        {
            myservo.write(servoPos);
            servoTimer = millis();
        }
        omtTimer = millis();
    }
}

float ultrasonicUpdate()
{
    float duration, distance;

    // trigger a 10us HIGH pulse at trigger pin
    digitalWrite(trigger_pin, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigger_pin, LOW);

    // measure time cost of pulse HIGH at echo pin
    duration = pulseIn(echo_pin, HIGH);

    // calculate the distance from duration
    distance = duration / 58;

    // wait for next calculation
    // delay(2000);
    return distance;
}