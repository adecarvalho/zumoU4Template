/*ZumoU4LIb for the bot Zumo32U4
version 06/2024
*/

#ifndef ZUMO_U4_LIB_H

#include <Arduino.h>

#define ZLED_17 17
#define ZLED_13 13
#define ZLED_PD5 PD5

#define ZBUZZER_PIN 6

#define ZPWM_LEFT 10
#define ZPWM_RIGHT 9
#define ZDIR_LEFT 16
#define ZDIR_RIGHT 15

#define ZSENSOR_LEDON 11

#define ZBUTTON_A_PIN 14

//
class ZumoU4
{
public:
    enum Zled
    {
        LED_17,
        LED_13,
        LED_PD5
    };

    // constructeur
    ZumoU4();

    virtual ~ZumoU4();

    void begin(int *tabSensorValues);

    float getBatteryVoltage();

    void setLed(Zled type, bool state);

    void buzzer(int frequency, int time_ms);

    void setLeftSpeed(int16_t speed);  // speed [-400,400]
    void setRightSpeed(int16_t speed); // speed [-400,400]

    void setMotorsSpeeds(int16_t left, int16_t right, uint32_t time_ms);

    void setMotorsStop(uint32_t time_ms);

    void LineSensorRead();

    void LineSensorActivate(bool state);

    void LineSensorPrintToSerial();

    bool isButtonAPressed();
    void waitButtonAPressed();
    void waitButtonAReleased();
    void waitButtonAPressedAndReleased();

private:
    void _beginLeds();
    void _beginBuzzer();
    void _beginMotors();
    void _beginSensor(int *tabValues);
    void _beginButton();
    //
    int *_tabSensorValues;
    uint8_t *_tabSensorPins;
};

#endif