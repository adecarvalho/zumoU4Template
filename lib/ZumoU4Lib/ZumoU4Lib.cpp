#include <ZumoU4Lib.h>
//
uint8_t theSensorPins[5] = {18, 20, 21, 4, 12};
//
ZumoU4::ZumoU4()
{
}
//
ZumoU4::~ZumoU4()
{
}
//
void ZumoU4::begin(int *tabSensorValues)
{
    _beginSensor(tabSensorValues);
    _beginLeds();
    _beginBuzzer();
    _beginMotors();
    _beginButton();
}
//
float ZumoU4::getBatteryVoltage()
{
    float res = 0;

    uint16_t num = analogRead(A1);
    res = (5.0 * num) / 1023.0;
    res = res * 2;

    return res;
}
//
void ZumoU4::setLed(Zled type, bool state)
{
    switch (type)
    {
    case LED_17:
        pinMode(ZLED_17, OUTPUT);
        digitalWrite(ZLED_17, !state);
        break;
    //
    case LED_13:
        pinMode(ZLED_13, OUTPUT);
        digitalWrite(ZLED_13, state);
        break;
        //
    case LED_PD5:
        pinMode(ZLED_PD5, OUTPUT);
        digitalWrite(ZLED_PD5, state);
        break;

    default:
        break;
    }
}
//
void ZumoU4::buzzer(int frequency, int time_ms)
{
    if (frequency < 131)
    {
        frequency = 131;
    }
    if (frequency > 3953)
    {
        frequency = 3953;
    }
    //
    uint32_t peri = 1000000 / frequency;
    bool state = false;

    for (int32_t len = 0; len < (long)time_ms * 1000; len += peri)
    {
        state = !state;
        digitalWrite(ZBUZZER_PIN, state);
        delayMicroseconds(peri - 50);
    }
    //
    delay(time_ms);
    digitalWrite(ZBUZZER_PIN, 0);
}
//
void ZumoU4::_beginLeds()
{
    pinMode(ZLED_17, OUTPUT);
    pinMode(ZLED_13, OUTPUT);
    pinMode(ZLED_PD5, OUTPUT);
    //
    digitalWrite(ZLED_17, 0);
    digitalWrite(ZLED_17, 0);
    digitalWrite(ZLED_17, 0);
}
//
void ZumoU4::_beginBuzzer()
{
    pinMode(ZBUZZER_PIN, OUTPUT);
    digitalWrite(ZBUZZER_PIN, 0);
}
//
void ZumoU4::_beginMotors()
{
    pinMode(ZPWM_LEFT, OUTPUT);
    pinMode(ZPWM_RIGHT, OUTPUT);
    pinMode(ZDIR_LEFT, OUTPUT);
    pinMode(ZDIR_LEFT, OUTPUT);
    // timer 1
    // phase correct PWM
    // max 400
    // freq: 16Mhz/1 (prescale)/2 (phase correct)/400 (top)=20KHz
    TCCR1A = 0b10100000;
    TCCR1B = 0b00010001;
    ICR1 = 400;
    OCR1A = 0;
    OCR1B = 0;
}
//
void ZumoU4::setLeftSpeed(int16_t speed)
{
    bool reverse = false;
    if (speed < 0)
    {
        speed = -speed;
        reverse = true;
    }
    //
    if (speed > 400)
    {
        speed = 400;
    }
    //
    OCR1B = speed;
    if (reverse)
    {
        digitalWrite(ZDIR_LEFT, 1);
    }
    else
    {
        digitalWrite(ZDIR_LEFT, 0);
    }
}
//
void ZumoU4::setRightSpeed(int16_t speed)
{
    bool reverse = false;
    if (speed < 0)
    {
        speed = -speed;
        reverse = true;
    }
    //
    if (speed > 400)
    {
        speed = 400;
    }
    //
    OCR1A = speed;
    if (reverse)
    {
        digitalWrite(ZDIR_RIGHT, 1);
    }
    else
    {
        digitalWrite(ZDIR_RIGHT, 0);
    }
}
//
void ZumoU4::setMotorsSpeeds(int16_t left, int16_t right, uint32_t time_ms)
{
    setLeftSpeed(left);
    setRightSpeed(right);
    //
    if (time_ms > 0)
    {
        delay(time_ms);
        setMotorsStop(0);
    }
}
//
void ZumoU4::setMotorsStop(uint32_t time_ms)
{
    setLeftSpeed(0);
    setRightSpeed(0);

    if (time_ms > 0)
    {
        delay(time_ms);
    }
}
//
void ZumoU4::_beginSensor(int *tabValues)
{
    _tabSensorValues = tabValues;
    _tabSensorPins = theSensorPins;
    //
    pinMode(ZSENSOR_LEDON, OUTPUT);
    digitalWrite(ZSENSOR_LEDON, 1);
    delayMicroseconds(200);
}
//
void ZumoU4::LineSensorRead()
{
    ///////////
    unsigned char i;

    for (i = 0; i < 5; i++)
    {
        _tabSensorValues[i] = 2000;
        digitalWrite(_tabSensorPins[i], HIGH); // make sensor line an output
        pinMode(_tabSensorPins[i], OUTPUT);    // drive sensor line high
    }

    delayMicroseconds(10); // charge lines for 10 us

    for (i = 0; i < 5; i++)
    {
        pinMode(_tabSensorPins[i], INPUT);    // make sensor line an input
        digitalWrite(_tabSensorPins[i], LOW); // important: disable internal pull-up!
    }

    unsigned long startTime = micros();
    while (micros() - startTime < 2000)
    {
        int time = micros() - startTime;
        for (i = 0; i < 5; i++)
        {
            if (digitalRead(_tabSensorPins[i]) == LOW && time < _tabSensorValues[i])
                _tabSensorValues[i] = time;
        }
    }
    //////////
}
//
void ZumoU4::LineSensorActivate(bool state)
{
    if (state)
    {
        digitalWrite(ZSENSOR_LEDON, 1);
    }
    else
    {
        digitalWrite(ZSENSOR_LEDON, 0);
    }
    //
    delayMicroseconds(200);
}
//
void ZumoU4::LineSensorPrintToSerial()
{
    Serial.println("------------");
    for (uint8_t i = 0; i < 5; i++)
    {
        Serial.print("Sensor: ");
        Serial.print(i);
        Serial.print(">");
        Serial.println(_tabSensorValues[i]);
        delay(50);
    }
    delay(500);
}
//
void ZumoU4::_beginButton()
{
    pinMode(ZBUTTON_A_PIN, INPUT_PULLUP);
}
//
bool ZumoU4::isButtonAPressed()
{
    bool res = false;
    if (digitalRead(ZBUTTON_A_PIN) == LOW)
    {
        res = true;
    }
    else
    {
        res = false;
    }
    delayMicroseconds(10);

    return res;
}
//
void ZumoU4::waitButtonAPressed()
{
    do
    {
        while (!isButtonAPressed())
        {
            // nop
        }
        delay(20);
    } while (!isButtonAPressed());
}
//
void ZumoU4::waitButtonAReleased()
{
    do
    {
        while (isButtonAPressed())
        {
            // nop
        }
        delay(20);
    } while (isButtonAPressed());
}
//
void ZumoU4::waitButtonAPressedAndReleased()
{
    waitButtonAPressed();
    waitButtonAReleased();
}
