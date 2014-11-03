/* DHT library 

   MIT license
   written by Adafruit Industries
   */

#include "DHT.h"

DHT::DHT(uint8_t pin, uint8_t type, uint8_t count) {
    _pin = pin;
    _type = type;
    _count = count;
    firstreading = true;
}

void DHT::begin(void) {
    // set up the pins!
    pinMode(_pin, INPUT);
    digitalWrite(_pin, HIGH);
    _lastreadtime = 0;

    pinMode(8, OUTPUT);
    digitalWrite(8, LOW);
}

boolean DHT::doRead(void)
{
    return read();
}

//boolean S == Scale.  True == Farenheit; False == Celcius
float DHT::readTemperature(bool S) {
    float f;

    switch (_type) {
        case DHT11:
            f = data[2];
            if(S)
                f = convertCtoF(f);

            return f;
        case DHT22:
        case DHT21:
            f = data[2] & 0x7F;
            f *= 256;
            f += data[3];
            f /= 10;
            if (data[2] & 0x80)
                f *= -1;
            if(S)
                f = convertCtoF(f);

            return f;
    }
    return NAN;
}

float DHT::convertCtoF(float c) {
    return c * 9 / 5 + 32;
}

float DHT::readHumidity(void) {
    float f;

    switch (_type) {
        case DHT11:
            f = data[0];
            return f;
        case DHT22:
        case DHT21:
            f = data[0];
            f *= 256;
            f += data[1];
            f /= 10;
            return f;
    }
    return NAN;
}

/*
void DHT::toggleSignal(void) {
    if (digitalRead(8) == LOW) {
        digitalWrite(8, HIGH);
    } else {
        digitalWrite(8, LOW);
    }
}
*/

int DHT::readSignal(void) {
    //toggleSignal();
    return digitalRead(_pin);
}

boolean DHT::read(void) {
    uint8_t laststate = LOW;
    uint16_t counter = 0;
    uint8_t j = 0, i;
    unsigned long currenttime;

    // pull the pin high and wait 250 milliseconds
    digitalWrite(_pin, HIGH);
    delay(250);

    /*
    currenttime = millis();
    if (currenttime < _lastreadtime) {
        // ie there was a rollover
        _lastreadtime = 0;
    }
    if (!firstreading && ((currenttime - _lastreadtime) < 2000)) {
        return true; // return last correct measurement
        //delay(2000 - (currenttime - _lastreadtime));
    }
    firstreading = false;
       Serial.print("Currtime: "); Serial.print(currenttime);
       Serial.print(" Lasttime: "); Serial.print(_lastreadtime);
    _lastreadtime = millis();
    */

    data[0] = data[1] = data[2] = data[3] = data[4] = 0;

    // now pull it low for ~20 milliseconds
    pinMode(_pin, OUTPUT);
    digitalWrite(_pin, LOW);
    delay(20);
    noInterrupts();
    digitalWrite(_pin, HIGH);
    delayMicroseconds(160);
    pinMode(_pin, INPUT);
    //toggleSignal();

    while (readSignal() != LOW);

    // read in timings
    for ( i=0; i< TOTALBYTES; i++) {
        for (j = 0; j < 8; j++) {
            counter = 0;
            while (readSignal() == laststate) {
                counter++;
                delayMicroseconds(1);
                if (counter == 30000) {
                    break;
                }
            }
            laststate = readSignal();

            if (counter == 30000) break;

            counter = 0;
            while (readSignal() == laststate) {
                counter++;
                delayMicroseconds(1);
                if (counter == 30000) {
                    break;
                }
            }
            laststate = readSignal();

            if (counter == 30000) break;

            if (counter > 20) {
                data[i] |= (1 << (7 - j));
                //toggleSignal();
            }
            //toggleSignal();
        }
    }

    interrupts();

    // check we read 40 bits and that the checksum matches
    if (data[4] == ((data[0] + data[1] + data[2] + data[3]) & 0xFF)) {
        return true;
    }
    //toggleSignal();
    //toggleSignal();
    return false;

}
