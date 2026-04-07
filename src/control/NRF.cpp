#include "NRF.h"

NRFReceiver::NRFReceiver(uint8_t cePin, uint8_t csnPin, const byte address[6])
    : radio(cePin, csnPin)
{
    memcpy(pipeAddress, address, 6);
}

void NRFReceiver::begin() {
    radio.begin();
    radio.openReadingPipe(0, pipeAddress);
    radio.setPALevel(RF24_PA_MAX);
    radio.startListening();

    // Failsafe defaults
    data.throttle = 1000;
    data.rollTarget = 0;
    data.pitchTarget = 0;
    data.yawTarget = 0;
}

bool NRFReceiver::available() {
    return radio.available();
}

void NRFReceiver::read() {
    if (radio.available()) {
        radio.read(&data, sizeof(RadioPacket));
        lastReceiveTime = millis();
    }
}