// FingerprintManager.h
#ifndef FINGERPRINT_MANAGER_H
#define FINGERPRINT_MANAGER_H

#include <Arduino.h>
#include <Adafruit_Fingerprint.h>
#include <functional>

typedef std::function<void(uint16_t id, uint8_t confidence, uint8_t sensor)> FingerprintCallback;

class FingerprintManager
{
private:
    HardwareSerial *serial1;
    HardwareSerial *serial2;
    Adafruit_Fingerprint *finger1;
    Adafruit_Fingerprint *finger2;
    FingerprintCallback matchCallback;

    bool sensor1Active;
    bool sensor2Active;

public:
    FingerprintManager(uint8_t tx1, uint8_t rx1, uint8_t tx2, uint8_t rx2);
    ~FingerprintManager();

    bool begin();
    void poll();

    // Enrollment functions
    bool enrollFingerprint(uint16_t id, uint8_t sensor = 1);
    bool deleteFingerprint(uint16_t id, uint8_t sensor = 1);
    void deleteAllFingerprints();
    void listFingerprints();

    // Configuration
    void setMatchCallback(FingerprintCallback callback);
    void setSecurityLevel(uint8_t level);

private:
    bool checkSensor(Adafruit_Fingerprint *sensor, uint8_t sensorNum);
    bool enrollOnSensor(Adafruit_Fingerprint *sensor, uint16_t id);
};

#endif