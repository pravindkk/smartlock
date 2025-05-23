// FingerprintManager.cpp
#include "FingerprintManager.h"
#include <esp_log.h>

#define TAG "FingerprintManager"

FingerprintManager::FingerprintManager(uint8_t tx1, uint8_t rx1, uint8_t tx2, uint8_t rx2)
{
    serial1 = new HardwareSerial(1);
    serial2 = new HardwareSerial(2);

    serial1->begin(57600, SERIAL_8N1, rx1, tx1);
    serial2->begin(57600, SERIAL_8N1, rx2, tx2);

    finger1 = new Adafruit_Fingerprint(serial1);
    finger2 = new Adafruit_Fingerprint(serial2);

    sensor1Active = false;
    sensor2Active = false;
}

FingerprintManager::~FingerprintManager()
{
    delete finger1;
    delete finger2;
    delete serial1;
    delete serial2;
}

bool FingerprintManager::begin()
{
    ESP_LOGI(TAG, "Initializing fingerprint sensors");

    // Initialize sensor 1
    if (finger1->verifyPassword())
    {
        ESP_LOGI(TAG, "Fingerprint sensor 1 found!");
        sensor1Active = true;

        // Get sensor parameters
        finger1->getParameters();
        ESP_LOGI(TAG, "Sensor 1 - Capacity: %d, Packet size: %d",
                 finger1->capacity, finger1->packet_len);
    }
    else
    {
        ESP_LOGW(TAG, "Fingerprint sensor 1 not found");
    }

    // Initialize sensor 2
    if (finger2->verifyPassword())
    {
        ESP_LOGI(TAG, "Fingerprint sensor 2 found!");
        sensor2Active = true;

        // Get sensor parameters
        finger2->getParameters();
        ESP_LOGI(TAG, "Sensor 2 - Capacity: %d, Packet size: %d",
                 finger2->capacity, finger2->packet_len);
    }
    else
    {
        ESP_LOGW(TAG, "Fingerprint sensor 2 not found");
    }

    return sensor1Active || sensor2Active;
}

void FingerprintManager::poll()
{
    // Check sensor 1
    if (sensor1Active)
    {
        checkSensor(finger1, 1);
    }

    // Check sensor 2
    if (sensor2Active)
    {
        checkSensor(finger2, 2);
    }
}

bool FingerprintManager::checkSensor(Adafruit_Fingerprint *sensor, uint8_t sensorNum)
{
    uint8_t result = sensor->getImage();

    if (result == FINGERPRINT_OK)
    {
        // Image captured, convert it
        result = sensor->image2Tz();
        if (result != FINGERPRINT_OK)
        {
            return false;
        }

        // Search for match
        result = sensor->fingerSearch();
        if (result == FINGERPRINT_OK)
        {
            ESP_LOGI(TAG, "Fingerprint match found on sensor %d!", sensorNum);
            ESP_LOGI(TAG, "ID: %d, Confidence: %d", sensor->fingerID, sensor->confidence);

            // Call callback if set
            if (matchCallback)
            {
                matchCallback(sensor->fingerID, sensor->confidence, sensorNum);
            }

            // Wait for finger to be removed
            while (sensor->getImage() != FINGERPRINT_NOFINGER)
            {
                delay(50);
            }

            return true;
        }
        else if (result == FINGERPRINT_NOTFOUND)
        {
            ESP_LOGW(TAG, "No match found on sensor %d", sensorNum);
        }
    }

    return false;
}

bool FingerprintManager::enrollFingerprint(uint16_t id, uint8_t sensorNum)
{
    Adafruit_Fingerprint *sensor = (sensorNum == 1) ? finger1 : finger2;

    if ((sensorNum == 1 && !sensor1Active) || (sensorNum == 2 && !sensor2Active))
    {
        ESP_LOGE(TAG, "Sensor %d is not active", sensorNum);
        return false;
    }

    return enrollOnSensor(sensor, id);
}

bool FingerprintManager::enrollOnSensor(Adafruit_Fingerprint *sensor, uint16_t id)
{
    ESP_LOGI(TAG, "Enrolling fingerprint ID %d", id);
    ESP_LOGI(TAG, "Waiting for finger...");

    // Wait for first image
    while (sensor->getImage() != FINGERPRINT_OK)
    {
        delay(50);
    }

    // Convert first image
    if (sensor->image2Tz(1) != FINGERPRINT_OK)
    {
        ESP_LOGE(TAG, "Failed to convert first image");
        return false;
    }

    ESP_LOGI(TAG, "Remove finger");
    delay(1000);

    // Wait for finger to be removed
    while (sensor->getImage() != FINGERPRINT_NOFINGER)
    {
        delay(50);
    }

    ESP_LOGI(TAG, "Place same finger again");

    // Wait for second image
    while (sensor->getImage() != FINGERPRINT_OK)
    {
        delay(50);
    }

    // Convert second image
    if (sensor->image2Tz(2) != FINGERPRINT_OK)
    {
        ESP_LOGE(TAG, "Failed to convert second image");
        return false;
    }

    // Create model
    ESP_LOGI(TAG, "Creating model...");
    if (sensor->createModel() != FINGERPRINT_OK)
    {
        ESP_LOGE(TAG, "Failed to create model");
        return false;
    }

    // Store model
    ESP_LOGI(TAG, "Storing model...");
    if (sensor->storeModel(id) != FINGERPRINT_OK)
    {
        ESP_LOGE(TAG, "Failed to store model");
        return false;
    }

    ESP_LOGI(TAG, "Fingerprint enrolled successfully!");
    return true;
}

bool FingerprintManager::deleteFingerprint(uint16_t id, uint8_t sensor)
{
    bool success = false;

    if (sensor == 1 || sensor == 0)
    {
        if (sensor1Active && finger1->deleteModel(id) == FINGERPRINT_OK)
        {
            ESP_LOGI(TAG, "Deleted fingerprint ID %d from sensor 1", id);
            success = true;
        }
    }

    if (sensor == 2 || sensor == 0)
    {
        if (sensor2Active && finger2->deleteModel(id) == FINGERPRINT_OK)
        {
            ESP_LOGI(TAG, "Deleted fingerprint ID %d from sensor 2", id);
            success = true;
        }
    }

    return success;
}

void FingerprintManager::deleteAllFingerprints()
{
    if (sensor1Active)
    {
        finger1->emptyDatabase();
        ESP_LOGI(TAG, "Cleared all fingerprints from sensor 1");
    }

    if (sensor2Active)
    {
        finger2->emptyDatabase();
        ESP_LOGI(TAG, "Cleared all fingerprints from sensor 2");
    }
}

void FingerprintManager::listFingerprints()
{
    ESP_LOGI(TAG, "Stored fingerprints:");

    if (sensor1Active)
    {
        ESP_LOGI(TAG, "Sensor 1:");
        for (uint16_t i = 1; i <= finger1->capacity; i++)
        {
            uint8_t result = finger1->loadModel(i);
            if (result == FINGERPRINT_OK)
            {
                ESP_LOGI(TAG, "  ID %d: Stored", i);
            }
        }
    }

    if (sensor2Active)
    {
        ESP_LOGI(TAG, "Sensor 2:");
        for (uint16_t i = 1; i <= finger2->capacity; i++)
        {
            uint8_t result = finger2->loadModel(i);
            if (result == FINGERPRINT_OK)
            {
                ESP_LOGI(TAG, "  ID %d: Stored", i);
            }
        }
    }
}

void FingerprintManager::setMatchCallback(FingerprintCallback callback)
{
    matchCallback = callback;
}

void FingerprintManager::setSecurityLevel(uint8_t level)
{
    if (sensor1Active)
    {
        finger1->setSecurityLevel(level);
    }
    if (sensor2Active)
    {
        finger2->setSecurityLevel(level);
    }
}