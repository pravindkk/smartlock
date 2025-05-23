// main.cpp - Complete Smart Lock Implementation with HomeKey
#include <Arduino.h>
#include <WiFi.h>
#include <HomeSpan.h>
#include <nvs_flash.h>
#include <esp_sleep.h>
#include "HomeKeyReader.h"
#include "FingerprintManager.h"

// Pin Definitions for ESP32 Dev Module
#define PN532_SS 5    // VSPI SS
#define PN532_SCK 18  // VSPI SCK
#define PN532_MISO 19 // VSPI MISO
#define PN532_MOSI 23 // VSPI MOSI

#define FP1_TX 16 // UART2 TX
#define FP1_RX 17 // UART2 RX
#define FP2_TX 25 // Software Serial
#define FP2_RX 26 // Software Serial

#define MOTOR_IN1 32
#define MOTOR_IN2 33
#define MOTOR_EN 27

#define STATUS_LED 2     // Built-in LED on most ESP32 dev boards
#define CONTROL_BUTTON 0 // BOOT button

#define BATTERY_ADC 34      // ADC pin for battery monitoring
#define PERIPHERAL_POWER 14 // Control power to peripherals

// Power Management Settings
#define SLEEP_TIMEOUT_MS 30000 // Stay awake for 30 seconds after activity
#define SLEEP_DURATION_MIN 30  // Wake every 30 minutes for HomeKit ping

// Global Objects
HomeKeyReader *homeKeyReader = nullptr;
FingerprintManager *fingerprintMgr = nullptr;
RTC_DATA_ATTR int bootCount = 0; // Persists across deep sleep

// Forward declaration
struct SmartLock;

// HomeKey NFC Service Implementation
struct NFCAccess : Service::NFCAccess
{
  SpanCharacteristic *configurationState;
  SpanCharacteristic *nfcControlPoint;
  SpanCharacteristic *nfcSupportedConfiguration;
  HomeKeyReader *reader;

  NFCAccess(HomeKeyReader *hkReader) : Service::NFCAccess()
  {
    LOG1("Configuring NFCAccess Service\n");
    reader = hkReader;

    // Configuration state indicates if NFC is provisioned
    configurationState = new Characteristic::ConfigurationState(0);

    // Control point for HomeKey operations
    nfcControlPoint = new Characteristic::NFCAccessControlPoint();

    // Supported configuration TLV8
    uint8_t config[] = {
        0x01, 0x01, 0x00, // Version 1.0
        0x02, 0x01, 0x00  // Supports all features
    };
    nfcSupportedConfiguration = new Characteristic::NFCAccessSupportedConfiguration(config, sizeof(config));
  }

  boolean update()
  {
    // Handle HomeKey provisioning commands
    if (nfcControlPoint->updated())
    {
      size_t len = nfcControlPoint->getNewDataLen();
      uint8_t *data = (uint8_t *)malloc(len);
      nfcControlPoint->getNewData(data);

      LOG1("NFC Control Point received %d bytes\n", len);

      // Process HomeKey provisioning data
      if (reader->processProvisioningData(data, len))
      {
        configurationState->setVal(1); // Mark as configured
        LOG1("HomeKey provisioning successful\n");
      }

      free(data);
    }
    return true;
  }
};

// Main Smart Lock Service
struct SmartLock : Service::LockManagement
{
  SpanCharacteristic *lockCurrentState;
  SpanCharacteristic *lockTargetState;
  SpanCharacteristic *lockControlPoint;
  SpanCharacteristic *version;

  // Battery service characteristics
  SpanCharacteristic *batteryLevel;
  SpanCharacteristic *batteryLow;
  SpanCharacteristic *chargingState;

  // Lock state
  bool isLocked = true;
  unsigned long lastActivityTime = 0;
  unsigned long lastBatteryCheck = 0;

  SmartLock() : Service::LockManagement()
  {
    LOG1("Creating Smart Lock Service\n");

    // Lock Management characteristics
    lockCurrentState = new Characteristic::LockCurrentState(1); // Initially locked
    lockTargetState = new Characteristic::LockTargetState(1);
    lockControlPoint = new Characteristic::LockControlPoint();
    version = new Characteristic::Version(); // Required for HomeKey

    // Initialize hardware
    initializeHardware();

    lastActivityTime = millis();
  }

  void initializeHardware()
  {
    // Configure motor control pins
    pinMode(MOTOR_IN1, OUTPUT);
    pinMode(MOTOR_IN2, OUTPUT);
    pinMode(MOTOR_EN, OUTPUT);
    ledcSetup(0, 5000, 8); // Channel 0, 5kHz, 8-bit resolution
    ledcAttachPin(MOTOR_EN, 0);

    // Configure peripheral power control
    pinMode(PERIPHERAL_POWER, OUTPUT);
    digitalWrite(PERIPHERAL_POWER, HIGH); // Enable peripherals

    // Configure battery monitoring
    pinMode(BATTERY_ADC, INPUT);

    // Initialize motor to locked position
    ensureLocked();
  }

  boolean update()
  {
    if (lockTargetState->updated())
    {
      uint8_t targetState = lockTargetState->getNewVal();

      LOG1("Lock target state changed to: %s\n",
           targetState == 1 ? "LOCKED" : "UNLOCKED");

      // Update current state to show we're working on it
      lockCurrentState->setVal(targetState == 1 ? 3 : 2); // LOCKING : UNLOCKING

      if (targetState == 1)
      {
        performLock();
      }
      else
      {
        performUnlock();
      }

      // Update current state to match target
      lockCurrentState->setVal(targetState);
      lastActivityTime = millis();
    }

    return true;
  }

  void performLock()
  {
    LOG1("Locking deadbolt...\n");
    digitalWrite(MOTOR_IN1, HIGH);
    digitalWrite(MOTOR_IN2, LOW);
    ledcWrite(0, 255); // Full power

    delay(2000); // Time for deadbolt to extend

    ledcWrite(0, 0); // Stop motor
    digitalWrite(MOTOR_IN1, LOW);

    isLocked = true;
  }

  void performUnlock()
  {
    LOG1("Unlocking deadbolt...\n");
    digitalWrite(MOTOR_IN1, LOW);
    digitalWrite(MOTOR_IN2, HIGH);
    ledcWrite(0, 255); // Full power

    delay(2000); // Time for deadbolt to retract

    ledcWrite(0, 0); // Stop motor
    digitalWrite(MOTOR_IN2, LOW);

    isLocked = false;

    // Schedule auto-relock after 10 seconds
    xTaskCreate([](void *param)
                {
            SmartLock* lock = (SmartLock*)param;
            vTaskDelay(10000 / portTICK_PERIOD_MS);
            if (!lock->isLocked) {
                lock->performLock();
                lock->lockTargetState->setVal(1);
                lock->lockCurrentState->setVal(1);
            }
            vTaskDelete(NULL); }, "AutoRelock", 2048, this, 1, NULL);
  }

  void ensureLocked()
  {
    // Always lock on startup for security
    performLock();
  }

  void checkActivity()
  {
    // Monitor battery periodically
    if (millis() - lastBatteryCheck > 60000)
    { // Every minute
      updateBatteryStatus();
      lastBatteryCheck = millis();
    }

    // Check if we should enter deep sleep
    if (millis() - lastActivityTime > SLEEP_TIMEOUT_MS)
    {
      prepareForSleep();
    }
  }

  void updateBatteryStatus()
  {
    // Read battery voltage (assuming voltage divider 2:1)
    int adcValue = analogRead(BATTERY_ADC);
    float voltage = (adcValue / 4095.0) * 3.3 * 2;

    // Map voltage to percentage (6V = 0%, 9V = 100%)
    int percentage = constrain(map(voltage * 100, 600, 900, 0, 100), 0, 100);

    if (batteryLevel)
    {
      batteryLevel->setVal(percentage);
    }
    if (batteryLow)
    {
      batteryLow->setVal(percentage < 20 ? 1 : 0);
    }

    LOG2("Battery: %.2fV (%d%%)\n", voltage, percentage);
  }

  void prepareForSleep()
  {
    LOG1("Preparing for deep sleep...\n");

    // Ensure lock is secured
    if (!isLocked)
    {
      performLock();
      lockTargetState->setVal(1);
      lockCurrentState->setVal(1);
    }

    // Power down peripherals
    digitalWrite(PERIPHERAL_POWER, LOW);
    delay(100);

    // Configure wake sources
    esp_sleep_enable_timer_wakeup(SLEEP_DURATION_MIN * 60 * 1000000ULL);
    esp_sleep_enable_ext0_wakeup(GPIO_NUM_0, 0); // Wake on button press

    // Save state and enter deep sleep
    LOG1("Entering deep sleep for %d minutes\n", SLEEP_DURATION_MIN);
    esp_deep_sleep_start();
  }

  void resetActivity()
  {
    lastActivityTime = millis();
  }
};

// Global lock instance
SmartLock *smartLock = nullptr;

// Callback for HomeKey authentication success
void onHomeKeyAuthenticated(const uint8_t *issuerId, const uint8_t *endpointId)
{
  LOG1("HomeKey authenticated!\n");
  LOG1("Issuer: ");
  for (int i = 0; i < 8; i++)
    LOG1("%02X", issuerId[i]);
  LOG1("\nEndpoint: ");
  for (int i = 0; i < 6; i++)
    LOG1("%02X", endpointId[i]);
  LOG1("\n");

  // Unlock the door
  smartLock->lockTargetState->setVal(0); // Unlock
  smartLock->update();
}

// Callback for fingerprint match
void onFingerprintMatched(uint16_t id, uint8_t confidence, uint8_t sensor)
{
  LOG1("Fingerprint matched! ID: %d, Confidence: %d, Sensor: %d\n",
       id, confidence, sensor);

  // Unlock the door
  smartLock->lockTargetState->setVal(0); // Unlock
  smartLock->update();
}

void setup()
{
  Serial.begin(115200);

  // Initialize NVS for persistent storage
  nvs_flash_init();

  LOG1("\n\nSmart Lock with HomeKey Starting...\n");
  LOG1("Boot count: %d\n", ++bootCount);

  // Check wake reason
  esp_sleep_wakeup_cause_t wakeup_reason = esp_sleep_get_wakeup_cause();
  switch (wakeup_reason)
  {
  case ESP_SLEEP_WAKEUP_EXT0:
    LOG1("Wakeup from button press\n");
    break;
  case ESP_SLEEP_WAKEUP_TIMER:
    LOG1("Wakeup from timer\n");
    break;
  default:
    LOG1("Normal power on\n");
    break;
  }

  // Initialize HomeKey reader
  homeKeyReader = new HomeKeyReader(PN532_SCK, PN532_MISO, PN532_MOSI, PN532_SS);
  homeKeyReader->begin();
  homeKeyReader->setAuthenticationCallback(onHomeKeyAuthenticated);

  // Initialize fingerprint manager
  fingerprintMgr = new FingerprintManager(FP1_TX, FP1_RX, FP2_TX, FP2_RX);
  fingerprintMgr->begin();
  fingerprintMgr->setMatchCallback(onFingerprintMatched);

  // Configure HomeSpan
  homeSpan.setHostNameSuffix("");
  homeSpan.setWifiCredentials("YOUR_WIFI_SSID", "YOUR_WIFI_PASSWORD");
  homeSpan.setApSSID("SmartLock-Setup");
  homeSpan.setApPassword("setup123");
  homeSpan.setApTimeout(300); // 5 minute timeout for setup mode
  homeSpan.setControlPin(CONTROL_BUTTON);
  homeSpan.setStatusPin(STATUS_LED);
  homeSpan.setLogLevel(1);
  homeSpan.setPortNum(1201); // Required for HomeKey

  // Start HomeSpan
  homeSpan.begin(Category::Locks, "Smart Lock HomeKey", "SLH", "SmartLock-HAP");

  // Create main accessory
  new SpanAccessory();
  new Service::AccessoryInformation();
  new Characteristic::Identify();
  new Characteristic::Manufacturer("DIY Smart Home");
  new Characteristic::Model("SmartLock-v1");
  new Characteristic::SerialNumber("SL-001");
  new Characteristic::FirmwareRevision("1.0.0");
  new Characteristic::HardwareFinish(); // Required for HomeKey

  // Lock Management Service
  smartLock = new SmartLock();

  // NFC Access Service for HomeKey
  new NFCAccess(homeKeyReader);

  // Battery Service
  new Service::BatteryService();
  smartLock->batteryLevel = new Characteristic::BatteryLevel(100);
  smartLock->batteryLow = new Characteristic::StatusLowBattery(0);
  smartLock->chargingState = new Characteristic::ChargingState(0); // Not charging

  // Set up CLI commands for fingerprint management
  new SpanUserCommand('e', "Enroll fingerprint - e[id]", [](const char *arg)
                      {
        int id = atoi(arg + 1);
        if (id >= 1 && id <= 127) {
            LOG1("Starting fingerprint enrollment for ID %d\n", id);
            LOG1("Please place finger on sensor 1...\n");
            
            if (fingerprintMgr->enrollFingerprint(id, 1)) {
                LOG1("Enrollment successful!\n");
            } else {
                LOG1("Enrollment failed!\n");
            }
        } });

  new SpanUserCommand('d', "Delete fingerprint - d[id]", [](const char *arg)
                      {
        int id = atoi(arg + 1);
        if (id >= 1 && id <= 127) {
            if (fingerprintMgr->deleteFingerprint(id)) {
                LOG1("Fingerprint ID %d deleted\n", id);
            }
        } });

  new SpanUserCommand('l', "List stored fingerprints", [](const char *arg)
                      { fingerprintMgr->listFingerprints(); });

  new SpanUserCommand('x', "Delete all fingerprints", [](const char *arg)
                      {
        fingerprintMgr->deleteAllFingerprints();
        LOG1("All fingerprints deleted\n"); });

  LOG1("Smart Lock ready!\n");
}

void loop()
{
  // Let HomeSpan handle its tasks
  homeSpan.poll();

  // Check for NFC cards (HomeKey)
  if (homeKeyReader)
  {
    homeKeyReader->poll();
  }

  // Check fingerprint sensors
  if (fingerprintMgr)
  {
    fingerprintMgr->poll();
  }

  // Check if we should sleep
  if (smartLock)
  {
    smartLock->checkActivity();
  }

  // Small delay to prevent watchdog issues
  delay(10);
}