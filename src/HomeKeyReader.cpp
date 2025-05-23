// HomeKeyReader.cpp
#include "HomeKeyReader.h"
#include <mbedtls/sha256.h>
#include <mbedtls/ecdh.h>
#include <mbedtls/entropy.h>
#include <mbedtls/ctr_drbg.h>
#include <nvs.h>
#include <esp_log.h>

#define TAG "HomeKeyReader"

// HomeKey Applet AID
const uint8_t HOMEKEY_AID[] = {0xA0, 0x00, 0x00, 0x08, 0x58, 0x01, 0x01};

HomeKeyReader::HomeKeyReader(uint8_t clk, uint8_t miso, uint8_t mosi, uint8_t ss)
{
    pn532spi = new PN532_SPI(clk, miso, mosi, ss);
    nfc = new PN532(*pn532spi);
    currentState = AuthState::IDLE;
}

HomeKeyReader::~HomeKeyReader()
{
    delete nfc;
    delete pn532spi;
}

bool HomeKeyReader::begin()
{
    ESP_LOGI(TAG, "Initializing HomeKey Reader");

    nfc->begin();

    uint32_t versiondata = nfc->getFirmwareVersion();
    if (!versiondata)
    {
        ESP_LOGE(TAG, "PN532 not found!");
        return false;
    }

    ESP_LOGI(TAG, "Found PN532 chip, firmware version: %d.%d",
             (versiondata >> 16) & 0xFF, (versiondata >> 8) & 0xFF);

    // Configure the PN532 for passive target detection
    nfc->setPassiveActivationRetries(0xFF);
    nfc->SAMConfig();

    // Load stored HomeKey data
    if (!loadFromNVS())
    {
        ESP_LOGW(TAG, "No stored HomeKey data found");
        // Generate new reader keys if none exist
        generateEphemeralKeyPair(); // This would generate permanent keys in production
    }

    return true;
}

void HomeKeyReader::poll()
{
    uint8_t uid[7];
    uint8_t uidLength;

    // Check for NFC cards
    if (nfc->readPassiveTargetID(PN532_MIFARE_ISO14443A, uid, &uidLength, 100))
    {
        ESP_LOGI(TAG, "NFC card detected, UID length: %d", uidLength);

        // Try to select HomeKey applet
        if (selectHomeKeyApplet())
        {
            ESP_LOGI(TAG, "HomeKey applet found, starting authentication");
            currentState = AuthState::SELECT_APPLET;

            // Try fast authentication first
            if (performFastAuth())
            {
                currentState = AuthState::SUCCESS;
                ESP_LOGI(TAG, "Fast authentication successful!");
            }
            else
            {
                // Fall back to standard authentication
                ESP_LOGI(TAG, "Fast auth failed, trying standard auth");
                if (performStandardAuth())
                {
                    currentState = AuthState::SUCCESS;
                    ESP_LOGI(TAG, "Standard authentication successful!");
                }
                else
                {
                    currentState = AuthState::FAILED;
                    ESP_LOGE(TAG, "Authentication failed");
                }
            }
        }

        // Reset state for next card
        currentState = AuthState::IDLE;

        // Wait for card to be removed
        while (nfc->readPassiveTargetID(PN532_MIFARE_ISO14443A, uid, &uidLength, 50))
        {
            delay(100);
        }
    }
}

bool HomeKeyReader::selectHomeKeyApplet()
{
    uint8_t apdu[13] = {
        0x00, // CLA
        0xA4, // INS - SELECT
        0x04, // P1 - Select by AID
        0x00, // P2
        0x07, // LC - Length of AID
    };
    memcpy(apdu + 5, HOMEKEY_AID, 7);
    apdu[12] = 0x00; // LE

    uint8_t response[32];
    uint8_t responseLen = sizeof(response);

    if (sendAPDU(apdu, sizeof(apdu), response, &responseLen))
    {
        // Check for 90 00 success code
        if (responseLen >= 2 && response[responseLen - 2] == 0x90 && response[responseLen - 1] == 0x00)
        {
            ESP_LOGI(TAG, "HomeKey applet selected successfully");

            // Parse version info from response
            if (responseLen > 2)
            {
                ESP_LOGI(TAG, "HomeKey version: %d.%d", response[0], response[1]);
            }
            return true;
        }
    }

    return false;
}

bool HomeKeyReader::performFastAuth()
{
    // Fast authentication flow
    // This is a simplified version - real implementation needs full crypto

    ESP_LOGI(TAG, "Attempting fast authentication");

    // Step 1: Send reader identifier
    uint8_t cmdAuth[64];
    cmdAuth[0] = 0x80; // CLA
    cmdAuth[1] = 0x80; // INS - AUTH0
    cmdAuth[2] = 0x00; // P1
    cmdAuth[3] = 0x00; // P2
    cmdAuth[4] = 0x1C; // LC

    // Add reader identifier (simplified - should be encrypted)
    memcpy(cmdAuth + 5, homeKeyData.readerIdentifier.data(), 16);

    // Add transaction identifier
    for (int i = 0; i < 8; i++)
    {
        currentTransactionId[i] = random(256);
    }
    memcpy(cmdAuth + 21, currentTransactionId, 8);

    cmdAuth[33] = 0x00; // LE

    uint8_t response[128];
    uint8_t responseLen = sizeof(response);

    if (!sendAPDU(cmdAuth, 34, response, &responseLen))
    {
        return false;
    }

    // Check response and extract endpoint info
    if (responseLen >= 16)
    {
        // In a real implementation, verify cryptographic signatures here

        // Extract issuer and endpoint IDs
        uint8_t issuerId[8];
        uint8_t endpointId[6];

        memcpy(issuerId, response, 8);
        memcpy(endpointId, response + 8, 6);

        // Verify against stored issuers
        for (const auto &issuer : homeKeyData.issuers)
        {
            if (memcmp(issuer.first.data(), issuerId, 8) == 0)
            {
                ESP_LOGI(TAG, "Issuer verified!");

                // Call success callback
                if (authCallback)
                {
                    authCallback(issuerId, endpointId);
                }
                return true;
            }
        }
    }

    return false;
}

bool HomeKeyReader::performStandardAuth()
{
    // Standard authentication flow with full ECDH key exchange
    ESP_LOGI(TAG, "Performing standard authentication");

    // Generate ephemeral key pair for this transaction
    generateEphemeralKeyPair();

    // Step 1: Send AUTH0 command with ephemeral public key
    uint8_t cmdAuth0[128];
    cmdAuth0[0] = 0x80; // CLA
    cmdAuth0[1] = 0x80; // INS - AUTH0
    cmdAuth0[2] = 0x00; // P1
    cmdAuth0[3] = 0x00; // P2
    cmdAuth0[4] = 0x51; // LC - 65 bytes public key + 16 bytes reader ID

    // Add ephemeral public key
    memcpy(cmdAuth0 + 5, ephemeralPublicKey, 65);

    // Add reader identifier
    memcpy(cmdAuth0 + 70, homeKeyData.readerIdentifier.data(), 16);

    cmdAuth0[86] = 0x00; // LE

    uint8_t response[256];
    uint8_t responseLen = sizeof(response);

    if (!sendAPDU(cmdAuth0, 87, response, &responseLen))
    {
        return false;
    }

    // Step 2: Process response and complete authentication
    if (responseLen >= 65)
    {
        // Extract device ephemeral public key and encrypted data
        uint8_t devicePublicKey[65];
        memcpy(devicePublicKey, response, 65);

        // Perform ECDH to derive shared secret
        // In production, use proper crypto library for this

        // Decrypt and verify the response
        // Extract issuer and endpoint information

        // For demo purposes, extract IDs from known positions
        uint8_t issuerId[8] = {0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08};
        uint8_t endpointId[6] = {0x0A, 0x0B, 0x0C, 0x0D, 0x0E, 0x0F};

        if (authCallback)
        {
            authCallback(issuerId, endpointId);
        }

        return true;
    }

    return false;
}

bool HomeKeyReader::sendAPDU(uint8_t *cmd, uint8_t cmdLen, uint8_t *response, uint8_t *responseLen)
{
    // Log APDU command
    ESP_LOGD(TAG, "Sending APDU:");
    ESP_LOG_BUFFER_HEX_LEVEL(TAG, cmd, cmdLen, ESP_LOG_DEBUG);

    bool success = nfc->inDataExchange(cmd, cmdLen, response, responseLen);

    if (success)
    {
        ESP_LOGD(TAG, "Received response:");
        ESP_LOG_BUFFER_HEX_LEVEL(TAG, response, *responseLen, ESP_LOG_DEBUG);
    }
    else
    {
        ESP_LOGE(TAG, "APDU communication failed");
    }

    return success;
}

void HomeKeyReader::generateEphemeralKeyPair()
{
    // In production, use proper ECDH key generation
    // This is a placeholder
    for (int i = 0; i < 32; i++)
    {
        ephemeralPrivateKey[i] = random(256);
    }

    // Generate public key from private key
    ephemeralPublicKey[0] = 0x04; // Uncompressed point
    for (int i = 1; i < 65; i++)
    {
        ephemeralPublicKey[i] = random(256);
    }
}

bool HomeKeyReader::processProvisioningData(uint8_t *data, size_t len)
{
    ESP_LOGI(TAG, "Processing provisioning data, length: %d", len);

    // Parse TLV8 encoded provisioning data
    // This would contain reader keys and issuer information

    // For demo, create some dummy data
    homeKeyData.readerIdentifier.resize(16);
    homeKeyData.readerGroupIdentifier.resize(8);
    for (int i = 0; i < 16; i++)
    {
        homeKeyData.readerIdentifier[i] = random(256);
    }
    for (int i = 0; i < 8; i++)
    {
        homeKeyData.readerGroupIdentifier[i] = random(256);
    }

    // Save to NVS
    return saveToNVS();
}

bool HomeKeyReader::saveToNVS()
{
    nvs_handle_t handle;
    esp_err_t err = nvs_open("homekey", NVS_READWRITE, &handle);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "Error opening NVS: %s", esp_err_to_name(err));
        return false;
    }

    // Save reader data
    err = nvs_set_blob(handle, "reader_id", homeKeyData.readerIdentifier.data(),
                       homeKeyData.readerIdentifier.size());

    nvs_commit(handle);
    nvs_close(handle);

    return err == ESP_OK;
}

bool HomeKeyReader::loadFromNVS()
{
    nvs_handle_t handle;
    esp_err_t err = nvs_open("homekey", NVS_READONLY, &handle);
    if (err != ESP_OK)
    {
        return false;
    }

    // Load reader data
    size_t length = 16;
    homeKeyData.readerIdentifier.resize(length);
    err = nvs_get_blob(handle, "reader_id", homeKeyData.readerIdentifier.data(), &length);

    nvs_close(handle);

    return err == ESP_OK;
}

void HomeKeyReader::setAuthenticationCallback(AuthCallback callback)
{
    authCallback = callback;
}