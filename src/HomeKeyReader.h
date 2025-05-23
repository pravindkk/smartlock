// HomeKeyReader.h
#ifndef HOMEKEY_READER_H
#define HOMEKEY_READER_H

#include <Arduino.h>
#include <SPI.h>
#include <PN532_SPI.h>
#include <PN532.h>
#include <vector>
#include <functional>

// HomeKey Authentication Flow States
enum class AuthState
{
    IDLE,
    SELECT_APPLET,
    FAST_AUTH,
    STANDARD_AUTH,
    ATTESTATION,
    SUCCESS,
    FAILED
};

// Callback type for authentication success
typedef std::function<void(const uint8_t *issuerId, const uint8_t *endpointId)> AuthCallback;

class HomeKeyReader
{
private:
    PN532_SPI *pn532spi;
    PN532 *nfc;
    AuthState currentState;
    AuthCallback authCallback;

    // HomeKey data storage
    struct
    {
        std::vector<uint8_t> readerPrivateKey;
        std::vector<uint8_t> readerPublicKey;
        std::vector<uint8_t> readerIdentifier;
        std::vector<uint8_t> readerGroupIdentifier;
        std::vector<std::pair<std::vector<uint8_t>, std::vector<uint8_t>>> issuers; // ID, PublicKey pairs
    } homeKeyData;

    // Authentication context
    uint8_t currentTransactionId[16];
    uint8_t ephemeralPrivateKey[32];
    uint8_t ephemeralPublicKey[65];

    // NFC communication helpers
    bool sendAPDU(uint8_t *cmd, uint8_t cmdLen, uint8_t *response, uint8_t *responseLen);
    bool selectHomeKeyApplet();
    bool performFastAuth();
    bool performStandardAuth();

    // Crypto helpers
    void generateEphemeralKeyPair();
    bool verifyIssuer(const uint8_t *issuerId, const uint8_t *signature);

public:
    HomeKeyReader(uint8_t clk, uint8_t miso, uint8_t mosi, uint8_t ss);
    ~HomeKeyReader();

    bool begin();
    void poll();
    bool processProvisioningData(uint8_t *data, size_t len);
    void setAuthenticationCallback(AuthCallback callback);

    // Data management
    bool saveToNVS();
    bool loadFromNVS();
    void clearStoredData();
};

#endif