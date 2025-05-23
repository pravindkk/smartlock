// HomeKeyReader.h - Complete Implementation
#ifndef HOMEKEY_READER_H
#define HOMEKEY_READER_H

#include <Arduino.h>
#include <SPI.h>
#include <Adafruit_PN532.h>
#include <vector>
#include <map>
#include <functional>
#include <Crypto.h>
#include <AES.h>
#include <SHA256.h>
#include <Curve25519.h>
#include <RNG.h>
#include <ArduinoJson.h>
#include <Preferences.h>

// HomeKey protocol constants - these come from reverse engineering Apple's implementation
#define HOMEKEY_APPLET_AID_SIZE 7
#define HOMEKEY_READER_ID_SIZE 16
#define HOMEKEY_GROUP_ID_SIZE 8
#define HOMEKEY_ISSUER_ID_SIZE 8
#define HOMEKEY_ENDPOINT_ID_SIZE 6
#define HOMEKEY_PUBLIC_KEY_SIZE 32
#define HOMEKEY_PRIVATE_KEY_SIZE 32
#define HOMEKEY_TRANSACTION_ID_SIZE 16

// Authentication flow types that HomeKey supports
enum class HomeKeyFlow
{
    FAST,       // Fastest, uses pre-shared encryption keys
    STANDARD,   // Standard ECDH key exchange
    ATTESTATION // Includes hardware attestation
};

// Current state of the authentication process
enum class AuthState
{
    IDLE,
    SELECTING_APPLET,
    AUTHENTICATING,
    SUCCESS,
    FAILED
};

// Structure to hold issuer information (like a user's Apple ID)
struct HomeKeyIssuer
{
    std::vector<uint8_t> issuerId;  // Unique identifier for this issuer
    std::vector<uint8_t> publicKey; // Issuer's public key for verification
    std::vector<uint8_t> endpoints; // List of authorized endpoints (devices)
};

// Structure to hold endpoint information (like a specific iPhone)
struct HomeKeyEndpoint
{
    std::vector<uint8_t> endpointId; // Unique identifier for this device
    std::vector<uint8_t> publicKey;  // Device's public key
    std::vector<uint8_t> counter;    // Anti-replay counter
};

// Callback function type for successful authentication
typedef std::function<void(const std::vector<uint8_t> &issuerId,
                           const std::vector<uint8_t> &endpointId)>
    AuthCallback;

class HomeKeyReader
{
private:
    // Hardware interface to the NFC chip
    Adafruit_PN532 *nfc;
    uint8_t _ss_pin;

    // Current authentication state
    AuthState currentState;
    HomeKeyFlow preferredFlow;
    AuthCallback authCallback;

    // Persistent storage for HomeKey data
    Preferences preferences;

    // Reader's own cryptographic identity
    struct ReaderIdentity
    {
        std::vector<uint8_t> privateKey;      // Reader's private key (keep secret!)
        std::vector<uint8_t> publicKey;       // Reader's public key (shared with devices)
        std::vector<uint8_t> identifier;      // Unique reader ID
        std::vector<uint8_t> groupIdentifier; // Group this reader belongs to
    } readerIdentity;

    // Database of authorized issuers and their endpoints
    std::vector<HomeKeyIssuer> authorizedIssuers;

    // Current transaction context (cleared after each authentication)
    struct TransactionContext
    {
        std::vector<uint8_t> transactionId;    // Unique ID for this auth attempt
        std::vector<uint8_t> ephemeralPrivKey; // Temporary private key
        std::vector<uint8_t> ephemeralPubKey;  // Temporary public key
        std::vector<uint8_t> sharedSecret;     // Derived shared secret
        std::vector<uint8_t> devicePubKey;     // Device's public key for this transaction
    } transaction;

public:
    HomeKeyReader(uint8_t ss_pin);
    ~HomeKeyReader();

    // Basic lifecycle functions
    bool begin();
    void poll();
    void setAuthenticationCallback(AuthCallback callback);
    void setPreferredFlow(HomeKeyFlow flow);

    // Provisioning functions (for adding new devices)
    bool processProvisioningData(const uint8_t *data, size_t length);
    bool addIssuer(const std::vector<uint8_t> &issuerId,
                   const std::vector<uint8_t> &publicKey);
    bool addEndpoint(const std::vector<uint8_t> &issuerId,
                     const std::vector<uint8_t> &endpointId,
                     const std::vector<uint8_t> &publicKey);

    // Data management
    bool saveToStorage();
    bool loadFromStorage();
    void clearAllData();

    // Debugging and status
    void printStoredData();
    size_t getIssuerCount() const;
    size_t getEndpointCount() const;

private:
    // Core NFC communication functions
    bool selectHomeKeyApplet();
    bool sendAPDU(const uint8_t *command, size_t cmdLen,
                  uint8_t *response, size_t *respLen);

    // Authentication flow implementations
    bool attemptFastAuth();
    bool attemptStandardAuth();
    bool attemptAttestationAuth();

    // Cryptographic helper functions
    void generateEphemeralKeys();
    bool deriveSharedSecret(const uint8_t *devicePublicKey);
    bool verifySignature(const uint8_t *data, size_t dataLen,
                         const uint8_t *signature, size_t sigLen,
                         const uint8_t *publicKey);

    // Protocol message handling
    bool parseAuthResponse(const uint8_t *response, size_t respLen,
                           std::vector<uint8_t> &issuerId,
                           std::vector<uint8_t> &endpointId);
    bool verifyEndpointAuthorization(const std::vector<uint8_t> &issuerId,
                                     const std::vector<uint8_t> &endpointId);

    // TLV8 encoding/decoding utilities (Apple's data format)
    bool encodeTLV8(uint8_t tag, const std::vector<uint8_t> &data,
                    std::vector<uint8_t> &output);
    bool decodeTLV8(const uint8_t *input, size_t inputLen,
                    std::map<uint8_t, std::vector<uint8_t>> &output);

    // Utility functions
    void clearTransaction();
    bool isValidEndpoint(const std::vector<uint8_t> &issuerId,
                         const std::vector<uint8_t> &endpointId);
};

#endif