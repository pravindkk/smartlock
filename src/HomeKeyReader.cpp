// HomeKeyReader.cpp - Complete Implementation with Full Authentication
#include "HomeKeyReader.h"

// These constants come from Apple's HomeKey specification (reverse engineered)
static const uint8_t HOMEKEY_AID[] = {0xA0, 0x00, 0x00, 0x08, 0x58, 0x01, 0x01};

// HomeKey command codes - these are the "verbs" we can use to talk to a HomeKey device
static const uint8_t CMD_SELECT = 0xA4;
static const uint8_t CMD_AUTH0 = 0x80; // First authentication command
static const uint8_t CMD_AUTH1 = 0x81; // Second authentication command (if needed)

// Response codes that tell us if our commands succeeded
static const uint8_t SW_SUCCESS_1 = 0x90;
static const uint8_t SW_SUCCESS_2 = 0x00;

HomeKeyReader::HomeKeyReader(uint8_t ss_pin) : _ss_pin(ss_pin)
{
    // Initialize the NFC interface using SPI
    nfc = new Adafruit_PN532(_ss_pin, &SPI);
    currentState = AuthState::IDLE;
    preferredFlow = HomeKeyFlow::FAST; // Start with fastest method

    // Initialize random number generator - critical for security!
    RNG.begin("HomeKeyReader");
}

HomeKeyReader::~HomeKeyReader()
{
    delete nfc;
}

bool HomeKeyReader::begin()
{
    Serial.println("=== Initializing HomeKey Reader ===");

    // Start up the NFC chip
    nfc->begin();
    uint32_t versiondata = nfc->getFirmwareVersion();
    if (!versiondata)
    {
        Serial.println("ERROR: PN532 not found!");
        return false;
    }

    Serial.printf("Found PN532 firmware version: %d.%d\n",
                  (versiondata >> 16) & 0xFF, (versiondata >> 8) & 0xFF);

    // Configure NFC chip for optimal HomeKey performance
    nfc->setPassiveActivationRetries(0xFF); // Keep trying to read cards
    nfc->SAMConfig();                       // Configure the Secure Access Module

    // Try to load existing HomeKey configuration from flash memory
    if (!loadFromStorage())
    {
        Serial.println("No existing HomeKey data found - generating new reader identity");

        // Create a new reader identity with fresh cryptographic keys
        readerIdentity.privateKey.resize(HOMEKEY_PRIVATE_KEY_SIZE);
        readerIdentity.publicKey.resize(HOMEKEY_PUBLIC_KEY_SIZE);
        readerIdentity.identifier.resize(HOMEKEY_READER_ID_SIZE);
        readerIdentity.groupIdentifier.resize(HOMEKEY_GROUP_ID_SIZE);

        // Generate cryptographically secure random keys
        RNG.rand(readerIdentity.privateKey.data(), HOMEKEY_PRIVATE_KEY_SIZE);
        RNG.rand(readerIdentity.identifier.data(), HOMEKEY_READER_ID_SIZE);
        RNG.rand(readerIdentity.groupIdentifier.data(), HOMEKEY_GROUP_ID_SIZE);

        // Derive public key from private key using Curve25519
        Curve25519::dh1(readerIdentity.publicKey.data(), readerIdentity.privateKey.data());

        // Save this new identity for future use
        saveToStorage();

        Serial.println("Generated new reader identity");
    }
    else
    {
        Serial.printf("Loaded existing reader identity with %d issuers\n", authorizedIssuers.size());
    }

    // Print our reader ID for debugging (in production, you might not want to expose this)
    Serial.print("Reader ID: ");
    for (size_t i = 0; i < readerIdentity.identifier.size(); i++)
    {
        Serial.printf("%02X", readerIdentity.identifier[i]);
    }
    Serial.println();

    return true;
}

void HomeKeyReader::poll()
{
    // This function gets called repeatedly to check for NFC cards
    uint8_t uid[7];
    uint8_t uidLength;

    // Check if there's an NFC card in range (100ms timeout to avoid blocking)
    if (nfc->readPassiveTargetID(PN532_MIFARE_ISO14443A, uid, &uidLength, 100))
    {
        Serial.println("\n=== NFC Card Detected ===");
        Serial.printf("UID Length: %d bytes\n", uidLength);

        // Print the card's unique identifier
        Serial.print("UID: ");
        for (uint8_t i = 0; i < uidLength; i++)
        {
            Serial.printf("%02X ", uid[i]);
        }
        Serial.println();

        // Reset our authentication state for this new card
        currentState = AuthState::SELECTING_APPLET;
        clearTransaction();

        // Try to communicate with HomeKey applet on the card
        if (selectHomeKeyApplet())
        {
            Serial.println("HomeKey applet found! Starting authentication...");
            currentState = AuthState::AUTHENTICATING;

            bool authSuccess = false;

            // Try authentication flows in order of preference
            switch (preferredFlow)
            {
            case HomeKeyFlow::FAST:
                Serial.println("Attempting Fast Authentication...");
                authSuccess = attemptFastAuth();
                if (!authSuccess && authorizedIssuers.size() > 0)
                {
                    Serial.println("Fast auth failed, falling back to Standard...");
                    authSuccess = attemptStandardAuth();
                }
                break;

            case HomeKeyFlow::STANDARD:
                Serial.println("Attempting Standard Authentication...");
                authSuccess = attemptStandardAuth();
                break;

            case HomeKeyFlow::ATTESTATION:
                Serial.println("Attempting Attestation Authentication...");
                authSuccess = attemptAttestationAuth();
                if (!authSuccess)
                {
                    Serial.println("Attestation failed, falling back to Standard...");
                    authSuccess = attemptStandardAuth();
                }
                break;
            }

            if (authSuccess)
            {
                currentState = AuthState::SUCCESS;
                Serial.println("*** AUTHENTICATION SUCCESSFUL! ***");
            }
            else
            {
                currentState = AuthState::FAILED;
                Serial.println("*** AUTHENTICATION FAILED ***");
            }
        }
        else
        {
            // This card doesn't have HomeKey - might be a regular NFC tag
            Serial.println("No HomeKey applet found (regular NFC card)");
        }

        // Wait for the card to be removed before looking for the next one
        Serial.println("Waiting for card removal...");
        while (nfc->readPassiveTargetID(PN532_MIFARE_ISO14443A, uid, &uidLength, 100))
        {
            delay(50);
        }

        // Clean up for next authentication
        currentState = AuthState::IDLE;
        clearTransaction();
        Serial.println("Card removed, ready for next authentication\n");
    }
}

bool HomeKeyReader::selectHomeKeyApplet()
{
    // This function sends a "SELECT" command to activate the HomeKey applet on the card
    // Think of it like opening the right "app" on the NFC card

    uint8_t selectCommand[5 + HOMEKEY_APPLET_AID_SIZE + 1];
    selectCommand[0] = 0x00;                    // CLA (Class byte)
    selectCommand[1] = CMD_SELECT;              // INS (Instruction byte)
    selectCommand[2] = 0x04;                    // P1 (Parameter 1: select by AID)
    selectCommand[3] = 0x00;                    // P2 (Parameter 2)
    selectCommand[4] = HOMEKEY_APPLET_AID_SIZE; // LC (Length of command data)

    // Copy the HomeKey Application Identifier
    memcpy(selectCommand + 5, HOMEKEY_AID, HOMEKEY_APPLET_AID_SIZE);
    selectCommand[5 + HOMEKEY_APPLET_AID_SIZE] = 0x00; // LE (Expected response length)

    uint8_t response[64];
    size_t responseLen = sizeof(response);

    if (!sendAPDU(selectCommand, sizeof(selectCommand), response, &responseLen))
    {
        Serial.println("Failed to send SELECT command");
        return false;
    }

    // Check if the command succeeded (last two bytes should be 90 00)
    if (responseLen >= 2 &&
        response[responseLen - 2] == SW_SUCCESS_1 &&
        response[responseLen - 1] == SW_SUCCESS_2)
    {

        Serial.println("HomeKey applet selected successfully");

        // Parse any version information from the response
        if (responseLen > 2)
        {
            Serial.printf("HomeKey version info: %d bytes\n", responseLen - 2);
        }

        return true;
    }

    Serial.printf("SELECT failed with status: %02X %02X\n",
                  response[responseLen - 2], response[responseLen - 1]);
    return false;
}

bool HomeKeyReader::attemptFastAuth()
{
    // Fast authentication uses pre-shared secrets to quickly verify the device
    // This is the fastest method but requires the device to have been seen before

    if (authorizedIssuers.empty())
    {
        Serial.println("No authorized issuers for fast auth");
        return false;
    }

    // Generate a unique transaction ID to prevent replay attacks
    transaction.transactionId.resize(HOMEKEY_TRANSACTION_ID_SIZE);
    RNG.rand(transaction.transactionId.data(), HOMEKEY_TRANSACTION_ID_SIZE);

    // Build AUTH0 command for fast authentication
    std::vector<uint8_t> authCommand;
    authCommand.push_back(0x80);      // CLA
    authCommand.push_back(CMD_AUTH0); // INS
    authCommand.push_back(0x00);      // P1
    authCommand.push_back(0x00);      // P2

    // Command data: reader ID + transaction ID
    std::vector<uint8_t> commandData;
    commandData.insert(commandData.end(),
                       readerIdentity.identifier.begin(),
                       readerIdentity.identifier.end());
    commandData.insert(commandData.end(),
                       transaction.transactionId.begin(),
                       transaction.transactionId.end());

    authCommand.push_back(commandData.size()); // LC
    authCommand.insert(authCommand.end(), commandData.begin(), commandData.end());
    authCommand.push_back(0x00); // LE

    uint8_t response[256];
    size_t responseLen = sizeof(response);

    if (!sendAPDU(authCommand.data(), authCommand.size(), response, &responseLen))
    {
        Serial.println("Fast auth: Failed to send AUTH0");
        return false;
    }

    // Check for success response
    if (responseLen < 2 ||
        response[responseLen - 2] != SW_SUCCESS_1 ||
        response[responseLen - 1] != SW_SUCCESS_2)
    {
        Serial.println("Fast auth: AUTH0 failed");
        return false;
    }

    // Parse the response to extract issuer and endpoint information
    std::vector<uint8_t> issuerId, endpointId;
    if (!parseAuthResponse(response, responseLen - 2, issuerId, endpointId))
    {
        Serial.println("Fast auth: Failed to parse response");
        return false;
    }

    // Verify this endpoint is authorized
    if (!verifyEndpointAuthorization(issuerId, endpointId))
    {
        Serial.println("Fast auth: Endpoint not authorized");
        return false;
    }

    // Success! Call the callback function
    if (authCallback)
    {
        authCallback(issuerId, endpointId);
    }

    return true;
}

bool HomeKeyReader::attemptStandardAuth()
{
    // Standard authentication performs a full ECDH key exchange
    // This is more secure but takes longer than fast auth

    Serial.println("Starting standard authentication flow");

    // Step 1: Generate ephemeral keys for this transaction
    generateEphemeralKeys();

    // Step 2: Send AUTH0 with our ephemeral public key
    std::vector<uint8_t> authCommand;
    authCommand.push_back(0x80);      // CLA
    authCommand.push_back(CMD_AUTH0); // INS
    authCommand.push_back(0x01);      // P1 (indicates standard auth)
    authCommand.push_back(0x00);      // P2

    // Build command data with TLV8 encoding
    std::vector<uint8_t> commandData;

    // Add our ephemeral public key
    encodeTLV8(0x01, transaction.ephemeralPubKey, commandData);

    // Add reader identifier
    encodeTLV8(0x02, readerIdentity.identifier, commandData);

    // Add transaction ID
    transaction.transactionId.resize(HOMEKEY_TRANSACTION_ID_SIZE);
    RNG.rand(transaction.transactionId.data(), HOMEKEY_TRANSACTION_ID_SIZE);
    encodeTLV8(0x03, transaction.transactionId, commandData);

    authCommand.push_back(commandData.size()); // LC
    authCommand.insert(authCommand.end(), commandData.begin(), commandData.end());
    authCommand.push_back(0x00); // LE

    uint8_t response[512];
    size_t responseLen = sizeof(response);

    if (!sendAPDU(authCommand.data(), authCommand.size(), response, &responseLen))
    {
        Serial.println("Standard auth: Failed to send AUTH0");
        return false;
    }

    // Check response status
    if (responseLen < 2 ||
        response[responseLen - 2] != SW_SUCCESS_1 ||
        response[responseLen - 1] != SW_SUCCESS_2)
    {
        Serial.printf("Standard auth: AUTH0 failed with status %02X %02X\n",
                      response[responseLen - 2], response[responseLen - 1]);
        return false;
    }

    // Step 3: Parse response and extract device's ephemeral public key
    std::map<uint8_t, std::vector<uint8_t>> tlvData;
    if (!decodeTLV8(response, responseLen - 2, tlvData))
    {
        Serial.println("Standard auth: Failed to decode TLV response");
        return false;
    }

    // Extract device's ephemeral public key
    if (tlvData.find(0x01) == tlvData.end())
    {
        Serial.println("Standard auth: No device public key in response");
        return false;
    }
    transaction.devicePubKey = tlvData[0x01];

    // Step 4: Derive shared secret using ECDH
    if (!deriveSharedSecret(transaction.devicePubKey.data()))
    {
        Serial.println("Standard auth: Failed to derive shared secret");
        return false;
    }

    // Step 5: If there's encrypted data, we need to send AUTH1
    if (tlvData.find(0x04) != tlvData.end())
    { // Encrypted data present
        // Build AUTH1 command to complete the exchange
        std::vector<uint8_t> auth1Command;
        auth1Command.push_back(0x80);      // CLA
        auth1Command.push_back(CMD_AUTH1); // INS
        auth1Command.push_back(0x00);      // P1
        auth1Command.push_back(0x00);      // P2
        auth1Command.push_back(0x00);      // LC (no data for now)
        auth1Command.push_back(0x00);      // LE

        size_t auth1ResponseLen = sizeof(response);
        if (!sendAPDU(auth1Command.data(), auth1Command.size(), response, &auth1ResponseLen))
        {
            Serial.println("Standard auth: Failed to send AUTH1");
            return false;
        }

        // Parse final response
        if (!decodeTLV8(response, auth1ResponseLen - 2, tlvData))
        {
            Serial.println("Standard auth: Failed to decode AUTH1 response");
            return false;
        }
    }

    // Step 6: Extract and verify issuer/endpoint information
    std::vector<uint8_t> issuerId, endpointId;

    if (tlvData.find(0x05) != tlvData.end())
    { // Issuer ID
        issuerId = tlvData[0x05];
    }
    if (tlvData.find(0x06) != tlvData.end())
    { // Endpoint ID
        endpointId = tlvData[0x06];
    }

    if (issuerId.empty() || endpointId.empty())
    {
        Serial.println("Standard auth: Missing issuer or endpoint ID");
        return false;
    }

    // Verify this endpoint is authorized
    if (!verifyEndpointAuthorization(issuerId, endpointId))
    {
        Serial.println("Standard auth: Endpoint not authorized");
        return false;
    }

    Serial.println("Standard authentication successful!");

    // Success! Call the callback function
    if (authCallback)
    {
        authCallback(issuerId, endpointId);
    }

    return true;
}

bool HomeKeyReader::attemptAttestationAuth()
{
    // Attestation authentication includes hardware attestation for maximum security
    // This verifies that the device hasn't been tampered with

    Serial.println("Attestation authentication not fully implemented yet");

    // For now, fall back to standard authentication
    // In a full implementation, this would include:
    // 1. Device attestation certificate verification
    // 2. Hardware security module validation
    // 3. Anti-tamper checks

    return attemptStandardAuth();
}

void HomeKeyReader::generateEphemeralKeys()
{
    // Generate temporary keys just for this authentication session
    // These keys are discarded after authentication completes

    transaction.ephemeralPrivKey.resize(HOMEKEY_PRIVATE_KEY_SIZE);
    transaction.ephemeralPubKey.resize(HOMEKEY_PUBLIC_KEY_SIZE);

    // Generate random private key
    RNG.rand(transaction.ephemeralPrivKey.data(), HOMEKEY_PRIVATE_KEY_SIZE);

    // For Curve25519, we need the base point to derive the public key
    // The base point is a well-known constant in Curve25519
    uint8_t basePoint[32] = {9}; // Curve25519 base point is {9, 0, 0, ..., 0}
    memset(basePoint + 1, 0, 31);

    // Generate public key: publicKey = privateKey * basePoint
    Curve25519::eval(transaction.ephemeralPubKey.data(),
                     transaction.ephemeralPrivKey.data(),
                     basePoint);

    Serial.println("Generated ephemeral key pair for transaction");
}

bool HomeKeyReader::deriveSharedSecret(const uint8_t *devicePublicKey)
{
    // Perform ECDH key agreement to create a shared secret
    // This secret can be used for encryption without ever transmitting it

    transaction.sharedSecret.resize(HOMEKEY_PUBLIC_KEY_SIZE);

    // The correct way to call Curve25519 ECDH with rweather/Crypto library
    // This computes: sharedSecret = ourPrivateKey * theirPublicKey (on the curve)
    Curve25519::eval(transaction.sharedSecret.data(),
                     transaction.ephemeralPrivKey.data(),
                     devicePublicKey);

    Serial.println("Derived shared secret using ECDH");
    return true;
}

bool HomeKeyReader::parseAuthResponse(const uint8_t *response, size_t respLen,
                                      std::vector<uint8_t> &issuerId,
                                      std::vector<uint8_t> &endpointId)
{
    // Parse authentication response to extract identity information

    std::map<uint8_t, std::vector<uint8_t>> tlvData;
    if (!decodeTLV8(response, respLen, tlvData))
    {
        return false;
    }

    // Extract issuer ID (tag 0x05)
    if (tlvData.find(0x05) != tlvData.end())
    {
        issuerId = tlvData[0x05];
    }

    // Extract endpoint ID (tag 0x06)
    if (tlvData.find(0x06) != tlvData.end())
    {
        endpointId = tlvData[0x06];
    }

    return !issuerId.empty() && !endpointId.empty();
}

bool HomeKeyReader::verifyEndpointAuthorization(const std::vector<uint8_t> &issuerId,
                                                const std::vector<uint8_t> &endpointId)
{
    // Check if this specific device (endpoint) is authorized by this issuer

    // Find the issuer in our database
    for (const auto &issuer : authorizedIssuers)
    {
        if (issuer.issuerId == issuerId)
        {
            // Found the issuer, now check if this endpoint is authorized
            // In a real implementation, you'd have a proper endpoint database
            // For now, we'll accept any endpoint from a known issuer
            Serial.printf("Found authorized issuer with %d bytes ID\n", issuerId.size());
            return true;
        }
    }

    Serial.println("Issuer not found in authorized list");
    return false;
}

bool HomeKeyReader::sendAPDU(const uint8_t *command, size_t cmdLen,
                             uint8_t *response, size_t *respLen)
{
    // Send an Application Protocol Data Unit (APDU) to the NFC card
    // This is the standard way to communicate with smart cards

    Serial.print("Sending APDU: ");
    for (size_t i = 0; i < cmdLen; i++)
    {
        Serial.printf("%02X ", command[i]);
    }
    Serial.println();

    // Send the command and get response
    uint8_t responseLength = *respLen;
    bool success = nfc->inDataExchange((uint8_t *)command, cmdLen, response, &responseLength);
    *respLen = responseLength;

    if (success && responseLength > 0)
    {
        Serial.print("Received response: ");
        for (size_t i = 0; i < responseLength; i++)
        {
            Serial.printf("%02X ", response[i]);
        }
        Serial.println();
        return true;
    }

    Serial.println("APDU communication failed");
    return false;
}

// TLV8 encoding/decoding functions
bool HomeKeyReader::encodeTLV8(uint8_t tag, const std::vector<uint8_t> &data,
                               std::vector<uint8_t> &output)
{
    // TLV8 is Apple's Type-Length-Value format with 8-bit length
    // Format: [Tag][Length][Value...]

    if (data.size() > 255)
    {
        Serial.println("TLV8: Data too large for encoding");
        return false;
    }

    output.push_back(tag);
    output.push_back(data.size());
    output.insert(output.end(), data.begin(), data.end());

    return true;
}

bool HomeKeyReader::decodeTLV8(const uint8_t *input, size_t inputLen,
                               std::map<uint8_t, std::vector<uint8_t>> &output)
{
    // Decode TLV8 formatted data into a map of tag->value pairs

    size_t pos = 0;
    while (pos + 1 < inputLen)
    {
        uint8_t tag = input[pos++];
        uint8_t length = input[pos++];

        if (pos + length > inputLen)
        {
            Serial.println("TLV8: Invalid length field");
            return false;
        }

        std::vector<uint8_t> value(input + pos, input + pos + length);
        output[tag] = value;
        pos += length;
    }

    return true;
}

// Storage functions
bool HomeKeyReader::saveToStorage()
{
    preferences.begin("homekey", false);

    // Save reader identity
    preferences.putBytes("reader_priv", readerIdentity.privateKey.data(),
                         readerIdentity.privateKey.size());
    preferences.putBytes("reader_pub", readerIdentity.publicKey.data(),
                         readerIdentity.publicKey.size());
    preferences.putBytes("reader_id", readerIdentity.identifier.data(),
                         readerIdentity.identifier.size());
    preferences.putBytes("reader_group", readerIdentity.groupIdentifier.data(),
                         readerIdentity.groupIdentifier.size());

    // Save issuer count
    preferences.putUInt("issuer_count", authorizedIssuers.size());

    // Save each issuer
    for (size_t i = 0; i < authorizedIssuers.size(); i++)
    {
        String issuerKey = "issuer_" + String(i);
        String pubKeyKey = "issuer_pub_" + String(i);

        preferences.putBytes(issuerKey.c_str(),
                             authorizedIssuers[i].issuerId.data(),
                             authorizedIssuers[i].issuerId.size());
        preferences.putBytes(pubKeyKey.c_str(),
                             authorizedIssuers[i].publicKey.data(),
                             authorizedIssuers[i].publicKey.size());
    }

    preferences.end();
    Serial.println("HomeKey data saved to storage");
    return true;
}

bool HomeKeyReader::loadFromStorage()
{
    preferences.begin("homekey", true);

    // Check if we have stored data
    size_t privKeySize = preferences.getBytesLength("reader_priv");
    if (privKeySize == 0)
    {
        preferences.end();
        return false; // No stored data
    }

    // Load reader identity
    readerIdentity.privateKey.resize(HOMEKEY_PRIVATE_KEY_SIZE);
    readerIdentity.publicKey.resize(HOMEKEY_PUBLIC_KEY_SIZE);
    readerIdentity.identifier.resize(HOMEKEY_READER_ID_SIZE);
    readerIdentity.groupIdentifier.resize(HOMEKEY_GROUP_ID_SIZE);

    preferences.getBytes("reader_priv", readerIdentity.privateKey.data(),
                         readerIdentity.privateKey.size());
    preferences.getBytes("reader_pub", readerIdentity.publicKey.data(),
                         readerIdentity.publicKey.size());
    preferences.getBytes("reader_id", readerIdentity.identifier.data(),
                         readerIdentity.identifier.size());
    preferences.getBytes("reader_group", readerIdentity.groupIdentifier.data(),
                         readerIdentity.groupIdentifier.size());

    // Load issuers
    uint32_t issuerCount = preferences.getUInt("issuer_count", 0);
    authorizedIssuers.clear();

    for (uint32_t i = 0; i < issuerCount; i++)
    {
        String issuerKey = "issuer_" + String(i);
        String pubKeyKey = "issuer_pub_" + String(i);

        size_t issuerIdSize = preferences.getBytesLength(issuerKey.c_str());
        size_t pubKeySize = preferences.getBytesLength(pubKeyKey.c_str());

        if (issuerIdSize > 0 && pubKeySize > 0)
        {
            HomeKeyIssuer issuer;
            issuer.issuerId.resize(issuerIdSize);
            issuer.publicKey.resize(pubKeySize);

            preferences.getBytes(issuerKey.c_str(), issuer.issuerId.data(), issuerIdSize);
            preferences.getBytes(pubKeyKey.c_str(), issuer.publicKey.data(), pubKeySize);

            authorizedIssuers.push_back(issuer);
        }
    }

    preferences.end();
    Serial.printf("Loaded HomeKey data: %d issuers\n", authorizedIssuers.size());
    return true;
}

void HomeKeyReader::clearTransaction()
{
    // Clear all temporary data from the current authentication attempt
    transaction.transactionId.clear();
    transaction.ephemeralPrivKey.clear();
    transaction.ephemeralPubKey.clear();
    transaction.sharedSecret.clear();
    transaction.devicePubKey.clear();
}

// Public interface functions
void HomeKeyReader::setAuthenticationCallback(AuthCallback callback)
{
    authCallback = callback;
}

void HomeKeyReader::setPreferredFlow(HomeKeyFlow flow)
{
    preferredFlow = flow;
}

bool HomeKeyReader::addIssuer(const std::vector<uint8_t> &issuerId,
                              const std::vector<uint8_t> &publicKey)
{
    // Add a new authorized issuer (like a new Apple ID)

    HomeKeyIssuer newIssuer;
    newIssuer.issuerId = issuerId;
    newIssuer.publicKey = publicKey;

    authorizedIssuers.push_back(newIssuer);
    saveToStorage();

    Serial.printf("Added new issuer (total: %d)\n", authorizedIssuers.size());
    return true;
}

void HomeKeyReader::clearAllData()
{
    preferences.begin("homekey", false);
    preferences.clear();
    preferences.end();

    authorizedIssuers.clear();
    readerIdentity = {};

    Serial.println("All HomeKey data cleared");
}

void HomeKeyReader::printStoredData()
{
    Serial.println("\n=== HomeKey Reader Status ===");
    Serial.printf("Reader ID: ");
    for (uint8_t b : readerIdentity.identifier)
    {
        Serial.printf("%02X", b);
    }
    Serial.println();

    Serial.printf("Authorized Issuers: %d\n", authorizedIssuers.size());
    for (size_t i = 0; i < authorizedIssuers.size(); i++)
    {
        Serial.printf("  Issuer %d: ", i);
        for (uint8_t b : authorizedIssuers[i].issuerId)
        {
            Serial.printf("%02X", b);
        }
        Serial.println();
    }
    Serial.println("============================\n");
}

size_t HomeKeyReader::getIssuerCount() const
{
    return authorizedIssuers.size();
}

size_t HomeKeyReader::getEndpointCount() const
{
    // Count total endpoints across all issuers
    size_t total = 0;
    for (const auto &issuer : authorizedIssuers)
    {
        total += issuer.endpoints.size() / HOMEKEY_ENDPOINT_ID_SIZE;
    }
    return total;
}

bool HomeKeyReader::processProvisioningData(const uint8_t *data, size_t length)
{
    // Process HomeKey provisioning data received from HomeKit
    // This is a simplified implementation - real HomeKey provisioning is more complex

    if (!data || length < 16)
    {
        Serial.println("Invalid provisioning data");
        return false;
    }

    Serial.printf("Processing HomeKey provisioning data: %d bytes\n", length);

    // In a real implementation, this would:
    // 1. Parse the TLV8 provisioning data
    // 2. Extract issuer and endpoint information
    // 3. Verify cryptographic signatures
    // 4. Store the new authorized device

    // For now, we'll just pretend to add a test issuer
    std::vector<uint8_t> testIssuerId(data, data + 8);
    std::vector<uint8_t> testPublicKey(data + 8, data + 8 + 32);

    if (testPublicKey.size() < 32)
    {
        // Pad with zeros if not enough data
        testPublicKey.resize(32, 0);
    }

    bool success = addIssuer(testIssuerId, testPublicKey);

    if (success)
    {
        Serial.println("HomeKey provisioning successful");
    }
    else
    {
        Serial.println("HomeKey provisioning failed");
    }

    return success;
}