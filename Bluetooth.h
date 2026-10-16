/**
 * @file Bluetooth.h
 * @brief Classic Bluetooth Serial Port Profile (SPP) link between the rover and the beacon
 *
 * @note Rover acts as the SPP slave, listening for the inbound connection dialed by the beacon.
         Only accepts a single connection, verified as `BEACON_BT_MAC`, rejecting and disconnecting
         anything else.
 */

#ifndef BLUETOOTH_H
#define BLUETOOTH_H

#include <BluetoothSerial.h> // Enables classic Bluetooth Serial Port Profile communication
#include "esp_spp_api.h" // Allows registration of Serial Port Profile Bluetooth callbacks
#include "esp_bt_device.h" // Allows reading the onboard chip's actual Bluetooth MAC address
#include "Logger.h" // Local logging helper class

extern Logger Logger; // Defined in rover.ino

BluetoothSerial SerialBT;
volatile bool bt_connected = false;
#define ROVER_BT_NAME "ROVER"
#define ROVER_BT_MAC "64:B7:08:66:8D:56"
#define BEACON_BT_MAC "B0:A7:32:1B:93:62"

// Beacon commands
#define CMD_LEFT "LEFT"
#define CMD_RIGHT "RIGHT"
#define CMD_TERMINATOR '\n'
#define BT_TOKEN_MAX_LEN 16 // Max length of a single command name or value read over Bluetooth, including null terminator

/**
 * @brief Helper function for comparing a plain text colon-separated MAC address with a raw MAC address
 *
 * @note `plain_text_mac` is parsed into a raw 6-byte MAC address before comparison with `raw_mac`.
          Used for comparing a hardcoded readable MAC to a raw MAC, such as the one returned from BluetoothSerial.
 *
 * @param plain_text_mac MAC address string, formatted as six colon-separated hex byte pairs (e.g. "AA:BB:CC:DD:EE:FF")
 * @param raw_mac Raw 6-byte MAC address to compare against
 *
 * @return `true` if the parsed address matches `raw_mac`, `false` otherwise
*/
bool compareMac(const char* plain_text_mac, const uint8_t raw_mac[6]) {
    // Parse plain text MAC into raw 6-byte MAC
    uint8_t parsed_mac[6];
    int parsed = sscanf(plain_text_mac, "%hhx:%hhx:%hhx:%hhx:%hhx:%hhx",
                        &parsed_mac[0], &parsed_mac[1], &parsed_mac[2],
                        &parsed_mac[3], &parsed_mac[4], &parsed_mac[5]);
    if (parsed != 6) return false; // Invalid MAC string structure, cannot match

    // Compare MACs
    for (uint8_t i = 0; i < 6; ++i) {
        if (parsed_mac[i] != raw_mac[i]) return false;
    }
    return true;
}

/**
 * @brief Handles Bluetooth connection events, accepting only the recognized BEACON device
 *
 * @note Registered as the callback for the Bluetooth SPP stack. Only a single connection verified as
         the beacon is accepted.
 *
 * @param event Type of Bluetooth event that occurred (e.g. device connected, device disconnected)
 * @param param Event-specific data, contains the remote device's raw MAC address on a connection event
*/
void bluetoothCallback(esp_spp_cb_event_t event, esp_spp_cb_param_t* param) {
    if (event == ESP_SPP_SRV_OPEN_EVT) {
        Logger.info("New BT device connected");

        // Beacon already connected, reject connection
        if (bt_connected) {
            Logger.info("BEACON already connected via BT, rejecting new connection");
            SerialBT.disconnect();
            return;
        }

        uint8_t* bda = param->srv_open.rem_bda; // New device raw 6-byte MAC
        bool match = compareMac(BEACON_BT_MAC, bda); // Compare new device MAC to beacon MAC

        // Beacon recognized
        if (match) {
            Logger.info("New BT device recognized as BEACON");
            bt_connected = true;
        }
        // Unknown device recognized, reject connection
        else {
            Logger.warn("New BT device not recognized, rejecting connection");
            SerialBT.disconnect();
        }
    } else if (event == ESP_SPP_CLOSE_EVT) {
        if (bt_connected) {
            Logger.error("BEACON disconnected from BT");
            bt_connected = false;
        }
    }
}

/**
 * @brief Initializes Bluetooth and begins listening for the beacon's inbound connection
 *
 * @note Registers `bluetoothCallback` for connection events and starts the Bluetooth stack in SPP
         slave mode under `ROVER_BT_NAME`. Also logs the onboard chip's actual BT MAC so it can be
         checked against the hardcoded `ROVER_BT_MAC` in beacon/Bluetooth.h. Should be called once
         from `setup()`.
*/
void beginBluetooth() {
    SerialBT.register_callback(bluetoothCallback); // Callback function fires every time BT stack has an event
    SerialBT.begin(ROVER_BT_NAME); // Bluetooth "server"; beacon connects to it

    // TEMPORARY: log the onboard chip's actual BT MAC to check it against the hardcoded ROVER_BT_MAC in beacon/Bluetooth.h
    const uint8_t* bt_mac = esp_bt_dev_get_address();
    char bt_mac_str[18];
    snprintf(bt_mac_str, sizeof(bt_mac_str), "%02X:%02X:%02X:%02X:%02X:%02X",
             bt_mac[0], bt_mac[1], bt_mac[2], bt_mac[3], bt_mac[4], bt_mac[5]);
    Logger.info(bt_mac_str);
}

/**
 * @brief Reads a single token from the beacon over Bluetooth into a buffer
 *
 * @note Blocks until a complete token ending in `CMD_TERMINATOR` is received or `buffer_len - 1`
         bytes have been read, matching the newline-terminated protocol sent by `sendCommand` in
         the beacon firmware. `buffer` is always null-terminated.
 *
 * @param buffer Output buffer to receive the token
 * @param buffer_len Size of `buffer`, including space for the null terminator
*/
void readBluetoothCommand(char* buffer, size_t buffer_len) {
    size_t len = SerialBT.readBytesUntil(CMD_TERMINATOR, buffer, buffer_len - 1);
    buffer[len] = '\0';
}

#endif
