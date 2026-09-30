/**
 * @file rover.ino
 * @brief Firmware for controlling a Bluetooth and UltraWide Band compatible rover
 *
 * @details Designed to run on a compatible microcontroller in combination with a Bluetooth antenna 
            and an UltraWide Band antenna. The prototype uses a MakerFabs ESP32 UWB, which features 
            the ESP32-D0WD-V3 microcontroller chip, a DW1000 UWB antenna, and an integrated onboard
            PCB trace WiFi and Bluetooth antenna.
 *
 * @note Rover firmware will ignore any unrecognized UWB connections and reject and disconnect from any 
         unrecognized BT connections. The MAC address of the connecting device's BT antenna must be  
         defined as `BEACON_BT_MAC` and short hardware address of the connecting device's DW1000 UWB antenna 
         must be defined as `BEACON_SHORT_ADDRESS`. This value is derived from the first 2 bytes of the antenna's 
         complete hardware address. Any connected serial monitor should run at 9600 baud. On boot, setup() waits 3 
         seconds to let the serial monitor attach before initialization output is printed.
 */

#include <SPI.h> // Enables Serial Peripheral Interface communication
#include <BluetoothSerial.h> // Enables classic Bluetooth Serial Port Profile communication
#include "esp_spp_api.h" // Allows registration of Serial Port Profile Bluetooth callbacks
#include "DW1000Ranging.h" // MakerFabs DW1000 UWB chip ranging library, imported as ZIP
#include "Logger.h" // Local logging helper class

Logger log; // Initialize logger on default serial

//---------- Bluetooth ----------
BluetoothSerial SerialBT;
bool bt_connected = false;
#define ROVER_BT_NAME "ROVER"
#define ROVER_BT_MAC "64:B7:08:66:8D:56"
#define BEACON_BT_MAC "B0:A7:32:1B:93:62"

//---------- UltraWide Band ----------
// Controller: Microcontroller, Peripheral: DW1000 UWB
const uint16_t DW1000_ANTENNA_DELAY = 16470; // 1 unit = 15.65ps (picoseconds)
const bool USE_RANGE_FILTER = true; // UWB range-smoothing filter
#define SPI_SCK 18  // SPI clock, keeps controller and peripheral in sync
#define PIN_PS 4    // SPI peripheral select (controller pulls to LOW to open transaction with controller)
#define SPI_MISO 19 // SPI data in (peripheral -> controller, data received from beacon)
#define SPI_MOSI 23 // SPI data out (controller -> peripheral, purely configuration data)
#define PIN_RST 27  // Hardware reset (controller pulls to LOW during init to reboot peripheral)
#define PIN_IRQ 34  // Interrupt (peripheral raises to HIGH when it has a completed measurement)
#define ROVER_HARDWARE_ADDRESS "7D:00:22:EA:82:60:3B:9C" // Hardware address of the onboard DW1000 UWB chip
#define BEACON_HARDWARE_ADDRESS "86:17:5B:D5:A9:9A:E2:9C" // Hardware address of beacon DW1000 UWB chip
#define BEACON_SHORT_ADDRESS 0x1786                        // Short address of beacon, derived from first two bytes of BEACON_HARDWARE_ADDRESS
const uint8_t NUM_RANGE_SAMPLES = 5; // Number of range samples to be recorded and averaged
float range_buf[NUM_RANGE_SAMPLES] = {0}; // Buffer to store range values for averaging, all zeroes by default
uint8_t range_buf_idx = 0; // Buffer index
bool uwb_connected = false; // Beacon DW1000 UWB chip connected

/**
 * @brief Helper function for comparing a plain text colon-separated MAC address with a raw MAC address
 *
 * @note `plain_text_mac` is parsed into a raw 6-byte MAC address before comparison with `raw_mac`.
          Used for comparing a hardcoded readable MAC to a raw MAC, such as the one returned from BluetoothSerial.
 *
 * @param plain_text_mac MAC address string, formatted as six colon-separated hex byte pairs (e.g. "AA:BB:CC:DD:EE:FF")
 * @param raw_mac Raw 6-byte MAC address to compare against
 *
 * @return `true` if the parsed address matches `raw_mac`, false otherwise
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
 * @brief Records a new range reading between onboard DW1000 UWB chip and connected device
 *
 * @note Registered as the callback function for when a complete ranging exchange occurs.
         Range readings are added to `range_buf` to later be averaged for more accurate readings.
*/
void newRangeCallback() {
    if (DW1000Ranging.getDistantDevice()->getShortAddress() != BEACON_SHORT_ADDRESS) return;
    range_buf[range_buf_idx] = DW1000Ranging.getDistantDevice()->getRange(); // Record range (meters), write to range buffer
    range_buf_idx = (range_buf_idx + 1) % NUM_RANGE_SAMPLES; // Increment index, overflows back to 0
}

/**
 * @brief Activates a new connected DW1000 UWB device
 *
 * @note Registered as the callback function for when a previously unseen UWB device 
         joins the ranging exchange. Ignores any device that is not recognized as the beacon.
 *
 * @param device New device to be activated
*/
void newDeviceCallback(DW1000Device *device) {
    log.info("New UWB device detected");
    if (device->getShortAddress() != BEACON_SHORT_ADDRESS) {
        log.warn("New UWB device not recognized as BEACON, rejecting connection");
        return;
    }

    // Beacon activated
    log.info("New UWB device recognized as BEACON");
    uwb_connected = true;
}

/**
 * @brief Deactivates a connected DW1000 UWB device
 *
 * @note Registered as the callback function for a DW1000 UWB device has not been heard from
 *
 * @param device Device being deactivated 
*/
void inactiveDeviceCallback(DW1000Device *device) {
    if (device->getShortAddress() != BEACON_SHORT_ADDRESS) {
        log.info("Unrecognized UWB device disconnected");
        return;
    }

    // Beacon deactivated
    uwb_connected = false;
    for (uint8_t i = 0; i < NUM_RANGE_SAMPLES; ++i) range_buf[i] = 0; // Zero out range buffer
    log.error("BEACON disconnected from UWB");
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
        log.info("New BT device connected");

        // Beacon already connected, reject connection
        if (bt_connected) {
            log.info("BEACON already connected via BT, rejecting new connection");
            SerialBT.disconnect();
            return;
        }

        uint8_t* bda = param->srv_open.rem_bda; // New device raw 6-byte MAC
        bool match = compareMac(BEACON_BT_MAC, bda); // Compare new device MAC to beacon MAC

        // Beacon recognized
        if (match) {
            log.info("New BT device recognized as BEACON");
            bt_connected = true;
        }
        // Unknown device recognized, reject connection
        else {
            log.warn("New BT device not recognized, rejecting connection");
            SerialBT.disconnect();
        }
    } else if (event == ESP_SPP_CLOSE_EVT) {
        if (bt_connected) {
            log.error("BEACON disconnected from BT");
            bt_connected = false;
        }
    }
}

void setup() {
    Serial.begin(9600); // Baud rate 9600 Bd
    delay(3000); // Allow serial monitor time to open

    SerialBT.register_callback(bluetoothCallback); // Callback function fires every time BT stack has an event
    SerialBT.begin(ROVER_BT_NAME); // Bluetooth "server"; beacon connects to it

    SPI.begin(SPI_SCK, SPI_MISO, SPI_MOSI); // Initialize SPI bus, hardwired to onboard DW1000 UWB chip
    DW1000Ranging.initCommunication(PIN_RST, PIN_PS, PIN_IRQ); // Initialize DW1000 UWB chip
    DW1000.setAntennaDelay(DW1000_ANTENNA_DELAY); // Delay after UWB signal arrives or before it leaves, fine-tuned for accurate distance readings

    DW1000Ranging.attachNewRange(newRangeCallback); // Callback function fires every time a complete ranging exchange between UWB chips occurs
    DW1000Ranging.attachNewDevice(newDeviceCallback); // Callback function fires when a previously unseen device joins the ranging exchange; only fires on initial detection of beacon
    DW1000Ranging.attachInactiveDevice(inactiveDeviceCallback); // Callback function fires when a device hasn't been heard from (lost connection, out of range)

    DW1000Ranging.useRangeFilter(USE_RANGE_FILTER); // Enable or disable builtin library range-smoothing feature
    DW1000Ranging.startAsTag(ROVER_HARDWARE_ADDRESS, DW1000.MODE_LONGDATA_RANGE_LOWPOWER); // Start UWB radio as a tag; long range low power mode
}

void loop() {
    DW1000Ranging.loop(); // Ranging library's main state machine, must be called repeatedly
}

/**
 * @brief Returns the distance between onboard DW1000 UWB chip and connected device (meters)
 *
 * @note Averages the last `NUM_RANGE_SAMPLES` range readings from the range buffer
 *
 * @return Distance between rover and beacon if beacon is detected, `NAN` if beacon is not detected
 */
float getBeaconDistance() {
    if (!uwb_connected) return NAN; // Beacon not detected, range buffer inaccurate

    // Calculate average of range buffer
    float avg = 0;
    for (uint8_t i = 0; i < NUM_RANGE_SAMPLES; ++i) avg += range_buf[i];
    avg /= NUM_RANGE_SAMPLES;

    return avg;
}
