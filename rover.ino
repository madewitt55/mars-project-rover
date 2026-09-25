#include <SPI.h>
#include "DW1000Ranging.h" // MakerFabs DW1000 UWB chip ranging library, imported as ZIP

//---------- DW1000 UWB ----------
// Controller: ESP32-WROOM-32E, Peripheral: DW1000 UWB
const uint16_t DW1000_ANTENNA_DELAY = 16470; // 1 unit = 15.65ps (picoseconds)
const bool USE_RANGE_FILTER = true; // UWB range-smoothing filter
#define SPI_SCK 18  // SPI clock, keeps controller and peripheral in sync
#define PIN_PS 4    // SPI peripheral select (controller pulls to LOW to open transaction with controller)
#define SPI_MISO 19 // SPI data in (peripheral -> controller, data recieved from beacon)
#define SPI_MOSI 23 // SPI data out (controller -> peripheral, purely configuration data)
#define PIN_RST 27  // Hardware reset (controller pulls to LOW during init to reboot peripheral)
#define PIN_IRQ 34  // Interrupt (peripheral raises to HIGH when it has a completed measurement)
#define ROVER_HARDWARE_ADDRESS "7D:00:22:EA:82:60:3B:9C" // Hardware address of the onboard DW1000 UWB chip
#define BEACON_HARDWARE_ADDRESS "86:17:5B:D5:A9:9A:E2:9C" // Hardware address of beacon DW1000 UWB chip
#define BEACON_SHORT_ADDRESS 0x1786                        // Short address of beacon, derived from first two bytes of BEACON_HARDWARE_ADDRESS
const uint8_t  NUM_RANGE_SAMPLES = 5; // Number of range samples to be recorded and averaged
float range_buf[NUM_RANGE_SAMPLES] = {0}; // Buffer to store range values for averaging, all zeroes by default
uint8_t range_buf_idx = 0; // Buffer index
bool beacon_activated = false; // Beacon DW1000 UWB chip connected

// Logging functions
inline void LOG_DEBUG(const String& msg) { Serial.print("[  DEBUG  ] "); Serial.println(msg); }
inline void LOG_INFO(const String& msg)  { Serial.print("[  INFO   ] "); Serial.println(msg); }
inline void LOG_WARN(const String& msg)  { Serial.print("[! WARN ! ] "); Serial.println(msg); }
inline void LOG_ERROR(const String& msg) { Serial.print("[** ERROR **] "); Serial.println(msg); }

void setup() {
    Serial.begin(9600); // Baud rate 9600 Bd
    delay(1000);

    SPI.begin(SPI_SCK, SPI_MISO, SPI_MOSI); // Initialize SPI bus, hardwired to oboard DW1000 UWB chip
    DW1000Ranging.initCommunication(PIN_RST, PIN_PS, PIN_IRQ); // Initialize DW1000 UWB chip
    DW1000.setAntennaDelay(DW1000_ANTENNA_DELAY); // Delay after UWB signal arrives or before it leaves, fine-tuned for accurate distance readings
    // Register callback functions
    DW1000Ranging.attachNewRange(newRange); // Called every time a complete ranging exchange between UWB chips occurs
    DW1000Ranging.attachNewDevice(newDevice); // Called when an unseen device joins the ranging exchange; only fires on initial detection of beacon
    DW1000Ranging.attachInactiveDevice(inactiveDevice); // Called when a device hasn't been heard from (lost connection, out of range)

    DW1000Ranging.useRangeFilter(USE_RANGE_FILTER); // Enable or disable builtin library range-smoothing feature
    DW1000Ranging.startAsTag(ROVER_HARDWARE_ADDRESS, DW1000.MODE_LONGDATA_RANGE_LOWPOWER); // Start UWB radio as a tag; long range low power mode
}

unsigned long last_measurement_millis = millis();
void loop() {
    DW1000Ranging.loop(); // Ranging library's main state machine, must be called repeatedly

    if (beacon_activated && millis() - last_measurement_millis > 1000) {
        LOG_DEBUG("Distance from BEACON: " + String(getRoverDistance()) + "m");
        last_measurement_millis = millis();
    }
}

/**
 * @brief Records a new range reading between onboard DW1000 UWB chip and connected device
 *
 * Range readings are added to `range_buf` to later be averaged for more accurate readings
*/
void newRange() {
    if (DW1000Ranging.getDistantDevice()->getShortAddress() != BEACON_SHORT_ADDRESS) return;
    range_buf[range_buf_idx] = DW1000Ranging.getDistantDevice()->getRange(); // Record range (meters), write to range buffer
    range_buf_idx = (range_buf_idx + 1) % NUM_RANGE_SAMPLES; // Increment index, overflows back to 0
}
/**
 * @brief Activates a new connected DW1000 UWB device
 *
 * @param device New DW1000 UWB device to be activated
*/
void newDevice(DW1000Device *device) {
    LOG_INFO("New device activated");
    if (device->getShortAddress() != BEACON_SHORT_ADDRESS) {
        LOG_WARN("New device not recognized as BEACON");
        return;
    }

    // Beacon activated
    LOG_INFO("BEACON activated");
    beacon_activated = true;
}
/**
 * @brief Deactivates a connected DW1000 UWB device
 *
 * Sets `beacon_activated` to `false` and fills `range_buf` with zeroes
 *
 * @param device Pointer to connected device
*/
void inactiveDevice(DW1000Device *device) {
    if (device->getShortAddress() != BEACON_SHORT_ADDRESS) {
        LOG_WARN("Unrecognized device disconected");
        return;
    }

    // Beacon deactivated
    beacon_activated = false;
    for (uint8_t i = 0; i < NUM_RANGE_SAMPLES; ++i) range_buf[i] = 0; // Zero out range buffer
    LOG_ERROR("BEACON disconnected");
}

/**
 * @brief Returns the distance between onboard DW1000 UWB chip and connected device (meters)
 * 
 * Returns the average value of the last `NUM_RANGE_SAMPLES` range readings
 *
 * @return Distance between rover and beacon if beacon is detected, `NAN` if beacon is not detected
 */
float getRoverDistance() {
    if (!beacon_activated) return NAN; // Beacon not detected, range buffer innacurate

    // Calculate average of range buffer
    float avg = 0;
    for (uint8_t i = 0; i < NUM_RANGE_SAMPLES; ++i) avg += range_buf[i];
    avg /= NUM_RANGE_SAMPLES;

    return avg;
}
