#include "configuration.h"

#if !MESHTASTIC_EXCLUDE_ENVIRONMENTAL_SENSOR

#include "DS18B20Sensor.h"
#include "../mesh/generated/meshtastic/telemetry.pb.h"

namespace {
constexpr uint8_t kMaxTStringSensors = 6;
constexpr uint16_t kTStringAddressOffset = 8;
constexpr uint16_t kTStringAddressBytes = kMaxTStringSensors * 8;
constexpr uint8_t kDs18b20FamilyCode = 0x28;
// DS18B20 sentinel values that indicate bad/missing readings
constexpr float kDs18b20MaxValidTemp = 125.0f;  // DS18B20 specified max; all-bits-high error patterns exceed this at any resolution
constexpr float kDs18b20PowerOnReset = 85.0f;   // factory default value at power-on
}

DS18B20Sensor::DS18B20Sensor() : TelemetrySensor(meshtastic_TelemetrySensorType_DS18B20, "DS18B20"), oneWire(nullptr), sensors(nullptr)
{
    pin = PIN_WIRE_DS18B20;
}

void DS18B20Sensor::logRawOneWireDevices()
{
    if (!oneWire) {
        return;
    }

    uint8_t address[8];
    uint8_t foundCount = 0;

    oneWire->reset_search();
    while (oneWire->search(address)) {
        if (OneWire::crc8(address, 7) != address[7]) {
            LOG_WARN("OneWire device with invalid CRC on GPIO %u", pin);
            continue;
        }

        foundCount++;
        LOG_INFO("OneWire raw device %u on GPIO %u family 0x%02X addr %02X:%02X:%02X:%02X:%02X:%02X:%02X:%02X", foundCount,
                 pin, address[0], address[0], address[1], address[2], address[3], address[4], address[5], address[6],
                 address[7]);
    }
    oneWire->reset_search();

    if (foundCount == 0) {
        LOG_WARN("No raw OneWire devices found on GPIO %u", pin);
    }
}

int32_t DS18B20Sensor::runOnce()
{
    LOG_INFO("Init sensor: %s", sensorName);

    // Reassert the weak internal pull-up before each discovery pass.
    // OneWire devices still need a proper external pull-up, but this avoids leaving the pin floating.
    pinMode(pin, INPUT_PULLUP);

    if (!oneWire) {
        oneWire = new OneWire(pin);
    }

    if (!sensors) {
        sensors = new DallasTemperature(oneWire);
    }

    sensors->begin();
    // Parasite-power mode: reduce resolution to lower conversion current demand.
    // At 12-bit (750ms) a 4.7kΩ pull-up cannot supply enough current for multiple sensors.
    // 9-bit (93.75ms) cuts conversion time and peak current by ~8x.
    sensors->setResolution(9);
    logRawOneWireDevices();
    status = sensors->getDeviceCount() > 0;
    LOG_INFO("Detected %d DS18B20 sensors", sensors->getDeviceCount());
    LOG_INFO("Parasite power mode: %s", sensors->isParasitePowerMode() ? "YES" : "NO");

    if (!status) {
        LOG_DEBUG("No DS18B20 sensors found on initial scan; keeping sensor active for retry");
    } else {
        setup();
    }

    initialized = true;
    return DEFAULT_SENSOR_MINIMUM_WAIT_TIME_BETWEEN_READS;
}

void DS18B20Sensor::setup() {}

static bool isValidTempReading(float tempC)
{
    return tempC != DEVICE_DISCONNECTED_C && tempC != kDs18b20PowerOnReset && tempC <= kDs18b20MaxValidTemp;
}

bool DS18B20Sensor::isValidDs18b20Address(const uint8_t address[8]) const
{
    return address[0] == kDs18b20FamilyCode && OneWire::crc8(address, 7) == address[7];
}

void DS18B20Sensor::addReading(const uint8_t address[8], float tempC, float tempF)
{
    sensorReadings.push_back({});
    SensorReading &reading = sensorReadings.back();
    memcpy(reading.address, address, sizeof(reading.address));
    reading.tempC = tempC;
    reading.tempF = tempF;
}

bool DS18B20Sensor::readAddressesFromEeprom(const uint8_t ds2431Address[8], std::vector<SensorAddress> &addresses)
{
    if (ds2431Address[0] != DS2431::ONE_WIRE_FAMILY_CODE) {
        return false;
    }

    DS2431 eeprom(*oneWire);
    uint8_t buffer[kTStringAddressBytes] = {0};

    uint8_t addressCopy[8];
    memcpy(addressCopy, ds2431Address, sizeof(addressCopy));
    eeprom.begin(addressCopy);
    eeprom.read(kTStringAddressOffset, buffer, sizeof(buffer));

    bool looksBlank = true;
    for (uint16_t i = 0; i < sizeof(buffer); ++i) {
        if (buffer[i] != 0x00 && buffer[i] != 0xFF) {
            looksBlank = false;
            break;
        }
    }
    if (looksBlank) {
        LOG_WARN("DS2431 data appears blank, skipping EEPROM decode");
        return false;
    }

    for (uint8_t index = 0; index < kMaxTStringSensors; ++index) {
        SensorAddress sensorAddress = {};
        const uint8_t *storedAddress = buffer + (index * 8);

        for (uint8_t byteIndex = 0; byteIndex < 8; ++byteIndex) {
            sensorAddress.address[byteIndex] = storedAddress[7 - byteIndex];
        }

        if (!isValidDs18b20Address(sensorAddress.address)) {
            continue;
        }

        if (!sensors->isConnected(sensorAddress.address)) {
            LOG_WARN("DS2431 slot %u points to DS18B20 not present on bus, ignoring", index + 1);
            continue;
        }

        addresses.push_back(sensorAddress);
        LOG_DEBUG("DS2431 slot %u -> DS18B20 %02X:%02X:%02X:%02X:%02X:%02X:%02X:%02X", index + 1,
                  sensorAddress.address[0], sensorAddress.address[1], sensorAddress.address[2], sensorAddress.address[3],
                  sensorAddress.address[4], sensorAddress.address[5], sensorAddress.address[6], sensorAddress.address[7]);
    }

    return !addresses.empty();
}

bool DS18B20Sensor::readTStringAddresses(std::vector<SensorAddress> &addresses)
{
    // Use cache if available; never re-discover during telemetry cycles to avoid bus contention
    if (hasCachedTStringAddresses) {
        addresses = cachedTStringAddresses;
        return !cachedTStringAddresses.empty();
    }

    // If we've already attempted discovery and failed, return cached empty set
    if (discoveredTString) {
        return false;
    }

    uint8_t oneWireAddress[8];
    bool foundDs2431 = false;
    uint8_t ds2431Count = 0;

    cachedDs2431SensorCounts.clear();
    oneWire->reset_search();

    while (oneWire->search(oneWireAddress)) {
        if (OneWire::crc8(oneWireAddress, 7) != oneWireAddress[7]) {
            LOG_WARN("Skipping DS2431 with invalid CRC");
            continue;
        }

        if (oneWireAddress[0] != DS2431::ONE_WIRE_FAMILY_CODE) {
            continue;
        }

        uint8_t ds2431Address[8];
        memcpy(ds2431Address, oneWireAddress, sizeof(ds2431Address));

        foundDs2431 = true;
        ds2431Count++;
        LOG_INFO("Found DS2431 T-String EEPROM #%u at %02X:%02X:%02X:%02X:%02X:%02X:%02X:%02X", ds2431Count,
                 ds2431Address[0], ds2431Address[1], ds2431Address[2], ds2431Address[3],
                 ds2431Address[4], ds2431Address[5], ds2431Address[6], ds2431Address[7]);

        SensorAddress ds2431Uid;
        memcpy(ds2431Uid.address, ds2431Address, 8);
        cachedDs2431Uids.push_back(ds2431Uid);

        size_t countBefore = addresses.size();
        readAddressesFromEeprom(ds2431Address, addresses);
        cachedDs2431SensorCounts.push_back((uint8_t)(addresses.size() - countBefore));
    }

    oneWire->reset_search();

    discoveredTString = true;

    if (!foundDs2431) {
        LOG_DEBUG("No DS2431 EEPROM found on OneWire bus; falling back to direct DS18B20 scan");
    }

    if (!addresses.empty()) {
        cachedTStringAddresses = addresses;
        hasCachedTStringAddresses = true;
        LOG_INFO("T-String discovery complete: %u sensors across %u DS2431 chip(s)", (unsigned)addresses.size(), ds2431Count);
    } else {
        hasCachedTStringAddresses = false;
        cachedTStringAddresses.clear();
        cachedDs2431Uids.clear();
        cachedDs2431SensorCounts.clear();
    }

    return !addresses.empty();
}

bool DS18B20Sensor::getMetrics(meshtastic_Telemetry *measurement)
{
    if (!sensors) {
        LOG_DEBUG("DS18B20 metrics requested before sensor init completed");
        return false;
    }

    status = sensors->getDeviceCount() > 0;
    if (!status) {
        LOG_DEBUG("No DS18B20 sensors currently detected");
    }

    sensorReadings.clear();

    std::vector<SensorAddress> tStringAddresses;
    bool parasitePower = sensors->isParasitePowerMode();

    if (readTStringAddresses(tStringAddresses)) {
        for (uint8_t index = 0; index < tStringAddresses.size(); ++index) {
            float tempC;
            if (parasitePower) {
                // Convert one sensor at a time: peak draw is 1×1.5mA instead of N×1.5mA
                sensors->requestTemperaturesByAddress(tStringAddresses[index].address);
                tempC = sensors->getTempC(tStringAddresses[index].address);
            } else {
                if (index == 0) sensors->requestTemperatures();
                tempC = sensors->getTempC(tStringAddresses[index].address);
            }
            float tempF = (tempC * 9.0f / 5.0f) + 32.0f;

            if (!isValidTempReading(tempC)) {
                LOG_WARN("T-String DS18B20 slot %u invalid reading: %.4f°C", index + 1, tempC);
                continue;
            }

            addReading(tStringAddresses[index].address, tempC, tempF);
            LOG_INFO("T-String DS18B20 %u: %02X:%02X:%02X:%02X:%02X:%02X:%02X:%02X - Temp: %.2f°C / %.2f°F", index + 1,
                     tStringAddresses[index].address[0], tStringAddresses[index].address[1], tStringAddresses[index].address[2],
                     tStringAddresses[index].address[3], tStringAddresses[index].address[4], tStringAddresses[index].address[5],
                     tStringAddresses[index].address[6], tStringAddresses[index].address[7], tempC, tempF);
        }
    } else {
        sensors->requestTemperatures();
        uint8_t deviceCount = sensors->getDeviceCount();

        for (uint8_t i = 0; i < deviceCount; ++i) {
            uint8_t address[8];
            if (!sensors->getAddress(address, i)) {
                continue;
            }

            float tempC = parasitePower ? sensors->getTempC(address) : sensors->getTempCByIndex(i);
            float tempF = (tempC * 9.0f / 5.0f) + 32.0f;

            if (!isValidTempReading(tempC)) {
                continue;
            }

            addReading(address, tempC, tempF);
            LOG_INFO("DS18B20 sensor %d: %02X:%02X:%02X:%02X:%02X:%02X:%02X:%02X - Temp: %.2f°C / %.2f°F", i,
                     address[0], address[1], address[2], address[3], address[4], address[5], address[6], address[7], tempC,
                     tempF);
        }
    }

    if (!sensorReadings.empty()) {
        measurement->variant.environment_metrics.has_temperature1 = false;
        measurement->variant.environment_metrics.has_temperature2 = false;
        measurement->variant.environment_metrics.has_temperature3 = false;
        measurement->variant.environment_metrics.has_temperature4 = false;
        measurement->variant.environment_metrics.has_temperature5 = false;
        measurement->variant.environment_metrics.has_temperature6 = false;

        for (uint8_t i = 0; i < sensorReadings.size() && i < kMaxTStringSensors; ++i) {
            float tempToReport = sensorReadings[i].tempC;

            if (i == 0) {
                measurement->variant.environment_metrics.has_temperature1 = true;
                measurement->variant.environment_metrics.temperature1 = tempToReport;
            } else if (i == 1) {
                measurement->variant.environment_metrics.has_temperature2 = true;
                measurement->variant.environment_metrics.temperature2 = tempToReport;
            } else if (i == 2) {
                measurement->variant.environment_metrics.has_temperature3 = true;
                measurement->variant.environment_metrics.temperature3 = tempToReport;
            } else if (i == 3) {
                measurement->variant.environment_metrics.has_temperature4 = true;
                measurement->variant.environment_metrics.temperature4 = tempToReport;
            } else if (i == 4) {
                measurement->variant.environment_metrics.has_temperature5 = true;
                measurement->variant.environment_metrics.temperature5 = tempToReport;
            } else if (i == 5) {
                measurement->variant.environment_metrics.has_temperature6 = true;
                measurement->variant.environment_metrics.temperature6 = tempToReport;
            }
        }

        // Map DS2431 T-String UIDs into address slots (one per T-String, not per sensor)
        for (uint8_t i = 0; i < cachedDs2431Uids.size() && i < kMaxTStringSensors; ++i) {
            if (i == 0) {
                const uint8_t *uid = cachedDs2431Uids[i].address;
                snprintf(measurement->variant.environment_metrics.address1, 17,
                         "%02X%02X%02X%02X%02X%02X%02X%02X",
                         uid[0], uid[1], uid[2], uid[3], uid[4], uid[5], uid[6], uid[7]);
            } else if (i == 1) {
                measurement->variant.environment_metrics.has_address2 = true;
                memcpy(measurement->variant.environment_metrics.address2, cachedDs2431Uids[i].address, 8);
            } else if (i == 2) {
                measurement->variant.environment_metrics.has_address3 = true;
                memcpy(measurement->variant.environment_metrics.address3, cachedDs2431Uids[i].address, 8);
            } else if (i == 3) {
                measurement->variant.environment_metrics.has_address4 = true;
                memcpy(measurement->variant.environment_metrics.address4, cachedDs2431Uids[i].address, 8);
            } else if (i == 4) {
                measurement->variant.environment_metrics.has_address5 = true;
                memcpy(measurement->variant.environment_metrics.address5, cachedDs2431Uids[i].address, 8);
            } else if (i == 5) {
                measurement->variant.environment_metrics.has_address6 = true;
                memcpy(measurement->variant.environment_metrics.address6, cachedDs2431Uids[i].address, 8);
            }
        }

        measurement->variant.environment_metrics.has_temperature = true;
        measurement->variant.environment_metrics.has_multiple_temperatures = sensorReadings.size() > 1;

        if (sensorReadings.size() > 1) {
            float tempSum = 0.0f;
            for (uint8_t i = 0; i < sensorReadings.size(); ++i) {
                tempSum += sensorReadings[i].tempC;
            }
            measurement->variant.environment_metrics.temperature = tempSum / sensorReadings.size();
        } else {
            measurement->variant.environment_metrics.temperature = sensorReadings[0].tempC;
        }

        LOG_INFO("DS18B20 set: temp=%.2f, temp1=%.2f has_temp1=%d",
            measurement->variant.environment_metrics.temperature,
            measurement->variant.environment_metrics.temperature1,
            measurement->variant.environment_metrics.has_temperature1);

        return true;
    }

    return false;
}

bool DS18B20Sensor::getMetricsForTString(uint8_t tStringIndex, meshtastic_Telemetry *measurement)
{
    if (!sensors || tStringIndex >= cachedDs2431Uids.size() || tStringIndex >= cachedDs2431SensorCounts.size()) {
        return false;
    }

    uint8_t startIdx = 0;
    for (uint8_t i = 0; i < tStringIndex; i++) {
        startIdx += cachedDs2431SensorCounts[i];
    }
    uint8_t count = cachedDs2431SensorCounts[tStringIndex];

    if (startIdx >= cachedTStringAddresses.size() || count == 0) {
        return false;
    }

    float tempSum = 0;
    uint8_t validCount = 0;
    uint8_t slotNum = 0;

    for (uint8_t j = 0; j < count && (startIdx + j) < cachedTStringAddresses.size(); j++) {
        const uint8_t *addr = cachedTStringAddresses[startIdx + j].address;

        sensors->requestTemperaturesByAddress(addr);
        float tempC = sensors->getTempC(addr);

        if (!isValidTempReading(tempC)) {
            LOG_WARN("T-String %u sensor %u invalid: %.4f C", tStringIndex + 1, j + 1, tempC);
            continue;
        }

        LOG_INFO("T-String %u sensor %u: %.2f C", tStringIndex + 1, j + 1, tempC);
        tempSum += tempC;

        switch (slotNum++) {
            case 0:
                measurement->variant.environment_metrics.has_temperature1 = true;
                measurement->variant.environment_metrics.temperature1 = tempC;
                break;
            case 1:
                measurement->variant.environment_metrics.has_temperature2 = true;
                measurement->variant.environment_metrics.temperature2 = tempC;
                break;
            case 2:
                measurement->variant.environment_metrics.has_temperature3 = true;
                measurement->variant.environment_metrics.temperature3 = tempC;
                break;
            case 3:
                measurement->variant.environment_metrics.has_temperature4 = true;
                measurement->variant.environment_metrics.temperature4 = tempC;
                break;
            case 4:
                measurement->variant.environment_metrics.has_temperature5 = true;
                measurement->variant.environment_metrics.temperature5 = tempC;
                break;
            case 5:
                measurement->variant.environment_metrics.has_temperature6 = true;
                measurement->variant.environment_metrics.temperature6 = tempC;
                break;
        }
        validCount++;
    }

    if (validCount == 0) {
        return false;
    }

    measurement->variant.environment_metrics.has_temperature = true;
    measurement->variant.environment_metrics.temperature = tempSum / validCount;
    measurement->variant.environment_metrics.has_multiple_temperatures = validCount > 1;

    const uint8_t *uid = cachedDs2431Uids[tStringIndex].address;
    snprintf(measurement->variant.environment_metrics.address1, 17,
             "%02X%02X%02X%02X%02X%02X%02X%02X",
             uid[0], uid[1], uid[2], uid[3], uid[4], uid[5], uid[6], uid[7]);

    return true;
}

#endif
