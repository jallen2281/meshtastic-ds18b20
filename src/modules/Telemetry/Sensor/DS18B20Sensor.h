#include "configuration.h"

#if !MESHTASTIC_EXCLUDE_ENVIRONMENTAL_SENSOR

#include "TelemetrySensor.h"
#include <DallasTemperature.h>
#include <DS2431.h>
#include <OneWire.h>
#include <vector>

// Struct to hold sensor reading with address
struct SensorReading {
    uint8_t address[8];
    float tempC;
    float tempF;
};

struct SensorAddress {
    uint8_t address[8];
};

class DS18B20Sensor : public TelemetrySensor {
  private:
    OneWire *oneWire;
    DallasTemperature *sensors;
    uint8_t pin;
    std::vector<SensorReading> sensorReadings;
    std::vector<SensorAddress> cachedTStringAddresses;
    std::vector<SensorAddress> cachedDs2431Uids;
    std::vector<uint8_t> cachedDs2431SensorCounts;
    bool hasCachedTStringAddresses = false;
    bool discoveredTString = false;

    void logRawOneWireDevices();
    bool readTStringAddresses(std::vector<SensorAddress> &addresses);
    bool readAddressesFromEeprom(const uint8_t ds2431Address[8], std::vector<SensorAddress> &addresses);
    bool isValidDs18b20Address(const uint8_t address[8]) const;
    void addReading(const uint8_t address[8], float tempC, float tempF);

  public:
    DS18B20Sensor();
    virtual int32_t runOnce() override;
    virtual void setup() override;
    virtual bool getMetrics(meshtastic_Telemetry *measurement) override;
    bool getMetricsForTString(uint8_t tStringIndex, meshtastic_Telemetry *measurement);
    uint8_t getTStringCount() const { return (uint8_t)cachedDs2431Uids.size(); }
};

#endif