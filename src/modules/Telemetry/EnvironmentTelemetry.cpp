#include "configuration.h"

#if !MESHTASTIC_EXCLUDE_ENVIRONMENTAL_SENSOR

#include "../mesh/generated/meshtastic/telemetry.pb.h"
#include "Default.h"
#include "EnvironmentTelemetry.h"
#include "MeshService.h"
#include "NodeDB.h"
#include "PowerFSM.h"
#include "RTC.h"
#include "Router.h"
#include "UnitConversions.h"
#include "main.h"
#include "power.h"
#include "sleep.h"
#include "target_specific.h"
#include <OLEDDisplay.h>
#include <OLEDDisplayUi.h>

#ifdef HAS_NEOPIXEL
#include <graphics/NeoPixel.h>
#endif

// Sensors
#if !MESHTASTIC_DS18B20_TEMP_ONLY
#include "Sensor/AHT10.h"
#include "Sensor/BME280Sensor.h"
#include "Sensor/BME680Sensor.h"
#include "Sensor/BMP085Sensor.h"
#include "Sensor/BMP280Sensor.h"
#include "Sensor/BMP3XXSensor.h"
#include "Sensor/CGRadSensSensor.h"
#include "Sensor/DFRobotLarkSensor.h"
#include "Sensor/LPS22HBSensor.h"
#include "Sensor/MCP9808Sensor.h"
#include "Sensor/MLX90632Sensor.h"
#include "Sensor/NAU7802Sensor.h"
#include "Sensor/OPT3001Sensor.h"
#include "Sensor/RCWL9620Sensor.h"
#include "Sensor/SHT31Sensor.h"
#include "Sensor/SHT4XSensor.h"
#include "Sensor/SHTC3Sensor.h"
#include "Sensor/T1000xSensor.h"
#include "Sensor/TSL2591Sensor.h"
#include "Sensor/VEML7700Sensor.h"
#endif
#include "Sensor/DS18B20Sensor.h"

#if !MESHTASTIC_DS18B20_TEMP_ONLY
BMP085Sensor bmp085Sensor;
BMP280Sensor bmp280Sensor;
BME280Sensor bme280Sensor;
BME680Sensor bme680Sensor;
MCP9808Sensor mcp9808Sensor;
SHTC3Sensor shtc3Sensor;
LPS22HBSensor lps22hbSensor;
SHT31Sensor sht31Sensor;
VEML7700Sensor veml7700Sensor;
TSL2591Sensor tsl2591Sensor;
OPT3001Sensor opt3001Sensor;
SHT4XSensor sht4xSensor;
RCWL9620Sensor rcwl9620Sensor;
AHT10Sensor aht10Sensor;
MLX90632Sensor mlx90632Sensor;
DFRobotLarkSensor dfRobotLarkSensor;
NAU7802Sensor nau7802Sensor;
BMP3XXSensor bmp3xxSensor;
#ifdef T1000X_SENSOR_EN
T1000xSensor t1000xSensor;
#endif
CGRadSensSensor cgRadSens;
#endif
DS18B20Sensor ds18b20Sensor;

#define FAILED_STATE_SENSOR_READ_MULTIPLIER 10
#define DISPLAY_RECEIVEID_MEASUREMENTS_ON_SCREEN true

#ifndef USERPREFS_TEMP_LED_COLD_C
#define USERPREFS_TEMP_LED_COLD_C -10.0f
#endif

#ifndef USERPREFS_TEMP_LED_HOT_C
#define USERPREFS_TEMP_LED_HOT_C 40.0f
#endif

#ifndef USERPREFS_TEMP_LED_BRIGHTNESS
#define USERPREFS_TEMP_LED_BRIGHTNESS 50
#endif

#ifndef USERPREFS_NEOPIXEL_BOOT_TEST
#define USERPREFS_NEOPIXEL_BOOT_TEST 1
#endif

namespace {
#ifdef HAS_NEOPIXEL
float getRuntimeColdSetpointC()
{
#if MESHTASTIC_DS18B20_TEMP_ONLY
    // Variant override: use Ambient Lighting blue channel as encoded cold setpoint.
    // Encoding: value = tempC + 40, so 0 keeps compile-time default.
    if (moduleConfig.ambient_lighting.blue != 0) {
        return static_cast<float>(moduleConfig.ambient_lighting.blue) - 40.0f;
    }
#endif
    return USERPREFS_TEMP_LED_COLD_C;
}

float getRuntimeHotSetpointC()
{
#if MESHTASTIC_DS18B20_TEMP_ONLY
    // Variant override: use Ambient Lighting red channel as encoded hot setpoint.
    // Encoding: value = tempC + 40, so 0 keeps compile-time default.
    if (moduleConfig.ambient_lighting.red != 0) {
        return static_cast<float>(moduleConfig.ambient_lighting.red) - 40.0f;
    }
#endif
    return USERPREFS_TEMP_LED_HOT_C;
}
#endif

#ifdef HAS_NEOPIXEL
void runNeoPixelBootDataLineTest()
{
    if (USERPREFS_NEOPIXEL_BOOT_TEST == 0) {
        return;
    }

    struct Step {
        uint8_t r;
        uint8_t g;
        uint8_t b;
        const char *name;
    };

    static const Step steps[] = {
        {255, 255, 255, "white"},
        {255, 0, 0, "red"},
        {0, 255, 0, "green"},
        {0, 0, 255, "blue"},
        {128, 0, 128, "purple"},
        {0, 0, 0, "off"},
    };

    LOG_INFO("NeoPixel: running boot signal test on GPIO %d", NEOPIXEL_DATA);
    pixels.setBrightness(USERPREFS_TEMP_LED_BRIGHTNESS);
    for (uint8_t i = 0; i < (sizeof(steps) / sizeof(steps[0])); ++i) {
        pixels.fill(pixels.Color(steps[i].r, steps[i].g, steps[i].b), 0, NEOPIXEL_COUNT);
        pixels.show();
        delayMicroseconds(350);
        pinMode(NEOPIXEL_DATA, OUTPUT);
        digitalWrite(NEOPIXEL_DATA, HIGH);
        LOG_INFO("NeoPixel test step: %s", steps[i].name);
        delay(250);
    }
}
#endif

void updateTemperatureLedColor(const meshtastic_EnvironmentMetrics &metrics)
{
#ifdef HAS_NEOPIXEL
    if (!metrics.has_temperature) {
        return;
    }

    // Temperature is stored in Celsius. Map cold..hot to blue->green->red.
    const float coldC = getRuntimeColdSetpointC();
    const float hotC = getRuntimeHotSetpointC();
    if (hotC <= coldC) {
        return;
    }

    const float t = metrics.temperature;
    float clamped = t;
    if (clamped < coldC)
        clamped = coldC;
    if (clamped > hotC)
        clamped = hotC;

    const float span = hotC - coldC;
    const float midpoint = coldC + (span * 0.5f);

    uint8_t red = 0;
    uint8_t green = 0;
    uint8_t blue = 0;

    if (clamped <= midpoint) {
        const float ratio = (clamped - coldC) / (midpoint - coldC); // cold..mid -> 0..1
        green = static_cast<uint8_t>(255.0f * ratio);
        blue = static_cast<uint8_t>(255.0f * (1.0f - ratio));
    } else {
        const float ratio = (clamped - midpoint) / (hotC - midpoint); // mid..hot -> 0..1
        red = static_cast<uint8_t>(255.0f * ratio);
        green = static_cast<uint8_t>(255.0f * (1.0f - ratio));
    }

    if (moduleConfig.ambient_lighting.current == 0) {
        moduleConfig.ambient_lighting.current = USERPREFS_TEMP_LED_BRIGHTNESS;
    }

    static uint8_t lastRed = 0xFF;
    static uint8_t lastGreen = 0xFF;
    static uint8_t lastBlue = 0xFF;
    static uint8_t lastBrightness = 0xFF;

    // Avoid unnecessary NeoPixel refreshes to reduce power transients on weak supplies.
    if (red != lastRed || green != lastGreen || blue != lastBlue || moduleConfig.ambient_lighting.current != lastBrightness) {
        pixels.setBrightness(moduleConfig.ambient_lighting.current);
        pixels.fill(pixels.Color(red, green, blue), 0, NEOPIXEL_COUNT);
        pixels.show();
        // nRF52 NeoPixel uses I2S DMA; after show() the pin may not be OUTPUT.
        // Re-assert OUTPUT HIGH to re-enable 3V3_S rail (PIN_3V3_EN shares GPIO 34).
        delayMicroseconds(350);
        pinMode(NEOPIXEL_DATA, OUTPUT);
        digitalWrite(NEOPIXEL_DATA, HIGH);
        lastRed = red;
        lastGreen = green;
        lastBlue = blue;
        lastBrightness = moduleConfig.ambient_lighting.current;
    }
#else
    (void)metrics;
#endif
}
} // namespace

#include "graphics/ScreenFonts.h"
#include <Throttle.h>

int32_t EnvironmentTelemetryModule::runOnce()
{
    LOG_DEBUG("EnvironmentTelemetryModule::runOnce() called");
    if (sleepOnNextExecution == true) {
        sleepOnNextExecution = false;
        uint32_t nightyNightMs = Default::getConfiguredOrDefaultMs(moduleConfig.telemetry.environment_update_interval,
                                                                   default_telemetry_broadcast_interval_secs);
        LOG_DEBUG("Sleep for %ims, then awake to send metrics again", nightyNightMs);
        doDeepSleep(nightyNightMs, true, false);
    }

    uint32_t result = UINT32_MAX;
    /*
        Uncomment the preferences below if you want to use the module
        without having to configure it from the PythonAPI or WebUI.
    */

    // Force telemetry on so DS18B20 sensors are always discovered and read
    moduleConfig.telemetry.environment_measurement_enabled = 1;
    moduleConfig.telemetry.environment_screen_enabled = 1;

    if (!(moduleConfig.telemetry.environment_measurement_enabled || moduleConfig.telemetry.environment_screen_enabled)) {
        // If this module is not enabled, and the user doesn't want the display screen don't waste any OSThread time on it
        return disable();
    }

    if (firstTime) {
        // This is the first time the OSThread library has called this function, so do some setup
        firstTime = 0;

#ifdef HAS_NEOPIXEL
        // Init NeoPixel immediately at boot with a "waiting for sensors" purple indicator
        pixels.begin();
        // pixels.begin() drives data pin LOW which disables 3V3_S rail (PIN_3V3_EN shares GPIO 34).
        // Re-assert OUTPUT HIGH immediately so peripherals keep power.
        pinMode(NEOPIXEL_DATA, OUTPUT);
        digitalWrite(NEOPIXEL_DATA, HIGH);
        pixels.setBrightness(USERPREFS_TEMP_LED_BRIGHTNESS);
        pixels.fill(pixels.Color(128, 0, 128), 0, NEOPIXEL_COUNT);
        pixels.show();
        // nRF52 NeoPixel uses I2S DMA; after show() the pin may not be OUTPUT.
        // Re-assert OUTPUT HIGH to re-enable 3V3_S rail (PIN_3V3_EN shares GPIO 34).
        delayMicroseconds(350);
        pinMode(NEOPIXEL_DATA, OUTPUT);
        digitalWrite(NEOPIXEL_DATA, HIGH);
        runNeoPixelBootDataLineTest();
        LOG_INFO("NeoPixel: boot init on GPIO %d, showing purple (waiting for sensors)", NEOPIXEL_DATA);
#endif

        if (moduleConfig.telemetry.environment_measurement_enabled) {
            LOG_INFO("Environment Telemetry: init");
            // it's possible to have this module enabled, only for displaying values on the screen.
            // therefore, we should only enable the sensor loop if measurement is also enabled
#ifdef T1000X_SENSOR_EN
            result = t1000xSensor.runOnce();
#else
            if (dfRobotLarkSensor.hasSensor())
                result = dfRobotLarkSensor.runOnce();
            if (bmp085Sensor.hasSensor())
                result = bmp085Sensor.runOnce();
            if (bmp280Sensor.hasSensor())
                result = bmp280Sensor.runOnce();
            if (bme280Sensor.hasSensor())
                result = bme280Sensor.runOnce();
            if (bmp3xxSensor.hasSensor())
                result = bmp3xxSensor.runOnce();
            if (bme680Sensor.hasSensor())
                result = bme680Sensor.runOnce();
            if (mcp9808Sensor.hasSensor())
                result = mcp9808Sensor.runOnce();
            if (shtc3Sensor.hasSensor())
                result = shtc3Sensor.runOnce();
            if (lps22hbSensor.hasSensor())
                result = lps22hbSensor.runOnce();
            if (sht31Sensor.hasSensor())
                result = sht31Sensor.runOnce();
            if (sht4xSensor.hasSensor())
                result = sht4xSensor.runOnce();
            if (ina219Sensor.hasSensor())
                result = ina219Sensor.runOnce();
            if (ina260Sensor.hasSensor())
                result = ina260Sensor.runOnce();
            if (ina3221Sensor.hasSensor())
                result = ina3221Sensor.runOnce();
            if (veml7700Sensor.hasSensor())
                result = veml7700Sensor.runOnce();
            if (tsl2591Sensor.hasSensor())
                result = tsl2591Sensor.runOnce();
            if (opt3001Sensor.hasSensor())
                result = opt3001Sensor.runOnce();
            if (rcwl9620Sensor.hasSensor())
                result = rcwl9620Sensor.runOnce();
            if (aht10Sensor.hasSensor())
                result = aht10Sensor.runOnce();
            if (mlx90632Sensor.hasSensor())
                result = mlx90632Sensor.runOnce();
            if (nau7802Sensor.hasSensor())
                result = nau7802Sensor.runOnce();
            if (max17048Sensor.hasSensor())
                result = max17048Sensor.runOnce();
            if (cgRadSens.hasSensor())
                result = cgRadSens.runOnce();
            // DS18B20 is OneWire, not found by I2C scan. Register it manually so hasSensor() returns true.
            nodeTelemetrySensorsMap[meshtastic_TelemetrySensorType_DS18B20].first = PIN_WIRE_DS18B20;
            result = ds18b20Sensor.runOnce();
#endif
        }
        return result;
    } else {
        // if we somehow got to a second run of this module with measurement disabled, then just wait forever
        if (!moduleConfig.telemetry.environment_measurement_enabled) {
            return disable();
        } else {
            if (bme680Sensor.hasSensor())
                result = bme680Sensor.runTrigger();
        }

        if (((lastSentToMesh == 0) ||
             !Throttle::isWithinTimespanMs(lastSentToMesh, Default::getConfiguredOrDefaultMsScaled(
                                                               moduleConfig.telemetry.environment_update_interval,
                                                               default_telemetry_broadcast_interval_secs, numOnlineNodes))) &&
            airTime->isTxAllowedChannelUtil(config.device.role != meshtastic_Config_DeviceConfig_Role_SENSOR) &&
            airTime->isTxAllowedAirUtil()) {
            LOG_INFO("EnvironmentTelemetry: Conditions met, calling sendTelemetry() to mesh");
            sendTelemetry();
            lastSentToMesh = millis();
        } else if (((lastSentToPhone == 0) || !Throttle::isWithinTimespanMs(lastSentToPhone, sendToPhoneIntervalMs)) &&
                   (service->isToPhoneQueueEmpty())) {
            // Just send to phone when it's not our time to send to mesh yet
            // Only send while queue is empty (phone assumed connected)
            LOG_INFO("EnvironmentTelemetry: Sending to phone only");
            sendTelemetry(NODENUM_BROADCAST, true);
            lastSentToPhone = millis();
        } else {
            LOG_DEBUG("EnvironmentTelemetry: Skipping send - throttled or not allowed");
        }
    }
    return min(sendToPhoneIntervalMs, result);
}

bool EnvironmentTelemetryModule::wantUIFrame()
{
    return moduleConfig.telemetry.environment_screen_enabled;
}

void EnvironmentTelemetryModule::drawFrame(OLEDDisplay *display, OLEDDisplayUiState *state, int16_t x, int16_t y)
{
    display->setTextAlignment(TEXT_ALIGN_LEFT);
    display->setFont(FONT_SMALL);

    if (lastMeasurementPacket == nullptr) {
        // If there's no valid packet, display "Environment"
        display->drawString(x, y, "Environment");
        display->drawString(x, y += _fontHeight(FONT_SMALL), "No measurement");
        return;
    }

    // Decode the last measurement packet
    meshtastic_Telemetry lastMeasurement;
    uint32_t agoSecs = service->GetTimeSinceMeshPacket(lastMeasurementPacket);
    const char *lastSender = getSenderShortName(*lastMeasurementPacket);

    const meshtastic_Data &p = lastMeasurementPacket->decoded;
    if (!pb_decode_from_bytes(p.payload.bytes, p.payload.size, &meshtastic_Telemetry_msg, &lastMeasurement)) {
        display->drawString(x, y, "Measurement Error");
        LOG_ERROR("Unable to decode last packet");
        return;
    }

    // Display "Env. From: ..." on its own
    display->drawString(x, y, "Env. From: " + String(lastSender) + "(" + String(agoSecs) + "s)");

    if (lastMeasurement.variant.environment_metrics.has_temperature ||
        lastMeasurement.variant.environment_metrics.has_relative_humidity) {
        String last_temp = String(lastMeasurement.variant.environment_metrics.temperature, 0) + "°C";
        if (moduleConfig.telemetry.environment_display_fahrenheit) {
            last_temp =
                String(UnitConversions::CelsiusToFahrenheit(lastMeasurement.variant.environment_metrics.temperature), 0) + "°F";
        }

        // Continue with the remaining details
            display->drawString(x, y += _fontHeight(FONT_SMALL),
                                "Avg Temp: " + last_temp);
    }

    if (lastMeasurement.variant.environment_metrics.has_multiple_temperatures) {
        if (lastMeasurement.variant.environment_metrics.has_temperature1) {
            String temp1 = String(lastMeasurement.variant.environment_metrics.temperature1, 1) + "°C";
            if (moduleConfig.telemetry.environment_display_fahrenheit)
                temp1 = String(UnitConversions::CelsiusToFahrenheit(lastMeasurement.variant.environment_metrics.temperature1), 1) +
                        "°F";
            display->drawString(x, y += _fontHeight(FONT_SMALL)-1, "T1: " + temp1);
        }
        if (lastMeasurement.variant.environment_metrics.has_temperature2) {
            String temp2 = String(lastMeasurement.variant.environment_metrics.temperature2, 1) + "°C";
            if (moduleConfig.telemetry.environment_display_fahrenheit)
                temp2 = String(UnitConversions::CelsiusToFahrenheit(lastMeasurement.variant.environment_metrics.temperature2), 1) +
                        "°F";
            display->drawString(x, y += _fontHeight(FONT_SMALL)-1, "T2: " + temp2);
        }
        if (lastMeasurement.variant.environment_metrics.has_temperature3) {
            String temp3 = String(lastMeasurement.variant.environment_metrics.temperature3, 1) + "°C";
            if (moduleConfig.telemetry.environment_display_fahrenheit)
                temp3 = String(UnitConversions::CelsiusToFahrenheit(lastMeasurement.variant.environment_metrics.temperature3), 1) +
                        "°F";
            display->drawString(x, y += _fontHeight(FONT_SMALL)-1, "T3: " + temp3);
        }
        if (lastMeasurement.variant.environment_metrics.has_temperature4) {
            String temp4 = String(lastMeasurement.variant.environment_metrics.temperature4, 1) + "°C";
            if (moduleConfig.telemetry.environment_display_fahrenheit)
                temp4 = String(UnitConversions::CelsiusToFahrenheit(lastMeasurement.variant.environment_metrics.temperature4), 1) +
                        "°F";
            display->drawString(x += 55, y -= 2*(_fontHeight(FONT_SMALL)-1), "T4: " + temp4);
        }
        if (lastMeasurement.variant.environment_metrics.has_temperature5) {
            String temp5 = String(lastMeasurement.variant.environment_metrics.temperature5, 1) + "°C";
            if (moduleConfig.telemetry.environment_display_fahrenheit)
                temp5 = String(UnitConversions::CelsiusToFahrenheit(lastMeasurement.variant.environment_metrics.temperature5), 1) +
                        "°F";
            display->drawString(x, y += _fontHeight(FONT_SMALL)-1, "T5: " + temp5);
        }
        if (lastMeasurement.variant.environment_metrics.has_temperature6) {
            String temp6 = String(lastMeasurement.variant.environment_metrics.temperature6, 1) + "°C";
            if (moduleConfig.telemetry.environment_display_fahrenheit)
                temp6 = String(UnitConversions::CelsiusToFahrenheit(lastMeasurement.variant.environment_metrics.temperature6), 1) +
                        "°F";
            display->drawString(x, y += _fontHeight(FONT_SMALL)-1, "T6: " + temp6);
        }
    }

    if (lastMeasurement.variant.environment_metrics.barometric_pressure != 0) {
        display->drawString(x, y += _fontHeight(FONT_SMALL),
                            "Press: " + String(lastMeasurement.variant.environment_metrics.barometric_pressure, 0) + "hPA");
    }

    if (lastMeasurement.variant.environment_metrics.voltage != 0) {
        display->drawString(x, y += _fontHeight(FONT_SMALL),
                            "Volt/Cur: " + String(lastMeasurement.variant.environment_metrics.voltage, 0) + "V / " +
                                String(lastMeasurement.variant.environment_metrics.current, 0) + "mA");
    }

    if (lastMeasurement.variant.environment_metrics.iaq != 0) {
        display->drawString(x, y += _fontHeight(FONT_SMALL), "IAQ: " + String(lastMeasurement.variant.environment_metrics.iaq));
    }

    if (lastMeasurement.variant.environment_metrics.distance != 0)
        display->drawString(x, y += _fontHeight(FONT_SMALL),
                            "Water Level: " + String(lastMeasurement.variant.environment_metrics.distance, 0) + "mm");

    if (lastMeasurement.variant.environment_metrics.weight != 0)
        display->drawString(x, y += _fontHeight(FONT_SMALL),
                            "Weight: " + String(lastMeasurement.variant.environment_metrics.weight, 0) + "kg");

    if (lastMeasurement.variant.environment_metrics.radiation != 0)
        display->drawString(x, y += _fontHeight(FONT_SMALL),
                            "Rad: " + String(lastMeasurement.variant.environment_metrics.radiation, 2) + "µR/h");
}

bool EnvironmentTelemetryModule::handleReceivedProtobuf(const meshtastic_MeshPacket &mp, meshtastic_Telemetry *t)
{
    if (t->which_variant == meshtastic_Telemetry_environment_metrics_tag) {
#ifdef DEBUG_PORT
        const char *sender = getSenderShortName(mp);

        LOG_INFO("(Received from %s): barometric_pressure=%f, current=%f, gas_resistance=%f, relative_humidity=%f, "
                 "temperature=%f",
                 sender, t->variant.environment_metrics.barometric_pressure, t->variant.environment_metrics.current,
                 t->variant.environment_metrics.gas_resistance, t->variant.environment_metrics.relative_humidity,
                 t->variant.environment_metrics.temperature);
        LOG_INFO("(Received from %s): voltage=%f, IAQ=%d, distance=%f, lux=%f", sender, t->variant.environment_metrics.voltage,
                 t->variant.environment_metrics.iaq, t->variant.environment_metrics.distance, t->variant.environment_metrics.lux);

        // Log individual temperature readings if available
        if (t->variant.environment_metrics.has_temperature1 || t->variant.environment_metrics.has_temperature2 ||
            t->variant.environment_metrics.has_temperature3 || t->variant.environment_metrics.has_temperature4 ||
            t->variant.environment_metrics.has_temperature5 || t->variant.environment_metrics.has_temperature6) {
            LOG_INFO("(Received from %s): Multiple sensors - temp1=%f, temp2=%f, temp3=%f", sender,
                     t->variant.environment_metrics.temperature1, t->variant.environment_metrics.temperature2,
                     t->variant.environment_metrics.temperature3);
            LOG_INFO("(Received from %s): Multiple sensors - temp4=%f, temp5=%f, temp6=%f", sender,
                     t->variant.environment_metrics.temperature4, t->variant.environment_metrics.temperature5,
                     t->variant.environment_metrics.temperature6);
        }

        LOG_INFO("(Received from %s): wind speed=%fm/s, direction=%d degrees, weight=%fkg", sender,
                 t->variant.environment_metrics.wind_speed, t->variant.environment_metrics.wind_direction,
                 t->variant.environment_metrics.weight);

        LOG_INFO("(Received from %s): radiation=%fµR/h", sender, t->variant.environment_metrics.radiation);
        
        // Always log individual temperature values to debug
        LOG_INFO("(Received from %s): temp1=%.2f has_temp1=%d", sender,
                 t->variant.environment_metrics.temperature1,
                 t->variant.environment_metrics.has_temperature1);

#endif
        // release previous packet before occupying a new spot
        if (lastMeasurementPacket != nullptr)
            packetPool.release(lastMeasurementPacket);

        lastMeasurementPacket = packetPool.allocCopy(mp);
    }

    return false; // Let others look at this message also if they want
}

bool EnvironmentTelemetryModule::getEnvironmentTelemetry(meshtastic_Telemetry *m)
{
    bool valid = true;
    bool hasSensor = false;
    m->time = getTime();
    m->which_variant = meshtastic_Telemetry_environment_metrics_tag;
    m->variant.environment_metrics = meshtastic_EnvironmentMetrics_init_zero;

#ifdef T1000X_SENSOR_EN // add by WayenWeng
    valid = valid && t1000xSensor.getMetrics(m);
    hasSensor = true;
#else
#if MESHTASTIC_DS18B20_TEMP_ONLY
    if (ds18b20Sensor.hasSensor()) {
        valid = valid && ds18b20Sensor.getMetrics(m);
        hasSensor = true;
    }
#else
    if (dfRobotLarkSensor.hasSensor()) {
        valid = valid && dfRobotLarkSensor.getMetrics(m);
        hasSensor = true;
    }
    if (sht31Sensor.hasSensor()) {
        valid = valid && sht31Sensor.getMetrics(m);
        hasSensor = true;
    }
    if (sht4xSensor.hasSensor()) {
        valid = valid && sht4xSensor.getMetrics(m);
        hasSensor = true;
    }
    if (lps22hbSensor.hasSensor()) {
        valid = valid && lps22hbSensor.getMetrics(m);
        hasSensor = true;
    }
    if (shtc3Sensor.hasSensor()) {
        valid = valid && shtc3Sensor.getMetrics(m);
        hasSensor = true;
    }
    if (bmp085Sensor.hasSensor()) {
        valid = valid && bmp085Sensor.getMetrics(m);
        hasSensor = true;
    }
    if (bmp280Sensor.hasSensor()) {
        valid = valid && bmp280Sensor.getMetrics(m);
        hasSensor = true;
    }
    if (bme280Sensor.hasSensor()) {
        valid = valid && bme280Sensor.getMetrics(m);
        hasSensor = true;
    }
    if (bmp3xxSensor.hasSensor()) {
        valid = valid && bmp3xxSensor.getMetrics(m);
        hasSensor = true;
    }
    if (bme680Sensor.hasSensor()) {
        valid = valid && bme680Sensor.getMetrics(m);
        hasSensor = true;
    }
    if (mcp9808Sensor.hasSensor()) {
        valid = valid && mcp9808Sensor.getMetrics(m);
        hasSensor = true;
    }
    if (ina219Sensor.hasSensor()) {
        valid = valid && ina219Sensor.getMetrics(m);
        hasSensor = true;
    }
    if (ina260Sensor.hasSensor()) {
        valid = valid && ina260Sensor.getMetrics(m);
        hasSensor = true;
    }
    if (ina3221Sensor.hasSensor()) {
        valid = valid && ina3221Sensor.getMetrics(m);
        hasSensor = true;
    }
    if (veml7700Sensor.hasSensor()) {
        valid = valid && veml7700Sensor.getMetrics(m);
        hasSensor = true;
    }
    if (tsl2591Sensor.hasSensor()) {
        valid = valid && tsl2591Sensor.getMetrics(m);
        hasSensor = true;
    }
    if (opt3001Sensor.hasSensor()) {
        valid = valid && opt3001Sensor.getMetrics(m);
        hasSensor = true;
    }
    if (mlx90632Sensor.hasSensor()) {
        valid = valid && mlx90632Sensor.getMetrics(m);
        hasSensor = true;
    }
    if (rcwl9620Sensor.hasSensor()) {
        valid = valid && rcwl9620Sensor.getMetrics(m);
        hasSensor = true;
    }
    if (nau7802Sensor.hasSensor()) {
        valid = valid && nau7802Sensor.getMetrics(m);
        hasSensor = true;
    }
    if (aht10Sensor.hasSensor()) {
        if (!bmp280Sensor.hasSensor() && !bmp3xxSensor.hasSensor()) {
            valid = valid && aht10Sensor.getMetrics(m);
            hasSensor = true;
        } else if (bmp280Sensor.hasSensor()) {
            // prefer bmp280 temp if both sensors are present, fetch only humidity
            meshtastic_Telemetry m_ahtx = meshtastic_Telemetry_init_zero;
            LOG_INFO("AHTX0+BMP280 module detected: using temp from BMP280 and humy from AHTX0");
            aht10Sensor.getMetrics(&m_ahtx);
            m->variant.environment_metrics.relative_humidity = m_ahtx.variant.environment_metrics.relative_humidity;
            m->variant.environment_metrics.has_relative_humidity = m_ahtx.variant.environment_metrics.has_relative_humidity;
        } else {
            // prefer bmp3xx temp if both sensors are present, fetch only humidity
            meshtastic_Telemetry m_ahtx = meshtastic_Telemetry_init_zero;
            LOG_INFO("AHTX0+BMP3XX module detected: using temp from BMP3XX and humy from AHTX0");
            aht10Sensor.getMetrics(&m_ahtx);
            m->variant.environment_metrics.relative_humidity = m_ahtx.variant.environment_metrics.relative_humidity;
            m->variant.environment_metrics.has_relative_humidity = m_ahtx.variant.environment_metrics.has_relative_humidity;
        }
    }
    if (max17048Sensor.hasSensor()) {
        valid = valid && max17048Sensor.getMetrics(m);
        hasSensor = true;
    }
    if (cgRadSens.hasSensor()) {
        valid = valid && cgRadSens.getMetrics(m);
        hasSensor = true;
    }
    if (ds18b20Sensor.hasSensor()) {
        valid = valid && ds18b20Sensor.getMetrics(m);
	    hasSensor = true;
    }
#endif

#endif
    return valid && hasSensor;
}

meshtastic_MeshPacket *EnvironmentTelemetryModule::allocReply()
{
    if (currentRequest) {
        auto req = *currentRequest;
        const auto &p = req.decoded;
        meshtastic_Telemetry scratch;
        meshtastic_Telemetry *decoded = NULL;
        memset(&scratch, 0, sizeof(scratch));
        if (pb_decode_from_bytes(p.payload.bytes, p.payload.size, &meshtastic_Telemetry_msg, &scratch)) {
            decoded = &scratch;
        } else {
            LOG_ERROR("Error decoding EnvironmentTelemetry module!");
            return NULL;
        }
        // Check for a request for environment metrics
        if (decoded->which_variant == meshtastic_Telemetry_environment_metrics_tag) {
            meshtastic_Telemetry m = meshtastic_Telemetry_init_zero;
            if (getEnvironmentTelemetry(&m)) {
                LOG_INFO("Environment telemetry reply to request");
                return allocDataProtobuf(m);
            } else {
                return NULL;
            }
        }
    }
    return NULL;
}

bool EnvironmentTelemetryModule::sendTelemetry(NodeNum dest, bool phoneOnly)
{
    meshtastic_Telemetry m = meshtastic_Telemetry_init_zero;
    m.which_variant = meshtastic_Telemetry_environment_metrics_tag;
    m.time = getTime();
#ifdef T1000X_SENSOR_EN
    if (t1000xSensor.getMetrics(&m)) {
#else
    if (getEnvironmentTelemetry(&m)) {
#endif
        LOG_INFO("Send: barometric_pressure=%f, current=%f, gas_resistance=%f, relative_humidity=%f, temperature=%f",
                 m.variant.environment_metrics.barometric_pressure, m.variant.environment_metrics.current,
                 m.variant.environment_metrics.gas_resistance, m.variant.environment_metrics.relative_humidity,
                 m.variant.environment_metrics.temperature);
        LOG_INFO("Send: voltage=%f, IAQ=%d, distance=%f, lux=%f", m.variant.environment_metrics.voltage,
                 m.variant.environment_metrics.iaq, m.variant.environment_metrics.distance, m.variant.environment_metrics.lux);

        // Log individual temperature readings if available
        if (m.variant.environment_metrics.has_temperature1 || m.variant.environment_metrics.has_temperature2 ||
            m.variant.environment_metrics.has_temperature3 || m.variant.environment_metrics.has_temperature4 ||
            m.variant.environment_metrics.has_temperature5 || m.variant.environment_metrics.has_temperature6) {
            LOG_INFO("Send: Multiple sensors - temp1=%f, temp2=%f, temp3=%f",
                     m.variant.environment_metrics.temperature1, m.variant.environment_metrics.temperature2,
                     m.variant.environment_metrics.temperature3);
            LOG_INFO("Send: Multiple sensors - temp4=%f, temp5=%f, temp6=%f",
                     m.variant.environment_metrics.temperature4, m.variant.environment_metrics.temperature5,
                     m.variant.environment_metrics.temperature6);
        }

        LOG_INFO("Send: wind speed=%fm/s, direction=%d degrees, weight=%fkg", m.variant.environment_metrics.wind_speed,
                 m.variant.environment_metrics.wind_direction, m.variant.environment_metrics.weight);

        LOG_INFO("Send: radiation=%fµR/h", m.variant.environment_metrics.radiation);
        
        LOG_INFO("Before encode - temp1=%.2f has_temp1=%d", m.variant.environment_metrics.temperature1, 
                 m.variant.environment_metrics.has_temperature1);

        updateTemperatureLedColor(m.variant.environment_metrics);

        sensor_read_error_count = 0;

        meshtastic_MeshPacket *p = allocDataProtobuf(m);
        p->to = dest;
        p->decoded.want_response = false;
        if (config.device.role == meshtastic_Config_DeviceConfig_Role_SENSOR)
            p->priority = meshtastic_MeshPacket_Priority_RELIABLE;
        else
            p->priority = meshtastic_MeshPacket_Priority_BACKGROUND;
        // release previous packet before occupying a new spot
        if (lastMeasurementPacket != nullptr)
            packetPool.release(lastMeasurementPacket);

        lastMeasurementPacket = packetPool.allocCopy(*p);
        if (phoneOnly) {
            LOG_INFO("Send packet to phone");
            service->sendToPhone(p);
        } else {
            LOG_INFO("Send packet to mesh - calling service->sendToMesh()");
            service->sendToMesh(p, RX_SRC_LOCAL, true);
            LOG_INFO("Packet sent to mesh, returning");

            if (config.device.role == meshtastic_Config_DeviceConfig_Role_SENSOR && config.power.is_power_saving) {
                LOG_DEBUG("Start next execution in 5s, then sleep");
                sleepOnNextExecution = true;
                setIntervalFromNow(5000);
            }
        }
        return true;
    }
    return false;
}

AdminMessageHandleResult EnvironmentTelemetryModule::handleAdminMessageForModule(const meshtastic_MeshPacket &mp,
                                                                                 meshtastic_AdminMessage *request,
                                                                                 meshtastic_AdminMessage *response)
{
    AdminMessageHandleResult result = AdminMessageHandleResult::NOT_HANDLED;
    if (dfRobotLarkSensor.hasSensor()) {
        result = dfRobotLarkSensor.handleAdminMessage(mp, request, response);
        if (result != AdminMessageHandleResult::NOT_HANDLED)
            return result;
    }
    if (sht31Sensor.hasSensor()) {
        result = sht31Sensor.handleAdminMessage(mp, request, response);
        if (result != AdminMessageHandleResult::NOT_HANDLED)
            return result;
    }
    if (lps22hbSensor.hasSensor()) {
        result = lps22hbSensor.handleAdminMessage(mp, request, response);
        if (result != AdminMessageHandleResult::NOT_HANDLED)
            return result;
    }
    if (shtc3Sensor.hasSensor()) {
        result = shtc3Sensor.handleAdminMessage(mp, request, response);
        if (result != AdminMessageHandleResult::NOT_HANDLED)
            return result;
    }
    if (bmp085Sensor.hasSensor()) {
        result = bmp085Sensor.handleAdminMessage(mp, request, response);
        if (result != AdminMessageHandleResult::NOT_HANDLED)
            return result;
    }
    if (bmp280Sensor.hasSensor()) {
        result = bmp280Sensor.handleAdminMessage(mp, request, response);
        if (result != AdminMessageHandleResult::NOT_HANDLED)
            return result;
    }
    if (bme280Sensor.hasSensor()) {
        result = bme280Sensor.handleAdminMessage(mp, request, response);
        if (result != AdminMessageHandleResult::NOT_HANDLED)
            return result;
    }
    if (bmp3xxSensor.hasSensor()) {
        result = bmp3xxSensor.handleAdminMessage(mp, request, response);
        if (result != AdminMessageHandleResult::NOT_HANDLED)
            return result;
    }
    if (bme680Sensor.hasSensor()) {
        result = bme680Sensor.handleAdminMessage(mp, request, response);
        if (result != AdminMessageHandleResult::NOT_HANDLED)
            return result;
    }
    if (mcp9808Sensor.hasSensor()) {
        result = mcp9808Sensor.handleAdminMessage(mp, request, response);
        if (result != AdminMessageHandleResult::NOT_HANDLED)
            return result;
    }
    if (ina219Sensor.hasSensor()) {
        result = ina219Sensor.handleAdminMessage(mp, request, response);
        if (result != AdminMessageHandleResult::NOT_HANDLED)
            return result;
    }
    if (ina260Sensor.hasSensor()) {
        result = ina260Sensor.handleAdminMessage(mp, request, response);
        if (result != AdminMessageHandleResult::NOT_HANDLED)
            return result;
    }
    if (ina3221Sensor.hasSensor()) {
        result = ina3221Sensor.handleAdminMessage(mp, request, response);
        if (result != AdminMessageHandleResult::NOT_HANDLED)
            return result;
    }
    if (veml7700Sensor.hasSensor()) {
        result = veml7700Sensor.handleAdminMessage(mp, request, response);
        if (result != AdminMessageHandleResult::NOT_HANDLED)
            return result;
    }
    if (tsl2591Sensor.hasSensor()) {
        result = tsl2591Sensor.handleAdminMessage(mp, request, response);
        if (result != AdminMessageHandleResult::NOT_HANDLED)
            return result;
    }
    if (opt3001Sensor.hasSensor()) {
        result = opt3001Sensor.handleAdminMessage(mp, request, response);
        if (result != AdminMessageHandleResult::NOT_HANDLED)
            return result;
    }
    if (mlx90632Sensor.hasSensor()) {
        result = mlx90632Sensor.handleAdminMessage(mp, request, response);
        if (result != AdminMessageHandleResult::NOT_HANDLED)
            return result;
    }
    if (rcwl9620Sensor.hasSensor()) {
        result = rcwl9620Sensor.handleAdminMessage(mp, request, response);
        if (result != AdminMessageHandleResult::NOT_HANDLED)
            return result;
    }
    if (nau7802Sensor.hasSensor()) {
        result = nau7802Sensor.handleAdminMessage(mp, request, response);
        if (result != AdminMessageHandleResult::NOT_HANDLED)
            return result;
    }
    if (aht10Sensor.hasSensor()) {
        result = aht10Sensor.handleAdminMessage(mp, request, response);
        if (result != AdminMessageHandleResult::NOT_HANDLED)
            return result;
    }
    if (max17048Sensor.hasSensor()) {
        result = max17048Sensor.handleAdminMessage(mp, request, response);
        if (result != AdminMessageHandleResult::NOT_HANDLED)
            return result;
    }
    if (cgRadSens.hasSensor()) {
        result = cgRadSens.handleAdminMessage(mp, request, response);
        if (result != AdminMessageHandleResult::NOT_HANDLED)
            return result;
    }
    return result;
}

#endif
