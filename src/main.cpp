/**
 *
 * 1. compress measuring message to JSON format.
 * 2. platformio run -t upload --upload-port xxx.xxx.xxx.xxx
 * ainconController1: 10.10.200.60
 * ainconController2: 10.10.200.57
 * ainconController3: 10.10.200.59
 *
 * */
#include <Arduino.h>
#include <PZEM004Tv30.h>
#include <WiFi.h>
#include <MQTT.h>
#include <string.h>
#include <ESPmDNS.h>
#include <WiFiUdp.h>
#include <ArduinoOTA.h>
#include <ArduinoJson.h>
#include <Preferences.h>
#include <time.h>

#if !defined(PZEM_RX_PIN) && !defined(PZEM_TX_PIN)
#define PZEM_RX_PIN 16
#define PZEM_TX_PIN 17
#endif

#if !defined(PZEM_SERIAL)
#define PZEM_SERIAL Serial2
#endif

// Task handles
TaskHandle_t otaTask = NULL;
TaskHandle_t mqttTask = NULL;
TaskHandle_t pzemTask = NULL;
TaskHandle_t ledTask = NULL;
TaskHandle_t watchdogTask = NULL;

// Constants
const char *ssid = "cpciot1";
const char *password = "10987654";
const char *mqtt_server = "10.10.200.70";
const int mqtt_port = 1883;
/*const char *ssid = "true_home2G_UM3";
const char *password = "Kk67Dc54";
const char *mqtt_server = "192.168.1.54";
const int mqtt_port = 11883;*/

// Timing constants (milliseconds)
const unsigned long PZEM_READ_INTERVAL = 5000;                   // Read PZEM every 5 seconds
const unsigned long LED_BLINK_INTERVAL = 1000;                   // LED blink every 1 second
const unsigned long WIFI_RECONNECT_TIMEOUT = 60000;              // 60 seconds
const unsigned long MQTT_RECONNECT_TIMEOUT = 30000;              // 30 seconds
const int MAX_PZEM_ERRORS = 5;

// NTP Configuration
const char* ntpServer = "pool.ntp.org";
const long gmtOffset_sec = 7 * 3600;  // GMT+7 for Thailand
const int daylightOffset_sec = 0;

// Global variables for auto restart
int lastRebootHour = -1;  // Track last reboot hour (-1 = not set yet)

String clientId = "";
String deviceTopic = "";
const char* mqttUserName = "hrvmbcju";
const char* mqttPassword = "g7usW2NJz0H_";
float voltage, current, power, energy, frequency;
int ledState = LOW, bootCount = 0, pzemErrorCount = 0;
bool cmdFromServer = false, serverIsOnline = false;

// Debug variables
unsigned long lastDebugPrint = 0;
const unsigned long DEBUG_PRINT_INTERVAL = 30000;  // Print debug info every 30 seconds

// JSON buffers
const size_t electricalVariableJsonSize = JSON_OBJECT_SIZE(5);
char electricalVariableJsonOutput[JSON_OBJECT_SIZE(5) + 80];

const size_t devicePropertiesJsonSize = JSON_OBJECT_SIZE(4); // Increased for hostname
char devicePropertiesJsonOutput[JSON_OBJECT_SIZE(4) + 100];

// Global objects
PZEM004Tv30 pzem(PZEM_SERIAL, PZEM_RX_PIN, PZEM_TX_PIN);
WiFiClient espClient;
MQTTClient client(1024);
DynamicJsonDocument electricalVariableJsonDoc(electricalVariableJsonSize);
DynamicJsonDocument devicePropertiesJsonDoc(devicePropertiesJsonSize);

// Function declarations
void setup_wifi();
void on_message(String &topic, String &payload);
bool mqtt_connect();
void handle_ota(void *parameter);
void handle_mqtt(void *parameter);
void handle_pzem(void *parameter);
void handle_led(void *parameter);
void handle_watchdog(void *parameter);
void readAndPublishPZEM();
void printDebugInfo();
String getResetReason(esp_reset_reason_t reason);

void setup()
{
    Serial.begin(115200);
    WiFi.setAutoReconnect(true);

    // Device identification based on MAC address
    String macAddress = WiFi.macAddress();
    macAddress.replace(":", "");  // Remove colons: F0:08:D1:D7:6D:F8 -> F008D1D76DF8
    clientId = "aircon_" + macAddress;
    deviceTopic = "myFinalProject/airconController/" + clientId + "/";

    // Read boot count from Preferences
    Preferences prefs;
    prefs.begin("my-app", false);
    bootCount = prefs.getUInt("bootcnt", 0);
    bootCount++;
    prefs.putUInt("bootcnt", bootCount);
    prefs.end();

    pinMode(LED_BUILTIN, OUTPUT);
    pinMode(25, OUTPUT);
    digitalWrite(25, LOW);

    // Print reset reason for debugging
    Serial.println("========================================");
    Serial.println("ESP32 Reset Info:");
    Serial.print("Reset Reason: ");
    Serial.println(getResetReason(esp_reset_reason()));
    Serial.printf("Boot Count: %d\n", bootCount);
    Serial.printf("Free Heap: %u bytes\n", ESP.getFreeHeap());
    Serial.printf("Heap Size: %u bytes\n", ESP.getHeapSize());
    Serial.printf("Free PSRAM: %u bytes\n", ESP.getFreePsram());
    Serial.printf("Chip Model: %s\n", ESP.getChipModel());
    Serial.printf("Chip Cores: %d\n", ESP.getChipCores());
    Serial.printf("CPU Freq: %d MHz\n", ESP.getCpuFreqMHz());
    Serial.println("========================================");

    // Blink LED on startup
    for (int i = 0; i < 10; i++)
    {
        digitalWrite(LED_BUILTIN, HIGH);
        delay(50);
        digitalWrite(LED_BUILTIN, LOW);
        delay(50);
    }

    setup_wifi();

    // Initialize NTP time synchronization
    configTime(gmtOffset_sec, daylightOffset_sec, ntpServer);

    client.begin(mqtt_server, mqtt_port, espClient);
    client.onMessage(on_message);

    // Set up LWT (Last Will and Testament)
    static String ipCacheLWT;
    ipCacheLWT = WiFi.localIP().toString();
    devicePropertiesJsonDoc["wifiLocalIP"] = ipCacheLWT.c_str();
    devicePropertiesJsonDoc["online"] = false;
    devicePropertiesJsonDoc["bootcount"] = bootCount;
    devicePropertiesJsonDoc["hostname"] = WiFi.getHostname();
    serializeJson(devicePropertiesJsonDoc, devicePropertiesJsonOutput);
    client.setWill((deviceTopic + "properties").c_str(), devicePropertiesJsonOutput, true, 2);

    mqtt_connect();

    // Initialize JSON values
    electricalVariableJsonDoc["voltage"] = "null";
    electricalVariableJsonDoc["current"] = "null";
    electricalVariableJsonDoc["power"] = "null";
    electricalVariableJsonDoc["energy"] = "null";
    electricalVariableJsonDoc["frequency"] = "null";

    // Setup ArduinoOTA
    ArduinoOTA.onStart([]()
                       {
        String type;
        if (ArduinoOTA.getCommand() == U_FLASH)
            type = "sketch";
        else
            type = "filesystem";
        Serial.println("Start updating " + type); })
        .onEnd([]()
               { Serial.println("\nEnd"); })
        .onProgress([](unsigned int progress, unsigned int total)
                    { Serial.printf("Progress: %u%%\r", (progress / (total / 100))); })
        .onError([](ota_error_t error)
                 {
        Serial.printf("Error[%u]: ", error);
        if (error == OTA_AUTH_ERROR)
            Serial.println("Auth Failed");
        else if (error == OTA_BEGIN_ERROR)
            Serial.println("Begin Failed");
        else if (error == OTA_CONNECT_ERROR)
            Serial.println("Connect Failed");
        else if (error == OTA_RECEIVE_ERROR)
            Serial.println("Receive Failed");
        else if (error == OTA_END_ERROR)
            Serial.println("End Failed"); });

    ArduinoOTA.begin();

    // Create FreeRTOS tasks

    // OTA Task - Highest priority, Core 0
    xTaskCreatePinnedToCore(
        &handle_ota,
        "OTA_Handler",
        8096,
        NULL,
        3, // Highest priority
        &otaTask,
        0 // Core 0
    );

    // MQTT Task - High priority, Core 1
    xTaskCreatePinnedToCore(
        &handle_mqtt,
        "MQTT_Handler",
        8096,
        NULL,
        2, // High priority
        &mqttTask,
        1 // Core 1
    );

    // PZEM Task - Medium priority, Core 1
    xTaskCreatePinnedToCore(
        &handle_pzem,
        "PZEM_Handler",
        4096,
        NULL,
        1, // Medium priority
        &pzemTask,
        1 // Core 1
    );

    // LED Task - Low priority, Core 0
    xTaskCreatePinnedToCore(
        &handle_led,
        "LED_Handler",
        2048,
        NULL,
        1, // Low priority
        &ledTask,
        0 // Core 0
    );

    // Watchdog Task - Low priority, Core 0
    xTaskCreatePinnedToCore(
        &handle_watchdog,
        "Watchdog_Handler",
        2048,
        NULL,
        1, // Low priority
        &watchdogTask,
        0 // Core 0
    );

    Serial.println("All tasks created successfully");
}

void loop()
{
    // Main loop is empty - all work is done in FreeRTOS tasks
    vTaskDelay(60000 / portTICK_PERIOD_MS);
}

// ========== OTA Task ==========
void handle_ota(void *parameter)
{
    while (true)
    {
        ArduinoOTA.handle();
        vTaskDelay(10 / portTICK_PERIOD_MS); // Check OTA every 10ms
    }
}

// ========== MQTT Task ==========
void handle_mqtt(void *parameter)
{
    while (true)
    {
        client.loop();

        // Check connection and reconnect if needed
        if (!client.connected())
        {
            static unsigned long lastAttempt = 0;
            unsigned long now = millis();

            if (now - lastAttempt >= 5000) // Try to reconnect every 5 seconds
            {
                lastAttempt = now;
                Serial.println("MQTT disconnected, attempting to reconnect...");
                Serial.printf("WiFi Status: %d\n", WiFi.status());
                Serial.printf("RSSI: %d dBm\n", WiFi.RSSI());
                Serial.printf("Free Heap: %u bytes\n", ESP.getFreeHeap());
                if (!mqtt_connect())
                {
                    // Connection failed
                    Serial.println("MQTT reconnection failed");
                }
            }
        }

        // Print debug info periodically
        unsigned long now = millis();
        if (now - lastDebugPrint >= DEBUG_PRINT_INTERVAL)
        {
            lastDebugPrint = now;
            printDebugInfo();
        }

        vTaskDelay(50 / portTICK_PERIOD_MS); // Check MQTT every 50ms
    }
}

// ========== PZEM Task ==========
void handle_pzem(void *parameter)
{
    while (true)
    {
        readAndPublishPZEM();
        vTaskDelay(PZEM_READ_INTERVAL / portTICK_PERIOD_MS); // Read every 5 seconds
    }
}

void readAndPublishPZEM()
{
    // Check for Serial2 errors
    if (Serial2.available() != 0)
    {
        pzemErrorCount++;
        Serial.println("========================================");
        Serial.println("PZEM ERROR DETECTED!");
        Serial.printf("PZEM Error Count: %d / %d\n", pzemErrorCount, MAX_PZEM_ERRORS);
        Serial.printf("Serial2 available: %d bytes\n", Serial2.available());
        Serial.printf("Free Heap: %u bytes\n", ESP.getFreeHeap());
        Serial.printf("WiFi RSSI: %d dBm\n", WiFi.RSSI());

        // Clear the buffer to prevent accumulation
        while (Serial2.available() > 0)
        {
            int c = Serial2.read();
            Serial.printf("Clearing byte: 0x%02X\n", c);
        }

        if (pzemErrorCount >= MAX_PZEM_ERRORS)
        {
            Serial.println("========================================");
            Serial.println("Too many PZEM errors, restarting...");
            Serial.printf("Final Error Count: %d\n", pzemErrorCount);
            Serial.printf("Uptime before restart: %lu ms\n", millis());
            Serial.println("========================================");
            delay(1000);
            ESP.restart();
        }
        Serial.println("========================================");
    }

    // Read all values from PZEM
    voltage = pzem.voltage();
    electricalVariableJsonDoc["voltage"] = isnan(voltage) ? 0.0 : voltage;

    current = pzem.current();
    electricalVariableJsonDoc["current"] = isnan(current) ? 0.0 : current;

    power = pzem.power();
    electricalVariableJsonDoc["power"] = isnan(power) ? 0.0 : power;

    energy = pzem.energy();
    electricalVariableJsonDoc["energy"] = isnan(energy) ? 0.0 : energy;

    frequency = pzem.frequency();
    electricalVariableJsonDoc["frequency"] = isnan(frequency) ? 0.0 : frequency;

    // Publish to MQTT
    serializeJson(electricalVariableJsonDoc, electricalVariableJsonOutput);

    // Only publish if MQTT is connected
    if (client.connected())
    {
        client.publish((deviceTopic + "measure").c_str(), electricalVariableJsonOutput, false, 0);
    }
}

// ========== LED Task ==========
void handle_led(void *parameter)
{
    unsigned long previousMillis = 0;

    while (true)
    {
        unsigned long currentMillis = millis();

        if (currentMillis - previousMillis >= LED_BLINK_INTERVAL)
        {
            previousMillis = currentMillis;
            ledState = (ledState == LOW) ? HIGH : LOW;
            digitalWrite(LED_BUILTIN, ledState);
        }

        vTaskDelay(100 / portTICK_PERIOD_MS);
    }
}

// ========== Watchdog Task ==========
void handle_watchdog(void *parameter)
{
    while (true)
    {
        // Get current time from NTP
        struct tm timeinfo;
        if (!getLocalTime(&timeinfo))
        {
            Serial.println("Failed to obtain time, waiting for NTP sync...");
        }
        else
        {
            int currentHour = timeinfo.tm_hour;
            int currentMinute = timeinfo.tm_min;
            int currentSecond = timeinfo.tm_sec;

            // Check if we're at 06:00:00 or 18:00:00 and haven't rebooted this hour yet
            // Use a small time window (within first 5 seconds of the minute) to avoid missing the exact moment
            bool isRebootTime = ((currentHour == 6 || currentHour == 18) &&
                                 currentMinute == 0 &&
                                 currentSecond < 5 &&
                                 lastRebootHour != currentHour);

            if (isRebootTime)
            {
                lastRebootHour = currentHour;  // Mark this hour as rebooted

                Serial.println("========================================");
                Serial.println("Watchdog: Scheduled restart triggered");
                Serial.printf("Reason: Scheduled reboot at %02d:00:00\n", currentHour);
                Serial.printf("Current time: %02d:%02d:%02d\n",
                              currentHour, currentMinute, currentSecond);
                Serial.printf("Date: %04d-%02d-%02d\n",
                              timeinfo.tm_year + 1900, timeinfo.tm_mon + 1, timeinfo.tm_mday);
                Serial.println("========================================");

                // Small delay to allow MQTT LWT to be sent
                vTaskDelay(1000 / portTICK_PERIOD_MS);
                ESP.restart();
            }
        }

        // Check every 10 seconds for precise timing
        vTaskDelay(10000 / portTICK_PERIOD_MS);
    }
}

// ========== WiFi Setup ==========
void setup_wifi()
{
    Serial.println();
    Serial.print("Connecting to ");
    Serial.println(ssid);

    WiFi.begin(ssid, password);

    unsigned long startMillis = millis();
    while (WiFi.status() != WL_CONNECTED)
    {
        delay(500);
        Serial.print(".");

        if (millis() - startMillis >= WIFI_RECONNECT_TIMEOUT)
        {
            Serial.println("\nWiFi connection timeout, restarting...");
            ESP.restart();
        }
    }

    Serial.println("");
    Serial.println("WiFi connected");
    Serial.println("IP address: ");
    Serial.println(WiFi.localIP());
    Serial.printf("Gateway: %s\n", WiFi.gatewayIP().toString().c_str());
    Serial.printf("Subnet Mask: %s\n", WiFi.subnetMask().toString().c_str());
    Serial.printf("DNS: %s\n", WiFi.dnsIP().toString().c_str());
    Serial.printf("RSSI: %d dBm\n", WiFi.RSSI());
    Serial.printf("WiFi Channel: %d\n", WiFi.channel());
}

// ========== MQTT Functions ==========
bool mqtt_connect()
{
    if (client.connect(clientId.c_str(), mqttUserName, mqttPassword))
    {
        static String ipCache;
        ipCache = WiFi.localIP().toString();
        devicePropertiesJsonDoc["wifiLocalIP"] = ipCache.c_str();
        devicePropertiesJsonDoc["online"] = true;
        devicePropertiesJsonDoc["bootcount"] = bootCount;
        devicePropertiesJsonDoc["hostname"] = WiFi.getHostname();
        serializeJson(devicePropertiesJsonDoc, devicePropertiesJsonOutput);
        client.publish((deviceTopic + "properties").c_str(), devicePropertiesJsonOutput, true, 2);

        Serial.println("MQTT Connected");

        // Subscribe to command topics
        static String commandTopic;
        commandTopic = "myFinalProject/server/airconController/" + clientId + "/command";
        client.subscribe(commandTopic.c_str(), 2);
        client.subscribe("myFinalProject/server/properties/online", 2);

        Serial.printf("Boot count: %d\n", bootCount);

        return true;
    }
    else
    {
        Serial.printf("MQTT connection failed, error code: %d\n", client.lastError());
        digitalWrite(25, LOW);
        return false;
    }
}

// ========== MQTT Message Handler ==========
void on_message(String &topic, String &payload)
{
    // Handle command messages for any controller
    if (topic.endsWith("/command"))
    {
        cmdFromServer = (payload == "true");
        Serial.printf("Received command: %s\n", cmdFromServer ? "ON" : "OFF");
    }
    // Handle server online status
    else if (topic == "myFinalProject/server/properties/online")
    {
        serverIsOnline = (payload == "true");
        Serial.printf("Server online: %s\n", serverIsOnline ? "true" : "false");
    }

    // Update GPIO based on server status and command
    digitalWrite(25, (serverIsOnline && cmdFromServer) ? HIGH : LOW);
}

// ========== Debug Functions ==========
void printDebugInfo()
{
    Serial.println("========================================");
    Serial.println("DEBUG INFO:");

    // WiFi Status
    Serial.println("--- WiFi ---");
    Serial.printf("Status: %d\n", WiFi.status());
    Serial.printf("RSSI: %d dBm\n", WiFi.RSSI());
    Serial.printf("IP: %s\n", WiFi.localIP().toString().c_str());
    Serial.printf("Connected: %s\n", WiFi.status() == WL_CONNECTED ? "Yes" : "No");

    // MQTT Status
    Serial.println("--- MQTT ---");
    Serial.printf("Connected: %s\n", client.connected() ? "Yes" : "No");
    if (!client.connected())
    {
        Serial.printf("Last Error: %d\n", client.lastError());
    }

    // Memory Status
    Serial.println("--- Memory ---");
    Serial.printf("Free Heap: %u bytes\n", ESP.getFreeHeap());
    Serial.printf("Largest Free Block: %u bytes\n", ESP.getMaxAllocHeap());
    Serial.printf("Heap Size: %u bytes\n", ESP.getHeapSize());
    if (ESP.getPsramSize() > 0)
    {
        Serial.printf("Free PSRAM: %u bytes\n", ESP.getFreePsram());
        Serial.printf("PSRAM Size: %u bytes\n", ESP.getPsramSize());
    }

    // PZEM Status
    Serial.println("--- PZEM ---");
    Serial.printf("Error Count: %d / %d\n", pzemErrorCount, MAX_PZEM_ERRORS);
    Serial.printf("Voltage: %.2f V\n", voltage);
    Serial.printf("Current: %.2f A\n", current);
    Serial.printf("Power: %.2f W\n", power);

    // Task Status
    Serial.println("--- Tasks ---");
    Serial.printf("Uptime: %lu ms\n", millis());
    Serial.printf("Boot Count: %d\n", bootCount);

    Serial.println("========================================");
}

String getResetReason(esp_reset_reason_t reason)
{
    switch (reason)
    {
    case ESP_RST_POWERON:
        return "Power on reset (not due to crash)";
    case ESP_RST_EXT:
        return "External reset";
    case ESP_RST_SW:
        return "Software reset via ESP.restart()";
    case ESP_RST_PANIC:
        return "Software reset due to exception/panic";
    case ESP_RST_INT_WDT:
        return "Reset (interrupted watchdog)";
    case ESP_RST_TASK_WDT:
        return "Reset (task watchdog)";
    case ESP_RST_WDT:
        return "Reset (other watchdog)";
    case ESP_RST_DEEPSLEEP:
        return "Reset after exiting deep sleep mode";
    case ESP_RST_BROWNOUT:
        return "Brownout reset (voltage dropped too low)";
    case ESP_RST_SDIO:
        return "Reset over SDIO";
    default:
        return "Unknown reset reason";
    }
}
