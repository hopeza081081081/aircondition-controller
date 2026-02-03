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
const unsigned long PZEM_READ_INTERVAL = 5000;      // Read PZEM every 5 seconds
const unsigned long LED_BLINK_INTERVAL = 1000;      // LED blink every 1 second
const unsigned long WIFI_RECONNECT_TIMEOUT = 60000; // 60 seconds
const unsigned long MQTT_RECONNECT_TIMEOUT = 30000; // 30 seconds
const int MAX_PZEM_ERRORS = 5;

// NTP Configuration
const char *ntpServer = "pool.ntp.org";
const long gmtOffset_sec = 7 * 3600; // GMT+7 for Thailand
const int daylightOffset_sec = 0;

// Global variables for auto restart
uint32_t lastRebootToken = 0; // Persisted last reboot marker (yyyymmddHH)

// OTA control flag
volatile bool otaInProgress = false; // Flag to indicate OTA is in progress

String clientId = "";
String deviceTopic = "";
float voltage, current, power, energy, frequency;
int ledState = LOW, bootCount = 0, pzemErrorCount = 0;
bool cmdFromServer = false, serverIsOnline = false;

// JSON buffers
const size_t electricalVariableJsonSize = JSON_OBJECT_SIZE(6); // Added RSSI
char electricalVariableJsonOutput[JSON_OBJECT_SIZE(6) + 100];

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

void setup()
{
    Serial.begin(115200);
    WiFi.setAutoReconnect(true);

    // Device identification based on MAC address
    String macAddress = WiFi.macAddress();
    macAddress.replace(":", ""); // Remove colons: F0:08:D1:D7:6D:F8 -> F008D1D76DF8
    clientId = "aircon_" + macAddress;
    deviceTopic = "myFinalProject/airconController/" + clientId + "/";

    // Read boot count from Preferences
    Preferences prefs;
    prefs.begin("my-app", false);
    bootCount = prefs.getUInt("bootcnt", 0);
    bootCount++;
    prefs.putUInt("bootcnt", bootCount);
    lastRebootToken = prefs.getUInt("lastreboot", 0);
    prefs.end();

    pinMode(LED_BUILTIN, OUTPUT);
    pinMode(25, OUTPUT);
    digitalWrite(25, LOW);

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
    electricalVariableJsonDoc["rssi"] = 0;

    // Setup ArduinoOTA
    ArduinoOTA.onStart([]()
                       {
        otaInProgress = true;
        Serial.println("========================================");
        Serial.println("OTA Started - Suspending all tasks...");

        // Suspend all other tasks to give full resources to OTA
        if (mqttTask != NULL) vTaskSuspend(mqttTask);
        if (pzemTask != NULL) vTaskSuspend(pzemTask);
        if (ledTask != NULL) vTaskSuspend(ledTask);
        if (watchdogTask != NULL) vTaskSuspend(watchdogTask);

        Serial.println("All tasks suspended. OTA has full control.");
        Serial.println("========================================");

        String type;
        if (ArduinoOTA.getCommand() == U_FLASH)
            type = "sketch";
        else
            type = "filesystem";
        Serial.println("Start updating " + type); })
        .onEnd([]()
               {
        Serial.println("\n========================================");
        Serial.println("OTA Completed successfully!");
        Serial.println("========================================");
        otaInProgress = false; })
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
            Serial.println("End Failed");

        // On error, resume tasks so device can continue operating
        Serial.println("OTA Error - Resuming all tasks...");
        if (mqttTask != NULL) vTaskResume(mqttTask);
        if (pzemTask != NULL) vTaskResume(pzemTask);
        if (ledTask != NULL) vTaskResume(ledTask);
        if (watchdogTask != NULL) vTaskResume(watchdogTask);
        otaInProgress = false;
        Serial.println("========================================"); });

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
    bool wasOtaActive = false;

    while (true)
    {
        ArduinoOTA.handle();

        // Track OTA state changes
        if (otaInProgress && !wasOtaActive)
        {
            // OTA just started
            wasOtaActive = true;
            Serial.println("OTA Handler: OTA in progress detected");
        }
        else if (!otaInProgress && wasOtaActive)
        {
            // OTA just finished successfully
            wasOtaActive = false;
            Serial.println("========================================");
            Serial.println("OTA Handler: OTA finished - Resuming all tasks...");
            if (mqttTask != NULL)
                vTaskResume(mqttTask);
            if (pzemTask != NULL)
                vTaskResume(pzemTask);
            if (ledTask != NULL)
                vTaskResume(ledTask);
            if (watchdogTask != NULL)
                vTaskResume(watchdogTask);
            Serial.println("All tasks resumed. Normal operation restored.");
            Serial.println("========================================");
        }

        vTaskDelay(10 / portTICK_PERIOD_MS); // Check OTA every 10ms
    }
}

// ========== MQTT Task ==========
void handle_mqtt(void *parameter)
{
    while (true)
    {
        // Skip all work if OTA is in progress
        if (otaInProgress)
        {
            vTaskDelay(100 / portTICK_PERIOD_MS);
            continue;
        }

        client.loop();

        // Check connection and reconnect if needed
        if (!client.connected())
        {
            static unsigned long lastAttempt = 0;
            unsigned long now = millis();

            if (now - lastAttempt >= 5000) // Try to reconnect every 5 seconds
            {
                lastAttempt = now;
                if (!mqtt_connect())
                {
                    // Connection failed
                }
            }
        }

        vTaskDelay(50 / portTICK_PERIOD_MS); // Check MQTT every 50ms
    }
}

// ========== PZEM Task ==========
void handle_pzem(void *parameter)
{
    while (true)
    {
        // Skip all work if OTA is in progress
        if (otaInProgress)
        {
            vTaskDelay(1000 / portTICK_PERIOD_MS);
            continue;
        }

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
        Serial.printf("PZEM Error Count: %d\n", pzemErrorCount);
        if (pzemErrorCount >= MAX_PZEM_ERRORS)
        {
            Serial.println("Too many PZEM errors, restarting...");
            ESP.restart();
        }
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

    // Read WiFi RSSI
    int rssi = WiFi.RSSI();
    electricalVariableJsonDoc["rssi"] = rssi;

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
        // Skip all work if OTA is in progress
        if (otaInProgress)
        {
            vTaskDelay(100 / portTICK_PERIOD_MS);
            continue;
        }

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
        // Skip all work if OTA is in progress (don't restart during OTA!)
        if (otaInProgress)
        {
            vTaskDelay(1000 / portTICK_PERIOD_MS);
            continue;
        }

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
            int currentYear = timeinfo.tm_year + 1900;
            int currentMonth = timeinfo.tm_mon + 1;
            int currentDay = timeinfo.tm_mday;

            // If time looks invalid (not synced yet), skip reboot logic
            if (currentYear < 2020)
            {
                Serial.println("NTP time not valid yet, skipping scheduled restart check...");
            }
            else
            {
                // Build a unique token per hour: yyyymmddHH
                uint32_t rebootToken = (uint32_t)(currentYear * 1000000UL +
                                                  currentMonth * 10000UL +
                                                  currentDay * 100UL +
                                                  currentHour);

                // Check if we're at 06:00:00 or 18:00:00 and haven't rebooted this hour+day yet
                // Use a small time window (within first 10 seconds of the minute) to avoid missing the exact moment
                bool isRebootTime = ((currentHour == 6 || currentHour == 18) &&
                                     currentMinute == 0 &&
                                     currentSecond < 10 &&
                                     lastRebootToken != rebootToken);

                if (isRebootTime)
                {
                    lastRebootToken = rebootToken; // Mark this hour as rebooted

                    // Persist last reboot token to avoid reboot loops after restart
                    Preferences prefs;
                    prefs.begin("my-app", false);
                    prefs.putUInt("lastreboot", lastRebootToken);
                    prefs.end();

                    Serial.println("========================================");
                    Serial.println("Watchdog: Scheduled restart triggered");
                    Serial.printf("Reason: Scheduled reboot at %02d:00:00\n", currentHour);
                    Serial.printf("Current time: %02d:%02d:%02d\n",
                                  currentHour, currentMinute, currentSecond);
                    Serial.printf("Date: %04d-%02d-%02d\n",
                                  currentYear, currentMonth, currentDay);
                    Serial.println("========================================");

                    // Small delay to allow MQTT LWT to be sent
                    vTaskDelay(1000 / portTICK_PERIOD_MS);
                    ESP.restart();
                }
            }
        }

        // Check every 1 second for precise timing
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}

// ========== WiFi Setup ==========
void setup_wifi()
{
    Serial.println();
    Serial.print("Connecting to ");
    Serial.println(ssid);

    WiFi.begin(ssid, password);
    WiFi.setSleep(false); // Disable WiFi low-power mode for stable connection

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
}

// ========== MQTT Functions ==========
bool mqtt_connect()
{
    if (client.connect(clientId.c_str(), "admin", "5617091"))
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
