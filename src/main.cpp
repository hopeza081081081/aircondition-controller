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
#include <EEPROM.h>

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
const unsigned long AUTO_RESTART_INTERVAL = 12 * 60 * 60 * 1000; // 12 hours
const int MAX_PZEM_ERRORS = 5;

// Global variables for auto restart
unsigned long bootMillis = 0;

String clientId = "";
String deviceTopic = "";
float voltage, current, power, energy, frequency;
int ledState = LOW, bootCount = 0, pzemErrorCount = 0;
bool cmdFromServer = false, serverIsOnline = false;

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

void setup()
{
    Serial.begin(115200);
    WiFi.setAutoReconnect(true);

    // Device identification based on MAC address
    if (WiFi.macAddress() == "F0:08:D1:D7:6D:F8")
    {
        clientId = "airconController3";
        deviceTopic = "myFinalProject/airconController3/";
    }
    else if (WiFi.macAddress() == "8C:AA:B5:93:69:34")
    {
        clientId = "airconController2";
        deviceTopic = "myFinalProject/airconController2/";
    }
    else if (WiFi.macAddress() == "8C:AA:B5:94:1E:5C")
    {
        clientId = "airconController1";
        deviceTopic = "myFinalProject/airconController1/";
    }

    EEPROM.begin(4);
    pinMode(LED_BUILTIN, OUTPUT);
    pinMode(25, OUTPUT);
    digitalWrite(25, LOW);

    // Read boot count
    bootCount = EEPROM.read(0);
    bootCount++;
    EEPROM.write(0, bootCount);
    EEPROM.commit();

    // Blink LED on startup
    for (int i = 0; i < 10; i++)
    {
        digitalWrite(LED_BUILTIN, HIGH);
        delay(50);
        digitalWrite(LED_BUILTIN, LOW);
        delay(50);
    }

    setup_wifi();

    // Record boot time for auto restart
    bootMillis = millis();

    client.begin(mqtt_server, mqtt_port, espClient);
    client.onMessage(on_message);

    // Set up LWT (Last Will and Testament)
    devicePropertiesJsonDoc["wifiLocalIP"] = WiFi.localIP().toString().c_str();
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
    vTaskDelay(1000 / portTICK_PERIOD_MS);
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
        // Check for auto restart (every 12 hours)
        unsigned long currentMillis = millis();
        if (currentMillis - bootMillis >= AUTO_RESTART_INTERVAL)
        {
            Serial.println("========================================");
            Serial.println("Watchdog: Auto restart triggered");
            Serial.println("Reason: 12 hours uptime elapsed");
            Serial.printf("Uptime: %lu ms (%.2f hours)\n",
                          currentMillis - bootMillis,
                          (currentMillis - bootMillis) / 3600000.0);
            Serial.println("========================================");

            // Small delay to allow MQTT LWT to be sent
            delay(1000);
            ESP.restart();
        }

        // Check every minute (no need to check frequently)
        vTaskDelay(60000 / portTICK_PERIOD_MS); // 60 seconds
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
}

// ========== MQTT Functions ==========
bool mqtt_connect()
{
    if (client.connect(clientId.c_str(), "admin", "5617091"))
    {
        devicePropertiesJsonDoc["wifiLocalIP"] = WiFi.localIP().toString().c_str();
        devicePropertiesJsonDoc["online"] = true;
        devicePropertiesJsonDoc["bootcount"] = bootCount;
        devicePropertiesJsonDoc["hostname"] = WiFi.getHostname();
        serializeJson(devicePropertiesJsonDoc, devicePropertiesJsonOutput);
        client.publish((deviceTopic + "properties").c_str(), devicePropertiesJsonOutput, true, 2);

        Serial.println("MQTT Connected");

        // Subscribe to command topics
        client.subscribe("myFinalProject/server/electricalAppliances/airconController1/command", 2);
        client.subscribe("myFinalProject/server/electricalAppliances/airconController2/command", 2);
        client.subscribe("myFinalProject/server/electricalAppliances/airconController3/command", 2);
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
