#define IDLE_WAIT 1
#define MOVING_UP 2
#define MOVING_DOWN 3
#define STOP_AT_CURRENT 4
#define SET_ENDSTOPS 5

#include "OneButton.h"
#include <ESP8266WiFi.h>
#include <PubSubClient.h>
#include <AccelStepper.h>

// Pin definitions
#define SW_ACT_PIN 2      // Button pin
#define STEPPER_STEP 13   // Stepper STEP pin
#define STEPPER_DIR 12    // Stepper DIR pin
#define ENABLE_PIN 14     // Stepper driver enable pin
#define RELAY_PIN 16      // Relay control pin

// MQTT Configuration
const char* ssid = "SSID";
const char* password = "SSID_PW";
const char* mqtt_server = "ADDRESS_TO_MQTT_SERVER";
const char* MQTT_USER = "MQTT_USER";
const char* MQTT_PASSWORD = "MQTT_PASSWORD";
const char* outTopic = "cover-1/out";
const char* inTopic = "cover-1/in";
const char* STATUS_OPEN = "open";
const char* STATUS_CLOSED = "closed";

// Button configuration
OneButton ButtonAct(SW_ACT_PIN, false);  // false = button is pulled up

// Network clients
WiFiClient espClient;
PubSubClient client(espClient);
unsigned long lastMqttReconnectAttempt = 0;
const unsigned long MQTT_RECONNECT_DELAY = 5000; // 5 seconds

// Stepper configuration
AccelStepper stepper(AccelStepper::DRIVER, STEPPER_STEP, STEPPER_DIR);
const unsigned long MOTOR_UNLOCK_DELAY = 15; // ms
const float MAX_SPEED = 8000.0;
const float NORMAL_ACCELERATION = 8000.0;
const float ENDSTOP_SPEED = 2000.0;

// Position tracking
struct CurtainLimits {
    int maxPos = 0;
    int minPos = 0;
    bool isCalibrated = false;
} limits;

// Calibration state
struct CalibrationState {
    bool movingDown = false;
    bool movingUp = false;
    bool doneDown = false;
    bool doneUp = false;
} calibration;

byte state = IDLE_WAIT;

void setupWiFi() {
    Serial.println();
    Serial.print("Connecting to WiFi network: ");
    Serial.println(ssid);

    WiFi.begin(ssid, password);

    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
    }

    Serial.println();
    Serial.println("WiFi connected");
    Serial.print("IP address: ");
    Serial.println(WiFi.localIP());
}

void handleMQTTMessage(char* topic, byte* payload, unsigned int length) {
    Serial.print("Message arrived [");
    Serial.print(topic);
    Serial.print("] ");
    
    String message;
    for (unsigned int i = 0; i < length; i++) {
        message += (char)payload[i];
    }
    Serial.println(message);

    if (message == "0") {
        state = MOVING_UP;
        Serial.println("Command: Opening cover");
        client.publish(outTopic, STATUS_OPEN);
    } 
    else if (message == "1") {
        state = MOVING_DOWN;
        Serial.println("Command: Closing cover");
        client.publish(outTopic, STATUS_CLOSED);
    } 
    else if (message == "2") {
        state = STOP_AT_CURRENT;
        Serial.println("Command: Stop movement");
        publishStatus();
    }
}

void reconnectMQTT() {
    if (!client.connected()) {
        unsigned long now = millis();
        if (now - lastMqttReconnectAttempt > MQTT_RECONNECT_DELAY) {
            lastMqttReconnectAttempt = now;
            Serial.print("Attempting MQTT connection...");
            
            String clientId = "ESP8266Client-";
            clientId += String(random(0xffff), HEX);
            
            if (client.connect(clientId.c_str(), MQTT_USER, MQTT_PASSWORD)) {
                Serial.println("connected");
                client.publish(outTopic, "Curtain controller online");
                client.subscribe(inTopic);
            } else {
                Serial.print("failed, rc=");
                Serial.print(client.state());
                Serial.println(" retry in 5 seconds");
            }
        }
    }
}

void lockMotor() {
    digitalWrite(ENABLE_PIN, HIGH);
    delay(MOTOR_UNLOCK_DELAY);
    digitalWrite(RELAY_PIN, LOW);
}

void unlockMotor() {
    digitalWrite(RELAY_PIN, HIGH);
    delay(MOTOR_UNLOCK_DELAY);
    digitalWrite(ENABLE_PIN, LOW);
}

void publishStatus() {
    const char* status = (stepper.currentPosition() >= limits.maxPos/2) ? STATUS_CLOSED : STATUS_OPEN;
    client.publish(outTopic, status);
}

void buttonDisabled() {
    // Intentionally empty - used when button should do nothing
}

void handleIdleState() {
    if (stepper.distanceToGo() == 0) {
        lockMotor();
    }
    
    if (stepper.isRunning()) {
        ButtonAct.attachClick([]() { state = STOP_AT_CURRENT; });
    } 
    else if (stepper.currentPosition() == limits.maxPos) {
        unlockMotor();
        ButtonAct.attachClick([]() { state = MOVING_UP; });
    } 
    else {
        unlockMotor();
        ButtonAct.attachClick([]() { state = MOVING_DOWN; });
    }
    
    // Configure double-click behavior
    if (stepper.currentPosition() == limits.minPos) {
        ButtonAct.attachDoubleClick(buttonDisabled);
    } else {
        ButtonAct.attachDoubleClick([]() { state = MOVING_UP; });
    }
    
    // Configure long-press for calibration
    ButtonAct.attachLongPressStart([]() {
        state = SET_ENDSTOPS;
        calibration = {true, false, false, false};
    });
    
    // Disable other button functions
    ButtonAct.attachDuringLongPress(buttonDisabled);
    ButtonAct.attachLongPressStop(buttonDisabled);
}

void finishDownCalibration() {
    stepper.stop();
    calibration.movingDown = false;
    limits.maxPos = stepper.currentPosition();
    Serial.print("Calibration: Max position set to: ");
    Serial.println(limits.maxPos);
    calibration.movingUp = true;
}

void finishUpCalibration() {
    stepper.stop();
    calibration.movingUp = false;
    limits.minPos = stepper.currentPosition();
    Serial.print("Calibration: Min position set to: ");
    Serial.println(limits.minPos);
    
    // Reset stepper to normal operation
    stepper.setMaxSpeed(MAX_SPEED);
    stepper.setAcceleration(NORMAL_ACCELERATION);
    
    limits.isCalibrated = true;
    state = IDLE_WAIT;
    Serial.println("Calibration complete, returning to idle");
}

void handleCalibrationState() {
    if (calibration.movingDown && stepper.distanceToGo() == 0) {
        unlockMotor();
        stepper.setMaxSpeed(ENDSTOP_SPEED);
        stepper.move(1000000);
        ButtonAct.attachClick([]() { calibration.doneDown = true; });
    }
    
    if (calibration.movingDown && calibration.doneDown) {
        finishDownCalibration();
    }
    
    if (calibration.movingUp && stepper.distanceToGo() == 0) {
        stepper.setMaxSpeed(ENDSTOP_SPEED);
        stepper.move(-1000000);
        ButtonAct.attachClick([]() { calibration.doneUp = true; });
    }
    
    if (calibration.movingUp && calibration.doneUp) {
        finishUpCalibration();
    }
    
    // Disable other button functions during calibration
    ButtonAct.attachLongPressStart(buttonDisabled);
    ButtonAct.attachDoubleClick(buttonDisabled);
    ButtonAct.attachDuringLongPress(buttonDisabled);
    ButtonAct.attachLongPressStop(buttonDisabled);
}

void handleMovementState() {
    if (stepper.distanceToGo() == 0) {
        unlockMotor();
        if (state == MOVING_UP) {
            stepper.moveTo(limits.minPos);
            Serial.println("Moving to minimum position");
        } else {
            stepper.moveTo(limits.maxPos);
            Serial.println("Moving to maximum position");
        }
        publishStatus();
        state = IDLE_WAIT;
        Serial.println("Movement complete, returning to idle");
    }
}

void handleStopState() {
    stepper.stop();
    Serial.print("Stopping at position: ");
    Serial.println(stepper.currentPosition());
    publishStatus();
    state = IDLE_WAIT;
}

void setup() {
    Serial.begin(115200);
    
    // Initialize pins
    pinMode(ENABLE_PIN, OUTPUT);
    pinMode(RELAY_PIN, OUTPUT);
    
    // Configure stepper
    stepper.setMaxSpeed(MAX_SPEED);
    stepper.setAcceleration(NORMAL_ACCELERATION);
    
    // Configure button
    ButtonAct.setClickTicks(600);    // Debounce time
    ButtonAct.setPressTicks(1000);   // Long press time
    
    // Setup network
    setupWiFi();
    client.setServer(mqtt_server, 1883);
    client.setCallback(handleMQTTMessage);
    
    Serial.println("System initialized, waiting for commands");
}

void loop() {
    // Handle MQTT connection
    if (!client.connected()) {
        reconnectMQTT();
    }
    client.loop();
    
    // Update button and stepper state
    ButtonAct.tick();
    stepper.run();
    
    // State machine
    switch(state) {
        case IDLE_WAIT:
            handleIdleState();
            break;
            
        case SET_ENDSTOPS:
            handleCalibrationState();
            break;
            
        case MOVING_UP:
        case MOVING_DOWN:
            handleMovementState();
            break;
            
        case STOP_AT_CURRENT:
            handleStopState();
            break;
    }
}
