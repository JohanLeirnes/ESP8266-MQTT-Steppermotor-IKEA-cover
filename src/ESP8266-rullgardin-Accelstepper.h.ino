
#define IDLE_WAIT 1
#define MOVING_UP 2
#define MOVING_DOWN 3
#define STOP_AT_CURRENT 4
#define SET_ENDSTOPS 5

#include "OneButton.h"
#include <ESP8266WiFi.h>
#include <PubSubClient.h>

#define SW_ACT_PIN 2  // hook up this pin to VCC
OneButton ButtonAct(SW_ACT_PIN, 0);

const char* ssid = "SSID";
const char* password = "SSID_PW";
const char* mqtt_server = "ADDRESS_TO_MQTT_SERVER";
const char* MQTT_USER = "MQTT_USER";
const char* MQTT_PASSWORD = "MQTT_PASSWORD";

WiFiClient espClient;
PubSubClient client(espClient);
long lastMsg = 0;
char msg[50];
int value = 0;

const char* outTopic = "cover-1/out"; //change to whatever you would like (needs to be different for every cover)
const char* inTopic = "cover-1/in"; //change to whatever you would like (needs to be different for every cover)

int maxPos = 0;
int minPos = 0;
int set_moving_up = 0;
int set_moving_down = 0;
int done_down = 0;
int done_up = 0;

#include <AccelStepper.h>
// Define a stepper and the pins it will use
AccelStepper stepper(AccelStepper::DRIVER, 13, 12); // to stepperdriver. pin 13 to STEP and pin 12 to DIR
// Enable pin for the stepper driver
const int enPin = 14; // Pin 14 to stepperdriver Enable pin
const int relayPin = 16; // when this is low the coils of the stepper motor is shortcircuited and when this is high the steppermotor is connected to the stepperdriver.

byte state;

void setup_wifi() {

  delay(10);
  // We start by connecting to a WiFi network
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(ssid);

  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
}

void callback(char* topic, byte* payload, unsigned int length) {
  Serial.print("Message arrived [");
  Serial.print(topic);
  Serial.print("] ");
  for (int i = 0; i < length; i++) {
    Serial.print((char)payload[i]);
  }
  Serial.println();


  if ((char)payload[0] == '0') {
    moveup();
    Serial.println("Opening cover");
    client.publish(outTopic, "open");
  } else if ((char)payload[0] == '1') {
    movedown();
    Serial.println("Closing cover");
    client.publish(outTopic, "closed");
  } else if ((char)payload[0] == '2') {
    movestop();
    Serial.println("stopped movement");
    if(stepper.currentPosition() >= maxPos/2){
    client.publish(outTopic, "closed");
    } else {client.publish(outTopic, "open");
    }
  }
}

void reconnect() {
  // Loop until we're reconnected
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    // Attempt to connect
    String clientId = "ESP8266Client-";
    clientId += String(random(0xffff), HEX);
    if (client.connect(clientId.c_str(), MQTT_USER, MQTT_PASSWORD)) {
      Serial.println("connected");
      // Once connected, publish an announcement...
      client.publish(outTopic, "Rullgardin-1 booted");
      // ... and resubscribe
      client.subscribe(inTopic);
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      // Wait 5 seconds before retrying
      delay(5000);
    }
  }
}

void setup()
{
  Serial.begin(115200);
  stepper.setMaxSpeed(8000.0);
  stepper.setAcceleration(10000.0);
//  stepper.moveTo(51200);
  pinMode(enPin,OUTPUT);
  pinMode(relayPin,OUTPUT)
  ButtonAct.setClickTicks(600); //600 default (debounce 60)
  ButtonAct.setPressTicks(1000); // 1000 default
  state = IDLE_WAIT;
  setup_wifi();
  client.setServer(mqtt_server, 1883);
  client.setCallback(callback);
  Serial.println("Idle, waiting for command");
}
void loop()
{
  if(state == IDLE_WAIT){
    if(stepper.distanceToGo() == 0){
      digitalWrite(enPin,HIGH);
      delay(15)
      digitalWrite(relayPin,LOW);
    }
    if(stepper.isRunning() == true){
      ButtonAct.attachClick(movestop);
    }else if(maxPos == stepper.currentPosition() && stepper.isRunning() == false){
      digitalWrite(relayPin,HIGH);
      delay(15)
      ButtonAct.attachClick(moveup);
    }else if(stepper.isRunning() == false){
      digitalWrite(relayPin,HIGH);
      delay(15)
      ButtonAct.attachClick(movedown);
    }
    if(stepper.currentPosition() == minPos){
      ButtonAct.attachDoubleClick(buttonDisabled);
    }else{
      ButtonAct.attachDoubleClick(moveup);
    }
    ButtonAct.attachLongPressStart(setendstop);

    // disable other functions
    ButtonAct.attachDuringLongPress(buttonDisabled);
    ButtonAct.attachLongPressStop(buttonDisabled);
  }
  if(state == SET_ENDSTOPS){
    if(set_moving_down == 1 && stepper.distanceToGo() == 0){
    digitalWrite(relayPin,HIGH);
    delay(15)
    digitalWrite(enPin,LOW);
    stepper.setMaxSpeed(2000.0);
    //stepper.setAcceleration(40000.0);
    stepper.move(1000000);
    ButtonAct.attachClick(donedown);
    }else if(set_moving_down == 1 && set_moving_up == 0 && done_down == 1){
    stepper.stop();
    set_moving_down = 0;
    maxPos = stepper.currentPosition();
    Serial.println("Max position is: " + String(maxPos));
    set_moving_up = 1;
    }else if(set_moving_up == 1 && set_moving_down == 0 && stepper.distanceToGo() == 0){
    stepper.setMaxSpeed(2000.0);
    //stepper.setAcceleration(40000.0);
    stepper.move(-1000000);
    ButtonAct.attachClick(doneup);
    }else if(set_moving_down == 0 && set_moving_up == 1 && done_up == 1){
    stepper.stop();
    set_moving_up = 0;
    minPos = stepper.currentPosition();
    Serial.println("Min position is: " + String(minPos));
    stepper.setMaxSpeed(8000.0);
    stepper.setAcceleration(8000.0);
    state = IDLE_WAIT;
    Serial.println("Idle, waiting for command");
    }
    ButtonAct.attachLongPressStart(buttonDisabled);
    ButtonAct.attachDoubleClick(buttonDisabled);
    ButtonAct.attachDuringLongPress(buttonDisabled);
    ButtonAct.attachLongPressStop(buttonDisabled);
  }
  if(state == MOVING_DOWN){
    if (stepper.distanceToGo() == 0){
      digitalWrite(enPin,LOW);
      stepper.moveTo(maxPos);
    }
    Serial.println("Moving to maximum position");
    state = IDLE_WAIT;
    client.publish(outTopic, "closed");
    Serial.println("Idle, waiting for command");
  }
  if(state == MOVING_UP){
    if (stepper.distanceToGo() == 0){
      digitalWrite(enPin,LOW);
      stepper.moveTo(minPos);
    }
    Serial.println("Moving to minimum position");
    state = IDLE_WAIT;
    client.publish(outTopic, "open");
    Serial.println("Idle, waiting for command");
  }
  if(state == STOP_AT_CURRENT){
    stepper.stop();
    Serial.println("Stopping at: " + String(stepper.currentPosition()));
    state = IDLE_WAIT;
    if(stepper.currentPosition() >= maxPos/2){
    client.publish(outTopic, "closed");
    } else {client.publish(outTopic, "open");
    Serial.println("Idle, waiting for command");
  }
  if (!client.connected()) {
    reconnect();
  }
  client.loop();
  ButtonAct.tick();
  stepper.run();
}

void moveup() {state = MOVING_UP;}
void movedown() {state = MOVING_DOWN;}
void movestop() {state = STOP_AT_CURRENT;}
void setendstop() {
  state = SET_ENDSTOPS;
  set_moving_down = 1;
  set_moving_up = 0;
}
void donedown() {done_down = 1;}
void doneup() {done_up = 1;}

void buttonDisabled()
{
  // do nothing
}
