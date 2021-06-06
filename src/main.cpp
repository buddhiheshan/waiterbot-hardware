#include <Arduino.h>
#include <Wire.h>
#include <WiFi.h>
extern "C"
{
#include "freertos/FreeRTOS.h"
#include "freertos/timers.h"
}
#include <AsyncMqttClient.h>
#include <LiquidCrystal_I2C.h>
#include <HX711.h>

#define WIFI_SSID "Dialog 4G 255"
#define WIFI_PASSWORD "Dialog311/13Wifi"

// Raspberry Pi Mosquitto MQTT Broker
#define MQTT_HOST IPAddress(192, 168, 8, 197)
#define MQTT_PORT 1883
#define SUB_TOPIC "waiterbot/6012f05ad0a67143ebd177db"
#define PUB_TOPIC "operator/6012f05ad0a67143ebd177db"

#define lcdColumns 16
#define lcdRows 2

#define soundSpeed 343.0
#define obstrucleDistance 50

float calibration_factor = 410965;

// Pin Definitions
// BatteryLevel
#define BATTERY_LEVEL 33
// UltraSound Pin Definitions
#define ULTRASOUND_TRIG 15
#define LEFT_ULTRASOUND_ECHO 27
#define RIGHT_ULTRASOUND_ECHO 14
// Weight Sensor Pin Definitions
#define WEIGHT_DATA 2
#define WEIGHT_CLK 18
// IRSensors
#define IR0 26 //Front
#define IR1 25 //Left most
#define IR2 32
#define IR3 35
#define IR4 34
#define IR5 39
#define IR6 36 //Right most
// Motors
#define LEFT_FRONT_DIRECTION 23
#define LEFT_FRONT_STEP 19
// #define RIGHT_FRONT_DIRECTION 5
// #define RIGHT_FRONT_STEP 17
// #define LEFT_BACK_DIRECTION 12
// #define LEFT_BACK_STEP 13

#define STOPPED 0
#define FOLLOWING_LINE 1
#define NO_LINE 2

// Control Variables
boolean delivering = false;
boolean obstrucle = true;
boolean foodOnTray = false;
boolean mainRoad = true;
boolean delivered = false;
boolean atJunction = false;
int destinationJunctions;
int totalJunctions;
int returnJunctions;
int junctionCount;

// Variables
// Battery related variables
float batteryLevel;
int batteryPollTime = 10000;
unsigned long lastBatteryTime;
// Ulrasound related variables
int distancePollTime = 1000;
unsigned long lastDistanceTime;
volatile unsigned long startTime[2];
volatile long travelTime[2] = {10000, 10000};
float distance[2];
// Weight Related Variables
float initialWeight;
float minimumWeight = 0.1;
// IR sensor Related Variables
int IRSensor[7];
int error;
int mode;
// Stepper Related Varibles
const int stepsPerRevolution = 250;

AsyncMqttClient mqttClient;
TimerHandle_t mqttReconnectTimer;
TimerHandle_t wifiReconnectTimer;

// set LCD address, number of columns and rows
LiquidCrystal_I2C lcd(0x27, lcdColumns, lcdRows);

HX711 scale;

// Function Signatures
void connectToWifi();
void connectToMqtt();
void WiFiEvent(WiFiEvent_t event);
void onMqttConnect(bool sessionPresent);
void onMqttDisconnect(AsyncMqttClientDisconnectReason reason);
void onMqttSubscribe(uint16_t packetId, uint8_t qos);
void onMqttMessage(char *topic, char *payload, AsyncMqttClientMessageProperties properties, size_t len, size_t index, size_t total);
void onMqttPublish(uint16_t packetId);

void printLCD(String message, int col, int row, boolean clearAndPrint);

void doDistanceMeasurement();
void IRAM_ATTR leftUltraSoundInterupt();
void IRAM_ATTR rightUltraSoundInterupt();
void ultrasoundInteruptHandler(bool pinState, int nIRQ);

float measureWeight();

void getIRReadingAndError();

void getBatteryLevel();

void rotateStepper();

void setup()
{
  // Define pin modes
  pinMode(BATTERY_LEVEL, INPUT);
  pinMode(ULTRASOUND_TRIG, OUTPUT);
  pinMode(LEFT_ULTRASOUND_ECHO, INPUT);
  pinMode(RIGHT_ULTRASOUND_ECHO, INPUT);
  pinMode(IR0, INPUT);
  pinMode(IR1, INPUT);
  pinMode(IR2, INPUT);
  pinMode(IR3, INPUT);
  pinMode(IR4, INPUT);
  pinMode(IR5, INPUT);
  pinMode(IR6, INPUT);
  pinMode(LEFT_FRONT_STEP, OUTPUT);
  pinMode(LEFT_FRONT_DIRECTION, OUTPUT);
  // pinMode(RIGHT_FRONT_STEP, OUTPUT);
  // pinMode(RIGHT_FRONT_DIRECTION, OUTPUT);
  // pinMode(LEFT_BACK_STEP, OUTPUT);
  // pinMode(LEFT_BACK_DIRECTION, OUTPUT);

  Serial.begin(9600);
  Serial.println("\n########## Serial Monitor Started ##########\n");

  // initialize LCD
  lcd.init();
  // turn on LCD backlight
  lcd.backlight();
  printLCD("Welcome...", 0, 0, true);
  delay(1000);

  // Set LeftUltraSoundEcho pin as interrupt, assign interrupt function and set CHANGE mode
  attachInterrupt(digitalPinToInterrupt(LEFT_ULTRASOUND_ECHO), leftUltraSoundInterupt, CHANGE);
  attachInterrupt(digitalPinToInterrupt(RIGHT_ULTRASOUND_ECHO), rightUltraSoundInterupt, CHANGE);

  mqttReconnectTimer = xTimerCreate("mqttTimer", pdMS_TO_TICKS(2000), pdFALSE, (void *)0, reinterpret_cast<TimerCallbackFunction_t>(connectToMqtt));
  wifiReconnectTimer = xTimerCreate("wifiTimer", pdMS_TO_TICKS(2000), pdFALSE, (void *)0, reinterpret_cast<TimerCallbackFunction_t>(connectToWifi));

  WiFi.onEvent(WiFiEvent);

  mqttClient.onConnect(onMqttConnect);
  mqttClient.onDisconnect(onMqttDisconnect);
  mqttClient.onSubscribe(onMqttSubscribe);
  // mqttClient.onUnsubscribe(onMqttUnsubscribe);
  mqttClient.onMessage(onMqttMessage);
  mqttClient.onPublish(onMqttPublish);
  mqttClient.setServer(MQTT_HOST, MQTT_PORT);
  // If your broker requires authentication (username and password), set them below
  //mqttClient.setCredentials("REPlACE_WITH_YOUR_USER", "REPLACE_WITH_YOUR_PASSWORD");

  printLCD("Connecting to ", 0, 0, true);
  printLCD((String)WIFI_SSID, 0, 1, false);
  connectToWifi();
  while (WiFi.status() != WL_CONNECTED)
  {
    delay(500);
    Serial.print(".");
  }
  printLCD("WiFi connected.", 0, 0, true);
  while (mqttClient.connected() == false)
  {
    delay(500);
    Serial.print(".");
  }
  printLCD("MQTT connected.", 0, 0, true);
  delay(1000);

  scale.begin(WEIGHT_DATA, WEIGHT_CLK);
  scale.set_scale();
  scale.tare(); //Reset the scale to 0

  getBatteryLevel();
  delay(1000);
}

void loop()
{
  // if (millis() - lastBatteryTime > batteryPollTime)
  // {

  //   lastBatteryTime = millis();
  // }
  if (delivering)
  {
    // Has pollTime passed ?
    if (millis() - lastDistanceTime > distancePollTime)
    {
      doDistanceMeasurement();

      Serial.print("Left = ");
      Serial.print(distance[0]);
      Serial.print(" Right = ");
      Serial.println(distance[1]);

      if (distance[0] <= obstrucleDistance || distance[1] <= obstrucleDistance)
      {
        if (!obstrucle)
        {
          printLCD("Obstrucle Detected", 0, 0, false);
          mqttClient.publish(PUB_TOPIC, 1, false, "obstrucle");
          Serial.println("Obstrucle detected.");
        }
        obstrucle = true;
        // stop_robot();
      }
      else
      {
        obstrucle = false;
      }

      lastDistanceTime = millis();
    }
    if (measureWeight() < (initialWeight - 0.1))
    {
      if (foodOnTray)
      {
        Serial.println("Food stolen");
        printLCD("Food stolen", 0, 0, true);
        mqttClient.publish(PUB_TOPIC, 1, false, "stolen");
      }
      foodOnTray = false;
    }
    else if (!obstrucle)
    {
      foodOnTray = true;
      getIRReadingAndError();
      Serial.printf("%d %d %d %d %d %d %d\n", IRSensor[0], IRSensor[1], IRSensor[2], IRSensor[3], IRSensor[4], IRSensor[5], IRSensor[6]);
      Serial.println(error);

      switch (mode)
      {
      case STOPPED:
      {
        printLCD("JUNCTION/END", 0, 0, true);
        Serial.print("Junction ");
        Serial.println(junctionCount);
        if (!atJunction)
        {
          junctionCount++;
        }
        atJunction = true;

        if (junctionCount != destinationJunctions)
        {
          // move forward
          printLCD("Not correct junc", 0, 1, false);
          Serial.print("Not at correct junction");
          delay(5000);
          break;
        }
        if (mainRoad && delivered)
        {
          // at station
          // reset all
          // back to idle state
          // move forward
          delivering = false;
          delivered = false;
          printLCD("At station", 0, 1, false);
          Serial.print("At station");
          mqttClient.publish(PUB_TOPIC, 1, false, "ready");
          delay(5000);
          break;
        }
        if (mainRoad && (!delivered))
        {
          junctionCount = 0;
          destinationJunctions = 1;
          mainRoad = false;
          // turn direction
          // move forward
          printLCD("Correct junction", 0, 1, false);
          Serial.print("At correct junction.");
          delay(5000);
          break;
        }
        if ((!mainRoad) && (!delivered))
        {
          // wait till items take
          // send mqtt msg
          // turn 180
          // move forward
          printLCD("Delivery point", 0, 1, false);
          Serial.print("At delivery point");
          delay(5000);
          while (measureWeight() >= (initialWeight - 0.1))
          {
            Serial.println("Take food");
            printLCD("Take food.......", 0, 1, false);
          }
          printLCD("Enjoy your food.", 0, 1, false);
          mqttClient.publish(PUB_TOPIC, 1, false, "delivered");
          initialWeight = 0;
          destinationJunctions = 1;
          junctionCount = 0;
          delivered = true;

          delay(5000);
          break;
        }
        junctionCount = 0;
        destinationJunctions = returnJunctions;
        mainRoad = true;
        // turn
        // move forward
        printLCD("Return to main", 0, 1, false);
        Serial.print("Return to main road");
        delay(5000);
        break;
      }
      case NO_LINE:
        //
        printLCD("No line detected", 0, 0, true);
        Serial.println("No line");
        break;
      case FOLLOWING_LINE:
        printLCD("Follwing Line", 0, 0, true);
        String temp = "Error= " + (String)error;
        printLCD(temp, 0, 1, false);
        // rotateStepper();
        break;
      }
    }
  }
  else
  {
    initialWeight = measureWeight();
    if (initialWeight <= minimumWeight)
    {
      printLCD("Place food item.", 0, 0, false);
      Serial.println("No food on the tray");
      foodOnTray = false;
    }
    else
    {
      printLCD("Ready to deliver", 0, 0, false);
      Serial.println("Ready to deliver");
      foodOnTray = true;
    }
    // Serial.println("Ready to deliver");
  }
}

void connectToWifi()
{
  Serial.println("Connecting to Wi-Fi...");
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
}

void connectToMqtt()
{
  Serial.println("Connecting to MQTT...");
  mqttClient.connect();
}

void WiFiEvent(WiFiEvent_t event)
{
  // Serial.printf("[WiFi-event] event: %d\n", event);
  switch (event)
  {
  case SYSTEM_EVENT_STA_GOT_IP:
    Serial.println("WiFi connected");
    Serial.print("IP address: ");
    Serial.println(WiFi.localIP());
    connectToMqtt();
    break;
  case SYSTEM_EVENT_STA_DISCONNECTED:
    Serial.println("WiFi lost connection");
    xTimerStop(mqttReconnectTimer, 0); // ensure we don't reconnect to MQTT while reconnecting to Wi-Fi
    xTimerStart(wifiReconnectTimer, 0);
    break;
  }
}

void onMqttConnect(bool sessionPresent)
{
  Serial.println("Connected to MQTT.");
  Serial.print("Session present: ");
  Serial.println(sessionPresent);
  mqttClient.subscribe(SUB_TOPIC, 1);
}

void onMqttDisconnect(AsyncMqttClientDisconnectReason reason)
{
  Serial.println("Disconnected from MQTT.");
  if (WiFi.isConnected())
  {
    xTimerStart(mqttReconnectTimer, 0);
  }
}

void onMqttSubscribe(uint16_t packetId, uint8_t qos)
{
  Serial.println("Subscribe acknowledged.");
  Serial.print("  packetId: ");
  Serial.println(packetId);
  Serial.print("  qos: ");
  Serial.println(qos);
}
// void onMqttUnsubscribe(uint16_t packetId)
// {
//   Serial.println("Unsubscribe acknowledged.");
//   Serial.print("  packetId: ");
//   Serial.println(packetId);
// }

void onMqttMessage(char *topic, char *payload, AsyncMqttClientMessageProperties properties, size_t len, size_t index, size_t total)
{
  Serial.println("Publish received.");
  Serial.print("  topic: ");
  Serial.println(topic);
  Serial.print("  qos: ");
  Serial.println(properties.qos);
  Serial.print("  dup: ");
  Serial.println(properties.dup);
  Serial.print("  retain: ");
  Serial.println(properties.retain);
  Serial.print("  len: ");
  Serial.println(len);
  Serial.print("  index: ");
  Serial.println(index);
  Serial.print("  total: ");
  Serial.println(total);
  Serial.println((String)payload);
  char *token = strtok(payload, " ");

  if (!strcmp(token, "deliver"))
  {
    token = strtok(NULL, " ");
    destinationJunctions = atoi(token);

    token = strtok(NULL, " ");
    totalJunctions = atoi(token);

    returnJunctions = totalJunctions - destinationJunctions;
    junctionCount = 0;

    Serial.printf("%d, %d, %d, %d\n", destinationJunctions, totalJunctions, returnJunctions, junctionCount);

    initialWeight = measureWeight();

    if (foodOnTray)
    {
      delivering = true;
      mqttClient.publish(PUB_TOPIC,1, false, "delivering");
    }
    else{
      mqttClient.publish(PUB_TOPIC,1, false, "empty");
    }
  }
}

void onMqttPublish(uint16_t packetId)
{
  Serial.print("Publish acknowledged.");
  Serial.print("  packetId: ");
  Serial.println(packetId);
}

void printLCD(String message, int col, int row, boolean clearAndPrint)
{

  if (clearAndPrint)
  {
    lcd.clear();
  }
  // set cursor to first column, first row
  lcd.setCursor(col, row);
  // print message
  lcd.print(message);
}

void doDistanceMeasurement()
{
  // First measurement(s) will be 0
  noInterrupts(); // cli()
  distance[0] = travelTime[0] * (float)soundSpeed / 20000.0;
  distance[1] = travelTime[1] * (float)soundSpeed / 20000.0;
  interrupts(); // sei();

  // Set trigger signal ...
  digitalWrite(ULTRASOUND_TRIG, LOW);  // set pin 2 LOW
  delayMicroseconds(2);                // Wait 2µs
  digitalWrite(ULTRASOUND_TRIG, HIGH); // Set pin 2 HIGH
  delayMicroseconds(10);               // Wait 10µs
  digitalWrite(ULTRASOUND_TRIG, LOW);  // Set pin 2 low again
}

// Interrupt handling for INT0 (pin 2 on Arduino Uno)
// Keep this as FAST and LIGHT (cpu load) as possible !
void IRAM_ATTR leftUltraSoundInterupt()
{
  byte pinRead = digitalRead(LEFT_ULTRASOUND_ECHO); // same as digitalRead(2) but faster
  ultrasoundInteruptHandler(pinRead, 0);
}

void IRAM_ATTR rightUltraSoundInterupt()
{
  byte pinRead = digitalRead(RIGHT_ULTRASOUND_ECHO); // same as digitalRead(2) but faster
  ultrasoundInteruptHandler(pinRead, 1);
}

void ultrasoundInteruptHandler(bool pinState, int nIRQ)
{
  unsigned long currentTime = micros(); // Get current time (in µs)
  if (pinState)
  {
    // If pin state has changed to HIGH -> remember start time (in µs)
    startTime[nIRQ] = currentTime;
  }
  else
  {
    // If pin state has changed to LOW -> calculate time passed (in µs)
    travelTime[nIRQ] = currentTime - startTime[nIRQ];
  }
}

float measureWeight()
{
  scale.set_scale(calibration_factor); //Adjust to this calibration factor

  float weight = scale.get_units(1);

  Serial.print("Weight: ");
  Serial.print(weight);
  Serial.println(" KG");
  Serial.println();

  return weight;
}

void getIRReadingAndError()
{
  IRSensor[0] = digitalRead(IR0);
  IRSensor[1] = digitalRead(IR1);
  IRSensor[2] = digitalRead(IR2);
  IRSensor[3] = digitalRead(IR3);
  IRSensor[4] = digitalRead(IR4);
  IRSensor[5] = digitalRead(IR5);
  IRSensor[6] = digitalRead(IR6);

  if ((IRSensor[0] == 0) && (IRSensor[1] == 0) && (IRSensor[2] == 0) && (IRSensor[3] == 0) && (IRSensor[4] == 0) && (IRSensor[5] == 0) && (IRSensor[6] == 1))
  {
    error = 4;
    mode = FOLLOWING_LINE;
  }

  else if ((IRSensor[0] == 0) && (IRSensor[1] == 0) && (IRSensor[2] == 0) && (IRSensor[3] == 0) && (IRSensor[4] == 0) && (IRSensor[5] == 1) && (IRSensor[6] == 1))
  {
    error = 3;
    mode = FOLLOWING_LINE;
  }

  else if ((IRSensor[0] == 0) && (IRSensor[1] == 0) && (IRSensor[2] == 0) && (IRSensor[3] == 0) && (IRSensor[4] == 1) && (IRSensor[5] == 1) && (IRSensor[6] == 0))
  {
    error = 2;
    mode = FOLLOWING_LINE;
  }

  else if ((IRSensor[0] == 1) && (IRSensor[1] == 0) && (IRSensor[2] == 0) && (IRSensor[3] == 0) && (IRSensor[4] == 1) && (IRSensor[5] == 1) && (IRSensor[6] == 0))
  {
    error = 1;
    atJunction = false;
    mode = FOLLOWING_LINE;
  }

  else if ((IRSensor[0] == 1) && (IRSensor[1] == 0) && (IRSensor[2] == 0) && (IRSensor[3] == 1) && (IRSensor[4] == 1) && (IRSensor[5] == 0) && (IRSensor[6] == 0))
  {
    error = 0;
    atJunction = false;
    mode = FOLLOWING_LINE;
  }

  else if ((IRSensor[0] == 1) && (IRSensor[1] == 0) && (IRSensor[2] == 1) && (IRSensor[3] == 1) && (IRSensor[4] == 0) && (IRSensor[5] == 0) && (IRSensor[6] == 0))
  {
    error = -1;
    atJunction = false;
    mode = FOLLOWING_LINE;
  }

  else if ((IRSensor[0] == 0) && (IRSensor[1] == 0) && (IRSensor[2] == 1) && (IRSensor[3] == 1) && (IRSensor[4] == 0) && (IRSensor[5] == 0) && (IRSensor[6] == 0))
  {
    error = -2;
    mode = FOLLOWING_LINE;
  }

  else if ((IRSensor[0] == 0) && (IRSensor[1] == 1) && (IRSensor[2] == 1) && (IRSensor[3] == 0) && (IRSensor[4] == 0) && (IRSensor[5] == 0) && (IRSensor[6] == 0))
  {
    error = -3;
    mode = FOLLOWING_LINE;
  }

  else if ((IRSensor[0] == 0) && (IRSensor[1] == 1) && (IRSensor[2] == 0) && (IRSensor[3] == 0) && (IRSensor[4] == 0) && (IRSensor[5] == 0) && (IRSensor[6] == 0))
  {
    error = -4;
    mode = FOLLOWING_LINE;
  }
  else if ((IRSensor[0] == 1) && (IRSensor[1] == 1) && (IRSensor[2] == 1) && (IRSensor[3] == 1) && (IRSensor[4] == 1) && (IRSensor[5] == 1) && (IRSensor[6] == 1))
  {
    error = -4;
    mode = STOPPED;
  }
  else if ((IRSensor[0] == 0) && (IRSensor[1] == 0) && (IRSensor[2] == 0) && (IRSensor[3] == 0) && (IRSensor[4] == 0) && (IRSensor[5] == 0) && (IRSensor[6] == 0))
  {
    error = 0;
    mode = NO_LINE;
  }
  else
  {
    error = 0;
    mode = FOLLOWING_LINE;
  }
}

void getBatteryLevel()
{
  batteryLevel = map(analogRead(BATTERY_LEVEL), 0, 4095, 0, 100);
  Serial.print("Battery Level = ");
  Serial.println(batteryLevel);
  String temp = "Battery = " + (String)batteryLevel + "%";
  printLCD(temp, 0, 1, false);
}

void rotateStepper()
{
  // Set motor direction clockwise
  digitalWrite(LEFT_FRONT_DIRECTION, HIGH);
  // digitalWrite(RIGHT_FRONT_DIRECTION, HIGH);
  // digitalWrite(LEFT_BACK_STEP, HIGH);

  // Spin motor slowly
  for (int x = 0; x < stepsPerRevolution; x++)
  {
    digitalWrite(LEFT_FRONT_STEP, HIGH);
    // digitalWrite(RIGHT_FRONT_STEP, HIGH);
    // digitalWrite(LEFT_BACK_STEP, HIGH);
    delayMicroseconds(1000);
    digitalWrite(LEFT_FRONT_STEP, LOW);
    // digitalWrite(RIGHT_FRONT_STEP, LOW);
    // digitalWrite(LEFT_BACK_STEP, LOW);
    delayMicroseconds(1000);
  }
}