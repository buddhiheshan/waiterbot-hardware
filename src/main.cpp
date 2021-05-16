#include <Arduino.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include <LiquidCrystal_I2C.h>

// UltraSound Pin Definitions
#define UltraSoundTrig 15
#define LeftUltraSoundEcho 27
#define RightUltraSoundEcho 14

#define soundSpeed 343.0
#define obstrucleDistance 50

// Wifi SSID and password config
const char *ssid = "Dialog 4G 255";
const char *password = "Dialog311/13Wifi";

// MQTT broker IP config
const char *mqtt_server = "192.168.8.197";

// Definition of the variables
int pollTime = 1000;
unsigned long lastMillis; // is uint32_t
volatile unsigned long startTime[2];
volatile long travelTime[2];
float distance[2];
boolean obstrucle = true;

// set the LCD number of columns and rows
int lcdColumns = 16;
int lcdRows = 2;

WiFiClient espClient;
PubSubClient client(espClient);

// set LCD address, number of columns and rows
// if you don't know your display address, run an I2C scanner sketch
LiquidCrystal_I2C lcd(0x27, lcdColumns, lcdRows);

void setup_wifi();
void reconnect();
void doDistanceMeasurement();
void stop_robot();
void IRAM_ATTR leftUltraSoundInterupt();
void IRAM_ATTR rightUltraSoundInterupt();
void ultrasoundInteruptHandler(bool pinState, int nIRQ);
void Task0code(void *pvParameters);
void printLCD(String message, int col, int row, boolean clearAndPrint);

void callback(char *topic, byte *message, unsigned int length)
{
  Serial.print("Message arrived on topic: ");
  Serial.print(topic);
  Serial.print(". Message: ");
  String messageTemp;

  for (int i = 0; i < length; i++)
  {
    Serial.print((char)message[i]);
    messageTemp += (char)message[i];
  }
  Serial.println();

  // Feel free to add more if statements to control more GPIOs with MQTT
  // Handling incoming mqtt msgs
}

void setup()
{
  // Define pin modes
  pinMode(UltraSoundTrig, OUTPUT);
  pinMode(LeftUltraSoundEcho, INPUT);
  pinMode(RightUltraSoundEcho, INPUT);

  // Set LeftUltraSoundEcho pin as interrupt, assign interrupt function and set CHANGE mode
  attachInterrupt(digitalPinToInterrupt(LeftUltraSoundEcho), leftUltraSoundInterupt, CHANGE);
  attachInterrupt(digitalPinToInterrupt(RightUltraSoundEcho), rightUltraSoundInterupt, CHANGE);

  Serial.begin(9600);

  // initialize LCD
  lcd.init();
  // turn on LCD backlight
  lcd.backlight();

  setup_wifi();
  client.setServer(mqtt_server, 1883);
  client.setCallback(callback);
}

void loop()
{
  // Serial.println("hello from esp32");
  if (!client.connected())
  {
    reconnect();
  }

  client.loop();

  // Has pollTime passed ?
  if (millis() - lastMillis > pollTime)
  {
    doDistanceMeasurement();

    Serial.print("Left = ");
    Serial.print(distance[0]);
    Serial.print(" Right = ");
    Serial.print(distance[1]);

    if (distance[0] <= obstrucleDistance || distance[1] <= obstrucleDistance)
    {
      obstrucle = true;
      printLCD("Please clear the path", 0, 0, true);
      stop_robot();
    }
    else
    {
      obstrucle = false;
    }

    lastMillis = millis();
  }

  if (!obstrucle)
  {
    printLCD("Ready to deliver...", 0, 0, true);
  }
}

void setup_wifi()
{
  delay(10);
  // We start by connecting to a WiFi network
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(ssid);

  printLCD("Connecting to ", 0, 0, true);
  printLCD((String)ssid, 0, 1, false);

  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED)
  {
    delay(500);
    Serial.print(".");
  }
  printLCD("WiFi connected.", 0, 0, true);

  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
}

// Interrupt handling for INT0 (pin 2 on Arduino Uno)
// Keep this as FAST and LIGHT (cpu load) as possible !
void IRAM_ATTR leftUltraSoundInterupt()
{
  byte pinRead = digitalRead(LeftUltraSoundEcho); // same as digitalRead(2) but faster
  ultrasoundInteruptHandler(pinRead, 0);
}

void IRAM_ATTR rightUltraSoundInterupt()
{
  byte pinRead = digitalRead(RightUltraSoundEcho); // same as digitalRead(2) but faster
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

void reconnect()
{
  // Loop until we're reconnected
  while (!client.connected())
  {
    Serial.print("Attempting MQTT connection...");
    // Attempt to connect
    if (client.connect("ESP8266Client"))
    {
      printLCD("MQTT connected.", 0, 0, true);
      Serial.println("connected");
      // Subscribe
      client.subscribe("robot1");
    }
    else
    {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      // Wait 5 seconds before retrying
      delay(5000);
    }
  }
}

void stop_robot()
{
  client.publish("robot1", "obstrucle detected");
}

void doDistanceMeasurement()
{
  // First measurement(s) will be 0
  noInterrupts(); // cli()
  distance[0] = travelTime[0] * (float)soundSpeed / 20000.0;
  distance[1] = travelTime[1] * (float)soundSpeed / 20000.0;
  interrupts(); // sei();

  // Set trigger signal ...
  digitalWrite(UltraSoundTrig, LOW);  // set pin 2 LOW
  delayMicroseconds(2);               // Wait 2µs
  digitalWrite(UltraSoundTrig, HIGH); // Set pin 2 HIGH
  delayMicroseconds(10);              // Wait 10µs
  digitalWrite(UltraSoundTrig, LOW);  // Set pin 2 low again
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
  // delay(1000);
  // // clears the display to print new message
  // lcd.clear();
}