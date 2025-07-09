#include <TinyGPSPlus.h>

#include <MFRC522Constants.h>
#include <MFRC522Debug.h>
#include <MFRC522Driver.h>
#include <MFRC522DriverI2C.h>
#include <MFRC522DriverPin.h>
#include <MFRC522DriverPinSimple.h>
#include <MFRC522DriverSPI.h>
#include <MFRC522Hack.h>
#include <MFRC522v2.h>
#include <require_cpp11.h>

#include <MFRC522v2.h>
#include <MFRC522DriverSPI.h>
#include <MFRC522DriverPinSimple.h>
#include <MFRC522Debug.h>
#include <WiFi.h>
#include <HTTPClient.h>
#include <TinyGPS++.h>

// WiFi credentials
const char* ssid = "bhihnu dh fibernet";
const char* password = "Nepal@123";

// Server URL
String serverName = "http://192.168.18.5:8000/ride/";
String cardIdServerName = "http://192.168.18.5:8000/check-card/";

// Timers
unsigned long lastTime = 0;
unsigned long timerDelay = 5000;

// Define the RX and TX pins for Serial 2
#define RXD2 16
#define TXD2 17
#define GPS_BAUD 9600

// Blue LED pin
#define BLUE_LED_PIN 2

// TinyGPS++ object
TinyGPSPlus gps;

// HardwareSerial instance for GPS
HardwareSerial gpsSerial(2);

// RFID reader configuration
MFRC522DriverPinSimple ss_pin(5);
MFRC522DriverSPI driver{ss_pin};
MFRC522 mfrc522{driver};

// Variable to store RFID card UUID
String cardUUID;

// Variables to store GPS coordinates
float latitude = 0.0;
float longitude = 0.0;

void setup() {
  // Initialize serial communication
  Serial.begin(115200);
  gpsSerial.begin(GPS_BAUD, SERIAL_8N1, RXD2, TXD2);
  pinMode(BLUE_LED_PIN, OUTPUT);
  digitalWrite(BLUE_LED_PIN, LOW);

  // Initialize RFID reader
  while (!Serial);
  mfrc522.PCD_Init();
  MFRC522Debug::PCD_DumpVersionToSerial(mfrc522, Serial);

  // Connect to WiFi
  WiFi.begin(ssid, password);
  Serial.println("Connecting to WiFi...");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nConnected to WiFi. IP Address: " + WiFi.localIP().toString());
}

void loop() {
  // Check for new RFID card
  if (mfrc522.PICC_IsNewCardPresent() && mfrc522.PICC_ReadCardSerial()) {
    // Turn on the blue LED
    digitalWrite(BLUE_LED_PIN, HIGH);
    delay(500);
    digitalWrite(BLUE_LED_PIN, LOW);

    // Store RFID card UUID
    cardUUID = "";
    for (byte i = 0; i < mfrc522.uid.size; i++) {
      cardUUID += String(mfrc522.uid.uidByte[i], HEX);
      if (i < mfrc522.uid.size - 1) cardUUID += ":";
    }
    Serial.println("RFID Card UUID: " + cardUUID);

    sendCardIdToServer(cardUUID);


    unsigned long start = millis();

  while (millis() - start < 1000) {
    while (gpsSerial.available() > 0) {
      gps.encode(gpsSerial.read());
    }
    if (gps.location.isUpdated()) {
      latitude = gps.location.lat();
      longitude = gps.location.lng();
      Serial.println(String(gps.date.year()) + "/" + String(gps.date.month()) + "/" + String(gps.date.day()) + "," + String(gps.time.hour()) + ":" + String(gps.time.minute()) + ":" + String(gps.time.second()));
      Serial.println("");
    }
  }
    // Read GPS data
    // if (gpsSerial.available() > 0) {
    //   gps.encode(gpsSerial.read());
    //   if (gps.location.isUpdated()) {
    //     latitude = gps.location.lat();
    //     longitude = gps.location.lng();
    //     Serial.println("Latitude: " + String(latitude, 6));
    //     Serial.println("Longitude: " + String(longitude, 6));
    //   } else {
    //     Serial.println("Invalid GPS data.");
    //   }
    // } else {
    //   Serial.println("No GPS data available.");
    // }

    // Send data to server
    sendToServer();
  }

  delay(1000);  // Delay for stability
}

void sendToServer() {
  if (WiFi.status() == WL_CONNECTED) {
    WiFiClient client;
    HTTPClient http;

    http.begin(client, serverName);

    // Prepare JSON payload
    String jsonPayload = "{\"card_id\":\"" + cardUUID + "\",\"latitude\":" + String(latitude, 6) + ",\"longitude\":" + String(longitude, 6) + "}";
    http.addHeader("Content-Type", "application/json");

    // Send POST request
    int httpResponseCode = http.POST(jsonPayload);

    // Debugging output
    Serial.print("HTTP Response Code: ");
    Serial.println(httpResponseCode);
    Serial.println("Payload: " + jsonPayload);
    if (httpResponseCode > 0) {
      String responseBody = http.getString();
      Serial.println("Response Body: " + responseBody);
    } else {
      Serial.println("Error sending request.");
    }
    http.end();
  } else {
    Serial.println("WiFi Disconnected. Unable to send data.");
  }
}

void sendCardIdToServer(String cardId) {
  if (WiFi.status() == WL_CONNECTED) {
    WiFiClient client;
    HTTPClient http;

    http.begin(client, cardIdServerName);

    // Prepare JSON payload
    String jsonPayload = "{\"card_id\":\"" + cardId + "\"}";
    http.addHeader("Content-Type", "application/json");

    // Send POST request
    int httpResponseCode = http.POST(jsonPayload);

    // Print response status and body
    Serial.print("Card ID HTTP Response Code: ");
    Serial.println(httpResponseCode);
    if (httpResponseCode > 0) {
      String responseBody = http.getString();
      Serial.println("Response Body: " + responseBody);
    } else {
      Serial.println("Error sending request.");
    }
    if (httpResponseCode != 200) {
    digitalWrite(BLUE_LED_PIN, HIGH);
    delay(500);
    digitalWrite(BLUE_LED_PIN, LOW);
    }

    http.end();
  } else {
    Serial.println("WiFi Disconnected. Unable to send data.");
  }
}