#define BLYNK_PRINT Serial

#include "DHT.h"
#include <String.h>
#include <ESP8266_Lib.h>
#include <BlynkSimpleShieldEsp8266.h>
#include <SoftwareSerial.h>

#define ESP8266_BAUD 112500
#define DHTPIN 2
#define DHTTYPE DHT11
#define Atomizer 3

char auth[] = "ugjTpyeW_fNXwQJWf8WLAMG0UbdElKk-";
char ssid[] = "DODO-2814";
char pass[] = "WH6SH8NMTR";

int Pump = 11;
int leftLED = 8;
int rightLED = 6;
int buttonPressCount = 0;
const int mistButton = 7;

boolean buttonState = LOW;
boolean lastButtonState = LOW;
boolean currentButtonState = LOW;

const unsigned long pumpRunTime = 35000; //Time to fill diffuser
unsigned long lastDebounceTime = 0;
unsigned long debounceDelay = 20;
unsigned long pumpStart;
unsigned long pumpRun;
int mode;
int pumpStatus;

SoftwareSerial EspSerial(9, 10); // RX, TX

ESP8266 wifi(&EspSerial);

DHT dht(DHTPIN, DHTTYPE);
BlynkTimer timer;

void sendSensor() // Send humidity and temperature signals to blynk
{
  float h = dht.readHumidity();
  float t = dht.readTemperature(); // 
  if (isnan(h) || isnan(t)) {
    Serial.println("FaileftLED to read from DHT sensor!");
    return;
  }
  // You can send any value at any time.
  // Please don't send more that 10 values per second.
  Blynk.virtualWrite(V5, h);
  Blynk.virtualWrite(V6, t);
}

void connection_check() // Reconnect app
{
  if (Blynk.connected() == true) {              // if blynk is connected
    Blynk.run();                                // runs blynk communication
    Serial.println("Blynk Connected");
  }         //

  if (Blynk.connected() == false) {             // if blynk is not connected
    Serial.println("Reconnecting...");          //
    Blynk.connect();                            // trying to connect to led
  }
}

void setup() {
  Serial.begin(115200);
  EspSerial.begin(ESP8266_BAUD);
  
  if (wifi.joinAP(ssid, pass)) 
  {
    Serial.println("Connect to WiFi OK");
    Serial.print("IP: ");
    Serial.println(wifi.getLocalIP().c_str());
  } 
  else 
  {
    Serial.println("Connect to WiFi failed");
  }
  delay(10);
  Blynk.begin(auth, wifi, ssid, pass);
  pinMode(Pump, OUTPUT);
  pinMode(leftLED, OUTPUT);
  pinMode(rightLED, OUTPUT);
  pinMode(Atomizer, OUTPUT);
  pinMode(mistButton, INPUT_PULLUP);
  dht.begin();


  timer.setInterval(1000L, sendSensor);
  timer.setInterval(5000L, connection_check);
}

void FieldMist() // Enables diffuser to be run by button click
{
  currentButtonState = digitalRead(mistButton);
  if (currentButtonState != lastButtonState) {
    lastDebounceTime = millis();
  }
  if ((millis() - lastDebounceTime) > debounceDelay)
    delay(20);
  {
    if (currentButtonState != buttonState)
    {
      buttonState = currentButtonState;

      if (buttonState == LOW)
      {
        buttonPressCount++;
        Serial.println(buttonPressCount);
        if (buttonPressCount >= 0) { // Put buttin into "on" state
          digitalWrite(Atomizer, HIGH);
          digitalWrite(leftLED, HIGH);
          digitalWrite(rightLED, HIGH);
        }
        if (buttonPressCount == 2) // Put button back in "off" state
        {
          digitalWrite(Atomizer, LOW);
          digitalWrite(leftLED, LOW);
          digitalWrite(rightLED, LOW);
          buttonPressCount = 0;
        }
      }
    }
  }
}


void loop() {

  FieldMist();

  Blynk.run();
  timer.run();

  pumpStatus = digitalRead(Pump);
  if (pumpStatus == 0) {
    pumpStart = millis();
  }
  if ((millis() - pumpStart) >= pumpRunTime) // If pump goes beyond fill time, indicate that diffuser is full by flashing lights
  {
    digitalWrite(leftLED, HIGH);
    digitalWrite(rightLED, HIGH);
    delay(500);
    digitalWrite(leftLED, LOW);
    digitalWrite(rightLED, LOW);
    delay(500);
    digitalWrite(leftLED, HIGH);
    digitalWrite(rightLED, HIGH);
    delay(500);
    digitalWrite(leftLED, LOW);
    digitalWrite(rightLED, LOW);
    delay(500);
    digitalWrite(leftLED, HIGH);
    digitalWrite(rightLED, HIGH);
    delay(500);
    digitalWrite(leftLED, LOW);
    digitalWrite(rightLED, LOW);
    digitalWrite(Pump, LOW);
  }

  if (Blynk.connected() == false)
  {
    Serial.println("Field Mode Forced");
    FieldMist();
    delay(50);
  }
}

BLYNK_WRITE(V4) {
  float h = dht.readHumidity();
  int modeButton = param.asInt();
  if (modeButton == 1) { // Run humidifer in manual 
    Serial.println("Manual Mode Selected");
  }
  else { // Humidifier will run based on humidity
    if (h > 80) {
      digitalWrite(Atomizer, LOW);
      digitalWrite(leftLED, LOW);
      digitalWrite(rightLED, LOW);
    }
    if (h < 80) {
      digitalWrite(Atomizer, HIGH);
      digitalWrite(leftLED, HIGH);
      digitalWrite(rightLED, HIGH);
    }

  }
}

BLYNK_WRITE(V2) {
  int diffuserSwitch = param.asInt();
  if (diffuserSwitch == 1) {
    Serial.println("Diffuser Running");
    digitalWrite(Atomizer, HIGH); // Diffuser On
    digitalWrite(leftLED, HIGH);
    digitalWrite(rightLED, HIGH);
  }
  else {
    digitalWrite(Atomizer, LOW); // Diffuser Off
    digitalWrite(leftLED, LOW);
    digitalWrite(rightLED, LOW);
  }
}


BLYNK_WRITE(V3) {
  int button = param.asInt(); // Assigning incoming value from pin V3 to a variable
  if (button == 1) {
    Serial.println("Pump Running");
    digitalWrite(Pump, HIGH); // Pump On
  }
  else {
    digitalWrite(Pump, LOW); // Pump Off
  }
  delay(100);

}
