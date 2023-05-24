#include <SoftwareSerial.h>
#include <ESP8266WiFi.h>
#include <ESP8266HTTPClient.h>
#include <WiFiClient.h>

EspSoftwareSerial::UART STM;

// const char* ssid = "Byte's iPhone";
// const char* pass = "11119999";

const char* ssid = "aisfibre_2.4G_9809b2";
const char* pass = "549809b2";

String url = "http://144.24.138.249:5000/nodemcu";

int SENDING_INTERVAL_MS = 30*1000;
int FETCHING_INTERVAL_MS = 5*1000;

long last = -999999;
long lastFetch = -999999;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  STM.begin(9600, EspSoftwareSerial::SWSERIAL_8N1, D2, D3, false, 95, 11);
  WiFi.begin(ssid, pass);

  pinMode(D0, OUTPUT);
  digitalWrite(D0, HIGH);

  Serial.println("");
  Serial.print("Connecting...");
  while (WiFi.status() != WL_CONNECTED) {
    Serial.print(".");
    delay(1000);
  }
  Serial.println("");

  Serial.println("Wi-Fi connected.");
  Serial.print("IP Address : ");
  Serial.println(WiFi.localIP());
  while (STM.available()) STM.read();
}

unsigned int rightRotate(unsigned int n, unsigned int d) {
  return (n >> d) | (n << (32 - d));
}

int readInt() {
  unsigned int value = 0;
  for (int i = 0; i < 4; i++) {
    while (!STM.available())
      ;  // poll input until we get a byte
    // value <<= 8;
    value |= STM.read();            // OR in one byte.
    value = rightRotate(value, 8);  // shift 8 bits
  }
  return (int)value;
}

void loop() {
  // put your main code here, to run repeatedly:
  long now = millis();
  if (now - last >= SENDING_INTERVAL_MS) {
    STM.print("x");
    Serial.println("Sending message...");
    last = now;
  }

  if (STM.available() > 0) {
    int dust = readInt();
    int light = readInt();
    int temp = readInt();
    int humid = readInt();

    // // say what you got:
    Serial.printf("From STM: %d, %d, %d, %d (%d) + sending\n\r", dust, light, temp, humid, now);

    WiFiClient client;
    HTTPClient http;
    String data = String("") + "dust=" + String(dust) + "&"
                  + "light=" + String(light) + "&"
                  + "temp=" + String(temp) + "&"
                  + "humid=" + String(humid);

    http.begin(client, url);
    http.addHeader("Content-Type", "application/x-www-form-urlencoded");
    int code = http.POST(data);
    String content = http.getString();
    Serial.println(code);
    Serial.println(content);
    http.end();
  }

  if (now - lastFetch >= FETCHING_INTERVAL_MS && now-last >= 1000){
    WiFiClient client;
    HTTPClient http;
    http.begin(client, url);
    int code = http.GET();
    String content = http.getString();
    Serial.println(code);
    Serial.println(content);
    bool on = !(content == String("1"));
    digitalWrite(D0, on);
    http.end();
    lastFetch = now;
  }
}
