#include <RadioLib.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP280.h>
#include <SPI.h>
#include <Wire.h>
#include <SD.h>
#include <TinyGPS++.h>
#include <HardwareSerial.h>
#include "DHT.h"

#define DHTPIN 7 // DATA Pin
#define DHTTYPE DHT11

DHT dht(DHTPIN, DHTTYPE);


HardwareSerial gpsSerial(1);
TinyGPSPlus gps;

// -----------------------------
// BMP280
// -----------------------------

Adafruit_BMP280 bmp;

// -----------------------------
// LoRa pins
// -----------------------------

#define LORA_SCK  9
#define LORA_MISO 11
#define LORA_MOSI 10
#define LORA_NSS  8
#define LORA_RST  12
#define LORA_BUSY 13
#define LORA_DIO1 14
#define VEXT_CTRL 45

SX1262 radio = new Module(LORA_NSS, LORA_DIO1, LORA_RST, LORA_BUSY);

// -----------------------------
// Camera + SD
// -----------------------------

#define CS_SD  4

// -----------------------------
// Scheduler
// -----------------------------

struct Task {
  unsigned long interval;
  unsigned long lastRun;
};

Task telemetryTask = {1000, 0};
Task photoTask = {2500, 0};

// -----------------------------

bool spiBusy = false;
int imgID = 0;
unsigned long missionStart;

// -----------------------------
// SPI lock
// -----------------------------

bool lockSPI() {
  if (spiBusy) return false;
  spiBusy = true;
  return true;
}

void unlockSPI() {
  spiBusy = false;
}

// -----------------------------
// Scheduler runner
// -----------------------------

void runTask(Task &task, void (*func)()) {

  if (millis() - task.lastRun >= task.interval) {
    task.lastRun = millis();
    func();
  }

}

// ------------------------------------------------

void setup() {

  Serial.begin(115200);
  delay(1000);

  missionStart = millis();

  // Power
  pinMode(VEXT_CTRL, OUTPUT);
  digitalWrite(VEXT_CTRL, LOW);

  pinMode(2, OUTPUT);
  digitalWrite(2, HIGH);

  // SPI
  SPI.begin(LORA_SCK, LORA_MISO, LORA_MOSI);
  SPI.setFrequency(8000000);

  // I2C
  Wire.begin(40, 41);

  gpsSerial.begin(9600, SERIAL_8N1, 42, 45);

  // -----------------------------
  // BMP280
  // -----------------------------

  if (!bmp.begin(0x76)) {
    Serial.println("BMP280 no encontrado");
    //while (1);
  } else Serial.println("BMP280 OK");

  dht.begin();

  // -----------------------------
  // LoRa
  // -----------------------------

  Serial.println("Inicializando LoRa...");

  int state = radio.begin(868.0);

  if (state != RADIOLIB_ERR_NONE) {
    Serial.println("Error LoRa");
    while (true);
  }

  radio.setDio2AsRfSwitch(true);
  radio.setTCXO(1.8);

  radio.setOutputPower(22);
  radio.setSpreadingFactor(10);
  radio.setBandwidth(125.0);
  radio.setCodingRate(6);
  radio.setPreambleLength(8);
  radio.setCRC(true);
  radio.setSyncWord(0xF2);

  Serial.println("LoRa OK");

  // -----------------------------
  // SD
  // -----------------------------

  Serial.println("Inicializando SD...");

  pinMode(CS_SD, OUTPUT);

  if (!SD.begin(CS_SD)) {
    Serial.println("Fallo SD");
    //while (1);
  } else Serial.println("SD OK");
}

// ------------------------------------------------

void loop() {
  GPS();
  runTask(telemetryTask, enviarTelemetria);

}

// ------------------------------------------------
// GPS
// ------------------------------------------------
void GPS() {
  while (gpsSerial.available())
  {
    char data;
    data = gpsSerial.read();
    gps.encode(data);
    Serial.print(data);
  }
}

struct __attribute__((packed)) Payload {
  float temp;
  float pressure;
  float altitud;
  float humedad;
  float lat;
  float lon;
  bool gpsStatus;
};

void enviarTelemetria() {

  if (!lockSPI()) return;

  float temp = bmp.readTemperature();
  float pressure = bmp.readPressure() / 100.0;
  float humedad = dht.readHumidity();

  float presionNorm = pressure / 1018.0;
  float altitud = (44330.0 * (1 - pow(presionNorm, 0.190284)));

  float lat = 0;
  float lon = 0;
  bool gS = false; 

  if (gps.location.isValid()) {
    lat = gps.location.lat();
    lon = gps.location.lng();
    gS = true;
  } 

  Payload data;

  data.temp = temp;
  data.pressure = pressure;
  data.altitud = altitud;
  data.humedad = humedad;
  data.lat = lat;
  data.lon = lon;
  data.gpsStatus = gS;

  int state = radio.transmit((uint8_t*)&data, sizeof(data));

  if (state != RADIOLIB_ERR_NONE) {
    Serial.print("Error LoRa: ");
    Serial.println(state);
  }

  unlockSPI();
}
