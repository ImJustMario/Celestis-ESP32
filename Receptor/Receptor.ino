#include <RadioLib.h>
#include <ArduinoJson.h>

#define LORA_SCK  9
#define LORA_MISO 11
#define LORA_MOSI 10
#define LORA_NSS  8
#define LORA_RST  12
#define LORA_BUSY 13
#define LORA_DIO1 14
#define VEXT_CTRL 45

volatile bool receivedFlag = false;

SX1262 radio = new Module(LORA_NSS, LORA_DIO1, LORA_RST, LORA_BUSY);

void setFlag(void) {
  receivedFlag = true;
}

JsonDocument doc;


void setup() {
  Serial.begin(115200);
   pinMode(VEXT_CTRL, OUTPUT);
  digitalWrite(VEXT_CTRL, LOW);
  pinMode(2, OUTPUT);
  digitalWrite(2, HIGH);
  delay(500);
  SPI.begin(LORA_SCK, LORA_MISO, LORA_MOSI, LORA_NSS);
  while (!Serial); 

  Serial.print(F("[SX1262 Receptor] Inicializando... "));

  int state = radio.begin(868.0);

  if (state != RADIOLIB_ERR_NONE) {
     Serial.println("Error al iniciar");
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

  radio.setDio1Action(setFlag);
  
  radio.startReceive();
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

void loop() {
  if (receivedFlag) {
  receivedFlag = false;

  Payload data;

  int state = radio.receive((uint8_t*)&data, sizeof(data));

  if (state == RADIOLIB_ERR_NONE) {
    Serial.print("[RX] ");
    Serial.print("RSSI: ");
    Serial.print(radio.getRSSI());
    Serial.print(" | SNR: ");
    Serial.print(radio.getSNR());
    Serial.println("");
    Serial.print(millis());
    Serial.print(" ms: ");
    Serial.println("");
    doc["T"] = data.temp;
    doc["H"] = data.humedad;
    doc["A"] = (int) (data.altitud * 100) / 100.0;
    doc["GPS"]["S"] = data.gpsStatus;
    doc["P"] = data.pressure;
    if(data.gpsStatus) {
      doc["LAT"] = data.lat;
      doc["LON"] = data.lon;
    }
    serializeJson(doc, Serial);
    Serial.println();
  } else {
    Serial.print("Error de lectura: ");
    Serial.println(state);
  }

  radio.startReceive();  
}
}
