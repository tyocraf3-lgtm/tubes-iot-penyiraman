/*
  ALAT PENYIRAM TANAMAN OTOMATIS
  Board  : ESP32 Dev Module
  Sensor : Sensor tanah NPK 7-in-1 (RS485 Modbus RTU)
  Pompa  : Pompa celup mini 3-5V (dikendalikan relay)
  Cloud  : ThingSpeak
*/

#include <WiFi.h>
#include <ThingSpeak.h>
#include <time.h>

// ================= WIFI =================
const char* WIFI_SSID     = "WIJAYA E-SPORT";
const char* WIFI_PASSWORD = "Bismillah";

// ================= THINGSPEAK =================
unsigned long CHANNEL_ID    = 3573227;            // Channel ID Anda
const char*   WRITE_API_KEY = "QXX0ML7RKXMYP97D"; // Write API Key Anda

// ================= PIN =================
#define RS485_RX_PIN 16   // RO modul RS485 -> GPIO16
#define RS485_TX_PIN 17   // DI modul RS485 -> GPIO17
#define RS485_DE_RE  4    // DE & RE modul RS485 -> GPIO4
#define PUMP_PIN     27   // IN relay -> D27 (GPIO27)
#define RELAY_ON     LOW  // relay active-low: LOW = relay menyala
#define RELAY_OFF    HIGH // HIGH = relay mati

// ================= PENGATURAN PENYIRAMAN =================
const float         MOISTURE_LOW   = 35.0;      // % : di bawah ini -> mulai siram
const float         MOISTURE_HIGH  = 60.0;      // % : di atas ini -> pompa berhenti
const unsigned long PUMP_MAX_MS    = 600000UL;  // batas aman terus menyala (10 menit). 0 = tanpa batas
const unsigned long SOAK_MS        = 60000UL;   // jeda setelah batas aman tercapai
const unsigned long SENSOR_LOST_MS = 30000UL;   // pompa berhenti jika sensor tidak terbaca selama ini

// ================= JAM SIRAM (WIB) =================
const long GMT_OFFSET_SEC = 7 * 3600;  // WIB = UTC+7
const int  JAM_MULAI      = 18;        // pompa boleh jalan mulai jam 18:00
const int  JAM_SELESAI    = 6;         // sampai jam 06:00 (melewati tengah malam)

// ================= PENGATURAN SENSOR & WAKTU =================
const uint8_t       SENSOR_ADDR       = 0x01;     // alamat Modbus sensor
const long          SENSOR_BAUD       = 4800;     // baud sensor (sudah terbukti CRC OK)
const unsigned long SENSOR_READ_MS    = 2000;     // baca sensor tiap 2 detik
const unsigned long SEND_INTERVAL_MS  = 30000;    // kirim ke ThingSpeak tiap 30 detik

// ================= VARIABEL GLOBAL =================
WiFiClient client;

struct SoilData {
  float    moisture;     // kelembaban (%)
  float    temperature;  // suhu (derajat C)
  float    ec;           // EC (uS/cm)
  float    ph;           // pH
  uint16_t nitrogen;     // N (mg/kg)
  uint16_t phosphorus;   // P (mg/kg)
  uint16_t potassium;    // K (mg/kg)
};
SoilData soil;
bool sensorOK = false;

bool          pumpOn        = false;
bool          jamAktif      = false;  // hasil cek jam, diperbarui tiap pembacaan sensor
unsigned long pumpStartTime = 0;
unsigned long pumpReadyAt   = 0;      // pompa baru boleh menyala lagi setelah waktu ini
unsigned long lastSensorOK  = 0;      // waktu terakhir sensor terbaca valid

unsigned long lastSensorRead = 0;
unsigned long lastSend       = 0;
unsigned long lastWifiTry    = 0;

// ================= CRC & MODBUS =================
uint16_t modbusCRC(const uint8_t* data, uint8_t len) {
  uint16_t crc = 0xFFFF;
  for (uint8_t i = 0; i < len; i++) {
    crc ^= data[i];
    for (uint8_t b = 0; b < 8; b++) {
      if (crc & 0x0001) crc = (crc >> 1) ^ 0xA001;
      else              crc >>= 1;
    }
  }
  return crc;
}

// Membaca 'qty' holding register (function 0x03) mulai dari 'startAddr'
bool readRegisters(uint16_t startAddr, uint16_t qty, uint16_t* out) {
  uint8_t req[8];
  req[0] = SENSOR_ADDR;
  req[1] = 0x03;
  req[2] = startAddr >> 8;
  req[3] = startAddr & 0xFF;
  req[4] = qty >> 8;
  req[5] = qty & 0xFF;
  uint16_t crc = modbusCRC(req, 6);
  req[6] = crc & 0xFF;   // CRC low byte dulu
  req[7] = crc >> 8;     // lalu CRC high byte

  while (Serial2.available()) Serial2.read();   // bersihkan buffer

  digitalWrite(RS485_DE_RE, HIGH);              // mode kirim
  Serial2.write(req, sizeof(req));
  Serial2.flush();
  delay(10);
  digitalWrite(RS485_DE_RE, LOW);               // mode terima

  uint8_t expected = 5 + 2 * qty;               // addr, fc, bytecount, data, crc(2)
  uint8_t resp[32];
  uint8_t idx = 0;
  unsigned long t0 = millis();
  while (idx < expected && millis() - t0 < 1000) {
    if (Serial2.available()) resp[idx++] = Serial2.read();
  }
  if (idx < expected)         return false;      // timeout
  if (resp[0] != SENSOR_ADDR) return false;
  if (resp[1] != 0x03)        return false;
  if (resp[2] != 2 * qty)     return false;

  uint16_t rcrc = resp[expected - 2] | (resp[expected - 1] << 8);
  if (modbusCRC(resp, expected - 2) != rcrc) return false;   // CRC salah

  for (uint16_t i = 0; i < qty; i++) {
    out[i] = ((uint16_t)resp[3 + 2 * i] << 8) | resp[4 + 2 * i];
  }
  return true;
}

// Membaca 7 register sensor NPK
bool readSoilSensor() {
  uint16_t reg[7];
  if (!readRegisters(0x0000, 7, reg)) return false;

  soil.moisture    = reg[0] / 10.0;
  soil.temperature = (int16_t)reg[1] / 10.0;   // suhu bisa negatif
  soil.ec          = reg[2];
  soil.ph          = reg[3] / 10.0;
  soil.nitrogen    = reg[4];
  soil.phosphorus  = reg[5];
  soil.potassium   = reg[6];
  return true;
}

// ================= JAM =================
bool jamSiramAktif() {
  struct tm t;
  if (!getLocalTime(&t, 100)) return false;      // jam belum sinkron -> pompa tidak jalan
  int jam = t.tm_hour;
  if (JAM_MULAI < JAM_SELESAI) return jam >= JAM_MULAI && jam < JAM_SELESAI;
  return jam >= JAM_MULAI || jam < JAM_SELESAI;  // rentang melewati tengah malam
}

void cetakJam() {
  struct tm t;
  if (getLocalTime(&t, 100)) {
    char buf[6];
    snprintf(buf, sizeof(buf), "%02d:%02d", t.tm_hour, t.tm_min);
    Serial.print(buf);
  } else {
    Serial.print("belum sinkron");
  }
}

void printSoil() {
  Serial.print("Lembab: ");  Serial.print(soil.moisture, 1);
  Serial.print("%  Suhu: "); Serial.print(soil.temperature, 1);
  Serial.print("C  EC: ");   Serial.print(soil.ec, 0);
  Serial.print("  pH: ");    Serial.print(soil.ph, 1);
  Serial.print("  N: ");     Serial.print(soil.nitrogen);
  Serial.print("  P: ");     Serial.print(soil.phosphorus);
  Serial.print("  K: ");     Serial.print(soil.potassium);

  Serial.print("  | pompa: ");
  Serial.print(pumpOn ? "ON" : "OFF");

  Serial.print("  | jam: ");
  cetakJam();
  Serial.print(jamAktif ? " (jam siram)" : " (luar jam siram)");

  if (!pumpOn && millis() < pumpReadyAt) {
    Serial.print("  | jeda meresap: ");
    Serial.print((pumpReadyAt - millis()) / 1000);
    Serial.print(" dtk");
  }
  Serial.println();
}

// ================= POMPA (RELAY) =================
void pumpStart() {
  digitalWrite(PUMP_PIN, RELAY_ON);
  pumpOn = true;
  pumpStartTime = millis();
  Serial.println(">> POMPA ON");
}

void pumpStop(const char* alasan) {
  digitalWrite(PUMP_PIN, RELAY_OFF);
  if (pumpOn) {
    Serial.print(">> POMPA OFF (");
    Serial.print(alasan);
    Serial.println(")");
  }
  pumpOn = false;
}

void controlPump() {
  if (pumpOn) {
    if (!jamAktif) {
      pumpStop("di luar jam siram");
    } else if (sensorOK && soil.moisture >= MOISTURE_HIGH) {
      pumpStop("kelembaban sudah cukup");
    } else if (millis() - lastSensorOK >= SENSOR_LOST_MS) {
      pumpStop("sensor tidak terbaca");
    } else if (PUMP_MAX_MS > 0 && millis() - pumpStartTime >= PUMP_MAX_MS) {
      pumpStop("batas aman tercapai");
      pumpReadyAt = millis() + SOAK_MS;   // jeda meresap setelah batas aman
    }
    return;
  }

  // Pompa mati: menyala jika jam siram, sensor valid, tanah kering, dan jeda sudah lewat
  if (jamAktif && sensorOK && soil.moisture < MOISTURE_LOW && millis() >= pumpReadyAt) {
    pumpStart();
  }
}

// ================= WIFI =================
void connectWiFi() {
  Serial.print("Menghubungkan WiFi");
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  unsigned long t0 = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - t0 < 20000) {
    delay(500);
    Serial.print(".");
  }
  if (WiFi.status() == WL_CONNECTED) {
    Serial.print("\nTerhubung, IP: ");
    Serial.println(WiFi.localIP());
  } else {
    Serial.println("\nGagal, akan dicoba lagi otomatis.");
  }
  lastWifiTry = millis();
}

void checkWiFi() {
  if (WiFi.status() == WL_CONNECTED) return;
  if (millis() - lastWifiTry < 30000UL) return;   // coba ulang tiap 30 detik
  lastWifiTry = millis();
  Serial.println("WiFi putus, mencoba menyambung ulang...");
  WiFi.disconnect();
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
}

// ================= THINGSPEAK =================
void sendToThingSpeak() {
  if (!sensorOK) return;                          // jangan kirim data error
  if (WiFi.status() != WL_CONNECTED) return;

  ThingSpeak.setField(1, soil.moisture);
  ThingSpeak.setField(2, soil.temperature);
  ThingSpeak.setField(3, soil.ec);
  ThingSpeak.setField(4, soil.ph);
  ThingSpeak.setField(5, soil.nitrogen);
  ThingSpeak.setField(6, soil.phosphorus);
  ThingSpeak.setField(7, soil.potassium);
  ThingSpeak.setField(8, pumpOn ? 1 : 0);

  int code = ThingSpeak.writeFields(CHANNEL_ID, WRITE_API_KEY);
  if (code == 200) {
    Serial.println("ThingSpeak: OK");
  } else {
    Serial.print("ThingSpeak: GAGAL, kode ");
    Serial.println(code);
  }
}

// ================= SETUP & LOOP =================
void setup() {
  Serial.begin(115200);

  digitalWrite(PUMP_PIN, RELAY_OFF);  // set level dulu, relay tidak sempat aktif
  pinMode(PUMP_PIN, OUTPUT);

  pinMode(RS485_DE_RE, OUTPUT);
  digitalWrite(RS485_DE_RE, LOW);     // mode terima

  Serial2.begin(SENSOR_BAUD, SERIAL_8N1, RS485_RX_PIN, RS485_TX_PIN);

  connectWiFi();

  configTime(GMT_OFFSET_SEC, 0, "pool.ntp.org", "time.google.com");
  struct tm t;
  if (getLocalTime(&t, 10000)) Serial.println("Jam tersinkron via NTP");
  else Serial.println("Jam belum tersinkron, pompa tidak akan jalan sampai sinkron");

  ThingSpeak.begin(client);
}

void loop() {
  checkWiFi();

  // 1. Baca sensor NPK dan cek jam
  if (millis() - lastSensorRead >= SENSOR_READ_MS) {
    lastSensorRead = millis();
    sensorOK = readSoilSensor();
    if (sensorOK) lastSensorOK = millis();
    jamAktif = jamSiramAktif();
    if (sensorOK) printSoil();
    else          Serial.println("Sensor tidak merespon / data tidak valid");
  }

  // 2. Kontrol pompa
  controlPump();

  // 3. Kirim ke ThingSpeak
  if (millis() - lastSend >= SEND_INTERVAL_MS) {
    lastSend = millis();
    sendToThingSpeak();
  }
}
