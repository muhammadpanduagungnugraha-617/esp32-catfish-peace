#include <Arduino.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <WiFi.h>
#include <time.h>
#include <Firebase_ESP_Client.h>
#include "addons/TokenHelper.h"
#include "addons/RTDBHelper.h"
#include "secrets.h"

// Kontrak data dengan app Flutter: lihat docs/data-contract.md di repo app.
//   kolam_status   (ESP32 tulis)  : sensor + is_connected + last_updated
//   kolam_control  (APP tulis)    : mode_otomatis + perintah aktuator
//   kolam_relay    (ESP32 tulis)  : status ASLI relay (hasil eksekusi)
//   sensor_history (ESP32 tulis)  : riwayat untuk grafik, +/-tiap 5 menit

// ======================== PINOUT DEFINITION (ESP32 38-PIN) ========================
#define ONE_WIRE_BUS 4
#define PH_PIN 35
#define TURBIDITY_PIN 34
#define MQ135_PIN 32
#define TRIG_PIN 25
#define ECHO_PIN 26

#define RELAY_PEMBUANGAN 19
#define RELAY_PENGISIAN 18
#define RELAY_AERATOR 5
#define RELAY_PROBIOTIK 17
#define BUZZER_PIN 23

// Relay modul active-low: LOW = nyala, HIGH = mati
#define RELAY_ON LOW
#define RELAY_OFF HIGH

// Heartbeat RTDB: tulis setiap 2 detik. Flutter Terputus jika last_updated > 6 detik.
static const unsigned long INTERVAL_UPDATE_MS = 2000;
// Status relay: tulis saat berubah (min. 1 detik antar tulis) dan sebagai heartbeat.
static const unsigned long INTERVAL_RELAY_MIN_MS = 1000;
static const unsigned long INTERVAL_RELAY_HEARTBEAT_MS = 10000;
// Riwayat grafik: JANGAN tiap 2 detik (43.200 entri per hari).
static const unsigned long INTERVAL_HISTORY_MS = 5UL * 60UL * 1000UL;
// Coba sambung ulang Wi-Fi / Firebase bila gagal.
static const unsigned long INTERVAL_COBA_SAMBUNG_MS = 15000;
// Fail-safe: mode manual dibatalkan bila perintah tidak terbaca selama ini.
static const unsigned long CONTROL_FAILSAFE_MS = 30000;

// ======================== SETUP SENSOR & VARIABEL ========================
OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);

const float CALIBRATION_FACTOR = 1.0;
float voltageAtPH7 = 1.65;

const unsigned long TIMEOUT_POMPA_MS = 120000; // Maksimal pompa 2 menit
unsigned long tSiklusMulai = 0;

enum WaterSystemState { IDLE, DRAINING, FILLING, PROBIOTIC_INJECT };
WaterSystemState currentWaterState = IDLE;
unsigned long probioticStartTime = 0;

FirebaseData fbdo;
FirebaseData fbdoStream;
FirebaseAuth auth;
FirebaseConfig config;

bool firebaseSiap = false;
bool controlNodeSiap = false;
bool wifiTerkonfigurasi = false;
bool ntpDikonfigurasi = false;

// Perintah dari app (kolam_control). Default aman: otomatis, semua mati.
bool modeOtomatis = true;
bool cmdPompaBuang = false;
bool cmdPompaIsi = false;
bool cmdPompaProbiotik = false;
bool cmdAerator = false;

struct RelaySnapshot {
  bool buang;
  bool isi;
  bool probiotik;
  bool aerator;
};
RelaySnapshot relayTerakhirDikirim = {false, false, false, false};
bool relayPernahDikirim = false;

unsigned long tUpdateTerakhir = 0;
unsigned long tRelayTulis = 0;
unsigned long tHistoryTerakhir = 0;
bool historyPernahDitulis = false;
unsigned long tCobaSambung = 0;
unsigned long tKontrolOk = 0; // terakhir kolam_control berhasil dibaca
uint8_t percobaanCekControl = 0;

static void setRelay(int pin, bool nyala) {
  digitalWrite(pin, nyala ? RELAY_ON : RELAY_OFF);
}

static bool relayNyala(int pin) {
  return digitalRead(pin) == RELAY_ON;
}

static RelaySnapshot bacaRelay() {
  RelaySnapshot r;
  r.buang = relayNyala(RELAY_PEMBUANGAN);
  r.isi = relayNyala(RELAY_PENGISIAN);
  r.probiotik = relayNyala(RELAY_PROBIOTIK);
  r.aerator = relayNyala(RELAY_AERATOR);
  return r;
}

static void matikanPompaAir() {
  setRelay(RELAY_PEMBUANGAN, false);
  setRelay(RELAY_PENGISIAN, false);
  setRelay(RELAY_PROBIOTIK, false);
}

static const char *labelStatus(float suhu, float ph, float ntu, float amonia) {
  bool bahaya = (suhu < 24.0f || suhu > 31.0f) ||
                (ph < 6.0f || ph > 9.0f) ||
                (ntu > 60.0f) ||
                (amonia >= 0.60f);
  if (bahaya) return "Bahaya";

  bool waspada = (suhu < 25.0f || suhu > 30.0f) ||
                 (ph < 6.5f || ph > 8.5f) ||
                 (ntu > 50.0f) ||
                 (amonia > 0.50f);
  if (waspada) return "Waspada";
  return "Aman";
}

// Epoch DETIK (UTC) dari NTP. time() selalu UTC; offset zona hanya memengaruhi localtime().
static time_t epochDetik() {
  time_t now = time(nullptr);
  if (now < 1700000000) return 0; // NTP belum siap
  return now;
}

// DS18B20 yang lepas/rusak mengembalikan -127. Nilai seperti itu TIDAK boleh
// dikirim: app akan menampilkannya sebagai pembacaan asli.
static bool sensorValid(float suhu, float ph, float ntu, float amonia) {
  if (isnan(suhu) || isinf(suhu) || suhu <= -100.0f) return false;
  if (isnan(ph) || isinf(ph)) return false;
  if (isnan(ntu) || isinf(ntu)) return false;
  if (isnan(amonia) || isinf(amonia)) return false;
  return true;
}

static void sambungWifi() {
  if (String(WIFI_SSID) == "NAMA_WIFI") {
    Serial.println("[WIFI] Isi WIFI_SSID dan WIFI_PASSWORD di secrets.h dulu.");
    return;
  }

  wifiTerkonfigurasi = true;
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  Serial.printf("[WIFI] Menyambung ke %s", WIFI_SSID);
  unsigned long t0 = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - t0 < 20000) {
    delay(400);
    Serial.print(".");
  }
  Serial.println();
  if (WiFi.status() == WL_CONNECTED) {
    Serial.printf("[WIFI] OK  IP=%s\n", WiFi.localIP().toString().c_str());
  } else {
    Serial.println("[WIFI] Gagal. Logika lokal tetap jalan, akan dicoba lagi.");
  }
}

static void aturNtp() {
  if (ntpDikonfigurasi || WiFi.status() != WL_CONNECTED) return;
  // UTC (offset 0): kontrak memakai epoch UTC, bukan jam lokal.
  configTime(0, 0, "pool.ntp.org", "time.nist.gov");
  ntpDikonfigurasi = true;
}

static void initFirebase() {
  if (WiFi.status() != WL_CONNECTED) return;

  config.api_key = FIREBASE_API_KEY;
  config.database_url = FIREBASE_DATABASE_URL;
  config.token_status_callback = tokenStatusCallback;

  if (String(FIREBASE_USER_EMAIL).length() > 0) {
    auth.user.email = FIREBASE_USER_EMAIL;
    auth.user.password = FIREBASE_USER_PASSWORD;
  } else {
    if (!Firebase.signUp(&auth, &config, "", "")) {
      Serial.printf("[FB] Signup anonim gagal: %s\n", config.signer.signupError.message.c_str());
      Serial.println("[FB] Aktifkan Anonymous Auth, atau isi FIREBASE_USER_EMAIL di secrets.h.");
      return;
    }
    Serial.println("[FB] Login anonim OK.");
  }

  Firebase.begin(&config, &auth);
  Firebase.reconnectWiFi(true);
  Firebase.setDoubleDigits(2);
  firebaseSiap = true;
}

// Dipanggil tiap loop: pulihkan Wi-Fi / NTP / Firebase bila sempat gagal saat boot
// atau putus di tengah jalan. Tanpa ini alat tetap "Terputus" di app sampai reboot.
static void jagaKoneksi() {
  const unsigned long now = millis();

  if (WiFi.status() == WL_CONNECTED) {
    aturNtp();
    if (!firebaseSiap && now - tCobaSambung >= INTERVAL_COBA_SAMBUNG_MS) {
      tCobaSambung = now;
      initFirebase();
    }
    return;
  }

  // Wi-Fi putus: coba sambung ulang berkala.
  if (wifiTerkonfigurasi && now - tCobaSambung >= INTERVAL_COBA_SAMBUNG_MS) {
    tCobaSambung = now;
    Serial.println("[WIFI] Terputus, mencoba menyambung ulang...");
    WiFi.reconnect();
  }
}

// Node kosong sudah diperlakukan app dan firmware sebagai "otomatis, semua mati",
// jadi mengisi default hanya merapikan Console. Hanya diisi bila node BENAR-BENAR
// kosong, supaya kegagalan jaringan tidak menimpa perintah user.
static void pastikanControlDefault() {
  if (!Firebase.ready() || controlNodeSiap) return;

  if (!Firebase.RTDB.get(&fbdo, "/kolam_control")) {
    Serial.printf("[FB] Cek kolam_control gagal: %s\n", fbdo.errorReason().c_str());
    if (++percobaanCekControl >= 3) controlNodeSiap = true; // berhenti mencoba
    return;
  }

  if (fbdo.dataType() == "null") {
    FirebaseJson json;
    json.set("mode_otomatis", true);
    json.set("pompa_buang_air", false);
    json.set("pompa_isi_air", false);
    json.set("pompa_probiotik", false);
    json.set("aerator", false);
    if (Firebase.RTDB.setJSON(&fbdo, "/kolam_control", &json)) {
      Serial.println("[FB] kolam_control default (otomatis, semua pompa mati).");
    }
  }
  controlNodeSiap = true;
}

// CATATAN: library Firebase Arduino Client (4.4.x) TIDAK mendukung onDisconnect,
// jadi is_connected tidak bisa otomatis menjadi false saat alat putus mendadak.
// Itu tidak masalah: app menilai "Terputus" dari last_updated yang basi (> 6 detik),
// bukan dari is_connected. is_connected hanya dikirim true bersama tiap pembaruan.

// Baca seluruh kolam_control dalam SATU request (dulu 4 request terpisah).
// Field hilang = mati; mode_otomatis hilang = otomatis (sesuai kontrak).
// Mengembalikan true bila node berhasil dibaca.
static bool bacaPerintahKontrol() {
  if (!Firebase.ready()) return false;

  if (!Firebase.RTDB.getJSON(&fbdo, "/kolam_control")) {
    Serial.printf("[FB] Gagal baca kolam_control: %s\n", fbdo.errorReason().c_str());
    return false;
  }

  FirebaseJson &json = fbdo.jsonObject();
  FirebaseJsonData r;

  bool otomatis = true;
  if (json.get(r, "mode_otomatis") && r.success) otomatis = r.boolValue;
  modeOtomatis = otomatis;

  cmdPompaBuang = (json.get(r, "pompa_buang_air") && r.success) ? r.boolValue : false;
  cmdPompaIsi = (json.get(r, "pompa_isi_air") && r.success) ? r.boolValue : false;
  cmdPompaProbiotik = (json.get(r, "pompa_probiotik") && r.success) ? r.boolValue : false;
  cmdAerator = (json.get(r, "aerator") && r.success) ? r.boolValue : false;
  return true;
}

// Status ASLI relay ke /kolam_relay (bukan ke kolam_control: node itu milik app).
// App menampilkan "Sedang nyala/mati" dari sini. Ditulis saat ada perubahan dan
// sebagai heartbeat tiap INTERVAL_RELAY_HEARTBEAT_MS.
static void kirimStatusRelay() {
  if (!Firebase.ready()) return;

  const RelaySnapshot r = bacaRelay();
  const bool berubah = !relayPernahDikirim ||
                       r.buang != relayTerakhirDikirim.buang ||
                       r.isi != relayTerakhirDikirim.isi ||
                       r.probiotik != relayTerakhirDikirim.probiotik ||
                       r.aerator != relayTerakhirDikirim.aerator;

  const unsigned long now = millis();
  const unsigned long sejakTulis = now - tRelayTulis;
  if (!berubah && sejakTulis < INTERVAL_RELAY_HEARTBEAT_MS) return;
  if (berubah && relayPernahDikirim && sejakTulis < INTERVAL_RELAY_MIN_MS) return;

  tRelayTulis = now; // juga saat gagal, supaya kegagalan tidak memblokir tiap loop

  FirebaseJson json;
  json.set("pompa_buang_air", r.buang);
  json.set("pompa_isi_air", r.isi);
  json.set("pompa_probiotik", r.probiotik);
  json.set("aerator", r.aerator);

  if (Firebase.RTDB.setJSON(&fbdo, "/kolam_relay", &json)) {
    relayTerakhirDikirim = r;
    relayPernahDikirim = true;
  } else {
    Serial.printf("[FB] Gagal tulis kolam_relay: %s\n", fbdo.errorReason().c_str());
  }
}

static void kirimKolamStatus(float suhu, float ph, float ntu, float amonia, float jarakCm) {
  if (!Firebase.ready()) return;

  // Tanpa waktu yang benar, last_updated tidak berarti. Lebih baik tidak
  // menulis (app: "belum ada data"/Terputus) daripada menulis 0.
  const time_t epoch = epochDetik();
  if (epoch == 0) {
    Serial.println("[NTP] Waktu belum sinkron, kolam_status tidak dikirim.");
    return;
  }

  // Sensor rusak/lepas: JANGAN perbarui last_updated. PATCH hanya mengganti
  // field yang dikirim, jadi menulis timestamp baru di atas nilai lama akan
  // tampak seperti data segar. Dibiarkan basi, app menampilkan "Terputus".
  if (!sensorValid(suhu, ph, ntu, amonia)) {
    Serial.println("[SENSOR] Pembacaan tidak valid (sensor lepas/rusak?), tidak dikirim.");
    return;
  }

  FirebaseJson json;
  json.set("is_connected", true);
  json.set("status_kondisi", labelStatus(suhu, ph, ntu, amonia));
  json.set("suhu", suhu);
  json.set("ph", ph);
  json.set("kekeruhan", ntu);
  json.set("amonia", amonia);
  json.set("tinggi_air", jarakCm);
  json.set("last_updated", (int)epoch);

  if (!Firebase.RTDB.updateNode(&fbdo, "/kolam_status", &json)) {
    Serial.printf("[FB] Gagal tulis kolam_status: %s\n", fbdo.errorReason().c_str());
    return;
  }

  // Riwayat hanya tiap INTERVAL_HISTORY_MS. Key = epoch detik (terurut waktu).
  const unsigned long now = millis();
  if (!historyPernahDitulis || now - tHistoryTerakhir >= INTERVAL_HISTORY_MS) {
    String histPath = "/sensor_history/" + String((int)epoch);
    FirebaseJson hist;
    hist.set("timestamp", (int)epoch);
    hist.set("suhu", suhu);
    hist.set("ph", ph);
    hist.set("kekeruhan", ntu);
    hist.set("amonia", amonia);
    if (Firebase.RTDB.setJSON(&fbdo, histPath.c_str(), &hist)) {
      historyPernahDitulis = true;
      tHistoryTerakhir = now;
    }
  }
}

float bacaJarakAir() {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);
  long duration = pulseIn(ECHO_PIN, HIGH, 30000);
  if (duration == 0) return 999;
  return duration * 0.034 / 2;
}

void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("=== SISTEM FISIK INISIALISASI ===");

  sensors.begin();
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  pinMode(PH_PIN, INPUT);
  pinMode(TURBIDITY_PIN, INPUT);
  pinMode(MQ135_PIN, INPUT);

  pinMode(RELAY_PEMBUANGAN, OUTPUT);
  pinMode(RELAY_PENGISIAN, OUTPUT);
  pinMode(RELAY_AERATOR, OUTPUT);
  pinMode(RELAY_PROBIOTIK, OUTPUT);
  pinMode(BUZZER_PIN, OUTPUT);

  setRelay(RELAY_PEMBUANGAN, false);
  setRelay(RELAY_PENGISIAN, false);
  setRelay(RELAY_AERATOR, false);
  setRelay(RELAY_PROBIOTIK, false);
  digitalWrite(BUZZER_PIN, LOW);

  sambungWifi();
  aturNtp();
  initFirebase();
}

void loop() {
  jagaKoneksi();

  sensors.requestTemperatures();
  float dataSuhu = sensors.getTempCByIndex(0);

  float phAdcSum = 0;
  for (int i = 0; i < 20; i++) {
    phAdcSum += analogRead(PH_PIN);
    delay(5);
  }
  float phVoltage = (phAdcSum / 20.0 / 4095.0) * 3.3;
  float dataPH = 7.00 + ((voltageAtPH7 - phVoltage) / 0.18);
  if (dataPH < 0) dataPH = 0;
  if (dataPH > 14) dataPH = 14;

  float turbAdcSum = 0;
  for (int i = 0; i < 20; i++) {
    turbAdcSum += analogRead(TURBIDITY_PIN);
    delay(5);
  }
  float turbVoltage = (turbAdcSum / 20.0 / 4095.0) * 3.3 * CALIBRATION_FACTOR;
  if (turbVoltage > 4.2) turbVoltage = 4.2;
  if (turbVoltage < 2.5) turbVoltage = 2.5;
  float dataTurbidity = -1120.4 * (turbVoltage * turbVoltage) + 5742.3 * turbVoltage - 4352.9;
  if (dataTurbidity < 0) dataTurbidity = 0;

  float mqAdcSum = 0;
  for (int i = 0; i < 20; i++) {
    mqAdcSum += analogRead(MQ135_PIN);
    delay(5);
  }
  float mqVoltage = (mqAdcSum / 20.0 / 4095.0) * 3.3;
  if (mqVoltage < 0.01) mqVoltage = 0.01;
  float rZero = 20.0;
  float rs = ((3.3 * 10.0) / mqVoltage) - 10.0;
  float dataAmonia = 5.2 * pow((rs / rZero), -1.48);

  float jarakKeAir = bacaJarakAir();

  const bool saatnyaSinkron = (millis() - tUpdateTerakhir >= INTERVAL_UPDATE_MS);
  if (saatnyaSinkron && firebaseSiap && Firebase.ready()) {
    pastikanControlDefault();
    if (bacaPerintahKontrol()) tKontrolOk = millis();
  }

  // Fail-safe: mode manual hanya sah selama perintah masih bisa dibaca. Bila
  // Wi-Fi/Firebase putus, pompa yang terakhir dinyalakan manual tidak boleh
  // menyala selamanya; kembali ke otomatis (yang tetap jalan tanpa internet).
  if (!modeOtomatis && (millis() - tKontrolOk > CONTROL_FAILSAFE_MS)) {
    modeOtomatis = true;
    Serial.println("[FAILSAFE] Perintah tidak terbaca > 30 dtk, kembali ke mode otomatis.");
  }

  const bool amoniaBahaya = (dataAmonia >= 0.60);
  digitalWrite(BUZZER_PIN, amoniaBahaya ? HIGH : LOW);

  if (modeOtomatis) {
    if (dataSuhu > 31.0) {
      setRelay(RELAY_AERATOR, true);
    } else if (dataSuhu <= 25.0 && currentWaterState == IDLE && !amoniaBahaya) {
      setRelay(RELAY_AERATOR, false);
    }

    if (amoniaBahaya) {
      setRelay(RELAY_AERATOR, true);
      if (currentWaterState == IDLE) {
        currentWaterState = DRAINING;
        tSiklusMulai = millis();
      }
    }

    if ((dataPH < 6.0 || dataPH > 9.0 || dataTurbidity > 60.0) && currentWaterState == IDLE) {
      currentWaterState = DRAINING;
      tSiklusMulai = millis();
    }

    if (currentWaterState != IDLE && (millis() - tSiklusMulai > TIMEOUT_POMPA_MS)) {
      Serial.println("[CRITICAL ERROR] POMPA OVERTIME/MACET! Mematikan pompa demi keamanan.");
      matikanPompaAir();
      currentWaterState = IDLE;
      delay(5000);
      return;
    }

    switch (currentWaterState) {
      case DRAINING:
        setRelay(RELAY_PEMBUANGAN, true);
        setRelay(RELAY_PENGISIAN, false);
        setRelay(RELAY_AERATOR, true);
        if (jarakKeAir >= 80.0 || jarakKeAir == 999) {
          setRelay(RELAY_PEMBUANGAN, false);
          currentWaterState = FILLING;
          tSiklusMulai = millis();
        }
        break;

      case FILLING:
        setRelay(RELAY_PEMBUANGAN, false);
        setRelay(RELAY_PENGISIAN, true);
        if (jarakKeAir <= 5.0) {
          setRelay(RELAY_PENGISIAN, false);
          probioticStartTime = millis();
          currentWaterState = PROBIOTIC_INJECT;
        }
        break;

      case PROBIOTIC_INJECT:
        setRelay(RELAY_PROBIOTIK, true);
        if (millis() - probioticStartTime >= 10000) {
          setRelay(RELAY_PROBIOTIK, false);
          currentWaterState = IDLE;
        }
        break;

      case IDLE:
        setRelay(RELAY_PEMBUANGAN, false);
        setRelay(RELAY_PENGISIAN, false);
        break;
    }
  } else {
    // Mode manual: ikuti perintah dari app (kolam_control).
    currentWaterState = IDLE;
    setRelay(RELAY_PEMBUANGAN, cmdPompaBuang);
    setRelay(RELAY_PENGISIAN, cmdPompaIsi);
    setRelay(RELAY_AERATOR, cmdAerator);
    setRelay(RELAY_PROBIOTIK, cmdPompaProbiotik);
  }

  // Laporkan status relay yang sebenarnya segera setelah berubah.
  if (firebaseSiap) kirimStatusRelay();

  if (millis() - tUpdateTerakhir >= INTERVAL_UPDATE_MS) {
    tUpdateTerakhir = millis();
    if (firebaseSiap) {
      kirimKolamStatus(dataSuhu, dataPH, dataTurbidity, dataAmonia, jarakKeAir);
    }
    Serial.printf(
      "S=%.1f pH=%.1f NTU=%.1f NH3=%.2f d=%.0fcm auto=%d epoch=%ld\n",
      dataSuhu, dataPH, dataTurbidity, dataAmonia, jarakKeAir,
      modeOtomatis ? 1 : 0, (long)epochDetik());
  }
}
