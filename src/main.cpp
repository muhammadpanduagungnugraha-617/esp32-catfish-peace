#include <Arduino.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <WiFi.h>
#include <time.h>
#include <Firebase_ESP_Client.h>
#include "addons/TokenHelper.h"
#include "addons/RTDBHelper.h"
#include "secrets.h"

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

static const unsigned long INTERVAL_UPDATE_MS = 2000; 
static const unsigned long INTERVAL_KONTROL_MS = 1000;   // seberapa sering perintah app dibaca
static const unsigned long INTERVAL_RELAY_MIN_MS = 1000;
static const unsigned long INTERVAL_RELAY_HEARTBEAT_MS = 10000;
// ======================== MODE UJI SKENARIO ========================
// 1 = nilai sensor bisa dipaksa lewat Serial Monitor (kirim '1','2','3','0','a').
// WAJIB 0 untuk pemasangan sungguhan. Saat uji, LEPAS beban relay: mode
// otomatis tetap bereaksi pada nilai palsu (bahaya = siklus kuras/isi).
#define SIMULASI_SKENARIO 1

#if SIMULASI_SKENARIO
static const unsigned long INTERVAL_HISTORY_MS = 10UL * 1000UL;  // cepat, agar log cepat terisi
#else
static const unsigned long INTERVAL_HISTORY_MS = 1UL * 60UL * 1000UL;
#endif
static const unsigned long INTERVAL_COBA_SAMBUNG_MS = 15000;
static const unsigned long CONTROL_FAILSAFE_MS = 30000;
static const unsigned long COOLDOWN_SIKLUS_MS = 60000;   // jeda setelah siklus pompa kena timeout

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
FirebaseAuth auth;
FirebaseConfig config;

bool firebaseSiap = false;
bool controlNodeSiap = false;
bool wifiTerkonfigurasi = false;
bool ntpDikonfigurasi = false;

// Perintah dari app. Default aman: otomatis, semua mati.
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
unsigned long tKontrolOk = 0;
unsigned long tKontrolPoll = 0;
unsigned long tCooldownSampai = 0;
bool modeSebelumnya = true;

// Hasil sampling sensor terakhir (diambil tiap INTERVAL_UPDATE_MS, bukan tiap loop)
float gSuhu = 0, gPH = 0, gNTU = 0, gAmonia = 0, gJarak = 0;
unsigned long tSampel = 0;
bool sampelAda = false;
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
  if (suhu == 0 || ph == 0) return "Waspada"; 
  
  bool bahaya = (suhu < 24.0f || suhu > 31.0f) || (ph < 6.0f || ph > 9.0f) || (ntu > 60.0f) || (amonia >= 0.60f);
  if (bahaya) return "Bahaya";

  bool waspada = (suhu < 25.0f || suhu > 30.0f) || (ph < 6.5f || ph > 8.5f) || (ntu > 50.0f) || (amonia > 0.50f);
  if (waspada) return "Waspada";
  
  return "Aman";
}

static time_t epochDetik() {
  time_t now = time(nullptr);
  if (now < 1700000000) return 0; 
  return now;
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
  }
}

static void aturNtp() {
  if (ntpDikonfigurasi || WiFi.status() != WL_CONNECTED) return;
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
    Serial.println("[FB] ERROR: Email Firebase kosong. Tidak bisa bypass auth != null.");
    return;
  }

  // Set timeout & buffer yang lebih aman untuk koneksi handshake Firebase ESP32
  Firebase.begin(&config, &auth);
  
  // ================= PERBAIKAN REBOOT =================
  // Paksa alokasi buffer SSL/TLS 16KB agar handshake Google tidak gagal (memori tidak jebol)
  fbdo.setBSSLBufferSize(16384, 16384);
  fbdo.setResponseSize(16384); 
  // ====================================================

  Firebase.reconnectWiFi(true);
  Firebase.setDoubleDigits(2);
  firebaseSiap = true;
}

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

  if (wifiTerkonfigurasi && now - tCobaSambung >= INTERVAL_COBA_SAMBUNG_MS) {
    tCobaSambung = now;
    Serial.println("[WIFI] Terputus, mencoba menyambung ulang...");
    WiFi.reconnect();
  }
}

static void pastikanControlDefault() {
  if (!Firebase.ready() || controlNodeSiap) return;
  if (!Firebase.RTDB.get(&fbdo, "/kolam_control")) {
    if (++percobaanCekControl >= 3) controlNodeSiap = true; 
    return;
  }
  if (fbdo.dataType() == "null") {
    FirebaseJson json;
    json.set("mode_otomatis", true);
    json.set("pompa_buang_air", false);
    json.set("pompa_isi_air", false);
    json.set("pompa_probiotik", false);
    json.set("aerator", false);
    Firebase.RTDB.setJSON(&fbdo, "/kolam_control", &json);
  }
  controlNodeSiap = true;
}

static bool bacaPerintahKontrol() {
  if (!Firebase.ready()) return false;
  if (!Firebase.RTDB.getJSON(&fbdo, "/kolam_control")) return false;

  FirebaseJson &json = fbdo.jsonObject();
  FirebaseJsonData r;

  modeOtomatis = (json.get(r, "mode_otomatis") && r.success) ? r.boolValue : true;
  cmdPompaBuang = (json.get(r, "pompa_buang_air") && r.success) ? r.boolValue : false;
  cmdPompaIsi = (json.get(r, "pompa_isi_air") && r.success) ? r.boolValue : false;
  cmdPompaProbiotik = (json.get(r, "pompa_probiotik") && r.success) ? r.boolValue : false;
  cmdAerator = (json.get(r, "aerator") && r.success) ? r.boolValue : false;
  
  return true;
}

static void kirimStatusRelay() {
  if (!Firebase.ready()) return;

  const RelaySnapshot r = bacaRelay();
  const bool berubah = !relayPernahDikirim || r.buang != relayTerakhirDikirim.buang ||
                       r.isi != relayTerakhirDikirim.isi || r.probiotik != relayTerakhirDikirim.probiotik ||
                       r.aerator != relayTerakhirDikirim.aerator;

  const unsigned long now = millis();
  const unsigned long sejakTulis = now - tRelayTulis;
  if (!berubah && sejakTulis < INTERVAL_RELAY_HEARTBEAT_MS) return;
  if (berubah && relayPernahDikirim && sejakTulis < INTERVAL_RELAY_MIN_MS) return;

  tRelayTulis = now; 

  FirebaseJson json;
  json.set("pompa_buang_air", r.buang);
  json.set("pompa_isi_air", r.isi);
  json.set("pompa_probiotik", r.probiotik);
  json.set("aerator", r.aerator);

  if (Firebase.RTDB.setJSON(&fbdo, "/kolam_relay", &json)) {
    relayTerakhirDikirim = r;
    relayPernahDikirim = true;
  }
}

static void kirimKolamStatus(float suhu, float ph, float ntu, float amonia, float jarakCm) {
  if (!Firebase.ready()) return;

  const time_t epoch = epochDetik();
  if (epoch == 0) return; 

  FirebaseJson json;
  json.set("is_connected", true);
  json.set("status_kondisi", labelStatus(suhu, ph, ntu, amonia));
  json.set("suhu", suhu);
  json.set("ph", ph);
  json.set("kekeruhan", ntu);
  json.set("amonia", amonia);
  json.set("tinggi_air", jarakCm);
  json.set("last_updated", (int)epoch);

  if (!Firebase.RTDB.setJSON(&fbdo, "/kolam_status", &json)) {
    Serial.printf("[FB] Gagal tulis kolam_status: %s\n", fbdo.errorReason().c_str());
    return;
  }

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

#if SIMULASI_SKENARIO
// 0 = sensor asli, 1 = aman, 2 = waspada, 3 = bahaya, 9 = siklus otomatis
static int skenario = 0;
static unsigned long tSkenario = 0;

static void bacaPerintahSerial() {
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '0' || c == '1' || c == '2' || c == '3') {
      skenario = c - '0';
      Serial.printf("[SIM] Skenario = %d (0 asli, 1 aman, 2 waspada, 3 bahaya)\n", skenario);
    } else if (c == 'a' || c == 'A') {
      skenario = 9;
      tSkenario = millis();
      Serial.println("[SIM] Siklus otomatis: aman -> waspada -> bahaya, tiap 90 dtk");
    }
  }
}

static void terapkanSkenario(float &suhu, float &ph, float &ntu, float &amonia) {
  int aktif = skenario;
  if (aktif == 9) aktif = 1 + (int)(((millis() - tSkenario) / 90000UL) % 3);
  switch (aktif) {
    case 1: suhu = 28.0f; ph = 7.2f; ntu = 6.5f;  amonia = 0.15f; break;  // aman
    case 2: suhu = 30.5f; ph = 8.7f; ntu = 55.0f; amonia = 0.55f; break;  // waspada
    case 3: suhu = 32.0f; ph = 9.5f; ntu = 70.0f; amonia = 0.70f; break;  // bahaya
    default: break;
  }
}
#endif

void setup() {
  Serial.begin(115200);
  delay(1000);

  sensors.begin();
  sensors.setWaitForConversion(false);
  sensors.requestTemperatures();
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

  float dataSuhu = gSuhu, dataPH = gPH, dataTurbidity = gNTU;
  float dataAmonia = gAmonia, jarakKeAir = gJarak;

  // Sensor dibaca tiap INTERVAL_UPDATE_MS saja, supaya loop tetap cepat
  // menanggapi perintah aktuator dari app.
  if (!sampelAda || millis() - tSampel >= INTERVAL_UPDATE_MS) {
  tSampel = millis();
  sampelAda = true;

  // DS18B20 non-blocking: ambil hasil konversi sebelumnya, minta konversi baru.
  dataSuhu = sensors.getTempCByIndex(0);
  sensors.requestTemperatures();
  if (isnan(dataSuhu) || isinf(dataSuhu) || dataSuhu <= -100.0f) dataSuhu = 0.0f;

  float phAdcSum = 0;
  for (int i = 0; i < 20; i++) {
    phAdcSum += analogRead(PH_PIN);
    delay(5);
  }
  float phVoltage = (phAdcSum / 20.0 / 4095.0) * 3.3;
  dataPH = 7.00 + ((voltageAtPH7 - phVoltage) / 0.18);
  if (isnan(dataPH) || isinf(dataPH) || dataPH < 0) dataPH = 0.0f;
  if (dataPH > 14.0f) dataPH = 14.0f;

  float turbAdcSum = 0;
  for (int i = 0; i < 20; i++) {
    turbAdcSum += analogRead(TURBIDITY_PIN);
    delay(5);
  }
  float turbVoltage = (turbAdcSum / 20.0 / 4095.0) * 3.3 * CALIBRATION_FACTOR;
  if (turbVoltage > 4.2) turbVoltage = 4.2;
  if (turbVoltage < 2.5) turbVoltage = 2.5;
  dataTurbidity = -1120.4 * (turbVoltage * turbVoltage) + 5742.3 * turbVoltage - 4352.9;
  if (isnan(dataTurbidity) || isinf(dataTurbidity) || dataTurbidity < 0) dataTurbidity = 0.0f;

  float mqAdcSum = 0;
  for (int i = 0; i < 20; i++) {
    mqAdcSum += analogRead(MQ135_PIN);
    delay(5);
  }
  float mqVoltage = (mqAdcSum / 20.0 / 4095.0) * 3.3;
  if (mqVoltage < 0.01) mqVoltage = 0.01;
  float rZero = 20.0;
  float rs = ((3.3 * 10.0) / mqVoltage) - 10.0;
  dataAmonia = 5.2 * pow((rs / rZero), -1.48);
  if (isnan(dataAmonia) || isinf(dataAmonia) || dataAmonia < 0) dataAmonia = 0.0f;

  jarakKeAir = bacaJarakAir();
  if (jarakKeAir == 999) jarakKeAir = 0.0f;

  gSuhu = dataSuhu; gPH = dataPH; gNTU = dataTurbidity;
  gAmonia = dataAmonia; gJarak = jarakKeAir;
  }

#if SIMULASI_SKENARIO
  bacaPerintahSerial();
  terapkanSkenario(dataSuhu, dataPH, dataTurbidity, dataAmonia);
#endif

  const bool saatnyaSinkron = (millis() - tKontrolPoll >= INTERVAL_KONTROL_MS);
  if (saatnyaSinkron && firebaseSiap && Firebase.ready()) {
    tKontrolPoll = millis();
    pastikanControlDefault();
    if (bacaPerintahKontrol()) tKontrolOk = millis();
  }

  if (!modeOtomatis && (millis() - tKontrolOk > CONTROL_FAILSAFE_MS)) {
    modeOtomatis = true;
    Serial.println("[FAILSAFE] Perintah tidak terbaca > 30 dtk, otomatis dihidupkan.");
  }

  // Peralihan mode: bersihkan sisa keadaan mode sebelumnya.
  if (modeOtomatis != modeSebelumnya) {
    currentWaterState = IDLE;
    if (modeOtomatis) {
      // manual -> otomatis: matikan semua, biar logika otomatis yang memutuskan
      matikanPompaAir();
      setRelay(RELAY_AERATOR, false);
    }
    Serial.printf("[MODE] %s\n", modeOtomatis ? "OTOMATIS" : "MANUAL");
    modeSebelumnya = modeOtomatis;
  }

  const bool amoniaBahaya = (dataAmonia >= 0.60);
  const bool bolehMulaiSiklus = ((long)(millis() - tCooldownSampai) >= 0);
  digitalWrite(BUZZER_PIN, amoniaBahaya ? HIGH : LOW);

  if (modeOtomatis) {
    if (dataSuhu > 31.0 && dataSuhu != 0.0f) {
      setRelay(RELAY_AERATOR, true);
    } else if (dataSuhu <= 25.0 && currentWaterState == IDLE && !amoniaBahaya) {
      setRelay(RELAY_AERATOR, false);
    }

    if (amoniaBahaya) {
      setRelay(RELAY_AERATOR, true);
      if (currentWaterState == IDLE && bolehMulaiSiklus) {
        currentWaterState = DRAINING;
        tSiklusMulai = millis();
      }
    }

    if ((dataPH < 6.0 || dataPH > 9.0 || dataTurbidity > 60.0) && currentWaterState == IDLE && bolehMulaiSiklus && dataPH != 0.0f) {
      currentWaterState = DRAINING;
      tSiklusMulai = millis();
    }

    if (currentWaterState != IDLE && (millis() - tSiklusMulai > TIMEOUT_POMPA_MS)) {
      matikanPompaAir();
      currentWaterState = IDLE;
      tCooldownSampai = millis() + COOLDOWN_SIKLUS_MS;
      Serial.println("[POMPA] Siklus melewati batas waktu, dihentikan. Jeda 60 dtk.");
    }

    switch (currentWaterState) {
      case DRAINING:
        setRelay(RELAY_PEMBUANGAN, true);
        setRelay(RELAY_PENGISIAN, false);
        setRelay(RELAY_AERATOR, true);
        if (jarakKeAir >= 80.0 || jarakKeAir == 0) {
          setRelay(RELAY_PEMBUANGAN, false);
          currentWaterState = FILLING;
          tSiklusMulai = millis();
        }
        break;

      case FILLING:
        setRelay(RELAY_PEMBUANGAN, false);
        setRelay(RELAY_PENGISIAN, true);
        if (jarakKeAir <= 5.0 && jarakKeAir != 0) {
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
    currentWaterState = IDLE;
    setRelay(RELAY_PEMBUANGAN, cmdPompaBuang);
    setRelay(RELAY_PENGISIAN, cmdPompaIsi);
    setRelay(RELAY_AERATOR, cmdAerator);
    setRelay(RELAY_PROBIOTIK, cmdPompaProbiotik);
  }

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

  delay(10);  // beri waktu task WiFi/idle
}