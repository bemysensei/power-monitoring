
#include <Arduino.h>
#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <MQTT.h>             
#include <ArduinoJson.h>      
#include <ModbusMaster.h>    
#include <Preferences.h>     
#include <time.h>
#include <math.h>
#include <string.h>

const char* MQTT_HOST = "139-59-217-239.sslip.io";
constexpr uint16_t DEFAULT_MQTT_PORT = 8883; 
const char* ISRG_ROOT_X1 = R"PEM(-----BEGIN CERTIFICATE-----
MIIFazCCA1OgAwIBAgIRAIIQz7DSQONZRGPgu2OCiwAwDQYJKoZIhvcNAQELBQAw
TzELMAkGA1UEBhMCVVMxKTAnBgNVBAoTIEludGVybmV0IFNlY3VyaXR5IFJlc2Vh
cmNoIEdyb3VwMRUwEwYDVQQDEwxJU1JHIFJvb3QgWDEwHhcNMTUwNjA0MTEwNDM4
WhcNMzUwNjA0MTEwNDM4WjBPMQswCQYDVQQGEwJVUzEpMCcGA1UEChMgSW50ZXJu
ZXQgU2VjdXJpdHkgUmVzZWFyY2ggR3JvdXAxFTATBgNVBAMTDElTUkcgUm9vdCBY
MTCCAiIwDQYJKoZIhvcNAQEBBQADggIPADCCAgoCggIBAK3oJHP0FDfzm54rVygc
h77ct984kIxuPOZXoHj3dcKi/vVqbvYATyjb3miGbESTtrFj/RQSa78f0uoxmyF+
0TM8ukj13Xnfs7j/EvEhmkvBioZxaUpmZmyPfjxwv60pIgbz5MDmgK7iS4+3mX6U
A5/TR5d8mUgjU+g4rk8Kb4Mu0UlXjIB0ttov0DiNewNwIRt18jA8+o+u3dpjq+sW
T8KOEUt+zwvo/7V3LvSye0rgTBIlDHCNAymg4VMk7BPZ7hm/ELNKjD+Jo2FR3qyH
B5T0Y3HsLuJvW5iB4YlcNHlsdu87kGJ55tukmi8mxdAQ4Q7e2RCOFvu396j3x+UC
B5iPNgiV5+I3lg02dZ77DnKxHZu8A/lJBdiB3QW0KtZB6awBdpUKD9jf1b0SHzUv
KBds0pjBqAlkd25HN7rOrFleaJ1/ctaJxQZBKT5ZPt0m9STJEadao0xAH0ahmbWn
OlFuhjuefXKnEgV4We0+UXgVCwOPjdAvBbI+e0ocS3MFEvzG6uBQE3xDk3SzynTn
jh8BCNAw1FtxNrQHusEwMFxIt4I7mKZ9YIqioymCzLq9gwQbooMDQaHWBfEbwrbw
qHyGO0aoSCqI3Haadr8faqU9GY/rOPNk3sgrDQoo//fb4hVC1CLQJ13hef4Y53CI
rU7m2Ys6xt0nUW7/vGT1M0NPAgMBAAGjQjBAMA4GA1UdDwEB/wQEAwIBBjAPBgNV
HRMBAf8EBTADAQH/MB0GA1UdDgQWBBR5tFnme7bl5AFzgAiIyBpY9umbbjANBgkq
hkiG9w0BAQsFAAOCAgEAVR9YqbyyqFDQDLHYGmkgJykIrGF1XIpu+ILlaS/V9lZL
ubhzEFnTIZd+50xx+7LSYK05qAvqFyFWhfFQDlnrzuBZ6brJFe+GnY+EgPbk6ZGQ
3BebYhtF8GaV0nxvwuo77x/Py9auJ/GpsMiu/X1+mvoiBOv/2X/qkSsisRcOj/KK
NFtY2PwByVS5uCbMiogziUwthDyC3+6WVwW6LLv3xLfHTjuCvjHIInNzktHCgKQ5
ORAzI4JMPJ+GslWYHb4phowim57iaztXOoJwTdwJx4nLCgdNbOhdjsnvzqvHu7Ur
TkXWStAmzOVyyghqpZXjFaH3pO3JLF+l+/+sKAIuvtd7u+Nxe5AW0wdeRlN8NwdC
jNPElpzVmbUq4JUagEiuTDkHzsxHpFKVK7q4+63SM1N95R1NbdWhscdCb+ZAJzVc
oyi3B43njTOQ5yOf+1CceWxG1bQVs5ZufpsMljq4Ui0/1lvh+wjChP4kqKOJ2qxq
4RgqsahDYVvTH9w7jXbyLeiNdd8XM2w9U/t7y0Ff/9yi0GE44Za4rF2LN9d11TPA
mRGunUHBcnWEvgJBQl9nJEiU0Zsnvgc/ubhPgXRR4Xq37Z0j4r7g1SgEEzwxA57d
emyPxgcYxn/eR44/KJ4EBs+lVDR3veyJm+kXQ99b21/+jh5Xos1AnX5iItreGCc=
-----END CERTIFICATE-----
)PEM";
//RS485
constexpr uint8_t  METER_SLAVE_ID = 1;
constexpr uint32_t METER_BAUD = 19200;
constexpr uint32_t METER_SERIAL_MODE = SERIAL_8E1;
constexpr int8_t UART_TX_PIN = 23;
constexpr int8_t UART_RX_PIN = 24;
constexpr int16_t MODBUS_REGISTER_OFFSET = -1;
constexpr bool SWAP_16BIT_WORDS = false;
//REGISTER MAP
constexpr bool READ_UNVERIFIED_FOR_DIAGNOSTICS = true;
constexpr bool MAP_LL_VERIFIED      = true;
constexpr bool MAP_LN_VERIFIED      = true;
constexpr bool MAP_CURRENT_VERIFIED = true;
constexpr bool MAP_POWER_VERIFIED   = true;
constexpr bool MAP_PF_VERIFIED      = true;
constexpr bool MAP_HZ_VERIFIED      = true;
constexpr bool MAP_KWH_VERIFIED     = true;
constexpr uint16_t REG_LL      = 3020;
constexpr uint16_t REG_LN      = 3028;
constexpr uint16_t REG_CURRENT = 3000;
constexpr uint16_t REG_KW      = 3054;
constexpr uint16_t REG_PF      = 3078;
constexpr uint16_t REG_HZ      = 3110;
constexpr uint16_t REG_KWH     = 2700;
//delay
constexpr uint32_t SAMPLE_EVERY_MS       = 30000;
constexpr uint32_t WIFI_RETRY_EVERY_MS   = 20000;
constexpr uint32_t MQTT_RETRY_EVERY_MS   = 10000;
constexpr uint16_t MODBUS_GROUP_GAP_MS   = 80;
constexpr long BANGLADESH_OFFSET_SECONDS = 6L * 3600L;

HardwareSerial MeterUART(1);
ModbusMaster meter;
WiFiClient plaintextTransport;
WiFiClientSecure secureTransport;
MQTTClient mqttClient(512, 2048); 
Preferences prefs;                
String wifiSSID;
String wifiPassword;
String deviceId;
String mqttPassword;
String mqttHost;
String telemetryTopic;
String statusTopic;
uint16_t mqttPort = DEFAULT_MQTT_PORT;
bool gSawMeterReply = false;
String serialLine;

uint32_t lastSampleAt = 0;
uint32_t lastWifiTryAt = 0;
uint32_t lastMqttTryAt = 0;
uint32_t sequenceCounter = 0;
uint32_t bootNonce = 0;
bool clockConfigured = false;

struct MeterSample {
  float ll[3], ln[3], current[3], powerKW[4], pfRaw[4];
  float hz, importedKWh;
  bool okLL, okLN, okI, okP, okPF, okHz, okKWh;
  bool anyResponse;
};

void initSample(MeterSample& s) {
  for (int i=0; i<3; ++i) {
    s.ll[i] = s.ln[i] = s.current[i] = NAN;
  }
  for (int i=0; i<4; ++i) {
    s.powerKW[i] = s.pfRaw[i] = NAN;
  }
  s.hz = s.importedKWh = NAN;
  s.okLL = s.okLN = s.okI = s.okP = s.okPF = s.okHz = s.okKWh = false;
  s.anyResponse = false;
}

float readSwappedFloat(uint16_t word0, uint16_t word1) {
  uint32_t bits = SWAP_16BIT_WORDS
    ? (uint32_t(word1) << 16) | word0
    : (uint32_t(word0) << 16) | word1;
  static_assert(sizeof(float) == sizeof(uint32_t), "Float32 required");
  float out;
  memcpy(&out, &bits, sizeof(out));
  return out;
}

bool inRange(float v, float lo, float hi) {
  return isfinite(v) && v >= lo && v <= hi;
}

bool readFloatBlock(const char* name, bool mappingVerified, uint16_t label,
                    uint8_t count, float* dst, float lo, float hi) {
  if (!mappingVerified && !READ_UNVERIFIED_FOR_DIAGNOSTICS) {
    Serial.printf("[MAP] %-7s disabled until verified in ModScan32\n", name);
    return false;
  }
  int32_t wireAddress = int32_t(label) + MODBUS_REGISTER_OFFSET;
  if (wireAddress < 0 || count < 1 || count > 20) return false;

  meter.clearResponseBuffer();
  uint8_t status = meter.readHoldingRegisters(uint16_t(wireAddress), count*2);
  if (status == meter.ku8MBSuccess || (status >= 1 && status <= 11)) {
    gSawMeterReply = true;
  }
  if (status != meter.ku8MBSuccess) {
    Serial.printf("[MODBUS] %-7s label=%u wire=%ld failed 0x%02X\n",
                  name, label, (long)wireAddress, status);
    return false;
  }

  bool groupOK = mappingVerified;
  for (uint8_t i=0; i<count; ++i) {
    uint16_t a = meter.getResponseBuffer(2*i);
    uint16_t b = meter.getResponseBuffer(2*i + 1);
    float f = readSwappedFloat(a, b);
    bool valid = inRange(f, lo, hi);
    dst[i] = (mappingVerified && valid) ? f : NAN;
    groupOK &= valid;
    Serial.printf("[%s] %-7s +%u raw=%04X %04X decoded=%.6f%s\n",
                  mappingVerified ? "MODBUS" : "CANDIDATE",
                  name, unsigned(2*i), a, b, f,
                  !valid ? "  INVALID" : (mappingVerified ? "" : "  VERIFY ON METER LCD"));
  }
  return groupOK;
}

float pf4qToIec(float raw) {
  if (!inRange(raw, -2.0f, 2.0f)) return NAN;
  if (raw > 1.0f)  return 2.0f - raw;
  if (raw < -1.0f) return -2.0f - raw;
  return raw;
}

MeterSample sampleMeter() {
  MeterSample s;
  initSample(s);
  gSawMeterReply = false;
  s.okLL  = readFloatBlock("VLL", MAP_LL_VERIFIED, REG_LL, 3, s.ll, 0, 650);
  delay(MODBUS_GROUP_GAP_MS);
  s.okLN  = readFloatBlock("VLN", MAP_LN_VERIFIED, REG_LN, 3, s.ln, 0, 400);
  delay(MODBUS_GROUP_GAP_MS);
  s.okI   = readFloatBlock("AMPS", MAP_CURRENT_VERIFIED, REG_CURRENT, 3, s.current, 0, 100);
  delay(MODBUS_GROUP_GAP_MS);
  s.okP   = readFloatBlock("KW", MAP_POWER_VERIFIED, REG_KW, 4, s.powerKW, -100, 100);
  delay(MODBUS_GROUP_GAP_MS);
  s.okPF  = readFloatBlock("PF", MAP_PF_VERIFIED, REG_PF, 4, s.pfRaw, -2, 2);
  delay(MODBUS_GROUP_GAP_MS);
  s.okHz  = readFloatBlock("HZ", MAP_HZ_VERIFIED, REG_HZ, 1, &s.hz, 40, 70);
  delay(MODBUS_GROUP_GAP_MS);
  s.okKWh = readFloatBlock("KWH", MAP_KWH_VERIFIED, REG_KWH, 1, &s.importedKWh, 0, 1e9f);
  s.anyResponse = gSawMeterReply;
  return s;
}

void addFloatOrNull(JsonArray arr, float x) {
  if (isfinite(x)) arr.add(x);
  else arr.add(nullptr);
}

void putFloatOrNull(JsonDocument& doc, const char* key, float x) {
  if (isfinite(x)) doc[key] = x;
  else doc[key] = nullptr;
}

bool isClockSynced() {
  return time(nullptr) > 1735689600;
}

void fillTime(JsonDocument& doc) {
  doc["timestamp_synced"] = isClockSynced();
  doc["epoch"] = isClockSynced() ? (uint32_t)time(nullptr) : 0;
  if (!isClockSynced()) {
    doc["timestamp_utc"] = nullptr;
    doc["timestamp_bdt"] = nullptr;
    return;
  }
  time_t utc = time(nullptr);
  struct tm utcTM;
  gmtime_r(&utc, &utcTM);
  char utcText[28];
  strftime(utcText, sizeof(utcText), "%Y-%m-%dT%H:%M:%SZ", &utcTM);
  doc["timestamp_utc"] = utcText;

  time_t bdt = utc + BANGLADESH_OFFSET_SECONDS;
  struct tm bdtTM;
  gmtime_r(&bdt, &bdtTM);
  char bdtText[32];
  strftime(bdtText, sizeof(bdtText), "%Y-%m-%dT%H:%M:%S+06:00", &bdtTM);
  doc["timestamp_bdt"] = bdtText;
}

float avgValid(const float* v, uint8_t n) {
  float sum = 0.0f;
  uint8_t k = 0;
  for (uint8_t i = 0; i < n; ++i) {
    if (isfinite(v[i])) { sum += v[i]; ++k; }
  }
  return k ? sum / k : NAN;
}

float powerForServer(float kw) {
  if (!isfinite(kw)) return NAN;
  if (kw < 0.0f && kw > -0.005f) return 0.0f;
  return kw >= 0.0f ? kw : NAN;
}

float chipTemperatureC() {
  float t = temperatureRead();
  return inRange(t, -40.0f, 120.0f) ? t : NAN;
}

String encodeSample(const MeterSample& s) {
  JsonDocument doc;
  doc["schema_version"]    = 1;
  doc["device_id"]         = deviceId;
  doc["meter_model"]       = "Schneider PM2130D";
  doc["protocol"]          = "modbus_rtu";
  doc["meter_slave_id"]    = METER_SLAVE_ID;
  doc["ct_primary_a"]      = 60;
  doc["ct_secondary_a"]    = 5;
  doc["uptime_ms"]         = millis();
  doc["sample_id"]         = deviceId + "-" + String(bootNonce, HEX) + "-" + String(++sequenceCounter);
  fillTime(doc);
  const bool allGroupsOK = s.okLL && s.okLN && s.okI && s.okP && s.okPF && s.okHz && s.okKWh;

  doc["st"] = !s.anyResponse ? "no_response" : (allGroupsOK ? "ok" : "partial");

  JsonArray ll = doc["voltage_ll_v"].to<JsonArray>();
  JsonArray ln = doc["voltage_ln_v"].to<JsonArray>();
  JsonArray ii = doc["phase_current_a"].to<JsonArray>();
  JsonArray pp = doc["phase_power_kw"].to<JsonArray>();
  JsonArray pf = doc["phase_pf_iec"].to<JsonArray>();
  for (uint8_t i=0; i<3; ++i) {
    addFloatOrNull(ll, s.ll[i]);
    addFloatOrNull(ln, s.ln[i]);
    addFloatOrNull(ii, s.current[i]);
    addFloatOrNull(pp, s.powerKW[i]);
    addFloatOrNull(pf, pf4qToIec(s.pfRaw[i]));
  }
  putFloatOrNull(doc, "total_power_kw", s.powerKW[3]);
  putFloatOrNull(doc, "total_pf_iec", pf4qToIec(s.pfRaw[3]));
  putFloatOrNull(doc, "frequency_hz", s.hz);
  putFloatOrNull(doc, "import_energy_kwh", s.importedKWh);

  JsonObject q = doc["quality"].to<JsonObject>();
  q["voltage_ll_ok"] = s.okLL;
  q["voltage_ln_ok"] = s.okLN;
  q["current_ok"] = s.okI;
  q["power_ok"] = s.okP;
  q["power_factor_ok"] = s.okPF;
  q["frequency_ok"] = s.okHz;
  q["kwh_ok"] = s.okKWh;
  q["partial"] = !(s.okLL && s.okLN && s.okI && s.okP && s.okPF && s.okHz && s.okKWh);

  JsonObject verified = doc["map_verified"].to<JsonObject>();
  verified["voltage_ll"] = MAP_LL_VERIFIED;
  verified["voltage_ln"] = MAP_LN_VERIFIED;
  verified["current"] = MAP_CURRENT_VERIFIED;
  verified["power"] = MAP_POWER_VERIFIED;
  verified["power_factor"] = MAP_PF_VERIFIED;
  verified["frequency"] = MAP_HZ_VERIFIED;
  verified["import_energy"] = MAP_KWH_VERIFIED;

  float vAvg = avgValid(s.ln, 3);
  float iAvg = avgValid(s.current, 3);
  float pTot = powerForServer(s.powerKW[3]);
  float tC   = chipTemperatureC();
  putFloatOrNull(doc, "voltage", vAvg);
  putFloatOrNull(doc, "current", iAvg);
  putFloatOrNull(doc, "power", pTot);
  putFloatOrNull(doc, "energy", s.importedKWh);
  putFloatOrNull(doc, "temperature", tC);

  const char* vKey[3] = {"voltage_1", "voltage_2", "voltage_3"};
  const char* iKey[3] = {"current_1", "current_2", "current_3"};
  const char* pKey[3] = {"power_1", "power_2", "power_3"};
  for (uint8_t i = 0; i < 3; ++i) {
    putFloatOrNull(doc, vKey[i], s.ln[i]);
    putFloatOrNull(doc, iKey[i], s.current[i]);
    putFloatOrNull(doc, pKey[i], powerForServer(s.powerKW[i]));
  }

  if (!isfinite(vAvg))          Serial.println("[MEASUREMENT] voltage unavailable");
  if (!isfinite(iAvg))          Serial.println("[MEASUREMENT] current unavailable");
  if (!isfinite(pTot))          Serial.println("[MEASUREMENT] total power unavailable/negative; verify import/export interpretation");
  if (!isfinite(s.importedKWh)) Serial.println("[MEASUREMENT] energy (kWh) unavailable");
  if (!isfinite(tC))            Serial.println("[MEASUREMENT] chip temperature unavailable");

  String json;
  serializeJson(doc, json);
  return json;
}

void onWiFiDiagnostic(WiFiEvent_t event, WiFiEventInfo_t info) {
  switch (event) {
    case ARDUINO_EVENT_WIFI_STA_CONNECTED:
      Serial.println("[WIFI] Associated with access point.");
      break;
    case ARDUINO_EVENT_WIFI_STA_DISCONNECTED:
      Serial.printf("[WIFI] Disconnected; reason=%u\n",
                    info.wifi_sta_disconnected.reason);
      break;
    case ARDUINO_EVENT_WIFI_STA_GOT_IP:
      Serial.println("[WIFI] DHCP assigned an IP address.");
      break;
    default: break;
  }
}

bool validDeviceId(const String& v) {
  if (v.length() < 1 || v.length() > 48) return false;
  for (size_t i = 0; i < v.length(); ++i) {
    char c = v[i];
    if (!((c >= 'a' && c <= 'z') || (c >= 'A' && c <= 'Z') ||
          (c >= '0' && c <= '9') || c == '-' || c == '_')) return false;
  }
  return true;
}

void loadConfig() {
  prefs.begin("powernet", false);
  deviceId = prefs.getString("device_id", "");
  mqttPassword = prefs.getString("mqtt_pwd", "");
  mqttHost = prefs.getString("mqtt_host", MQTT_HOST);
  if (mqttHost.length() == 0) mqttHost = MQTT_HOST;
  wifiSSID = prefs.getString("wifi_ssid", "");
  wifiPassword = prefs.getString("wifi_pwd", "");
  mqttPort = prefs.getUShort("mqtt_port", DEFAULT_MQTT_PORT);
  if (mqttPort != 8883 && mqttPort != 1883) {
    Serial.println("[CONFIG] Unsupported stored port; reverting to 8883.");
    mqttPort = DEFAULT_MQTT_PORT;
  }
  telemetryTopic = String("device/") + deviceId + "/telemetry";
  statusTopic = String("device/") + deviceId + "/status";
}

void printConfigSummary() {
  Serial.printf("[CONFIG] MQTT host: %s\n", mqttHost.c_str());
  Serial.printf("[CONFIG] MQTT port: %u (%s)\n", mqttPort,
                mqttPort == 8883 ? "verified TLS" : "PLAINTEXT - TEST ONLY");
  Serial.printf("[CONFIG] Device ID / username / client ID: %s\n",
                deviceId.length() ? deviceId.c_str() : "<NOT SET>");
  Serial.printf("[CONFIG] MQTT password: %s\n", mqttPassword.length() ? "SET (hidden)" : "NOT SET");
  Serial.printf("[CONFIG] Wi-Fi SSID: %s\n", wifiSSID.length() ? wifiSSID.c_str() : "<NOT SET>");
  Serial.printf("[CONFIG] Wi-Fi password: %s\n", wifiPassword.length() ? "SET (hidden)" : "NOT SET");
  Serial.printf("[CONFIG] Telemetry topic: %s\n", telemetryTopic.c_str());
  Serial.printf("[CONFIG] Status topic: %s\n", statusTopic.c_str());
}

void printProvisionHelp() {
  Serial.println("[CONFIG] Commands (Serial Monitor: 115200 baud; line ending NEWLINE):");
  Serial.println("  SHOW_CONFIG");
  Serial.println("  SET_ID <gateway-id>              (also MQTT username & client ID)");
  Serial.println("  SET_MQTT_PASSWORD <secret>        (will not be echoed by firmware)");
  Serial.println("  SET_MQTT_HOST <DNS name>          (for future domain changes)");
  Serial.println("  SET_WIFI_SSID <Wi-Fi name>");
  Serial.println("  SET_WIFI_PASSWORD <secret>        (will not be echoed by firmware)");
  Serial.println("  SET_MQTT_PORT 8883                (TLS; default)");
  Serial.println("  SET_MQTT_PORT 1883                (unencrypted; test only)");
  Serial.println("[CONFIG] Each SET is saved in NVS and restarts the device.");
}

void handleSerialCommand(String cmd) {
  cmd.trim();
  if (!cmd.length()) return;
  if (cmd == "SHOW_CONFIG") { printConfigSummary(); return; }
  if (cmd == "HELP") { printProvisionHelp(); return; }
  int pos = cmd.indexOf(' ');
  String key = pos >= 0 ? cmd.substring(0, pos) : cmd;
  String value = pos >= 0 ? cmd.substring(pos + 1) : String("");
  value.trim();
  bool saved = false;
  if (key == "SET_ID") {
    if (!validDeviceId(value)) {
      Serial.println("[CONFIG] Invalid ID (1-48 chars: A-Z, a-z, 0-9, _, -)."); return;
    }
    saved = prefs.putString("device_id", value) > 0;
  } else if (key == "SET_MQTT_PASSWORD") {
    if (value.length() < 1 || value.length() > 160) {
      Serial.println("[CONFIG] MQTT password must be 1-160 characters."); return;
    }
    saved = prefs.putString("mqtt_pwd", value) > 0;
  } else if (key == "SET_MQTT_HOST") {
    if (value.length() < 4 || value.length() > 120 || value.indexOf(' ') >= 0 ||
        value.indexOf('/') >= 0 || value.indexOf(':') >= 0) {
      Serial.println("[CONFIG] Invalid hostname (DNS name, no port or URL scheme)."); return;
    }
    saved = prefs.putString("mqtt_host", value) > 0;
  } else if (key == "SET_WIFI_SSID") {
    if (value.length() < 1 || value.length() > 32) {
      Serial.println("[CONFIG] Wi-Fi SSID must be 1-32 characters."); return;
    }
    saved = prefs.putString("wifi_ssid", value) > 0;
  } else if (key == "SET_WIFI_PASSWORD") {
    if (value.length() > 63) {
      Serial.println("[CONFIG] Wi-Fi password too long."); return;
    }
    
    size_t count = prefs.putString("wifi_pwd", value);
    saved = (count > 0 || value.length() == 0);
  } else if (key == "SET_MQTT_PORT") {
    if (value != "8883" && value != "1883") {
      Serial.println("[CONFIG] Choose 8883 (TLS) or 1883 (test-only plaintext)."); return;
    }
    saved = prefs.putUShort("mqtt_port", (uint16_t)value.toInt()) == 2;
  } else {
    Serial.println("[CONFIG] Unknown command. Type HELP."); return;
  }
  
  Serial.println(saved ? "[CONFIG] Saved in NVS. Rebooting ..." : "[CONFIG] NVS save failed.");
  if (saved) { Serial.flush(); delay(250); ESP.restart(); }
}

void serviceSerial() {
  while (Serial.available()) {
    char c = (char)Serial.read();
    if (c == '\r') continue;
    if (c == '\n') {
      String command = serialLine;
      serialLine = "";
      handleSerialCommand(command);
    } else if (serialLine.length() < 240) {
      serialLine += c;
    } else {
      serialLine = "";
      Serial.println("[CONFIG] Serial command too long; discarded.");
    }
  }
}

//Network

void maintainWiFi() {
  if (wifiSSID.isEmpty()) return;
  if (WiFi.status() == WL_CONNECTED) {
    if (!clockConfigured) {
      configTime(0, 0, "pool.ntp.org", "time.google.com");
      clockConfigured = true;
      Serial.printf("[WIFI] Connected, IP=%s; NTP requested.\n",
                    WiFi.localIP().toString().c_str());
    }
    return;
  }
  clockConfigured = false;
  if ((uint32_t)(millis() - lastWifiTryAt) >= WIFI_RETRY_EVERY_MS) {
    lastWifiTryAt = millis();
    Serial.printf("[WIFI] Waiting for automatic reconnect, status=%d.\n", (int)WiFi.status());
    
  }
}

bool credentialsReady() {
  return validDeviceId(deviceId) && mqttPassword.length() > 0;
}

void setupMQTT() {
  if (!credentialsReady()) {
    Serial.println("[MQTT] Provision device ID and MQTT password using serial console.");
    return;
  }
  mqttClient.setOptions(60, false, 5000); 
  
  mqttClient.setWill(statusTopic.c_str(), "offline", true, 1);

  if (mqttPort == 8883) {
    secureTransport.setCACert(ISRG_ROOT_X1); 
    mqttClient.begin(mqttHost.c_str(), mqttPort, secureTransport);
  } else {
    Serial.println("[SECURITY] WARNING: port 1883 transmits credentials UNENCRYPTED.");
    mqttClient.begin(mqttHost.c_str(), mqttPort, plaintextTransport);
  }
}

void maintainMQTT() {
  if (!credentialsReady() || WiFi.status() != WL_CONNECTED) return;
  if (mqttPort == 8883 && !isClockSynced()) {
    static uint32_t lastNtpWait = 0;
    if ((uint32_t)(millis() - lastNtpWait) > 15000) {
      lastNtpWait = millis();
      Serial.println("[MQTT] Waiting for NTP time before certificate validation.");
    }
    return;
  }
  if (mqttClient.connected()) {
    mqttClient.loop();
    return;
  }
  if ((uint32_t)(millis() - lastMqttTryAt) < MQTT_RETRY_EVERY_MS) return;
  lastMqttTryAt = millis();

  Serial.printf("[MQTT] Connecting to %s:%u as device=%s (clientID MUST equal username) ...\n",
                mqttHost.c_str(), mqttPort, deviceId.c_str());
  if (mqttClient.connect(deviceId.c_str(), deviceId.c_str(), mqttPassword.c_str())) {
    Serial.println("[MQTT] Connected (keepalive=60, cleanSession=false).");
    if (!mqttClient.publish(statusTopic.c_str(), "online", true, 1)) {
      Serial.printf("[MQTT] ONLINE QoS1 retained publish failed, lastError=%d\n",
                    (int)mqttClient.lastError());
    } else {
      Serial.printf("[MQTT] ONLINE QoS1 retained published to %s\n", statusTopic.c_str());
    }
  } else {
    Serial.printf("[MQTT] CONNECT failed: returnCode=%d lastError=%d (no plaintext fallback).\n",
                  (int)mqttClient.returnCode(), (int)mqttClient.lastError());
  }
}

bool publishMQTT(const String& json) {
  if (!mqttClient.connected()) {
    Serial.println("[MQTT] Offline; sample not uploaded. No persistent data queue in this version.");
    return false;
  }
  // QoS1 publish; library waits for broker acknowledgement while connected.
  bool ok = mqttClient.publish(telemetryTopic.c_str(), json.c_str(), false, 1);
  Serial.printf("[MQTT] Telemetry QoS1 %s: %u JSON bytes -> %s (error=%d)\n",
                ok ? "ACKNOWLEDGED" : "FAILED", (unsigned)json.length(),
                telemetryTopic.c_str(), (int)mqttClient.lastError());
  mqttClient.loop();
  return ok;
}

void setup() {
  Serial.begin(115200);
  delay(1500);
  Serial.println("\nPower.Net PM2130D ESP32-C5: verified Modbus + authenticated MQTT TLS");
  Serial.println("Modbus timings, registers, word order and pins retained from working sketch.");
  bootNonce = esp_random();
  loadConfig();
  printConfigSummary();
  printProvisionHelp();

  MeterUART.begin(METER_BAUD, METER_SERIAL_MODE, UART_RX_PIN, UART_TX_PIN);
  meter.begin(METER_SLAVE_ID, MeterUART);
  Serial.printf("[MODBUS] Slave=%u baud=%lu 8E1 TX=%d RX=%d offset=%d swap=%d\n",
                METER_SLAVE_ID, (unsigned long)METER_BAUD, UART_TX_PIN, UART_RX_PIN,
                MODBUS_REGISTER_OFFSET, (int)SWAP_16BIT_WORDS);

  setupMQTT();
  WiFi.onEvent(onWiFiDiagnostic);
  WiFi.mode(WIFI_STA);
  WiFi.setAutoReconnect(true);
  if (!wifiSSID.isEmpty()) {
    WiFi.begin(wifiSSID.c_str(), wifiPassword.c_str());
    lastWifiTryAt = millis();
    Serial.println("[WIFI] Connecting (automatic reconnection enabled)...");
  } else {
    Serial.println("[WIFI] Not configured. Provision SSID and password via Serial Monitor.");
  }
}

void loop() {
  serviceSerial();
  maintainWiFi();
  maintainMQTT();

  static bool first = true;
  if (first || (uint32_t)(millis() - lastSampleAt) >= SAMPLE_EVERY_MS) {
    first = false;
    lastSampleAt = millis();
    Serial.println("\n================ READ PM2130D ================");
    MeterSample s = sampleMeter();
    String payload = encodeSample(s);
    Serial.printf("[JSON] %s\n", payload.c_str());
    maintainMQTT();
    bool sent = publishMQTT(payload);
    if (!sent) {
      Serial.println("[GATE] Sample not delivered. Offline buffering is not yet implemented.");
    }
    Serial.println("============== END SAMPLE ==================");
  }
  delay(30);
}
