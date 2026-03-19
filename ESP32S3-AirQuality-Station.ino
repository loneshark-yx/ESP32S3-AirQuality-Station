#include <WiFi.h>
#include <PubSubClient.h>
#include <Wire.h>
#include <bsec2.h>
#include <HTTPClient.h>
#include <ArduinoJson.h>

// 引入敏感信息
#include "secrets.h"

// ===== RGB_LED =====
#define RGB_LED_Pin 48
uint32_t lastLedCycle = 0;
bool ledOn = false;

// 参数
uint32_t ledPeriod = 5000;   // 周期
uint32_t ledOnTime = 500;    // 点亮时间固定0.5秒

uint8_t ledR = 0, ledG = 0, ledB = 0;
bool ledEnable = false;
bool ledNightMute = false;

// ===== OLED =====
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#include <time.h>         // NTP 时间
#include <Preferences.h>  // NVS for BSEC state

// ---------- OLED 配置 ----------
#define OLED_ADDR 0x3C
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);

// ---------- WiFi / MQTT 配置已移至 secrets.h ----------

// ---------- Weather 配置 ----------
struct WeatherData {
  float temp = NAN;
  float feels_like = NAN;
  float humidity = NAN;
  float pressure = NAN;
  String description = "--";
  String icon = "";
  bool valid = false;
} g_weather;

// ... (topics 保持不变)
const char* TOPIC_TEMP   = "modbus-mqtt/sensor/bme680_temperature";
const char* TOPIC_HUM    = "modbus-mqtt/sensor/bme680_humidity";
const char* TOPIC_PRES   = "modbus-mqtt/sensor/bme680_pressure";
const char* TOPIC_GAS    = "modbus-mqtt/sensor/bme680_gas";
const char* TOPIC_IAQ    = "modbus-mqtt/sensor/bme680_iaq";
const char* TOPIC_TVOC   = "modbus-mqtt/sensor/ags10_tvoc";
const char* TOPIC_JSON   = "modbus-mqtt/sensor/esp32_telemetry";
const char* TOPIC_STATUS = "modbus-mqtt/sensor/bme680_status";   // online/offline (LWT)

// ... (I2C 引脚等保持不变)
#define SDA_PIN 4
#define SCL_PIN 5
static const uint32_t I2C_FAST_HZ = 100000;
static const uint32_t I2C_AGS10_HZ = 10000; // 说明书要求 <=15kHz

// ---------- Weather Fetch ----------
void updateWeather() {
  if (WiFi.status() == WL_CONNECTED) {
    HTTPClient http;
    http.begin(WEATHER_URL);
    Serial.print("[HTTP] GET weather... ");
    int httpCode = http.GET();
    if (httpCode == HTTP_CODE_OK) {
      String payload = http.getString();
      JsonDocument doc;
      DeserializationError error = deserializeJson(doc, payload);
      if (!error) {
        g_weather.temp = doc["main"]["temp"];
        g_weather.feels_like = doc["main"]["feels_like"];
        g_weather.humidity = doc["main"]["humidity"];
        g_weather.pressure = doc["main"]["pressure"];
        g_weather.description = doc["weather"][0]["main"].as<String>();
        g_weather.icon = doc["weather"][0]["icon"].as<String>();
        g_weather.valid = true;
        Serial.printf("OK! Temp: %.1f C, Desc: %s, Icon: %s\n", g_weather.temp, g_weather.description.c_str(), g_weather.icon.c_str());
      } else {
        Serial.print("JSON Parse Fail: ");
        Serial.println(error.c_str());
      }
    } else {
      Serial.printf("Failed, error: %s\n", http.errorToString(httpCode).c_str());
    }
    http.end();
  } else {
    Serial.println("WiFi not connected, skipping weather update");
  }
}

// ---------- AGS10 ----------
#define AGS10_ADDR 0x1A
static const uint32_t AGS10_SAMPLE_INTERVAL_MS = 2000;   // 说明书：采样周期 >= 2s
static const uint32_t AGS10_PROCESS_TIME_MS   = 1500;    // 说明书：数据采集处理时间 1500ms
static const uint32_t AGS10_WARMUP_MS         = 120000;  // 说明书：预热 >= 120s

#define SAMPLE_RATE BSEC_SAMPLE_RATE_LP
#define ARRAY_LEN(x) (sizeof(x) / sizeof((x)[0]))

// ---------- Keys (OLED module buttons) ----------
#define K1_PIN 6
#define K2_PIN 7
#define K3_PIN 15
#define K4_PIN 16

static const uint32_t OLED_TIMEOUT_MS = 60UL * 1000UL;      // 1 minute inactivity -> screen off
static const uint32_t MQTT_PUB_INTERVAL_MS = 60UL * 1000UL; // publish telemetry every 60s

// Simple wake flag + debounce
static volatile bool g_keyPressed = false;
static volatile uint32_t g_lastKeyIsrMs = 0;
static uint32_t g_lastUserActionMs = 0;
static bool g_oledOn = true;

// ISR: any key triggers wake
void IRAM_ATTR onAnyKeyISR() {
  uint32_t now = millis();
  if (now - g_lastKeyIsrMs > 50) { // 50ms debounce
    g_lastKeyIsrMs = now;
    g_keyPressed = true;
  }
}

// ---------- NTP / 时区 ----------
#define TZ_INFO "CST-8"
static const char* NTP1 = "pool.ntp.org";
static const char* NTP2 = "time.nist.gov";

WiFiClient espClient;
PubSubClient mqtt(espClient);
Bsec2 envSensor;

// ====== NVS for BSEC state ======
Preferences prefs;
static uint8_t bsecState[BSEC_MAX_STATE_BLOB_SIZE];
static uint32_t lastStateSaveMs = 0;
static int lastAccSaved = -1;

// ====== callback -> loop 数据 ======
volatile bool g_hasNew = false;

struct {
  float temp = NAN;
  float hum = NAN;
  float pres_hpa = NAN;
  float gas_kohm = NAN;

  float iaq = NAN;
  int   iaq_acc = -1;
  float static_iaq = NAN;
  float co2_eq = NAN;
  float bvoc_eq = NAN;

  uint32_t tvoc_ppb = 0;
  bool tvoc_valid = false;
  bool ags10_present = false;
  uint32_t ts_ms = 0;
} g_data;

// ---------- MQTT ----------
static void publishFloat(const char* topic, float v, uint8_t decimals = 2) {
  if (isnan(v)) return;
  char buf[32];
  dtostrf(v, 0, decimals, buf);
  mqtt.publish(topic, buf, true);
}

static void publishUInt32(const char* topic, uint32_t v) {
  char buf[16];
  snprintf(buf, sizeof(buf), "%lu", (unsigned long)v);
  mqtt.publish(topic, buf, true);
}

// ---------- AGS10 ----------
static uint8_t calcCRC8(const uint8_t* dat, uint8_t len) {
  uint8_t crc = 0xFF;
  for (uint8_t byte = 0; byte < len; byte++) {
    crc ^= dat[byte];
    for (uint8_t i = 0; i < 8; i++) {
      if (crc & 0x80) crc = (uint8_t)((crc << 1) ^ 0x31);
      else crc <<= 1;
    }
  }
  return crc;
}

static bool ags10SendReadCommand() {
  Wire.setClock(I2C_AGS10_HZ);
  Wire.beginTransmission(AGS10_ADDR);
  Wire.write(0x00);  // 数据采集寄存器
  uint8_t rc = Wire.endTransmission();
  Wire.setClock(I2C_FAST_HZ);
  return (rc == 0);
}

static bool ags10ReadResult(uint32_t& tvoc_ppb) {
  Wire.setClock(I2C_AGS10_HZ);
  uint8_t n = Wire.requestFrom((uint8_t)AGS10_ADDR, (uint8_t)5);
  if (n != 5) {
    Wire.setClock(I2C_FAST_HZ);
    return false;
  }

  uint8_t buf[5];
  for (int i = 0; i < 5; i++) {
    buf[i] = Wire.read();
  }
  Wire.setClock(I2C_FAST_HZ);

  // CRC 校验：Data1~Data4
  if (calcCRC8(buf, 4) != buf[4]) {
    Serial.println("AGS10 CRC mismatch");
    return false;
  }

  // Data1 bit0 = RDY, 0 表示数据已更新；1 表示数据未更新/采集中
  if (buf[0] & 0x01) {
    Serial.println("AGS10 RDY=1 (data not ready)");
    return false;
  }

  tvoc_ppb = ((uint32_t)buf[1] << 16) |
             ((uint32_t)buf[2] << 8)  |
             ((uint32_t)buf[3]);
  return true;
}

static void updateAGS10IfDue() {
  static bool commandPending = false;
  static uint32_t lastCycleMs = 0;
  static uint32_t commandSentMs = 0;

  uint32_t now = millis();

  // 还没到预热时间，只探测地址，不采信数据
  if (now < AGS10_WARMUP_MS) {
    return;
  }

  // 先发采样命令
  if (!commandPending) {
    if (now - lastCycleMs < AGS10_SAMPLE_INTERVAL_MS) return;

    if (ags10SendReadCommand()) {
      g_data.ags10_present = true;
      commandPending = true;
      commandSentMs = now;
      lastCycleMs = now;
    } else {
      g_data.ags10_present = false;
      g_data.tvoc_valid = false;
      Serial.println("AGS10 command write failed");
      lastCycleMs = now;
    }
    return;
  }

  // 到处理时间后再读取 5 字节结果
  if (now - commandSentMs >= AGS10_PROCESS_TIME_MS) {
    uint32_t tvoc = 0;
    if (ags10ReadResult(tvoc)) {
      g_data.tvoc_ppb = tvoc;
      g_data.tvoc_valid = true;
    } else {
      g_data.tvoc_valid = false;
    }
    commandPending = false;
  }
}

static bool probeAGS10() {
  Wire.setClock(I2C_AGS10_HZ);
  Wire.beginTransmission(AGS10_ADDR);
  uint8_t rc = Wire.endTransmission();
  Wire.setClock(I2C_FAST_HZ);
  return (rc == 0);
}

// ---------- RGB LED ----------
void updateLEDByTVOC(uint32_t tvoc) {

  if (tvoc <= 220) {
    ledEnable = false;
    neopixelWrite(RGB_LED_Pin, 0, 0, 0);
    return;
  }

  ledEnable = true;

  if (tvoc <= 660) {
    // 蓝色：5秒周期
    ledR = 0; ledG = 0; ledB = 20;
    ledPeriod = 5000;
  }
  else if (tvoc <= 2200) {
    // 黄色：2秒周期
    ledR = 80; ledG = 80; ledB = 0;
    ledPeriod = 2000;
  }
  else {
    // 红色：1秒周期
    ledR = 150; ledG = 0; ledB = 0;
    ledPeriod = 1000;
  }
}

void updateLedNightMode() {
  struct tm tinfo;
  if (getLocalTime(&tinfo, 50)) {
    ledNightMute = (tinfo.tm_hour >= 22 || tinfo.tm_hour < 7);
  }
}

void handleLED() {

  if (ledNightMute) {
    if (ledOn) {
      neopixelWrite(RGB_LED_Pin, 0, 0, 0);
      ledOn = false;
    }
    return;
  }

  if (!ledEnable) {
    if (ledOn) {
      neopixelWrite(RGB_LED_Pin, 0, 0, 0);
      ledOn = false;
    }
    return;
  }

  uint32_t now = millis();
  uint32_t elapsed = now - lastLedCycle;

  // 新周期开始
  if (elapsed >= ledPeriod) {
    lastLedCycle = now;
    elapsed = 0;
  }

  // 前0.5秒亮，其余时间灭
  if (elapsed < ledOnTime) {
    if (!ledOn) {
      neopixelWrite(RGB_LED_Pin, ledR, ledG, ledB);
      ledOn = true;
    }
  } else {
    if (ledOn) {
      neopixelWrite(RGB_LED_Pin, 0, 0, 0);
      ledOn = false;
    }
  }
}

// ---------- BSEC2 State ----------
static bool loadBsecState() {
  prefs.begin("bsec2", true);
  size_t n = prefs.getBytesLength("state");
  if (n != sizeof(bsecState)) {
    prefs.end();
    Serial.printf("BSEC state not found (len=%u)\n", (unsigned)n);
    return false;
  }
  prefs.getBytes("state", bsecState, sizeof(bsecState));
  prefs.end();

  bool ok = envSensor.setState(bsecState);
  Serial.printf("BSEC setState => %s\n", ok ? "OK" : "FAIL");
  return ok;
}

static void saveBsecStateIfNeeded(int iaqAcc) {
  const uint32_t intervalMs = 6UL * 60UL * 60UL * 1000UL;
  uint32_t now = millis();

  bool accUp = (iaqAcc >= 0 && iaqAcc > lastAccSaved);
  bool timeDue = (now - lastStateSaveMs) >= intervalMs;

  if (!accUp && !timeDue) return;

  bool ok = envSensor.getState(bsecState);
  if (!ok) {
    Serial.println("BSEC getState => FAIL");
    return;
  }

  prefs.begin("bsec2", false);
  prefs.putBytes("state", bsecState, sizeof(bsecState));
  prefs.end();

  lastStateSaveMs = now;
  if (accUp) lastAccSaved = iaqAcc;

  Serial.printf("BSEC state saved (acc=%d)\n", iaqAcc);
}

// ---------- BSEC2 Status ----------
static void errHalt() { while (1) delay(1000); }

static void checkBsecStatus(Bsec2 bsec) {
  if (bsec.status < BSEC_OK) {
    Serial.print("BSEC error code: ");
    Serial.println(bsec.status);
    errHalt();
  } else if (bsec.status > BSEC_OK) {
    Serial.print("BSEC warning code: ");
    Serial.println(bsec.status);
  }

  if (bsec.sensor.status < BME68X_OK) {
    Serial.print("BME68X error code: ");
    Serial.println(bsec.sensor.status);
    errHalt();
  } else if (bsec.sensor.status > BME68X_OK) {
    Serial.print("BME68X warning code: ");
    Serial.println(bsec.sensor.status);
  }
}

// ---------- BSEC2 Callback ----------
void newDataCallback(const bme68xData data, const bsecOutputs outputs, Bsec2 bsec) {
  (void)data; (void)bsec;
  if (!outputs.nOutputs) return;

  g_data.ts_ms = millis();

  for (uint8_t i = 0; i < outputs.nOutputs; i++) {
    const bsecData o = outputs.output[i];
    switch (o.sensor_id) {
      case BSEC_OUTPUT_RAW_TEMPERATURE: g_data.temp = o.signal; break;
      case BSEC_OUTPUT_RAW_HUMIDITY:    g_data.hum = o.signal; break;
      case BSEC_OUTPUT_RAW_PRESSURE:    g_data.pres_hpa = o.signal; break; // 如果显示成 101000，请改成 o.signal / 100.0f
      case BSEC_OUTPUT_RAW_GAS:         g_data.gas_kohm = o.signal / 1000.0f; break;
      case BSEC_OUTPUT_IAQ:             g_data.iaq = o.signal; g_data.iaq_acc = (int)o.accuracy; break;
      case BSEC_OUTPUT_STATIC_IAQ:      g_data.static_iaq = o.signal; break;
      case BSEC_OUTPUT_CO2_EQUIVALENT:  g_data.co2_eq = o.signal; break;
      case BSEC_OUTPUT_BREATH_VOC_EQUIVALENT: g_data.bvoc_eq = o.signal; break;
      default: break;
    }
  }

  g_hasNew = true;
}

// ---------- WiFi / MQTT ----------
static void wifiConnect() {
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASS);

  Serial.print("WiFi connecting");
  while (WiFi.status() != WL_CONNECTED) {
    delay(300);
    Serial.print(".");
  }
  Serial.print("\nWiFi OK, IP=");
  Serial.println(WiFi.localIP());
  WiFi.setSleep(true);
}

static void mqttConnect() {
  mqtt.setServer(MQTT_HOST, MQTT_PORT);

  while (!mqtt.connected()) {
    String clientId = "esp32s3-bme680-" + String((uint32_t)ESP.getEfuseMac(), HEX);

    Serial.printf("MQTT connecting to %s:%u as %s (user=%s)\n",
                  MQTT_HOST, MQTT_PORT, clientId.c_str(), MQTT_USER);

    bool ok = mqtt.connect(clientId.c_str(), MQTT_USER, MQTT_PASS,
                           TOPIC_STATUS, 1, true, "offline");
    if (ok) {
      Serial.println("MQTT OK");
      mqtt.publish(TOPIC_STATUS, "online", true);
      mqtt.publish("test/esp32s3/hello", "hello", false);
    } else {
      Serial.printf("MQTT fail, rc=%d\n", mqtt.state());
      delay(1000);
    }
  }
}

// ---------- NTP ----------
static void ntpInit() {
  configTzTime(TZ_INFO, NTP1, NTP2);
}

// ---------- OLED ----------
static void oledPower(bool on) {
  if (!display.width()) return;
  if (on && !g_oledOn) {
    display.ssd1306_command(SSD1306_DISPLAYON);
    g_oledOn = true;
  } else if (!on && g_oledOn) {
    display.ssd1306_command(SSD1306_DISPLAYOFF);
    g_oledOn = false;
  }
}

static const char* weekdayCN(int wday) {
  static const char* w[] = {"Sun","Mon","Tue","Wed","Thu","Fri","Sta"};
  if (wday < 0 || wday > 6) return "--";
  return w[wday];
}

static const char* tvocGrade(uint32_t v) {
  if (v <= 65) return "EXC";
  if (v <= 220) return "GOOD";
  if (v <= 660) return "LIGHT";
  if (v <= 2200) return "MOD";
  return "BAD";
}

static void drawWeatherIcon(int x, int y, String icon) {
  if (icon.length() < 2) return;
  String type = icon.substring(0, 2);
  
  if (type == "01") { // Clear sky
    display.drawCircle(x + 7, y + 7, 4, SSD1306_WHITE);
    for(int i=0; i<360; i+=45) {
      float rad = i * 3.14159 / 180.0;
      display.drawLine(x+7 + cos(rad)*5, y+7 + sin(rad)*5, x+7 + cos(rad)*7, y+7 + sin(rad)*7, SSD1306_WHITE);
    }
  } else if (type == "02" || type == "03" || type == "04") { // Clouds
    display.fillCircle(x + 4, y + 9, 3, SSD1306_WHITE);
    display.fillCircle(x + 8, y + 9, 4, SSD1306_WHITE);
    display.fillCircle(x + 8, y + 5, 3, SSD1306_WHITE);
    display.fillCircle(x + 12, y + 9, 3, SSD1306_WHITE);
  } else if (type == "09" || type == "10" || type == "11") { // Rain
    display.fillCircle(x + 4, y + 6, 3, SSD1306_WHITE);
    display.fillCircle(x + 8, y + 6, 4, SSD1306_WHITE);
    display.fillCircle(x + 12, y + 6, 3, SSD1306_WHITE);
    display.drawLine(x + 4, y + 10, x + 2, y + 13, SSD1306_WHITE);
    display.drawLine(x + 8, y + 10, x + 6, y + 13, SSD1306_WHITE);
    display.drawLine(x + 12, y + 10, x + 10, y + 13, SSD1306_WHITE);
  } else if (type == "13") { // Snow
    display.drawPixel(x+8, y+4, SSD1306_WHITE);
    display.drawPixel(x+4, y+8, SSD1306_WHITE);
    display.drawPixel(x+12, y+8, SSD1306_WHITE);
    display.drawPixel(x+8, y+12, SSD1306_WHITE);
    display.drawLine(x+5, y+5, x+11, y+11, SSD1306_WHITE);
    display.drawLine(x+11, y+5, x+5, y+11, SSD1306_WHITE);
  } else { // Mist or unknown
    display.drawLine(x+2, y+4, x+14, y+4, SSD1306_WHITE);
    display.drawLine(x+5, y+8, x+11, y+8, SSD1306_WHITE);
    display.drawLine(x+2, y+12, x+14, y+12, SSD1306_WHITE);
  }
}

static void drawHeader() {
  display.setCursor(0, 0);
  display.setTextColor(SSD1306_WHITE);
  display.setTextSize(1);

  struct tm tinfo;
  bool ok = getLocalTime(&tinfo, 1000);

  if (ok) {
    char buf[20];
    strftime(buf, sizeof(buf), "%Y-%m-%d", &tinfo);
    display.println(buf);

    display.setCursor(0, 8);
    display.print(weekdayCN(tinfo.tm_wday));
    display.print(" ");
    char buf2[10];
    strftime(buf2, sizeof(buf2), "%H:%M", &tinfo);
    display.println(buf2);
  } else {
    display.println("---- -- --");
    display.setCursor(0, 8);
    display.println("NTP syncing...");
  }

  display.drawLine(0, 15, 127, 15, SSD1306_WHITE);
}

static void drawPage(uint8_t page) {
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  int y = 18;

  if (page == 0) {
    // 文字恢复左对齐
    display.setCursor(0, y);
    display.print("Wuhan: "); display.println(g_weather.description);
    display.setCursor(0, y + 12);
    display.print("T: "); display.print(isnan(g_weather.temp) ? 0.0 : g_weather.temp, 1); display.println(" C");

    // 如果有图标，在右上角显示 (x=110)
    if (g_weather.valid && g_weather.icon.length() > 0) {
      drawWeatherIcon(110, y, g_weather.icon);
    }

    display.setCursor(0, y + 24);
    display.print("Feels: "); display.print(isnan(g_weather.feels_like) ? 0.0 : g_weather.feels_like, 1); display.println(" C");

    display.setCursor(0, y + 36);
    display.print("H: "); display.print(isnan(g_weather.humidity) ? 0.0 : g_weather.humidity, 0); display.println(" %");

    display.setCursor(64, y + 36);
    display.print("P:"); display.print(isnan(g_weather.pressure) ? 0.0 : g_weather.pressure, 0);
  }
  else if (page == 1) {
    display.setCursor(0, y);
    display.print("T: "); display.print(isnan(g_data.temp) ? 0.0 : g_data.temp, 1); display.println(" C");

    display.setCursor(0, y + 12);
    display.print("H: "); display.print(isnan(g_data.hum) ? 0.0 : g_data.hum, 0); display.println(" %");

    display.setCursor(0, y + 24);
    display.print("P: "); display.print(isnan(g_data.pres_hpa) ? 0.0 : g_data.pres_hpa, 1); display.println(" hPa");

    display.setCursor(0, y + 36);
    display.print("G: "); display.print(isnan(g_data.gas_kohm) ? 0.0 : g_data.gas_kohm, 1); display.println(" KOhm");
  }
  else if (page == 2) {
    display.setCursor(0, y);
    display.print("IAQ: "); display.print(isnan(g_data.iaq) ? 0.0 : g_data.iaq, 0);
    display.print(" A:"); display.println(g_data.iaq_acc);

    display.setCursor(0, y + 12);
    display.print("CO2: "); display.print(isnan(g_data.co2_eq) ? 0.0 : g_data.co2_eq, 0); display.println(" ppm");

    display.setCursor(0, y + 24);
    display.print("bVOC:"); display.print(isnan(g_data.bvoc_eq) ? 0.0 : g_data.bvoc_eq, 2); display.println(" ppm");

    display.setCursor(0, y + 36);
    display.print("TVOC:");
    if (g_data.tvoc_valid) display.print(g_data.tvoc_ppb);
    else display.print("--");
    display.println(" ppb");
  }
  else if (page == 3) {
    display.setCursor(0, y);
    if (millis() < AGS10_WARMUP_MS) {
      display.print("AGS10 Warmup: ");
      display.print((AGS10_WARMUP_MS - millis()) / 1000);
      display.println("s");
    } else {
      display.print("TVOC: ");
      if (g_data.tvoc_valid) display.print(g_data.tvoc_ppb);
      else display.print("--");
      display.println(" ppb");
    }

    display.setCursor(0, y + 12);
    display.print("Grade: ");
    if (g_data.tvoc_valid) display.println(tvocGrade(g_data.tvoc_ppb));
    else display.println("--");

    display.setCursor(0, y + 24);
    display.print("StaticIAQ: "); display.println(isnan(g_data.static_iaq) ? 0.0 : g_data.static_iaq, 0);

    display.setCursor(0, y + 36);
    display.print("AGS10: "); display.println(g_data.ags10_present ? "OK" : "MISS");
  }
  else {
    display.setCursor(0, y);
    display.print("IP: "); display.println(WiFi.localIP());

    display.setCursor(0, y + 12);
    display.print("Uptime: ");
    display.print(millis() / 1000);
    display.println(" s");

    display.setCursor(0, y + 24);
    display.print("MQTT: ");
    display.println(mqtt.connected() ? "OK" : "DOWN");

    display.setCursor(0, y + 36);
    display.print("Broker: ");
    display.print(MQTT_HOST);
    display.print(":");
    display.println(MQTT_PORT);
  }
}

static void oledRenderPaged() {
  static uint32_t lastFlip = 0;
  static uint8_t page = 0;

  uint32_t now = millis();
  if (now - lastFlip >= 3000) { // 稍微增加翻页间隔
    lastFlip = now;
    page = (page + 1) % 5; // 增加到5页
  }

  display.clearDisplay();
  drawHeader();
  drawPage(page);
  display.display();
}

void setup() {
  Serial.begin(115200);
  delay(500);

  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(I2C_FAST_HZ);

  // RGB LED init
  pinMode(RGB_LED_Pin, OUTPUT);

  // OLED init
  if (!display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR)) {
    Serial.println("OLED init failed (addr maybe 0x3D?)");
  } else {
    display.clearDisplay();
    display.setTextColor(SSD1306_WHITE);
    display.setTextSize(1);
    display.setCursor(0, 0);
    display.println("OLED OK");
    display.display();
    delay(300);
  }

  // Keys: use internal pull-ups, press -> LOW
  pinMode(K1_PIN, INPUT_PULLUP);
  pinMode(K2_PIN, INPUT_PULLUP);
  pinMode(K3_PIN, INPUT_PULLUP);
  pinMode(K4_PIN, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(K1_PIN), onAnyKeyISR, FALLING);
  attachInterrupt(digitalPinToInterrupt(K2_PIN), onAnyKeyISR, FALLING);
  attachInterrupt(digitalPinToInterrupt(K3_PIN), onAnyKeyISR, FALLING);
  attachInterrupt(digitalPinToInterrupt(K4_PIN), onAnyKeyISR, FALLING);

  g_lastUserActionMs = millis();

  wifiConnect();
  ntpInit();
  mqttConnect();
  updateWeather(); // 初始化时获取一次天气

  // BSEC2 subscribe list
  bsecSensor sensorList[] = {
    BSEC_OUTPUT_IAQ,
    BSEC_OUTPUT_STATIC_IAQ,
    BSEC_OUTPUT_CO2_EQUIVALENT,
    BSEC_OUTPUT_BREATH_VOC_EQUIVALENT,
    BSEC_OUTPUT_RAW_TEMPERATURE,
    BSEC_OUTPUT_RAW_HUMIDITY,
    BSEC_OUTPUT_RAW_PRESSURE,
    BSEC_OUTPUT_RAW_GAS
  };

  if (!envSensor.begin(BME68X_I2C_ADDR_LOW, Wire)) {
    checkBsecStatus(envSensor);
  }
  loadBsecState();
  if (!envSensor.updateSubscription(sensorList, ARRAY_LEN(sensorList), SAMPLE_RATE)) {
    checkBsecStatus(envSensor);
  }
  envSensor.attachCallback(newDataCallback);

  g_data.ags10_present = probeAGS10();
  Serial.printf("AGS10 probe @0x%02X => %s\n", AGS10_ADDR, g_data.ags10_present ? "OK" : "FAIL");
  Serial.println("AGS10 warmup >=120s, first valid TVOC will come later");

  Serial.println("BSEC2 + MQTT + OLED + AGS10 + Weather ready");
}

void loop() {
  if (WiFi.status() != WL_CONNECTED) wifiConnect();
  if (!mqtt.connected()) mqttConnect();
  mqtt.loop();

  if (!envSensor.run()) {
    checkBsecStatus(envSensor);
  }

  // Poll AGS10 with manual-specified timing / CRC / RDY handling
  updateAGS10IfDue();

  static uint32_t lastPubMs = 0;
  static uint32_t lastWeatherMs = 0;

  // Any key wakes the screen and resets inactivity timer
  uint32_t now = millis();

  // 每 10 分钟更新一次天气
  if (now - lastWeatherMs >= 600000UL || lastWeatherMs == 0) {
    lastWeatherMs = now;
    updateWeather();
  }

  if (g_keyPressed) {
    g_keyPressed = false;
    g_lastUserActionMs = now;
    oledPower(true);
  }

  if (g_hasNew) {
    g_hasNew = false;
    saveBsecStateIfNeeded(g_data.iaq_acc);
  }

  if (now - lastPubMs >= MQTT_PUB_INTERVAL_MS) {
    lastPubMs = now;

    publishFloat(TOPIC_TEMP, g_data.temp, 2);
    publishFloat(TOPIC_HUM,  g_data.hum, 2);
    publishFloat(TOPIC_PRES, g_data.pres_hpa, 2);
    publishFloat(TOPIC_GAS,  g_data.gas_kohm, 2);
    publishFloat(TOPIC_IAQ,  g_data.iaq, 1);
    if (g_data.tvoc_valid) {
      publishUInt32(TOPIC_TVOC, g_data.tvoc_ppb);
      updateLEDByTVOC(g_data.tvoc_ppb);
    }

    char json[384];
    snprintf(json, sizeof(json),
      "{\"ts_ms\":%lu,"
      "\"temp\":%.2f,\"hum\":%.2f,\"pres\":%.2f,\"gas_kohm\":%.2f,"
      "\"iaq\":%.1f,\"iaq_acc\":%d,\"static_iaq\":%.1f,\"co2_eq\":%.0f,\"bvoc_eq\":%.2f,"
      "\"tvoc_ppb\":%lu}",
      (unsigned long)g_data.ts_ms,
      isnan(g_data.temp) ? 0.0f : g_data.temp,
      isnan(g_data.hum) ? 0.0f : g_data.hum,
      isnan(g_data.pres_hpa) ? 0.0f : g_data.pres_hpa,
      isnan(g_data.gas_kohm) ? 0.0f : g_data.gas_kohm,
      isnan(g_data.iaq) ? 0.0f : g_data.iaq,
      g_data.iaq_acc,
      isnan(g_data.static_iaq) ? 0.0f : g_data.static_iaq,
      isnan(g_data.co2_eq) ? 0.0f : g_data.co2_eq,
      isnan(g_data.bvoc_eq) ? 0.0f : g_data.bvoc_eq,
      (unsigned long)(g_data.tvoc_valid ? g_data.tvoc_ppb : 0UL)
    );
    mqtt.publish(TOPIC_JSON, json, false);
    Serial.println(json);
  }

  if (g_oledOn && (now - g_lastUserActionMs >= OLED_TIMEOUT_MS)) {
    oledPower(false);
  }

  if (g_oledOn) {
    oledRenderPaged();
  }

  delay(10);

  static uint32_t lastNightCheck = 0;
  if (now - lastNightCheck >= 1000) {
    lastNightCheck = now;
    updateLedNightMode();
  }

  handleLED();
}
