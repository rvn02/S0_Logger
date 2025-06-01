#include <Arduino.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include <driver/pcnt.h>
#include <ModbusIP_ESP8266.h>
#include "config.h"
#include <esp_task_wdt.h>

//10 seconds WDT
#define WDT_TIMEOUT 10

const float impPerkWh = IMPULSES_PER_KWH; // Beispielwert: 1000 Impulse pro kWh
const int period_s = 5;

const int power_mb_register = 1000;
const int register_type = 1;

const char* ssid = WIFI_SSID;
const char* password = WIFI_PASSWORD;
const char* mqtt_server = MQTT_SERVER;
const int mqtt_port = MQTT_PORT;

WiFiClient espClient;
PubSubClient client(espClient);

ModbusIP mb;

// PCNT Konfiguration
#define PCNT_INPUT_SIG_IO  4  // Pulse Input GPIO
#define PCNT_CTRL_GPIO    5  // Control GPIO (optional, für Richtungskontrolle)
#define PCNT_FILTER_VALUE 1020 // Filterwert in APB Taktzyklen

pcnt_config_t pcnt_config = {
    // Set PCNT input signal and control GPIOs
    .pulse_gpio_num = PCNT_INPUT_SIG_IO,
    .ctrl_gpio_num = PCNT_CTRL_GPIO,
    .lctrl_mode = PCNT_MODE_KEEP, // Control Signal irrelevant
    .hctrl_mode = PCNT_MODE_KEEP, // Control Signal irrelevant
    .pos_mode = PCNT_COUNT_INC,   // Zählen von positiven Flanken
    .neg_mode = PCNT_COUNT_DIS,   // Ignoriere negative Flanken
    .counter_h_lim = 10000,
    .counter_l_lim = -1,
    .unit = PCNT_UNIT_0,
    .channel = PCNT_CHANNEL_0,
};

void reconnectMQTT(void);
void reconnectWiFi(void);
float convertImpulsesToKW(int impulses, long millis);
float calculatePowerInWatts(int counts, unsigned long millis);

void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);
  Serial.begin(115200);
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.println("Connecting to WiFi..");
  }
  
  client.setServer(mqtt_server, mqtt_port);

  // PCNT Konfiguration initialisieren
  pcnt_unit_config(&pcnt_config);

  // Filter konfigurieren und aktivieren
  pcnt_set_filter_value(PCNT_UNIT_0, PCNT_FILTER_VALUE);
  pcnt_filter_enable(PCNT_UNIT_0);

  pcnt_counter_pause(PCNT_UNIT_0);
  pcnt_counter_clear(PCNT_UNIT_0);
  pcnt_counter_resume(PCNT_UNIT_0);

  // setup modbus stuff
  mb.server();		//Start Modbus IP
  mb.addHreg(power_mb_register, 2);

  digitalWrite(LED_BUILTIN, LOW);

  esp_task_wdt_init(WDT_TIMEOUT, true); //enable panic so ESP32 restarts
  esp_task_wdt_add(NULL); //add current thread to WDT watch
}

void loop() {
  if (WiFi.status() != WL_CONNECTED) {
    digitalWrite(LED_BUILTIN, HIGH);
    reconnectWiFi();
    digitalWrite(LED_BUILTIN, LOW);
  }
  if (!client.connected()) {
    digitalWrite(LED_BUILTIN, HIGH);
    reconnectMQTT();
    digitalWrite(LED_BUILTIN, LOW);
  }
  client.loop();
  mb.task();

  esp_task_wdt_reset();

  static long lastMsg = 0;
  long now = millis();
  if (now - lastMsg > (period_s * 1000)) {
    int16_t count = 0;
    pcnt_get_counter_value(PCNT_UNIT_0, &count);
    // PCNT zurücksetzen
    pcnt_counter_clear(PCNT_UNIT_0);

    Serial.print("Count: ");
    Serial.println(count);

    float kW = calculatePowerInWatts(count, now - lastMsg); // Angenommen, 2 Sekunden = 0.0005556 Stunden
    Serial.print("Leistung: ");
    Serial.print(kW);
    Serial.println(" kW");

    // MQTT-Nachricht senden
    char msg[50];
    snprintf(msg, 50, "%.2f", kW);
    client.publish("/energie/pv/produktion", msg);

    // set mb register
    int W = kW;
    mb.Hreg(power_mb_register, W);

    lastMsg = now;
  }
}

void reconnectWiFi() {
  while (WiFi.status() != WL_CONNECTED) {
    WiFi.disconnect();
    WiFi.reconnect();
    delay(1000); // Kurze Pause, um dem Modul Zeit zu geben, sich zu verbinden
  }
}

void reconnectMQTT() {
  // Loop until we're reconnected

  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    // Attempt to connect
    if (client.connect("ESP32Client")) {
      Serial.println("connected");
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 2 seconds");
      // Wait 5 seconds before retrying
      delay(2000);
    }
  }
}

float convertImpulsesToKW(int impulses, long millis) {
    float kWh = impulses / impPerkWh; // Umrechnung von Impulsen in kWh
    float W = kWh * 3600000 / millis; // Umrechnung von kWh in kW basierend auf der Zeitdauer
    return W;
}

float calculatePowerInWatts(int counts, unsigned long millis) {
    //const float impPerKWh = 10000.0; // Impulse pro kWh
    // Umrechnung der vergangenen Zeit von Millisekunden in Stunden
    float hours = millis / 3600000.0;
    
    // Berechnung der Energie in kWh
    float kWh = counts / impPerkWh;
    
    // Berechnung der Leistung in kW und Umwandlung in Watt
    float watts = (kWh / hours) * 1000; // Multipliziere mit 1000, um kW in W umzurechnen
    
    return watts;
}
