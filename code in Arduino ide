#include <WiFi.h>
#include <WebServer.h>
#include <PubSubClient.h>
#include <ModbusMaster.h>
#include <ArduinoJson.h>
#include <math.h>

// =====================================================
// Wi-Fi และ MQTT
// =====================================================
const char* wifi_ssid       = " "; //ชื่อไวไฟ 
const char* wifi_password   = " "; //รหัสไวไฟ
const char* mqtt_broker_ip  = " "; //IP Address ของ MQTT Broker 
const char* mqtt_username   = " ";      // ชื่อ mqtt ที่เราตั้งไว้
const char* mqtt_password   = " "; // รหัส mqtt ที่เราตั้งไว้

WiFiClient wifi_client;
PubSubClient mqtt_client(wifi_client);
WebServer web_server(80);

// =====================================================
// RS485
// =====================================================
#define PIN_RS485_RX 16
#define PIN_RS485_TX 17

ModbusMaster pzem_meter;

// =====================================================
// ตั้งค่าระบบ
// =====================================================
uint8_t meter1_address = 1; // slave id ตั้งตามที่เราตั้งไว้ใน Modbus Poll
uint8_t meter2_address = 2; // slave id ตั้งตามที่เราตั้งไว้ใน Modbus Poll
unsigned long read_interval_ms = 1000;

// =====================================================
// ตัวนับสถานะ (Counters) สำหรับเก็บสถิติ สำเร็จ/ล้มเหลว
// =====================================================
unsigned long meter1_success_count = 0;
unsigned long meter1_fail_count = 0;
unsigned long meter2_success_count = 0;
unsigned long meter2_fail_count = 0;

// =====================================================
// ค่า Threshold สำหรับตรวจจับความผิดปกติ
// =====================================================
float undervoltage_threshold = 200.0;
float overcurrent_threshold = 10.0;

// =====================================================
// Timer
// =====================================================
unsigned long previous_read_time = 0;
unsigned long last_mqtt_reconnect_attempt = 0;

// =====================================================
// ตั้งค่า Wi-Fi
// =====================================================
void setup_wifi() {
  delay(10);
  Serial.println();
  Serial.print("Connecting to WiFi: ");
  Serial.println(wifi_ssid);

  WiFi.begin(wifi_ssid, wifi_password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  Serial.println();
  Serial.println("WiFi Connected!");
  Serial.print("ESP32 IP Address: ");
  Serial.println(WiFi.localIP());
}

// =====================================================
// MQTT Reconnect
// =====================================================
void maintain_mqtt(unsigned long current_time) {
  if (!mqtt_client.connected()) {
    if (current_time - last_mqtt_reconnect_attempt > 5000) {
      last_mqtt_reconnect_attempt = current_time;
      Serial.print("Connecting to MQTT Broker...");

      if (mqtt_client.connect("ESP32_PZEM_Monitor", mqtt_username, mqtt_password)) {
        Serial.println(" Connected!");
      } else {
        Serial.print(" Failed, rc=");
        Serial.println(mqtt_client.state());
      }
    }
  } else {
    mqtt_client.loop();
  }
}

// =====================================================
// ตรวจสอบความผิดปกติ
// =====================================================
String check_abnormal_condition(float voltage, float current) {
  String warning = "";

  if (voltage < undervoltage_threshold) {
    warning += "UNDERVOLTAGE";
    Serial.print("WARNING: Undervoltage detected! V=");
    Serial.println(voltage);
  }

  if (current > overcurrent_threshold) {
    if (warning.length() > 0) {
      warning += ",";
    }
    warning += "OVERCURRENT";
    Serial.print("WARNING: Overload detected! I=");
    Serial.println(current);
  }

  if (warning.length() == 0) {
    warning = "NORMAL";
  }

  return warning;
}

// =====================================================
// ส่ง Alarm ผ่าน MQTT
// =====================================================
void send_alarm(uint8_t meter_id, float voltage, float current, String warning) {
  if (!mqtt_client.connected()) {
    return;
  }

  StaticJsonDocument<256> alarm_doc;
  alarm_doc["meter_id"] = meter_id;
  alarm_doc["voltage"] = voltage;
  alarm_doc["current"] = current;
  alarm_doc["status"] = warning;

  char alarm_payload[256];
  serializeJson(alarm_doc, alarm_payload);

  char alarm_topic[80];
  sprintf(alarm_topic, "home/pzem/%d/alarm", meter_id);
  mqtt_client.publish(alarm_topic, alarm_payload);
  mqtt_client.loop();
}

// =====================================================
// อ่าน PZEM
// =====================================================
void read_and_send_pzem(uint8_t meter_id) {
  pzem_meter.begin(meter_id, Serial2);

  uint8_t read_status;
  uint8_t retries = 3;

  while (retries > 0) {
    // Clear RS485 buffer
    while (Serial2.available()) {
      Serial2.read();
    }
    delay(20);

    // อ่าน Register 0x0000 จำนวน 10 registers
    read_status = pzem_meter.readInputRegisters(0x0000, 10);

    if (read_status == pzem_meter.ku8MBSuccess) {
      break;
    }

    retries--;
    if (retries > 0) {
      delay(50);
    }
  }

  // ===================================================
  // อ่านสำเร็จ
  // ===================================================
  if (read_status == pzem_meter.ku8MBSuccess) {
    
    // บันทึกสถิติสำเร็จ
    if (meter_id == meter1_address) {
      meter1_success_count++;
    } else if (meter_id == meter2_address) {
      meter2_success_count++;
    }

    float meter_voltage = pzem_meter.getResponseBuffer(0x0000) / 10.0;
    
    // แก้ไขวงเล็บ casting ให้ถูกต้อง
    float meter_current = (((uint32_t)pzem_meter.getResponseBuffer(0x0002) << 16) | pzem_meter.getResponseBuffer(0x0001)) / 1000.0;
    float meter_power = (((uint32_t)pzem_meter.getResponseBuffer(0x0004) << 16) | pzem_meter.getResponseBuffer(0x0003)) / 10.0;
    
    // ดึงค่าพลังงานสะสม (Energy) หน่วยเป็น Wh และแปลงเป็น kWh
    float meter_energy_kwh = (((uint32_t)pzem_meter.getResponseBuffer(0x0006) << 16) | pzem_meter.getResponseBuffer(0x0005)) / 1000.0;
    
    float meter_pf = pzem_meter.getResponseBuffer(0x0008) / 100.0;
    float apparent_power = meter_voltage * meter_current;
    
    float reactive_power = 0.0;
    if (apparent_power > meter_power) {
      reactive_power = sqrt((apparent_power * apparent_power) - (meter_power * meter_power));
    }

    String warning = check_abnormal_condition(meter_voltage, meter_current);

    // =================================================
    // Serial Monitor (แสดงจำนวนนับ และค่า kWh)
    // =================================================
    Serial.printf(
      "[Meter: %d] V:%.1f | I:%.3f | P:%.1f | PF:%.2f | S:%.1f | Q:%.1f | E:%.3f kWh | STATUS:%s | OK:%lu | FAIL:%lu\n",
      meter_id, meter_voltage, meter_current, meter_power, meter_pf, apparent_power, reactive_power, meter_energy_kwh, warning.c_str(),
      (meter_id == meter1_address) ? meter1_success_count : meter2_success_count,
      (meter_id == meter1_address) ? meter1_fail_count : meter2_fail_count
    );

    // =================================================
    // ส่งข้อมูล MQTT
    // =================================================
    if (mqtt_client.connected()) {
      StaticJsonDocument<512> doc;

      doc["voltage"] = meter_voltage;
      doc["current"] = meter_current;
      doc["power"] = meter_power;
      doc["power_factor"] = meter_pf;
      doc["apparent_power"] = apparent_power;
      doc["reactive_power"] = reactive_power;
      doc["energy"] = meter_energy_kwh;
      doc["status"] = warning;
      
      // ส่งจำนวนนับขึ้น MQTT
      doc["success_count"] = (meter_id == meter1_address) ? meter1_success_count : meter2_success_count;
      doc["fail_count"] = (meter_id == meter1_address) ? meter1_fail_count : meter2_fail_count;

      char json_payload[512];
      serializeJson(doc, json_payload);

      char mqtt_topic_path[60];
      sprintf(mqtt_topic_path, "home/pzem/%d", meter_id);

      mqtt_client.publish(mqtt_topic_path, json_payload);
      mqtt_client.loop();

      if (warning != "NORMAL") {
        send_alarm(meter_id, meter_voltage, meter_current, warning);
      }
    }
  }
  // ===================================================
  // อ่านไม่สำเร็จ
  // ===================================================
  else {
    // บันทึกสถิติล้มเหลว
    if (meter_id == meter1_address) {
      meter1_fail_count++;
    } else if (meter_id == meter2_address) {
      meter2_fail_count++;
    }

    Serial.printf(
      "[Meter ID: %d] Read Failed! Error: 0x%02X | OK:%lu | FAIL:%lu\n",
      meter_id, read_status,
      (meter_id == meter1_address) ? meter1_success_count : meter2_success_count,
      (meter_id == meter1_address) ? meter1_fail_count : meter2_fail_count
    );
  }
}

// =====================================================
// Web Page
// =====================================================
void handle_root() {
  String html = R"rawliteral(
<!DOCTYPE html>
<html>
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width, initial-scale=1.0">
<title>ESP32 PZEM Setting</title>
<style>
body { font-family: Arial; background: #f2f2f2; padding: 20px; }
.container { max-width: 500px; margin: auto; background: white; padding: 25px; border-radius: 15px; box-shadow: 0 2px 10px #aaa; }
h2 { text-align: center; }
input { width: 100%; padding: 10px; margin-top: 5px; margin-bottom: 15px; box-sizing: border-box; }
button { width: 100%; padding: 12px; background: #2196F3; color: white; border: none; border-radius: 8px; font-size: 16px; }
button:hover { background: #1976D2; }
.warning { background: #fff3cd; padding: 15px; border-radius: 8px; line-height: 1.6; }
.stats { font-weight: bold; color: #333; margin-top: 10px; border-top: 1px solid #ddd; padding-top: 10px; }
.ok-text { color: #2e7d32; }
.fail-text { color: #c62828; }
</style>
</head>

<body>
<div class="container">
<h2>ESP32 PZEM Monitor</h2>
<form action="/save" method="GET">
<label>Meter 1 Address</label>
<input type="number" name="addr1" min="1" max="247" value="%ADDR1%">

<label>Meter 2 Address</label>
<input type="number" name="addr2" min="1" max="247" value="%ADDR2%"
