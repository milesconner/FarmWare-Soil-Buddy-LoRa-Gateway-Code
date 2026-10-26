
#include <Arduino.h>

//libraries for LoRa comms
#include <SPI.h>
#include <LoRa.h>

//libraries for WiFi and MQTT server
#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <PubSubClient.h>
#include <time.h>

//LoRa transceiver pins
#define IRQ 27
#define CS 32
#define RST 33

//LoRa defines
#define FREQUENCY_BAND 915E6
#define SYNC_WORD 0x12
#define SPREADING_FACTOR 10
#define BANDWIDTH 125E3
#define CODING_RATE 8
#define TRANSMIT_POWER 20
#define NETWORK_CODE 0x3D8A
#define GATEWAY_ID 0x00
#define SOIL_BUDDY_ID 0x01

//WiFi, MQTT defines
#define WIFI_TIMEOUT 15000000
#define MQTT_TIMEOUT 15000000

//system configuration defaults
#define NEXT_BIG_CYCLE_DEFAULT 1
#define SAMPLING_INTERVAL_DEFAULT 500
#define NUMBER_OF_STABILIZING_SAMPLES_DEFAULT 5
#define NUMBER_OF_ACTUAL_SAMPLES_DEFAULT 10

//LoRa packet header struct
struct __attribute__((packed)) Header {
  uint16_t network_code;
  uint8_t recipient_id;
  uint8_t sender_id;
};

//LoRa packet config payload struct
//Gateway -> Soil Buddy
struct __attribute__((packed)) ConfigPayload {
  uint16_t next_big_cycle = NEXT_BIG_CYCLE_DEFAULT;
  uint16_t sampling_interval = SAMPLING_INTERVAL_DEFAULT;
  uint8_t number_of_stabilizing_samples = NUMBER_OF_STABILIZING_SAMPLES_DEFAULT;
  uint8_t number_of_actual_samples = NUMBER_OF_ACTUAL_SAMPLES_DEFAULT;
};

//LoRa packet data payload struct
//Soil Buddy -> Gateway
//uses fixed-point arithmetic scaling to reduce the size of data
struct __attribute__((packed)) DataPayload {
  int16_t probe_temp;

  uint16_t stemma_moisture;

  uint8_t bme_acc;
  uint16_t bme_iaq;
  uint16_t bme_co2;
  int16_t bme_temp;
  uint16_t bme_humid;
};

//volatile flag that gets set every time a new LoRa transmission is received
volatile bool pending_lora_packet = false;
//size of the pending packet
int pending_packet_size = 0;

//WiFi credentials (only one should be uncommented)

//Miles' apartment
//const char* wifi_ssid = "Tremors 7";
//const char* wifi_password = "gloverforn64";

//OSU
const char* wifi_ssid = "Registered4OSU";
const char* wifi_password = "ASYeVEP6pp3aKTfV";

//MQTT server credentials
const char* mqtt_server_url = "9ccaa78e62aa4522ab541b0ca1426b5c.s1.eu.hivemq.cloud";
const int mqtt_server_port = 8883; //use 8884 in the web socket
const char* mqtt_server_username = "SoilBuddyPrototype";
const char* mqtt_server_password = "FarmWareTopSecretPassword12";

//WiFi and MQTT client objects
WiFiClientSecure wifi_client;
PubSubClient mqtt_client(wifi_client);

//device name
const char* device_name = "Soil Buddy LoRa Gateway";

//MQTT topics
const char* bme_acc_topic = "soil-buddy-prototype/bme-acc";
const char* bme_iaq_topic = "soil-buddy-prototype/bme-iaq";
const char* bme_co2_topic = "soil-buddy-prototype/bme-co2";
const char* bme_temp_topic = "soil-buddy-prototype/bme-temp";
const char* bme_humid_topic = "soil-buddy-prototype/bme-humid";
const char* bme_vpd_topic = "soil-buddy-prototype/bme-vpd";
const char* probe_temp_topic = "soil-buddy-prototype/probe-temp";
const char* stemma_moisture_topic = "soil-buddy-prototype/stemma-moisture";
const char* data_time_topic = "soil-buddy-prototype/data-time";
const char* next_big_cycle_topic = "soil-buddy-prototype/next-big-cycle";
const char* sampling_interval_topic = "soil-buddy-prototype/sampling-interval";
const char* stabilizing_samples_topic = "soil-buddy-prototype/stabilizing-samples";
const char* actual_samples_topic = "soil-buddy-prototype/actual-samples";

//holds the Soil Buddy's system configuration
ConfigPayload soil_buddy_config;

//parameters for getting the current time of day from the internet
const char* ntp_server = "pool.ntp.org"; //NTP server to get time of day from
const long gmt_offset_sec = 3600 * -5; //eastern time
const int daylight_offset_sec = 3600; //3600 during daylight savings, 0 otherwise

//holds local time of day in 12 hour format, call update_local_time() before accessing
char local_time[50];

//LoRa functions
bool init_lora(); //sets up LoRa, returns whether it succeeded
void lora_packet_received(int packet_size); //gets called every time a new LoRa transmission is received
void process_lora_packet(); //function to process received LoRa packets

//WiFi and MQTT server functions
bool connect_to_wifi(); //connects to WiFi, returns whether it succeeded
bool connect_to_mqtt_server(); //connects to MQTT server, returns whether it succeeded
void mqtt_callback_function(char* topic, uint8_t* message, unsigned int length); //gets called every time a new message is published to a subscribed MQTT topic

//utility functions
float calculate_VPD(float temperature, float humidity); //calculates VPD from BME680 temperature and humidity
void print_data(DataPayload data); //prints Soil Buddy data
void print_config(); //prints Soil Buddy config
void update_local_time(); //updates the local time of day string (local_time)

void setup() {
  //begin the serial monitor
  Serial.begin(115200);
  Serial.printf("\n\nSoil Buddy LoRa Gateway is awake...\n");

  //connect to WiFi
  connect_to_wifi();

  //sync local time of day from NTP server
  Serial.printf("Getting local time of day from NTP server...\n");
  configTime(gmt_offset_sec, daylight_offset_sec, ntp_server);

  //connect to MQTT server
  connect_to_mqtt_server();

  //initialize LoRa
  init_lora();
}

void loop() {
  //loop the MQTT client
  mqtt_client.loop();

  //if a LoRa transmission came in, process it and clear the flag
  if(pending_lora_packet) {
    process_lora_packet();

    pending_packet_size = 0;
    pending_lora_packet = false;
  }  
}

bool init_lora() {
  LoRa.setPins(CS, RST, IRQ);

  if(! LoRa.begin(FREQUENCY_BAND)) {
    Serial.printf("LoRa failed to initialize!\n");
    return false;
  }

  LoRa.setSyncWord(SYNC_WORD);
  LoRa.setSpreadingFactor(SPREADING_FACTOR);       // Choices: 7 to 12 (Higher = more range, slower data)
  LoRa.setSignalBandwidth(BANDWIDTH);              // 125E3 (125 kHz) or 62.5E3 for narrower/deeper reach
  LoRa.setCodingRate4(CODING_RATE);                // 4/5, 4/6, 4/7, or 4/8 (4/8 gives max error correction)
  LoRa.setTxPower(TRANSMIT_POWER);                 // Max power (adjust based on your module's legal/hardware limits, e.g., 17-20 dBm)
  LoRa.enableCrc();                                // Enable cyclic redundancy check for packet integrity

  LoRa.onReceive(lora_packet_received);
  LoRa.receive();

  Serial.printf("LoRa online...\n");

  return true;
}

void lora_packet_received(int packet_size) {
  pending_packet_size = packet_size;
  pending_lora_packet = true;
}

void process_lora_packet() {
  //packet size mismatch, throw it away
  if(pending_packet_size < (sizeof(Header) + sizeof(DataPayload))) {
    return;
  }

  //read header
  Header header;
  LoRa.readBytes((uint8_t*) &header, sizeof(Header));

  //network code doesn't match, this packet isn't from one of our devices, throw it away
  if(header.network_code != NETWORK_CODE) {
    return;
  }

  //we aren't the intended recipient, packet was meant for another device, throw it away
  if(header.recipient_id != GATEWAY_ID) {
    return;
  }

  //capture time that data came in
  update_local_time();

  //read data payload
  DataPayload data_payload;
  LoRa.readBytes((uint8_t*) &data_payload, sizeof(DataPayload));

  Serial.printf("Received data from Soil Buddy!\n");
  print_data(data_payload);

  //build header
  header.network_code = NETWORK_CODE;
  header.recipient_id = SOIL_BUDDY_ID;
  header.sender_id = GATEWAY_ID;

  //send header + Soil Buddy config
  LoRa.beginPacket();
  LoRa.write((uint8_t*) &header, sizeof(Header));
  LoRa.write((uint8_t*) &soil_buddy_config, sizeof(ConfigPayload));
  LoRa.endPacket();

  Serial.printf("Sent configuration acknowledgement to Soil Buddy!\n");
  print_config();

  //convert data to strings
  char bme_acc_message[16];
  char bme_iaq_message[16];
  char bme_co2_message[16];
  char bme_temp_message[16];
  char bme_humid_message[16];
  char bme_vpd_message[16];
  char probe_temp_message[16];
  char stemma_moisture_message[16];

  itoa(data_payload.bme_acc, bme_acc_message, 10);
  dtostrf(((float) data_payload.bme_iaq / 100.0F), 1, 2, bme_iaq_message);
  dtostrf(((float) data_payload.bme_co2 / 10.0F), 1, 1, bme_co2_message);
  dtostrf(((float) data_payload.bme_temp / 100.0F), 1, 2, bme_temp_message);
  dtostrf(((float) data_payload.bme_humid / 100.0F), 1, 2, bme_humid_message);
  dtostrf(calculate_VPD(((float) data_payload.bme_temp / 100.0F), ((float) data_payload.bme_humid / 100.0F)), 1, 2, bme_vpd_message);
  dtostrf(((float) data_payload.probe_temp / 100.0F), 1, 2, probe_temp_message);
  itoa(data_payload.stemma_moisture, stemma_moisture_message, 10);

  //publish data to the MQTT server
  mqtt_client.publish(bme_acc_topic, bme_acc_message, true);
  mqtt_client.publish(bme_iaq_topic, bme_iaq_message, true);
  mqtt_client.publish(bme_co2_topic, bme_co2_message, true);
  mqtt_client.publish(bme_temp_topic, bme_temp_message, true);
  mqtt_client.publish(bme_humid_topic, bme_humid_message, true);
  mqtt_client.publish(bme_vpd_topic, bme_vpd_message, true);
  mqtt_client.publish(probe_temp_topic, probe_temp_message, true);
  mqtt_client.publish(stemma_moisture_topic, stemma_moisture_message, true);
  mqtt_client.publish(data_time_topic, local_time, true);

  Serial.printf("Published data to the MQTT server!\n");

  //start listening for Soil Buddy data
  LoRa.onReceive(lora_packet_received);
  LoRa.receive();
}

bool connect_to_wifi() {
  int64_t start_time = esp_timer_get_time();

  wifi_client.setInsecure();

  Serial.printf("Connecting to WiFi network...");
  
  WiFi.begin(wifi_ssid, wifi_password);

  while(WiFi.status() != WL_CONNECTED) {
    if(esp_timer_get_time() - start_time > WIFI_TIMEOUT) {
      Serial.printf("\nFailed to connect to WiFi network!\n");
      return false;
    }

    delay(500);
    Serial.printf(".");
  }

  Serial.printf("\nConnected to WiFi network!\n");

  return true;
}

bool connect_to_mqtt_server() {
  int64_t start_time = esp_timer_get_time();

  mqtt_client.setServer(mqtt_server_url, mqtt_server_port);
  mqtt_client.setCallback(mqtt_callback_function);
  
  Serial.printf("Connecting to MQTT server...");

  while(! mqtt_client.connected()) {
    if(esp_timer_get_time() - start_time > MQTT_TIMEOUT) {
      Serial.printf("\nFailed to connect to MQTT server!\n");
      return false;
    }

    delay(500);
    mqtt_client.connect(device_name, mqtt_server_username, mqtt_server_password);
    Serial.printf(".");
  }

  Serial.printf("\nConnected to MQTT server!\n");

  mqtt_client.subscribe(next_big_cycle_topic);
  mqtt_client.subscribe(sampling_interval_topic);
  mqtt_client.subscribe(stabilizing_samples_topic);
  mqtt_client.subscribe(actual_samples_topic);

  return true;
}

void mqtt_callback_function(char* topic, uint8_t* message, unsigned int length) {
  String incoming_message = "";

  for(int i = 0; i < length; i++) {
    incoming_message += (char) message[i];
  }

  if(String(topic) == next_big_cycle_topic) {
    soil_buddy_config.next_big_cycle = incoming_message.toInt();
    Serial.printf("Received 'Next Big Cycle' configuration update from MQTT server...\n");
  }

  if(String(topic) == sampling_interval_topic) {
    soil_buddy_config.sampling_interval = incoming_message.toInt();
    Serial.printf("Received 'Sampling Interval' configuration update from MQTT server...\n");
  }

  if(String(topic) == stabilizing_samples_topic) {
    soil_buddy_config.number_of_stabilizing_samples = incoming_message.toInt();
    Serial.printf("Received 'Number of Stabilizing Samples' configuration update from MQTT server...\n");
  }

  if(String(topic) == actual_samples_topic) {
    soil_buddy_config.number_of_actual_samples = incoming_message.toInt();
    Serial.printf("Received 'Number of Actual Samples' configuration update from MQTT server...\n");
  }
}

float calculate_VPD(float temperature, float humidity) {
  float SVP = 0.61078 * exp((17.27 * temperature) / (temperature + 237.3)); //Saturation Vapor Pressure  
  float AVP = SVP * (humidity / 100.0); //Actual Vapor Pressure
  float VPD = SVP - AVP; //Vapor Pressure Deficit
    
  return (VPD * 1000.0); //Convert to Pa
}

void print_data(DataPayload data) {
  Serial.printf("-------------------------- SOIL BUDDY DATA -------------------------\n");
  Serial.printf("Measurement Time: ");
  Serial.printf(local_time);
  Serial.printf("\n");
  Serial.printf("BME680 Accuracy Score:              %u\n", data.bme_acc);
  Serial.printf("BME680 IAQ:                         %.2f\n", ((float) data.bme_iaq / 100.0F));
  Serial.printf("BME680 CO2:                         %.1f PPM\n", ((float) data.bme_co2 / 10.0F));
  Serial.printf("BME680 Compensated Temperature:     %.2f C\n", ((float) data.bme_temp / 100.0F));
  Serial.printf("BME680 Compensated Humidity:        %.2f %%\n", ((float) data.bme_humid / 100.0F));
  Serial.printf("BME680 VPD:                         %.2f PA\n", calculate_VPD(((float) data.bme_temp / 100.0F), ((float) data.bme_humid / 100.0F)));
  Serial.printf("DS18B20 Probe Temperature:          %.2f C\n", ((float) data.probe_temp / 100.0F));
  Serial.printf("STEMMA Soil Capacitance:            %hu\n", data.stemma_moisture);
  Serial.printf("--------------------------------------------------------------------\n");
}

void print_config() {
  Serial.printf("------------------------- SOIL BUDDY CONFIG ------------------------\n");
  Serial.printf("Next Big Cycle:                     wake cycle %hu\n", soil_buddy_config.next_big_cycle);
  Serial.printf("Sampling Interval:                  %hu milliseconds\n", soil_buddy_config.sampling_interval);
  Serial.printf("Number of Stabilizing Samples:      %u\n", soil_buddy_config.number_of_stabilizing_samples);
  Serial.printf("Number of Actual Samples:           %u\n", soil_buddy_config.number_of_actual_samples);
  Serial.printf("--------------------------------------------------------------------\n");
}

void update_local_time() {
  struct tm time_info;

  if(! getLocalTime(&time_info)) {
    Serial.printf("Failed to update local time!\n");
    return;
  }

  Serial.printf("Updated local time of day...\n");

  strftime(local_time, sizeof(local_time), "%I:%M:%S %p", &time_info);
}
