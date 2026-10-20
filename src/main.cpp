
#include <Arduino.h>

//libraries for LoRa comms
#include <SPI.h>
#include <LoRa.h>

//libraries for WiFi and MQTT server
#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <PubSubClient.h>

//LoRa transceiver pins
#define IRQ 27
#define CS 32
#define RST 33

//LoRa defines
#define FREQUENCY_BAND 915E6
#define SYNC_WORD 0x12
#define NETWORK_CODE 0x3D8A
#define GATEWAY_ID 0x00
#define SOIL_BUDDY_ID 0x01
#define LORA_TIMEOUT 5000000
#define MAX_ATTEMPTS 5

//WiFi, MQTT defines
#define WIFI_TIMEOUT 10000000
#define MQTT_TIMEOUT 10000000

//LoRa packet header struct
struct Header {
  uint16_t network_code;
  uint8_t recipient_id;
  uint8_t sender_id;
};

//LoRa packet config payload struct
//Gateway -> Soil Buddy
struct ConfigPayload {
  uint16_t next_big_cycle;
  uint16_t sampling_interval;
  uint8_t number_of_stabilizing_samples;
  uint8_t number_of_actual_samples;
};

//LoRa packet data payload struct
//Soil Buddy -> Gateway
//uses fixed-point arithmetic scaling to reduce the size of data
struct DataPayload {
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

//MQTT topic for finding sleep configuration
const char* sleep_config_topic = "soil-buddy-prototype/sleep-config";

//MQTT topic for publishing temperature data
const char* temperature_data_topic = "soil-buddy-prototype/temperature-data";

//MQTT topic for publishing moisture data
const char* moisture_data_topic = "soil-buddy-prototype/moisture-data";

//MQTT topic for publishing air quality data
const char* air_quality_data_topic = "soil-buddy-prototype/air-quality-data";

//LoRa functions
bool init_lora(); //sets up LoRa, returns whether it succeeded
void lora_packet_received(int packet_size); //gets called every time a new LoRa transmission is received
void process_lora_packet(); //function to process received LoRa packets

//WiFi and MQTT server functions
bool connect_to_wifi(); //connects to WiFi, returns whether it succeeded
bool connect_to_mqtt_server(); //connects to MQTT server, returns whether it succeeded
void mqtt_callback_function(char* topic, uint8_t* message, unsigned int length); //gets called every time a new message is published to a subscribed MQTT topic

//utility functions
float calculate_VPD(float temperature, float humidity);
void print_data(DataPayload data);

void setup() {
  //begin the serial monitor
  Serial.begin(115200);
  Serial.printf("\n\nSoil Buddy LoRa Gateway is awake...\n");

  //connect to WiFi
  connect_to_wifi();

  //connect to MQTT server
  connect_to_mqtt_server();

  //initialize LoRa
  init_lora();

  //give the MQTT client time to receive the sleep config message
  //uint64_t start_time = esp_timer_get_time();

  //while(esp_timer_get_time() - start_time < 5000000) {
    //mqtt_client.loop();
  //}

  //read sensors
  //float temp = read_temperature_sensor();
  //uint16_t cap = read_moisture_sensor();
  //uint32_t air = read_air_quality_sensor();

  //convert sensor data to strings
  //char temp_message[10];
  //char cap_message[10];
  //char air_message[10];

  //dtostrf(temp, 1, 2, temp_message);
  //itoa(cap, cap_message, 10);
  //itoa(air, air_message, 10);

  //publish sensor data to the MQTT server
  //mqtt_client.publish(temperature_data_topic, temp_message);
  //mqtt_client.publish(moisture_data_topic, cap_message);
  //mqtt_client.publish(air_quality_data_topic, air_message);

  //Serial.printf("\nPublished data to MQTT server.\n\nGoing into deep sleep for %d minutes.\nSee you then!\n\n", sleep_period);
}

void loop() {
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

  //read data payload
  DataPayload data_payload;
  LoRa.readBytes((uint8_t*) &data_payload, sizeof(DataPayload));

  Serial.printf("Received data from Soil Buddy!\n");
  print_data(data_payload);

  //build the header
  Header response_header;

  response_header.network_code = NETWORK_CODE;
  response_header.recipient_id = SOIL_BUDDY_ID;
  response_header.sender_id = GATEWAY_ID;

  //build configuration payload
  ConfigPayload config_payload;

  config_payload.next_big_cycle = 2;
  config_payload.number_of_actual_samples = 20;
  config_payload.number_of_stabilizing_samples = 3;
  config_payload.sampling_interval = 300;

  //send header + config
  LoRa.beginPacket();
  LoRa.write((uint8_t*) &response_header, sizeof(Header));
  LoRa.write((uint8_t*) &config_payload, sizeof(ConfigPayload));
  LoRa.endPacket();

  Serial.printf("Sent configuration acknowledgement to Soil Buddy!\n");

  //start listening for Gateway response
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

  //mqtt_client.subscribe(sleep_config_topic);

  return true;
}

void mqtt_callback_function(char* topic, uint8_t* message, unsigned int length) {
  String incoming_message = "";

  for(int i = 0; i < length; i++) {
    incoming_message += (char) message[i];
  }

  //if(String(topic) == sleep_config_topic) {
    //sleep_period = incoming_message.toInt();
  //}
}

float calculate_VPD(float temperature, float humidity) {
  float SVP = 0.61078 * exp((17.27 * temperature) / (temperature + 237.3)); //Saturation Vapor Pressure  
  float AVP = SVP * (humidity / 100.0); //Actual Vapor Pressure
  float VPD = SVP - AVP; //Vapor Pressure Deficit
    
  return (VPD * 1000.0); //Convert to Pa
}

void print_data(DataPayload data) {
  Serial.printf("-------------------------- SOIL BUDDY DATA -------------------------\n");
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
