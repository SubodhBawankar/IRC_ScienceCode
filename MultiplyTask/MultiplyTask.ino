#include <Arduino.h>
#include <SHT1x-ESP.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <DHT.h>
#include <DHT_U.h>
#include <Adafruit_BMP085.h>
#include "WiFi.h"
#include <esp_now.h>
#include <ESP32Servo.h>
#include <Adafruit_MLX90614.h>


// I2C
#define dataPin  21 // blue 
#define clockPin 22 // yellow
// for SHT10 Red VCC; Green Gnd

#define pin_mq_1 34
#define pin_mq_2 35

#define DHTPIN_1 4
#define DHTPIN_2 0
#define DHTTYPE DHT11

// Johnson Motor Pins
#define PWMR 15 // Clockwise
#define PWML 2 // Anticlockwise
#define JOHNSON_DELAY 20000 // Delay in milisec

// Servo Pins 
#define ServoPin1 18
#define ServoPin2 5

// Prestaltic Pumps Pins
// total 4 Pumps 
#define Pump_INA1 32 
#define Pump_INA2 33
#define Pump_INB1 25 
#define Pump_INB2 26
#define Pump_INC1 27 
#define Pump_INC2 14
#define Pump_IND1 12 
#define Pump_IND2 13 

// Wormgear Motors
#define Worm_PWMR_1 16
#define Worm_PWML_1 17
#define Worm_PWMR_2 23
#define Worm_PWML_2 19
/* Note:
88rev -- 60 sec
timedelay = angle_x * (1180.15/720)
*/

struct science{
  float mq1_ppm;
  float mq2_ppm;
  float dht1_temp;
  float dht1_humidity;
  float dht2_temp;
  float dht2_humidity;
  float bmp_pressure;
  float sht_temp;
  float sht_humidity;
  float mlx_objtemp;
};
struct science data_t = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

struct commands{
  int johnson; // 0->Stationary; 1->UP; 2->DOWN
  int wormgear_pwm_1; // 0->Stationary; 1->UP; 2->DOWN
  int wormgear_pwm_2; // 0->Stationary; 1->UP; 2->DOWN
  int servo_shake; // 0->OFF; 1->ON 
  int pumps_1; // false->OFF; true->ON
  int pumps_2; // false->OFF; true->ON
};
struct commands data_COMMAND = {0, 0, 0, 0, 0, 0};

DHT_Unified dht_1(DHTPIN_1, DHTTYPE);
DHT_Unified dht_2(DHTPIN_2, DHTTYPE);
Adafruit_BMP085 bmp;
Adafruit_MLX90614 mlx = Adafruit_MLX90614();
SHT1x sht1x(dataPin, clockPin);
sensors_event_t event_1;
sensors_event_t event_2;
Servo servo1;
Servo servo2;

// Task Handlers
TaskHandle_t Sensors;
TaskHandle_t WIFI;
TaskHandle_t Motor;

// MAC ADDRESS of JETSONS`ESP32:- 08:B6:1F:3B:0C:78
uint8_t broadcastAddress[] = {0x08, 0xB6, 0x1F, 0x3B, 0x0C, 0x78};
esp_now_peer_info_t peerInfo;

void Sensor_Task(void * parameter){
  while(true){
  data_t.mq1_ppm = (float)analogRead(pin_mq_1);
  data_t.mq2_ppm = (float)analogRead(pin_mq_2);

  dht_1.temperature().getEvent(&event_1);
  data_t.dht1_temp = (float)event_1.temperature;
  dht_1.humidity().getEvent(&event_1);
  data_t.dht1_humidity = (float)event_1.relative_humidity;
  dht_2.temperature().getEvent(&event_2);
  data_t.dht2_temp = (float)event_2.temperature;
  dht_2.humidity().getEvent(&event_2);
  data_t.dht2_humidity = (float)event_2.relative_humidity;
  if(isnan(data_t.dht1_temp) || isnan(data_t.dht1_humidity) || isnan(data_t.dht2_temp) || isnan(data_t.dht2_humidity)){
    data_t.dht1_humidity = 0;
    data_t.dht1_temp = 0;
    data_t.dht2_humidity = 0;
    data_t.dht2_temp = 0;
  }
  if(data_t.bmp_pressure != NULL){
    data_t.bmp_pressure = (float)bmp.readPressure();
  }
  if(data_t.mlx_objtemp != NULL){
    data_t.mlx_objtemp = (float)mlx.readObjectTempC();
  }
  
  data_t.sht_temp = sht1x.readTemperatureC();
  data_t.sht_humidity = sht1x.readHumidity();
  
  // // Printing Sensors Values 
  Serial.print("MQ_1 PPM: "); Serial.println(data_t.mq1_ppm);
  Serial.print("MQ_2 PPM: "); Serial.println(data_t.mq2_ppm);
  Serial.print("DHT_1 Temp: "); Serial.println(data_t.dht1_temp);
  Serial.print("DHT_1 Humidity: "); Serial.println(data_t.dht1_humidity);
  Serial.print("DHT_2 Temp: "); Serial.println(data_t.dht1_temp);
  Serial.print("DHT_2 Humidity: "); Serial.println(data_t.dht1_humidity);
  // Serial.print("BMP Pressure: "); Serial.println(data_t.bmp_pressure);
  // Serial.print("MLX Object Temp: "); Serial.println(data_t.mlx_objtemp);
  // Serial.print("SHT Temp: "); Serial.println(data_t.sht_temp);
  // Serial.print("SHT Humidity: "); Serial.println(data_t.sht_humidity);
  // delay(1000);
  delay(500);
  }
}

// Callback when data is sent
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  // Serial.print("\r\nLast Packet:\t");
  // Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
}

// callback function that will be executed when data is received
void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) {
  memcpy(&data_COMMAND, incomingData, sizeof(data_COMMAND));
  Serial.print("Johnson \t"); Serial.println(data_COMMAND.johnson);
  Serial.print("Worm1\t"); Serial.println(data_COMMAND.wormgear_pwm_1);
  Serial.print("Worm2\t"); Serial.println(data_COMMAND.wormgear_pwm_2);
  Serial.print("Servo \t"); Serial.println(data_COMMAND.servo_shake);
  Serial.print("Pump1\t"); Serial.println(data_COMMAND.pumps_1);
  Serial.print("Pump2\t"); Serial.println(data_COMMAND.pumps_2);
  // motor_control();
}

void pumps_control(){
  int val1, val2;
  if(data_COMMAND.pumps_1){
    val1 = 60; // Low speed
  }
  else{
    val1 = 0; // stoping 
  }
  if(data_COMMAND.pumps_2){
    val2 = 60; // Low speed
  }
  else{
    val2 = 0; // stopping 
  }

  analogWrite(Pump_INA1, val1);
  analogWrite(Pump_INA2, 0);
  analogWrite(Pump_INB1, val1);
  analogWrite(Pump_INB2, 0);
  analogWrite(Pump_INC1, val2);
  analogWrite(Pump_INC2, 0);
  analogWrite(Pump_IND1, val2); 
  analogWrite(Pump_IND2, 0);
}

void WormControl_1(){
  if(data_COMMAND.wormgear_pwm_1 == 1){
    analogWrite(Worm_PWMR_1, 90);
    analogWrite(Worm_PWML_1, 0);
  }
  else if(data_COMMAND.wormgear_pwm_1 == 2){
    analogWrite(Worm_PWMR_1, 0);
    analogWrite(Worm_PWML_1, 90);
  }
  else{
    analogWrite(Worm_PWMR_1, 0);
    analogWrite(Worm_PWML_1, 0);
  }
}
void WormControl_2(){
  if(data_COMMAND.wormgear_pwm_2 == 1){
    analogWrite(Worm_PWMR_2, 90);
    analogWrite(Worm_PWML_2, 0);
  }
  else if(data_COMMAND.wormgear_pwm_2 == 2){
    analogWrite(Worm_PWMR_2, 0);
    analogWrite(Worm_PWML_2, 90);
  }
  else{
    analogWrite(Worm_PWMR_2, 0);
    analogWrite(Worm_PWML_2, 0);
  }
}

void Motor_Task(void * parameter){
  while(true){

    // ADD SERVO FUNCTION HERE
    if(data_COMMAND.servo_shake == 1){
      servo1.write(0);
      servo2.write(180);
    }
    else{
      servo1.write(180);
      servo2.write(0);
    }

    // Checking Command for Johnson Motor
    if(data_COMMAND.johnson == 1){
      // Lower 
      analogWrite(PWMR, 0);
      analogWrite(PWML, 255);
    }
    else if(data_COMMAND.johnson == 2){
      // Up 
      analogWrite(PWMR, 255);
      analogWrite(PWML, 0);
    }
    else{
      analogWrite(PWMR, 0);
      analogWrite(PWML, 0);  
    } 
    // data_COMMAND.johnson = 0;
    
    // Pumps Control --> true or false
    pumps_control();  

    // WormGear Control
    WormControl_1();
    WormControl_2();
    delay(10);
  }
}

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);

  Wire.begin();
  dht_1.begin();
  dht_2.begin();
  if (!bmp.begin()) {
	Serial.println("BMP Error, Check Wiring!");
  data_t.bmp_pressure = NULL;
	// while (1) {} //Infinite Loop
  }
  if (!mlx.begin()) {
    Serial.println("MLX Error. Check wiring.");
    data_t.mlx_objtemp = NULL;
  //  while (1);
  };

  pinMode(pin_mq_1, INPUT);
  pinMode(pin_mq_2, INPUT);
  

  // Johnson Motor
  pinMode(PWMR, OUTPUT);
  pinMode(PWML, OUTPUT);
  // Rest value for Motors
  analogWrite(PWMR, 0);
  analogWrite(PWML, 0);
  

  // Servo Motor
  servo1.attach(ServoPin1);
  servo1.write(0); // Reference Value
  servo2.attach(ServoPin2);
  servo2.write(0); // Reference Value
  
  // Pumps Setup for 4 Pumps A, B, C and D
  pinMode(Pump_INA1, OUTPUT);
  pinMode(Pump_INA2, OUTPUT);
  pinMode(Pump_INB1, OUTPUT);
  pinMode(Pump_INB2, OUTPUT);
  pinMode(Pump_INC1, OUTPUT);
  pinMode(Pump_INC2, OUTPUT);
  pinMode(Pump_IND1, OUTPUT);
  pinMode(Pump_IND2, OUTPUT);
  // Rest value for Motors
  analogWrite(Pump_INA1, 0);
  analogWrite(Pump_INA2, 0);
  analogWrite(Pump_INB1, 0);
  analogWrite(Pump_INB2, 0);
  analogWrite(Pump_INC1, 0);
  analogWrite(Pump_INC2, 0);
  analogWrite(Pump_IND1, 0);
  analogWrite(Pump_IND2, 0);
  
  // Wormgear Motor
  pinMode(Worm_PWMR_1, OUTPUT);
  pinMode(Worm_PWML_1, OUTPUT);
  pinMode(Worm_PWMR_2, OUTPUT);
  pinMode(Worm_PWML_2, OUTPUT);
  // Rest Value
  analogWrite(Worm_PWMR_1, 0);
  analogWrite(Worm_PWML_1, 0);
  analogWrite(Worm_PWMR_2, 0);
  analogWrite(Worm_PWML_2, 0);

  // Wifi Setup:
  WiFi.mode(WIFI_STA);
  // Init ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }
  // Once ESPNow is successfully Init, Register for Send Callback to
  // get the status of Trasnmitted packet
  esp_now_register_send_cb(OnDataSent);

  // Register peer
  memcpy(peerInfo.peer_addr, broadcastAddress, 6);
  peerInfo.channel = 0;  
  peerInfo.encrypt = false;
  
  // Add peer        
  if (esp_now_add_peer(&peerInfo) != ESP_OK){
    Serial.println("Failed to add peer");
    delay(1000);
    return;
  }
  // Register for a callback function that will be called when data is received
  esp_now_register_recv_cb(OnDataRecv);
  // Done Wifi Setup
    Serial.println("HERE");

  // Creating Task for esp32
  // Sensors and Wifi Data Publishing on Core 0
  xTaskCreatePinnedToCore(
          Sensor_Task, // Task function
          "Sensors", // Name of Task
          10000, // Stack Size
          NULL, // Parameters
          2, // Priority
          &Sensors, // Task Handler
          0 // Core
  );
  // Motor Control Task on Core 1
  xTaskCreatePinnedToCore(
          Motor_Task, // Task function
          "Motor", // Name of Task
          10000, // Stack Size
          NULL, // Parameters
          1, // Priority
          &Motor, // Task Handler
          0 // Core
  );
}

void loop() {
  // put your main code here, to run repeatedly:
  // Nothing
  // Send Sensors Data via ESP-NOW
    esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *) &data_t, sizeof(data_t));   
    if (result == ESP_OK) {
      //Serial.println("Sent with success");
    }
    else {
      //Serial.println("Error sending the data");
    }
    delay(10);
}
