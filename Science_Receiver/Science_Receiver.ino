#include <esp_now.h>
#include <WiFi.h>

uint8_t broadcastAddress[] = {0x44, 0x17, 0x93, 0xE6, 0x37, 0x08};
// MAC Address of SCIENCE`s ESP32 44:17:93:E6:37:08
esp_now_peer_info_t peerInfo;

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
struct science data_t_in;

struct commands{
  int johnson; // 0->Stationary; 1->UP; 2->DOWN
  int wormgear_pwm_1; // 0->Stationary; 1->UP; 2->DOWN
  int wormgear_pwm_2; // 0->Stationary; 1->UP; 2->DOWN
  int servo_shake; // 0->OFF; 1->ON 
  int pumps_1; // false->OFF; true->ON
  int pumps_2; // false->OFF; true->ON
};
struct commands data_t_out = {0, 0, 0, 0, 0, 0};

const int numValues = 6;
int receivedValues[numValues];

// Function Prototyping
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status);
void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len);
void SEND_DATA(struct commands t);
 
void setup() {
  // Initialize Serial Monitor
  Serial.begin(115200);
 
  // Set device as a Wi-Fi Station
  WiFi.mode(WIFI_STA);
  // Init ESP-NOW
  if (esp_now_init() != ESP_OK) {
    // Serial.println("Error initializing ESP-NOW");
    return;
  }
  // Once ESPNow is successfully Init, we will register for Send CB to
  // get the status of Trasnmitted packet
  esp_now_register_send_cb(OnDataSent);
  // Register peer
  memcpy(peerInfo.peer_addr, broadcastAddress, 6);
  peerInfo.channel = 0;  
  peerInfo.encrypt = false;
  // Add peer        
  if (esp_now_add_peer(&peerInfo) != ESP_OK){
    // Serial.println("Failed to add peer");
    return;
  }
  // Register for a callback function that will be called when data is received
  esp_now_register_recv_cb(OnDataRecv);
}

void loop() {

  /*CODE FOR UPDATING VALUE OF DATA_T_OUT BELOW*/
  // Furquan
  // data_t_out = Value from Serial Commmunication

  while (Serial.available() < numValues * 2) {
    delay(10);
  }
  

  // Read and store the three integer values
  for (int i = 0; i < numValues; i++) {
    receivedValues[i] = Serial.read() << 8 | Serial.read();
  }

  data_t_out.johnson = receivedValues[2];
  data_t_out.wormgear_pwm_1 = receivedValues[0];
  data_t_out.wormgear_pwm_2 = receivedValues[1];
  data_t_out.servo_shake = receivedValues[4]; 
  data_t_out.pumps_1 = receivedValues[5];
  data_t_out.pumps_2 = receivedValues[6];

  // Reset the received values array
  memset(receivedValues, 0, sizeof(receivedValues));

  SEND_DATA(data_t_out);
  delay(10);
}


// Callback when data is sent
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  // Serial.print("\r\nLast Packet:\t");
  // Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
}

// Callback function that will be executed when data is received
void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) {
  memcpy(&data_t_in, incomingData, sizeof(data_t_in));
  /*
  ADD CODE for Sending Senosr Data to JETSON
  */

  Serial.println(data_t_in.mq1_ppm);
  Serial.println(data_t_in.mq2_ppm);
  Serial.println(data_t_in.dht1_temp);
  Serial.println(data_t_in.dht1_humidity);
  Serial.println(data_t_in.dht2_temp);
  Serial.println(data_t_in.dht2_humidity);
  Serial.println(data_t_in.bmp_pressure);
  Serial.println(data_t_in.mlx_objtemp);
  Serial.println(data_t_in.sht_temp);
  Serial.println(data_t_in.sht_humidity);
  delay(1000);
}

void SEND_DATA(struct commands t){
  esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *) &t, sizeof(t)); 
}