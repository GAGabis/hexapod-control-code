#include <esp_now.h>
#include <WiFi.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>


#define SDA 21 // ENTRADA SDA DO DISPLAY LCD
#define SCL 22 // ENTRADA SCL DO DISPLAY LCD

#define JX 35 // EIXO X DO JOYSTICK
#define JY 34 // EIXO Y DO JOYSTICK
#define JZ 39 // EIXO Z DO JOYSTICK


LiquidCrystal_I2C lcd(0x27, 20, 4);


uint8_t broadcastAddress[] = {0x0C, 0xB8, 0x15, 0xC2, 0x48, 0xDC};

typedef struct struct_message {
  char a;

} struct_message;


struct_message myData;

esp_now_peer_info_t peerInfo;

// callback when data is sent
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  Serial.print("\r\nLast Packet Send Status:\t");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
}
 
void setup() {
  
  Serial.begin(115200);
  
  pinMode(JX, INPUT);
  pinMode(JY, INPUT);
  pinMode(JZ, INPUT);

  lcdStart();
  
  // Set device as a Wi-Fi Station
  WiFi.mode(WIFI_STA);

  // Init ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }


  esp_now_register_send_cb(OnDataSent);
  

  memcpy(peerInfo.peer_addr, broadcastAddress, 6);
  peerInfo.channel = 0;  
  peerInfo.encrypt = false;
    
  if (esp_now_add_peer(&peerInfo) != ESP_OK){
    Serial.println("Failed to add peer");
    return;
  }
}
 
void loop() {
  
  lcdLoop();
  
  int joystick_x = analogRead(JX);
  int joystick_y = analogRead(JY);
  
  int mappingX = map(joystick_x, 0, 4095, 0, 3);
  int mappingY = map(joystick_y, 0, 4095, 0, 3);
  
  Serial.println("Valor de X: " + mappingX);
  Serial.println("Valor de Y: " + mappingY);
  Serial.println("Dado recebido: " + myData.a);
  
  joystickMoviment(mappingX, mappingY);
  
  //esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *) &myData, sizeof(myData));
   

}

void joystickMoviment(int x, int y){

  if(x == 1 && y == 1){
    myData.a = ' ';
    esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *) &myData, sizeof(myData));
  if (result == ESP_OK) {
      Serial.println("Sent with success");
    }
    else {
      Serial.println("Error sending the data");
    }
  }

  
  if(x < 1){
    myData.a = 'L';
    esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *) &myData, sizeof(myData));
  if (result == ESP_OK) {
    Serial.println("Sent with success");
  }
  else {
    Serial.println("Error sending the data");
  }
 }
 if(x > 1){
    myData.a = 'R';
    esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *) &myData, sizeof(myData));
  if (result == ESP_OK) {
    Serial.println("Sent with success");
  }else{
    Serial.println("Error sending the data");
  }
 }

  if(y > 1){
      myData.a = 'F';
      esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *) &myData, sizeof(myData));
    if (result == ESP_OK) {
      Serial.println("Sent with success");
    }else{
      Serial.println("Error sending the data");
    }
  }
  if(y < 1){
    myData.a = 'B';
    esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *) &myData, sizeof(myData));
   if (result == ESP_OK){
    Serial.println("Sent with success");
   }else{
    Serial.println("Error sending the data");
   }
    
  }

  
}
void lcdStart(){
  lcd.begin();
  lcd.backlight();
  lcd.clear();
  lcd.setCursor(4,1);
  lcd.print("HEXAPOD 2023");
  lcd.setCursor(4,2);
  lcd.print("CONTROLE V01");
  Serial.println("LCD Pronto!");
}

void lcdLoop(){
  lcd.begin();
  lcd.backlight();
  lcd.clear();
  lcd.setCursor(4,0);
  lcd.print("HEXAPOD 2023");
  lcd.setCursor(4,1);
  lcd.print("CONTROLE V01");
  lcd.setCursor(3,2);
  lcd.print("MODO: JOYSTICK");
  lcd.setCursor(1,3);
  lcd.print(myData.a);


}
