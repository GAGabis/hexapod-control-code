#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <esp_now.h>
//#include "BluetoothSerial.h"

//#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
//#error Bluetooth is not enabled! Please run `make menuconfig` to and enable it
//#endif

//BluetoothSerial SerialBT;


#define PWM_DRIVER1_ADDR 0x60 //Patas Esquerda - Placa na direita
#define PWM_DRIVER2_ADDR 0x40 //Patas Direita - Placa na esquerda

Adafruit_PWMServoDriver myServo = Adafruit_PWMServoDriver(PWM_DRIVER1_ADDR); //Patas Esquerda
Adafruit_PWMServoDriver myServo2 = Adafruit_PWMServoDriver(PWM_DRIVER2_ADDR); //Patas Direita


typedef struct struct_message {
  char a;
} struct_message;

struct_message myData;

esp_now_peer_info_t peerInfo;


char incomingByte;

//[]Perna []Motor(Ombro/Cot/Braco) []pos/setPointm []posPWM/setPointPWM
int set[6][3][4] = { {{0, 0, 0}, {0, 0, 0}, {0, 0, 0}}, //Perna 1
  {{0, 0, 0}, {0, 0, 0}, {0, 0, 0}}, //Perna 2
  {{0, 0, 0}, {0, 0, 0}, {0, 0, 0}}, //Perna 3
  {{0, 0, 0}, {0, 0, 0}, {0, 0, 0}}, //Perna 4
  {{0, 0, 0}, {0, 0, 0}, {0, 0, 0}}, //Perna 5
  {{0, 0, 0}, {0, 0, 0}, {0, 0, 0}}
}; //Perna 6


//Ombro/Cot/Braco
int PERNAS_PINS[6][3] = {

  { 0,   1,  2 },  //Perna 0 - Esquerda- myServo (0x40)
  { 6,  5,  4},  //Perna 1 - Direita - myServo2 (0x60)
  {13,  14, 15 },  //Perna 2 - Esquerda- myServo (0x40)


  {15, 14, 13 },   //Perna 3 - Direita - myServo (0x60)
  {8,  9,  10 },   //Perna 4 - Esquerda- myServo2 (0x40)
  {2 ,  1, 0  }    //Perna 5 - Direita - myServo (0x60)
};




unsigned long tempo;
unsigned long tempo2;
unsigned long tempo3;

int frame = 1;
int frame2 = 1;
int frame3 = 1;

void setup() {
  Wire.begin();

  myServo.begin();
  myServo2.begin();

  myServo.setPWMFreq(60);
  myServo2.setPWMFreq(60);

  tempo = millis();
  tempo2 = millis();
  tempo3 = millis();
  Serial.begin(115200);
  //SerialBT.begin("hexapod32");
  WiFi.mode(WIFI_MODE_STA);
  esp_now_register_recv_cb(OnDataRecv);


  idle();
  delay(3000);
}


//myServo (0x60) - Pata Esquerda - Patas 2 4 0
//myServo2 (0x40) - Pata Direita -  Patas 5 1 3


/* Numeração das patas
        O_O
   2 \       / 5
      \     /
  4 ---     --- 1
      /     \
   0 /       \ 3
*/

int count = 0;
bool mov = false;
bool movimentando = false;
bool movimentando2 = false;
bool movimentando3 = false;
bool paraFrente = false;

int counter = 50;
void loop() {

  updateSerialBT();
  frente();
  girarHorario();
  girarAntiHorario();
          
}


void calibrationServo(){
  if(Serial.available() > 0){
    char incomingByte = Serial.read();
    if(incomingByte == 'f'){
     counter = counter + 10;
     movimenta(myServo, 4, 130, counter, 90); //mid left <-
           Serial.println(counter);

    }else if(incomingByte == 'b'){
      counter = counter - 10;
      movimenta(myServo, 4, 130, counter, 90); //mid left <-
      Serial.println(counter);
    }
  }
      
}

void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) {
  memcpy(&myData, incomingData, sizeof(myData));
  Serial.print("Bytes received: ");
  Serial.println(len);
  Serial.print("Char: ");
  Serial.println(myData.a);

}
void updateSerialWIFI(){
  
    if(myData.a == 'F'){
      movimentando = true;
      paraFrente = true;
      Serial.println(myData.a);
    }else if(myData.a == 'B'){
      movimentando = true;
      paraFrente = false;
      Serial.println(myData.a);
    }else if(myData.a == 'R'){
      movimentando2 = true;
      Serial.println(myData.a);
    }else if(myData.a == 'L'){
      movimentando3 = true;
      Serial.println(myData.a);
    }
}
void updateSerialBT() {

  if (SerialBT.available()) {
    incomingByte = SerialBT.read();
    Serial.println(incomingByte);
    if (incomingByte == 'F') {
      Serial.println("Moving Forward!");
      //frente(true);
      movimentando = true;
      paraFrente = true;
    } else if (incomingByte == 'B') {
      Serial.println("Moving Backward!");
      // frente(false);
      movimentando = true;
      paraFrente = false;
    }
    else if (incomingByte == 'R') {
      movimentando2 = true;

    } else if (incomingByte == 'L') {
      movimentando3 = true;
    }
  }
}

void girarAntiHorario(){
  if (movimentando3 == true){
    if(millis() > tempo3 + 250){
        switch(frame3){
          case 1:
            movimenta(myServo2, 3, 60, 30, 90);
            movimenta(myServo, 4, 130, 150, 90); //mid left <-
            movimenta(myServo2, 5, 50, 30, 90);
            delay(100);
            movimenta(myServo, 0, 110, 180, 30);
            movimenta(myServo2, 1, 70, 0, 40); // mid right ->
            movimenta(myServo, 2, 110, 180, 40);
          break;
          case 2:
            movimenta(myServo, 0, 130, 160, 30);
            movimenta(myServo2, 1, 50, 30, 40); // mid right ->
            movimenta(myServo, 2, 130, 170, 40);
            delay(100);
            movimenta(myServo2, 3, 70, 0, 90);
            movimenta(myServo, 4, 110, 180, 90); //mid left <-
            movimenta(myServo2, 5, 70, 0, 90);

          break;
          case 3:
          
            movimenta(myServo,  0, 130, 160, 90);
            movimenta(myServo2, 1, 50,  30,  90); // mid right ->
            movimenta(myServo,  2, 130, 170, 90);
            delay(100);
            movimenta(myServo2, 3, 70, 0, 90);
            movimenta(myServo, 4, 110, 180, 90); //mid left <-
            movimenta(myServo2, 5, 70, 0, 90);

          break;
          case 4:
            movimenta(myServo2, 3, 60, 30, 90);
            movimenta(myServo, 4, 130, 150, 90); //mid left <-
            movimenta(myServo2, 5, 50, 30, 90);
            delay(100);
            movimenta(myServo,  0, 110, 180, 150);
            movimenta(myServo2, 1,  70,   0, 140); // mid right ->
            movimenta(myServo,  2, 110, 180, 140);

          break;
          default:
          break;
        }
        frame3 = frame3 + 1;      
      
      if (frame3 >= 5) {
        frame3 = 1;
        movimentando3 = false;
        idle();
      }
        
      tempo3 = millis();
    }
  }
}

void girarHorario(){
  if (movimentando2 == true){
    if(millis() > tempo2 + 150){
        switch(frame2){
          case 1:
            movimenta(myServo2, 3, 60, 30, 90);
            movimenta(myServo, 4, 130, 150, 90); //mid left <-
            movimenta(myServo2, 5, 50, 30, 90);
            delay(100);
            movimenta(myServo, 0, 110, 180, 150);
            movimenta(myServo2, 1, 70, 0, 140); // mid right ->
            movimenta(myServo, 2, 110, 180, 140);
          break;
          case 2:
            movimenta(myServo, 0, 130, 160, 150);
            movimenta(myServo2, 1, 50, 30, 140); // mid right ->
            movimenta(myServo, 2, 130, 170, 140);
            delay(100);
            movimenta(myServo2, 3, 70, 0, 90);
            movimenta(myServo, 4, 110, 180, 90); //mid left <-
            movimenta(myServo2, 5, 70, 0, 90); 
            

          break;
          case 3:
          
            movimenta(myServo,  0, 130, 160, 90);
            movimenta(myServo2, 1, 50,  30,  90); // mid right ->
            movimenta(myServo,  2, 130, 170, 90);
            delay(100);
            movimenta(myServo2, 3, 70, 0, 90);
            movimenta(myServo, 4, 110, 180, 90); //mid left <-
            movimenta(myServo2, 5, 70, 0, 90);

          break;
          case 4:
            movimenta(myServo2, 3, 60, 30, 90);
            movimenta(myServo, 4, 130, 150, 90); //mid left <-
            movimenta(myServo2, 5, 50, 30, 90);
            delay(100);
            movimenta(myServo,  0, 110, 180, 30);
            movimenta(myServo2, 1,  70,   0, 40); // mid right ->
            movimenta(myServo,  2, 110, 180, 40);

          break;
          default:
          break;
        }
        frame2 = frame2 + 1;
      if (frame2 >= 5) {
        frame2 = 1;
        movimentando2 = false;
        idle();
      }
      tempo2 = millis();
    }
  }
}


void idle() {

  //running motors sequence [2,1,0]

          movimenta(myServo, 0,  130, 160, 90);
          movimenta(myServo2, 1, 50,  30,  90); // mid right ->
          movimenta(myServo, 2,  130, 170, 90); //
         
          movimenta(myServo2, 3, 60,  30,   90);
          movimenta(myServo, 4,  130, 150, 90); //mid left <-
          movimenta(myServo2, 5, 50,  30,   90);
}

void frente() {
  if (movimentando == true ) {
    if (millis() - tempo > 150) {
      switch (frame) {
        case 1:
          
          movimenta(myServo2, 3, 60, 30, 120);
          movimenta(myServo, 4, 130 , 150, 60); //mid left <-
          movimenta(myServo2, 5, 50, 30, 120);
          delay(100);
          movimenta(myServo, 0, 110, 170, 120); //20 - 10 
          movimenta(myServo2, 1, 70, 0, 60); // mid right -> // 20 - 30
          movimenta(myServo, 2, 110, 180, 120); // 20 - 10
          
          
          break;
        case 2:
          movimenta(myServo, 0, 130, 160, 120);
          movimenta(myServo2, 1, 50, 30, 60); // mid right ->
          movimenta(myServo, 2, 130, 170, 120);
          delay(100);
          movimenta(myServo2, 3, 70, 0, 120);
          movimenta(myServo, 4, 110, 180, 60); //mid left <-
          movimenta(myServo2, 5, 70, 0, 120);
          
          break;
        case 3:
          
          movimenta(myServo, 0,  130, 160, 60);
          movimenta(myServo2, 1, 50,  30,  120); // mid right ->
          movimenta(myServo, 2,  130, 170, 60); //
          delay(100);
          movimenta(myServo2, 3, 70,  0,   60);
          movimenta(myServo, 4,  110, 180, 120); //mid left <-
          movimenta(myServo2, 5, 70,  0,   60);
          
          break;
        case 4:
          delay(100);
          movimenta(myServo2, 3, 50, 30, 60);
          movimenta(myServo, 4, 130 , 150, 120); //mid left <-
          movimenta(myServo2, 5, 50, 30, 60);
          delay(100);
          movimenta(myServo, 0, 130, 160, 60);
          movimenta(myServo2, 1, 50, 30, 120); // mid right ->
          movimenta(myServo, 2, 130, 170, 60);
          delay(100);
          
          break;

        default:
          break;
      }

      if (paraFrente == true) {
        frame = frame + 1;
      } else {
        frame = frame - 1;
      }

      if (frame >= 5) {
        frame = 1;
        movimentando = false;
        idle();
      } else if (frame <= 0) {
        frame = 4;
        movimentando = false;
        idle();
      }
      tempo = millis();
    }
  }
}

/* Numeração das patas
        O_O
   2 \       / 5
      \     /
  4 ---     --- 1
      /     \
   0 /       \ 3
*/


void movimenta(Adafruit_PWMServoDriver &driver, int perna, int setOmbro, int setCot, int setBraco) {

  set[perna][0][1] = setOmbro;
  set[perna][1][1] = setCot;
  set[perna][2][1] = setBraco;

  //Serial.println(set[perna][0][1]);
  //Serial.println(set[perna][0][0]);
  //Serial.println(mov);
  //Serial.println("-----------------------------");


  set[perna][0][2] = map(setOmbro, 0, 180, 220, 540);
  set[perna][1][2] = map(setCot, 0, 180, 220, 540);
  set[perna][2][2] = map(setBraco, 0, 180, 220, 540);

  /* setPoint(perna,0);
    setPoint(perna,1);
    setPoint(perna,2);*/




  switch (perna) {
    case 0:
      //Perna 0
      driver.setPWM(PERNAS_PINS[perna][0], 0, set[perna][0][2]);
      driver.setPWM(PERNAS_PINS[perna][1], 0, set[perna][1][2]);
      driver.setPWM(PERNAS_PINS[perna][2], 0, set[perna][2][2]);
      set[perna][0][0] = setOmbro;
      set[perna][1][0] = setCot;
      set[perna][2][0] = setBraco;
      break;
    case 1:
      //Perna 1
      driver.setPWM(PERNAS_PINS[perna][0], 0, set[perna][0][2]);
      driver.setPWM(PERNAS_PINS[perna][1], 0, set[perna][1][2]);
      driver.setPWM(PERNAS_PINS[perna][2], 0, set[perna][2][2]);
      set[perna][0][0] = setOmbro;
      set[perna][1][0] = setCot;
      set[perna][2][0] = setBraco;
      break;
    case 2:
      driver.setPWM(PERNAS_PINS[perna][0], 0, set[perna][0][2]);
      driver.setPWM(PERNAS_PINS[perna][1], 0, set[perna][1][2]);
      driver.setPWM(PERNAS_PINS[perna][2], 0, set[perna][2][2]);
      set[perna][0][0] = setOmbro;
      set[perna][1][0] = setCot;
      set[perna][2][0] = setBraco;
      break;
    case 3:
      driver.setPWM(PERNAS_PINS[perna][0], 0, set[perna][0][2]);
      driver.setPWM(PERNAS_PINS[perna][1], 0, set[perna][1][2]);
      driver.setPWM(PERNAS_PINS[perna][2], 0, set[perna][2][2]);
      set[perna][0][0] = setOmbro;
      set[perna][1][0] = setCot;
      set[perna][2][0] = setBraco;
      break;
    case 4:
      driver.setPWM(PERNAS_PINS[perna][0], 0, set[perna][0][2]);
      driver.setPWM(PERNAS_PINS[perna][1], 0, set[perna][1][2]);
      driver.setPWM(PERNAS_PINS[perna][2], 0, set[perna][2][2]);
      set[perna][0][0] = setOmbro;
      set[perna][1][0] = setCot;
      set[perna][2][0] = setBraco;
      break;
    case 5:
      driver.setPWM(PERNAS_PINS[perna][0], 0, set[perna][0][2]);
      driver.setPWM(PERNAS_PINS[perna][1], 0, set[perna][1][2]);
      driver.setPWM(PERNAS_PINS[perna][2], 0, set[perna][2][2]);
      set[perna][0][0] = setOmbro;
      set[perna][1][0] = setCot;
      set[perna][2][0] = setBraco;
      break;
    default:
      break;
  }

}

void setPoint(int perna, int motor) {

  //Incremento apenas para subir ou frente
  if (set[perna][motor][1] != set[perna][motor][0]) {
    if (set[perna][motor][1] > set[perna][motor][0]  && mov == true) {
      set[perna][motor][0] = set[perna][motor][0] + 1;
    } else {
      mov = false;
    }
    if (set[perna][motor][1] < set[perna][motor][0] && mov == false) {
      set[perna][motor][0] = set[perna][motor][0] - 1;
    } else {
      mov = true;
    }
    set[perna][motor][2] = map(set[perna][motor][0], 0, 180, 220, 540);

  }
}

bool pataMovendo() {
  for (int i = 0; i <= 5; i++) {
    for (int j = 0; j <= 3; j++) {
      if (set[i][j][0] != set[i][j][1]) {
        return true;


      }
    }
    return false;
  }
}
