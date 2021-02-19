//
// Inclusao biblioteca MODBUS RTU
 #include <ModbusRtu.h>
//Definicao do pino TX Enable para conversor RS485
 #define TXEN 2
//
/**
 *  Modbus object declaration
 *  u8id : node id = 0 for master, = 1..247 for slave
 *  u8serno : serial port (use 0 for Serial para UNO :: 0,1,2,3 para MEGA 2560R3)
 *  u8txenpin : 0 for RS-232 and USB-FTDI 
 *               or any pin number > 1 for RS-485
 */
Modbus slave(11,0,TXEN); // this is slave @11 and RS-485
//
// Inclusao das bibliotecas para display nokia 5110 azul (5V)
//
#include <SPI.h>
#include <Adafruit_GFX.h>
#include <Adafruit_PCD8544.h>
//
// Definiçao dos pinos para o display nokia 5110
//
// pin 8 - Serial clock out (SCLK)
// pin 9 - Serial data out (DIN)
// pin 10 - Data/Command select (D/C)
// pin 11 - LCD chip select (CS/CE)
// pin 12 - LCD reset (RST)
//
Adafruit_PCD8544 display = Adafruit_PCD8544(8, 9, 10, 11, 12);
//
//  Inclusao da biblioteca para sensor de temperatura e humidade
//  DHT11
//
#include <DHT.h>
// Definiçao dos pinos para sensor DHT11
// pino 5 como saida para dados do DHT11
#define DHTPIN 5
#define DHTTYPE DHT11
//  Definiçao do tipo do sensor para a biblioteca DHT
DHT dht(DHTPIN,DHTTYPE);
//
// Definiçao de pinos para Ultrassom
//
#define trigPin 4
#define echoPin 3
//
// Definiçao de pinos para os sensores Boia
//
#define boiaNivelBaixo 6
#define boiaNivelAlto 7
//
#define dEBUG 14
//
// Variaveis Modbus rtu :h,t,ur,bna,bnb,st
  float h;
  float t;
  float ur;
  long bna;
  long bnb;
  long st;
//
  float di;
  float du;
  float sp;
  String sbnb;
  String sbna;
//
  uint16_t au16data[6];
//
void setup(){
// 
// Inicializa canal serial e velocidade  
// Serial.begin (9600);
//
//
// Inicializar slave
  slave.begin( 19200 ); // baud-rate at 19200

// Inicializa display nokia 5110  
  display.begin();
  display.setContrast(25); //Ajusta o contraste do display
  display.clearDisplay();   //Apaga o buffer e o display
  display.setTextSize(1);  //Seta o tamanho do texto
  display.drawRect(0,0, 84,48, BLACK); //Desenha o retangulo da borda
  display.display();

// Inicializa sensor DHT 
  dht.begin();

// Define pinos de entrada / saida sensor ultrassom e boias   
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
  
// Define pinos de entrada / saida sensor boias  
  pinMode(boiaNivelBaixo, INPUT);
  pinMode(boiaNivelAlto, INPUT);

// Define debug via display e serial
  pinMode(dEBUG, INPUT);
  pinMode(13, OUTPUT);
//
//  End Setup 
//
}
void loop() {
//
// 
  ur = dht.readHumidity();
  t = dht.readTemperature();
//
// Ativa ultrassom
//
  digitalWrite(trigPin, LOW); 
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  
  du = pulseIn(echoPin, HIGH);
  sp = 331.4 + (0.606 * t) + (0.0124 * ur);
  di = (du / 2) * (sp / 10000);
  
  if (di >= 400 || di <= 2){
    di = 999;
    h = 999;
  // se overflow leitura ultrassom erro status st=1   
    st=1;
  }
  else {
    st=0;
    h = di;
  }
//
//  Testa boias de nivel
//
  bnb=digitalRead(boiaNivelBaixo);
    if (bnb==HIGH){
      sbnb="OFF";
    }
    else {
      sbnb="ON ";
    }
  bna=digitalRead(boiaNivelAlto);
    if (bna==HIGH){
      sbna="OFF";
    }
    else {
      sbna="ON ";
    }
// se boia nivel baixo = 0 e boia nivel alto = 1 erro status st=2    
    if (bnb==LOW && bna==HIGH){
      st=2;
    }
    else {
      st=0;
    } 
//
//
//
//    PrintDisplay();
//    
  boolean dbg=digitalRead(dEBUG);
  if (!dbg) {
   PrintDisplay();
   PrintSerialData();
    delay(500);
  }
 
   RegUpdate();
   slave.poll( au16data, 6 );
//    BlinkLed13();
 
}

//
// blink led pin 13
//
void BlinkLed13(void){
//
  digitalWrite  (13,LOW);
//  delay(75);
  digitalWrite  (13,HIGH);
  delay(75);
  digitalWrite  (13,LOW);
//
}

//
//  Atualiza registros Hreg salve
//
void RegUpdate(void){

    au16data[0] = h * 100; // distancia x100 para incluir dois digitos apos virgula
    au16data[1] = t;
    au16data[2] = ur;
    au16data[3] = bna;
    au16data[4] = bnb;
    au16data[5] = st;
//
//  slave.poll( au16data, 6 );
//
}
//
//  Display dados
//
void PrintDisplay (void){
//
    
    display.drawRect(0,0, 84,48, BLACK);
//    display.setTextColor(BLACK,WHITE); //Seta a cor do texto
    display.setCursor(3,4);  //Seta a posição do cursor
    display.print("Temp: ");  
    display.print(t);
//
    display.println(" C");
    display.setCursor(3,13);
    display.print("UmiR: ");
    display.print(ur);
    display.println(" %");
//  
    display.setCursor(3,21);
    display.print("Dist: ");
    display.print(h);
//
    display.setCursor(3,29);
    display.print("BNA : ");
    display.print(sbna) ;
    display.setCursor(3,37);
    display.print("BNB : ");
    display.print(sbnb);
//
    
    display.display();
    delay(1000);
    display.clearDisplay();   //Apaga o buffer e o display
    display.display();
//
}
void PrintSerialData (void){
//
    Serial.println("--x--");
    Serial.print("Distance = ");
    Serial.print(di);
    Serial.println(" cm");
    Serial.print(" temp = ");
    Serial.print(t);
    Serial.println(" C");
    Serial.print(" humi = ");
    Serial.print(ur);
    Serial.println(" %HR");
    Serial.print(" BNA = ");
    Serial.println(sbna);
    Serial.print(" BNB = ");
    Serial.println(sbnb); 
//
}
