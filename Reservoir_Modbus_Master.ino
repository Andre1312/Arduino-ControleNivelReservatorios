
/*
  Modbus-Arduino Example - Servo (Modbus IP)
 Copyright by André Sarmento Barbosa
 http://github.com/andresarmento/modbus-arduino
 */

#include <SPI.h>
#include <Ethernet.h>
#include <Modbus.h>
#include <ModbusIP.h>

// ModbusIP object
ModbusIP mb;
//
//long tcp16data[14]; //registros tipo long 16 bits para supervisorio modbus tcp




/**
 *  Modbus master example 2:
 *  The purpose of this example is to query several sets of data
 *  from an external Modbus slave device. 
 *  The link media can be USB or RS232.
 *
 *  Recommended Modbus slave: 
 *  diagslave http://www.modbusdriver.com/diagslave.html
 *
 *  In a Linux box, run 
 *  "./diagslave /dev/ttyUSB0 -b 19200 -d 8 -s 1 -p none -m rtu -a 1"
 * 	This is:
 * 		serial port /dev/ttyUSB0 at 19200 baud 8N1
 *		RTU mode and address @1
 */

#include <ModbusRtuV2.h>

uint16_t au16data[14]; //!< data array for modbus network sharing
uint8_t u8state; //!< machine state
uint8_t u8query; //!< pointer to message query

/**
 *  Modbus object declaration
 *  u8id : node id = 0 for master, = 1..247 for slave
 *  u8serno : serial port (use 0 for Serial)
 *  u8txenpin : 0 for RS-232 and USB-FTDI 
 *               or any pin number > 1 for RS-485
 */

#define TXEN 25

ModbusRtuV2 master(0,1,TXEN); // this is master and RS-232 or USB-FTDI - rs485 txen pin 25 serial 1 do mega2560r3

/**
 * This is an structe which contains a query to an slave device
 */
modbusrtu_t telegram[2];

unsigned long u32wait;

// Definicao pino de saida bomba dagua

#define PUMP 49

void setup() {

  Serial.begin(9600);

  //definicao da saida da bomba dagua

  pinMode(PUMP, OUTPUT);
  digitalWrite(PUMP,HIGH);

  // The media access control (ethernet hardware) address for the shield
  byte mac[] = { 
    0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED                   };  
  // The IP address for the shield
  byte ip[] = { 
    10, 135, 255, 120                   };   
  // Config Modbus IP 
  mb.config(mac, ip);
  // 
  for (int i=0;i<=13;i++){
    mb.addHreg(i,0);
  }
  Serial.println("Server modbus TCP Config OK");

  // telegram 0: read registers
  telegram[0].u8id = 10; // slave address
  telegram[0].u8fct = 3; // function code (this one is registers read)
  telegram[0].u16RegAdd = 0; // start address in slave
  telegram[0].u16CoilsNo = 6; // number of elements (coils or registers) to read
  telegram[0].au16reg = au16data; // pointer to a memory array in the Arduino

  Serial.println("Telegram slave 10 OK");

  // telegram 1: write a single register
  telegram[1].u8id = 11; // slave address
  telegram[1].u8fct = 3; // function code (this one is write a single register)
  telegram[1].u16RegAdd = 0; // start address in slave
  telegram[1].u16CoilsNo = 6; // number of elements (coils or registers) to read
  telegram[1].au16reg = au16data+6; // pointer to a memory array in the Arduino

  Serial.println("Telegram slave 11 OK");

  master.begin( 19200 ); // baud-rate at 19200
  master.setTimeOut( 5000 ); // if there is no answer in 5000 ms, roll over
  u32wait = millis() + 1000;
  u8state = u8query = 0; 


  Serial.println("Master OK");


}

void loop() {

  //server modbus tcp 
  mb.task();

  // maquna de estados modbus rtu
  switch( u8state ) {
  case 0: 
    if (millis() > u32wait) u8state++; // wait state
    break;
  case 1: 
    master.query( telegram[u8query] ); // send query (only once)
    u8state++;
    u8query++;
    if (u8query > 2) u8query = 0; // verificar se u8query>1 ou do jeito que ta
    break;
  case 2:
    master.poll(); // check incoming messages
    if (master.getState() == COM_IDLE) {
      u8state = 0;
      u32wait = millis() + 1000; 
    }
    break;
  }
  //verificar se st1 e st2 for zero testa boias
  //se boia tanque slave 11 for bna=on e bnb slave 10 for on liga a bomba

  //slave 2
  //  long d2 = au16data[6];
  //  long t2 = au16data[7];
  //  long ur2= au16data[8];
  long bna2=au16data[9];
  long bnb2=au16data[10];
  //  long st2=au16data[11];
  //slave 1
  //  long d1 = au16data[0];
  //  long t1 = au16data[1];
  //  long ur1= au16data[2];
  long bna1=au16data[3];
  long bnb1=au16data[4];
  // long st1=au16data[5];


  // Liga bomba dagua se b nivel baixo tanque1 estiver ativa e se o b nivel alto tanque 2 estiver desativado
  if (bna2==1) {

    if (bnb1==0) {
      digitalWrite(PUMP,LOW);
      au16data[12] = 1;
    }
    else {
      digitalWrite(PUMP,HIGH);
      au16data[12] = 0;
    }
  }
  

  //
  Serial.println("Slave 2");
  for (int i=6; i <= 11; i++)
  {
    Serial.println (au16data[i]);
  }
  Serial.println("Slave 1");
  for (int i=0; i <= 5; i++)
  {
    Serial.println (au16data[i]);
  }

  //  atualiza registros tcp server

  for (int i=0;i<=13;i++){

    mb.Hreg(i,au16data[i]);

  }
  delay(133);

}












