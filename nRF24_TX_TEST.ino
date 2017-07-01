/*
 * 
 * modulo transmisor nRF24L01+
 * conectado en Arduino UNO rev 3.
 * 
 */
 
#include <SPI.h>

#include <NRF24L01P.h>

#define CEpin         9    // pin #3-CE en el RF24L01+

nRF24L01P nrf(CEpin); 

int data[2];

uint32_t lastMicros;

void setup() {
  
  // put your setup code here, to run once:

  Serial.begin(38400);

  nrf.begin(); 

  nrf.startTx(CHANNEL1);

  //nrf.enableLiveStream();

}

void loop() {
  
  // put your main code here, to run repeatedly:

  data[0] = 750;

  data[1] = 23000;  

  lastMicros = micros();
   
  nrf.sendData(data, sizeof(data));

  lastMicros = micros() - lastMicros;

  if (nrf.dataSent()) {

    Serial.print("-Enviados "); Serial.print(sizeof(data)); Serial.print(" bytes"); Serial.print(", en ");
    
    Serial.print(lastMicros); Serial.println(" microSegundos");
    
  }

}









