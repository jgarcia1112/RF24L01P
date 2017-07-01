/*
 * 
 * modulo receptor nRF24L01+
 * conectado en Arduino Mega rev 3.
 * 
 */

#include <SPI.h>

#include <NRF24L01P.h>

#define CEpin         9    // pin #3-CE en el RF24L01+

nRF24L01P nrf(CEpin); 

int data[2];

byte lenData, sender;

uint32_t lastMicros;

void setup() {

  // put your setup code here, to run once:

  Serial.begin(38400);

  nrf.begin(); 

  nrf.startRx();

  //nrf.enableLiveStream();

}

void loop() {
 
  // put your main code here, to run repeatedly:

  while (nrf.dataReady()) {

    lastMicros = micros(); 
    
    nrf.download(data, lenData, sender);
    
    lastMicros = micros() - lastMicros;
    
    Serial.print("-Recibidos "); Serial.print(lenData); Serial.print(" bytes, del canal: 0"); Serial.print(sender); 
    
    Serial.print(", en "); Serial.print(lastMicros); Serial.println(" microSegundos");

  }

}





