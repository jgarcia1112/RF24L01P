//
// nRF24L01+ Wireless driver para Arduino UNO rev. 3
// partiendo casi desde CERO
//
// pagina de apoyo inicial: https://arduino-info.wikispaces.com/Nrf24L01-2.4GHz-HowTo#ex1
// de esa pagina he tomado parte de codigo que he modificado ligeramente asi como tambien ideas
// para realizar los procedimientos y funciones adaptados a las necesidades de mi proyecto.
// conociendo SPI: https://www.arduino.cc/en/Reference/SPI
//
// J. Garcia @ 16 Sep 2015, TAB. MEX.
// Creado el 16 de Sept 2015
// Modificado el 08 de Oct 2015 - en etapa de desarrollo
//
// AVISO: ESTE AUN ES UN DESARROLLO PARCIAL. EL OBJETO CLASE DE ESTE DISPOSITIVO CONTINUA EN DESARROLLO
// Y AUN SE TIENEN ALGUNAS CARACTERISTICAS IMPORTANTES EN ESPERA, POR FAVOR TENGA ESTO EN CUENTA.
// SI ESTE CODIGO PARCIAL TE ES DE UTILIDAD EN ALGO, ES TU DECISION USARLO ASI COMO ESTA, SIN OFRECER
// GARANTIA DE NIGUN TIPO Y CON LA LIBERTAD DE MODIFICARLO AL GUSTO!!


// Incluye las librerias  SPI y la del modulo RF
#include <SPI.h>
#include "RF24L01P.h"


// Pines en Arduino UNO
#define CEpin       9    // pin #3-CE  en el RF24L01+
#define CSNpin     10    // pin10-CSN  en el RF24L01+

//Crea un objeto clase del modulo
RF24L01P rf(CEpin, 10);

    // ** declara las variables: **//

    // cuantos datos a enviar (maximo 32 bytes)
    // tomar en cuenta que un tipo INT son 2 bytes
    // un LONG son 4 bytes
    uint8_t bufferData[1];

    // remitente (PTXn). de quien se reciven los datos
    uint8_t ptxSender;

void setup() {
    //inicializa el puerto serial para ver los datos por la terminal
    Serial.begin(38400);
    
    // inicializa el modulo (por default queda en modo PTX)
    rf.initialize();

    // selecciona el modo de operacion
    rf.setPRXmode();
    
    Serial.println();
    Serial.println("...monitoreando el aire!!...");
}

void loop() {

// ****************************************************************************
//    Ejemplo de recepcion de datos con (PRX mode)
// ****************************************************************************

    while ( rf.receive(&bufferData, &ptxSender, sizeof(bufferData)) ) {
  
        Serial.print("..datos recibidos de PTX0 ");
        Serial.print(ptxSender); Serial.print("  :  ");
        
        
        for (uint8_t i=0; i < sizeof(&bufferData)-1; i++)
            { Serial.print(bufferData[i]); Serial.print(", "); }
        Serial.println();

        if (bufferData[0])
        { 
            noTone(4);
            tone(4, 440, 200);
            delay(200);
            noTone(4);
            // play a note on pin 7 for 500 ms:
            tone(4, 494, 500);
            delay(500);
            noTone(4);
            // play a note on pin 8 for 500 ms:
            tone(4, 523, 300);
            delay(300);       
        } 
        else 
        {
            noTone(4);
            tone(4, 440, 200);
            delay(200);  
        }
    }

// ****************************************************************************
// ****************************************************************************

}
