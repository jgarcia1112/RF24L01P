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
// Modificado el 25 de Oct 2015 - en fase de pruebas (Arduino Uno y Arduino Nano)
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
#define CSNpin     10    // pin #4-CSN en el RF24L01+

//Crea un objeto clase del modulo
RF24L01P rf(CEpin, 10);

    // ** declara las variables: **//

    // cuantos datos a enviar (maximo 32 bytes)
    // tomar en cuenta que un tipo INT son 2 bytes
    // un LONG son 4 bytes, etc.
    // a manera de ejemplo, estoy ocupando un array de 1 elemento tipo Byte
    uint8_t bufferData[1];

    // remitente (PTXn). de quien se reciven los datos
    uint8_t ptxSender;

    // variable para tomar medida del tiempo transcurrido
    uint32_t current;

    // variable generica solo para enviar un numero que se ira incrementando cada 10 segundos
    uint8_t veces = 1;

void setup() {

    //inicializa el puerto serial para ver los datos por la terminal
    Serial.begin(38400);
    Serial.println();
    Serial.println("Inicializando puerto serial...");

    // inicializa el modulo con coneccion SPI
    rf.initialize();
    Serial.println("Inicializando dispositivo nRF24L01P...");

    // inicializando modo de operacion
        // selecciona el modo de operacion (pasa como parametro el canal en el que
        // va a transmitir datos).
        // el rango de canales es de: PTX01 hasta PTX06. (es un canal por cada dispositivo RF)
        rf.setPRXmode();
        Serial.println("Entrando en modo PRX...");
    
    Serial.println();
    Serial.println("...monitoreando el aire!!...");
    Serial.println();

    current =  millis();
}

void loop() {

// ****************************************************************************
//    Ejemplo de recepcion de datos con (PRX mode) y transmision de carga con ACK
// ****************************************************************************
    
    // mientras sea verdadero, estara recibiendo datos para procesarlos
    while ( rf.receive(&bufferData, &ptxSender, sizeof(bufferData)) ) {
  
        Serial.print("... recibiendo datos de PTX0");               // de quien vienen
        Serial.print(ptxSender); Serial.print(" : ");               // los datos recibidos
        
        
        for (uint8_t i=0; i < sizeof(&bufferData)-1; i++)           // presenta el dato
            { Serial.print(bufferData[i]==0 ? "LOW":"HIGH"); }      // recibido, en la terminal
        Serial.println();

        if (bufferData[0])                                          // genera un aviso sonoro
        {                                                           // cuando recibe el valor HIGH
            noTone(4);                                              // del sensor de movimiento PIR
            tone(4, 440, 200);                                      // que esta en el otro lado
            delay(200);                                             
            noTone(4);
            
            tone(4, 494, 500);
            delay(500);
            noTone(4);
            
            tone(4, 523, 300);
            delay(300);       
        } 
        else 
        {                                                           // genera otro aviso sonoro
            noTone(4);                                              // cuando recibe un valor LOW
            tone(4, 440, 200);                                      // porque el sensor termino de 
            delay(200);                                             // percibir movimiento
        }
    }

    // Mientras el modulo no recibe datos para procesar,
    // cada 10 segundos se esta enviando un dato, que lo recibira
    // la proxima vez que el sensor transmita informacion.
    if ( (millis() - current) >= 10000) {
        bufferData[0] = veces++;
        rf.send_ackPayload(bufferData, PTX01, sizeof(bufferData));
        current = millis();
        Serial.println("Instuccion enviada al nodo PTX01");
    }

// ****************************************************************************
// ****************************************************************************

}


