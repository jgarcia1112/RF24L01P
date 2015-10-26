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


// Pines en Arduino Nano
#define CEpin       9    // pin #3-CE  en el RF24L01+
#define CSNpin     10    // pin #4-CSN en el RF24L01+
#define Pir         5    // Pir Sensor Signal


//Crea un objeto clase del modulo
RF24L01P rf(CEpin, CSNpin);

    // ** declara las variables: **//
    
    // cuantos datos a enviar (maximo 32 bytes)
    // tomar en cuenta que un tipo INT son 2 bytes
    // un LONG son 4 bytes
    uint8_t dataToSend[1];
    uint8_t recibidos[1];
    
    // inicializando variable del Sensor PIR
    uint8_t pirSensor = LOW;
    


void setup() {

    //inicializa el puerto serial para ver los datos por la terminal
    Serial.begin(38400);
    Serial.println();
    Serial.println("Inicializando puerto serial...");

    // inicializar el sensor PIR
    pinMode(Pir, INPUT);
    Serial.print("Inicializando sensor PIR...");

    Serial.print(" -Estatus del sensor: "); Serial.println(digitalRead(Pir) ? "HIGH-":"LOW-");

    // inicializa el modulo con coneccion SPI
    rf.initialize();
    Serial.println("Inicializando dispositivo nRF24L01P...");

    // inicializando modo de operacion
        // selecciona el modo de operacion (pasa como parametro el canal en el que
        // va a transmitir datos).
        // el rango de canales es de: PTX01 hasta PTX06. (es un canal por cada dispositivo RF)
        rf.setPTXmode(PTX01);
        Serial.println("Entrando en modo PTX...");

Serial.println();
Serial.println("  - Esperando deteccion de movimiento -");
Serial.println();

}

void loop() 
{
// ****************************************************************************
//    Ejemplo de transmision de datos con (PTX mode) y recepcion de carga con ACK
// ****************************************************************************

    // lee la señal del sensor PIR
    pirSensor = digitalRead(Pir);
    
    if (pirSensor)
    {
        // cuando detecta movimiento guardamos el valor HIGH en un array para su envio
        dataToSend[0] = HIGH;
        Serial.println("movimiento detectado");
    
        // realiza el envio y en seguida verifca si hay datos de regreso
        // si los hay, los presenta en la terminal
        rf.send(dataToSend, sizeof(dataToSend));
        if (rf.dataFlag) {
            rf.get_ackPayload(&recibidos, sizeof(recibidos));
            Serial.print( "...recibiendo instrucciones: "); Serial.println(recibidos[0]);
        }
    
        // esperamos a que el sensor deje de detectar movimiento
        while (digitalRead(Pir));
    
        // una vez que ya no hay movimiento, guardamos el valor LOW en el array para enviarlo
        // y enseguida verificamos si tenemos datos de regreso y los presenta en la terminal
        dataToSend[0] = LOW;
        rf.send(dataToSend, sizeof(dataToSend));
        if (rf.dataFlag) {
            rf.get_ackPayload(&recibidos, sizeof(recibidos));
            Serial.print( "...recibiendo instrucciones: "); Serial.println(recibidos[0]);
        }
    }
}
//
// ****************************************************************************


