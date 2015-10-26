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

/*
 Copyright (C) 2015 J. Garcia <jgarcia1112@gmail.com>
 
 This program is free software; you can redistribute it and/or
 modify it under the terms of the GNU General Public License
 as published by the Free Software Foundation.
 */

// ****************************************************************************
//
//      @file RF24L01P.cpp
//
//      Esta libreria, contempla el uso del modulo de RF usando las
//      caractetisticas del Enhanced ShockBurst™ con Dynamic Payload.
//      No utiliza compatibilidad con modelos anteriores, y esta diseñado para
//      trabajar en el modo 1:6 (Multiceiver), es decir un PRX y seis PTX.
//
//      En el Skecth solo hay que definir cual modulo va a estar en modo PRX
//      y para cada PTX se le va indicando con el numero PTX01, PTX02..PTX06.
//
// ****************************************************************************



#include <SPI.h>
#include "RF24L01P.h"

static uint8_t devAddress[] = { 0x78, 0x78, 0x78, 0x78, 0x78 };
static uint8_t buffer[]     = { 0x0, 0x0, 0x0, 0x0, 0x0 };
static bool noAck = false;




// ****************************************************************************
//
//      Constructor para el bus SPI del nRF24L01P. Este modulo soporta
//      velocidades de 10Mhz, asi que estamos creando una instancia con
//      los ajustes SPI del modulo, y asi no interferir con algun otro
//      dispositivo que se tenga conectado funcionando con diferentes ajustes.
//
// ****************************************************************************
//
        SPISettings nRF24(10000000, MSBFIRST, SPI_MODE0);
//
// ****************************************************************************





// ****************************************************************************
//
//      Constructor de la clase para modulo RF, solo pasa como parametros
//      el pin CE y CSN del modulo. Los demas pines son del SPI.
//      El pin del IRQ no lo utilizo en este software.
//
// ****************************************************************************
//
RF24L01P::RF24L01P(uint8_t CE_pin, uint8_t CSN_Pin) {

    CEpin  = CE_pin;
    CSNpin = CSN_Pin;
    pinMode (CEpin,   OUTPUT);    //pin09
    pinMode (CSNpin, OUTPUT);     //pin10
    SPI.begin();
}
//
// ****************************************************************************





// ****************************************************************************
//
//      Funcion para inicializar el modulo RF con los ajustes por default
//
//  Por default deja habilitadas las funciones del EnhancedShockBurstDynamic
//  Con una velocidad de datos de 2Mbps
//  Transmitiendo en la frecuencia de 2440 MHz (2.44 GHz)
//  Con una potencia de transmision de 00 dBi
//  Y con una direccion de dispositivo de 120,120,120,120, dejando preconfiguradas
//  las direcciones de todos los canales (pipes).
//
//  Todos estos parametros son configurables despues de haber inicializado
//  el modulo.
//
// ****************************************************************************
//
void RF24L01P::initialize()
{
    setPowerDown();
    setEnhancedShockBurstDynamic();
    setRate();
    setRadioFrequency();
    setTXLevel();
    initDeviceAddress();

}
//
// ****************************************************************************



/*
// ****************************************************************************
//
//      Funcion para realizar autoprueba del dispositivo conectado por SPI
//
// ****************************************************************************
//
bool RF24L01P::testDevice()
{
    // ***************************************************************
    //
    //    hace una escritura y comprueba si se realizo el cambio
    //
    // ***************************************************************
    //
            uint8_t test = getDevice(ReadCmd | RF_CONFIG, buffer, 1);
            buffer[0] = 0x7F;
            setDevice(WriteCmd | RF_CONFIG, buffer, 1);
            getDevice(ReadCmd | RF_CONFIG, buffer, 1);
            if (buffer[0] != 0x7F) {
                return false; }
            else {
                buffer[0] = test;
                setDevice(WriteCmd | RF_CONFIG, buffer, 1);
                return true;
    //
    // ***************************************************************
    }
}
//
// ****************************************************************************
*/








// ****************************************************************************
//
//    PTX MODE
//
//    -Transmision de datos (PRX mode) con o sin confirmacion (ack)
//    -Extraccion de datos recibidos con ACK (Ack_PayLoad)
//
// Con la funcion: send(,,)
// devuelve verdadero si los datos se enviaron con o sin datos de regreso (ack)
// devuelve falso si no se logro tener respuesta por parte del receptor.
//
// Con la funcion: send_noAck(,,)
// devuelve siempre verdadero, pues no se espera confirmacion por parte del
// receptor, y de igual forma, no se podran recibir datos de regreso porque
// no hay confirmacion (ack)
//
// Con la funcion: get_ackPayload(,,)
// se extraen los datos que vienen con la confirmacion (ack payload) y los
// devuelve por parametro.  Esto sucedera siempre y cuando la bandera (dataflag)
// sea verdadera. Con esa bandera, podemos saber si hay datos de regreso cada
// vez que se realiza un envio con confirmacion.
//
//
//      POR HACER:
//              VALIDAR QUE EL TAMAÑO DEL PAQUETE NO EXCEDA LOS 32 BYTES
//
// ****************************************************************************
//
bool RF24L01P::send_noAck(const void* txData, uint8_t packetSize)
{
    noAck = true;
    send(txData, packetSize);
    return true;
}
bool RF24L01P::send(const void* txData, uint8_t packetSize)
{
    uint8_t st;
    const int8_t *pTxData = (const int8_t*)txData;
    
    // ***************************************************************
    //
    //    carga el FIFO con datos y manda el pulso para su envio.
    //
    // ***************************************************************
    
            // manda el pulso bajo, para escribir la carga de datos
            sendPulse(LOW);
    
    
                    // escribe los datos en el FIFO
                    digitalWrite(CSNpin,LOW);
                    SPI.beginTransaction(nRF24);
                    SPI.transfer( (noAck==true ? RF_W_TX_PAYLOAD_NO_ACK:RF_W_TX_PAYLOAD) );
    
                    while ( packetSize-- ) SPI.transfer(*pTxData++);
    
                    SPI.endTransaction();
                    digitalWrite(CSNpin,HIGH);


            // manda el MicroPulso de envio para transmitir.
            sendPulse(HIGH);
    //
    // ***************************************************************
    
    
    if (!noAck) {
    // ***************************************************************
    //
    //    Ventana de tiempo para esperar confirmacion.
    //
    // ***************************************************************
    //
            while (true)
            {
                // Ventana de tiempo de espera (Window time To Wait)
                // checa los valores IRQ de:  TX_DS  y  MAX_RT
                // si alguno de ellos es alto, sale del bucle
                st = (getDevice(ReadCmd | RF_STATUS, buffer, 1)>>4);
                if (st) break;
            }
            // Al terminar de procesar los IRQ que se hayan disparado, entrara en STBY-I.
            // En el peor de los escenarios, si no encuentra respuesta con el maximo de intentos,
            // con el maximo de tiempo entre intentos, y con la maxima  carga de datos y
            // con la maxima velocidad (2Mbps) de envio, transcurriran menos de 60-70 milisegundos.
    
            // -Lo ponemos en modo STBY-I
            sendPulse(LOW);
    //
    // ***************************************************************
        
    
    
        // ***************************************************************
        //
        //  resetea los bits d TX_DS, RX_DR, MAX_RT para dejarlos disponibles
        //
        // ***************************************************************
        //
                buffer[0] |= B01110000;
                setDevice((WriteCmd | RF_STATUS), buffer, 1);
        //
        // ***************************************************************

        
        
    // ***************************************************************
    //
    //  Revisamos el estatus que nos dejo el disparo del IRQ y
    //  procedemos a darle el tratamiento apropiado.
    //
    // ***************************************************************
    //
    if ( st == 0x01 ) {
            // ***************************************************************
            //
            //  MAX_RT esta habilitado quiere decir que no se obtuvo
            //  respuesta del receptor y agoto todos los reintentos.
            //      -Vaciamos el buffer para dejarlo disponible.-
            //
                    getDevice((RF_FLUSH_TX), buffer, 0);
                    dataFlag = false;
                    return false;
            //
            // ***************************************************************
    }
    
    else if ( st == 0x02 ) {
            // ***************************************************************
            //
            //  TX_DS esta habilitado quiere decir que
            //  ya se realizo el envio y no hay datos de regreso.
            //
            // ***************************************************************
                    dataFlag = false;
                    return true;
            //
            // ***************************************************************
    }
    
    else if ( st == 0x06 ) {
            // ***************************************************************
            //
            //  TX_DS  &  RX_DR  estan habilitados, entonces quiere decir que
            //  ya se realizo el envio pero hay datos de regreso disponibles
            //
            // ***************************************************************
                    dataFlag = true;
                    return true;
            //
            // ***************************************************************
    }
    }
    // ***************************************************************
    //
    //  El RF se queda en STBY-I hasta que reciba nueva carga para enviar,
    //  y desactivamos el modo noAck (por si acaso estuviera activado)
    //
    // ***************************************************************
    sendPulse(LOW);
    noAck = false;
}
//
// ****************************************************************************

// ***************************************************************
//
// Estoy en modo PTX y si tengo un ACK con Payload tengo que
// obtener los datos de respuesta que vienen con el ACK (esto sucedera cuando
// la bandera (dataFlag) sea verdadera!.
//
// ***************************************************************
void RF24L01P::get_ackPayload(void* rxData, uint8_t packetSize)
{

    receive(rxData, 0, packetSize);
    
    // Solo se pueden contener 3 cargas de datos con ACK, si se
    // llega a ese limite, hay que liberar el RX_FIFO y tambien hay que
    // liberar el bit MAX_RT del IRQ para poder continuar con la exreaccion
    // de los datos!
    //
    // Esto sucede porque no se esta solicitando la extraccion de los
    // datos a tiempo!.  Dependiendo de la forma como se utilice, lo ideal
    // seria que cada que ve que hace un envio, se verifique si hay datos
    // de regreso para exreaerlos. De lo contrario, solamente nos dara el
    // ultimo dato que se haya quedado en la cola!.
    getDevice((RF_FLUSH_RX), buffer, 0);
    
    // resetea los bits d TX_DS, RX_DR, MAX_RT para dejarlos disponibles
    buffer[0] |= B01110000;
    setDevice((WriteCmd | RF_STATUS), buffer, 1);


    dataFlag = false;

}
//
// ****************************************************************************









// ****************************************************************************
//
//    PRX MODE
//
//    -Recepcion de datos (PRX mode)
//    -Envio de datos con ACK (Ack_PayLoad)
//
// Con la funcion: receive(,,,)
// devuelve verdadero si se recibe confrmacion del envio, y devuelve por parametro
// la cantidad de carga recibida asi como el remitente.
//
// Con la funcion: send_ackPayload(,,,)
// se realiza el envio de datos al remitente junto con la confirmacion (Ack)
//
//
//      POR HACER:
//              VALIDAR QUE EL TAMAÑO DEL PAQUETE NO EXCEDA LOS 32 BYTES
//
// ****************************************************************************
//
bool RF24L01P::receive(void* rxData, uint8_t* ptxSender, uint8_t packetSize)
{
    uint8_t st, stFifo;
    int8_t *pRxData = (int8_t*)rxData;

    // ***************************************************************
    //
    // Leemos el estatus del Registro (STATUS 0x07) y checamos si el
    // bit RX_DR esta encendido, indicando que hay paquetes recibidos.
    //
    // Tambien se monitorea el estatus del fifo (FIFO_STATUS 0x17) y checamos
    // si el bit RX_EMPTY esta apagado, indicando que aun hay datos en la cola
    //
    // ***************************************************************
    //
            st     = getDevice(ReadCmd | RF_STATUS,  buffer, 1);
            stFifo = getDevice(ReadCmd | RF_FIFO_ST, buffer, 1);
    //
    // ***************************************************************
    
    
    
    // ***************************************************************
    //
    // En el caso de que tengamos datos recibidos o pendientes en el
    // FIFO se extraen y se devuelven a los parametros de esta funcion.
    //
    // si no hay.... devolvemos falso!.
    //
    // ***************************************************************
    //
            // -si el status del IRQ esta "encendido" (RX_DR) entonces hay algun paquete
            // -si no, entonces checamos el bit "apagado" del FIFO (RX_EMPTY) para ver
            // si aun hay mas datos para devolverlos hasta vaciar el FIFO.
            if ( (st >> 4) || (!(stFifo & 0x1)) )
            {
                    // ejecuto la instruccion getFifoCount() para que devuelva
                    // la longitud de datos en el FIFO y la vacie en caso de que
                    // tenga "ruido" en los datos recibidos.
                    if ( getFifoCount() ) // control interno
                    {

                        // devuelve el remitente ( numero de pipe )
                        *ptxSender = (((getDevice(ReadCmd | RF_STATUS, buffer, 1)) >> 1) & 0xF) + 1;
        
            
                        //devuelve los datos
                        digitalWrite(CSNpin,LOW);
                        SPI.beginTransaction(nRF24);
                        SPI.transfer( RF_R_RX_PAYLOAD );
            
                            while ( packetSize-- ) *pRxData++ = SPI.transfer(0xff);
            
                        SPI.endTransaction();
                        digitalWrite(CSNpin,HIGH);
            

                        // Resetea los bits TX_DS, RX_DR y MAX_RT para dejarlos disponibles
                        getDevice(ReadCmd | RF_STATUS, buffer, 1);
                        buffer[0] |= B01110000;
                        setDevice((WriteCmd | RF_STATUS), buffer, 1);
                    
            
                        // regresa verdadero para salir
                        return true;
                    }
                
                    else return false;
            }
    //
    // ***************************************************************
    

        // si no es nada de lo anterior, regresa falso
        return false;
}
//
// ****************************************************************************
// ****************************************************************************
//
// Estoy en modo PRX y para enviar datos de regreso hay que cargar en el fifo
// los datos en el canal correcto (pipe) para que asi se pueda enviar junto
// con la confirmacion (ack).
//
// ****************************************************************************
//
void RF24L01P::send_ackPayload(const void* txData, uint8_t pipeChannel, uint8_t packetSize)
{
    const int8_t *pTxData = (const int8_t*)txData;
    
    // ***************************************************************
    //
    //    carga el FIFO con datos y manda el pulso para su envio.
    //
    // ***************************************************************
    
            // escribe los datos en el FIFO
            digitalWrite(CSNpin,LOW);
            SPI.beginTransaction(nRF24);
            SPI.transfer( (RF_W_ACK_PAYLOAD | pipeChannel) );
    
            while ( packetSize-- ) SPI.transfer(*pTxData++);
    
            SPI.endTransaction();
            digitalWrite(CSNpin,HIGH);
    
    //
    // ***************************************************************
}
//
// ****************************************************************************






// ****************************************************************************
//
//      Funcion para generar el pulso de envio de RadioFrecuencia
//
// ****************************************************************************
//
void RF24L01P::sendPulse(uint8_t value)
{
    if ( value == HIGH )
    {
        digitalWrite(CEpin, HIGH);
    }
    
    else //( value == LOW )
    {
        digitalWrite(CEpin, LOW);
    }
}
//
// ****************************************************************************







// ****************************************************************************
//
//      Procedimiento en desarrollo, para envios mayores a 32 bytes!!
//
//      Pretendo hacer fragmentos de paquetes de 32 bytes, de momento maximo
//      serian solo 8 paquetes, enviando en total 256 bytes!
//      Necesito un byte de control para que el remitente sepa cuantos bytes
//      espera recibir y hacer la confirmacion. Creo que seria algo como una
//      transmision en streaming. (aun ando viendo eso).
//
// ****************************************************************************
//
void RF24L01P::tx(const void *txData, uint8_t packetSize)
{
    
    uint8_t st, fragmentPacket, fifoCount = 0;
    const int8_t *pTxData = (const int8_t*)txData;
    
    fragmentPacket = ((packetSize + MAXFIFOSIZE - 1) >> 5);
    
    //memcpy(fifoBuff, (const int8_t *)txData, len);
    
    // manda el MicroPulso de envio para transmitir.
    // probablemente se quede en STBY-II en lo que se estan
    // enviando los paquetes en fragmentos (si es que son mas de uno).
    // De momento, maximo son 8 fragmentos de 32bytes cada uno = 256 bytes!
    sendPulse(HIGH);
    
    
    SPI.beginTransaction(nRF24);
    while ( fragmentPacket-- )
    {
        // manda el pulso bajo, para escribir la carga a transmitir
        //sendPulse(LOW);
        
        
        // escribe los datos en el FIFO
        digitalWrite(CSNpin,LOW);
        //SPI.beginTransaction(nRF24);
        //st = SPI.transfer( RF_W_TX_PAYLOAD );
        //SPI.transfer( RF_W_TX_PAYLOAD );
        SPI.transfer( RF_W_TX_PAYLOAD_NO_ACK );
        
        
        
        st = 0;
        while (true)
        {
            SPI.transfer(*(pTxData+fifoCount));
            fifoCount++; st++;
            if ( (fifoCount >= packetSize) || (st >= MAXFIFOSIZE) ) break;
        }
        
        
        
        //SPI.endTransaction();
        digitalWrite(CSNpin,HIGH);
        
        
        
        // manda el MicroPulso de envio para transmitir.
        //sendPulse(HIGH);
        //delayMicroseconds(10);
        
    }
    SPI.endTransaction();
    
    // manda el pulso bajo, para escribir la carga a transmitir
    sendPulse(LOW);
    
    
    
    /*
     Serial.print("tamaño del dato recibido: LEN: ");
     Serial.print(packetSize);     Serial.print("  pTxData: ");
     Serial.println(sizeof(&pTxData));
     */
}
//
// ****************************************************************************









// ****************************************************************************
//
//    Enhanced ShockBurst Dynamic
//
// ****************************************************************************
//
void RF24L01P::setEnhancedShockBurstDynamic()
{
    // ***************************************************************
    //
    // Habilita las caracteristicas del Enhanced ShockBurst(TM)
    // Aplicandolo a todos los canales (pipes) de forma general.
    // EN_DPL, EN_ACK_PAY y EN_DYN_ACK
    //
    // ***************************************************************
    //
            setDynamicFeature();
    //
    // ***************************************************************

    
    // ***************************************************************
    //
    // Esquema de CRC (Cyclic Redundancy Check) Robusto:
    // puede ser de 8 o 16 bits. Por default lo dejamos en 2 bytes.
    // Esto aplica para todas las transmisiones que se realizan.
    // -CRCO del registro CONFIG-
    //
    // ***************************************************************
    //
            setChecksum();
    //
    // ***************************************************************

    
    // ***************************************************************
    //
    // Modo de autoretransmision (ART) y Tiempo de Espera (ARD) entre cada intento.
    // este modo entra en funcion cuando no se recibe el auto-ACK. (SETUP_RETR)
    //
    // Los intentos (ART) van desde 1 hasta 15 veces.
    //
    // El tiempo de espera (ARD) entre cada intento van desde 250 microsegundos hasta
    // 4000 microsegundos en incrementos de 250 microsegundos.
    //
    // Los valores optimos segun la hoja de datos son:
    //      Para velocidades de 1Mbps o 2MBps = 500 Microsegundos (tiempo
    //      suficientemente largo para cualquier tamaño de datos)
    //
    //      Para velocidades de 250KBps = 1500 Microsegundos (tiempo
    //      suficientemente largo para cualquier tamaño de datos)
    //
    // Mas info -->> datasheet pagina 33 y 34, tabla 18.
    //
    //
    // Por default queda en 15 reintentos con 500 microseg entre reintentos.
    // tiempo suficiente para velocidades de (1Mbsp o 2Mbps) con cualquier
    // longitud de carga recibida.
    //
    // ***************************************************************
    //
            setAutoRetransmit();
    //
    // ***************************************************************

    
    // ***************************************************************
    //
    // Habilita la funcion de Auto Acknowledgment (EN_AA), caracteristica
    // principal del Enhanced ShockBurst™.
    // Por default se lo dejamos activado en todos los canales (Pipes)
    //
    // ***************************************************************
    //
            setAutoAckOnPipe();
    //
    // ***************************************************************

    
    // ***************************************************************
    //
    // Habilita la funcion de carga dinamica de datos (Dynamic Payload Length)
    // caracteristica principal del Enhanced ShockBurst™. (DYNPD)
    // Por default se lo dejamos activado en todos los canales (Pipes)
    //
    // ***************************************************************
    //
            setDynamicPayloadOnPipe();
    //
    // ***************************************************************

    
    // ***************************************************************
    //
    // Habilita la recepcion de direccion (RX Addresses) en cada canal
    // caracteristica principal del Enhanced ShockBurst™. (EN_RXADDR)
    // Por default se lo dejamos activado en todos los canales (Pipes)
    //
    // ***************************************************************
    //
            setRxAddressOnPipe();
    //
    // ***************************************************************
}
//
// ****************************************************************************


void RF24L01P::setChecksum(uint8_t value)
{
    // El CRC (Cyclic Redundancy Check) lo dejo preparado a 16 bits
    // para que cuando se habilite el (EN_AA) ya tengamos el valor listo
    getDevice((ReadCmd | RF_CONFIG), buffer, 1);
        if (value == 1)       bitWrite(buffer[0], RF_CRCO_BIT, 0);  // CRC =  8 bits
        else if (value == 2)  bitWrite(buffer[0], RF_CRCO_BIT, 1);  // CRC = 16 bits
        else                  bitWrite(buffer[0], RF_CRCO_BIT, 1);  // CRC = 16 bits
    setDevice((WriteCmd | RF_CONFIG), buffer, 1);
}
uint8_t RF24L01P::getChecksum()
{
    getDevice((ReadCmd | RF_CONFIG), buffer, 1);
    if ( bitRead((buffer[0]),RF_CRCO_BIT) ) return 2; return 1;
}


void RF24L01P::setDynamicFeature(bool value)
{
    getDevice((ReadCmd | RF_FEATURE), buffer, 1);
        if (value) buffer[0] |= B00000111;
        else       buffer[0] &= B11111000;
    setDevice((WriteCmd | RF_FEATURE), buffer, 1);

}
bool RF24L01P::getDynamicFeature()
{
    getDevice((ReadCmd | RF_FEATURE), buffer, 1);
    if (buffer[0] & 0x07) return true;
    return false;
}


void RF24L01P::setAutoAckOnPipe(uint8_t pipe, uint8_t turn)
{
    getDevice((ReadCmd | RF_EN_AA), buffer, 1);
        if      ( (pipe == PTXAll) && (turn) )   buffer[0] |= B00111111;
        else if ( (pipe == PTXAll) && (!turn) )  buffer[0] &= B11000000;
        else if ( (pipe >= 0) && (pipe <= 5) )   bitWrite(buffer[0], pipe, turn);
        else buffer[0] |= B00111111;
    setDevice((WriteCmd | RF_EN_AA), buffer, 1);
}


void RF24L01P::setDynamicPayloadOnPipe(uint8_t pipe, uint8_t turn)
{
    getDevice((ReadCmd | RF_DYNPD), buffer, 1);
        if      ( (pipe == PTXAll) && (turn) )   buffer[0] |= B00111111;
        else if ( (pipe == PTXAll) && (!turn) )  buffer[0] &= B11000000;
        else if ( (pipe >= 0) && (pipe <= 5) )   bitWrite(buffer[0], pipe, turn);
        else buffer[0] |= B00111111;
    setDevice((WriteCmd | RF_DYNPD), buffer, 1);
    
    // segun la hoja de datos, cuando se habilita este caracteristica
    // debe estar habilitada la caracteristica de Auto ACK (EN_AA) en su
    // respectivo Pipe. Asi que simplemente llamo la funcion con
    // los mismos parametros que aqui. Tambien, de forma automatica, el bit
    // del CRC se activa al activar esta caracteristica, el cual ya lo
    // tenemos configurado a 16 bits (2 bytes).
    setAutoAckOnPipe(pipe, turn);
}


void RF24L01P::setRxAddressOnPipe(uint8_t pipe, uint8_t turn)
{
    // habilta la direccion del canal por el que se va a trans/reciv.
    getDevice((ReadCmd | RF_EN_RXADDR), buffer, 1);
        if      ( (pipe == PTXAll) && (turn) )   buffer[0] |= B00111111;
        else if ( (pipe == PTXAll) && (!turn) )  buffer[0] &= B11000000;
        else if ( (pipe >= 0) && (pipe <= 5) )   bitWrite(buffer[0], pipe, turn);
        else buffer[0] |= B00111111;
    setDevice((WriteCmd | RF_EN_RXADDR), buffer, 1);
}


void RF24L01P::setAutoRetransmit(uint8_t tries, uint16_t timelapsed)
{
    // Se le indica cuantas veces debe re-intentar y el tiempo de espera entre cada intento
    // si la velocidad es de 250KBps, el tiempo maximo sera de 1500 Microsegundos)
    timelapsed = (getRate() == DATARATE_250Kbps && timelapsed < 1500 ? 1500:timelapsed);
    //timelapsed = (getRate() == DATARATE_250Kbps ? 1500:timelapsed);
    timelapsed = timelapsed >> 8;
    
    buffer[0] = ((timelapsed << 4) | tries); setDevice((WriteCmd | RF_SETUP_RETR), buffer, 1);
}
//
// ****************************************************************************






// ****************************************************************************
//
//    Tamaño de carga de datos recibidos (PLD - PayLoad Lenght)
//
// ****************************************************************************
//
uint8_t RF24L01P::getFifoCount()
{
    // ***************************************************************
    //
    // Lee la longitud del paquete, si es mayor a 32 se descarta
    // porque su contenido es erroneo, es ruido, usamos el comando
    // (FLUSH_RX 0xE1). Maximo deben ser 32 bytes.
    //
    // ***************************************************************
    //
            getDevice(RF_R_RX_PL_WID, buffer, 1);
            if (buffer[0] > 32) getDevice(RF_FLUSH_RX, buffer, 0);
    
            return buffer[0];
    //
    // ***************************************************************

}
//
// ****************************************************************************






// ****************************************************************************
//
//    Registro OBSERVE_TX, para el conteo de paquetes perdidos y reintentos
//          hay que tomar en cuenta que estos valores son transitorios,
//          es decir, se resetean bajo ciertas condiciones durante la
//          transmision de datos.    Datasheet -->> pag 75.
//
// ****************************************************************************
//
uint8_t RF24L01P::getLostPackets()
{
    return (getDevice((ReadCmd | OBSERVE_TX), buffer, 1) >> 4);
}

uint8_t RF24L01P::getRetriesPackets()
{
    return (getDevice((ReadCmd | OBSERVE_TX), buffer, 1) & 0x0F);
}
//
// ****************************************************************************







// ****************************************************************************
//
//    Asignacion de Direccion (Address) del Modulo RF de forma manual
//
// ****************************************************************************
//
void RF24L01P::setDeviceAddress(uint8_t Addr1, uint8_t Addr2, uint8_t Addr3, uint8_t Addr4, uint8_t Addr5)
{
    // ***************************************************************
    //
    // Almacenamos la direccion proporcionada en una variable privada.
    //
    // Por default, desde la inicializacion, tiene asignada la direccion:
    //          "120,120,120,120,120"
    //
    // ***************************************************************
    //
            devAddress[0] = Addr1; devAddress[1] = Addr2;
            devAddress[2] = Addr3; devAddress[3] = Addr4;
            devAddress[4] = Addr5;
    //
    // ***************************************************************
    
}
//
// ****************************************************************************






// ****************************************************************************
//
//    Inicializador de la direccion (Address) del modulo RF por default.
//
// ****************************************************************************
//
void RF24L01P::initDeviceAddress()
{
    // ***************************************************************
    //
    // Confirguramos la longitud de la direccion (SETUP_AW  0x03) a 40bits.
    // A aprtir de esta direccion, se asignan las direcciones al resto de
    // los canales (pipes) y debe ser unica para cada canal.
    //
    // Por default lo dejamos configurado con longitud de 5 bytes (40bits).
    // y con la direccion "120,120,120,120,120".
    //
    // ***************************************************************
    //
            getDevice((ReadCmd | RF_SETUP_AW), buffer, 1);
            buffer[0] |= B00000011;
            setDevice((WriteCmd | RF_SETUP_AW), buffer, 1);
    //
    // ***************************************************************

            devAddress[0] = 120;
            devAddress[1] = 120;
            devAddress[2] = 120;
            devAddress[3] = 120;
            devAddress[4] = 120;
    //
    // ***************************************************************
    
    // ****************************************************************************
    //
    // El direccionamiento final queda asi; -con la direccion: "120,120,120,120,120"-
    //
    //          RX_ADDR_P0 = 120:120:120:120:120 = PTX01
    //          RX_ADDR_P1 = 120:120:120:120:190 = PTX02
    //          RX_ADDR_P2 = 120:120:120:120:189 = PTX03
    //          RX_ADDR_P3 = 120:120:120:120:188 = PTX04
    //          RX_ADDR_P4 = 120:120:120:120:187 = PTX05
    //          RX_ADDR_P5 = 120:120:120:120:186 = PTX06
    //
    //  *"PTX??"*  es el "alias" del nodo, donde "??" es numero del canal (Pipe).
    //
    // ****************************************************************************
    
}
//
// ***************************************************************






// ****************************************************************************
//
//    Seleccion de Canal de comunicacion Logica (Pipe)
//
// ****************************************************************************
//
void RF24L01P::setPipeChannel(uint8_t Pipe)
{

    if ( Pipe == PTX01 )
    {
            // ***************************************************************
            //
            // Canal 01 = PIPE0 = RX_ADDR_P0
            //
            // ***************************************************************
            //
                    memcpy(buffer, devAddress, 5);
                    setDevice((WriteCmd | RF_TX_ADDR), buffer, 5);
        
                    getDevice((ReadCmd  | RF_TX_ADDR),   buffer, 5);
                    setDevice((WriteCmd | RF_RX_ADDR_P0),buffer, 5);
            //
            // ***************************************************************
    }
    
    else if ( Pipe == PTX02 )
    {
            // ***************************************************************
            //
            // Canal 02 = PIPE1 = RX_ADDR_P1
            //
            // ***************************************************************
            //
                    memcpy(buffer, devAddress, 5);
                    buffer[0] = (devAddress[0] ^ 0xC6);
                    setDevice((WriteCmd | RF_TX_ADDR), buffer, 5);

                    getDevice((ReadCmd  | RF_TX_ADDR),   buffer, 5);
                    setDevice((WriteCmd | RF_RX_ADDR_P0),buffer, 5);
            //
            // ***************************************************************
    }
    
    else if ( Pipe == PTX03 )
    {
            // ***************************************************************
            //
            // Canal 03 = PIPE2 = RX_ADDR_P2
            //
            // ***************************************************************
            //
                    memcpy(buffer, devAddress, 5);
                    buffer[0] = (devAddress[0] ^ 0xC5);
                    setDevice((WriteCmd | RF_TX_ADDR), buffer, 5);

                    getDevice((ReadCmd  | RF_TX_ADDR),   buffer, 5);
                    setDevice((WriteCmd | RF_RX_ADDR_P0),buffer, 5);
            //
            // ***************************************************************
    }
    
    else if ( Pipe == PTX04 )
    {
            // ***************************************************************
            //
            // Canal 04 = PIPE3 = RX_ADDR_P3
            //
            // ***************************************************************
            //
                    memcpy(buffer, devAddress, 5);
                    buffer[0] = (devAddress[0] ^ 0xC4);
                    setDevice((WriteCmd | RF_TX_ADDR), buffer, 5);

                    getDevice((ReadCmd  | RF_TX_ADDR),   buffer, 5);
                    setDevice((WriteCmd | RF_RX_ADDR_P0),buffer, 5);
           //
            // ***************************************************************
    }
    
    else if ( Pipe == PTX05 )
    {
            // ***************************************************************
            //
            // Canal 05 = PIPE4 = RX_ADDR_P4
            //
            // ***************************************************************
            //
                    memcpy(buffer, devAddress, 5);
                    buffer[0] = (devAddress[0] ^ 0xC3);
                    setDevice((WriteCmd | RF_TX_ADDR), buffer, 5);

                    getDevice((ReadCmd  | RF_TX_ADDR),   buffer, 5);
                    setDevice((WriteCmd | RF_RX_ADDR_P0),buffer, 5);
            //
            // ***************************************************************
    }
    
    else if ( Pipe == PTX06 )
    {
            // ***************************************************************
            //
            // Canal 06 = PIPE5 = RX_ADDR_P5
            //
            // ***************************************************************
            //
                    memcpy(buffer, devAddress, 5);
                    buffer[0] = (devAddress[0] ^ 0xC2);
                    setDevice((WriteCmd | RF_TX_ADDR), buffer, 5);

                    getDevice((ReadCmd  | RF_TX_ADDR),   buffer, 5);
                    setDevice((WriteCmd | RF_RX_ADDR_P0),buffer, 5);
            //
            // ***************************************************************
    }
    
    else if ( Pipe == PRXMode )
    {
            // ***************************************************************
            //
            // Todos los canales se configuran con las caracteristicas del
            // Enhanced ShockBurst™
            //
            // Dejamos asignada las direcciones de cada canal en modo PRX,
            // de esta forma esta listo para recibir en los 6 canales.
            //
            //
            // ***************************************************************
            //
                    setRxAddressOnPipe();
            
                    setDynamicPayloadOnPipe();
            
                    setAutoAckOnPipe();

                    memcpy(buffer, devAddress, 5);
                    setDevice((WriteCmd | RF_RX_ADDR_P0),buffer, 5);
        
                    memcpy(buffer, devAddress, 5);
                    buffer[0] = (devAddress[0] ^ 0xC6);
                    setDevice((WriteCmd | RF_RX_ADDR_P1), buffer, 5);
        
                    buffer[0] = (devAddress[4] ^ 0xC5); setDevice((WriteCmd | RF_RX_ADDR_P2), buffer, 1);
                    buffer[0] = (devAddress[4] ^ 0xC4); setDevice((WriteCmd | RF_RX_ADDR_P3), buffer, 1);
                    buffer[0] = (devAddress[4] ^ 0xC3); setDevice((WriteCmd | RF_RX_ADDR_P4), buffer, 1);
                    buffer[0] = (devAddress[4] ^ 0xC2); setDevice((WriteCmd | RF_RX_ADDR_P5), buffer, 1);
            //
            // ***************************************************************
    }
}
//
// ****************************************************************************






// ****************************************************************************
//
//    Seleccion de Potencia de transmision del Modulo RF
//
// ****************************************************************************
//
void RF24L01P::setTXLevel(uint8_t value)
{
    getDevice((ReadCmd | RF_SETUP), buffer, 1);
    
    if      ( value == TX_LEVEL_00dBm ) {
            bitWrite(buffer[0], RF_PWR_LOW_BIT,  1);
            bitWrite(buffer[0], RF_PWR_HIGH_BIT, 1); }
    
    else if (value == TX_LEVEL_06dBm ) {
            bitWrite(buffer[0], RF_PWR_LOW_BIT,  1);
            bitWrite(buffer[0], RF_PWR_HIGH_BIT, 0); }
    
    else if ( value == TX_LEVEL_12dBm ) {
            bitWrite(buffer[0], RF_PWR_LOW_BIT,  0);
            bitWrite(buffer[0], RF_PWR_HIGH_BIT, 1); }
    
    else if ( value == TX_LEVEL_18dBm ) {
            bitWrite(buffer[0], RF_PWR_LOW_BIT,  0);
            bitWrite(buffer[0], RF_PWR_HIGH_BIT, 0); }
    
    else    {
            bitWrite(buffer[0], RF_PWR_LOW_BIT,  1);
            bitWrite(buffer[0], RF_PWR_HIGH_BIT, 1); }
    
    setDevice((WriteCmd | RF_SETUP), buffer, 1);
}
//
// ****************************************************************************
uint8_t RF24L01P::getTXLevel()
{
    getDevice(((ReadCmd | RF_SETUP) ), buffer, 1);
    return buffer[0] & TX_LEVEL_MASK;
}
//
// ****************************************************************************






// ****************************************************************************
//
//    Detector de potencia de señal del canal en uso (modo RX)
//
// ****************************************************************************
//
uint8_t RF24L01P::getReceivedPowerDetector()
{
    // ***************************************************************
    //
    // Received Power Detector, tambien llamado anteriormente como Carrier Detect.
    // Devuelve la potencia de la señal detectada cuando esta en modo RX en el
    // bit 0 del registro 0x09. Es decir, nos ofrece una instantanea del nivel
    // de potencia de recepcion que tiene el canal en ese momento.  Se dispara cuando
    // detecta una fuerza de señal por encima de los -64dBm del canal en uso.
    //
    // ***************************************************************
    //
            // verifica que este en modo RX (PRIM_RX = 1)
            getDevice((ReadCmd | RF_CONFIG), buffer, 1);
            if ( buffer[0] & 0x01 )
            {
                getDevice((ReadCmd | RX_RPD), buffer, 1);
                return bitRead(buffer[0], 0);
            }
            else
                return 0;
    //
    // ***************************************************************
}
//
// ****************************************************************************






// ****************************************************************************
//
//    Seleccion de la Frecuencia (GHz) de transmision del Modulo RF
//
// ****************************************************************************
//
void RF24L01P::setRadioFrequency(uint8_t value)
{
    // ***************************************************************
    //
    // La frecuencia de canal del RF, determina el centro del canal que usa el nRF24L01+.
    // A velocidades de 1Mbps o 250Kbps el ancho de banda es de 1MHz
    // A velocidades de 2Mbps el ancho de banda es de 2MHz
    // La resolucion del canal es de 1MHz, asi que para velocidades de
    // 2Mbps hay que dejar un espacio entre canales de 2MHz o mas!
    //
    // El RF opera en el rango de 2.400MHz hasta 2.525MHz. (2.4GHz - 2.52GHz)
    //
    // Entonces para velocidades de 1Mbps o 250Kbps tenemos 125 canales (1MHz de resolucion por canal)
    // Y para velocidades de 2Mbps tenemos 41 canales con 3MHz de resolucion por canal (yo estoy
    // dejando 3MHz de resolucion para 2Mbps, para evitar un "overlaping").
    // Formula de la hoja de datos:
    //
    //                              F0 = 2400 + RF_CH [MHz]
    //
    // ***************************************************************
    //
            bool DataRate_2Mbps = (getRate() == DATARATE_2Mbps ? 1:0);
    
            if ( DataRate_2Mbps && (value >= RF_MaxChannels_2Mbps) ) value = RF_MaxChannels_2Mbps;
            if ( DataRate_2Mbps && (value <= 0) ) value = 1;
            if ( DataRate_2Mbps ) value += (value << 1);
    
            if ( !DataRate_2Mbps && (value > RF_MaxChannels_1Mbps) ) value = RF_MaxChannels_1Mbps;
            if ( !DataRate_2Mbps && (value <= 0) ) value = 1;
    
            buffer[0] = value;
            setDevice((WriteCmd | RF_CHANNEL), buffer, 1);
    //
    // ***************************************************************

}
//
// ****************************************************************************
uint8_t RF24L01P::getRadioFrequency()
{
    getDevice((ReadCmd | RF_CHANNEL), buffer, 1);
    
    return buffer[0];
}
//
// ****************************************************************************







// ****************************************************************************
//
//    Seleccion de velocidad de datos en el aire del Modulo RF
//
// ****************************************************************************
//
void RF24L01P::setRate(uint8_t value)
{
    getDevice((ReadCmd | RF_SETUP), buffer, 1);
    
    if      ( value == DATARATE_250Kbps ) {
            bitWrite(buffer[0], RF_DR_LOW_BIT,  1);
            bitWrite(buffer[0], RF_DR_HIGH_BIT, 0); }
    
    else if ( value == DATARATE_1Mbps ) {
            bitWrite(buffer[0], RF_DR_LOW_BIT,  0);
            bitWrite(buffer[0], RF_DR_HIGH_BIT, 0); }
    
    else if ( value == DATARATE_2Mbps ) {
            bitWrite(buffer[0], RF_DR_LOW_BIT,  0);
            bitWrite(buffer[0], RF_DR_HIGH_BIT, 1); }
    
    else    { // data rate = 2Mbps
            bitWrite(buffer[0], RF_DR_LOW_BIT,  0);
            bitWrite(buffer[0], RF_DR_HIGH_BIT, 1); }
    
    setDevice((WriteCmd | RF_SETUP), buffer, 1);
}
//
// ****************************************************************************
uint8_t RF24L01P::getRate()
{
    getDevice((ReadCmd | RF_SETUP), buffer, 1);
    return buffer[0] & DATARATE_MASK;
}
//
// ****************************************************************************







// ****************************************************************************
//
//    Modos de inicializacion, segun corresponda! (en el void Setup del sketch)
//    (PTX mode) o (PRX mode).
//
// ****************************************************************************
//
void RF24L01P::setPowerDown()
{
    // ***************************************************************
    //
    // PowerDown, utilizado para escribir/cambiar la configuracion del SETUP
    //
    // ***************************************************************
    //
            getDevice((ReadCmd | RF_CONFIG), buffer, 1);
            bitWrite(buffer[0], RF_PWR_UP_BIT, 0);
            setDevice((WriteCmd | RF_CONFIG), buffer, 1);
            sendPulse(LOW);
    //
    // ***************************************************************
}
void RF24L01P::setPTXmode(uint8_t channel)
{
    // ***************************************************************
    //
    // PTXMode, para poner el dispositivo en modo transmision
    //
    // ***************************************************************
    //
            // Selecciona el canal en que va a operar (pipe)
            setPipeChannel(channel);
    
            // configura en modo PTX
            getDevice((ReadCmd | RF_CONFIG), buffer, 1);
            bitWrite(buffer[0], RF_PWR_UP_BIT,  1);
            bitWrite(buffer[0], RF_PRIM_RX_BIT, 0);
            setDevice((WriteCmd | RF_CONFIG), buffer, 1);

            // resetea los bits TX_DS, RX_DR, MAX_RT para dejarlos disponibles
            getDevice((ReadCmd | RF_STATUS), buffer, 1);
            buffer[0] |= B01110000;
            setDevice((WriteCmd | RF_STATUS), buffer, 1);

            // listo para "despertar" del modo powerdown
            sendPulse(HIGH);
    //
    // ***************************************************************
}
void RF24L01P::setPRXmode()
{
    // ***************************************************************
    //
    // PRXMode, para poner el dispositivo en modo recepcion
    //
    // ***************************************************************
    //
            // Selecciona los ajustes para modo PRX
            setPipeChannel(PRXMode);

            // configura en modo PRX
            getDevice((ReadCmd | RF_CONFIG), buffer, 1);
            bitWrite(buffer[0], RF_PWR_UP_BIT,  1);
            bitWrite(buffer[0], RF_PRIM_RX_BIT, 1);
            setDevice((WriteCmd | RF_CONFIG), buffer, 1);

            // resetea los bits TX_DS, RX_DR, MAX_RT para dejarlos disponibles
            getDevice((ReadCmd | RF_STATUS), buffer, 1);
            buffer[0] |= B01110000;
            setDevice((WriteCmd | RF_STATUS), buffer, 1);

            // listo para "despertar" del modo powerdown y desde
            // este momento, el RF esta monitoreando el aire.
            sendPulse(HIGH);
    //
    // ***************************************************************
}
//
// ****************************************************************************








// ****************************************************************************
//
//    Comandos SPI de Acceso al Modulo RF
//
//      Son dos funciones hechas a conveniencia para facilitarme y "automatizar"
//      el proceso de estar leyendo y escribiendo en el bus SPI
//
// ****************************************************************************
//
uint8_t RF24L01P::setDevice(uint8_t regAddr, uint8_t* pBuff, uint8_t len)
{
    digitalWrite(CSNpin, LOW);
    SPI.beginTransaction(nRF24);
    SPI.transfer(regAddr);
    for (uint8_t i = 0; i < len; i++) { SPI.transfer((uint8_t) pBuff[i]); }
    SPI.endTransaction();
    digitalWrite(CSNpin, HIGH);
    return pBuff[0];
}
uint8_t RF24L01P::getDevice(uint8_t regAddr, uint8_t* pBuff, uint8_t len)
{
    digitalWrite(CSNpin, LOW);
    SPI.beginTransaction(nRF24);
    SPI.transfer(regAddr);
    for (uint8_t i = 0; i < len; i++) { pBuff[i] = SPI.transfer(0x00); }
    SPI.endTransaction();
    digitalWrite(CSNpin, HIGH);
    return pBuff[0];
}
//
// ****************************************************************************











