/*
 *
 *  Nordic Semiconductor nRF24L01+
 *
 *  NRF24L01P.cpp
 *
 *  Creado: 25 Ago 2016 por jgarcia1112@gmail.com
 *  Villahermosa, Tabasco. Mexico
 *
 *  Modificado: 05 Jul 2017 por jgarcia1112@gmail.com
 *
 *  Version 2.0 - Este codigo es de dominio publico y sin ninguna garantia
 *
 *
 */



#include <SPI.h>
#include "NRF24L01P.h"




/*
 *
 *  constructor de clase.
 *
 *  inicia la clase y establece el pin de control CE del dispositivo
 *
 *  en este modo no se maneja el pin de Interupcion (IRQ) del dispositivo
 *
 *  parametros: 
 *  
 *          cepin   -> Chip Enable Pin
 *
 *  devuelve: 
 *
 *          un objeto clase enlazado el dispositivo nrf24l01+ usando el pin de CE
 *
 */
nRF24L01P::nRF24L01P(uint8_t cepin) {
    
    // establece el pin CE (Chip Enable) en modo OUTPUT
    
    CEpin = cepin;
    
    pinMode(CEpin,  OUTPUT);

    /*
     *
     *  al iniciar el SPI, el pin CSN (SS) lo establece en modo OUTPUT por que es parte
     *
     *  del los 4 cables del BUS SPI (SS, MOSI, MISO, SCK)
     *
     */

}


/*
 *
 *  inicializa el dispositivo (encendido) con valores por default.
 *
 *  parametros:
 *  
 *          ninguno
 *
 *  devuelve: 
 *
 *          nada
 *
 */
void nRF24L01P::begin() {
    
    // inicializa el dispotivo con valores predefinidos por default
    
    SPI.begin();
    
    // realiza el encendido del dispositivo
    
    powerUp();
    
    // activa la funcion de Enhanced ShockBurst (ESB)
    
    setFeatures();
    
}


/*
 *
 *  funcion para establecer el modo PTX (transmisor) pasando por parametro el canal por el que se va a transmitir.
 *
 *  permanece en modo standby-I, a menos que se tenga la opcion de enableLiveStream().
 *
 *  utilizar standby() para detener toda la actividad del modulo y dejarlo en reposo.
 *
 */
void nRF24L01P::startTx(uint8_t logicChannel, uint8_t* _address, uint8_t _addressWidth) {

    setAddrWidth(_addressWidth - 2);

    spiWrite(W_REGISTER | TX_ADDR,    _address, _addressWidth, _addressWidth);

    spiWrite(W_REGISTER | RX_ADDR_P0, _address, _addressWidth, _addressWidth);

    SET_PTX_MODE_DEVICE;

}


/*
 *
 *  establece el modo PRX (receptor)
 *
 *
 */
void nRF24L01P::startRx(uint8_t* rxAddresses, uint8_t _addressWidth) {
    
    //  asigna las direcciones a los canales del dispositivo
    
    setRxAddresses(rxAddresses, _addressWidth);

    //  establece el dispositivo en modo PRX (recepcion)
    
    SET_PRX_MODE_DEVICE;
    
    //  comienza a monitorear el ambiente en espera de recibir datos
    
    SET_CHIP_ENABLE;
    
}


/*
 *
 *  establece las direcciones de los canales logicos (pipe)
 *
 *  en modo receptor (PRX)
 *
 */
void nRF24L01P::setRxAddresses(uint8_t* rxAddresses, uint8_t _addressWidth) {

    setAddrWidth(_addressWidth - 2);
    
    spiWrite(W_REGISTER | RX_ADDR_P0, rxAddresses, _addressWidth, (CHANNEL1 * _addressWidth) + _addressWidth);
    
    spiWrite(W_REGISTER | RX_ADDR_P1, rxAddresses, _addressWidth, (CHANNEL2 * _addressWidth) + _addressWidth);
    
    _addressWidth *= 2;

    spiWrite(W_REGISTER | RX_ADDR_P2, rxAddresses, 1, _addressWidth + 1);
    
    spiWrite(W_REGISTER | RX_ADDR_P3, rxAddresses, 1, _addressWidth + 2);
    
    spiWrite(W_REGISTER | RX_ADDR_P4, rxAddresses, 1, _addressWidth + 3);
    
    spiWrite(W_REGISTER | RX_ADDR_P5, rxAddresses, 1, _addressWidth + 4);
    
}


/*
 *
 *  detiene el modo PRX (receptor) y PTX (si fuera el caso) y permanece en standby-I
 *
 */
void nRF24L01P::standby() {

    RST_CHIP_ENABLE;
    
}


/*
 *
 *  inicializa el modo ESB (Enhanced ShockBurst) en todos los puertos (pipes) -por default-.
 *
 *  inicializa el Dynamic Payload Length en todos los puertos (pipes).
 *
 *  tambien inicializa las funciones mejoradas (FEATURE) del dispositivo:
 *
 *      - Dynamic Payload Length
 *
 *      - Payload with ACK
 *
 *      - Enables the W_TX_PAYLOAD_NOACK command
 *
 */
void nRF24L01P::setFeatures() {
    
    // habilita las funciones mejoradas del dispositivo
    
    SET_FEATURE_REGISTER();

    // modo ESB (enable auto acknowledgement) -por default ya viene habilitado-
    
    //SET_ESB_PORTS();
    
    // modo de retransmision y tiempo de espera
    
    SET_AUTO_RTX();
    
    // carga dinamica (enable dynamic payload length)
    
    SET_DPL_PORTS();
    
    // habilita los puertos de los canales logicos (pipe) para recepcion
    
    SET_RX_PORTS(LOGIC_CHANNEL_PORTS_ALL);

}


/*
 *
 *  inicializa las funciones mejoradas del dispositivo.
 *
 *  establece la carga dinamica, y el modo ESB
 *
 *  parametros:
 *          
 *          ninguno
 *
 *  devuelve:
 *
 *          nada
 *
 *  habilita las funciones mejoradas del dispositivo.
 *
 */
void nRF24L01P::resetFeatures() {
    
    // deshabilita las funciones mejoradas del dispositivo
    
    RST_FEATURE_REGISTER();
    
    // modo ESB (enable auto acknowledgement)
    
    //RST_ESB_PORTS();
    
    // modo de retransmision y tiempo de espera
    
    RST_AUTO_RTX();

    // carga dinamica (enable dynamic payload length)
    
    RST_DPL_PORTS();
    
    // deshabilita los puertos de los canales logicos (pipe) para recepcion
    
    RST_RX_PORTS();
}


/*
 *
 *  funcion para realizar el encendido del dispositivo.
 *
 */
void nRF24L01P::powerUp() {
    
    //  instruccion para encender el dispositivo
    
    SET_POWER_UP_DEVICE;

    //  esperamos un momento para que el modulo termine de encender! (tiempo Maximo)
    
    delayMicroseconds(4500);

}


/*
 *
 *  funcion para realizar el apagado del dispositivo.
 *
 */
void nRF24L01P::powerDown() {
    
    // simplemente, lo apaga ....  y ya!
    
    SET_POWER_DOWN_DEVICE;

}


/*
 *
 *  inicializa y establece la frecuencia de comunicacion del radio en el dispositivo.
 *
 *  establece la frecuencia, la potencia de transmision, y la velocidad de datos
 *
 *  parametros:
 *
 *          radioFrequency  -> frecuencia de operacion del modulo (desde 0 hasta 126 = (2400Mhz hasta 2528Mhz)
 *
 *          outputPower     -> potencia de transmision: 0dBm, -6dBm, -12dBm y -18dBm
 *
 *          radioDataRate   -> velocidad de datos en aire: 256Kbps, 1Mbps y 2Mbps
 *
 *          crcMode         -> modo de verificacion de error: 1 byte, 2 bytes o deshabilitado
 *
 *  devuelve:
 *
 *          nada
 *
 *
 */
void nRF24L01P::setRadioLayer(uint8_t radioFrequency, uint8_t outputPower, uint8_t radioDataRate, uint8_t crcMode) {
    
    SET_RF_RADIO(radioFrequency);
    
    SET_RF_OUTPUT_POWER(outputPower);
    
    SET_RF_DATARATE(radioDataRate);
    
    SET_CRC(crcMode);
    
}


/*
 *
 *  realiza la peticion del valor de un registro del mapa de registros.
 *
 *  se le pasan por parametros, la instruccion o la direccion del registro, 
 *  
 *  y el valor que se le va a escribir o leer al registro.
 *
 *  siempre devuelve dos bytes (16 bits):
 *
 *      el primer byte es el status, (highByte)
 *
 *      el segundo byte el valor del registro solicitado (si lo hubiera). (lowByte)
 *
 */
uint8_t nRF24L01P::RQST_TO_DEVICE(uint8_t _reg, uint8_t _data) {
    
    SET_CHIP_SELECT;
    
        SET_SPI_COMMAND(_reg);

        uint8_t val = SPI.transfer(_data);
    
    RST_CHIP_SELECT;
    
    return val;

}


/*
 *
 *  funcion para realizar la lectura de uno o multiples bytes por el BUS SPI.
 *
 *  utiliza tres parametros, el primero es la instruccion del registro
 *
 *  el segundo es un buffer donde devuelve el valor leido, 
 *
 *  y el tercer parametro es la cantidad de bytes que se van a leer.
 *
 *  devuelve el valor en el segundo parametro [ pBuffer ]
 *
 */
void nRF24L01P::spiRead(uint8_t cmd, int8_t* pBuffer, uint8_t lenBuffer) {
    
    SET_CHIP_SELECT;
    
    SET_SPI_COMMAND(cmd);

    while (lenBuffer-- > 0)
        
         pBuffer[lenBuffer] = SPI.transfer(NOP);

    RST_CHIP_SELECT;
        
}


/*
 *
 *  funcion para realizar la escritura de uno o multiples bytes por el BUS SPI.
 *
 *  utiliza tres parametros, el primero es la instruccion del registro
 *
 *  el segundo es un buffer donde viene el valor que se va a escribir,
 *
 *  y el tercer parametro es la cantidad de bytes que se van a escribir.
 *
 *  no devuelve ningun valor.
 *
 */
void nRF24L01P::spiWrite(uint8_t cmd, const int8_t* pBuffer, uint8_t lenBuffer, uint8_t start) {
    
    SET_CHIP_SELECT;
    
    SET_SPI_COMMAND(cmd);

    while (lenBuffer-- > 0)

        SPI.transfer((uint8_t)pBuffer[--start] );

    RST_CHIP_SELECT;
    
}


/*
 *
 *  funcion para realizar la lectura de la cantidad de bytes que se encuentran
 *  
 *  en el fifo RX, se utiliza para en el modo Dynamic Payload Length.
 *
 *  parametros: ninguno.
 *
 *  devuelve: el total de bytes recibdos en el FIFO o 0 (cero) si no hay datos, o hay error en la recepcion.
 *
 */
uint8_t nRF24L01P::GET_PAYLOAD_SIZE() {
    
    uint8_t size = RQST_TO_DEVICE(R_RX_PL_WID);
    
    if (size > MAX_LEN_FIFO_SIZE) {
        
        CLEAR_RX_FIFO();
        
        CLEAR_IRQ(DATA_READY);

        return 0;
        
    }

    return size;
}


/*
 *
 *  funcion para monitorear el estatus de los datos presentes en el aire en modo PRX.
 *
 *  en cuanto recibe un paquete valido devuelve la cantidad de bytes recibidos o
 *
 *  falso si los FIFOs se encuentran vacios.
 *
 *  util e indispensable para extraer los datos del FIFO hasta que se vacie por completo.
 *
 *  cuando el dispositivo esta en modo PTX sirve para extraer los datos que vienen adjuntos
 *
 *  a la confirmacion de transmision (ACK).
 *
 *  parametros: ninguno.
 *
 *  devuelve: la cantidad de bytes en el fifo (verdadero) o falso (cero) si no hay datos recibidos
 *
 */
bool nRF24L01P::dataReady() {

    // realiza una lectura para actualizar la variable devInfo.status
    
    // ademas, por alguna razon, a la hora de hacer la descarga, se estabiliza el tiempo de
    
    // descarga (normalmente entre 72 y 80 microSegundos)

    RQST_TO_DEVICE(NOP, NOP);

    if ( GET_SENDER_PORT() < 0x0C )
        
        return true;
    
    return false;
    
}


/*
 *
 *
 */
bool nRF24L01P::dataSent() {

    RQST_TO_DEVICE(NOP, NOP);
    
    return DATA_IS_SENT;

}


/*
 *
 *
 */
bool nRF24L01P::lostConnection() {

    RQST_TO_DEVICE(NOP, NOP);

    return LOST_CONNECTION;

}


/*
 *
 *
 */
void nRF24L01P::enableLiveStream() {
    
    SET_CHIP_ENABLE;

}


/*
 *
 *
 */
void nRF24L01P::disableLiveStream() {
    
    RST_CHIP_ENABLE;

}


/*
 *
 *
 */
void nRF24L01P::sendData(void* buffer, uint8_t lenBuffer, uint8_t cmd) {

    uint8_t *pTxData = (int8_t*)buffer;
    
    //  si el FIFO esta lleno, se limpia para permiter el envio de datos
    
    //  le damos prioridad a vaciar el FIFO para que el envio salga lo mas pronto posible
    
    //  lo que se haya vaciado contara como paquete perdido, lo cual muy probablemente sea por
    
    //  haber alcanzado el maximo de reintentos que maneja el ESB.
    
    CLEAR_IRQ(IRQ_MASK);

    if (TX_FIFO_IS_FULL) CLEAR_TX_FIFO();

    spiWrite(cmd, pTxData, lenBuffer, lenBuffer);

    /*if (IS_LIVE_STREAM() == false)*/ SEND_PULSE(11);

    //  esperamos respuesta del estatus de la transmision realizada
    
    //  (la espera debe ser de apenas unos cuantos microsegundos)
    
    //  -esta pausa es necesaria para que el IRQ establezca el bit correspondiente-

    WAIT_FOR_IRQ_REPLY();

}


/*
 *
 *
 */
void nRF24L01P::sendDataFast(void* buffer, uint8_t lenBuffer) {
    
    sendData(buffer, lenBuffer, W_TX_PAYLOAD_NO_ACK);

}


/*
 *
 *
 */
void nRF24L01P::attachToSender(void* buffer, uint8_t lenBuffer, uint8_t senderPortPTX) {
    
    sendData(buffer, lenBuffer, W_ACK_PAYLOAD | senderPortPTX);

}


/*
 *
 *
 */
void nRF24L01P::resendLastData() {

    //  si el fifo ya esta vacio, no hay nada que hacer aqui...  regresamos!
    
    if (TX_FIFO_IS_EMPTY) return;
    
    RQST_TO_DEVICE(REUSE_TX_PL);
    
    SEND_PULSE(11);
    
    //  esperamos respuesta del estatus de la transmision realizada
    
    //  (la espera debe ser de apenas unos cuantos microsegundos)
    
    //  -esta pausa es necesaria para que el IRQ establezca el bit correspondiente-

    WAIT_FOR_IRQ_REPLY();
}


/*
 *
 *  procedimiento para realizar la descarga de datos que se reciben por aire.
 *
 *  lo ideal es usar la funcion dataReady() para confirmar que tenemos datos en el FIFO
 *
 *  y despues descargar el fifo usando este procedimiento.
 *
 *  paremetros: 
 *  
 *      buffer: es donde se almacenaran los datos que se extraeran del FIFO
 *  
 *      lenBuffer: indica la cantidad en bytes de datos recibidos
 *  
 *      senderPort: indica quien es el remitente de los datos (pipe)
 *
 *  devuelve:
 *
 *      los datos recibidos que se tienen almacenadoss en el FIFO. 
 *
 */
void nRF24L01P::download(void *buffer, uint8_t &lenBuffer, uint8_t &senderPort) {

    int8_t *pBuffer = (int8_t*)buffer;

    //  si hay datos en el FIFO, devolvemos al usuario:
    
    //      - la cantidad de datos en Bytes (lenBuffer)
    
    //      - el numero del puerto de quien envia los datos (senderPort)
    
    //      - y los datos que se recibieron en el FIFO (buffer)

    lenBuffer = GET_PAYLOAD_SIZE();
    
    senderPort = GET_SENDER_PORT();
    
    spiRead(R_RX_PAYLOAD, pBuffer, lenBuffer);

    CLEAR_IRQ(IRQ_MASK);

}


/*
 *
 *  funcion para devolver el modo de operacion del dispositivo
 *
 *  devuelve 0 = PTX, 1 = PRX
 *
 */
uint8_t nRF24L01P::getDeviceMode() {
    
    return IS_PRX_MODE();

}


/*
 *
 *  funcion para establece el modo de codificacion de los datos (CRC Encoding),
 *  
 *  puede ser de 8 bits, 16 bits o deshabilitado.
 *
 *  por default lo establece a 8 bits (1 Byte).
 *  
 *  no devuelve ningun valor.
 *
 */
void nRF24L01P::setCrcMode(uint8_t mode) {
    
    SET_CRC(mode);

}


/*
 *
 *  funcion para devolver el modo CRC del dispositivo
 *
 *  devuelve 0 = 8 Bits, 1 = 16 bits
 *
 */
uint8_t nRF24L01P::getCrcMode() {
    
    return (devInfo.config & 0x0C);

}


/*
 *
 *  funcion para establecer la velocidad de transmision de datos.
 *  
 *  por default se establece a 2Mbps.
 *  
 *  no devuelve ningun valor.
 *
 */
void nRF24L01P::setDataRate(uint8_t rfDataRate) {
    
    if (rfDataRate == 0) SET_RF_DATARATE(RF_DATARATE_250Kbps);
    
    else if (rfDataRate == 1) SET_RF_DATARATE(RF_DATARATE_1Mbps);
    
    else if (rfDataRate == 2) SET_RF_DATARATE(RF_DATARATE_2Mbps);
  
}


/*
 *  
 *  funcion para devolver la velocidad de transmision
 *
 */
uint8_t nRF24L01P::getDataRate() {
    
    return (devInfo.setup & 0x28);

}


/*
 *
 *  funcion para establecer el nivel de potencia de transmision de datos.
 *
 *  por default se establece a -0dBm.
 *
 *  no devuelve ningun valor.
 *
 */
void nRF24L01P::setOutputPower(uint8_t outputPower) {
    
    if (outputPower == 0) SET_RF_OUTPUT_POWER(RF_OUTPUTPOWER_0dBm);
    
    else if (outputPower == 6) SET_RF_OUTPUT_POWER(RF_OUTPUTPOWER_6dBm);
    
    else if (outputPower == 12) SET_RF_OUTPUT_POWER(RF_OUTPUTPOWER_12dBm);

    else if (outputPower == 18) SET_RF_OUTPUT_POWER(RF_OUTPUTPOWER_18dBm);

}


/*
 *
 *  funcion para devolver la salida de potencia de transmision
 *
 */
uint8_t nRF24L01P::getOutputPower() {
    
    return (devInfo.setup & 0x06);

}


/*
 *
 *  funcion para establecer el ancho de datos de la direccion del dispositivo.
 *
 *  por default la establece e 40 bits (5 Bytes) y es comun para todos los puertos.
 *
 *  no devuelve ningun valor.
 *
 */
void nRF24L01P::setAddrWidth(uint8_t length) {
    
    RQST_TO_DEVICE(W_REGISTER | SETUP_AW, length);

}


/*
 *
 *  funcion para devolver el ancho de la direccion del dispositivo
 *
 */
uint8_t nRF24L01P::getAddrWidth() {
    
    return RQST_TO_DEVICE(R_REGISTER | SETUP_AW) & 0x03;
}


/*
 *
 *  funcion para establecer la frecuencia del canal en el cual se van a
 *
 *  transmitir/recibir datos en el dispositivo.
 *
 *  por default se establece al canal 2 = [ 2400MHz + 2 ] = 2402Mhz.
 *
 *  no devuelve ningun valor.
 *
 */
void nRF24L01P::setRadioFrequency(uint8_t radioFrequency) {
    
    //  hay que cuidar que el ancho de banda a velocidades de 250Kbps y 1Mbps
    
    //  sea de 1MHz o menos. Y cuando corre a velocidades de 2Mbps el ancho de banda debe ser de 2MHz o mas.
    
    //  esta consideracion es debido a que la resolucion es de 1MHz.
    
    //  este modulo opera en el rango de frecuencia de 2.400GHz a 2.525GHz
    
    if ((radioFrequency < 0) || (radioFrequency > 126)) radioFrequency = 2;
    
    SET_RF_RADIO(radioFrequency);

}


/*
 *
 *  funcion para devolver el canal se transmision (0-127)
 *
 */
uint8_t nRF24L01P::getRfFreq() {
    
    return RQST_TO_DEVICE(R_REGISTER | RF_CH) & 0x7F;
}


/*
 *
 *  funcion para establecer el tiempo entre reintentos de transmision y
 *  
 *  el numero de veces que va a re-transmitir un paquete de datos.
 *
 *  por default el retardo es de 750us entre cada intento y 15 intentos maximo.
 *
 *  el retardo va en incrementos de 250us, los valores son desde el 0 hasta 15, donde 0 = 250us,
 *
 *  1 = 500us, 2 = 750us, ... 15 = 4000us.
 *
 *  no devuelve ningun valor.
 *
 */
void nRF24L01P::setAutoRtx(uint8_t delay, uint8_t count) {
    
    RQST_TO_DEVICE(W_REGISTER | SETUP_RETR,((uint8_t)delay << 4) | count);
}


/*
 *
 *  funcion para obtener el numero de paquetes que no se enviaron despues del
 *
 *  del maximo de intentos especificado.
 */
uint8_t nRF24L01P::getPacketsLost() {
    
    return RQST_TO_DEVICE(R_REGISTER | OBSERVE_TX) & 0xF0;
    
}


/*
 *
 *  funcion para obtener el numero de veces que fue necesairo para retransmitir un paquete
 *
 */
uint8_t nRF24L01P::getPacketsCount() {
    
    return RQST_TO_DEVICE(R_REGISTER | OBSERVE_TX) & 0x0F;
    
}


/*
 *
 *  funcion para establecer una señal portadora constante demodulada centralizada en el canal actual
 *
 *  toma como parametros el canal (frecuencia) y el tiempo que va a permanecer la señal activa.
 *
 *  no devuelve ningun valor.
 */
void nRF24L01P::testCarrierWave(uint8_t rfChannel, uint16_t timeOut) {
    
    //startTx();
    
    SET_CARRIERWAVE();
 
    setRadioFrequency(rfChannel);
    
    SET_CHIP_ENABLE;
    
    delay(timeOut);
    
    RST_CHIP_ENABLE;
    
    RST_CARRIERWAVE();
 
}


/*
 *
 *  procedimiento para realizar un escaneo del canal de frecuencia actual
 *
 *  y acumular la veces que se detectan señales presentes en ella.
 *
 *  funciona solo en modo PRX y devuelve la cantidad de señales encontradas.
 *
 */
uint8_t nRF24L01P::scanThisChannel(uint8_t rfChannel) {
    
    if (IS_PRX_MODE() == false) return;
    
    resetFeatures();

    if (rfChannel != 255) setRadioFrequency(rfChannel);

    uint8_t signalCounts = 0;

    for (int x = 0; x < 588; x++) {
        
        SET_CHIP_ENABLE;
        
        delayMicroseconds(170);
        
        RST_CHIP_ENABLE;
        
        signalCounts += (RQST_TO_DEVICE(R_REGISTER | RPD) & 0x01);
        
    }
    
    setFeatures();

    return signalCounts;
    
}



 
