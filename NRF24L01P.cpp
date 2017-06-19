/*
 *  Nordic Semiconductor nRF24L01+
 *
 *  NRF24L01P.cpp
 *
 *  Creado: 25 Ago 2016 por jgarcia1112@gmail.com
 *  Villahermosa, Tabasco. Mexico
 *
 *  Modificado: 18 Jun 2017 por Javier Garcia.
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
    
    useIRQpin = false;
    
    pinMode(CEpin,  OUTPUT);
 
    /*
     *
     *  al iniciar el SPI, el pin CSN (SS) lo establece en modo OUTPUT por que es parte
     *
     *  del los 4 cables del SPI (SS, MOSI, MISO, SCK)
     *
     */

}


/*
 *
 *  constructor de clase.
 *
 *  inicia la clase y establece el pin de control IRQ y CE del dispositivo
 *
 *  en este modo se maneja el pin de Interupcion (IRQ) del dispositivo
 *
 *  parametros:
 *
 *          cepin   -> Chip Enable Pin
 *
 *          irqpin  -> IRQ pin
 *
 *  devuelve: 
 *
 *          un objeto clase enlazado el dispositivo nrf24l01+ usando el pin de CE y el pin IRQ
 *
 */
nRF24L01P::nRF24L01P(uint8_t cepin, uint8_t irqpin) {
    
    // establece el pin CE (Chip Enable) en modo OUTPUT
    
    CEpin = cepin;
    
    IRQpin = irqpin;
    
    useIRQpin = true;
    
    pinMode(CEpin,  OUTPUT);
    
    pinMode(IRQpin, OUTPUT);
    
    /*
     *
     *  al iniciar el SPI, el pin CSN (SS) lo establece en modo OUTPUT por que es parte
     *
     *  del los 4 cables del SPI (SS, MOSI, MISO, SCK)
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
    
    // inicializa la libreria de comunicacion SPI
    
    SPI.begin();
    
    // realiza el encendido del dispositivo
    
    powerUp();
    
    setFeatures();

}


/*
 *
 *  inicializa el dispositivo.
 *
 *  establece la frecuencia, la potencia de transmision, y la velocidad de datos
 *
 *  parametros:
 *
 *          channel     -> frecuencia de operacion del modulo (desde 0 hasta 126 = (2400Mhz hasta 2528Mhz)
 *
 *          outputPower -> potencia de transmision: 0dBm, -6dBm, -12dBm y -18dBm
 *
 *          dataRate    -> velocidad de datos en aire: 256Kbps, 1Mbps y 2Mbps
 *
 *  devuelve: 
 *  
 *          nada
 *
 *
 */
void nRF24L01P::begin(uint8_t channel, uint8_t outputPower, uint8_t dataRate) {
    
    begin();
    
    //  inicializa el modulo ajustando el canal, la potencia de salida y la velocidad de transferencia,
    
    //  esto es espcialmente util si se van a usar valores distintos al que trae por default el dispositivo.
    
    setRfSignal(channel, outputPower, dataRate);
    
    // funciones mejoradas del dispositivo
    
    //setFeatures();
    
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

    // modo ESB (enable auto acknowledgement)
    
    SET_ESB_PORTS();
    
    // carga dinamica (enable dynamic payload length)
    
    SET_DPL_PORTS();

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
    
    // habilita las funciones mejoradas del dispositivo
    
    RST_FEATURE_REGISTER();
    
    // modo ESB (enable auto acknowledgement)
    
    RST_ESB_PORTS();
    
    // carga dinamica (enable dynamic payload length)
    
    RST_DPL_PORTS();
    
}


/*
 *
 *  funcion para realizar el encendido del dispositivo.
 *
 */
void nRF24L01P::powerUp() {
    
    //  instruccion para encender el dispositivo
    
    SET_POWER_UP_DEVICE();
    
}


/*
 *
 *  funcion para realizar el apagado del dispositivo.
 *
 */
void nRF24L01P::powerDown() {
    
    // simplemente, lo apaga ....  y ya!
    
    SET_POWER_DOWN_DEVICE();

}


void nRF24L01P::setRfSignal(uint8_t rfChannel, uint8_t outputPower, uint8_t rfDataRate) {
    
    SET_RF_CHANNEL(rfChannel);
    
    SET_RF_OUTPUT_POWER(outputPower);
    
    SET_RF_DATARATE(rfDataRate);
    
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
    
    SET_CHIP_SELECT();
    
        SPI.transfer(_reg);
    
        uint8_t val = SPI.transfer(_data);
    
    RST_CHIP_SELECT();
    
    return val;

}


/*
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
    
    SET_CHIP_SELECT();
    
    SPI.transfer(cmd);
    
    for (uint8_t idx = 0; idx < lenBuffer; idx++)

        pBuffer[idx] = SPI.transfer(NOP);
    
    RST_CHIP_SELECT();
        
}


/*
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
void nRF24L01P::spiWrite(uint8_t cmd, int8_t* pBuffer, uint8_t lenBuffer) {
    
    SET_CHIP_SELECT();
    
    SPI.transfer(cmd);
    
    for (uint8_t idx = 0; idx < lenBuffer; idx++)
        
        SPI.transfer((int8_t) pBuffer[idx]);
    
    RST_CHIP_SELECT();
    
}


void nRF24L01P::spiData(uint8_t cmd, int8_t* pBuffer, uint8_t lenBuffer) {
    
    //int8_t* pTmpBuff[lenBuffer];

    SET_CHIP_SELECT();
    
    SPI.transfer(cmd);
    
    for (uint8_t idx = 0; idx < lenBuffer; idx++)

        pBuffer[idx] = SPI.transfer((int8_t) pBuffer[idx]);
    
    //pBuffer = pTmpBuff;

    RST_CHIP_SELECT();

}
/*
 *  funcion para realizar la lectura de la cantidad de bytes que se encuentran
 *  
 *  en el fifo RX, se utiliza para en el modo Dynamic Payload Length.
 *
 *  parametros: ninguno.
 *
 *  devuelve: el total de bytes recibdos en el FIFO o 0 si no hay datos, o hay error en la recepcion.
 *
 */
uint8_t nRF24L01P::GET_PAYLOAD_SIZE() {
    
    uint8_t ret = RQST_TO_DEVICE(R_RX_PL_WID);
    
    if (ret > MAX_LEN_FIFO_SIZE) {
        
        CLEAR_RX_FIFO();
        
        return 0;
        
    }
    
    return ret;
    
}


/*
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
uint8_t nRF24L01P::dataReady() {
    
    if (IS_PRX_MODE())
    {
        //  si estamos en modo recepcion checamos si el IRQ se disparo o si el FIFO aun contiene datos
        //  o falso si no hay datos recibidos aun.
        if (DATA_IS_READY() == true) return GET_PAYLOAD_SIZE();
        
        else if (RX_FIFO_IS_EMPTY() == false) return GET_PAYLOAD_SIZE();
        
        else  return false;
    }
    else
    {
        //  si estamos en modo transmisor vamos a checar si al recibir la confirmacion (ACK)
        //  del envio, vienen datos adjuntos, si es asi, devolvemos el tamaño (verdadero) o
        //  limpiamos el IRQ del estatus y devolvemos falso porque no se recibio nada.
        
        if (DATA_SENT_AND_READY() == true) return GET_PAYLOAD_SIZE();
        
        else {
            
            // limpia los bits del IRQ del registro estatus
            
            //CLEAR_IRQ(IRQ_MASK);
            
            if (LOST_CONNECTION()) CLEAR_IRQ(MAX_RETRY);
            
            if (DATA_IS_SENT()) CLEAR_IRQ(DATA_SENT);

            if (DATA_IS_READY()) CLEAR_IRQ(DATA_READY);
            
            return false;
        }
    }
}


/*
 *
 *
 */
uint8_t nRF24L01P::dataSent() {
    
    return DATA_IS_SENT();
    
}


/*
 *
 *
 */
uint8_t nRF24L01P::lostConnection() {
    
    return LOST_CONNECTION();
    
}


/*
 *
 *
 *  POR HACER:
 *
 *      UNA INSTRUCCION O COMANDO O FUNCION EN DONDE PASE DOS ESTRUCTURAS DE DATOS COMO PARAMETROS
 *      EL PRIMERO SERA UNA ESTRUCTURA CON LOS DATOS A ENVIAR Y EL TAMAÑO (LARGO) TOTAL DE DATOS
 *      EL SEGUNDO SERA UNA ESTRUCTURA CON LA INFORMACION DE LA RADIOFRECUENCIA: 
 *          - MODO CRC, CANAL DE FRECUENCIA, VELOCIDAD DE TRANSMISION, DIRECCION DESTINO,
 *          - TIPO DE ENVIO (NORMAL, RAPIDO O DATO ADJUTNO), NIVEL DE POTENCIA DE TRANSMISION
 *
 *  
 *      AUNQUE IGUAL PIESNO QUE SE PODRIA USAR COMO PARAMETRO EN EL BEGIN Y USAR UN PROCEDIMIENTO COMO
 *      : CONFIGUREPTX Y CONFIGUREPRX CON PARAMETROS POR DEFAULT (UNA ESTRUCTURA DE DATOS)
 *
 *
 */


void nRF24L01P::enableLiveStream() {
    
    SET_CHIP_ENABLE();
    
}

void nRF24L01P::disableLiveStream() {
    
    RST_CHIP_ENABLE();
    
}


/*
 *
 *
 */
void nRF24L01P::sendData(void* buffer, uint8_t lenBuffer) {
    
    //  si no estamos en modo transmisor, no hay nada que hacer aqui... regresamos!

    if (IS_PRX_MODE() == true) return;
    
        int8_t *pTxData = (int8_t*)buffer;
    
        //  si el FIFO esta lleno, se limpia para permiter el envio de datos
        //  le damos prioridad a vaciar el FIFO para que el envio salga lo mas pronto posible
        //  lo que se haya vaciado contara como paquete perdido, lo cual muy probablemente sea por
        //  haber alcanzado el maximo de reintentos que maneja el ESB.
  
        if (LOST_CONNECTION()) CLEAR_IRQ(MAX_RETRY);
    
        if (DATA_IS_SENT()) CLEAR_IRQ(DATA_SENT);
    
        if (DATA_IS_READY()) CLEAR_IRQ(DATA_READY);
 
        if (TX_FIFO_IS_FULL()) CLEAR_TX_FIFO();
    
        spiData(W_TX_PAYLOAD, pTxData, lenBuffer);
 
        if (IS_LIVE_STREAM() == false) SEND_PULSE(11);
    
        //  esperamos respuesta del estatus de la transmision realizada
        //  (la espera debe ser de apenas unos cuantos microsegundos)
        //  -esta pausa es necesaria para que el IRQ establezca el bit correspondiente-
        WAIT_FOR_IRQ_REPLY(); // si el ESB no esta habilitado, aqui se queda en un loop infinito!
 
}


/*
 *
 *
 */
void nRF24L01P::sendDataFast(void* buffer, uint8_t lenBuffer) {

    //  si no estamos en modo transmisor, no hay nada que hacer aqui... regresamos!

    if (IS_PRX_MODE() == true) return;

        int8_t *pTxData = (int8_t*)buffer;

        //  si el FIFO esta lleno, se limpia para permiter el envio de datos
        //  le damos prioridad a vaciar el FIFO para que el envio salga lo mas pronto posible
        //  lo que se haya vaciado contara como paquete perdido, lo cual muy probablemente sea por
        //  haber alcanzado el maximo de reintentos que maneja el ESB.

        if (lostConnection()) CLEAR_IRQ(MAX_RETRY);
    
        if (dataSent()) CLEAR_IRQ(DATA_SENT);

        if (TX_FIFO_IS_FULL()) CLEAR_TX_FIFO();

        spiData(W_TX_PAYLOAD_NO_ACK, pTxData, lenBuffer);
        
        if (IS_LIVE_STREAM() == false) { SEND_PULSE(11);

        //  no esperamos respuesta del estatus de la transmision realizada
        //  porque le envio se realizo sin respuesta de confirmacion (NO_ACK)
        //  unicamente confirma que localmente el envio se realizo.
        //  -esta pausa es necesaria para que el IRQ establezca el bit correspondiente-

        WAIT_FOR_IRQ_REPLY(); }
}


/*
 *
 *
 */
void nRF24L01P::attachToSender(void* buffer, uint8_t lenBuffer, uint8_t senderPortPTX) {
    
    //  si no estamos en modo receptor, no hay nada que hacer aqui... regresamos!

    if (IS_PRX_MODE() == false) return;

        int8_t *pTxData = (int8_t*)buffer;

        //  si el FIFO esta lleno, se limpia para permiter el envio de datos
        //  le damos prioridad a vaciar el FIFO para que el envio salga lo mas pronto posible
        //  lo que se haya vaciado contara como paquete perdido, lo cual muy probablemente sea por
        //  haber alcanzado el maximo de reintentos que maneja el ESB.

        if (TX_FIFO_IS_FULL()) CLEAR_TX_FIFO();
        
        spiData(W_ACK_PAYLOAD | senderPortPTX, pTxData, lenBuffer);
        
        SEND_PULSE(11);

        //  esperamos respuesta del estatus de la transmision realizada
        //  (la espera debe ser de apenas unos cuantos microsegundos)
        //  -esta pausa es necesaria para que el IRQ establezca el bit correspondiente-

        WAIT_FOR_IRQ_REPLY();
}


/*
 *
 *
 */
void nRF24L01P::resendLastData() {
    
    //  si no estamos en modo transmisor, no hay nada que hacer aqui... regresamos!
    
    if (IS_PRX_MODE() == true) return;
    
    //  si el fifo ya esta vacio, no hay nada que hacer aqui...  regresamos!
    
    if (TX_FIFO_IS_EMPTY()) return;
    
    RQST_TO_DEVICE(REUSE_TX_PL);
    
    SEND_PULSE(11);
    
    //  esperamos respuesta del estatus de la transmision realizada
    //  (la espera debe ser de apenas unos cuantos microsegundos)
    //  -esta pausa es necesaria para que el IRQ establezca el bit correspondiente-

    WAIT_FOR_IRQ_REPLY(); // si el ESB no esta habilitado, aqui se queda en un loop infinito!
}


/*
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
    
    //  si en el FIFO no hay datos, no hay nada que hacer aqui... regresamos!
    
    if (!dataReady()) return;
    
    //  si hay datos en el FIFO, devolvemos al usuario:
    
    //      - la cantidad de datos en Bytes (lenBuffer)
    
    //      - el numero del puerto de quien envia los datos (senderPort)
    
    //      - y los datos que se recibieron en el FIFO (buffer)
    
    int8_t *pBuffer = (int8_t*)buffer;
    
    lenBuffer = GET_PAYLOAD_SIZE();
    
    senderPort = GET_SENDER_PORT();
    
    //spiRead(R_RX_PAYLOAD, pBuffer, lenBuffer);
    spiData(R_RX_PAYLOAD, pBuffer, lenBuffer);
    
    //CLEAR_IRQ(IRQ_MASK);
    if (DATA_IS_READY()) CLEAR_IRQ(DATA_READY);
    
}


/*
 *  funcion para establecer el modo PTX manualmente (modo experto).
 *
 *  usa la direccion y el ancho de la direccion como argumento.
 *
 *  permanece en modo stby-I.
 *
 */
void nRF24L01P::setPtxMode(uint8_t* _txAddr, uint8_t len) {
    
    //  establece el ancho de bytes de la direccion
    
    setAddrWidth(len-2);
    
    //  establece la direccion del PTX
    
    spiData(W_REGISTER | TX_ADDR, _txAddr, len);
    
    //  establece la direccion del PRX igual al PTX para manejar auto ACK en modo ESB
    
    spiData(W_REGISTER | RX_ADDR_P0, _txAddr, len);
    
    //  establece el modo PTX
    
    setPtxMode();

 }


/*
 *  funcion para establecer el modo PTX en el puerto 0 (direccion del pipe 0).
 *
 *  igual que el de arriba, pero sin argumentos usando el puerto 0 (pipe 0)
 *
 *  la direccion 0xE7 (por default) es a 5 bytes y CRC de 8 bits.
 *
 *  permanece en modo stby-I
 *
 */
void nRF24L01P::setPtxMode() {
    
    //  lo mas simple, solo se habilita el modo PTX y tomara por default las
    
    //  direcciones del PTX y PRX propias del dispositivo (default)

    SET_PTX_MODE_DEVICE();
 
}


/*
 *  funcion para establecer el modo PRX en el puerto 0 y 1 (direccion del pipe 0 y 1).
 *
 *  igual que el de arriba, pero sin argumentos usando el puerto 0 y 1 (pipe 0, pipe 1)
 *
 *  la direccion 0xE7 (por default) es a 5 bytes y CRC de 8 bits.
 *
 *  permanece en modo stby-I
 *
 */
void nRF24L01P::setPrxMode() {
    
    //  lo mas simple, solo se habilita el modo PRX y tomara por default las
    
    //  direcciones del PTX y PRX propias del dispositivo (default)
    
    SET_PRX_MODE_DEVICE();

    SET_CHIP_ENABLE();
    
}


/*
 *  establece el modo PRX
 *
 *  por default solo escucha en el puerto 0 y 1 (EN_RXADDR: ERX_P0 y ERX_P1).
 *
 *  utiliza la direccion 0xE7 de 5 bytes, con CRC de 8 bits y
 *
 *  con auto ACK habilitado (EN_AA) y Dynamic Payload Length (DYNPD) en todos los puertos (pipes).
 *
 */
void nRF24L01P::setPrxMode(uint8_t* _rxAddr0, uint8_t* _rxAddr1, uint8_t len) {
    
    // establece el ancho de bytes de la direccion
    
    setAddrWidth(len-2);
    
    // establece la direccion del PRX en el puerto 1 (pipe 0)
    
    spiData(W_REGISTER | RX_ADDR_P0, _rxAddr0, len);

    // establece la direccion del PRX en el puerto 2 (pipe 1)
    
    spiData(W_REGISTER | RX_ADDR_P1, _rxAddr1, len);
    
    // establece la direccion del PRX en el puerto 3 (pipe 2)
    
    spiData(W_REGISTER | RX_ADDR_P2, _rxAddr1[len]+1, 1);
    
    // establece la direccion del PRX en el puerto 4 (pipe 3)
    
    spiData(W_REGISTER | RX_ADDR_P3, _rxAddr1[len]+2, 1);
    
    // establece la direccion del PRX en el puerto 5 (pipe 4)
    
    spiData(W_REGISTER | RX_ADDR_P4, _rxAddr1[len]+3, 1);
    
    // establece la direccion del PRX en el puerto 6 (pipe 5)
    
    spiData(W_REGISTER | RX_ADDR_P5, _rxAddr1[len]+4, 1);
    
    //  establece el modo PRX
    
    setPrxMode();
    
}


/*
 *  funcion para devolver el modo de operacion del dispositivo
 *
 *  devuelve 0 = PTX, 1 = PRX
 *
 */
uint8_t nRF24L01P::getDeviceMode() {
    
    return IS_PRX_MODE();
}


/*
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
    
    //if ((mode < CRC_DISABLE) || (mode > CRC_16_BITS)) mode = CRC_8_BITS;
    
    SET_CRC_MODE(mode);

}


/*
 *  funcion para devolver el modo CRC del dispositivo
 *
 *  devuelve 0 = 8 Bits, 1 = 16 bits
 *
 */
uint8_t nRF24L01P::getCrcMode() {

    return (GET_DEVICE_CONFIG() & 0x0C);
}


/*
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

    return (GET_RF_SETUP() & 0x28);
}


/*
 *  funcion para establecer el nivel de potencia de transmision de datos.
 *
 *  por default se establece a -0dBm.
 *
 *  no devuelve ningun valor.
 *
 */
void nRF24L01P::setOutputPower(uint8_t outputPower) {
    
    //if ((txLevel < TX_POWER_LEVEL_18) || (txLevel > TX_POWER_LEVEL_0)) txLevel = TX_POWER_LEVEL_0;
    
    //uint8_t reg = (RQST_TO_DEVICE(RCMD(RF_SETUP)) & (NOP ^ TX_POWER_LEVEL_MASK)) | txLevel;
    
    //RQST_TO_DEVICE(W_REGISTER | RF_SETUP,reg);
    
    //SET_RF_OUTPUT_POWER(outputPower);
    
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
    
    //return RQST_TO_DEVICE(RCMD(RF_SETUP) & TX_POWER_LEVEL_MASK);
    return (GET_RF_SETUP() & 0x06);
}


/*
 *  funcion para establecer el ancho de datos de la direccion del dispositivo.
 *
 *  por default la establece e 40 bits (5 Bytes) y es comun para todos los puertos.
 *
 *  no devuelve ningun valor.
 *
 */
void nRF24L01P::setAddrWidth(uint8_t length) {
    
    //if ((length < ADDR_SIZE_3_BYTES) || (length > ADDR_SIZE_5_BYTES)) length = ADDR_SIZE_5_BYTES;
    
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
 *  funcion para establecer la frecuencia del canal en el cual se van a
 *
 *  transmitir/recibir datos en el dispositivo.
 *
 *  por default se establece al canal 2 = [ 2400MHz + 2 ] = 2402Mhz.
 *
 *  no devuelve ningun valor.
 *
 */
void nRF24L01P::setChannel(uint8_t rfChannel) {
    
    //  hay que cuidar que el ancho de banda a velocidades de 250Kbps y 1Mbps
    
    //  sea de 1MHz o menos. Y cuando corre a velocidades de 2Mbps el ancho de banda debe ser de 2MHz o mas.
    
    //  esta consideracion es debido a que la resolucion es de 1MHz.
    
    //  este modulo opera en el rango de frecuencia de 2.400GHz a 2.525GHz
    
    if ((rfChannel < 0) || (rfChannel > 126)) rfChannel = 2;
    
    //RQST_TO_DEVICE(W_REGISTER | RF_CH, freq);
    
    SET_RF_CHANNEL(rfChannel);

}


/*
 *
 *  funcion para devolver el canal se transmision (0-127)
 *
 */
uint8_t nRF24L01P::getChannel() {
    
    return RQST_TO_DEVICE(R_REGISTER | RF_CH) & 0x7F;
}


/*
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
 *  funcion para obtener el numero de paquetes que no se enviaron despues del
 *
 *  del maximo de intentos especificado.
 */
uint8_t nRF24L01P::getPacketsLost() {
    
    return RQST_TO_DEVICE(R_REGISTER | OBSERVE_TX) & 0xF0;
    
}


/*
 *  funcion para obtener el numero de veces que fue necesairo para retransmitir un paquete
 *
 */
uint8_t nRF24L01P::getPacketsCount() {
    
    return RQST_TO_DEVICE(R_REGISTER | OBSERVE_TX) & 0x0F;
    
}


/*
 *  funcion para establecer una señal portadora constante demodulada centralizada en el canal actual
 *
 *  toma como parametros el canal (frecuencia) y el tiempo que va a permanecer la señal activa.
 *
 *  no devuelve ningun valor.
 */
void nRF24L01P::testCarrierWave(uint8_t rfChannel, uint16_t timeOut) {
    
    setPtxMode();
    
    SET_CARRIERWAVE();
 
    setChannel(rfChannel);
    
    SET_CHIP_ENABLE();
    
    delay(timeOut);
    
    RST_CHIP_ENABLE();
    
    RST_CARRIERWAVE();
 
}


/*
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

    if (rfChannel != 255) setChannel(rfChannel);

    uint8_t signalCounts = 0;

    for (int x = 0; x < 588; x++) {
        
        SET_CHIP_ENABLE();
        
        delayMicroseconds(170);
        
        RST_CHIP_ENABLE();
        
        signalCounts += (RQST_TO_DEVICE(R_REGISTER | RPD) & 0x01);
        
    }
    
    setFeatures();
    
    //SET_CHIP_ENABLE();
    
    return signalCounts;
    
}



 
