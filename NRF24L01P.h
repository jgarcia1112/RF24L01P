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



#ifndef NRF24L01P_h
#define NRF24L01P_h


/* Bit Mnemonics */
/* mapa de registros de la memoria */

/* configuration register */
#define CONFIG      0x00
#define MASK_RX_DR  6
#define MASK_TX_DS  5
#define MASK_MAX_RT 4
#define EN_CRC      3
#define CRCO        2
#define PWR_UP      1
#define PRIM_RX     0

/* enable auto acknowledgment */
#define EN_AA       0x01
#define ENAA_P5     5
#define ENAA_P4     4
#define ENAA_P3     3
#define ENAA_P2     2
#define ENAA_P1     1
#define ENAA_P0     0

/* enable rx addresses */
#define EN_RXADDR   0x02
#define ERX_P5      5
#define ERX_P4      4
#define ERX_P3      3
#define ERX_P2      2
#define ERX_P1      1
#define ERX_P0      0

/* setup of address width */
#define SETUP_AW    0x03
#define AW          0 /* 2 bits */

/* setup of auto re-transmission */
#define SETUP_RETR  0x04
#define ARD         4 /* 4 bits */
#define ARC         0 /* 4 bits */

/* RF channel register */
#define RF_CH       0x05

/* RF setup register */
#define RF_SETUP    0x06
#define CONT_WAVE   7
#define RF_DR_LOW   5
#define PLL_LOCK    4
#define RF_DR_HIGH  3
#define RF_PWR_LOW  2
#define RF_PWR_HIGH 1 /* 2 bits */

/* general status register */
#define STATUS      0x07
#define RX_DR       6
#define TX_DS       5
#define MAX_RT      4
#define RX_P_NO     1 /* 3 bits */
//#define TX_FULL     0

/* transmit observe register */
#define OBSERVE_TX  0x08
#define PLOS_CNT    4 /* 4 bits */
#define ARC_CNT     0 /* 4 bits */


#define RPD         0x09
#define RX_ADDR_P0  0x0A
#define RX_ADDR_P1  0x0B
#define RX_ADDR_P2  0x0C
#define RX_ADDR_P3  0x0D
#define RX_ADDR_P4  0x0E
#define RX_ADDR_P5  0x0F
#define TX_ADDR     0x10
#define RX_PW_P0    0x11
#define RX_PW_P1    0x12
#define RX_PW_P2    0x13
#define RX_PW_P3    0x14
#define RX_PW_P4    0x15
#define RX_PW_P5    0x16


/* FIFO Status Register */
#define FIFO_STATUS 0x17
#define TX_REUSE    6
#define TX_FULL     5
#define TX_EMPTY    4
#define RX_FULL     1
#define RX_EMPTY    0

/* Enable dynamic payload length */
#define DYNPD       0x1C
#define DPL_P0      0
#define DPL_P1      1
#define DPL_P2      2
#define DPL_P3      3
#define DPL_P4      4
#define DPL_P5      5

/* feature register */
#define FEATURE     0x1D
#define EN_DPL      2
#define EN_ACK_PAY  1
#define EN_DYN_ACK  0


/* Instruction Mnemonics */
#define R_REGISTER              0x00
#define W_REGISTER              0X20
#define R_RX_PAYLOAD            0x61
#define W_TX_PAYLOAD            0xA0
#define FLUSH_TX                0xE1
#define FLUSH_RX                0xE2
#define REUSE_TX_PL             0xE3
#define R_RX_PL_WID             0x60
#define W_ACK_PAYLOAD           0xA8
#define W_TX_PAYLOAD_NO_ACK     0xB0
#define NOP                     0xFF


/* definiciones, macros*/


/* nuevas definiciones */

#define RCMD(_addrMap)          (R_REGISTER|(0x1F & (_addrMap)))
 
#define spiParams               SPISettings(10000000, MSBFIRST, SPI_MODE0)
#define DEFAULT_RX_ADDR_PORT1   0xE7E7E7E7E7
#define DEFAULT_RX_ADDR_PORT2   0xC2C2C2C2C2
#define DEFAULT_RX_ADDR_PORT3   0xC2C2C2C2C3
#define DEFAULT_RX_ADDR_PORT4   0xC2C2C2C2C4
#define DEFAULT_RX_ADDR_PORT5   0xC2C2C2C2C5
#define DEFAULT_RX_ADDR_PORT6   0xC2C2C2C2C6
#define DEFAULT_TX_ADDR_PORT    DEFAULT_RX_ADDR_PORT1

#define DEFAULT_ADDR_SIZE       5               /* 5 bytes (40 bits) de ancho */
#define MAX_LEN_FIFO_SIZE       32              /* Maximo 32 bytes (256 bits) */
#define DATA_READY              0x40
#define DATA_SENT               0x20
#define MAX_RETRY               0x10
#define IRQ_MASK                0x70
#define TX_FIFO_MASK            0x10
#define RX_FIFO_MASK            0x01

#define SENDER_PORT_PTX1        0x00
#define SENDER_PORT_PTX2        0x02
#define SENDER_PORT_PTX3        0x04
#define SENDER_PORT_PTX4        0x06
#define SENDER_PORT_PTX5        0x08
#define SENDER_PORT_PTX6        0x0A
#define SENDER_PORT_MASK        0x0E

#define DATA_PIPE_PORT_PTX1     0x01
#define DATA_PIPE_PORT_PTX2     0x02
#define DATA_PIPE_PORT_PTX3     0x04
#define DATA_PIPE_PORT_PTX4     0x08
#define DATA_PIPE_PORT_PTX5     0x10
#define DATA_PIPE_PORT_PTX6     0x20
#define DATA_PIPE_PORT_MASK     0x3F

#define ALL_DATA_PIPE_PORTS     0x3F

/* hasta aqui las nuevas definiciones */


/* macros requests frecuentes */
#define SET_CHIP_ENABLE()       { digitalWrite(CEpin,HIGH); }
#define RST_CHIP_ENABLE()       { digitalWrite(CEpin,LOW); }
#define SEND_PULSE(x)           { SET_CHIP_ENABLE(); delayMicroseconds(x); RST_CHIP_ENABLE(); }
#define SET_CHIP_SELECT()       { SPI.beginTransaction(spiParams); digitalWrite(SS,LOW); }
#define RST_CHIP_SELECT()       { digitalWrite(SS,HIGH); SPI.endTransaction(); }
#define IS_LIVE_STREAM()        ( digitalRead(CEpin) )

#define GET_DEVICE_CONFIG()     ( RQST_TO_DEVICE(R_REGISTER | CONFIG) )
#define GET_DEVICE_STATUS()     ( RQST_TO_DEVICE(R_REGISTER | STATUS) )

#define POWER_UP_DEVICE         ( GET_DEVICE_CONFIG() |  (1 << PWR_UP) )
#define POWER_DOWN_DEVICE       ( GET_DEVICE_CONFIG() & ~(1 << PWR_UP) )
#define PRX_MODE_DEVICE         ( GET_DEVICE_CONFIG() |  (1 << PRIM_RX) )
#define PTX_MODE_DEVICE         ( GET_DEVICE_CONFIG() & ~(1 << PRIM_RX) )
#define CRC_DISABLE             ( GET_DEVICE_CONFIG() & ~(1 << EN_CRC) )
#define CRC_MODE_1BYTE          ( GET_DEVICE_CONFIG() |  (1 << EN_CRC)  & ~(1 << CRCO) )
#define CRC_MODE_2BYTE          ( GET_DEVICE_CONFIG() |  (1 << EN_CRC)  |  (1 << CRCO) )
#define SET_CRC_MODE(x)         ( RQST_TO_DEVICE(W_REGISTER | CONFIG, (x)) )
#define SET_POWER_UP_DEVICE()   ( RQST_TO_DEVICE(W_REGISTER | CONFIG, POWER_UP_DEVICE) )
#define SET_POWER_DOWN_DEVICE() ( RQST_TO_DEVICE(W_REGISTER | CONFIG, POWER_DOWN_DEVICE) )
#define SET_PRX_MODE_DEVICE()   ( RQST_TO_DEVICE(W_REGISTER | CONFIG, PRX_MODE_DEVICE) )
#define SET_PTX_MODE_DEVICE()   ( RQST_TO_DEVICE(W_REGISTER | CONFIG, PTX_MODE_DEVICE) )
#define IS_PRX_MODE()           ( GET_DEVICE_CONFIG() &  0x01 )

#define GET_RF_SETUP()          ( RQST_TO_DEVICE(R_REGISTER | RF_SETUP) )
#define RF_DATARATE_1Mbps       ( GET_RF_SETUP() & ~(1 << RF_DR_LOW)  & ~(1 << RF_DR_HIGH) )
#define RF_DATARATE_2Mbps       ( GET_RF_SETUP() & ~(1 << RF_DR_LOW)  |  (1 << RF_DR_HIGH) )
#define RF_DATARATE_250Kbps     ( GET_RF_SETUP() |  (1 << RF_DR_LOW)  & ~(1 << RF_DR_HIGH) )
#define RF_OUTPUTPOWER_18dBm    ( GET_RF_SETUP() & ~(1 << RF_PWR_LOW) & ~(1 << RF_PWR_HIGH) )
#define RF_OUTPUTPOWER_12dBm    ( GET_RF_SETUP() & ~(1 << RF_PWR_LOW) |  (1 << RF_PWR_HIGH) )
#define RF_OUTPUTPOWER_6dBm     ( GET_RF_SETUP() |  (1 << RF_PWR_LOW) & ~(1 << RF_PWR_HIGH) )
#define RF_OUTPUTPOWER_0dBm     ( GET_RF_SETUP() |  (1 << RF_PWR_LOW) |  (1 << RF_PWR_HIGH) )
#define CARRIER_WAVE_ON         ( GET_RF_SETUP() |  (1 << CONT_WAVE)  |  (1 << PLL_LOCK) )
#define CARRIER_WAVE_OFF        ( GET_RF_SETUP() & ~(1 << CONT_WAVE)  & ~(1 << PLL_LOCK) )

#define SET_RF_OUTPUT_POWER(x)  ( RQST_TO_DEVICE(W_REGISTER | RF_SETUP, (x)) )
#define SET_RF_DATARATE(x)      ( RQST_TO_DEVICE(W_REGISTER | RF_SETUP, (x)) )
#define SET_CARRIERWAVE()       ( RQST_TO_DEVICE(W_REGISTER | RF_SETUP, CARRIER_WAVE_ON) )
#define RST_CARRIERWAVE()       ( RQST_TO_DEVICE(W_REGISTER | RF_SETUP, CARRIER_WAVE_OFF) )

#define SET_RF_CHANNEL(x)       ( RQST_TO_DEVICE(W_REGISTER | RF_CH, (x)) )

#define GET_FIFO_STATUS()       ( RQST_TO_DEVICE(R_REGISTER | FIFO_STATUS) )
#define TX_FIFO_IS_EMPTY()      ( GET_FIFO_STATUS() & TX_FIFO_MASK )
#define RX_FIFO_IS_EMPTY()      ( GET_FIFO_STATUS() & RX_FIFO_MASK )
#define TX_FIFO_IS_FULL()       ( GET_FIFO_STATUS() & (TX_FIFO_MASK << 1) )
#define RX_FIFO_IS_FULL()       ( GET_FIFO_STATUS() & (RX_FIFO_MASK << 1) )

#define CLEAR_TX_FIFO()         ( RQST_TO_DEVICE(FLUSH_TX) )
#define CLEAR_RX_FIFO()         ( RQST_TO_DEVICE(FLUSH_RX) )
#define CLEAR_IRQ(_irqBit)      ( RQST_TO_DEVICE(W_REGISTER | STATUS,(IRQ_MASK & (_irqBit))) )

#define WAIT_FOR_IRQ_REPLY()    { while(!(GET_DEVICE_STATUS() & IRQ_MASK)) {}; }
#define GET_SENDER_PORT()       ( GET_DEVICE_STATUS() & SENDER_PORT_MASK )
#define DATA_IS_READY()         ( GET_DEVICE_STATUS() & DATA_READY )
#define DATA_IS_SENT()          ( GET_DEVICE_STATUS() & DATA_SENT )
#define LOST_CONNECTION()       ( GET_DEVICE_STATUS() & MAX_RETRY )
#define DATA_SENT_AND_READY()   ( GET_DEVICE_STATUS() & (DATA_READY | DATA_SENT) )

#define SET_FEATURE_REGISTER()  ( RQST_TO_DEVICE(W_REGISTER | FEATURE,0x07))
#define RST_FEATURE_REGISTER()  ( RQST_TO_DEVICE(W_REGISTER | FEATURE,0x00))

#define SET_ESB_PORTS()         { RQST_TO_DEVICE(W_REGISTER | EN_AA,ALL_DATA_PIPE_PORTS);SET_AUTO_RTX();}
#define RST_ESB_PORTS()         { RQST_TO_DEVICE(W_REGISTER | EN_AA); RST_AUTO_RTX(); }
#define SET_AUTO_RTX()          ( RQST_TO_DEVICE(W_REGISTER | SETUP_RETR,0x2F))
#define RST_AUTO_RTX()          ( RQST_TO_DEVICE(W_REGISTER | SETUP_RETR,0x00))

#define SET_DPL_PORTS()         ( RQST_TO_DEVICE(W_REGISTER | DYNPD,ALL_DATA_PIPE_PORTS))
#define RST_DPL_PORTS()         ( RQST_TO_DEVICE(W_REGISTER | DYNPD,0x00))


/* fin de las macro request frecuentes */



class nRF24L01P {
    
private:
    
    uint8_t
    
        CEpin,
    
        IRQpin,
        
        GET_PAYLOAD_SIZE(),
        
        RQST_TO_DEVICE(uint8_t _reg, uint8_t _data = NOP);
    
    bool
    
        useIRQpin;
    
    void
    
        spiRead(uint8_t cmd, int8_t* pBuffer, uint8_t lenBuffer = 1),
    
        spiWrite(uint8_t cmd, int8_t* pBuffer, uint8_t lenBuffer = 1),
    
        spiData(uint8_t cmd, int8_t* pBuffer, uint8_t lenBuffer = 1);

    
public:
    
    nRF24L01P(uint8_t cepin);
    
    nRF24L01P(uint8_t cepin, uint8_t irqpin);

    void
    
        begin(),
    
        begin(uint8_t channel, uint8_t txPower, uint8_t speed),
    
        powerUp(),

        powerDown(),
    
        setRfSignal(uint8_t rfChannel, uint8_t outputPower, uint8_t rfDataRate),
    
        setFeatures(),
    
        resetFeatures(),

        setPtxMode(uint8_t* _txAddr, uint8_t len),
    
        setPtxMode(),

        setPrxMode(uint8_t* _rxAddr0, uint8_t* _rxAddr1, uint8_t len),

        setPrxMode(),
    
        setCrcMode(uint8_t mode),
    
        setAddrWidth(uint8_t length),
    
        setChannel(uint8_t freq = 2),
    
        setDataRate(uint8_t rfDataRate),
    
        setOutputPower(uint8_t outputPower),
    
        setAutoRtx(uint8_t delay = 0x02, uint8_t count = 0x0F),
    
        testCarrierWave(uint8_t rfChannel = 2, uint16_t timeOut = 2000),
    
        enableLiveStream(),

        disableLiveStream(),
    
        sendData(void* buffer, uint8_t lenBuffer = 1),
    
        sendDataFast(void* buffer, uint8_t lenBuffer = 1),
    
        attachToSender(void* buffer, uint8_t lenBuffer = 1, uint8_t senderPortPTX = SENDER_PORT_PTX1),
    
        resendLastData(),
    
        download(void *buffer, uint8_t &lenBuffer, uint8_t &senderPort);
    
    uint8_t
    
        getCrcMode(),
    
        getDeviceMode(),
    
        getAddrWidth(),
    
        getChannel(),
    
        getDataRate(),
    
        getOutputPower(),
    
        lostConnection(),
    
        dataSent(),

        dataReady(),
        
        getPacketsLost(),
    
        getPacketsCount(),

        scanThisChannel(uint8_t channel = 255);
    
};

#endif /* NRF24L01P_h */



