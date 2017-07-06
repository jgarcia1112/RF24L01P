/*
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

/* enable rx addresses */
#define EN_RXADDR   0x02

/* setup of address width */
#define SETUP_AW    0x03

/* setup of auto re-transmission */
#define SETUP_RETR  0x04

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

/* Enable dynamic payload length */
#define DYNPD       0x1C

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

#define RCMD(_addrMap)          (R_REGISTER|(0x1F & (_addrMap)))
#define spiParams               SPISettings(10000000, MSBFIRST, SPI_MODE0)

#define MAX_LEN_FIFO_SIZE       32              /* Maximo 32 bytes (256 bits) */
#define ADDRESS_WIDTH           5

#define LOGIC_CHANNEL_PORT1     0x01
#define LOGIC_CHANNEL_PORT2     0x02
#define LOGIC_CHANNEL_PORT3     0x04
#define LOGIC_CHANNEL_PORT4     0x08
#define LOGIC_CHANNEL_PORT5     0x10
#define LOGIC_CHANNEL_PORT6     0x20
#define LOGIC_CHANNEL_PORT_MASK 0x3F
#define LOGIC_CHANNEL_PORTS_ALL LOGIC_CHANNEL_PORT_MASK
#define LOGIC_CHANNEL_DEFAULT   0x03

#define LOGIC_CHANNEL1          0x00
#define LOGIC_CHANNEL2          0x02
#define LOGIC_CHANNEL3          0x04
#define LOGIC_CHANNEL4          0x06
#define LOGIC_CHANNEL5          0x08
#define LOGIC_CHANNEL6          0x0A
#define LOGIC_CHANNEL_MASK      0x0E

#define CHANNEL1                0
#define CHANNEL2                1
#define CHANNEL3                2
#define CHANNEL4                3
#define CHANNEL5                4
#define CHANNEL6                5
#define MAX_CHANNELS            6

#define DATA_READY              0x40
#define DATA_SENT               0x20
#define MAX_RETRY               0x10
#define IRQ_MASK                0x70
#define TX_FIFO_MASK            0x10
#define RX_FIFO_MASK            0x01

#define SET_CHIP_ENABLE         { digitalWrite(CEpin,HIGH); }
#define RST_CHIP_ENABLE         { digitalWrite(CEpin,LOW); }
#define SEND_PULSE(x)           { SET_CHIP_ENABLE; uint8_t i=50; while(i--); RST_CHIP_ENABLE; }
#define SET_CHIP_SELECT         { SPI.beginTransaction(spiParams); digitalWrite(SS,LOW); }
#define SET_SPI_COMMAND(x)      { devInfo.status = SPI.transfer(x); }
#define RST_CHIP_SELECT         { digitalWrite(SS,HIGH); SPI.endTransaction(); }
#define IS_LIVE_STREAM()        ( digitalRead(CEpin) )

#define GET_DEVICE_STATUS()     ( RQST_TO_DEVICE(R_REGISTER | STATUS) )

#define POWER_UP_DEVICE         ( devInfo.config  |  (1 << PWR_UP)  )
#define POWER_DOWN_DEVICE       ( devInfo.config  & ~(1 << PWR_UP)  )
#define PRX_MODE_DEVICE         ( devInfo.config  |  (1 << PRIM_RX) )
#define PTX_MODE_DEVICE         ( devInfo.config  & ~(1 << PRIM_RX) )
#define CRC_MODE_1              ( devInfo.config  |  (1 << EN_CRC)  & ~(1 << CRCO) )
#define CRC_MODE_2              ( devInfo.config  |  (1 << EN_CRC)  |  (1 << CRCO) )
#define CRC_DISABLE             ( devInfo.config  & ~(1 << EN_CRC)  )
#define IS_PRX_MODE()           ( devInfo.config  &   0x01 )

#define SET_CRC(x)              ( RQST_TO_DEVICE(W_REGISTER | CONFIG, (x)) )
#define SET_POWER_UP_DEVICE     ( RQST_TO_DEVICE(W_REGISTER | CONFIG, POWER_UP_DEVICE) )
#define SET_POWER_DOWN_DEVICE   ( RQST_TO_DEVICE(W_REGISTER | CONFIG, POWER_DOWN_DEVICE) )
#define SET_PRX_MODE_DEVICE     ( RQST_TO_DEVICE(W_REGISTER | CONFIG, PRX_MODE_DEVICE) )
#define SET_PTX_MODE_DEVICE     ( RQST_TO_DEVICE(W_REGISTER | CONFIG, PTX_MODE_DEVICE) )

#define RF_DATARATE_1Mbps       ( devInfo.setup & ~(1 << RF_DR_LOW)  & ~(1 << RF_DR_HIGH)  )
#define RF_DATARATE_2Mbps       ( devInfo.setup & ~(1 << RF_DR_LOW)  |  (1 << RF_DR_HIGH)  )
#define RF_DATARATE_250Kbps     ( devInfo.setup |  (1 << RF_DR_LOW)  & ~(1 << RF_DR_HIGH)  )
#define RF_OUTPUTPOWER_0dBm     ( devInfo.setup |  (1 << RF_PWR_LOW) |  (1 << RF_PWR_HIGH) )
#define RF_OUTPUTPOWER_6dBm     ( devInfo.setup |  (1 << RF_PWR_LOW) & ~(1 << RF_PWR_HIGH) )
#define RF_OUTPUTPOWER_12dBm    ( devInfo.setup & ~(1 << RF_PWR_LOW) |  (1 << RF_PWR_HIGH) )
#define RF_OUTPUTPOWER_18dBm    ( devInfo.setup & ~(1 << RF_PWR_LOW) & ~(1 << RF_PWR_HIGH) )
#define CARRIER_WAVE_ON         ( devInfo.setup |  (1 << CONT_WAVE)  |  (1 << PLL_LOCK)    )
#define CARRIER_WAVE_OFF        ( devInfo.setup & ~(1 << CONT_WAVE)  & ~(1 << PLL_LOCK)    )

#define SET_RF_OUTPUT_POWER(x)  ( RQST_TO_DEVICE(W_REGISTER | RF_SETUP, (x)) )
#define SET_RF_DATARATE(x)      ( RQST_TO_DEVICE(W_REGISTER | RF_SETUP, (x)) )
#define SET_CARRIERWAVE()       ( RQST_TO_DEVICE(W_REGISTER | RF_SETUP, CARRIER_WAVE_ON) )
#define RST_CARRIERWAVE()       ( RQST_TO_DEVICE(W_REGISTER | RF_SETUP, CARRIER_WAVE_OFF) )

#define SET_RF_RADIO(x)         ( RQST_TO_DEVICE(W_REGISTER | RF_CH, (x)) )

#define GET_FIFO_STATUS()       ( RQST_TO_DEVICE(R_REGISTER | FIFO_STATUS) )
#define TX_FIFO_IS_EMPTY        ( GET_FIFO_STATUS() & TX_FIFO_MASK )
#define TX_FIFO_IS_FULL         ( devInfo.status & (TX_FIFO_MASK >> 4) )
#define RX_FIFO_IS_EMPTY        ( devInfo.status & LOGIC_CHANNEL_MASK )

#define CLEAR_TX_FIFO()         ( RQST_TO_DEVICE(FLUSH_TX) )
#define CLEAR_RX_FIFO()         ( RQST_TO_DEVICE(FLUSH_RX) )
#define CLEAR_IRQ(_irqBit)      ( RQST_TO_DEVICE(W_REGISTER | STATUS,(IRQ_MASK & (_irqBit))) )

#define WAIT_FOR_IRQ_REPLY()    { while(!(GET_DEVICE_STATUS() & IRQ_MASK)) {}; }
#define GET_SENDER_PORT()       ( devInfo.status &  LOGIC_CHANNEL_MASK )
#define DATA_IS_SENT            ( devInfo.status &  DATA_SENT  )
#define LOST_CONNECTION         ( devInfo.status &  MAX_RETRY  )

#define SET_FEATURE_REGISTER()  ( RQST_TO_DEVICE(W_REGISTER | FEATURE,0x07))
#define RST_FEATURE_REGISTER()  ( RQST_TO_DEVICE(W_REGISTER | FEATURE,0x00))

#define SET_ESB_PORTS()         { RQST_TO_DEVICE(W_REGISTER | EN_AA,LOGIC_CHANNEL_PORTS_ALL);SET_AUTO_RTX();}
#define RST_ESB_PORTS()         { RQST_TO_DEVICE(W_REGISTER | EN_AA); RST_AUTO_RTX(); }
#define SET_AUTO_RTX()          ( RQST_TO_DEVICE(W_REGISTER | SETUP_RETR,0x4C)) 
#define RST_AUTO_RTX()          ( RQST_TO_DEVICE(W_REGISTER | SETUP_RETR,0x00))

#define SET_DPL_PORTS()         ( RQST_TO_DEVICE(W_REGISTER | DYNPD,LOGIC_CHANNEL_PORTS_ALL))
#define RST_DPL_PORTS()         ( RQST_TO_DEVICE(W_REGISTER | DYNPD,0x00))

#define SET_RX_PORTS(x)         ( RQST_TO_DEVICE(W_REGISTER | EN_RXADDR, (x)))
#define RST_RX_PORTS()          ( RQST_TO_DEVICE(W_REGISTER | EN_RXADDR,LOGIC_CHANNEL_DEFAULT))




typedef struct {
    
    uint8_t         config = { 0x0A }; // CRC_MODE_1 | POWER_UP_DEVICE | PTX_MODE_DEVICE
    
    uint8_t         setup  = { 0x0E }; // RF_DATARATE_2Mbps | RF_OUTPUTPOWER_0dBm
    
    uint8_t         status;

} DeviceInfo;





class nRF24L01P {
    
private:

    DeviceInfo
    
        devInfo;
    
    uint8_t
    
        CEpin,
    
        GET_PAYLOAD_SIZE(),
        
        RQST_TO_DEVICE(uint8_t _reg, uint8_t _data = NOP);
    
    void
    
        setRxAddresses(uint8_t* rxAddresses, uint8_t _addressWidth),

        spiRead(uint8_t cmd, int8_t* pBuffer, uint8_t lenBuffer = 1),
    
        spiWrite(uint8_t cmd, const int8_t* pBuffer, uint8_t lenBuffer = 1, uint8_t start = 1);

public:
    
    nRF24L01P(uint8_t cepin);
    
    void
    
        begin(),
    
        powerUp(),

        powerDown(),
    
    
        setRadioLayer(uint8_t radioFrequency, uint8_t outputPower, uint8_t radioDataRate, uint8_t crcMode),
    
        setRadioFrequency(uint8_t freq = 2),
    
        setFeatures(),
    
        resetFeatures(),

    
        startRx(uint8_t* rxAddresses, uint8_t _addressWidth),

        startTx(uint8_t logicChannel, uint8_t* _address, uint8_t _addressWidth),


        standby(),
    
 
        setCrcMode(uint8_t mode),
    
        setAddrWidth(uint8_t length),
    
    
        setDataRate(uint8_t rfDataRate),
    
        setOutputPower(uint8_t outputPower),
    
    
        setAutoRtx(uint8_t delay = 0x02, uint8_t count = 0x0F),
    
        testCarrierWave(uint8_t rfChannel = 2, uint16_t timeOut = 2000),
    
    
        enableLiveStream(),

        disableLiveStream(),
    
    
        sendData(void* buffer, uint8_t lenBuffer = 1, uint8_t cmd = W_TX_PAYLOAD),
    
        sendDataFast(void* buffer, uint8_t lenBuffer = 1),
    
        attachToSender(void* buffer, uint8_t lenBuffer = 1, uint8_t senderPortPTX = CHANNEL1),
    
        resendLastData(),
    
        download(void *buffer, uint8_t &lenBuffer, uint8_t &senderPort);

    uint8_t
    
        getCrcMode(),
    
        getDeviceMode(),
    
        getAddrWidth(),
    
        getRfFreq(),
    
        getDataRate(),
    
        getOutputPower(),

        getPacketsLost(),
    
        getPacketsCount(),

        scanThisChannel(uint8_t channel = 255);
    
    bool
    
        dataReady(),
    
        lostConnection(),
    
        dataSent();

};

#endif /* NRF24L01P_h */



