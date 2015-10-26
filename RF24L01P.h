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

/**
 * @file RF24L01P.h
 *
 */


#ifndef _RF24L01P_H_
#define _RF24L01P_H_


// comandos del modulo RF
#define ReadCmd                0x00
#define WriteCmd               0x20
#define RF_R_RX_PAYLOAD        0x61
#define RF_W_TX_PAYLOAD        0xA0
#define RF_FLUSH_TX            0xE1
#define RF_FLUSH_RX            0xE2
#define RF_REUSE_TX_PL         0xE3
#define RF_R_RX_PL_WID         0x60
#define RF_W_ACK_PAYLOAD       0xA8
#define RF_W_TX_PAYLOAD_NO_ACK 0xB0
#define NOP                    0xFF

// posiciones del bit en el registro CONFIG
#define RF_CONFIG              0x00
#define RF_PRIM_RX_BIT         0
#define RF_PWR_UP_BIT          1
#define RF_CRCO_BIT            2

// posiciones del bit en el registro EN_AA
#define RF_EN_AA               0x01
#define RF_ENAA_P5_BIT         5
#define RF_ENAA_P4_BIT         4
#define RF_ENAA_P3_BIT         3
#define RF_ENAA_P2_BIT         2
#define RF_ENAA_P1_BIT         1
#define RF_ENAA_P0_BIT         0

// posiciones del bit en el registro EN_RXADDR
#define RF_EN_RXADDR           0x02
#define RF_ERX_P5_BIT          5
#define RF_ERX_P4_BIT          4
#define RF_ERX_P3_BIT          3
#define RF_ERX_P2_BIT          2
#define RF_ERX_P1_BIT          1
#define RF_ERX_P0_BIT          0

// posiciones del bit en el registro SETUP_AW
#define RF_SETUP_AW            0x03
#define RF_AW_LOW_BIT          1
#define RF_AW_HIGH_BIT         0

// posiciones del bit en el registro RF_CH
#define RF_CHANNEL             0x05

// posiciones del bit en el registro SETUP
#define RF_SETUP               0x06
#define RF_DR_LOW_BIT          5
#define RF_DR_HIGH_BIT         3
#define RF_PWR_LOW_BIT         2
#define RF_PWR_HIGH_BIT        1

// posiciones del bit en el registro RPD
#define RX_RPD                 0x09

// posiciones del bit en el registro FEATURE
#define RF_FEATURE             0x1D
#define RF_EN_DYN_ACK_BIT      0
#define RF_EN_ACK_PAY_BIT      1
#define RF_EN_DPL_BIT          2

// posiciones del bit en el registro DYNPD
#define RF_DYNPD               0x1C
#define RF_DPL_P5_BIT          5
#define RF_DPL_P4_BIT          4
#define RF_DPL_P3_BIT          3
#define RF_DPL_P2_BIT          2
#define RF_DPL_P1_BIT          1
#define RF_DPL_P0_BIT          0

// posiciones del bit en el registro SETUP_RETR
#define RF_SETUP_RETR          0x04
#define RF_ARD_BIT_MASK        0xF0
#define RF_ARC_BIT_MASK        0x0F

// posiciones del bit en el registro STATUS
#define RF_STATUS              0x07
#define RF_TX_FULL_BIT         0
#define RF_MAX_RT_BIT          4
#define RF_TX_DS_BIT           5
#define RF_RX_DR_BIT           6
#define RF_RX_P_NO_BIT         0xE

// posiciones del bit en el registro OBSERVE_TX
#define OBSERVE_TX             0x08
#define PLOS_CNT               0xF0
#define ARC_CNT                0x0F

// posiciones del bit en el registro RX_ADDR_P0
#define RF_RX_ADDR_P0          0x0A
#define RF_RX_ADDR_P1          0x0B
#define RF_RX_ADDR_P2          0x0C
#define RF_RX_ADDR_P3          0x0D
#define RF_RX_ADDR_P4          0x0E
#define RF_RX_ADDR_P5          0x0F

// mapa de los registros del modulo
#define RF_TX_ADDR             0x10
#define RF_FIFO_ST             0x17

// Velocidades de transmision de datos en el aire
#define DATARATE_1Mbps      0x00
#define DATARATE_2Mbps      0x08
#define DATARATE_250Kbps    0x20
#define DATARATE_MASK       0x28

// Rango de Operacion y frecuencias en MHz
#define RF_Min_MHz             2400
#define RF_Max_MHz             2525
#define RF_MaxChannels_1Mbps   (RF_Max_MHz - RF_Min_MHz)
#define RF_MaxChannels_2Mbps   ((RF_Max_MHz - RF_Min_MHz) / 3)

// Niveles de Potencia de Transmision
#define TX_LEVEL_00dBm          0x06
#define TX_LEVEL_06dBm          0x04
#define TX_LEVEL_12dBm          0x02
#define TX_LEVEL_18dBm          0x00
#define TX_LEVEL_MASK           0x06

// modos de confirmacion de envio con o sin carga
#define ACK_AND_PAYLOAD        1
#define ACK_AND_NO_PAYLOAD     0

// tamaño maximo de bytes en FIFO (RX y TX)
#define MAXFIFOSIZE            32

// canales logicos (pipes)
#define PTX01                  0x00     //pipe0
#define PTX02                  0x01     //pipe1
#define PTX03                  0x02     //pipe2
#define PTX04                  0x03     //pipe3
#define PTX05                  0x04     //pipe4
#define PTX06                  0x05     //pipe5
#define PTXAll                 0x3F     //todos
#define PRXMode                0x64     //Modo PRX



class RF24L01P {
public:
    
    RF24L01P(uint8_t CE_pin, uint8_t CSN_Pin);
    bool dataFlag = false;
    void initialize();
    void setChecksum(uint8_t value = 2);
    void setRate(uint8_t value = DATARATE_2Mbps);
    void setRadioFrequency(uint8_t value = 40);
    void setTXLevel(uint8_t value = TX_LEVEL_00dBm);
    void setDeviceAddress(uint8_t Addr1, uint8_t Addr2, uint8_t Addr3, uint8_t Addr4, uint8_t Addr5);
    void setPRXmode();
    void setPTXmode(uint8_t channel = PTX01);
    bool receive(void* rxData, uint8_t* ptxSender, uint8_t packetSize);
    bool send(const void* txData, uint8_t packetSize);
    bool send_noAck(const void* txData, uint8_t packetSize);
    void get_ackPayload(void* rxData, uint8_t packetSize); // complemento de send
    void send_ackPayload(const void* txData, uint8_t pipeChannel, uint8_t packetSize); // complemento de Receive)



    // ***************************
    // de prueba  (en desarrollo)
    //
            void tx(const void *txData, uint8_t packetSize);
    //
    // ***************************
    
    
    
    

private:
    uint8_t CEpin, CSNpin;
    void setPipeChannel(uint8_t pipe);
    void sendPulse(uint8_t value);
    void setEnhancedShockBurstDynamic();
    void initDeviceAddress();
    void setDynamicFeature(bool value = true);
    void setAutoAckOnPipe(uint8_t pipe = PTXAll, uint8_t turn = HIGH);
    void setDynamicPayloadOnPipe(uint8_t pipe = PTXAll, uint8_t turn = HIGH);
    void setRxAddressOnPipe(uint8_t pipe = PTXAll, uint8_t turn = HIGH);
    void setAutoRetransmit(uint8_t tries = 15, uint16_t timelapsed = 1500);
    void setPowerDown();
    //bool testDevice();
    bool getDynamicFeature();
    uint8_t setDevice(uint8_t regAddr, uint8_t* pBuff, uint8_t len);
    uint8_t getDevice(uint8_t regAddr, uint8_t* pBuff, uint8_t len);
    uint8_t getLostPackets();
    uint8_t getRetriesPackets();
    uint8_t getChecksum();
    uint8_t getFifoCount();
    uint8_t getTXLevel();
    uint8_t getReceivedPowerDetector();
    uint8_t getRadioFrequency();
    uint8_t getRate();

    

};

#endif // _RF24L01P_H_



