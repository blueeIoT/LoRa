/*
 Name:		BlueeLoRa.cpp
 Created:	20/05/2022 19:09:24
 Author:	dabe
 Editor:	http://www.visualmicro.com
*/

#include "BlueeLoRa.h"



/*********************************************************************************/

BlueeLoRaProtocol::BlueeLoRaProtocol(byte network, byte destination, byte source)
{
    clear();
    setNetworkID(network);
    setDestinationID(destination);
    setSourceID(source);
}

BlueeLoRaProtocol::~BlueeLoRaProtocol()
{
    freeMem();
}

void BlueeLoRaProtocol::freeMem()
{
    free(dataBuffer);
    dataBuffer = NULL;
}

void BlueeLoRaProtocol::clear()
{
    freeMem();
    dataBuffer = (char*) calloc(DEFAULT_SIZE_BUFFER, sizeof(char));
    memset(dataBuffer, 0, DEFAULT_SIZE_BUFFER);
    size = 0;
}

void BlueeLoRaProtocol::clearData()
{
    int idN = getNetworkID();
    int idD = getDestinationID();
    int idS = getSourceID();
    clear();
    setNetworkID(idN);
    setDestinationID(idD);
    setSourceID(idS);
}

void BlueeLoRaProtocol::setNetworkID(byte id)
{
    dataBuffer[0] = id;
}

void BlueeLoRaProtocol::setDestinationID(byte id)
{
    dataBuffer[1] = id;
}

void BlueeLoRaProtocol::setSourceID(byte id)
{
    dataBuffer[2] = id;
}

byte BlueeLoRaProtocol::getNetworkID()
{
    return dataBuffer[0];
}

byte BlueeLoRaProtocol::getDestinationID()
{
    return dataBuffer[1];
}

byte BlueeLoRaProtocol::getSourceID()
{
    return dataBuffer[2];
}

void BlueeLoRaProtocol::setData(byte data[], int length)
{ 
    dataBuffer = (char *) realloc(dataBuffer, length + DEFAULT_SIZE_BUFFER);
    dataBuffer[3] = length;
    memcpy(&dataBuffer[4], data, length);
}

void BlueeLoRaProtocol::setData(String data)
{
    setData((byte*)&data[0], data.length());
}

void BlueeLoRaProtocol::addData(byte data)
{
    dataBuffer[3]++;
    dataBuffer = (char*)realloc(dataBuffer, DEFAULT_SIZE_BUFFER + dataBuffer[3]);
    memcpy(&dataBuffer[dataBuffer[3] + DEFAULT_SIZE_BUFFER - 2], &data, 1);
}

String BlueeLoRaProtocol::getData()
{
    String dataS;
    char* data = (char *) calloc (dataBuffer[3] + 1, sizeof(char));
    memcpy(&data[0], &dataBuffer[4], dataBuffer[3]);
    dataS = (String) data;
    free(data);
    return dataS;
}

char* BlueeLoRaProtocol::getDataAsBuffer()
{
    return &dataBuffer[4];
}

int BlueeLoRaProtocol::getDataSize()
{
    return (int) dataBuffer[3];
}

char * BlueeLoRaProtocol::getBuffer()
{
    return dataBuffer;
}

char BlueeLoRaProtocol::getDataOnBuffer(int i)
{
    return dataBuffer[4 + i];
}

char BlueeLoRaProtocol::get(int i)
{
    return dataBuffer[i];
}

/************************************************************************/
BlueeLoRa * pLoRa;

BlueeLoRa::BlueeLoRa() :
    validateIDs(true),
    isReceiving(false),
    isWaitCAD(false),
    isContinueReceiving(false),
    _spiSettings(LORA_DEFAULT_SPI_FREQUENCY, MSBFIRST, SPI_MODE0),
    _spi(&LORA_DEFAULT_SPI),
    _ss(LORA_DEFAULT_SS_PIN), _reset(LORA_DEFAULT_RESET_PIN), _dio0(LORA_DEFAULT_DIO0_PIN),
    _frequency(0),
    _packetIndex(0),
    _implicitHeaderMode(0),
    _onReceive(NULL)
{
    pLoRa = this;
    setTimeout(10);
    dataTx.clear();
    dataRx.clear();
    setIDs();
}

int BlueeLoRa::begin(long frequency)
{
    pinMode(_ss, OUTPUT);
    digitalWrite(_ss, HIGH);
    if (_reset != -1) {
        pinMode(_reset, OUTPUT);
        digitalWrite(_reset, LOW);
        delay(10);
        digitalWrite(_reset, HIGH);
        delay(10);
    }
    _spi->begin();
    sleep();
    setFrequency(frequency);
    writeRegister(REG_FIFO_TX_BASE_ADDR, 0);
    writeRegister(REG_FIFO_RX_BASE_ADDR, 0);
    writeRegister(REG_LNA, readRegister(REG_LNA) | 0x03);
    writeRegister(REG_MODEM_CONFIG_3, 0x04);
    setTxPower(17);
    idle(); 
    setupInterruptsCallback();
    return 1;
}

void BlueeLoRa::end()
{
    sleep();
    _spi->end();
}

int BlueeLoRa::beginPacket(int implicitHeader)
{
    if (isTransmitting()) {
        return 0;
    }
    idle();
    if (implicitHeader) {
        implicitHeaderMode();
    }
    else {
        explicitHeaderMode();
    }
    writeRegister(REG_FIFO_ADDR_PTR, 0);
    writeRegister(REG_PAYLOAD_LENGTH, 0);
    return 1;
}

int BlueeLoRa::endPacket(bool async)
{
    if ((async)) {
        writeRegister(REG_DIO_MAPPING_1, 0x40); // DIO0 => TXDONE
        writeRegister(REG_IRQ_FLAGS_MASK, 0x00);
    }
    writeRegister(REG_OP_MODE, MODE_LONG_RANGE_MODE | MODE_TX);
    if (!async) {
        while ((readRegister(REG_IRQ_FLAGS) & IRQ_TX_DONE_MASK) == 0) {
            yield();
        }
        writeRegister(REG_IRQ_FLAGS, IRQ_TX_DONE_MASK);
    }
    return 1;
}

void BlueeLoRa::modeCAD()
{
    isWaitCAD = true;
    writeRegister(REG_OP_MODE, MODE_LONG_RANGE_MODE | MODE_CAD);
    writeRegister(REG_DIO_MAPPING_1, 0x80); // DIO0 => CADDONE
    int timeOut = 5000;
    do {
        delay(1);
        if (!isWaitCAD) {
            break;
        }
    } while (--timeOut);
}

bool BlueeLoRa::isTransmitting()
{
    if ((readRegister(REG_OP_MODE) & MODE_TX) == MODE_TX) {
        return true;
    }
    if (readRegister(REG_IRQ_FLAGS) & IRQ_TX_DONE_MASK) {
        writeRegister(REG_IRQ_FLAGS, IRQ_TX_DONE_MASK);
    }
    return false;
}

int BlueeLoRa::parsePacket(int size)
{
    int packetLength = 0;
    int irqFlags = readRegister(REG_IRQ_FLAGS);

    if (size > 0) {
        implicitHeaderMode();

        writeRegister(REG_PAYLOAD_LENGTH, size & 0xff);
    }
    else {
        explicitHeaderMode();
    }
    writeRegister(REG_IRQ_FLAGS, irqFlags);
    if ((irqFlags & IRQ_RX_DONE_MASK) && (irqFlags & IRQ_PAYLOAD_CRC_ERROR_MASK) == 0) {
        _packetIndex = 0;
        if (_implicitHeaderMode) {
            packetLength = readRegister(REG_PAYLOAD_LENGTH);
        }
        else {
            packetLength = readRegister(REG_RX_NB_BYTES);
        }
        writeRegister(REG_FIFO_ADDR_PTR, readRegister(REG_FIFO_RX_CURRENT_ADDR));
        idle();
    }
    else if (readRegister(REG_OP_MODE) != (MODE_LONG_RANGE_MODE | MODE_RX_SINGLE)) {
        writeRegister(REG_FIFO_ADDR_PTR, 0);
        writeRegister(REG_OP_MODE, MODE_LONG_RANGE_MODE | MODE_RX_SINGLE);
    }
    return packetLength;
}

int BlueeLoRa::packetRssi()
{
    return (readRegister(REG_PKT_RSSI_VALUE) - (_frequency < RF_MID_BAND_THRESHOLD ? RSSI_OFFSET_LF_PORT : RSSI_OFFSET_HF_PORT));
}

float BlueeLoRa::packetSnr()
{
    return ((int8_t)readRegister(REG_PKT_SNR_VALUE)) * 0.25;
}

long BlueeLoRa::packetFrequencyError()
{
    int32_t freqError = 0;
    freqError = static_cast<int32_t>(readRegister(REG_FREQ_ERROR_MSB) & B111);
    freqError <<= 8L;
    freqError += static_cast<int32_t>(readRegister(REG_FREQ_ERROR_MID));
    freqError <<= 8L;
    freqError += static_cast<int32_t>(readRegister(REG_FREQ_ERROR_LSB));

    if (readRegister(REG_FREQ_ERROR_MSB) & B1000) { // Sign bit is on
        freqError -= 524288; // B1000'0000'0000'0000'0000
    }

    const float fXtal = 32E6; // FXOSC: crystal oscillator (XTAL) frequency (2.5. Chip Specification, p. 14)
    const float fError = ((static_cast<float>(freqError) * (1L << 24)) / fXtal) * (getSignalBandwidth() / 500000.0f); // p. 37

    return static_cast<long>(fError);
}

int BlueeLoRa::rssi()
{
    return (readRegister(REG_RSSI_VALUE) - (_frequency < RF_MID_BAND_THRESHOLD ? RSSI_OFFSET_LF_PORT : RSSI_OFFSET_HF_PORT));
}

size_t BlueeLoRa::write(uint8_t byte)
{
    return write(&byte, sizeof(byte));
}

size_t BlueeLoRa::write(const uint8_t* buffer, size_t size)
{
    int currentLength = readRegister(REG_PAYLOAD_LENGTH);
    if ((currentLength + size) > MAX_PKT_LENGTH) {
        size = MAX_PKT_LENGTH - currentLength;
    }
    for (size_t i = 0; i < size; i++) {
        writeRegister(REG_FIFO, buffer[i]);
    }
    writeRegister(REG_PAYLOAD_LENGTH, currentLength + size);
    return size;
}

int BlueeLoRa::available()
{
    return (readRegister(REG_RX_NB_BYTES) - _packetIndex);
}

int BlueeLoRa::read()
{
    if (!available()) {
        return -1;
    }
    _packetIndex++;
    return readRegister(REG_FIFO);
}

int BlueeLoRa::peek()
{
    if (!available()) {
        return -1;
    }
    int currentAddress = readRegister(REG_FIFO_ADDR_PTR);
    uint8_t b = readRegister(REG_FIFO);
    writeRegister(REG_FIFO_ADDR_PTR, currentAddress);
    return b;
}

void BlueeLoRa::setValidateIDs(bool validate)
{
    validateIDs = validate;
}

void BlueeLoRa::onReceive(void(*callback)(int))
{
    _onReceive = callback;
    isContinueReceiving = true;
}

void BlueeLoRa::setupInterruptsCallback()
{
    pinMode(_dio0, INPUT);
    if (_onReceive) {
#ifdef SPI_HAS_NOTUSINGINTERRUPT
        SPI.usingInterrupt(digitalPinToInterrupt(_dio0));
#endif
        attachInterrupt(digitalPinToInterrupt(_dio0), BlueeLoRa::onDio0Rise, RISING);
    }   else {
        detachInterrupt(digitalPinToInterrupt(_dio0));
#ifdef SPI_HAS_NOTUSINGINTERRUPT
        SPI.notUsingInterrupt(digitalPinToInterrupt(_dio0));
#endif
    }
    if (isContinueReceiving) {
        receive();
    }
}
void BlueeLoRa::receive(int size)
{
    if (isReceiving) return;
    writeRegister(REG_DIO_MAPPING_1, 0x00); // DIO0 => RXDONE
    writeRegister(REG_IRQ_FLAGS_MASK, 0x00);
    if (size > 0) {
        implicitHeaderMode();
        writeRegister(REG_PAYLOAD_LENGTH, size & 0xff);
    }
    else {
        explicitHeaderMode();
    }
    isReceiving = true;
    writeRegister(REG_OP_MODE, MODE_LONG_RANGE_MODE | MODE_RX_CONTINUOUS);
}


void BlueeLoRa::idle()
{
    isReceiving = false;
    writeRegister(REG_OP_MODE, MODE_LONG_RANGE_MODE | MODE_STDBY);
}

void BlueeLoRa::sleep()
{
    writeRegister(REG_OP_MODE, MODE_LONG_RANGE_MODE | MODE_SLEEP);
    delay(20);
}

void BlueeLoRa::setTxPower(int level, int outputPin)
{
    if (PA_OUTPUT_RFO_PIN == outputPin) {
        // RFO
        if (level < 0) {
            level = 0;
        }
        else if (level > 14) {
            level = 14;
        }
        writeRegister(REG_PA_CONFIG, 0x70 | level);
    }
    else {
        // PA BOOST
        if (level > 17) {
            if (level > 20) {
                level = 20;
            }
            // subtract 3 from level, so 18 - 20 maps to 15 - 17
            level -= 3;
            // High Power +20 dBm Operation (Semtech SX1276/77/78/79 5.4.3.)
            writeRegister(REG_PA_DAC, 0x87);
            setOCP(140);
        }
        else {
            if (level < 2) {
                level = 2;
            }
            //Default value PA_HF/LF or +17dBm
            writeRegister(REG_PA_DAC, 0x84);
            setOCP(100);
        }

        writeRegister(REG_PA_CONFIG, PA_BOOST | (level - 2));
    }
}

void BlueeLoRa::setFrequency(long frequency)
{
    _frequency = frequency;
    uint64_t frf = ((uint64_t)frequency << 19) / 32000000;
    writeRegister(REG_FRF_MSB, (uint8_t)(frf >> 16));
    writeRegister(REG_FRF_MID, (uint8_t)(frf >> 8));
    writeRegister(REG_FRF_LSB, (uint8_t)(frf >> 0));
}

int BlueeLoRa::getSpreadingFactor()
{
    return readRegister(REG_MODEM_CONFIG_2) >> 4;
}

void BlueeLoRa::setSpreadingFactor(int sf)
{
    if (sf < 6) {
        sf = 6;
    }
    else if (sf > 12) {
        sf = 12;
    }
    if (sf == 6) {
        writeRegister(REG_DETECTION_OPTIMIZE, 0xc5);
        writeRegister(REG_DETECTION_THRESHOLD, 0x0c);
    }
    else {
        writeRegister(REG_DETECTION_OPTIMIZE, 0xc3);
        writeRegister(REG_DETECTION_THRESHOLD, 0x0a);
    }
    writeRegister(REG_MODEM_CONFIG_2, (readRegister(REG_MODEM_CONFIG_2) & 0x0f) | ((sf << 4) & 0xf0));
    setLdoFlag();
}

long BlueeLoRa::getSignalBandwidth()
{
    byte bw = (readRegister(REG_MODEM_CONFIG_1) >> 4);
    switch (bw) {
    case 0: return 7.8E3;
    case 1: return 10.4E3;
    case 2: return 15.6E3;
    case 3: return 20.8E3;
    case 4: return 31.25E3;
    case 5: return 41.7E3;
    case 6: return 62.5E3;
    case 7: return 125E3;
    case 8: return 250E3;
    case 9: return 500E3;
    }
    return -1;
}

void BlueeLoRa::send(BlueeLoRaProtocol& data, bool continueReceiving)
{
    isReceiving = false;
    isContinueReceiving = continueReceiving;
    while (beginPacket()==0);        // start packet    
    write(data.getNetworkID());      // add network id
    write(data.getDestinationID());  // add destination address
    write(data.getSourceID());       // add sender address
    write(data.getDataSize());       // add payload length
    if (data.getDataSize() > 0) {
        for (int i = 0; i < data.getDataSize(); i++) {
            write(data.getDataOnBuffer(i));       // add payload
        }
        print(""); //force to send data when not a string
    }
    endPacket(true);
}

void BlueeLoRa::send(String data, bool continueReceiving)
{
    dataTx.clearData();
    dataTx.setData(data);
    send(dataTx, continueReceiving);
 }

void BlueeLoRa::send(int value, bool continueReceiving)
{
    String data(value);
    send(data, continueReceiving);
}

 void BlueeLoRa::getReceived(BlueeLoRaProtocol& dataReceived)
{
     dataReceived = dataRx;
}

String BlueeLoRa::getReceivedData()
{
    return dataRx.getData();
}

void BlueeLoRa::setSignalBandwidth(long sbw)
{
    int bw;

    if (sbw <= 7.8E3) {
        bw = 0;
    }
    else if (sbw <= 10.4E3) {
        bw = 1;
    }
    else if (sbw <= 15.6E3) {
        bw = 2;
    }
    else if (sbw <= 20.8E3) {
        bw = 3;
    }
    else if (sbw <= 31.25E3) {
        bw = 4;
    }
    else if (sbw <= 41.7E3) {
        bw = 5;
    }
    else if (sbw <= 62.5E3) {
        bw = 6;
    }
    else if (sbw <= 125E3) {
        bw = 7;
    }
    else if (sbw <= 250E3) {
        bw = 8;
    }
    else /*if (sbw <= 250E3)*/ {
        bw = 9;
    }

    writeRegister(REG_MODEM_CONFIG_1, (readRegister(REG_MODEM_CONFIG_1) & 0x0f) | (bw << 4));
    setLdoFlag();
}

void BlueeLoRa::setLdoFlag()
{
    // Section 4.1.1.5
    long symbolDuration = 1000 / (getSignalBandwidth() / (1L << getSpreadingFactor()));

    // Section 4.1.1.6
    boolean ldoOn = symbolDuration > 16;

    uint8_t config3 = readRegister(REG_MODEM_CONFIG_3);
    bitWrite(config3, 3, ldoOn);
    writeRegister(REG_MODEM_CONFIG_3, config3);
}

void BlueeLoRa::setCodingRate4(int denominator)
{
    if (denominator < 5) {
        denominator = 5;
    }
    else if (denominator > 8) {
        denominator = 8;
    }
    int cr = denominator - 4;
    writeRegister(REG_MODEM_CONFIG_1, (readRegister(REG_MODEM_CONFIG_1) & 0xf1) | (cr << 1));
}

void BlueeLoRa::setPreambleLength(long length)
{
    writeRegister(REG_PREAMBLE_MSB, (uint8_t)(length >> 8));
    writeRegister(REG_PREAMBLE_LSB, (uint8_t)(length >> 0));
}

void BlueeLoRa::setSyncWord(int sw)
{
    writeRegister(REG_SYNC_WORD, sw);
}

void BlueeLoRa::enableCrc()
{
    writeRegister(REG_MODEM_CONFIG_2, readRegister(REG_MODEM_CONFIG_2) | 0x04);
}

void BlueeLoRa::disableCrc()
{
    writeRegister(REG_MODEM_CONFIG_2, readRegister(REG_MODEM_CONFIG_2) & 0xfb);
}

void BlueeLoRa::enableInvertIQ()
{
    writeRegister(REG_INVERTIQ, 0x66);
    writeRegister(REG_INVERTIQ2, 0x19);
}

void BlueeLoRa::disableInvertIQ()
{
    writeRegister(REG_INVERTIQ, 0x27);
    writeRegister(REG_INVERTIQ2, 0x1d);
}

void BlueeLoRa::setOCP(uint8_t mA)
{
    uint8_t ocpTrim = 27;

    if (mA <= 120) {
        ocpTrim = (mA - 45) / 5;
    }
    else if (mA <= 240) {
        ocpTrim = (mA + 30) / 10;
    }

    writeRegister(REG_OCP, 0x20 | (0x1F & ocpTrim));
}

void BlueeLoRa::setGain(uint8_t gain)
{
    if (gain > 6) {
        gain = 6;
    }
    idle();
    if (gain == 0) {
        // if gain = 0, enable AGC
        writeRegister(REG_MODEM_CONFIG_3, 0x04);
    }
    else {
        // disable AGC
        writeRegister(REG_MODEM_CONFIG_3, 0x00);
        // clear Gain and set LNA boost
        writeRegister(REG_LNA, 0x03);
        // set gain
        writeRegister(REG_LNA, readRegister(REG_LNA) | (gain << 5));
    }
}

byte BlueeLoRa::random()
{
    return readRegister(REG_RSSI_WIDEBAND);
}


void BlueeLoRa::setIDs(byte networkID, byte destinationID, byte sourceID)
{
    dataTx.setNetworkID(networkID);
    dataTx.setDestinationID(destinationID);
    dataTx.setSourceID(sourceID);
}

void BlueeLoRa::setPins(int ss, int reset, int dio0)
{
    _ss = ss;
    _reset = reset;
    _dio0 = dio0;
}

void BlueeLoRa::setSPI(SPIClass& spi)
{
    _spi = &spi;
}

void BlueeLoRa::setSPIFrequency(uint32_t frequency)
{
    _spiSettings = SPISettings(frequency, MSBFIRST, SPI_MODE0);
}

void BlueeLoRa::explicitHeaderMode()
{
    _implicitHeaderMode = 0;

    writeRegister(REG_MODEM_CONFIG_1, readRegister(REG_MODEM_CONFIG_1) & 0xfe);
}

void BlueeLoRa::implicitHeaderMode()
{
    _implicitHeaderMode = 1;

    writeRegister(REG_MODEM_CONFIG_1, readRegister(REG_MODEM_CONFIG_1) | 0x01);
}

void BlueeLoRa::handleDio0Rise()
{
    int irqFlags = readRegister(REG_IRQ_FLAGS);
    delay(10);
    if (irqFlags != 0xFF) {
        writeRegister(REG_IRQ_FLAGS, irqFlags);
        if ((irqFlags & IRQ_CAD_DONE_MASK)) {
            isWaitCAD = false;
        }
        if ((irqFlags & IRQ_PAYLOAD_CRC_ERROR_MASK) == 0) {
            if ((irqFlags & IRQ_RX_DONE_MASK) != 0) {
                _packetIndex = 0;
                int packetLength = _implicitHeaderMode ? readRegister(REG_PAYLOAD_LENGTH) : readRegister(REG_RX_NB_BYTES);
                writeRegister(REG_FIFO_ADDR_PTR, readRegister(REG_FIFO_RX_CURRENT_ADDR));
                bool validPacket = true;
                if (packetLength != 0) {
                    if (_onReceive) {
                        if (validateIDs) {
                            validPacket = processRxData();
                        }
                        if (validPacket) {
                            _onReceive(packetLength);
                        }

                        writeRegister(REG_FIFO_ADDR_PTR, 0);
                    }
                }
            }
        }
    }
    writeRegister(REG_IRQ_FLAGS, 0xFF);
    if (!isWaitCAD) {
        if (isContinueReceiving && !isReceiving) {
            receive();
        }
    }
}

bool BlueeLoRa::processRxData() {
   
    dataRx.clear();
    dataRx.setNetworkID(read());
    dataRx.setDestinationID(read());
    dataRx.setSourceID(read());
    byte incomingLength = read();    // incoming msg length
    while (available()) {            
        dataRx.addData((char) read()); // add bytes one by one
    }
    if (dataRx.getDataSize() != incomingLength) {   // check length for error
        return false;    
    }
    // if the recipient isn't this device or broadcast or network
    if ( dataRx.getNetworkID() == dataTx.getNetworkID() || dataRx.getNetworkID() == DEFAULT_BROADCAST_ID) {
        if (dataRx.getDestinationID() == dataTx.getSourceID() || dataRx.getDestinationID() == DEFAULT_BROADCAST_ID) {
             return true;   
        }
    }
    return false;
}

uint8_t BlueeLoRa::readRegister(uint8_t address)
{
    return singleTransfer(address & 0x7f, 0x00);
}

void BlueeLoRa::writeRegister(uint8_t address, uint8_t value)
{
    singleTransfer(address | 0x80, value);
}

uint8_t BlueeLoRa::singleTransfer(uint8_t address, uint8_t value)
{
    uint8_t response;
    digitalWrite(_ss, LOW);
    _spi->beginTransaction(_spiSettings);
    _spi->transfer(address);
    response = _spi->transfer(value);
    _spi->endTransaction();
    digitalWrite(_ss, HIGH);
    return response;
}

ISR_PREFIX void BlueeLoRa::onDio0Rise()
{
    pLoRa->handleDio0Rise();
}

