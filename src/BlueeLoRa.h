/*
 Name:		BlueeLoRa.h
 Created:	20/05/2022 19:09:24
 Author:	dabe
 Editor:	http://www.visualmicro.com
*/

#ifndef _BlueeLoRa_h
#define _BlueeLoRa_h

#if defined(ARDUINO) && ARDUINO >= 100
	#include "arduino.h"
#else
	#include "WProgram.h"
#endif

#include <SPI.h>

#define LORA_DEFAULT_SPI			SPI
#define LORA_DEFAULT_SPI_FREQUENCY	8E6 
#define LORA_DEFAULT_SS_PIN			10
#define LORA_DEFAULT_RESET_PIN		9
#define LORA_DEFAULT_DIO0_PIN		2

#define PA_OUTPUT_RFO_PIN			0
#define PA_OUTPUT_PA_BOOST_PIN		1

#define DEFAULT_BROADCAST_ID		0xFF

// registers
#define REG_FIFO					0x00
#define REG_OP_MODE					0x01
#define REG_FRF_MSB					0x06
#define REG_FRF_MID					0x07
#define REG_FRF_LSB					0x08
#define REG_PA_CONFIG				0x09
#define REG_OCP						0x0b
#define REG_LNA						0x0c
#define REG_FIFO_ADDR_PTR			0x0d
#define REG_FIFO_TX_BASE_ADDR		0x0e
#define REG_FIFO_RX_BASE_ADDR		0x0f
#define REG_FIFO_RX_CURRENT_ADDR	0x10
#define REG_IRQ_FLAGS_MASK			0x11 
#define REG_IRQ_FLAGS				0x12
#define REG_RX_NB_BYTES				0x13
#define REG_PKT_SNR_VALUE			0x19
#define REG_PKT_RSSI_VALUE			0x1a
#define REG_RSSI_VALUE				0x1b
#define REG_MODEM_CONFIG_1			0x1d
#define REG_MODEM_CONFIG_2			0x1e
#define REG_PREAMBLE_MSB			0x20
#define REG_PREAMBLE_LSB			0x21
#define REG_PAYLOAD_LENGTH			0x22
#define REG_MODEM_CONFIG_3			0x26
#define REG_FREQ_ERROR_MSB			0x28
#define REG_FREQ_ERROR_MID			0x29
#define REG_FREQ_ERROR_LSB			0x2a
#define REG_RSSI_WIDEBAND			0x2c
#define REG_DETECTION_OPTIMIZE		0x31
#define REG_INVERTIQ				0x33
#define REG_DETECTION_THRESHOLD		0x37
#define REG_SYNC_WORD				0x39
#define REG_INVERTIQ2				0x3b
#define REG_DIO_MAPPING_1			0x40
#define REG_VERSION					0x42
#define REG_PA_DAC					0x4d

// modes
#define MODE_LONG_RANGE_MODE		0x80
#define MODE_SLEEP					0x00
#define MODE_STDBY					0x01
#define MODE_TX						0x03
#define MODE_RX_CONTINUOUS			0x05
#define MODE_RX_SINGLE				0x06
#define MODE_CAD					0x07

// PA config
#define PA_BOOST					0x80

// IRQ masks
#define IRQ_TX_DONE_MASK			0x08
#define IRQ_PAYLOAD_CRC_ERROR_MASK	0x20
#define IRQ_RX_DONE_MASK			0x40
#define IRQ_CAD_DETECT_MASK			0x01
#define IRQ_CAD_DONE_MASK			0x04

#define RF_MID_BAND_THRESHOLD		525E6
#define RSSI_OFFSET_HF_PORT			157
#define RSSI_OFFSET_LF_PORT			164

#define MAX_PKT_LENGTH				250
#define DEFAULT_SIZE_BUFFER			5

#define ISR_PREFIX

class BlueeLoRaProtocol {

public:

	BlueeLoRaProtocol(byte network = DEFAULT_BROADCAST_ID, byte destination = DEFAULT_BROADCAST_ID, byte source = DEFAULT_BROADCAST_ID);
	~BlueeLoRaProtocol();
	void freeMem();
	void clear();
	void clearData();
	void setNetworkID(byte id);
	void setDestinationID(byte id);
	void setSourceID(byte id);
	byte getNetworkID();
	byte getDestinationID();
	byte getSourceID();
	void setData(byte data[], int length);
	void setData(String data);
	void addData(byte data);
	String getData();
	char* getDataAsBuffer();
	int getDataSize();
	char* getBuffer();
	char getDataOnBuffer(int i);
	char get(int i);

private:
	char * dataBuffer;
	int size;

};


class BlueeLoRa : public Stream {

public:
	BlueeLoRa();
	int begin(long frequency);
	void end();
	int beginPacket(int implicitHeader = false);
	int endPacket(bool async = false);
	int parsePacket(int size = 0);
	int packetRssi();
	float packetSnr();
	long packetFrequencyError();
	int rssi();
	
	// from Print
	virtual size_t write(uint8_t byte);
	virtual size_t write(const uint8_t* buffer, size_t size);
	
	// from Stream
	virtual int available();
	virtual int read();
	virtual int peek();

	void setValidateIDs(bool validate);
	void onReceive(void(*callback)(int));
	void receive(int size = 0);
	void idle();
	void sleep();
	void setTxPower(int level, int outputPin = PA_OUTPUT_PA_BOOST_PIN);
	void setFrequency(long frequency);
	void setSpreadingFactor(int sf);
	void setSignalBandwidth(long sbw);
	void setCodingRate4(int denominator);
	void setPreambleLength(long length);
	void setSyncWord(int sw);
	void enableCrc();
	void disableCrc();
	void enableInvertIQ();
	void disableInvertIQ();
	void setOCP(uint8_t mA); // Over Current Protection control
	void setGain(uint8_t gain); // Set LNA gain
	void send(BlueeLoRaProtocol & data, bool continueReceiving = true);
	void send(String data, bool continueReceiving = true);
	void send(int value, bool continueReceiving = true);
	void getReceived(BlueeLoRaProtocol & dataReceived);
	String getReceivedData();

	// deprecated
	void crc() { enableCrc(); }
	void noCrc() { disableCrc(); }

	byte random();
	void setIDs(byte networkID = DEFAULT_BROADCAST_ID, byte destinationID = DEFAULT_BROADCAST_ID, byte sourceID= DEFAULT_BROADCAST_ID);
	void setPins(int ss = LORA_DEFAULT_SS_PIN, int reset = LORA_DEFAULT_RESET_PIN, int dio0 = LORA_DEFAULT_DIO0_PIN);
	void setSPI(SPIClass& spi);
	void setSPIFrequency(uint32_t frequency);

private:
	void setupInterruptsCallback();
	void explicitHeaderMode();
	void implicitHeaderMode();
	void handleDio0Rise();
	void modeCAD();
	bool isTransmitting();
	void setLdoFlag();
	void writeRegister(uint8_t address, uint8_t value);
	bool processRxData();
	uint8_t readRegister(uint8_t address);
	uint8_t singleTransfer(uint8_t address, uint8_t value);
	int getSpreadingFactor();
	long getSignalBandwidth();
	static void onDio0Rise();

public:
	BlueeLoRaProtocol dataTx;
	BlueeLoRaProtocol dataRx;

private:
	bool isReceiving;
	bool isWaitCAD;
	bool cadDone;
	bool validateIDs;
	bool isContinueReceiving;
	SPISettings _spiSettings;
	SPIClass* _spi;
	int _ss;
	int _reset;
	int _dio0;
	long _frequency;
	int _packetIndex;
	int _implicitHeaderMode;
	void (*_onReceive)(int);
};

#endif

