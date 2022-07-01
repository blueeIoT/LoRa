#include "BlueeLoRa.h"

BlueeLoRa LoRa;             //Objeto LoRa

byte sourceID = 0x10;       // Dirección del dispositivo
byte destinationID = 0x11;  // Dirección del dispositivo de destino
byte networkID = 0x01;      // Identificador de la red de destino

void setup() {
    Serial.begin(9600);   
    LoRa.setPins(10, 5, 2);               // pines CS, reset, IRQ
    LoRa.setIDs(networkID, destinationID, sourceID);
    LoRa.onReceive(onReceive);              // Callback de recepción de datos
    if (!LoRa.begin(915E6)) {               // Iniciación de LoRa a 915 MHz
        Serial.println("LoRa error.");
        while (true);
    }
    Serial.println("LoRa inicializado.");
}

void loop() {
    if (timeOut++ > 2000) {
       timeOut = 0;
       LoRa.send(count);                  //Se envía el valor de un contador
       Serial.print("send: ");
       Serial.println(count);
    }
    delay(1);
}

void onReceive(int packetSize) {
    Serial.println("Recibido: " + LoRa.getReceivedData());		//Recepción de datos en formato String
}
