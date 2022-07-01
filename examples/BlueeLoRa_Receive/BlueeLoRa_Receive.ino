#include "BlueeLoRa.h"

BlueeLoRa LoRa;     //Objeto LoRa

void setup() {
    Serial.begin(9600);   
    LoRa.setPins(10, 5, 2);                 // pines CS, reset, IRQ
    LoRa.onReceive(onReceive);              // Callback de recepción de datos
    if (!LoRa.begin(915E6)) {               // Iniciación de LoRa a 915 MHz
        Serial.println("LoRa error.");
        while (true);
    }
    Serial.println("LoRa inicializado.");
}

void loop() {  
}

void onReceive(int packetSize) {
    Serial.println("Recibido: " + LoRa.getReceivedData());  //Recepción de datos en formato String
}
