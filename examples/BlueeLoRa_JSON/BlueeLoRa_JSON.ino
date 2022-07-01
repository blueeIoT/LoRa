#include <ArduinoJson.h>
#include "BlueeLoRa.h"

BlueeLoRa LoRa;     //Objeto LoRa

void setup() {
    Serial.begin(9600);   
    LoRa.setPins(10, 5, 2);                 // pines CS, reset, IRQ
    if (!LoRa.begin(915E6)) {               // Iniciación de LoRa a 915 MHz
        Serial.println("LoRa error.");
        while (true);
    }
    Serial.println("LoRa inicializado.");
}

void loop() {
    if (timeOut++ > 2000) {
       timeOut = 0;
       
       String dataJson;
       StaticJsonDocument<200> doc;
       doc["count"] = count++;
       serializeJson(doc, dataJson);
       LoRa.send(dataJson);
       
       Serial.print("send: ");
       Serial.println(dataJson);
    }
    delay(1);
}
