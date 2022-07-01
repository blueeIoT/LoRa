#include <BlueeLoRa.h>

BlueeLoRa LoRa;     //Objeto LoRa
int timeOut, count;

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
       LoRa.send(count++);                  //Se envía el valor de un contador
       Serial.print("send: ");
       Serial.println(count);
    }
    delay(1);
}
