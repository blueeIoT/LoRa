# Bluee LoRa

Librería para Arduino
===========================================
Descarga la librería y conecta tus proyectos a redes LoRa y a diversas plataformas en la nube con **bluee** .

Visita http://wwww.bluee.com.mx para más información.

- Librería para *BlueeLoRa*
- Basada en la librería de Arduino [LoRa]

# Inicialización

Antes de iniciar, declara un objeto de tipo **BlueeLoRa** como variable global:
``` C++
    BlueeLoRa LoRa;
````
Define los pines de control e interrupción con el dispositivo LoRa dentro de la función **setup()**. *CS, Reset, IRQ*.
``` C++
    LoRa.setPins(csPin, resetPin, irqPin); 
````
Define ID´s de control de red para enviar y recibir datos entre dispositivos LoRa dentro de la función **setup()**. *Utiliza 0xFF para enviar y recibir como broadcast*.
``` C++
    LoRa.setIDs(networkID, destinationID, sourceID);
````
Adiciona el callback para leer datos provenientes de la red LoRa.
``` C++
    LoRa.onReceive(onReceive);
````

- *Debes utilizar el callback para recibir datos*.

Debes declarar la funcion para recibir los datos:

- Función que se dispara al obtener datos de LoRa. *packetSize - cantidad de datos a recibidos*.
``` C++
	void onReceive(int packetSize) {
		...
	}
````

# Envío de datos

Utiliza la función **LoRa.send()** para enviar datos por medio de LoRa. *El parámetro continueReceiving te sirve para seguir recibiendo datos despues de enviar. Debes haber instanciado el callback de recepción de datos.*

*Considerar que solo se pueden enviar máximo 250 caracteres por trama.*

``` C++
	void loop() {
		LoRa.send("Hola desde LoRa");
		...
	}
````

# Recepción de datos
Utiliza la función **LoRa.getReceivedData()** para recibir datos de LoRa. *Los datos se regresan en tipo de dato String.*
``` C++
	void onReceive(int packetSize) {
		Serial.println("Recibido: " + LoRa.getReceivedData());
	}
````

# Objeto de datos BlueeLoRaProtocol

El objeto *BlueeLoRaProtocol* permite definir los ID´s de envío y leer los ID´s de recepción en las transmisiones LoRa.

*Declara las variables necesarias BlueeLoRaProtocol de forma global.*

- Crea una variable de tipo *BlueeLoRaProtocol* para utlizarlo.
``` C++
	BlueeLoRaProtocol data;
	BlueeLoRaProtocol data(networkID, destinationID, sourceID);
````

## Protocolo de datos

- La estructura de datos es:
``` C++
	[0] -> networkID
	[1] -> destinationID
	[2] -> sourceID
	[3] -> size of payload (250 bytes max)
	[..] -> payload
```

## Asignación de datos

- Asigna ID´s: *Los ID´s deben ser de 1 byte (0 a 255).*
``` C++
	data.setNetworkID(networkID);
	data.setDestinationID(destinationID);
	data.setSourceID(sourceID);
```

- Nota: **El valor 0xFF es definido como BroadcastID, por lo que todos escuchan si se envia como identificador de red o destino.**

- Asigna datos tipo *String*:
``` C++
	data.setData("hola");
```
- Asigna datos como *buffer* de tipo *byte*:
``` C++
	byte values[] = {0x02, 0x03, 0x04};
	data.setData(values, sizeof(values));
```
- Agrega valores tipo *byte*, uno por uno:
``` C++
	data.addData(0x55);
	data.addData('L');
	data.addData('o');
	data.addData('R');
	data.addData('a');
```
- Borra todos los datos:
``` C++
	data.clear();
```

## Lectura de datos

*Para recibir datos el destinationID del transmisor debe ser el sourceID del receptor o utilizar el ID Broadcast.*

- Lectura de toda la trama de tipo **BlueeLoRaProtocol** en el callback de recepción de datos:
``` C++
	BlueeLoRaProtocol data;
	LoRa.getReceived(data);
```
- Lectura de ID´s como **Int**:
``` C++
	int networkID = data.getNetworkID();
	int destinationID = data.getDestinationID();
	int sourceID = data.getSourceID();
```
- Lectura de datos como **String**:
``` C++
	String values = data.getData();
```
- Lectura de datos como buffer de tipo **Byte**:
``` C++
	byte value = data.getDataOnBuffer(position);
	...
	for (int i = 0; i < data.getDataSize(); i++) {
		Serial.print(data.getDataOnBuffer(i), HEX);       
	}
```

# Formato JSON

El traspaso de datos con las plataformas en la nube requieren sea en formato **JSON**. BlueeLoRa soporta enviar y recibir datos en este formado. Puedes utilizar la librería [ArduinoJson] para crearlos y leer valores provenientes de este formato.

- Puedes leer acerca de este formato en [JSON].

## Agregar un JSON como valor de un parámetro:

Utilizando la librería de [ArduinoJson], crear un JSON:
``` C++
	String dataJson;  
	StaticJsonDocument<200> doc;
	doc["nameOfValue"] = value
	serializeJson(doc, dataJson);
```
JSON output:
``` C++
	{ "nameOfValue" : value }
```
Agregar JSON como parámetro:
``` C++
	LoRa.send(dataJson);
```


# Documentación

Revisa la [documentación] de tu dispositivo BlueeLora para más información.
 
[documentación]: <https://bluee.com.mx/pages/ecosistema/documentacion>
  
[ArduinoJson]: <https://arduinojson.org/>

[JSON]: <https://www.json.org/json-es.html>

[LoRa]: <https://github.com/sandeepmistry/arduino-LoRa>