/***************************************************
 * Programa para conexión del Dispositivo esp8266
   al Broker EMQ, del CursoIOT, de CubaElectronica.

   IOT PIC18F2550 & ESP8266

   Autor: Ing. Yamir Hidalgo Peña
   Fecha: 05/2020

   Bajo las licencias Creative Commons.

   <-- correo: admin@cubaelectronica.com -->
 ***************************************************/
#include <Arduino.h>
#include <ESP8266WiFi.h>      // Libreria WIFI para el ESP8266
#include <PubSubClient.h>     // Libreria MQTT

//Credenciales para Conexion WIFI
#define WIFI_SSID "CRESPO"                     // SSID del WIFI
#define WIFI_PASS "16112005"                   // Password del WIFI

#define MQTT_CLIENT_NAME "ESP8266_0000001"       //Cliente ID MQTT, Base ESP32_ + DEVICE_ID
#define MQTT_CLIENT_USER "curso_iot"             //User MQTT
#define MQTT_CLIENT_PASSWORD "cubaelectronica"   //Password MQTT
/****************************************
 * Constantes & Variables
 ****************************************/
const String device_id = "0000001";         // ID del Dispositivo
char mqttBroker[] = "cubaelectronica.com";  // Direccíon del Broker 192.168.0.15
char payload[50];                           // Tamaño del mensaje
char topico[50];                            // Tamaño del topico
char topico2[50];                           // Tamaño del topico2
long lastMsg = 0;

String inputString = "";         // String para guardar el texto que llega
bool stringComplete = false;     // Booleano para indicar que la recepcion esta completada

/****************************************
 * Funciones Auxiliares
 ****************************************/
  WiFiClient espClient;
  PubSubClient client(espClient);

  /****************************************
   * Funsion recibir mensajes PubSubClient
   ****************************************/
  void callback(char *topic, byte *payload, unsigned int length)
    {
      String mensaje = "";
      Serial.print("Topico --> ");
      Serial.println(topic);

      for (int i = 0; i < length; i++) {
        mensaje += (char)payload[i];
      }

      mensaje.trim();   // Quitamos los posibles espacios en Blanco.

      Serial.println("Mensaje --> " + mensaje); // Sacamos por serial el mensaje

      String str_topic(topic); // Pasamos el Topico a una variable String

        if (str_topic == device_id + "/command"){
            if ( mensaje == "on") {
              Serial.print("led1=on*"); // Mandamos activar el LED en el DevPIC18F
              //////////////Respondemos OK al Broker///////////
              String topico_aux = device_id + "/respuesta";
              topico_aux.toCharArray(topico,25);
              client.publish(topico, "on_ok"); //Publicar respuesta on ok por MQTT
              ///////////////////////////////////////
            }
            if ( mensaje == "off") {
              Serial.print("led1=off*"); // Mandamos desactivar el LED en el DevPIC18F
              ///////////////Respondemos OK al Broker//////////
              String topico_aux = device_id + "/respuesta";
              topico_aux.toCharArray(topico,25);
              client.publish(topico, "off_ok"); //Publicar respuesta off ok por MQTT
              ///////////////////////////////////////
            }

            // Dimmer


         }
    }

    /****************************************
     * Funsion reconectar de PubSubClient
     ****************************************/
      void reconnect()
      {
          // Loop hasta se reconecta
          while (!client.connected())
          {
             Serial.println("Intentando conexión MQTT...");

             // Conexion al Servidor MQTT , ClienteID, Usuario, Password.
             // Ver documentación => https://pubsubclient.knolleary.net/api.html
             if (client.connect(MQTT_CLIENT_NAME, MQTT_CLIENT_USER, MQTT_CLIENT_PASSWORD))
                {
                  Serial.println("Conectado! a servidor MQTT CursoIOT CubaElectronica" );
                  // Nos suscribimos a comandos
                  String topico_serial = device_id + "/command";
                  topico_serial.toCharArray(topico,25);
                  client.subscribe(topico);
                  // Nos suscribimos a dimmer
                  String topico_serial2 = device_id + "/dimmer";
                  topico_serial2.toCharArray(topico2,25);
                  client.subscribe(topico2);
                }
                else
                {
                  Serial.print("Falló, Client_Status=");
                  Serial.print(client.state());
                  Serial.println(" Intentando en 2 segundos");
                  // Espero 2 segundo y lo vuelvo a intentar
                  delay(2000);
                }
          }
  }

  /****************************************
   * Configuramos la GPIO - 2
   ****************************************/
  struct InterrupcionSerial {
    const uint8_t PIN;
    bool active;
  };

  /****************************************/
  InterrupcionSerial interrupcionSerial = {2, false};
  char pin2 = 0;

  void IRAM_ATTR in1() {
     interrupcionSerial.active = true;
  }

  /****************************************
    * Función para separar un String por un un separador en especifico.
    ****************************************/
   String getStringPartByNr(String data, char separator, int index)
   {
       // Dividir una cadena y devolver el índice de parte Nr
       // Dividir por separador

       int stringData = 0;        // Variable para contar cantidad de datos Nr
       String dataPart = "";      // Variable para colocar el texto de retorno

       for(int i = 0; i<data.length()-1; i++) {    // Recorrer el texto letra por letra

         if(data[i]==separator) {
           // Cuenta el número de veces que el carácter separador aparece en el texto
           stringData++;

         }else if(stringData==index) {
           // Obtener el texto cuando el separador es el elegido
           dataPart.concat(data[i]);

         }else if(stringData>index) {
           // Devolver texto y detener si aparece el siguiente separador, para ahorrar tiempo de CPU
           return dataPart;
           break;

         }

       }
       // Devolver texto si esta es la última parte
       return dataPart;
   }

   /****************************************
    * Proceso el mensaje UART y lo envio por MQTT
    ****************************************/
   void enviamqtt(){

        if (stringComplete) {
             //Sacamos, del, String, que, llega, los datos separados por comas (,)
             //   0   ,  1 ,  INDEX n
             String temp = getStringPartByNr(inputString, ',', 0); //Posición 0 Temp DS18B20
             //String Volt= getStringPartByNr(inputString, ',', 1); //Posición 1

             float wifi = WiFi.RSSI(); // Capturamos el nivel de la señal WIFI
             String ip = WiFi.localIP().toString(); //Capturamos el IP de la red local

             // Publicamos el Topic con los valores, RSSI, IP.
             //               TempCPU ,   TempDHT ,      WIFI RSSI     ,   HumDHT  , IndTermDHT ,  IPLocal
             String to_send = temp + "," + "0" + "," + String(wifi) + "," + "0" + "," + "0" + "," + ip;
             to_send.toCharArray(payload, 50);
             String topico_aux = device_id + "/valores";
             topico_aux.toCharArray(topico,50);
             client.publish(topico, payload);

           }
       }

       /****************************************
        * Main Functions
        ****************************************/
       void setup()
       {
         Serial.begin(115200); // Iniciamos el Puerto Serial

         WiFi.begin(WIFI_SSID, WIFI_PASS); // Nos intentamos conectar al WIFI
         Serial.println();
         Serial.print("Esperando conexión WiFi...");

         while (WiFi.status() != WL_CONNECTED)
         {
           Serial.print(".");  // Mientras no conecte estamos dibujando ...... por serial
           delay(500); // Cada 500ms
         }

         // Mensajes por serial cuando nos conectamos
         Serial.println("");
         Serial.println("WiFi Connected");
         Serial.println("IP address: ");
         Serial.println(WiFi.localIP());

         client.setServer(mqttBroker, 1883); //Nos conectamos al Broker, Servidor, Puerto.
         client.setCallback(callback);

         pinMode(interrupcionSerial.PIN, INPUT_PULLUP); //Activar las resistencias PullUp internas
         //Activar la Interrupcion flanco de bajada (Cuando pasa de HIGH a LOW)
         attachInterrupt(interrupcionSerial.PIN, in1, FALLING);

       }

       void loop()
       {
         if (!client.connected())
         {
           reconnect(); //si no estamos conectados al Broker nos intentamos conectar.
         }

           //Si hay interrupciones por el GPIO 2
           if (interrupcionSerial.active) {
                while (Serial.available()) { // Mientras que el puerto serial este recibiendo datos
                     // Optenemos el nuevo dato que llega:
                     char inChar = (char)Serial.read();
                     // Agregamos ese dato al String para formar una cadena:
                     inputString += inChar;
                     // Seguimos haciendo esto hasta que nos llega una nueva linea
                         if (inChar == '\n') {
                           stringComplete = true; // Ponemos en verdadero la recepcion completa
                         }
       
                 }

                 enviamqtt(); // llamamos la función de enviar lo recivido por Serial al Broker MQTT
                 inputString = "";                   // Limpiamos la cadena una ves se envio.
                 interrupcionSerial.active = false;  // Limpiamos la vandera de interrupción.

             }

           client.loop(); // Loop del Cliente MQTT

       } // FIN
