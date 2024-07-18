
#define AWS_IOT_SUBSCRIBE_TOPIC "esp32/config"
#define AWS_TOPIC_VRMS "esp32/vrms"
#define AWS_TOPIC_PR "esp32/potencia_real"
#define AWS_TOPIC_APAR "esp32/potencia_aparente"
#define AWS_TOPIC_POTENCIA_REACTIVA "esp32/potencia_reactiva"
#define AWS_TOPIC_FACT_POTENCIA "esp32/fact_potencia"

#define TINY_GSM_MODEM_SIM7600
#define SerialMon Serial
#define SerialAT Serial1
#define TINY_GSM_DEBUG SerialMon

#include <TinyGsmClient.h>
#include <ArduinoHttpClient.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include <FS.h>

#include <SPIFFS.h>
#include <SSLClient.h>
#include "utilities.h"
#include "certs.h"
#include <EmonLib.h>
#include "secrets.h"
#include <SSLClientESP32.h>

EnergyMonitor emon1;
EnergyMonitor emon2;
EnergyMonitor emon3;

TinyGsm modem(SerialAT);
TinyGsmClient gsmClient(modem, 0);
SSLClientESP32 ssl_client(&gsmClient);

PubSubClient clientMqtt(ssl_client);

TinyGsmClient base_client1(modem, 1);
TinyGsmClient base_client2(modem, 2);
TinyGsmClient base_client3(modem, 3);
TinyGsmClient base_client4(modem, 4);
TinyGsmClient base_client5(modem, 5);
TinyGsmClient base_client6(modem, 6);
TinyGsmClient base_client7(modem, 7);
TinyGsmClient base_client8(modem, 8);

SSLClient secure_layer1(&base_client1);
SSLClient secure_layer2(&base_client2);
SSLClient secure_layer3(&base_client3);
SSLClient secure_layer4(&base_client4);
SSLClient secure_layer5(&base_client5);
SSLClient secure_layer6(&base_client6);
SSLClient secure_layer7(&base_client7);
SSLClient secure_layer8(&base_client8);

HttpClient client1 = HttpClient(secure_layer1, server, port);
HttpClient client2 = HttpClient(secure_layer2, server, port);
HttpClient client3 = HttpClient(secure_layer3, server, port);
HttpClient client4 = HttpClient(secure_layer4, server, port);
HttpClient client5 = HttpClient(secure_layer5, server, port);
HttpClient client6 = HttpClient(secure_layer6, server, port);
HttpClient client7 = HttpClient(secure_layer7, server, port);
HttpClient client8 = HttpClient(secure_layer8, server, port);

void light_sleep(uint32_t sec)
{
  esp_sleep_enable_timer_wakeup(sec * 1000000ULL);
  esp_light_sleep_start();
}

void messageHandler(char *topic, byte *payload, unsigned int length)
{
  JsonDocument doc;
  deserializeJson(doc, (const char *)payload, length);
  // Maneja el mensaje recibido
  SerialMon.print("Mensaje recibido en el topic: ");
  SerialMon.println(topic);
  SerialMon.print("Mensaje: ");
  serializeJson(doc, SerialMon);
  SerialMon.println();
}

void connectAWS()
{

  SerialMon.println("Setting up SSL certificates");
  ssl_client.setCACert(AWS_CERT_CA);
  ssl_client.setCertificate(AWS_CERT_CRT);
  ssl_client.setPrivateKey(AWS_CERT_PRIVATE);

  SerialMon.println("set ssl_client to mqtt");
  clientMqtt.setClient(ssl_client);

  SerialMon.println("Setting up MQTT server");
  clientMqtt.setServer(AWS_IOT_ENDPOINT, 8883);
  clientMqtt.setCallback(messageHandler);

  clientMqtt.setKeepAlive(60); // 1 min
  // Create a message handler

  Serial.println("Connecting to AWS IOT");

  while (!clientMqtt.connect(THINGNAME))
  {
    Serial.print(".");
    delay(100);
  }

  if (!clientMqtt.connected())
  {
    Serial.println("AWS IoT Timeout!");
    return;
  }

  // Subscribe to a topic
  clientMqtt.subscribe(AWS_IOT_SUBSCRIBE_TOPIC);
  Serial.print("Subscribed to: ");
  Serial.println(AWS_IOT_SUBSCRIBE_TOPIC);

  Serial.println("AWS IoT Connected!");
}

void reconnect()
{
  while (!clientMqtt.connected())
  {

    if (clientMqtt.connect(THINGNAME))
    {
      SerialMon.println("conectado");
      clientMqtt.subscribe(AWS_IOT_SUBSCRIBE_TOPIC);
    }
    else
    {
      SerialMon.print("falló, rc=");
      SerialMon.print(clientMqtt.state());
      SerialMon.println(" ; intentando nuevamente en 5 segundos");
      delay(5000);
    }
  }
}

void publishToMQTT(const char *topic, JsonDocument &doc)
{
  char buffer[4096];
  size_t n = serializeJson(doc, buffer);

  SerialMon.println("Publishing readings: " + String(buffer));

  if (clientMqtt.connected())
  {
    bool success = clientMqtt.publish(topic, buffer);
    if (success)
    {
      SerialMon.println("Readings published successfully");
    }
    else
    {
      SerialMon.println("Failed to publish readings");
    }
  }
  else
  {
    SerialMon.println("MQTT client not connected, unable to publish readings");
    reconnect(); // Reconnect to MQTT if not connected
  }
}
void buildJsonVrms(JsonDocument &doc, 
                   String id_Irms1, String Irms1, String id_Irms2, String Irms2, String id_Irms3, String Irms3,
                   String id_Vrms1, String Vrms1, String id_Vrms2, String Vrms2, String id_Vrms3, String Vrms3) {
    doc["id_sensor"] = id_sensor;

    JsonObject json_Irms1 = doc[id_Irms1].to<JsonObject>();
    json_Irms1["Irms1"] = Irms1;

    JsonObject json_Irms2 = doc[id_Irms2].to<JsonObject>();
    json_Irms2["Irms2"] = Irms2;

    JsonObject json_Irms3 = doc[id_Irms3].to<JsonObject>();
    json_Irms3["Irms3"] = Irms3;

    JsonObject json_Vrms1 = doc[id_Vrms1].to<JsonObject>();
    json_Vrms1["Vrms1"] = Vrms1;

    JsonObject json_Vrms2 = doc[id_Vrms2].to<JsonObject>();
    json_Vrms2["Vrms2"] = Vrms2;

    JsonObject json_Vrms3 = doc[id_Vrms3].to<JsonObject>();
    json_Vrms3["Vrms3"] = Vrms3;


}

// void buildJsonVrms(JsonDocument &doc, String id_Irms1, String Irms1, String id_Irms2, String Irms2, String id_Irms3, String Irms3,
//                    String id_Vrms1, String Vrms1, String id_Vrms2, String Vrms2, String id_Vrms3, String Vrms3)
// {
//   doc["id_sensor"] = id_sensor;

//   // Irms
//   doc["id_Irms1"] = id_Irms1;
//   doc["Irms1"] = Irms1;
//   doc["id_Irms2"] = id_Irms2;
//   doc["Irms2"] = Irms2;
//   doc["id_Irms3"] = id_Irms3;
//   doc["Irms3"] = Irms3;

//   // Vrms
//   doc["id_Vrms1"] = id_Vrms1;
//   doc["Vrms1"] = Vrms1;
//   doc["id_Vrms2"] = id_Vrms2;
//   doc["Vrms2"] = Vrms2;
//   doc["id_Vrms3"] = id_Vrms3;
//   doc["Vrms3"] = Vrms3;
// }

void buildJsonPR(JsonDocument &doc, 
                 String id_Potencia_real_1, String Potencia_real_1, 
                 String id_Potencia_real_2, String Potencia_real_2, 
                 String id_Potencia_real_3, String Potencia_real_3) {
    doc["id_sensor"] = id_sensor;

    JsonObject json_Potencia_real_1 = doc[id_Potencia_real_1].to<JsonObject>();
    json_Potencia_real_1["Potencia_real_1"] = Potencia_real_1;

    JsonObject json_Potencia_real_2 = doc[id_Potencia_real_2].to<JsonObject>();
    json_Potencia_real_2["Potencia_real_2"] = Potencia_real_2;

    JsonObject json_Potencia_real_3 = doc[id_Potencia_real_3].to<JsonObject>();
    json_Potencia_real_3["Potencia_real_3"] = Potencia_real_3;
}

void buildJsonApar(JsonDocument &doc, 
                   String id_Potencia_apar_1, String Potencia_apar_1, 
                   String id_Potencia_apar_2, String Potencia_apar_2, 
                   String id_Potencia_apar_3, String Potencia_apar_3) {
    doc["id_sensor"] = id_sensor;

    JsonObject json_Potencia_apar_1 = doc[id_Potencia_apar_1].to<JsonObject>();
    json_Potencia_apar_1["Potencia_apar_1"] = Potencia_apar_1;

    JsonObject json_Potencia_apar_2 = doc[id_Potencia_apar_2].to<JsonObject>();
    json_Potencia_apar_2["Potencia_apar_2"] = Potencia_apar_2;

    JsonObject json_Potencia_apar_3 = doc[id_Potencia_apar_3].to<JsonObject>();
    json_Potencia_apar_3["Potencia_apar_3"] = Potencia_apar_3;
}

void buildJsonPotenciaReact(JsonDocument &doc, 
                            String id_Potencia_reac_1, String Potencia_reac_1, 
                            String id_Potencia_reac_2, String Potencia_reac_2, 
                            String id_Potencia_reac_3, String Potencia_reac_3) {
    doc["id_sensor"] = id_sensor;

    JsonObject json_Potencia_reac_1 = doc[id_Potencia_reac_1].to<JsonObject>();
    json_Potencia_reac_1["Potencia_reac_1"] = Potencia_reac_1;

    JsonObject json_Potencia_reac_2 = doc[id_Potencia_reac_2].to<JsonObject>();
    json_Potencia_reac_2["Potencia_reac_2"] = Potencia_reac_2;

    JsonObject json_Potencia_reac_3 = doc[id_Potencia_reac_3].to<JsonObject>();
    json_Potencia_reac_3["Potencia_reac_3"] = Potencia_reac_3;
}

void buildJsonFact(JsonDocument &doc,  
                   String id_Fact_potencia_1, String Fact_potencia_1, 
                   String id_Fact_potencia_2, String Fact_potencia_2, 
                   String id_Fact_potencia_3, String Fact_potencia_3) {
    doc["id_sensor"] = id_sensor;

    JsonObject json_Fact_potencia_1 = doc[id_Fact_potencia_1].to<JsonObject>();
    json_Fact_potencia_1["Fact_potencia_1"] = Fact_potencia_1;

    JsonObject json_Fact_potencia_2 = doc[id_Fact_potencia_2].to<JsonObject>();
    json_Fact_potencia_2["Fact_potencia_2"] = Fact_potencia_2;

    JsonObject json_Fact_potencia_3 = doc[id_Fact_potencia_3].to<JsonObject>();
    json_Fact_potencia_3["Fact_potencia_3"] = Fact_potencia_3;
}
void agregar_lecturas(String id_Irms1, String Irms1, String id_Irms2, String Irms2, String id_Irms3, String Irms3,
                      String id_Vrms1, String Vrms1, String id_Vrms2, String Vrms2, String id_Vrms3, String Vrms3,
                      String id_Potencia_real_1, String Potencia_real_1, String id_Potencia_real_2, String Potencia_real_2, String id_Potencia_real_3, String Potencia_real_3,
                      String id_Potencia_apar_1, String Potencia_apar_1, String id_Potencia_apar_2, String Potencia_apar_2, String id_Potencia_apar_3, String Potencia_apar_3,
                      String id_Potencia_reac_1, String Potencia_reac_1, String id_Potencia_reac_2, String Potencia_reac_2, String id_Potencia_reac_3, String Potencia_reac_3,
                      String id_Fact_potencia_1, String Fact_potencia_1, String id_Fact_potencia_2, String Fact_potencia_2, String id_Fact_potencia_3, String Fact_potencia_3)
{
  JsonDocument doc;
  JsonDocument doc2;
  JsonDocument doc3;
  JsonDocument doc4;
  JsonDocument doc5;
  buildJsonVrms(doc, id_Irms1, Irms1, id_Irms2, Irms2, id_Irms3, Irms3,
                id_Vrms1, Vrms1, id_Vrms2, Vrms2, id_Vrms3, Vrms3);
  buildJsonPR(doc2, id_Potencia_real_1, Potencia_real_1, id_Potencia_real_2, Potencia_real_2, id_Potencia_real_3, Potencia_real_3);
  buildJsonApar(doc3, id_Potencia_apar_1, Potencia_apar_1, id_Potencia_apar_2, Potencia_apar_2, id_Potencia_apar_3, Potencia_apar_3);
  buildJsonPotenciaReact(doc4, id_Potencia_reac_1, Potencia_reac_1, id_Potencia_reac_2, Potencia_reac_2, id_Potencia_reac_3, Potencia_reac_3);
  buildJsonFact(doc5, id_Fact_potencia_1, Fact_potencia_1, id_Fact_potencia_2, Fact_potencia_2, id_Fact_potencia_3, Fact_potencia_3);

  publishToMQTT(AWS_TOPIC_VRMS, doc);
  publishToMQTT(AWS_TOPIC_PR, doc2);
  publishToMQTT(AWS_TOPIC_APAR, doc3);
  publishToMQTT(AWS_TOPIC_POTENCIA_REACTIVA, doc4);
  publishToMQTT(AWS_TOPIC_FACT_POTENCIA, doc5);
  // publishToMQTT(doc);
  // publishToMQTT(doc2);
}

void inicia_modem()
{
  bool res;
  bool connected = false; // Variable para verificar la conexión a la red

  // Restart takes quite some time
  // To skip it, call init() instead of restart()
  DBG("Initializing modem in inicia modem...");
  if (!modem.init())
  {
    DBG("Failed to restart modem, delaying 10s and retrying");
    return;
  }
  do
  {
    res = modem.setNetworkMode(13);
    delay(500);
  } while (!res);

  String name = modem.getModemName();
  DBG("Modem Name:", name);

  String modemInfo = modem.getModemInfo();
  DBG("Modem Info:", modemInfo);

#if TINY_GSM_TEST_GPRS
  // Unlock your SIM card with a PIN if needed
  if (GSM_PIN && modem.getSimStatus() != 3)
  {
    modem.simUnlock(GSM_PIN);
  }
#endif

#if TINY_GSM_TEST_GPRS && defined TINY_GSM_MODEM_XBEE
  // The XBee must run the gprsConnect function BEFORE waiting for network!
  modem.gprsConnect(apn, gprsUser, gprsPass);
#endif

  DBG("Waiting for network...");
  if (!modem.waitForNetwork())
  {
    delay(10000);
    return;
  }

  if (modem.isNetworkConnected())
  {
    DBG("Network connected");
  }

#if TINY_GSM_TEST_GPRS
  DBG("Connecting to", apn);
  if (!modem.gprsConnect(apn, gprsUser, gprsPass))
  {
    delay(10000);
    return;
  }

  bool resp = modem.isGprsConnected();
  DBG("GPRS status:", resp ? "connected" : "not connected");

  String ccid = modem.getSimCCID();
  DBG("CCID:", ccid);

  String imei = modem.getIMEI();
  DBG("IMEI:", imei);

  String cop = modem.getOperator();
  DBG("Operator:", cop);

  IPAddress local = modem.localIP();
  DBG("Local IP:", local);

  int csq = modem.getSignalQuality();
  DBG("Signal quality:", csq);

#endif

#if TINY_GSM_TEST_GPRS
  if (!modem.isGprsConnected())
  {
    DBG("GPRS disconnected");
  }
  else
  {
    DBG("GPRS disconnect: Failed.");
  }
#endif
}

void desconecta_modem()
{
  modem.gprsDisconnect();
  light_sleep(5);
  if (!modem.isGprsConnected())
  {
    DBG("GPRS disconnected");
  }
  else
  {
    DBG("GPRS disconnect: Failed.");
  }
}

void duerme_sensor()
{
  DBG("Enable deep sleep , Will wake up in", delayvalue, " minutes");

  // Wait moden power off
  light_sleep(5);

  esp_sleep_enable_timer_wakeup(delayvalue * uS_TO_S_FACTOR * 60);
  delay(200);
  esp_deep_sleep_start();
}

void apaga_modem()
{
  // Try to power-off (modem may decide to restart automatically)
  // To turn off modem completely, please use Reset/Enable pins
  modem.poweroff();
  DBG("Poweroff.");
}

void obtener_datos_sensor()
{
  DBG("Connecting to ", String(server) + String(consultar_datos_sensor));
  String postData = "id_sensor=" + String(id_sensor);

  client7.beginRequest();
  client7.post(consultar_datos_sensor);
  client7.sendHeader("Content-Type", "application/x-www-form-urlencoded");
  client7.sendHeader("Content-Length", postData.length());
  client7.sendHeader("User-Agent", "Mantox_IOT--1.0");
  client7.sendHeader("Connection: close");
  client7.beginBody();
  client7.print(postData);
  client7.endRequest();
  client7.println();

  int status_code = client7.responseStatusCode();
  String response = client7.responseBody();

  DBG("Status code: ", status_code);
  DBG("Response: ", response);

  client7.stop();

  DBG("Asignando a variables: ");
  JsonDocument json;
  DeserializationError error = deserializeJson(json, response);

  if (!error)
  {
    // voltaje = jsonDocument["voltaje"];
    delayvalue = json["delay"];
    factor_calib_I = json["factor_calib_I"];
    factor_calib_V = json["factor_calib_V"];
    activo = json["activo"];
    offset = json["offset"];

    // DBG("--> voltaje = ", voltaje);
    DBG("--> delay = ", delayvalue);
    DBG("--> factor_calib_I = ", factor_calib_I);
    DBG("--> factor_calib_V = ", factor_calib_V);
    DBG("--> activo = ", activo);
    DBG("--> offset = ", offset);

    // factor de calibracion= 363
    emon1.current(32, factor_calib_I);
    emon2.current(33, factor_calib_I);
    emon3.current(34, factor_calib_I);
    emon1.voltage(14, factor_calib_V, 1.5); // Voltage: input pin, calibration, phase_shift
    emon2.voltage(13, factor_calib_V, 1.5); // Voltage: input pin, calibration, phase_shift
    emon3.voltage(12, factor_calib_V, 1.5); // Voltage: input pin, calibration, phase_shift
  }
  else
  {
    DBG("Error al analizar la respuesta JSON.");
  }
}

void leer_corriente_voltaje()
{
  DBG("Leyendo corriente y voltaje");
  // Descartar las primera 1 lectura
  for (int i = 0; i < 5; i++)
  {
    emon1.calcIrms(13200);
    emon2.calcIrms(13200);
    emon3.calcIrms(13200);
    emon1.calcVI(3300, 1000);
    emon2.calcVI(3300, 1000);
    emon3.calcVI(3300, 1000);
    delay(200);
  }

  emon1.calcVI(6600, 1000);
  emon2.calcVI(6600, 1000);
  emon3.calcVI(6600, 1000);
  // Medir corrientes
  Irms1 = (emon1.calcIrms(16500)) / 10;
  Irms2 = (emon2.calcIrms(16500)) / 10;
  Irms3 = (emon3.calcIrms(16500)) / 10;

  Vrms1 = emon1.Vrms;
  Vrms2 = emon2.Vrms;
  Vrms3 = emon3.Vrms;

  Potencia_real_1 = emon1.realPower / 10;
  Potencia_apar_1 = emon1.apparentPower / 10;
  Fact_potencia_1 = emon1.powerFactor;
  Potencia_reac_1 = sqrt(pow(Potencia_apar_1, 2) - pow(Potencia_real_1, 2));

  Potencia_real_2 = emon2.realPower / 10;
  Potencia_apar_2 = emon2.apparentPower / 10;
  Fact_potencia_2 = emon2.powerFactor;
  Potencia_reac_2 = sqrt(pow(Potencia_apar_2, 2) - pow(Potencia_real_2, 2));

  Potencia_real_3 = emon3.realPower / 10;
  Potencia_apar_3 = emon3.apparentPower / 10;
  Fact_potencia_3 = emon3.powerFactor;
  Potencia_reac_3 = sqrt(pow(Potencia_apar_3, 2) - pow(Potencia_real_3, 2));
}

void leer_corriente_voltaje_prueba()
{
  DBG("Leyendo corriente y voltaje");
  // Descartar las primera 1 lectura
  for (int i = 0; i < 5; i++)
  {
    emon1.calcIrms(13200);
    emon2.calcIrms(13200);
    emon3.calcIrms(13200);
    emon1.calcVI(3300, 1000);
    emon2.calcVI(3300, 1000);
    emon3.calcVI(3300, 1000);
    delay(200);
  }

  emon1.calcVI(6600, 1000);
  emon2.calcVI(6600, 1000);
  emon3.calcVI(6600, 1000);
  // Medir corrientes
  Irms1 = 230 / 10;
  Irms2 = 200 / 10;
  Irms3 = 400 / 10;

  Vrms1 = 9;
  Vrms2 = 10;
  Vrms3 = 11;

  Potencia_real_1 = 343 / 10;
  Potencia_apar_1 = 323 / 10;
  Fact_potencia_1 = 334;
  Potencia_reac_1 = 4112 - 3234;

  Potencia_real_2 = 330 / 10;
  Potencia_apar_2 = 2330 / 10;
  Fact_potencia_2 = 230;
  Potencia_reac_2 = 32;

  Potencia_real_3 = 2 / 10;
  Potencia_apar_3 = 232 / 10;
  Fact_potencia_3 = 223;
  Potencia_reac_3 = 2233;
}

void actualiza_datos_sensor()
{

  DBG("Connecting to ", String(server) + String(actualizar_datos_sensor));

  // Construir los datos para la solicitud POST
  String postData = "id_sensor=" + String(id_sensor) + "&senal_calidad=" + String(senal_calidad);

  client8.beginRequest();
  client8.post(actualizar_datos_sensor);
  client8.sendHeader("Content-Type", "application/x-www-form-urlencoded");
  client8.sendHeader("Content-Length", postData.length());
  client8.sendHeader("User-Agent", "Mantox_IOT--1.0");

  client8.sendHeader("Connection: close");
  client8.beginBody();
  client8.print(postData);
  client8.endRequest();
  client8.println();

  int status_code = client8.responseStatusCode();
  String response = client8.responseBody();

  DBG("Status code: ", status_code);
  DBG("Response: ", response);

  client8.stop();
}

void setup()
{
  // Set console baud rate
  SerialMon.begin(115200);
  delay(10);

  // Onboard LED light, it can be used freely
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);

  // POWER_PIN : This pin controls the power supply of the SIM7600
  pinMode(POWER_PIN, OUTPUT);
  digitalWrite(POWER_PIN, HIGH);

  // PWR_PIN ： This Pin is the PWR-KEY of the SIM7600
  // The time of active low level impulse of PWRKEY pin to power on module , type 500 ms
  pinMode(PWR_PIN, OUTPUT);
  digitalWrite(PWR_PIN, HIGH);
  delay(500);
  digitalWrite(PWR_PIN, LOW);

  // IND_PIN: It is connected to the SIM7600 status Pin,
  // through which you can know whether the module starts normally.
  pinMode(IND_PIN, INPUT);

  DBG("Wait...");

  delay(3000);

  SerialAT.begin(UART_BAUD, SERIAL_8N1, PIN_RX, PIN_TX);
  secure_layer1.setCACert(root_ca);
  secure_layer2.setCACert(root_ca);
  secure_layer3.setCACert(root_ca);
  secure_layer4.setCACert(root_ca);
  secure_layer5.setCACert(root_ca);
  secure_layer6.setCACert(root_ca);
  secure_layer7.setCACert(root_ca);
  secure_layer8.setCACert(root_ca);

  // Restart takes quite some time
  // To skip it, call init() instead of restart()
  DBG("Initializing modem in setup...");
  // Salir si no está conectado
  do
  {
    DBG("... not connected");
    inicia_modem();
  } while (!modem.isGprsConnected());

  SerialMon.println("Modem initialized");
  // Configurar certificados SSL
  connectAWS();
  SerialMon.println("Setup complete");
}
void loop()
{

  if (!modem.isGprsConnected())
  {
    DBG("... not connected");
    return;
    // Salir si no está conectado
  }

  DBG("... connected");

  // obtener_datos_sensor();
  // actualiza_datos_sensor();
  // tomamos lecturas una vez desconectado el modem

  if (clientMqtt.connected())
  {

    // AGREGAR ESTA VALIDACION CUANDO SE HAGA LA API DE ACTUALIZAR DATOS SENSOR
    //  if (activo == 1)
    //  {

    // }
    leer_corriente_voltaje_prueba();

    if (!modem.isGprsConnected())
    {
      DBG("... not connected");
    }
    else
    {
      agregar_lecturas(id_Irms1, String(Irms1), id_Irms2, String(Irms2), id_Irms3, String(Irms3),
                       id_Vrms1, String(Vrms1), id_Vrms2, String(Vrms2), id_Vrms3, String(Vrms3),
                       id_Potencia_real_1, String(Potencia_real_1), id_Potencia_real_2, String(Potencia_real_2), id_Potencia_real_3, String(Potencia_real_3),
                       id_Potencia_apar_1, String(Potencia_apar_1), id_Potencia_apar_2, String(Potencia_apar_2), id_Potencia_apar_3, String(Potencia_apar_3),
                       id_Potencia_reac_1, String(Potencia_reac_1), id_Potencia_reac_2, String(Potencia_reac_2), id_Potencia_reac_3, String(Potencia_reac_3),
                       id_Fact_potencia_1, String(Fact_potencia_1), id_Fact_potencia_2, String(Fact_potencia_2), id_Fact_potencia_3, String(Fact_potencia_3));
    }
  }
  else
  {
    reconnect();
  }
  clientMqtt.loop();
  delay(5000);
  duerme_sensor();
}
