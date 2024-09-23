
#define AWS_IOT_SUBSCRIBE_TOPIC "esp32/config"
#define AWS_TOPIC_VRMS "esp32/vrms"
#define AWS_TOPIC_PR "esp32/potencia_real"
#define AWS_TOPIC_APAR "esp32/potencia_aparente"
#define AWS_TOPIC_POTENCIA_REACTIVA "esp32/potencia_reactiva"
#define AWS_TOPIC_FACT_POTENCIA "esp32/fact_potencia"
#define AWS_TOPIC_PROMEDIO_VRMS "esp32/promedio_vrms"
#define AWS_TOPIC_PROMEDIO_IRMS "esp32/promedio_irms"
#define AWS_TOPIC_ID_SENSOR "esp32/id_sensor"
#define AWS_TOPIC_SENAL_CALIDAD "esp32/senal_calidad"

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

void resetConfig()
{
  delayvalue = 0;
  factor_calib_I = 0;
  factor_calib_V = 0;
  activo = 0;
  offset = 0;
  configRecibida = false;
}

int obtener_porcentaje_senial()
{
  // Obtener la calidad de la señal utilizando la función getSignalQuality() del objeto modem
  int getSignalQuality = modem.getSignalQuality();

  // Verificar si la calidad de la señal es 99 o si el módem no está conectado a la red
  if (getSignalQuality == 99 || !modem.isNetworkConnected())
  {
    // Devolver un porcentaje de señal de 0 en caso de que la calidad sea 99 o el módem no esté conectado
    return 0;
  }
  else
  {
    // Mapear la calidad de la señal en un rango de 0 a 31 a un rango de 0 a 100
    return map(getSignalQuality, 0, 31, 0, 100);
  }
}

void messageHandler(char *topic, byte *payload, unsigned int length)
{
  SerialMon.print("Mensaje recibido en el topic: ");
  SerialMon.println(topic);
  SerialMon.print("Mensaje: ");
  for (unsigned int i = 0; i < length; i++)
  {
    SerialMon.print((char)payload[i]);
  }
  SerialMon.println();

  JsonDocument doc;
  deserializeJson(doc, payload);

  if (!doc.isNull())
  {
    if (String(topic) == AWS_IOT_SUBSCRIBE_TOPIC)
    {
      delayvalue = doc["delayvalue"];
      factor_calib_I = doc["factor_calib_I"];
      factor_calib_V = doc["factor_calib_V"];
      activo = doc["activo"].as<int>();
      offset = doc["offset"];

      DBG("--> delay = ", delayvalue);
      DBG("--> factor_calib_I = ", factor_calib_I);
      DBG("--> factor_calib_V = ", factor_calib_V);
      DBG("--> activo = ", activo);
      DBG("--> offset = ", offset);

      // Aplicar los valores de calibración
      emon1.current(32, factor_calib_I);
      emon2.current(33, factor_calib_I);
      emon3.current(34, factor_calib_I);
      emon1.voltage(14, factor_calib_V, 1.5);
      emon2.voltage(13, factor_calib_V, 1.5);
      emon3.voltage(12, factor_calib_V, 1.5);

      intervalo = delayvalue * 60 * 1000;
      SerialMon.println("Nuevo intervalo calculado: " + String(intervalo));
    }
  }
  else
  {
    SerialMon.println("Error al analizar la respuesta JSON.");
  }
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
    connectAWS();
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
                   String id_Vrms1, String Vrms1, String id_Vrms2, String Vrms2, String id_Vrms3, String Vrms3)
{
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

void buildJsonPR(JsonDocument &doc,
                 String id_Potencia_real_1, String Potencia_real_1,
                 String id_Potencia_real_2, String Potencia_real_2,
                 String id_Potencia_real_3, String Potencia_real_3)
{
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
                   String id_Potencia_apar_3, String Potencia_apar_3)
{
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
                            String id_Potencia_reac_3, String Potencia_reac_3)
{
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
                   String id_Fact_potencia_3, String Fact_potencia_3)
{
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

  float promedioVrms = (Vrms1.toFloat() + Vrms2.toFloat() + Vrms3.toFloat()) / 3;
  float promedioIrms = (Irms1.toFloat() + Irms2.toFloat() + Irms3.toFloat()) / 3;

  // Publicación de los promedios
  JsonDocument docPromedio;
  docPromedio["promedio_vrms"] = promedioVrms;
  docPromedio["promedio_irms"] = promedioIrms;

  publishToMQTT(AWS_TOPIC_PROMEDIO_VRMS, docPromedio);
  publishToMQTT(AWS_TOPIC_PROMEDIO_IRMS, docPromedio);

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
}

void inicia_modem()
{
  bool res;
  bool connected = false; // Variable para verificar la conexión a la red

  // Restart takes quite some time
  // To skip it, call init() instead of restart()

  DBG("Initializing modem in inicia modem...");

  // POWER_PIN : This pin controls the power supply of the SIM7600
  pinMode(POWER_PIN, OUTPUT);
  digitalWrite(POWER_PIN, HIGH);

  // PWR_PIN ： This Pin is the PWR-KEY of the SIM7600
  // The time of active low level impulse of PWRKEY pin to power on module , type 500 ms
  pinMode(PWR_PIN, OUTPUT);
  digitalWrite(PWR_PIN, HIGH);
  delay(500);
  digitalWrite(PWR_PIN, LOW);
  delay(1000);

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
  if (!modem.isGprsConnected())
  {
    DBG("GPRS disconnected");
  }
  else
  {
    DBG("GPRS disconnect: Failed.");
  }
}

void apaga_modem()
{
  // Try to power-off (modem may decide to restart automatically)
  // To turn off modem completely, please use Reset/Enable pins
  modem.gprsDisconnect();
  delay(500); // Esperar un segundo antes de apagar el módem
  modem.poweroff();
  DBG("Poweroff.");
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

  float promedioVrms = (Vrms1 + Vrms2 + Vrms3) / 3;
  float promedioIrms = (Irms1 + Irms2 + Irms3) / 3;
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
  Irms1 = 24440 / 10;
  Irms2 = 1232 / 10;
  Irms3 = 2140 / 10;

  Vrms1 = 24;
  Vrms2 = 23;
  Vrms3 = 31;

  Potencia_real_1 = 3221 / 10;
  Potencia_apar_1 = 122 / 10;
  Fact_potencia_1 = 3224;
  Potencia_reac_1 = 4122 - 3234;

  Potencia_real_2 = 31230 / 10;
  Potencia_apar_2 = 2330 / 10;
  Fact_potencia_2 = 23034;
  Potencia_reac_2 = 3244;

  Potencia_real_3 = 232 / 10;
  Potencia_apar_3 = 23232 / 10;
  Fact_potencia_3 = 2232112;
  Potencia_reac_3 = 2232333;
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

  SerialAT.begin(UART_BAUD, SERIAL_8N1, PIN_RX, PIN_TX);
}

bool primeraEjecucion = true;

void loop()
{
  if (primeraEjecucion || millis() - lastMillis >= intervalo)
  {
    primeraEjecucion = false;
    lastMillis = millis();

    // Conectar a la red y al servidor MQTT
    if (!modem.isGprsConnected())
    {
      do
      {
        DBG("... not connected");
        inicia_modem();
      } while (!modem.isGprsConnected());
    }

    if (!clientMqtt.connected())
    {
      reconnect();
      clientMqtt.subscribe("esp32/config");
      DBG("Subscribed to in loop: ", "esp32/config");
      JsonDocument doc;
      doc["id_sensor"] = id_sensor; // Si id_sensor es un número u otro tipo, conviértelo a String

      publishToMQTT(AWS_TOPIC_ID_SENSOR, doc);
      senal_calidad = obtener_porcentaje_senial();
      JsonDocument senal_doc;
      senal_doc["senal_calidad"] = senal_calidad;
      senal_doc["id_sensor"] = id_sensor; // Si id_sensor es un número u otro tipo, conviértelo a String
      publishToMQTT(AWS_TOPIC_SENAL_CALIDAD, senal_doc);
      DBG("Published to: ", AWS_TOPIC_ID_SENSOR);
      DBG("Pub to", AWS_TOPIC_SENAL_CALIDAD);
    }

    // Esperar configuración del sensor
    configRecibida = false;
    while (!configRecibida && millis() - lastMillis < intervalo)
    {
      clientMqtt.loop();
      // Verificar si la configuración fue recibida
      if (activo < 1)
      {
        Serial.println("El sensor no está activo, reiniciando...");
        lastMillis = millis(); // Reiniciar el contador para volver a esperar un minuto completo
        continue;              // Reiniciar el loop y esperar la configuración nuevamente
      }
      configRecibida = true;
    }

    // Desconectar el módem antes de leer los datos del sensor
    if (configRecibida)
    {
      if (clientMqtt.connected())
      {
        clientMqtt.disconnect();
        DBG("MQTT disconnected before turning off modem");
      }
      apaga_modem();
      leer_corriente_voltaje_prueba();

      // Reiniciar módem y reconectar
      DBG("Reconnecting to the network");
      modem.restart();

      attempts = 0;
      while (!modem.isGprsConnected() && attempts < maxAttempts)
      {
        inicia_modem();
        attempts++;
      }

      if (attempts >= maxAttempts)
      {
        DBG("Max attempts reached, restarting modem");
        modem.restart();
      }

      // Publicar las lecturas
      if (modem.isGprsConnected())
      {
        connectAWS();

        if (clientMqtt.connected())
        {
          agregar_lecturas(id_Irms1, String(Irms1), id_Irms2, String(Irms2), id_Irms3, String(Irms3),
                           id_Vrms1, String(Vrms1), id_Vrms2, String(Vrms2), id_Vrms3, String(Vrms3),
                           id_Potencia_real_1, String(Potencia_real_1), id_Potencia_real_2, String(Potencia_real_2), id_Potencia_real_3, String(Potencia_real_3),
                           id_Potencia_apar_1, String(Potencia_apar_1), id_Potencia_apar_2, String(Potencia_apar_2), id_Potencia_apar_3, String(Potencia_apar_3),
                           id_Potencia_reac_1, String(Potencia_reac_1), id_Potencia_reac_2, String(Potencia_reac_2), id_Potencia_reac_3, String(Potencia_reac_3),
                           id_Fact_potencia_1, String(Fact_potencia_1), id_Fact_potencia_2, String(Fact_potencia_2), id_Fact_potencia_3, String(Fact_potencia_3));

          if (clientMqtt.connected())
          {
            clientMqtt.disconnect();
            DBG("MQTT disconnected after reading published");
          }

          resetConfig();
          apaga_modem();
        }
      }
      else
      {
        DBG("GPRS not connected");
        attempts = 0;
        while (!modem.isGprsConnected() && attempts < maxAttempts)
        {
          inicia_modem();
          attempts++;
        }

        if (attempts >= maxAttempts)
        {
          DBG("Max attempts reached, restarting modem");
          modem.restart();
        }
      }
    }
    else
    {
      Serial.println("No se recibió la configuración del sensor.");
    }
  }

  // Loop del cliente MQTT para mantener la conexión
  clientMqtt.loop();
}
