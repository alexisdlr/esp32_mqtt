/* Time ESP32 will go to sleep (in seconds) */
#define TINY_GSM_TEST_GPRS true
#define TINY_GSM_TEST_WIFI false
#define TINY_GSM_TEST_CALL false
#define TINY_GSM_TEST_SMS false
#define TINY_GSM_TEST_USSD false
#define TINY_GSM_TEST_BATTERY false
#define TINY_GSM_TEST_GPS true
// powerdown modem after tests
#define TINY_GSM_POWERDOWN true
#define MODEM_TX 27
#define MODEM_RX 26
#define MODEM_PWRKEY 4
#define MODEM_DTR 32
#define MODEM_RI 33
#define MODEM_FLIGHT 25
#define MODEM_STATUS 34

#define SD_MISO 2
#define SD_MOSI 15
#define SD_SCLK 14
#define SD_CS 13

#define uS_TO_S_FACTOR 1000000ULL /* Conversion factor for micro seconds to seconds */
#define TIME_TO_SLEEP 60          /* Time ESP32 will go to sleep (in seconds) */

#define PIN_TX 27
#define PIN_RX 26
#define UART_BAUD 115200
#define PWR_PIN 4
#define LED_PIN 12
#define POWER_PIN 25
#define IND_PIN 36

#define GSM_PIN ""

const char apn[] = "internet.itelcel.com"; // Your operator APN
const char gprsUser[] = "webgprs";
const char gprsPass[] = "webgprs2002";

// datos HTTP
const char server[] = "login.mantox.mx";
const char consultar_datos_sensor[] = "/prueba_sensor_corriente/controller/buscar/datos_sensor/";
const char agregar_lecturas_sensor[] = "/prueba_sensor_corriente/controller/insertar/metrica_sensor2/";
const char actualizar_datos_sensor[] = "/prueba_sensor_corriente/controller/actualizar/datos_sensor2/";
const int port = 443;

const char *id_sensor = "1";

const char *id_Irms1 = "1";
const char *id_Irms2 = "2";
const char *id_Irms3 = "3";

const char *id_Vrms1 = "4";
const char *id_Vrms2 = "5";
const char *id_Vrms3 = "6";

const char *id_Potencia_real_1 = "7";
const char *id_Potencia_real_2 = "8";
const char *id_Potencia_real_3 = "9";

const char *id_Potencia_apar_1 = "10";
const char *id_Potencia_apar_2 = "11";
const char *id_Potencia_apar_3 = "12";

const char *id_Potencia_reac_1 = "13";
const char *id_Potencia_reac_2 = "14";
const char *id_Potencia_reac_3 = "15";

const char *id_Fact_potencia_1 = "16";
const char *id_Fact_potencia_2 = "17";
const char *id_Fact_potencia_3 = "18";

int activo = 0;
// int voltaje = 0;
int delayvalue = 5;

float factor_calib_I = 0;
float factor_calib_V = 0;
float offset = 0;

double Irms1 = 0;
double Irms2 = 0;
double Irms3 = 0;

double Vrms1 = 0;
double Vrms2 = 0;
double Vrms3 = 0;

double Potencia_real_1 = 0;
double Potencia_apar_1 = 0;
double Fact_potencia_1 = 0;
double Potencia_reac_1 = 0;

double Potencia_real_2 = 0;
double Potencia_apar_2 = 0;
double Fact_potencia_2 = 0;
double Potencia_reac_2 = 0;

double Potencia_real_3 = 0;
double Potencia_apar_3 = 0;
double Fact_potencia_3 = 0;
double Potencia_reac_3 = 0;

String locacion = "";
int senal_calidad = 0;
