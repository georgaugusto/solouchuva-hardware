/*

  Protótipo de IoT para Smart Agriculture com foco em Monitoramento Meteorológico

  Arquivo: iot-monitoramento-meteorologico.ino
  Versão: 0.1
  Autor:
    Georg Augusto Schlegel <georgaugusto@gmail.com>

  *Obs: Protótipo precisam de calibração

*/

#include <Wire.h> // Esta biblioteca permite que você se comunique com dispositivos I2C / TWI

// -------- Configurações do ESP8266 e Wi-fi --------

#include <ESP8266WiFi.h>  // Biblioteca ESP8266

#define WIFI_SSID "e eu o ROBIN"  // Nome da sua rede Wi-Fi
#define WIFI_PASSWORD "quesenha?" // Senha da sua rede Wi-Fi

// -------- Configurações do Firebase --------

#include <FirebaseArduino.h> // Biblioteca FireBase

#define FIREBASE_HOST "monitoramentometeorologico.firebaseio.com" // O endereço do nome do projeto a partir do firebase id
#define FIREBASE_AUTH "QeWROvnRHnaFY2jqyfNWZLVwL0dllZaW5ARdWvDH"  // A chave secreta gerada a partir do firebase

// -------- Configurações do Protocolo NTP --------

#include <NTPClient.h>  // Biblioteca do NTP.
#include <WiFiUdp.h>    // Biblioteca do UDP.

String formattedDate; // Váriavel que armazenara do NTP.
String dayStamp;      // Váriavel que armazenara o dia do NTP.
String timeStamp;     // Váriavel que armazenara o horario do NTP.

String lastDayStamp;  // Váriavel que armazenara o dia anterior.

WiFiUDP udp;          //Cria um objeto "UDP".
NTPClient timeClient(udp, "gps.jd.ntp.br", -3 * 3600, 60000); // Cria um objeto "NTP" com as configurações de data e hora utilizada no Brasil

// -------- Configurações do Sensor em Geral --------
/*
  |------------------------------------------|
  |   Intervalo de leitura do sensor em ms   |
  |------------------------------------------|
  |  INTERVALO  |   FREQ.  |   # UP/DIA      |
  |-------------|----------|-----------------|
  |  1.800.000  |  30 min  |     48 up/dia   |
  |  1.200.000  |  20 min  |     72 up/dia   |
  |    900.000  |  15 min  |     96 up/dia   |
  |    600.000  |  10 min  |    144 up/dia   |
  |    300.000  |   5 min  |    288 up/dia   |
  |     60.000  |   1 min  |  1.440 up/dia   |
  |     30.000  |  30 sec  |  2.880 up/dia   |
  |-------------|----------|-----------------|
*/

#define delay_sensor 60000 // Delay da leitura dos sensores em ms

// Variáveis ​​de controle de estado
unsigned long  lastReadMs = 0; // O último tempo de leitura dos sensores em ms
unsigned long currentReadingMs = 0;  // Leitura atual dos sensores em ms
int counterAvg = 0; // Variável que irá armazenar o número de leituras dos sensores

// -------- Configurações do Sensor DHT22 (Temperatura e umidade do ar) --------

#include <DHT.h>          // Biblioteca DHT
#define DHTPIN 00         // D3 - Pino digital conectado ao DHT22
#define DHTTYPE DHT22     // Inicialize o tipo DHT como DHT22
DHT dht(DHTPIN, DHTTYPE); // Definiçoes do sensor: pino, tipo
float h;                  // Váriavel que armazenara a umidade
float t;                  // Váriavel que armazenara a temperatura

// Definição de variaveis mínimas/máximas e Avgs do sensor DHT22  *Para efeito de comparativos entre os dados enviados e coletados
float hMin;    // Váriavel que armazenara o menor valor de umidade do sensor
String hMinTime;
float hMax;    // Váriavel que armazenara o maior valor de umidade do sensor
String hMaxTime;

float tMin;    // Váriavel que armazenara o menor valor de temperatura do sensor
String tMinTime;
float tMax;    // Váriavel que armazenara o maior valor de temperatura do sensor
String tMaxTime;

float hSum = 0; // Váriavel que armazenara a soma dos valor de umidade do sensor
float tSum = 0; // Váriavel que armazenara a soma dos valor de temperatura do sensor
float hAvg = 0; // Váriavel que armazenara a média dos valor de umidade do sensor
float tAvg = 0; // Váriavel que armazenara a média dos valor de temperatura do sensor

// -------- Configurações do Sensor BMP280 (Temperatura (Não usado), pressão e altitude) --------

#include <Adafruit_Sensor.h>  // Carrega a biblioteca
#include <Adafruit_BMP280.h>  // Carrega a biblioteca
Adafruit_BMP280 Bmp280;       // Definiçoes do sensor
float bmpp;           // Váriavel que armazenara o valor do sensor da pressao ATM
float bmpa;                   // Váriavel que armazenara o valor do sensor da altitude

// Definição de variaveis mínimas/máximas e Avgs do sensor BMP280  *Para efeito de comparativos entre os dados enviados e coletados
float  bmppMin;    // Váriavel que armazenara o menor valor de pressao ATM do sensor
String bmppMinTime;
float  bmppMax;    // Váriavel que armazenara o maior valor de pressao ATM do sensor
String bmppMaxTime;

float bmpaMin;    // Váriavel que armazenara o menor valor de altitude do sensor
String bmpaMinTime;
float bmpaMax;    // Váriavel que armazenara o maior valor de altitude do sensor
String bmpaMaxTime;

float bmppSum = 0; // Váriavel que armazenara a soma dos valor de pressao ATM do sensor
float bmpaSum = 0; // Váriavel que armazenara a soma dos valor de altitude do sensor
float bmppAvg = 0; // Váriavel que armazenara a média dos valor de pressao ATM do sensor
float bmpaAvg = 0; // Váriavel que armazenara a média dos valor de altitude do sensor

// -------- Configurações do Sensor ML8511 (Luz ultravioleta) --------

int16_t  sensorValueML8511 = 0;  // Váriavel que armazenara o valor sensor ML8511
int UvIndex = 0;                // Váriavel que armazenara o índice ultravioleta
float sensorVoltageML8511;     // Váriavel que armazenara a tensão

// Definição de variaveis mínimas/máximas e Avgs do sensor ML8511  *Para efeito de comparativos entre os dados enviados e coletados
float UvIndexMin;    // Váriavel que armazenara o menor valor de índice ultravioleta do sensor
String UvIndexMinTime;
float UvIndexMax;    // Váriavel que armazenara o maior valor de índice ultravioleta do sensor
String UvIndexMaxTime;

float UvIndexSum = 0; // Váriavel que armazenara a soma dos valor de índice ultravioleta do sensor
float UvIndexAvg = 0; // Váriavel que armazenara a média dos valor de índice ultravioleta do sensor

// -------- Configurações do Sensor BH1750 (Luminosidade) --------

#include <BH1750.h>   // Carrega a biblioteca BH1750
BH1750 Bh1750(0x23);  // Define o sensor BH1750 e o endereço
int bh;               // Váriavel que armazenara o valor do sensor da luminosidade

// Definição de variaveis mínimas/máximas e Avgs do sensor BH1750  *Para efeito de comparativos entre os dados enviados e coletados
float bhMin;    // Váriavel que armazenara o menor valor de luminosidade do sensor
String bhMinTime;
float bhMax;    // Váriavel que armazenara o maior valor de luminosidade do sensor
String bhMaxTime;

float bhSum = 0; // Váriavel que armazenara a soma dos valor de luminosidade do sensor
float bhAvg = 0; // Váriavel que armazenara a média dos valor de luminosidade do sensor

// -------- Configurações do Sensor de Umidade do Solo Capacitivo --------

float  sensorValueCSMSV12; // Váriavel que armazenara o valor do sensor de solo
float sensorRangeCSMSV12; // Váriavel que armazenara o valor do sensor de solo remapeado

// Calibração do sensor de umidade do solo capacitivo
float sensorMinCSMSV12 = 0;     // Váriavel que armazenara o valor mínimo *Obs: Precisam de calibração
float sensorMaxCSMSV12 = 3.03;  // Váriavel que armazenara o valor máximo *Obs: Precisam de calibração

// Definição de variaveis mínimas/máximas e Avgs do sensor de Umidade do Solo Capacitivo  *Para efeito de comparativos entre os dados enviados e coletados
float sensorRangeCSMSV12Min;    // Váriavel que armazenara o menor valor de umidade do solo do sensor
String sensorRangeCSMSV12MinTime;
float sensorRangeCSMSV12Max;    // Váriavel que armazenara o maior valor de umidade do solo do sensor
String sensorRangeCSMSV12MaxTime;

float sensorRangeCSMSV12Sum = 0; // Váriavel que armazenara a soma dos valor de umidade do solo do sensor
float sensorRangeCSMSV12Avg = 0; // Váriavel que armazenara a média dos valor de umidade do solo do sensor

// -------- Configurações do Sensor de Molhamento Foliar --------

float  sensorValueMF;  // Váriavel que armazenara o valor do sensor de molhamento foliar
float sensorRangeMF;  // Váriavel que armazenara o valor do sensor de molhamento foliar

// Calibração do sensor de molhamento foliar
float sensorMinMF = 0;     // Váriavel que armazenara o valor mínimo *Obs: Precisam de calibração
float sensorMaxMF = 3.33;  // Váriavel que armazenara o valor máximo *Obs: Precisam de calibração

// Definição de variaveis mínimas/máximas e Avgs do Sensor de Molhamento Foliar  *Para efeito de comparativos entre os dados enviados e coletados
float sensorRangeMFMin;    // Váriavel que armazenara o menor valor de molhamento foliar do solo do sensor
String sensorRangeMFMinTime;
float sensorRangeMFMax;    // Váriavel que armazenara o maior valor de molhamento foliar do sensor
String sensorRangeMFMaxTime;

float sensorRangeMFSum = 0; // Váriavel que armazenara a soma dos valor de molhamento foliar do sensor
float sensorRangeMFAvg = 0; // Váriavel que armazenara a média dos valor de molhamento foliar do sensor

// -------- Configurações do Sensor Hall US1881/U18 --------

#define pinHallSensorPluviometer 14     // D5 - Pino digital conectado ao sensor hall do pluviometro
#define delaySensorHallPluviometer 1000  // Delay do Sensor Hall US1881/U18

const float catchmentArea = 7797.0;  // Area de captação em mm2
volatile int weighbridgeCounter = 0;   // Váriavel que armazenara o contador da báscula
volatile float rainVolume = 0;     // Váriavel que armazenara o volume de chuva

volatile float rainVolumeMin;
String rainVolumeMinTime;
volatile float rainVolumeMax;
String rainVolumeMaxTime;

unsigned long lastReadingWeighbridgeMs = 0;      // Váriavel que armazenara a ultima leitura da bascula em ms
unsigned long readingCurrentWeighbridgeMs = 0;       // Váriavel que armazenara a leitura atual da bascula em ms
volatile float lastValueAccountWeighbridge = 0; // Váriavel que armazenara a última contagem do contador da báscula
volatile int currentValueCountWeighbridge = 0;    // Váriavel que armazenara a atual contagem do contador da báscula

float weighbridgeVolume = 3.22; // Volume da báscula do protótipo

// -------- Configurações do Sensor ADS1115 --------

#include <Adafruit_ADS1015.h>   // Biblioteca ADS1115
Adafruit_ADS1115 ads(0x48);     // Define o modulo ADS1115 e o endereço

int16_t adc0, adc1, adc2, adc3; // Criação das váriaveis do modulo ADS1115

// -------- Configurações do Anemômetro --------

const int pinSensorHallAnemometer = 13; // D7 - Pino digital conectado ao sensor hall do anemômetro

const float pi = 3.14159265;  // Váriavel que armazenara o número Pi
int period = 5000;           // Váriavel que armazenara o tempo de medida do sensor em ms
int radius = 103;               // Váriavel que armazenara o radius do anemometro em milímetro

// Variable definitions
unsigned int anemometerCounter = 0;  // Váriavel que armazenara uma variável de contador
unsigned int RPM = 0;                 // Váriavel que armazenara o numero de revoluções por minuto
float windSpeed = 0;            // Váriavel que armazenara a velocidade do vento em km/h

float windSpeedMin;
String windSpeedMinTime;
float windSpeedMax;
String windSpeedMaxTime;

float windSpeedSum = 0;
float windSpeedAvg = 0;

//  -------- Variáveis de sensação térmica --------

float thermalSensation; // Váriavel que armazenara o valor da sensação térmica

// Definição de variaveis mínimas/máximas e Avgs da sensação térmica
float thermalSensationMin;
String thermalSensationMinTime;
float thermalSensationMax;
String thermalSensationMaxTime;

float thermalSensationSum = 0;
float thermalSensationAvg = 0;

// -------- Setup --------

void setup() {
  pinMode(pinHallSensorPluviometer, INPUT_PULLUP);  // Define o pino do sensor hall do pluviometro como entrada
  pinMode(pinSensorHallAnemometer, INPUT_PULLUP);   // Define o pino do sensor hall do anemometro como entrada
  attachInterrupt(pinSensorHallAnemometer, addcontador, FALLING);

  Serial.begin(9600); // Inicia a comunicação serial

  Wire.begin();       // Inicia a comunicação I2C
  timeClient.begin(); // Inicia o Protocolo NTP
  dht.begin();        // Inicia o DHT22
  Bh1750.begin();     // Inicia o sensor BH1750
  Bmp280.begin();     // Inicia o sensor BMP280
  ads.begin();        // Inicia o sensor ADS1115

  if (digitalRead(pinHallSensorPluviometer) == 0) {
    redefinesPluviometerValues(); // Redefine os parametros do pluviômetro
  }

  connectWifi();                               // Conecta ao Wi-fi
  Firebase.begin(FIREBASE_HOST, FIREBASE_AUTH); // Inicia e conecta ao firebase

  collectsMinMaxValues(); // coleta os valores mininos e maximos dos sensores
  dateTime(); // Coleteta data e hora
  lastDayStamp = dayStamp;  // Define a ultima data
}

// -------- Loop --------

void loop() {
  while (!timeClient.update()) {
    timeClient.forceUpdate();   // Força atualização do Protocolo NTP para data e hora
  }

  testDelay();  // Verifica se o delay definido para continuar a fazer as leituras, etc...

}

void connectWifi() {
  delay(10);
  Serial.print("Conectando a ");
  Serial.print(WIFI_SSID);                // Mostra o nome do Wi-fi que está conectando
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);   // Inicia e conecta ao Wi-fi
  while (WiFi.status() != WL_CONNECTED) { // Enquanto não conecta mostra "."
    delay(500);
    Serial.print(".");
  }
  Serial.println();
  Serial.print("Conectado em ");
  Serial.println(WIFI_SSID);  // Mostra o nome do Wi-fi que você conectou
  Serial.print("O endereço de ip : ");
  Serial.println(WiFi.localIP()); // Mostra o endereço de ip local
}

void dateTime () {
  formattedDate = timeClient.getFormattedDate();  // Armazene a data coletada da biblioteca NTPClient

  int splitT = formattedDate.indexOf("T");
  dayStamp = formattedDate.substring(0, splitT); // Armazene a data formatada

  timeStamp = timeClient.getFormattedTime(); // Armazene a data em formato timeStamp
}

void showSerial() { // Mostra todos os dados do prototipo no monitor serial
  Serial.println("*****************************************************************");
  Serial.println("*                                                               *");
  Serial.print("*                  "); Serial.print(timeStamp); Serial.print(" - "); Serial.print(dayStamp); Serial.println("                        *");
  Serial.println("*                                                               *");
  Serial.print("*               "); Serial.print("Umidade: "); Serial.print(h); Serial.print("%"); Serial.println("                                 *");
  Serial.print("*               "); Serial.print("Temperatura: ");  Serial.print(t); Serial.print("°C"); Serial.println("                            *");
  Serial.print("*               "); Serial.print("Sensação térmica: ");  Serial.print(thermalSensation); Serial.print("°C"); Serial.println("                 *");
  Serial.print("*               "); Serial.print("Luminosidade: ");  Serial.print(bh);  Serial.print("Lux"); Serial.println("                             *");
  Serial.print("*               "); Serial.print("Pressao ATM: ");  Serial.print(bmpp);  Serial.print("hpa"); Serial.println("                           *");
  Serial.print("*               "); Serial.print("Altitude: ");  Serial.print(bmpa);  Serial.print("m "); Serial.println("                              *");
  Serial.print("*               "); Serial.print("Índice UV: ");  Serial.print(UvIndex); Serial.println("                                           *");
  Serial.print("*               "); Serial.print("Molhamento Foliar: ");  Serial.print(sensorRangeMF); Serial.println("%                            *");
  Serial.print("*               "); Serial.print("Sensor Solo: ");  Serial.print(sensorRangeCSMSV12); Serial.println("%                            *");
  Serial.print("*               "); Serial.print("Velocidade do Ventro: ");  Serial.print(windSpeed); Serial.println("km/h                    *");
  Serial.print("*               "); Serial.print("Pluviometro: ");  Serial.print(rainVolume);  Serial.print("mm "); Serial.println("                            *");
  Serial.println("*                                                               *");
  Serial.println("*****************************************************************");
  Serial.println("");
}

void publishReadings() {
  DynamicJsonBuffer jsonBuffer;  // Chama a biblioteca e lida com o gerenciamento de memória e chama o analisador
  JsonObject& root = jsonBuffer.createObject();  // Cria um objto Json que será enviado ao firebase
  JsonObject& rootTime = root.createNestedObject ("timestamp"); // Cria um objto Json de data e hora no formato timestamp
  root["dht22Moisture"] = h; // Cria a variável com o valor de umidade que será enviado ao firebase
  root["dht22Temperature"] = t;
  root["thermalSensation"] = thermalSensation;
  root["bh1750Brightness"] = bh;
  root["bmp280Pressure"] = bmpp;
  root["bmp280Altitude"] = bmpa;
  root["uvm30aIndexUv"] = UvIndex;
  root["mhrdWetting"] = sensorRangeMF;
  root["csmsv12Soil"] = sensorRangeCSMSV12;
  root["anemometer"] = windSpeed;
  root["pluviometer"] = rainVolume;
  rootTime[".sv"] = "timestamp";
  Firebase.push("Prototipo0/" + dayStamp + "/Readings", root); // Envia o objto Json para o firebase no caminho definido
  if (Firebase.failed()) {
    Serial.print("pushing /logs failed:"); // Caso o objto Json falhe avisa no monitor serial
    Serial.println(Firebase.error());
    return;
  }
}

void readSensorML8511() {
  sensorValueML8511 = analogRead(A0); // Armazena o valor coletado da porta A0 do ADS1115
  sensorVoltageML8511 = ( sensorValueML8511 * 0.1875) / 1000;  // Armazena e calcula voltagem da porta com os valores dados do datasheet do ADS1115
  UvIndex = sensorVoltageML8511 * 100; // Armazena o resultado do valor do indice UV

  if (UvIndex > UvIndexMax) {
    UvIndexMax = UvIndex; // Armazena o máximo valor coletado de indice UV
    UvIndexMaxTime = timeClient.getFormattedTime();
  }
  if (UvIndex < UvIndexMin) {
    UvIndexMin = UvIndex; // Armazena o mínimo valor coletado de indice UV
    UvIndexMinTime = timeClient.getFormattedTime();
  }

  UvIndexSum = UvIndexSum + UvIndex;  // Calcula a Avg do indice UV
  
}

void readSensorCSMSV12() {
  adc1 = ads.readADC_SingleEnded(1);  // Armazena o valor coletado da porta A1 do ADS1115
   sensorValueCSMSV12 = (adc1 * 0.1875) / 1000; // Armazena e calcula voltagem da porta com os valores dados do datasheet do ADS1115
  sensorRangeCSMSV12 = map1( sensorValueCSMSV12, sensorMinCSMSV12, sensorMaxCSMSV12, 100, 0);  // Mapeia os valores coletados da voltagem entre 0 e 100

  if (sensorRangeCSMSV12 > sensorRangeCSMSV12Max) {
    sensorRangeCSMSV12Max = sensorRangeCSMSV12; // Armazena o máximo valor coletado ddo sensor de umidade de solo
    sensorRangeCSMSV12MaxTime = timeClient.getFormattedTime();
  }
  if (sensorRangeCSMSV12 < sensorRangeCSMSV12Min) {
    sensorRangeCSMSV12Min = sensorRangeCSMSV12; // Armazena o mínimo valor coletado de umidade de solo
    sensorRangeCSMSV12MinTime = timeClient.getFormattedTime();
  }

  sensorRangeCSMSV12Sum = sensorRangeCSMSV12Sum + sensorRangeCSMSV12;  // Calcula a Avg de umidade de solo
  
}

void readSensorMhrd() {
  adc2 = ads.readADC_SingleEnded(2);  // Armazena o valor coletado da porta A2 do ADS1115
  sensorValueMF = (adc2 * 0.1875) / 1000;  // Armazena e calcula voltagem da porta com os valores dados do datasheet do ADS1115
  sensorRangeMF = map1( sensorValueMF, sensorMinMF, sensorMaxMF, 100, 0);  // Mapeia os valores coletados da voltagem entre 0 e 100

  if (sensorRangeMF > sensorRangeMFMax) {
    sensorRangeMFMax = sensorRangeMF; // Armazena o máximo valor coletado de molhamento foliar
    sensorRangeMFMaxTime = timeClient.getFormattedTime();
  }
  if (sensorRangeMF < sensorRangeMFMin) {
    sensorRangeMFMin = sensorRangeMF; // Armazena o mínimo valor coletado de molhamento foliar
    sensorRangeMFMinTime = timeClient.getFormattedTime();
  }

  sensorRangeMFSum = sensorRangeMFSum + sensorRangeMF;  // Calcula a Avg de molhamento foliar
  
}

void testDelay() {
  currentReadingMs = millis();  // Leitura atual em milisegundos
  if ((currentReadingMs -  lastReadMs) >= delay_sensor) { // Verifica se o delay configurado passou
    collectsData();  // Realiza a leitura e calculos dos sensores
  }
}

void collectsData() {
  dateTime(); // Chama a função para coletar a data e hora

  readSensors();  // Chama a função de ler os sensores
  calcAvgs();  // Chama a função que calcula a média

  showSerial();      // Chama a função que mostra o os dados no monitor serial
  publishReadings();   // Chama a função para publicar as leituras no firebase

  publishAvgs(); // Chama a função para publicar as medias no firebase
  publishMins();
  publishMaxs();

   lastReadMs = currentReadingMs;   // Armazene o milésimo atual para a próxima iteração
}

ICACHE_RAM_ATTR void contadorBascula() {
  weighbridgeCounter++; // Acrescenta o contator toda vez que o sensor passar pelo ímã 
}

void redefinesPluviometerValues() { // Redefine todos os valores das variaveis do pluviometro
  weighbridgeCounter = 0;
  lastValueAccountWeighbridge = 0;
  currentValueCountWeighbridge = 0;
  rainVolume = 0;
}

ICACHE_RAM_ATTR void  readPluviometer() {
  readingCurrentWeighbridgeMs = millis(); // Leitura atual da bascula em milis

  if (((readingCurrentWeighbridgeMs - lastReadingWeighbridgeMs) >= delaySensorHallPluviometer) || (lastReadingWeighbridgeMs == 0)) {  // Verifica se o delay configurado passou
    detachInterrupt(pinHallSensorPluviometer);       // Desativa a interrupção ao calcular
    if (weighbridgeCounter > lastValueAccountWeighbridge) { //Teve alguma mudança no contador?
      currentValueCountWeighbridge++;  // Aumentar a contagem do contador da bascula
      lastValueAccountWeighbridge = weighbridgeCounter; // Atualizar o valor do último valor do contador da bascula
      rainVolume = currentValueCountWeighbridge * weighbridgeVolume * 1000 / catchmentArea;  // Calcular o volume de chuva, convertendo a bascula em mm

      if (rainVolume > hMax) {
        rainVolumeMax = rainVolume; // Armazena o máximo valor coletado de umidade
        rainVolumeMaxTime = timeClient.getFormattedTime();
      }
      if (rainVolume < hMin) {
        rainVolumeMin = rainVolume; // Armazena o mínimo valor coletado de umidade
        rainVolumeMinTime = timeClient.getFormattedTime();
      }
    }
    lastReadingWeighbridgeMs = readingCurrentWeighbridgeMs; // Armazena a leitura da bascula atual para a próxima iteração
    attachInterrupt(digitalPinToInterrupt(pinHallSensorPluviometer), contadorBascula, CHANGE);  // Ativa a interrupção novamente, CHANGE = aciona a interrupção sempre que o pino altera o valor
  }
}

void readSensors() {
  counterAvg++;
  
  readDTH22Sensor();   // Chama a função que realiza a leitura do sensor de temperatura e umidade
  readSensorBh();      // Chama a função que realiza a leitura do sensor de temperatura e umidade
  readSensorBmp280();  // Chama a função que realiza a leitura do sensor de pressão atmosférica e altitude
  readSensorML8511();  // Chama a função que realiza a leitura do sensor de índice ultravioleta
  readSensorCSMSV12(); // Chama a função que realiza a leitura do sensor capacitivo do solo
  readSensorMhrd();    // Chama a função que realiza a leitura do sensor de molhamento foliar
  readPluviometer();   // Chama a função que realiza a leitura do pluviometro
  readAnemometer();    // Chama a função que realiza a leitura do anemometro

  thermalSensationCalc();

  if (dayStamp != lastDayStamp) { // Se mudar o data e redefine os valores do pluviometro 

    ESP.restart();

    redefinesPluviometerValues(); // Chama a função de redefinição dos valores do pluviometro

    redefineAvg();

    collectsMinMaxValues();

    lastDayStamp = dayStamp; // Pega a data e armazena como dia anterior
  }
}

void redefineAvg(){
    counterAvg = 0;
  
    hSum  = 0;
    tSum  = 0;

    thermalSensationSum = 0;
    
    bhSum = 0;

    bmppSum = 0;

    UvIndexSum = 0;

    sensorRangeMFSum = 0;

    sensorRangeCSMSV12Sum = 0;

    windSpeedSum = 0;
}

void readDTH22Sensor() {
  h = dht.readHumidity();     // Realiza a leitura da umidade e armazena o valor coletado
  t = dht.readTemperature();  // Realiza a leitura da temperatura e armazena o valor coletado

  if (h > hMax) {
    hMax = h; // Armazena o máximo valor coletado de umidade
    hMaxTime = timeClient.getFormattedTime();
  }
  if (h < hMin) {
    hMin = h; // Armazena o mínimo valor coletado de umidade
    hMinTime = timeClient.getFormattedTime();
  }

  if (t > tMax) {
    tMax = t; // Armazena o máximo valor coletado de temperatura
    tMaxTime = timeClient.getFormattedTime();
  }
  if (t < tMin) {
    tMin = t; // Armazena o mínimo valor coletado de temperatura
    tMinTime = timeClient.getFormattedTime();
  }
  
  hSum = hSum + h;
  tSum = tSum + tSum;
}

void  readSensorBh() {
  bh = Bh1750.readLightLevel(); // Realiza a leitura da luminosidade e armazena o valor coletado
  
  if (bh > bhMax) {
    bhMax = bh; // Armazena o máximo valor coletado de luminosidade
    bhMaxTime = timeClient.getFormattedTime();
  }
  if (bh < bhMin) {
    bhMin = bh; // Armazena o mínimo valor coletado de luminosidade
    bhMaxTime = timeClient.getFormattedTime();
  }

  bhSum = bhSum + bh;  // Armazena o todo valor coletado de luminosidade para calcular a Avg
  
}

void  readSensorBmp280() {
  bmpp = (Bmp280.readPressure()/100); // Realiza a leitura da pressão atmosférica e armazena o valor coletado
  bmpa = Bmp280.readAltitude(1014.66); // Realiza a leitura da altitude e armazena o valor coletado

  if (bmpp > bmppMax) {
    bmppMax = bmpp; // Armazena o máximo valor coletado da pressão atmosférica
    bmppMaxTime = timeClient.getFormattedTime();
  }
  if (bmpp < bmppMin) {
    bmppMin = bmpp; // Armazena o mínimo valor coletado da pressão atmosférica
    bmppMinTime = timeClient.getFormattedTime();
  }

  if (bmpa > bmpaMax) {
    bmpaMax = bmpa; // Armazena o máximo valor coletado da altitude
    bmpaMaxTime = timeClient.getFormattedTime();
  }
  if (bmpa < bmpaMin) {
    bmpaMin = bmpa; // Armazena o mínimo valor coletado da altitude
    bmpaMinTime = timeClient.getFormattedTime();
  }

  bmppSum = bmppSum + bmpp;  // Armazena o todo valor coletado da umidade para calcular a Avg
  bmpaSum = bmpaSum + bmpa;  // Armazena o todo valor coletado da altitude para calcular a Avg

}

void publishAvgs() {
  DynamicJsonBuffer jsonBuffer; // Chama a biblioteca e lida com o gerenciamento de memória e chama o analisador do json
  JsonObject& root = jsonBuffer.createObject(); // Cria um objto Json que será enviado ao firebase
  JsonObject& rootTime = root.createNestedObject ("timestamp"); // Cria um objto Json de data e hora no formato timestamp
  root["dht22MoistureAvg"] = hAvg; // Cria a variável com o valor da Avg da umidade que será enviado ao firebase
  root["dht22TemperatureAvg"] = tAvg;
  root["thermalSensationAvg"] = thermalSensationAvg;
  root["bh1750BrightnessAvg"] = bhAvg;
  root["bmp280PressureAvg"] = bmppAvg;
  root["uvm30aIndexUvAvg"] = UvIndexAvg;
  root["mhrdWettingAvg"] = sensorRangeMFAvg;
  root["csmsv12SoilAvg"] = sensorRangeCSMSV12Avg;
  root["anemometerAvg"] = windSpeedAvg;
  Firebase.set("Prototipo0/" + dayStamp + "/Avg", root); // Envia o objto Json para o firebase com as Avgs no caminho definido
  if (Firebase.failed()) {
    Serial.print("pushing /logs failed:"); // Caso o objto Json falhe avisa no monitor serial
    Serial.println(Firebase.error());
    return;
  }
}

void publishMaxs() {
  DynamicJsonBuffer jsonBuffer; // Chama a biblioteca e lida com o gerenciamento de memória e chama o analisador do json
  JsonObject& root = jsonBuffer.createObject(); // Cria um objto Json que será enviado ao firebase
  JsonObject& rootTime = root.createNestedObject ("timestamp"); // Cria um objto Json de data e hora no formato timestamp
  root["dht22MoistureMax"] = hMax; // Cria a variável com o valor da Avg da umidade que será enviado ao firebase
    root["dht22MoistureMaxTimestamp"] = hMaxTime;
  root["dht22TemperatureMax"] = tMax;
    root["dht22TemperatureMaxTimestamp"] = tMaxTime;
  root["thermalSensationMax"] = thermalSensationMax;
    root["thermalSensationMaxTimestamp"] = thermalSensationMaxTime;
  root["bh1750BrightnessMax"] = bhMax;
    root["bh1750BrightnessMaxTimestamp"] = bhMaxTime;
  root["bmp280PressureMax"] = bmppMax;
    root["bmp280PressureMaxTimestamp"] = bmppMaxTime;
  root["uvm30aIndexUvMax"] = UvIndexMax;
    root["uvm30aIndexUvMaxTimestamp"] = UvIndexMaxTime;
  root["mhrdWettingMax"] = sensorRangeMFMax;
    root["mhrdWettingMaxTimestamp"] = sensorRangeMFMaxTime;
  root["csmsv12SoilMax"] = sensorRangeCSMSV12Max;
    root["csmsv12SoilMaxTimestamp"] = sensorRangeCSMSV12MaxTime;
  root["anemometerMax"] = windSpeedMax;
    root["anemometerMaxTimestamp"] = windSpeedMaxTime;
  root["pluviometerMax"] = rainVolumeMax;;
    root["pluviometerMaxTimestamp"] = rainVolumeMaxTime;;
  Firebase.set("Prototipo0/" + dayStamp + "/Max", root); // Envia o objto Json para o firebase com as Avgs no caminho definido
  if (Firebase.failed()) {
    Serial.print("pushing /logs failed:"); // Caso o objto Json falhe avisa no monitor serial
    Serial.println(Firebase.error());
    return;
  }
}

void publishMins() {
  DynamicJsonBuffer jsonBuffer; // Chama a biblioteca e lida com o gerenciamento de memória e chama o analisador do json
  JsonObject& root = jsonBuffer.createObject(); // Cria um objto Json que será enviado ao firebase
  JsonObject& rootTime = root.createNestedObject ("timestamp"); // Cria um objto Json de data e hora no formato timestamp
  root["dht22MoistureMin"] = hMin; // Cria a variável com o valor da Avg da umidade que será enviado ao firebase
    root["dht22MoistureMinTimestamp"] = hMinTime;
  root["dht22TemperatureMin"] = tMin;
    root["dht22TemperatureMinTimestamp"] = tMinTime;
  root["thermalSensationMin"] = thermalSensationMin;
    root["thermalSensationMinTimestamp"] = thermalSensationMinTime;
  root["bh1750BrightnessMin"] = bhMin;
    root["bh1750BrightnessMinTimestamp"] = bhMinTime;
  root["bmp280PressureMin"] = bmppMin;
    root["bmp280PressureMinTimestamp"] = bmppMinTime;
  root["uvm30aIndexUvMin"] = UvIndexMin;
    root["uvm30aIndexUvMinTimestamp"] = UvIndexMinTime;
  root["mhrdWettingMin"] = sensorRangeMFMin;
    root["mhrdWettingMinTimestamp"] = sensorRangeMFMinTime;
  root["csmsv12SoilMin"] = sensorRangeCSMSV12Min;
    root["csmsv12SoilMinTimestamp"] = sensorRangeCSMSV12MinTime;
  root["anemometerMin"] = windSpeedMin;
    root["anemometerMinTimestamp"] = windSpeedMinTime;
  root["pluviometerMin"] = rainVolumeMin;
    root["pluviometerMinTimestamp"] = rainVolumeMinTime;
  Firebase.set("Prototipo0/" + dayStamp + "/Min", root); // Envia o objto Json para o firebase com as Avgs no caminho definido
  if (Firebase.failed()) {
    Serial.print("pushing /logs failed:"); // Caso o objto Json falhe avisa no monitor serial
    Serial.println(Firebase.error());
    return;
  }
}

float map1(float x, float in_min, float in_max, float out_min, float out_max) { // Função do map criada para os valores em float
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

void windSpeedCalc() {
  windSpeed = 0; // Váriavel que armazenara o valor da velocidade do vento
  anemometerCounter = 0; // Váriavel que armazenara o valor do contador do anemometro
  attachInterrupt(0, addcontador, RISING); // Configura a interrupção que aciona a função do contador 
  unsigned long millis(); // Cria a váriavel que retorna o número de milissegundos passados desde que a placa Arduino começou a executar o programa atual.
  long startTime = millis();
  while (millis() < startTime + period) {
    yield();  // Espera até o period terminar e realiza a função novamente
  }
}

void RPMcalc() {
  RPM = ((anemometerCounter) * 60) / (period / 1000); // Calcular revoluções por minuto (RPM)
}

void KMHcalc() {
  windSpeed = (((4 * pi * radius * RPM) / 60) / 1000) * 3.6; // Calcular a velocidade do vento em km/h

  if (windSpeed > windSpeedMax) {
    windSpeedMax = windSpeed; // Armazena o máximo valor coletado da velocidade do vento
    windSpeedMaxTime = timeClient.getFormattedTime();
  }
  if (windSpeed < windSpeedMin) {
    windSpeedMin = windSpeed; // Armazena o mínimo valor coletado da velocidade do vento
    windSpeedMinTime = timeClient.getFormattedTime();
  }
  
  windSpeedSum = windSpeedSum + windSpeed;  // Armazena todo valor coletado da velocidade do vento para calcular a Avg
}

ICACHE_RAM_ATTR void addcontador() {
  anemometerCounter++; // Acrescenta o contado do anemometro
}

void readAnemometer() {
  windSpeedCalc();  // Chama a função que le o senser hall do anemometro
  RPMcalc();  // Chama a função que calcula o numero de RPM
  KMHcalc(); // Chama a função que calcula a velocidade do vento em km/h
}

void collectsMinMaxValues() {
  while (!timeClient.update()) {
    timeClient.forceUpdate();   // Força atualização do Protocolo NTP para data e hora
  }

  hMin = dht.readHumidity(); // Realiza a leitura da umidade e armazena o valor coletado para verificar o minimo valor
  hMinTime = timeClient.getFormattedTime();
  hMax = dht.readHumidity(); // Realiza a leitura da umidade e armazena o valor coletado para verificar o maximo valor
  hMaxTime = timeClient.getFormattedTime();

  tMin = dht.readTemperature();  // Realiza a leitura da temperatura e armazena o valor coletado para verificar o minimo valor
  tMinTime = timeClient.getFormattedTime();
  tMax = dht.readTemperature();  // Realiza a leitura da temperatura e armazena o valor coletado para verificar o maximo valor
  tMaxTime = timeClient.getFormattedTime();

  bhMin = Bh1750.readLightLevel(); // Realiza a leitura da luminosidade e armazena o valor coletado para verificar o minimo valor
  bhMinTime = timeClient.getFormattedTime();
  bhMax = Bh1750.readLightLevel(); // Realiza a leitura da luminosidade e armazena o valor coletado para verificar o maximo valor
  bhMaxTime = timeClient.getFormattedTime();

  bmppMin = (Bmp280.readPressure()/100); // Realiza a leitura da pressão atmosférica e armazena o valor coletado para verificar o minimo valor
  bmppMinTime = timeClient.getFormattedTime();
  bmppMax = (Bmp280.readPressure()/100); // Realiza a leitura da pressão atmosférica e armazena o valor coletado para verificar o maximo valor
  bmppMaxTime = timeClient.getFormattedTime();

  bmpaMin = Bmp280.readAltitude(1014.66); // Realiza a leitura da altitude e armazena o valor coletado para verificar o minimo valor
  bmpaMinTime = timeClient.getFormattedTime();
  bmpaMax = Bmp280.readAltitude(1014.66); // Realiza a leitura da altitude e armazena o valor coletado para verificar o maximo valor
  bmpaMaxTime = timeClient.getFormattedTime();

  adc1 = ads.readADC_SingleEnded(1);  // Armazena o valor coletado da porta A1 do ADS1115
   sensorValueCSMSV12 = (adc1 * 0.1875) / 1000; // Armazena e calcula voltagem da porta com os valores dados do datasheet do ADS1115
  sensorRangeCSMSV12Min = map1( sensorValueCSMSV12, sensorMinCSMSV12, sensorMaxCSMSV12, 100, 0);  // Mapeia os valores coletados da voltagem entre 0 e 100
  sensorRangeCSMSV12MinTime = timeClient.getFormattedTime();
  sensorRangeCSMSV12Max = map1( sensorValueCSMSV12, sensorMinCSMSV12, sensorMaxCSMSV12, 100, 0);  // Mapeia os valores coletados da voltagem entre 0 e 
  sensorRangeCSMSV12MaxTime = timeClient.getFormattedTime();

  adc2 = ads.readADC_SingleEnded(2);  // Armazena o valor coletado da porta A2 do ADS1115
   sensorValueMF = (adc2 * 0.1875) / 1000;  // Armazena e calcula voltagem da porta com os valores dados do datasheet do ADS1115
  sensorRangeMFMin  = map1( sensorValueMF, sensorMinMF, sensorMaxMF, 100, 0);  // Mapeia os valores coletados da voltagem entre 0 e 100
  sensorRangeMFMinTime = timeClient.getFormattedTime();
  sensorRangeMFMax = map1( sensorValueMF, sensorMinMF, sensorMaxMF, 100, 0);  // Mapeia os valores coletados da voltagem entre 0 e 100
  sensorRangeMFMaxTime = timeClient.getFormattedTime();

  sensorValueML8511 = analogRead(A0); // Armazena o valor coletado da porta A0 do ADS1115
  sensorVoltageML8511 = ( sensorValueML8511 * 0.1875) / 1000;  // Armazena e calcula voltagem da porta com os valores dados do datasheet do ADS1115
  UvIndexMin = sensorVoltageML8511 * 100; // Armazena o resultado do valor do indice UV
  UvIndexMinTime = timeClient.getFormattedTime();
  UvIndexMax = sensorVoltageML8511 * 100; // Armazena o resultado do valor do indice UV
  UvIndexMaxTime = timeClient.getFormattedTime();

  windSpeedMin = (((4 * pi * radius * RPM) / 60) / 1000) * 3.6; // Calcular a velocidade do vento em km/h
  windSpeedMinTime = timeClient.getFormattedTime();
  windSpeedMax = (((4 * pi * radius * RPM) / 60) / 1000) * 3.6; // Calcular a velocidade do vento em km/h
  windSpeedMaxTime = timeClient.getFormattedTime();

  rainVolumeMinTime = timeClient.getFormattedTime();
  rainVolumeMaxTime = timeClient.getFormattedTime();

  thermalSensation = 33 + (10 * sqrt(windSpeed || 0) + (10.45 - windSpeed || 0)) * ((t - 33) / 22);
  thermalSensationMin = 33 + (10 * sqrt(windSpeed || 0) + (10.45 - windSpeed || 0)) * ((t - 33) / 22);
  thermalSensationMinTime = timeClient.getFormattedTime();
  thermalSensationMax = 33 + (10 * sqrt(windSpeed || 0) + (10.45 - windSpeed || 0)) * ((t - 33) / 22);
  thermalSensationMaxTime = timeClient.getFormattedTime();
}

void thermalSensationCalc() {
  
  thermalSensation = 33 + (10 * sqrt(windSpeed) + 10.45 - windSpeed) * ((t - 33) / 22);

  if (thermalSensation > thermalSensationMax) {
    thermalSensationMax = thermalSensation; // Armazena o máximo valor coletado da velocidade do vento
    thermalSensationMaxTime = timeClient.getFormattedTime();
  }
  if (thermalSensation < thermalSensationMin) {
    thermalSensationMin = thermalSensation; // Armazena o mínimo valor coletado da velocidade do vento
    thermalSensationMinTime = timeClient.getFormattedTime();
  }
  
  thermalSensationSum = thermalSensationSum + thermalSensation;  // Armazena todo valor coletado da velocidade do vento para calcular a Avg

}

void calcAvgs(){

    hAvg = hSum / counterAvg;  // Calcula a media de umidade
    tAvg = tSum / counterAvg;  // Calcula a media de temperatura
    thermalSensationAvg = thermalSensationSum / counterAvg;

    bhAvg = bhSum / counterAvg;  // Calcula a media de luminosidade

    bmppAvg = bmppSum / counterAvg;  // Calcula a media de pressão atmosférica

    UvIndexAvg = UvIndexSum / counterAvg;  // Calcula a media de altitude

    sensorRangeMFAvg = sensorRangeMFSum / counterAvg;  // Calcula a media de umidade de solo

    sensorRangeCSMSV12Avg = sensorRangeCSMSV12Sum / counterAvg;  // Calcula a media de molhamento foliar

    windSpeedAvg = windSpeedSum / counterAvg;  // Calcula a media da velocidade do vento

}
