/**
 * FreeRTOS
 */
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/timers.h"

/**
 * Libs
 */
#include <WiFi.h>
#include <Wire.h>
#include <SPI.h>
#include "EmonLib.h"
#include <SimpleDHT.h>
#include "Oled_api.h"

#include <SPI.h>
#include <RH_RF95.h>
#include <RHSoftwareSPI.h>


#include "RTClib.h"
//#include "LoRa_RadioHead.h"

RHSoftwareSPI spi;

RH_RF95 rf95(18, 26);
EnergyMonitor SCT013;
SimpleDHT22 dht22(23);
RTC_DS3231 rtc;
String msg;
//Variaveis DHT 22
float humidade;
float temperatura;

//Variaveis EmonLib
double Irms;      // CORRENTE MEDIDA
int pinSCT = A0;  //Pino analógico conectado ao SCT-013 36
int tensao = 220; // TENSÃO NOMINAL
int potencia;     // POTENCIA CALCULADA
double kwhTotal = 0;
double kwhTotal_Acc = 0;


String localAddress;
String destino = "b8:27:eb:8e:94:f2";
String projNome = "arCond" ;
String msgSensores;

int pegarTimeStamp(){
    DateTime now = rtc.now();
     int x = now.unixtime();
    Serial.println("Pegou o Time Stamp: " + String(x));
    return x;
}

void temp_DHT()
{

    int err = SimpleDHTErrSuccess;

    if ((err = dht22.read2(&temperatura, &humidade, NULL)) != SimpleDHTErrSuccess)
    {
        Serial.print("Read DHT22 failed, err=");
        Serial.println(err);
    }
    /*Serial.print(temperatura);
    Serial.print(" *C, ");
    Serial.print(humidade);
    Serial.println(" RH%");*/
}

void medicaoPotencia()
{
    int counter = 1;

    //Serial.println(xPortGetCoreID());
    // Calcula quantidade de tempo desde a última measurment realpower.
    double Irms1 = SCT013.calcIrms(1480);
    double Irms2 = SCT013.calcIrms(1480);
    double Irms3 = SCT013.calcIrms(1480);
    Irms = (Irms1 + Irms2 + Irms3) / 3;
    // Calcula o valor da Corrente
    //Serial.println("Irms = " + String(Irms, 2));
    if (Irms >= 0 and Irms <= 0.5)
    {
        Irms = 0;
        if (counter == 0)
        {
        }
        else
        {
            //enviar_info(0); COLOCAR O TIMER DE ENVIAR EM 0
        }
        counter = 0;
    }
    else
    {

        Irms = Irms;
        counter = 1;
    }

    potencia = Irms * tensao; // Calcula o valor da Potencia Instantanea
    // Calcular o número de hoje de kWh consumido.
    kwhTotal = kwhTotal + ((potencia / 1000.0) * 1.0 / 3600.0);
    //Serial.println("Corrente:" + String(Irms) + "   Potencia:" + String(potencia) + "  Kwh:" + String(kwhTotal));
}

void task_sensor(void *pvParameter)
{
    for (;;)
    {
        temp_DHT();
        medicaoPotencia();
        msgSensores = String(temperatura) + "!" + String(humidade) + "!" + String(Irms) + "!" + String(potencia) + "!" + String(kwhTotal_Acc, 5);
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}

void senderFactory(uint8_t *data) {
    
    localAddress.toLowerCase();
    
     msg = localAddress + "!" + destino + "!" + projNome + "!" + String(pegarTimeStamp())+"!"+msgSensores;
    for (int i = 0; i < msg.length(); i++) {
        data[i] = (uint8_t) msg[i];
    }
   
}

void task_enviar(void *pvParameter)
{
    for (;;)
    {
     Serial.println("Entrou na task Enviar");
     uint8_t data[RH_RF95_MAX_MESSAGE_LEN];  
     memset(data, '\0', sizeof(data));
     senderFactory(data);
  
     rf95.send(data, sizeof(data));
     Serial.println("Esperando Pacote ser enviado");
     rf95.waitPacketSent();
     Serial.println("Enviou a mensagem: ");
     Serial.println(msg);
      vTaskDelay(20000 / portTICK_PERIOD_MS);
    }
}

void setup() {
  Serial.begin(115200);
    Serial.println("Inicializando");
     if (! rtc.begin()){
    Serial.println("Couldn't find RTC");
    Serial.flush();
    abort();
  //oledInit();
  Serial.println("Inicializando OLED");
  delay(50);
  SCT013.current(pinSCT, 5.4);
    
    
  }
    Serial.println("RTC Inicializado com sucesso");
    
    //loraConfig(915);
    spi.setPins(19, 27, 5);
    if (!rf95.init())
      Serial.println("init failed");
    Serial.println("LoRa Inicializado com sucesso");  
    rf95.setFrequency(915);
  
    
    ////////////////////////////////////////
    // Convertendo o MAC para MAC byte
    ////////////////////////////////////////
    String MAC_LOCAL = WiFi.macAddress();
    char ChMacLocal[18];
    String(MAC_LOCAL).toCharArray(ChMacLocal, 18);
    char *InfoMacLocal[6];
    InfoMacLocal[0] = strtok(ChMacLocal, ":");
    InfoMacLocal[1] = strtok(NULL, ":");
    InfoMacLocal[2] = strtok(NULL, ":");
    InfoMacLocal[3] = strtok(NULL, ":");
    InfoMacLocal[4] = strtok(NULL, ":");
    InfoMacLocal[5] = strtok(NULL, ":");
    String Mac_Local_Full;
    for (int s = 0; s < 6; s++)
    {
        Mac_Local_Full += InfoMacLocal[s];
    }
    localAddress = Mac_Local_Full;
    localAddress = WiFi.macAddress();
    
    Serial.println("Inicializando Tasks");
    /*oledLimpar();
    oledEscrever(0, 0, "" + String(MAC_LOCAL));
    oledEscrever(0, 16, "Novo: " + String(Mac_Local_Full));
    oledEscrever(0, 32, "[ESP - AR COND]");
*/
    //Criação da task do sensores
    if ((xTaskCreate(task_sensor, "task_sensor", 4048, NULL, 5, NULL)) != pdTRUE)
    {
        ESP_LOGI(TAG, "error - nao foi possivel alocar task_sensor.\n");
        return;
    }
    else
    {
        Serial.println("Task_sensor criada com sucesso");
    }

    if ((xTaskCreate(task_enviar, "task_enviar", 4048, NULL, 3, NULL)) != pdTRUE)
    {
        ESP_LOGI(TAG, "error - nao foi possivel alocar task_sensor.\n");
        return;
    }
    else
    {
        Serial.println("Task_enviar criada com sucesso");
    }
    
}

void loop() {
 //task_enviar();
  vTaskDelay(5000 / portTICK_PERIOD_MS);
}
