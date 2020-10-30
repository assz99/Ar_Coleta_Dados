

/**
 * FreeRTOS
 */
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/timers.h"
#include <esp_system.h>
#include <time.h>
#include <sys/time.h>
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

struct RecvData
{
    char *buf{};
    String from{};
    String to{};
    String projName{};
    int timeStamp{};
    String recvMsg;
};
timeval tv;
int controleTimeStamp[10];
String controleMsg[10];
String localAddress;
String destino = "b8:27:eb:8e:94:f2";
String projNome = "arCond";
String msgSensores;
int timeStamp;

void updateRtcInterno(){
  DateTime now = rtc.now();
  int x = now.unixtime();
  Serial.println("Rtc externo = "+ String(x));
  tv.tv_sec = x;
  settimeofday(&tv, NULL);
  Serial.println("RTC Interno Atualizado");
  Serial.println("Rtc ixterno = "+ String(time(NULL)));
  }

int pegarTimeStamp()
{
    DateTime now = rtc.now();
    //int x = now.unixtime();
    int x = time(NULL);
    //Serial.println("Pegou o Time Stamp: " + String(x));
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
    Serial.println("Corrente:" + String(Irms) + "   Potencia:" + String(potencia) + "  Kwh:" + String(kwhTotal));
}

void task_medicao(void *pvParameter)
{
    for (;;)
    {
        int counter = 1;

        //Serial.println(xPortGetCoreID());
        // Calcula quantidade de tempo desde a última measurment realpower.
        double Irms1 = SCT013.calcIrms(1480);
        double Irms2 = SCT013.calcIrms(1480);
        double Irms3 = SCT013.calcIrms(1480);
        Irms = (Irms1 + Irms2 + Irms3) / 3;
        // Calcula o valor da Corrente
        //Serial.println("Irms Medicao = " + String(Irms, 2));
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
        //Serial.println("Corrente:" + String(Irms) + "   Potencia:" + String(potencia) + "  Kwh:" + String(kwhTotal, 4) + " Wh:" + String(kwhTotal * 1000));
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}

void task_sensor(void *pvParameter)
{
    for (;;)
    {
        temp_DHT();
        //medicaoPotencia();
        msgSensores = String(temperatura) + "!" + String(humidade) + "!" + String(Irms) + "!" + String(potencia) + "!" + String(kwhTotal, 4);
        vTaskDelay(5000 / portTICK_PERIOD_MS);
    }
}

void senderFactory(uint8_t *data)
{

    localAddress.toLowerCase();

    msg = localAddress + "!" + destino + "!" + projNome + "!" + String(pegarTimeStamp()) + "!" + msgSensores;
    for (int i = 0; i < msg.length(); i++)
    {
        data[i] = (uint8_t)msg[i];
    }
}

void enviarLoRa(int timeStamp,String msg){
  uint8_t data[RH_RF95_MAX_MESSAGE_LEN];
  memset(data, '\0', sizeof(data));
  localAddress.toLowerCase();
  msg = localAddress + "!" + destino + "!" + projNome + "!" + String(timeStamp) + "!" + msg;
  for (int i = 0; i < msg.length(); i++)
  {
      data[i] = (uint8_t)msg[i];
  }
  Serial.println("LoRa 1 = Criando Pacote ");
  rf95.send(data, sizeof(data));
  Serial.println("LoRa 2 = Esperando Pacote ser enviado");
  rf95.waitPacketSent();
  Serial.println("LoRa 3 = Enviou a mensagem: ");
  Serial.println(msg);
  
  
  }

void task_enviar_sensores(void *pvParameter)
{
    for (;;)
    {
        int _timeStamp = pegarTimeStamp();
        enviarLoRa(_timeStamp,msgSensores);
        //controleGravarLoRa(_timeStamp,msgSensores);
        vTaskDelay(20000 / portTICK_PERIOD_MS);
    }
}



void configRTC()
{
    String y = "";
    while (Serial.available() > 0)
    {
        // lê do buffer o dado recebido:
        y += (char)Serial.read();
    }
    if (y.length() > 0)
    {
        rtc.adjust((uint32_t)y.toInt());
        tv.tv_sec = y.toInt();
        settimeofday(&tv, NULL);
        DateTime now = rtc.now();
        Serial.println("Hora ajustada Externo. Unix = " + String(now.unixtime()));
        Serial.println("Hora ajustada Interno. Unix = " + String(time(NULL)));
        y = "";
    }
}

void recvDataFactory(char *buf, RecvData &data)
{

    data.buf = buf;

    if (String(buf).indexOf("!") > 0)
    {
        data.from = String(strtok(buf, "!"));
        data.to = String(strtok(nullptr, "!"));
        data.projName = String(strtok(nullptr, "!"));
        data.timeStamp = String(strtok(nullptr, "!")).toInt();
        data.recvMsg = String(strtok(nullptr, ""));
    }
}

void controleRetirarLoRa(int _timeStamp){
  
  for(int i = 0; i<11; i++){
    if(controleTimeStamp[i]==_timeStamp){
      Serial.println("Retirando Msg: " + String(controleTimeStamp[i]) +  " no Controle");
      controleTimeStamp[i] = 0;
      controleMsg[i] = "";
      return;
      }
    if(i==10){
      Serial.println("TimeStamp nao esta gravado");
      }
    }
  }

void controleGravarLoRa(int _timeStamp, String msg){
  
  for(int i =0; i< 11; i++){
    if(controleTimeStamp[i] == 0){
      controleTimeStamp[i] = _timeStamp;
      controleMsg[i] = msg;
      Serial.println("Gravando Msg: " + String(controleTimeStamp[i]) + " no Controle");
      return;
      }
      if(i == 10){
        Serial.println("Vetor Controle lotado");
        }
    }  
  }

void checarControle(void *pvParameter){
  for(;;){
  int controlTimeStamp = pegarTimeStamp();
  for(int i = 0; i<11; i++){
    if(controlTimeStamp - controleTimeStamp[i] >20){
      Serial.println("Re-enviando Mensagem do control");
      enviarLoRa(controleTimeStamp[i],controleMsg[i]);
      } 
    }
    vTaskDelay(20000 / portTICK_PERIOD_MS);
  }
}
void recv(void *pvParameter)
{
    for (;;)
    {   
        
        if (rf95.available())
        {
            Serial.println("Recebeu uma mensagem");
            // Should be a message for us now
            uint8_t buf[RH_RF95_MAX_MESSAGE_LEN];
            uint8_t len = sizeof(buf);

            memset(buf, '\0', len);

            if (rf95.recv(buf, &len))
            {
                Serial.println((char *)buf);
                static RecvData data{};
                recvDataFactory((char *)buf, data);
                if(data.to.equals(localAddress)){
                  Serial.println("Msg e para mim");
                    /*if(data.recvMsg == "OK"){
                      controleRetirarLoRa(data.timeStamp);
                        }*/
                    }else{
                      Serial.println("Msg nao e para mim");
                      }      
                }
            }
            vTaskDelay(100 / portTICK_PERIOD_MS);
        }
    }

void setup()
{
    Serial.begin(115200);
    Serial.println("Inicializando");
    SCT013.current(A0, 5.4);
    
    if (!rtc.begin())
    {
        Serial.println("Couldn't find RTC");
        Serial.flush();
        abort();
        //oledInit();
        Serial.println("Inicializando OLED");
        delay(50);
    }
    Serial.println("RTC Inicializado com sucesso");
    updateRtcInterno();
    
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
     //Criação da task da medição energia
    if ((xTaskCreate(task_medicao, "task_medicao", 4048, NULL, 5, NULL)) != pdTRUE)
    {
        ESP_LOGI(TAG, "error - nao foi possivel alocar task_sensor.\n");
        return;
    }
    else
    {
        Serial.println("task_medicao criada com sucesso");
    }
    //Criação da task do sensor DHT
    if ((xTaskCreate(task_sensor, "task_sensor", 4048, NULL, 5, NULL)) != pdTRUE)
    {
        ESP_LOGI(TAG, "error - nao foi possivel alocar task_sensor.\n");
        return;
    }
    else
    {
        Serial.println("Task_sensor criada com sucesso");
    }
    //Criação task Recebimento LoRa
    if ((xTaskCreate(recv, "recv_LoRa", 4048, NULL, 2, NULL)) != pdTRUE)
    {
        ESP_LOGI(TAG, "error - nao foi possivel alocar task_sensor.\n");
        return;
    }
    else
    {
        Serial.println("recv_LoRa criada com sucesso");
    }
    //Criação task para enviar dados sensores
    if ((xTaskCreate(task_enviar_sensores, "task_enviar_sensores", 4048, NULL, 3, NULL)) != pdTRUE)
    {
        ESP_LOGI(TAG, "error - nao foi possivel alocar task_sensor.\n");
        return;
    }
    else
    {
        Serial.println("task_enviar_sensores criada com sucesso");
    }
    //task para controle do LoRa
    /*if ((xTaskCreate(checarControle, "checarControle", 4048, NULL, 3, NULL)) != pdTRUE)
    {
        ESP_LOGI(TAG, "error - nao foi possivel alocar task_sensor.\n");
        return;
    }
    else
    {
        Serial.println("checarControle criada com sucesso");
    }*/
}


void loop()
{
    //task_enviar();
    configRTC();
    vTaskDelay(100 / portTICK_PERIOD_MS);
}
