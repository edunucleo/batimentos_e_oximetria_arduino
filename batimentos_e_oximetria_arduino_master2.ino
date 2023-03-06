/*
  Detecção otica de batimentos cardiacos e Oximetria (Algoritimo SPK) usando o MAX30102
  By: Eduardo Vieira de Campos
  Date: 28/02/2023
  https://github.com/????????????

   É melhor prender o sensor ao seu dedo usando um elástico ou outro meio de aperto
   dispositivo. Os humanos geralmente são ruins em aplicar pressão constante a uma coisa. Quando você
   pressione o dedo contra o sensor varia o suficiente para causar o sangue em seu
   dedo flua de maneira diferente, o que faz com que as leituras do sensor fiquem instáveis.

  Conexões de Hardware - arduino:
  -5V = 5V (3.3V is allowed)
  -GND = GND
  -SDA = A4 (or SDA)
  -SCL = A5 (or SCL)
  -INT = Not connected

   É melhor prender o sensor ao seu dedo usando um elástico ou outro meio de aperto
   O MAX30102 pode lidar com lógica I2C de 5 V ou 3,3 V. Recomendamos alimentar a placa com 5V
   mas também funcionará em 3,3V.
*/

#include <Wire.h>
#include "MAX30105.h"
#include "spo2_algorithm.h"

MAX30105 particleSensor;

#define MAX_BRIGHTNESS 255

#if defined(__AVR_ATmega328P__) || defined(__AVR_ATmega168__)
//Arduino Uno não tem SRAM suficiente para armazenar 100 amostras de dados de led infravermelho e dados de led vermelho no formato de 32 bits
//Para resolver este problema, o MSB de 16 bits dos dados amostrados será truncado. As amostras tornam-se dados de 16 bits.
uint16_t irBuffer[100]; //dado LED sensor infravermelho
uint16_t redBuffer[100];  //dado LED sensor vermelho
#else
uint32_t irBuffer[100]; //dado LED sensor infravermelho
uint32_t redBuffer[100];  //dado LED sensor vermelho
#endif

int32_t bufferLength; //tamanho do buffer
int32_t spo2; //SPO2(oximetria) valor
int8_t validSPO2; //indicador para mostrar se o cálculo do SPO2 é válido
int32_t heartRate; //batimento cardiaco valor
int8_t validHeartRate; //indicador para mostrar se o cálculo da frequência cardíaca é válido

byte pulseLED = 11; //será o pino PWM
byte readLED = 13; //Pisca a cada leitura de dados

void setup()
{
  Serial.begin(115200); 
  pinMode(7, OUTPUT);
  pinMode(pulseLED, OUTPUT);
  pinMode(readLED, OUTPUT);

  // inicializa o sensor
  if (!particleSensor.begin(Wire, I2C_SPEED_FAST)) //Use default I2C port, 400kHz speed
  {
    Serial.println(F("MAX30102 não foi encontrado. Veja a fiação ou energia."));
    while (1);
  }

  byte ledBrightness = 60; //Opções: 0=Off to 255=50mA
  byte sampleAverage = 4; //Opções: 1, 2, 4, 8, 16, 32
  byte ledMode = 2; //Opções: 1 = Red only, 2 = Red + IR, 3 = Red + IR + Green
  byte sampleRate = 100; //Opções: 50, 100, 200, 400, 800, 1000, 1600, 3200
  int pulseWidth = 411; //Opções: 69, 118, 215, 411
  int adcRange = 4096; //Opções: 2048, 4096, 8192, 16384

  particleSensor.setup(ledBrightness, sampleAverage, ledMode, sampleRate, pulseWidth, adcRange); //Configure sensor with these settings
}

void loop()
{
  digitalWrite(7, HIGH);
  bufferLength = 100; // comprimento do buffer de 100 armazena 4 segundos de amostras rodando a 25 sps

  //lê as primeiras 100 amostras e determina o alcance do sinal
  for (byte i = 0 ; i < bufferLength ; i++)
  {
    while (particleSensor.available() == false) //faça enquanto não tiver novos dados
      particleSensor.check(); //checa se há novos dados no sensor

    redBuffer[i] = particleSensor.getRed();
    irBuffer[i] = particleSensor.getIR();
    particleSensor.nextSample(); //proxima amostra
  }

  //calcula frequência cardíaca e SpO2(ox) após as primeiras 100 amostras (primeiros 4 segundos de amostras)
  maxim_heart_rate_and_oxygen_saturation(irBuffer, bufferLength, redBuffer, &spo2, &validSPO2, &heartRate, &validHeartRate);

  //Colhendo amostras continuamente de MAX30102. A frequência cardíaca e o SpO2 são calculados a cada 1 segundo
  while (1)
  {
    // despeja os primeiros 25 conjuntos de amostras na memória e desloca os últimos 75 conjuntos de amostras para o topo
    for (byte i = 25; i < 100; i++)
    {
      redBuffer[i - 25] = redBuffer[i];
      irBuffer[i - 25] = irBuffer[i];
    }

    //colhe 25 conjuntos de amostras antes de calcular a frequência cardíaca.
    for (byte i = 75; i < 100; i++)
    {
      while (particleSensor.available() == false) //do we have new data?
        particleSensor.check(); //Check the sensor for new data

      digitalWrite(readLED, !digitalRead(readLED)); //Blink onboard LED with every data read

      redBuffer[i] = particleSensor.getRed();
      irBuffer[i] = particleSensor.getIR();
      particleSensor.nextSample(); //proxima amostra

      //envia amostras e resultado de cálculo para programa de terminal através de UART/mudar para enviar para esp01
      Serial.print(F("Batimento Cardiaco="));
      Serial.print(heartRate, DEC);
      Serial.print(F(", spo="));
      Serial.println(spo2, DEC);

      Serial.write(spo2);
    }

     //Depois de coletar 25 novas amostras, recalcule BC e Saturação
    maxim_heart_rate_and_oxygen_saturation(irBuffer, bufferLength, redBuffer, &spo2, &validSPO2, &heartRate, &validHeartRate);
  }
}