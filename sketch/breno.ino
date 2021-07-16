/*
 *  SimpleReceiver.cpp    - Adaptada para este projeto por chicodorea@gmail.com
 *  Demonstrates receiving NEC IR codes with IRrecv
 *  Copyright (C) 2020-2021  Armin Joachimsmeyer
 *  armin.joachimsmeyer@gmail.com
 *  This file is part of Arduino-IRremote https://github.com/Arduino-IRremote/Arduino-IRremote.
 *  MIT License
 *  
 *  ---------------------------------------------------------------------------------------------
 *  PROJETO..: CARRINHO ARDUINO COM CONTROLE REMOTO INFRA VERMELHO PARA BRENO SILVA
 *  AUTOR....: FRANCISCO D�REA (chicodorea@academico.ufs.br ou chicodorea@gmail.com)
 *  DATA.....: JULHO/2021
 *  ---------------------------------------------------------------------------------------------
 *  SETUP DA MONTAGEM:
 *  - 01 kit chassi 2WD robo (https://www.sermaker.com/kit-chassi-2wd-robo)........... R$   60,00
 *  - 01 Arduino Nano V3 (https://www.sermaker.com/arduino-nano-v3-cabo-usb).......... R$   40,00
 *  - 01 Modulo Shield Adaptador Base Para Expansao Arduino Nano V3.0................. R$   21,90
 *    (https://www.sermaker.com/modulo-shield-adaptador-base-para-expansao-arduino-nano-v30)
 *  - Base acrilica para Arduino Uno e Protoboard 400 Furos........................... R$    9,89
 *    (https://www.sermaker.com/base-acrilica-para-arduino-uno-e-protoboard-400-furos
 *  - 01 Controle remoto universal IR................................................. R$   11,91
 *     (https://www.marinostore.com/sensores/controle-remoto-universal-ir?parceiro=3040)
 *  - 01 Driver Ponte H Duplo L9110s.................................................. R$   12,90
 *       (https://www.sermaker.com/driver-ponte-h-duplo-l9110s)
 *  - 04 Pilhas AA Comum Zinco Energy Elgin........................................... R$    5,50
 *  - 01 Protoboard de 400 furos...................................................... R$   12,00
 *       (https://www.sermaker.com/protoboard-400-pontos)
 *  - 01 Buzzer Ativo 5V (https://www.sermaker.com/buzzer-ativo-5v)................... R$    2,89
 *  - 04 LEDs Difusos 5mm (Vermelho, verde e amarelo)................................. R$    1,00
 *  - 01 Receptor Universal Infravermelho VS1838B 38Khz............................... R$    1,55
 *       (https://www.sermaker.com/receptor-universal-infravermelho-vs1838b-38khz)
 *  - 01 Transistor NPN BC548......................................................... R$    0,60
 *  - 01 Resistor de 2,2k (pullup Sensor 1838T)
 *  - 01 Resistor de 1k (base transistor BC548)
 *  - 04 Resistores de 220 Ohms (limitadores de corrente para os leds)................ R$    1,50
 *  - Jumpers diversos de 10cn e 20cm para interligacao............................... R$    8,00
 *                                                                     TOTAL.......... R$  189,64
 *  ---------------------------------------------------------------------------------------------
 *  OBSERVACOES:
 *  - Seguindo a sugestao da biblioteca IR Remote, o sensor IR deve ser ligado no pino 2
 *    e um led vermelho no pino 13 para sinalizar recepcao de sinal do controle remoto
 *  - Outros pinos sao mapeados por esta biblioteca mas nao devem influenciar neste projeto.
 *  ---------------------------------------------------------------------------------------------
 */

// Mapeamento das teclas do controle remoto (modelo TVBox MxQ-4K, protocolo NEC)
#define FRAHM_CONTROL
// #define MXQ_CONTROL

#ifdef MXQ_CONTROL
#define PATRULHA                0x11             // 1 codigo da tecla que ativa o modo patrulha
#define VERDE                   0xF              // 1 codigo da tecla que faz o led verde ascender ou apagar
#define AMARELO                 0x10             // 2 codigo da tecla que faz o led amarelo ascender ou apagar
#define VERMELHO                0x43             // 3 codigo da tecla que faz o led vermelho ascender ou apagar
#define BUZINA                  0x19             // 4 codigo da tecla que faz a buzina ligar ou desligar
#define PARAR                   0x13             // 5 codigo da tecla que faz o carro parar
#define FRENTE                  0x16             // 6 codigo da tecla que faz o carro avancar
#define RE                      0x1A             // 7 codigo da tecla que faz o carro retroceder
#define DIREITA                 0x50             // 8 codigo da tecla que faz o carro virar para direita
#define ESQUERDA                0x51             // 9 codigo da tecla que faz o carro virar para esquerda
#define ESPERAMENOS             0xF4 
#define ESPERAMAIS              0xF5 
#endif

#ifdef FRAHM_CONTROL
#define PATRULHA                0x1A             //  Mode    - codigo da tecla que ativa o modo patrulha
#define VERDE                   0x3              //  >>|     - codigo da tecla que faz o led verde ascender ou apagar
#define AMARELO                 0x2              //  <<|     - codigo da tecla que faz o led amarelo ascender ou apagar
#define VERMELHO                0x1              //  >|      - codigo da tecla que faz o led vermelho ascender ou apagar
#define BUZINA                  0x1E             //  Mute    - codigo da tecla que faz a buzina ligar ou desligar
#define PARAR                   0xD              //  Tecla 5 - codigo da tecla que faz o carro parar
#define FRENTE                  0x1B             //  Tecla 2 - codigo da tecla que faz o carro avancar
#define RE                      0xF              //  Tecla 8 - codigo da tecla que faz o carro retroceder
#define DIREITA                 0xE              //  Tecla 6 - codigo da tecla que faz o carro virar para direita
#define ESQUERDA                0xC              //  Tecla 4 - codigo da tecla que faz o carro virar para esquerda
#define ESPERAMENOS             0x5              //  Vol-    - codigo da tecla que diminui o tempo de espera para parar o motor
#define ESPERAMAIS              0x6              //  Vol+    - codigo da tecla que aumenta o tempo de espera para parar o motor
#endif

// Mapeamento de hardware e velocidades do motor
#define buzzer                  3                // pino digital para o buzzer
#define motor_a1a               7                // pino digital para o conector A-1A da Ponte H L9110s
#define motor_a1b               6                // pino digital para o conector A-1B da Ponte H L9110s
#define motor_b1a               8                // pino digital para o conector B-1A da Ponte H L9110s
#define motor_b1b               9                // pino digital para o conector B-1B da Ponte H L9110s
#define led_vermelho           10                // pino digital para controlar o led vermelho (indicativo de re)
#define led_amarelo            11                // pino digital para controlar o led amarelo  (indicativo de parado)
#define led_verde              12                // pino digital para controlar o led verde    (indicativo de avancar)
#define veloc1                255                // valor PWM para velocidade 1 (maxima)
#define veloc2                200                // valor PWM para velocidade 2 (intermediaria)
#define veloc3                155                // valor PWM para velocidade 3 (baixa)

// Parametros da comunicacao com Sensor InfraRed 1838T
#define DECODE_NEC                               // Adotado padrao NEC para controle remoto
#include <Arduino.h>
#include "PinDefinitionsAndMore.h"
#include <IRremote.h>                            // Biblioteca IRremote, https://github.com/Arduino-IRremote/Arduino-IRremote
                                                 // Acesso em 12/07/2021

// Variaveis globais
unsigned long demora = millis();
unsigned long espera = 180;                      // tempo em milisegundos para espera de outro comando antes de parar
boolean patrulha = false;
boolean motor_ligado = false;
boolean buzina = false;

/**********************************************************
                    S   E   T   U   P
**********************************************************/
void setup() {
    Serial.begin(115200);
    Serial.println(F("START " __FILE__ " from " __DATE__ "\r\nUsando biblioteca de versao " VERSION_IRREMOTE));
    IrReceiver.begin(IR_RECEIVE_PIN, ENABLE_LED_FEEDBACK, USE_DEFAULT_FEEDBACK_LED_PIN);
    Serial.print(F("Pronto para receber um sinal do sensor infra vermelho conectado ao pino "));
    Serial.println(IR_RECEIVE_PIN);

    pinMode(led_verde,          OUTPUT);         // Define os pinos dos leds e buzzer como sa�da
    pinMode(led_amarelo,        OUTPUT);
    pinMode(led_vermelho,       OUTPUT);
    pinMode(buzzer,             OUTPUT);
    pinMode(motor_a1a,          OUTPUT);         // Define os pinos dos motores como saida
    pinMode(motor_a1b,          OUTPUT);
    pinMode(motor_b1a,          OUTPUT);
    pinMode(motor_b1b,          OUTPUT);
    pinMode(LED_BUILTIN,        OUTPUT);
    digitalWrite(motor_a1a,     LOW);            // Desliga todos os motores
    digitalWrite(motor_a1b,     LOW);
    digitalWrite(motor_b1a,     LOW);
    digitalWrite(motor_b1b,     LOW);
    digitalWrite(buzzer, HIGH);
    delay(150);
    digitalWrite(buzzer, LOW);
    for (int k=0; k<3; k++)
    {
       digitalWrite(led_vermelho, HIGH);
       digitalWrite(led_amarelo, HIGH);
       digitalWrite(led_verde, HIGH);
       delay(150);
       digitalWrite(led_vermelho, LOW);
       digitalWrite(led_amarelo, LOW);
       digitalWrite(led_verde, LOW);      
       delay(150);
    }
    digitalWrite(led_amarelo, HIGH);
}

/**********************************************************
                    L   O   O   P
**********************************************************/
void loop() 
{
    if (IrReceiver.decode()) 
    {
       IrReceiver.printIRResultShort(&Serial);
       if (IrReceiver.decodedIRData.protocol == UNKNOWN) 
       {
          IrReceiver.printIRResultRawFormatted(&Serial, true);
          Serial.println("Comando desconhecido");
       }
       IrReceiver.resume(); // Enable receiving of the next value
       if (IrReceiver.decodedIRData.command        == VERDE) {
          if (digitalRead(led_verde) ==     HIGH)
             digitalWrite(led_verde,        LOW);
           else
             digitalWrite(led_verde,        HIGH);
       } else if (IrReceiver.decodedIRData.command == AMARELO) {
           if (digitalRead(led_amarelo) ==  HIGH)
              digitalWrite(led_amarelo,     LOW);
           else
              digitalWrite(led_amarelo,     HIGH);
       } else if (IrReceiver.decodedIRData.command == VERMELHO) {
           if (digitalRead(led_vermelho) == HIGH)
              digitalWrite(led_vermelho,    LOW);
           else
              digitalWrite(led_vermelho,    HIGH);
       } else if (IrReceiver.decodedIRData.command == BUZINA) {
           if (buzina) buzina = false;
           else buzina = true;
       } else if (IrReceiver.decodedIRData.command == PARAR) {
           motorA('0');
           motorB('0');
           patrulha = false;
           buzina = false;
           digitalWrite(buzzer, HIGH);
           digitalWrite(led_verde,          LOW);
           digitalWrite(led_amarelo,        HIGH);
           digitalWrite(led_vermelho,       LOW);
       } else if (IrReceiver.decodedIRData.command == FRENTE) {
           motorA('L');
           motorB('R');
           Serial.println("Ir para frete");
           if (!patrulha)
           {
              digitalWrite(led_verde,       HIGH);
              digitalWrite(led_amarelo,     LOW);
              digitalWrite(led_vermelho,    LOW);
           }
       } else if (IrReceiver.decodedIRData.command == RE) {
           motorA('R');
           motorB('L');
           Serial.println("Voltar de re");
           if (!patrulha)
           {
              digitalWrite(led_verde,       LOW);
              digitalWrite(led_amarelo,     LOW);
              digitalWrite(led_vermelho,    HIGH);
           }
       } else if (IrReceiver.decodedIRData.command == DIREITA) {
           motorA('R');
           motorB('R');
           Serial.println("Virar a direita");
           if (!patrulha)
           {
              digitalWrite(led_verde,       HIGH);
              digitalWrite(led_amarelo,     HIGH);
              digitalWrite(led_vermelho,    LOW);
           }
       } else if (IrReceiver.decodedIRData.command == ESQUERDA) {
           motorA('L');
           motorB('L');
           Serial.println("Virar a esquerda");
           if (!patrulha)
           {
              digitalWrite(led_verde,       LOW);
              digitalWrite(led_amarelo,     HIGH);
              digitalWrite(led_vermelho,    HIGH);
           }
       } else if (IrReceiver.decodedIRData.command == PATRULHA) {
           if (patrulha) patrulha = false;
           else patrulha = true;
       } else if (IrReceiver.decodedIRData.command == ESPERAMENOS) {
           espera -=  5;
           if (espera < 150) espera = 150;
       } else if (IrReceiver.decodedIRData.command == ESPERAMAIS) {
           espera +=  5;
           if (espera > 250) espera = 250;
       } // fim do if command
    }   // fim do if IrReceiver

    if (buzina) digitalWrite(buzzer, HIGH);
    else digitalWrite(buzzer, LOW);
    
    ver_estado();

    if ((millis()-demora)>espera)
    {
       if (motor_ligado)
       {
          motorA('0');
          motorB('0');
       }
       demora = millis();
       if (patrulha)
       {
          if (digitalRead(led_verde) == HIGH) {
             digitalWrite(buzzer, HIGH);
             digitalWrite(led_verde, LOW);
             digitalWrite(led_amarelo, HIGH);
          } else if (digitalRead(led_amarelo) == HIGH) {
             digitalWrite(buzzer, HIGH);
             digitalWrite(led_amarelo, LOW);
             digitalWrite(led_vermelho, HIGH);
          } else if (digitalRead(led_vermelho) == HIGH) {
             digitalWrite(buzzer, HIGH);
             digitalWrite(led_vermelho, LOW);
             digitalWrite(led_verde, HIGH);
          } // fim do if digitalRead
      } // fim do if patrulha
    } // fim do if millis
} // fim do loop

/**********************************************************
           F   U   N   C   T   I   O   N   S
**********************************************************/
//
// Funcoes para motores adaptadas do tutorial
// https://robojax.com/learn/arduino/?vid=robojax_L9110_DC_motor
//
void motorA(char d)
{
    if (d =='R') {
       digitalWrite(motor_a1a,  LOW);
       digitalWrite(motor_a1b,  HIGH);
    } else if (d =='L') {
       digitalWrite(motor_a1a,  HIGH);
       digitalWrite(motor_a1b,  LOW);
    } else {
       digitalWrite(motor_a1a,  LOW);
       digitalWrite(motor_a1b,  LOW);
    } // fim do if
} // motorA end

void motorB(char d)
{
    if (d =='R') {
       digitalWrite(motor_b1a,  LOW);
       digitalWrite(motor_b1b,  HIGH);
    } else if (d =='L') {
       digitalWrite(motor_b1a,  HIGH);
       digitalWrite(motor_b1b,  LOW);
    } else {
       digitalWrite(motor_b1a,  LOW);
       digitalWrite(motor_b1b,  LOW);
    } // fim do if
} // motorB end

void ver_estado()
{
  motor_ligado = false;
  if (digitalRead(motor_a1a) == HIGH) motor_ligado = true;
  if (digitalRead(motor_a1b) == HIGH) motor_ligado = true;
  if (digitalRead(motor_b1a) == HIGH) motor_ligado = true;
  if (digitalRead(motor_b1b) == HIGH) motor_ligado = true;
}
