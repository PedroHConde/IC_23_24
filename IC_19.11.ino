///////////////////////////////////////////////////////////////////
//  Programa: Controle de temperatura em MA e MF                  //
//  Autor: Daniel de A. Fernandes                                 //
//  Versão: v2.00 - 20/08/2023 - Inclusão da função rampa (Tref)  //
//  Projeto: SCT didático, portátil e de baixo custo              //
//  Hardware alvo: Arduino NANO (ATmega328 (Old Bootloader))      //
////////////////////////////////////////////////////////////////////



/* ----------------------------------------------------------------------------------------------------
  ------------------------------------------   BIBLIOTECAS   ------------------------------------------
  -------------------------------------------------------------------------------------------------- */
#include <DallasTemperature.h>  // <https://github.com/milesburton/Arduino-Temperature-Control-Library>
#include <OneWire.h>
#include <string.h>


/* ----------------------------------------------------------------------------------------------------
  -------------------------------------------   ENTRADAS   --------------------------------------------
  -------------------------------------------------------------------------------------------------- */
// A0 : Tensão proporcional à tensão de alimentação do circuito de potência (V+)
// A1 : Tensão proporcional à tensão de referência (Vref)
// A2 : Tensão de saída da rede resistiva que contém o NTC (Vdiv)



/* ----------------------------------------------------------------------------------------------------
  --------------------------------------------   SAÍDAS   ---------------------------------------------
  -------------------------------------------------------------------------------------------------- */
// D2  : Comunicação com o sensor de temperatura DS18B20
// D11 : Função PWM (frequência ajustada para 980Hz)
// D13 : LED nativo do Arduino



/* ----------------------------------------------------------------------------------------------------
  ------------------------------------------   DEFINIÇÕES   -------------------------------------------
  -------------------------------------------------------------------------------------------------- */

#define TmaxDegrau    98.0  // [ºC]
#define TminDegrau    48.0  // [ºC]



#define dTdtRampa     0.25  // [ºC/s] (mesma taxa de / e \)
#define tempoPreRampa 180   // [s] (degrau inicial)
#define TmaxRampa     98.0  // [ºC]
#define TminRampa     48.0  // [ºC]



#define tempoPreSen   180   // [s] (degrau inicial)
#define TmaxSen       98.0  // [ºC]
#define TmediaSen     70.0  // [ºC]
#define Tsen          600.0 // [s] (período da senoide)



#define PWM           11
#define sensorDS18B20 2
#define Vdiv          A2
#define Vmais         A0
#define Vref          A1



OneWire oneWire(sensorDS18B20);
DallasTemperature DS18B20(&oneWire);
DeviceAddress enderecoDS18B20;



/* ----------------------------------------------------------------------------------------------------
  ------------------------------------   CONSTANTES & VARIÁVEIS   -------------------------------------
  -------------------------------------------------------------------------------------------------- */

int duracaoDegrau = 30;  // [s]
int numDegraus  =  3 ;    // (quantidade de degraus)



bool aquecendo = true;
bool atualizacao = false;
bool comandoCompleto = false;
bool comandoCompletoConfig = false;
bool degrau = false;
bool ligaDesliga = false;
bool malhaFechada = true;
bool mostraUnicaVez = false;
bool mudadoUnicaVez = false;
bool primeiroCaractereComando = false;
bool rampa = false;
bool recebeDC = false;
bool recebeTempRef = false;
bool recebeConfigDeg = false;
bool saturacao = false;
bool senoide = false;
bool subindo = true;


bool entrouModoConfigDagora = false;


byte DC256 = 0;
byte DC256s[] = {0, 0, 0, 0, 0, 0, 0, 0};

const bool dadosPlotterOuMonitor = true;  // TRUE: Dados para "Plotter serial"; FALSE: Dados para "Monitor serial"

const float betaNTCinv = 1.0 / 3900.0;    // Inverso multiplicativo do coeficiente "beta" do NTC ~ 3900K
const float h = 0.5;                      // Período de amostragem do sistema [s]
const float Kd = 50.0;                    // Ganho derivativo do PID      |>  COM saturação do atuador: Kp = 5;     Ki = 0.05;   Kd = 50  <| Valores
const float Ki = 0.13;                    // Ganho integral do PID        |>  COM saturação do atuador: Kp = 2.25;  Ki = 0.015;  Kd = 25  <| teóricos
const float Kp = 10.0;                    // Ganho proporcional do PID    |>  SEM saturação do atuador: Kp = 1.8;   Ki = 0.01;   Kd = 20  <| (MATLAB)
const float K0C = 273.15;                 // Temperatura absoluta (K) correspondente a 0ºC
const float K25Cinv = 1.0 / (K0C + 25.0); // Inverso multiplicativo da temperatura absoluta (K) correspondente a 25ºC
const float Rs = 2179.0;                  // Resistência fixa "de cima" do divisor de tensão (valor nominal: 2k2; tolerância: 5%)
const float Rp = 1179.0;                  // Resistência fixa associada em paralelo com o NTC (valor nominal: 1k2; tolerância: 5%)
const float R25C = 12400.0;               // Resistência do NTC a 25ºC (valor nominal é 10k)
const float Vcc = 3.297;                  // Tensão (medida) da fonte de alimentação de 3V3 do Arduino
const float VrefInt1V1 = 1.096;           // Tensão de referência interna (1V1) do CAD do Arduino (medida diretamente no pino 18)

float amplitudeDegrau = (float) (TmaxDegrau - TminDegrau) / (numDegraus - 1);
float correnteShunt = 0.0;
float e = 0.0;
float eD = 0.0;
float eI = 0.0;
float ePrev = 0.0;
float ePrev2 = 0.0;
float ePrev3 = 0.0;
float mediasTempNTC[] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
float mediasTensaoAlimentacao[] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
float mediasTensaoReferencia[] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
float omegaSen = (float) 6.28318530717958647693 / Tsen;
float potenciaColetor = 0.0;
float tempDS18B20 = 0.0;
float tempNTC = 0.0;
float tempRef = 0.0;
float tensaoAlimentacao = 0.0;
float tensaoDivisor = 0.0;
float tensaoReferencia = 0.0;

String comando;
String comand = "";

unsigned int contadorTempo = 0;

unsigned long tempoAnterior = 0;
unsigned long tempoAtual = 0;
unsigned long tempoInicialLacoPrincipal = 0;
unsigned long tempoLacoPrincipal = 0;
unsigned long tempoZero = 0;



// ====================================================================================================
// ==========================================   PREPARAÇÃO   ==========================================
// ====================================================================================================
void setup() {

  /* Inicializa a comunicação serial e a espera responder */
  Serial.begin(500000);
  while (!Serial) {
    ;
  }



  /* Desabilita a função dos pinos 2 e 3 de atender interrupções externas */
  detachInterrupt(digitalPinToInterrupt(2));
  detachInterrupt(digitalPinToInterrupt(3));



  /* Inicializa a comunicação e configura o sensor de temperatura DS18B20 */
  DS18B20.begin();
  oneWire.reset_search();
  oneWire.search(enderecoDS18B20);
  DS18B20.setResolution(enderecoDS18B20, 11); // Altas resoluções (11 e 12 bits) elevam muito o tempo de processamento (biblioteca utilizada?)



  /* Configura os pinos utilizados do microcontrolador */
  TCCR2B = TCCR2B & B11111000 | B00000011; // Ajusta freq. do PWM D11 para ~980Hz

  analogReference(INTERNAL);  // NANO (ATmegaXX8): "DEFAULT", "INTERNAL" ou "EXTERNAL"

  pinMode(PWM, OUTPUT);
  pinMode(LED_BUILTIN, OUTPUT);

  analogWrite(PWM, 0);
}





// ====================================================================================================
// ========================================   LAÇO PRINCIPAL   ========================================
// ====================================================================================================
void loop() {

  // -------------------------------------   Temporização geral   -------------------------------------
  tempoAtual = millis();

  if (tempoAtual - tempoAnterior >= (int) (1000 * h)) {
    tempoInicialLacoPrincipal = millis();
    tempoAnterior = tempoAtual;
    atualizacao = true;

//    digitalWrite(LED_BUILTIN, HIGH);  // Acende LED_BUILTIN (piloto; indicador de atividade do Arduino)
  }



  // ----------------------------   Lê porta serial para receber comandos   ---------------------------
  comandoCompleto = false;
  primeiroCaractereComando = false;

  while (Serial.available()) {
    if (!primeiroCaractereComando) {
      comando = "";
    }
    else {
      char caractere = (char) Serial.read();

      if (caractere != '\n') {
        comando += caractere;
      }
      else {
        comandoCompleto = true;
      }

      if (comandoCompleto) {
        if (!malhaFechada && comando == "DC") { // Duty Cycle [%]. Válido somente em MA
          recebeDC = !recebeDC;
        }
        else if (!degrau && comando == "degrau") { // Degraus ascendentes e descendentes
          modoNeutro();
          contadorTempo = 0;
          degrau = true;
          tempRef = TminDegrau;
        }
        else if (malhaFechada && comando == "LD") { // Controle tipo Liga/Desliga ao invés do PID. Válido somente em MF
          ligaDesliga = !ligaDesliga;
        }
        else if (malhaFechada && comando == "MA") { // Operação em MA
          modoNeutro();
          malhaFechada = false;
        }
        else if (!rampa && comando == "rampa") { // Degrau inicial seguido de rampas ascendentes e descendentes
          modoNeutro();
          mudadoUnicaVez = false;
          rampa = true;
          tempoZero = millis();
          tempRef = TminRampa;
        }
        else if (comando == "ref") { // Modo manual de referência de temperatura
          bool varAux1 = recebeTempRef;
          float varAux2 = tempRef;
          modoNeutro();
          recebeTempRef = !varAux1;
          tempRef = varAux2;
        }
        else if (!senoide && comando == "sen") { // Degrau inicial seguido de uma senoide de valor médio idêntico ao do degrau
          modoNeutro();
          mudadoUnicaVez = false;
          senoide = true;
          tempoZero = millis();
          tempRef = TmediaSen;
        }
        else if (recebeConfigDeg){ //&& !entrouModoConfigDagora) {
          digitalWrite(LED_BUILTIN, LOW);
          comand=comando;
          duracaoDegrau = comando.substring(0, 4).toInt();
          numDegraus =comando.substring(5, 6).toInt();
          recebeConfigDeg = false;
        }
        else if (comando == "configD") {
          //if (!recebeConfigDeg) {
            recebeConfigDeg = true;
            comando="";
           // entrouModoConfigDagora = true;
            digitalWrite(LED_BUILTIN, HIGH);
            delay(500);
           

         // }
//          else {
//            entrouModoConfigDagora = false;
//          }
         
        }
       
        else if (comando == "zero") { // Modo neutro. Desliga tudo
          modoNeutro();
        }
        else if (((malhaFechada && recebeTempRef) || (!malhaFechada && recebeDC)) && stringInteiroComSinal(comando)) {
          int valor = 0;

          if (comando[0] == '-') {
            goto ignora;
          }
          else {
            if (comando[0] == '+') {
              valor = comando.substring(1).toInt();
            }
            else {
              valor = comando.toInt();
            }
          }

          if (valor >= 0 && valor <= 100) {
            if (malhaFechada) {
              tempRef = 1.0 * valor;
            }
            else {
              DC256 = (byte) (2.550 * valor);
              analogWrite(PWM, DC256);
            }
          }
          else {
ignora:     ;
          }
        }
        else { // Comando completo, porém inválido!
          ;
        }
      }
    }

    primeiroCaractereComando = true;
  }



  // -----------------------   Leitura de tensões e computação dos resultados   -----------------------
  // ---------   (filtradas através de médias móveis das médias de n leituras consecutivas)   ---------
  if (atualizacao) {
    byte numeroConversoes = 11;
    byte numeroMedias = (byte) sizeof(mediasTensaoAlimentacao) / 4; // Divisão pelo número de bytes ocupados pela variável do tipo float
    unsigned int acumuladorVmais = 0;
    unsigned int acumuladorVref = 0;

    for (byte i = numeroMedias - 1; i > 0; i--) {
      mediasTensaoAlimentacao[i] = mediasTensaoAlimentacao[i - 1];
      mediasTensaoReferencia[i] = mediasTensaoReferencia[i - 1];
    }

    for (byte i = 0; i < numeroConversoes; i++) {
      acumuladorVmais += analogRead(Vmais);
      acumuladorVref += analogRead(Vref);
      delay(1);
    }

    acumuladorVmais /= (float) numeroConversoes;
    mediasTensaoAlimentacao[0] = 12.719699411 * acumuladorVmais; // Calibração: 12.719699411
    tensaoAlimentacao = 0.0;

    acumuladorVref /= (float) numeroConversoes;
    mediasTensaoReferencia[0] = 12.728762824 * acumuladorVref; // Calibração: 12.728762824
    tensaoReferencia = 0.0;

    for (byte i = numeroMedias; i > 0; i--) {
      tensaoAlimentacao += mediasTensaoAlimentacao[i - 1];
      tensaoReferencia += mediasTensaoReferencia[i - 1];
    }

    tensaoAlimentacao /= numeroMedias;
    tensaoReferencia /= numeroMedias;

    float tensaoShunt = tensaoAlimentacao - tensaoReferencia;

    if (tensaoShunt >= 0.0) {
      correnteShunt = 0.998552737 * tensaoShunt / 4.7; // Calibração: 0.998552737; valor nominal do resistor "shunt" (4R7)
      potenciaColetor = tensaoShunt * correnteShunt;
    }
    else {
      correnteShunt = 0.0;
      potenciaColetor = 0.0;
    }

    if (tensaoAlimentacao < 9000.0 || tensaoAlimentacao > 11000.0 || correnteShunt > 750.0) { // Proteções contra sub e sobretensão (9V <= V+ <= 10,5V) e sobrecorrente (IshuntMax <= 750mA)
      modoNeutro();
    }
  }



  // ------------------   Leituras de temperatura através do NTC e do sensor DS18B20   -----------------
  // ----------   (a temperatura dos transistores que é indicada é filtrada através de uma   ----------
  // ----------   média móvel das médias de n leituras consecutivas da tensão sobre o NTC)   ----------
  if (atualizacao) {
    DS18B20.requestTemperatures(); // Temperatura ambiente medida pelo sensor DS18B20
    tempDS18B20 = DS18B20.getTempC(enderecoDS18B20);

    byte numeroConversoes = 11;
    byte numeroMedias = (byte) sizeof(mediasTempNTC) / 4; // Divisão pelo número de bytes ocupados pela variável do tipo float
    unsigned int acumuladorVdiv = 0;

    for (byte i = numeroMedias - 1; i > 0; i--) {
      mediasTempNTC[i] = mediasTempNTC[i - 1];
    }

    for (byte i = 0; i < numeroConversoes; i++) {
      acumuladorVdiv += analogRead(Vdiv);
      delay(1);
    }

    tensaoDivisor = (VrefInt1V1 / 1024.0) * (acumuladorVdiv / numeroConversoes);

    float Req = (Rs * tensaoDivisor) / (Vcc - tensaoDivisor);
    float Rntc = (Rp * Req) / (Rp - Req);

    mediasTempNTC[0] = 1.0 / (K25Cinv + betaNTCinv * log(Rntc / R25C)) - K0C;
    tempNTC = 0.0;

    for (byte i = numeroMedias; i > 0; i--) {
      tempNTC += mediasTempNTC[i - 1];
    }

    tempNTC = tempNTC / numeroMedias;

    if (tempNTC > 100.0) { // Proteção contra superaquecimento
      modoNeutro();
    }
  }



  // -----------------   Geração do sinal de referência de temperatura (somente MF)   -----------------
  if (atualizacao && malhaFechada) {
    if (degrau) { // Degraus ascendentes e descendentes
      if (contadorTempo == (unsigned int) (duracaoDegrau / h) - 1) { // Contagem em segundos
        contadorTempo = 0;

        if (subindo) {
          tempRef += amplitudeDegrau;
        }
        else {
          tempRef -= amplitudeDegrau;
        }

        if (tempRef <= 1.001 * TminDegrau) {
          subindo = true;
          tempRef = TminDegrau;
        }

        if (tempRef >= 0.999 * TmaxDegrau) {
          subindo = false;
          tempRef = TmaxDegrau;
        }
      }
      else {
        contadorTempo++;
      }
    }

    if (rampa) { // Degrau inicial seguido de rampas ascendentes e descendentes
      if (!mudadoUnicaVez) {
        tempRef = TminRampa;

        if (millis() - tempoZero >= (unsigned long) (1000.0 * tempoPreRampa)) { // Degrau inicial de duração igual a "tempoPreRampa" [s]
          mudadoUnicaVez = true;
          tempoZero = millis();
        }
      }
      else {
        if (subindo) {
          tempRef = TminRampa + dTdtRampa * h * 0.001 * (millis() - tempoZero);
        }
        else {
          tempRef = TmaxRampa - dTdtRampa * h * 0.001 * (millis() - tempoZero);
        }

        if (tempRef <= TminRampa) {
          subindo = true;
          tempoZero = millis();
          tempRef = TminRampa;
        }

        if (tempRef >= TmaxRampa) {
          subindo = false;
          tempoZero = millis();
          tempRef = TmaxRampa;
        }
      }
    }

    if (senoide) { // Degrau inicial seguido de uma senoide de valor médio idêntico ao do degrau
      if (!mudadoUnicaVez) {
        tempRef = TmediaSen;

        if (millis() - tempoZero >= (unsigned long) (1000.0 * tempoPreSen)) { // Degrau inicial de duração igual a "tempoPreSen" [s]
          mudadoUnicaVez = true;
          tempoZero = millis();
        }
      }
      else {
        tempRef = TmediaSen + (TmaxSen - TmediaSen) * sin(omegaSen * h * 0.001 * (millis() - tempoZero));
      }
    }
  }



  // --------------------------   Controle PID paralelo real (somente MF)   ---------------------------
  if (atualizacao && !ligaDesliga && malhaFechada) {
    if (tempRef > tempDS18B20) {
      ePrev3 = ePrev2;
      ePrev2 = ePrev;
      ePrev = e;
      e = tempRef - tempNTC;

      if (!saturacao) {
        eI += (h / 2.0) * (e + ePrev);
      }

      eD = (11.0 * e - 18.0 * ePrev + 9.0 * ePrev2 - 2.0 * ePrev3) / (6.0 * h);

      int varAux1 = (int) (2.550 * Kp * e + 2.550 * Ki * eI + 2.550 * Kd * eD);

      if (varAux1 < 0) {
        varAux1 = 0;
        saturacao = true;
      }
      else if (varAux1 > 255) {
        varAux1 = 255;
        saturacao = true;
      }
      else {
        saturacao = false;
      }

      DC256 = (byte) varAux1;

      byte numeroControles = (byte) sizeof(DC256s);

      for (byte i = numeroControles - 1; i > 0; i--) {
        DC256s[i] = DC256s[i - 1];
      }

      DC256s[0] = DC256;

      unsigned int varAux2 = 0;

      for (byte i = numeroControles; i > 0; i--) {
        varAux2 += DC256s[i - 1];
      }

      varAux2 /= numeroControles;
      DC256 = (byte) varAux2;
      analogWrite(PWM, DC256);
    }
    else {
      if (malhaFechada) {
        DC256 = 0;
        analogWrite(PWM, DC256);
      }

      byte numeroControles = (byte) sizeof(DC256s);

      for (byte i = 0; i < numeroControles; i++) {
        DC256s[i] = 0;
      }

      e = 0.0;
      eD = 0.0;
      eI = 0.0;
      ePrev = 0.0;
      ePrev2 = 0.0;
      ePrev3 = 0.0;
      saturacao = false;
    }
  }



  // ------------------------   Controle Liga-Desliga (ON-OFF) (somente MF)   -------------------------
  if (atualizacao && ligaDesliga && malhaFechada) {
    if (aquecendo) {
      if (tempNTC >= tempRef) {
        aquecendo = false;
        DC256 = 0;
      }
      else {
        DC256 = 255;
      }
    }
    else {
      if (tempNTC < tempRef) {
        aquecendo = true;
        DC256 = 255;
      }
      else {
        DC256 = 0;
      }
    }

    analogWrite(PWM, DC256);
  }



  // ----------------------------   Exibição dos valores (porta serial)   -----------------------------
  if (atualizacao) {
    if (dadosPlotterOuMonitor) {
      if (!mostraUnicaVez) { // Apresenta dados através do plotter serial. ATENÇÃO: Pode ocupar muita memória!
        mostraUnicaVez = true;
        //Serial.println((String) F("Tref[ºC]") + "\t" + F("102%Tref[ºC]") + "\t" + F("98%Tref[ºC]") + "\t" + F("Tntc[ºC]") + "\t" + F("Tamb[ºC]") + "\t" + F("DC[%]"));

      }

      //Serial.println((String) tempRef + "\t" + 1.02 * tempRef + "\t" + 0.98 * tempRef + "\t" + tempNTC + "\t" + tempDS18B20 + "\t" + DC256 / 2.550);
      Serial.print(tempRef);
      Serial.print(',');
      Serial.print(tempNTC);
      Serial.print(',');
      Serial.print(comand);

      //Serial.print(tempDS18B20);
      Serial.print(',');
      Serial.print(comand.substring(0,4));

      //Serial.print(DC256 / 2.550);
      Serial.print('\n');
     
    }
    else { // Apresenta dados através do monitor serial. ATENÇÃO: Pode ocupar muita memória!
      tempoLacoPrincipal = millis() - tempoInicialLacoPrincipal;
      Serial.println((String) F("    DC = ") + mostraValorReformatado(DC256 / 2.550, 2) + F("%    V+ = ") + mostraValorReformatado(tensaoAlimentacao / 1000.0, 2) + F("V    Vref = ") + mostraValorReformatado(tensaoReferencia / 1000.0, 2) + F("V    Ishunt = ") + mostraValorReformatado(correnteShunt, 2) + F("mA    Tref = ") + mostraValorReformatado(tempRef, 2) + F("ºC    Tntc = ") + mostraValorReformatado(tempNTC, 2) + F("ºC    Tamb = ") + mostraValorReformatado(tempDS18B20, 2) + F("ºC    tProc = ") + tempoLacoPrincipal + F("ms"));
      // Serial.println((String) F("   DC = ") + DC256 / 2.550 + F("%    V+ = ") + tensaoAlimentacao / 1000.0 + F("V    Vref = ") + tensaoReferencia / 1000.0 + F("V    Ishunt = ") + correnteShunt + F("mA    Tref = ") + tempRef + F("ºC    Tntc = ") + tempNTC + F("ºC    Tamb = ") + tempDS18B20 + F("ºC    tProc = ") + tempoLacoPrincipal + F("ms"));
      // Serial.println((String) F("   DC = ") + DC256 / 2.550 + F("%    Tref = ") + tempRef + F("ºC    Tntc = ") + tempNTC + F("ºC    Tamb = ") + tempDS18B20 + F("ºC"));
    }

    Serial.flush();
    atualizacao = false;
  }



  // --------------------------------------------   FIM   ---------------------------------------------
  //digitalWrite(LED_BUILTIN, LOW);  // Apaga LED_BUILTIN (piloto; indicador de atividade do Arduino)
}





// ====================================================================================================
// ===========================================   FUNÇÕES   ============================================
// ====================================================================================================

/* Entra em modo manual, ou seja, modo neutro/proteção (desativação total) */
void modoNeutro() {
  DC256 = 0;
  analogWrite(PWM, DC256);
  aquecendo = true;
  degrau = false;
  ligaDesliga = false;
  malhaFechada = true;
  rampa = false;
  recebeDC = false;
  recebeTempRef = false;
  senoide = false;
  subindo = true;
  tempRef = 0.0;
  return;
}



/* Converte uma variável do tipo float numa string com N casas decimais separadas por vírgula */
String mostraValorReformatado(float valor, int N) {
  char texto[20];
  dtostrf(valor, -1, N, texto);
  char *pontoDecimal = strchr(texto, '.');
  *pontoDecimal = ',';
  return texto;
}


/* Determina se uma string representa um número inteiro */
bool stringInteiroComSinal(String &str) {
  byte j = str.length();

  if (j > 4 || ((j == 1 || (str[0] != '+' && str[0] != '-')) && (str[0] < '0' || str[0] > '9'))) {
    return false;
  }

  byte i = 1;

  while (i < j) {
    if (str[i] < '0' || str[i] > '9') {
      return false;
    }
    else {
      i++;
    }
  }

  return true;
}
