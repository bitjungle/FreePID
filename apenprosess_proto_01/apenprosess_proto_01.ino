/*************************************************************************
 * Implementering av PID-regulator for ÅpenProsess.no - Prototype #01
 * Copyright (C) 2014 BITJUNGLE Rune Mathisen
 * Denne koden er lisensiert under en GPLv3-lisens. 
 * Se http://www.gnu.org/licenses/gpl-3.0.html 
 * 
 * For mer informasjon om hva denne koden gjør,
 * se http://xn--penprosess-05a.no/category/simulator/prototype01/
 * 
 * Koden er basert på Arduino PID Example Lab av Bret Comnes & A. La Rosa 
 * Se http://www.pdx.edu/nanogroup/sites/www.pdx.edu.nanogroup/files/2013_Arduino%20PID%20Lab_0.pdf
 * 
 * PID-bilbioteksfunksjonene er laget av Brett Beauregard
 * Se https://github.com/br3ttb/Arduino-PID-Library/
 * 
 *************************************************************************/

#include <PID_v1.h>
#include <LiquidCrystal.h>

// Globale konstanter
const char ID[8] = "AIC-001";// Navnet på kontroll-loopen vår
const int PHOTO_RES = A0;    // Inngangen som fotoresistoren er koblet til
const int MODE_BTN = 16;     // Knapp for å bytte mellom AUTO/MAN (16 = A2)
const int SP_POT = A1;       // Inngangen som potensiometeret er koblet til
const int CONTROL_LED = 9;   // LED-en som er utgangen i det regulerte systemet
const int STATUS_LED = 13;   // LED som brukes til å indikere systemstatus
const int SAMPLE_RATE = 1;   // Setter hvor ofte PID-loopen skal kjøre (millisekunder)
const int COM_DELAY = 1000;  // Hvor ofte vi sjekker/sender serielldata (millisekunder) 
const int DATA_DEC = 0;      // Hvor mange desimaler vi skal bruke i data som sendes ut

// Tuning-parametre for PID-regulatoren
float Kp=0.8;   // Startverdi for proporsjonal-forsterkning
float Ki=100;   // Startverdi for integral-forsterkning
float Kd=0.001; // Startverdi for differensial-forsterkning

// Globale variabler for SP, PV og OP
float lightLevel;                  // Variabel som lagrer den målte lysverdien
float lightLevelFilterParam = 0.5; // Parameter som brukes til filtrering av måling
double Setpoint, Input, Output;    // Variabler for intern lagring av data

// Setter opp PID-regulatoren
// Input er er-verdien (PV)
// Output er pådragssignalet (OP)
// Setpoint er skal-verdien (SP)
PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);

// Variabler for å holde rede på tiden
unsigned long nowTime = 0;     // Lagerplass for tiden akkurat nå
unsigned long lastMessage = 0; // Tidspunkt for siste utskrift av data

/* 
  I denne applikasjonen bruker vi et 16x2 LCD-display GDM1602K.
  Pinnene settes opp slik:
  1 til GND
  2 til 5V
  3 til til GND men med motstand som setter kontrast på skjerm
  4 til Arduino digital pinne 12
  5 til GND
  6 til Arduino digital pinne 11
  7 (ikke i bruk)
  8 (ikke i bruk)
  9 (ikke i bruk)
  10 (ikke i bruk)
  11 til Arduino digital pinne 5
  12 til Arduino digital pinne 4
  13 til Arduino digital pinne 3
  14 til Arduino digital pinne 2
  15 til 5V
  16 til GND
*/
LiquidCrystal lcd(12,11,5,4,3,2);

//========================================================================

void setup() {
  lightLevel = analogRead(PHOTO_RES);       // Les inn lysnivå
  Input = map(lightLevel, 0, 1024, 0, 255); // Skaler den innleste verdien
  Setpoint = map(analogRead(SP_POT), 0, 1024, 0, 255); // Hent SP fra pot
  myPID.SetMode(AUTOMATIC);         // Setter regulatoren i standard modus
  myPID.SetSampleTime(SAMPLE_RATE); // Setter samplingrate
  
  Serial.begin(9600);// Starter seriellkommunikasjon
  
  lcd.begin(16, 2);// Starter LCD
  lcd.clear();     // Sletter skjerminnhold  
  // Skriver inn tekst på LCD som skal stå fast hele tiden.
  lcd.setCursor(0,0);
  lcd.print("SP: ");
  lcd.setCursor(0,1);
  lcd.print("PV: ");
  lcd.setCursor(9,0);
  lcd.print("OP: ");
  lcd.setCursor(9,1);
  lcd.print("MODE: A");// Standard modus er AUTO
  
  pinMode(STATUS_LED, OUTPUT);  
  pinMode(MODE_BTN, INPUT);

  lastMessage = millis(); // Tidsstempel
}

//========================================================================

void loop(){
  digitalWrite(STATUS_LED, LOW); // Slukker status-LED
  
  Setpoint = map(analogRead(SP_POT), 0, 1024, 0, 255); // Leser SP fra pot
  //lightLevel = analogRead(PHOTO_RES);       // Leser PV fra målesensor
  lightLevel = digFilter(lightLevel, analogRead(PHOTO_RES), lightLevelFilterParam);
  Input = map(lightLevel, 0, 900, 0, 255);  // Skalerer verdien fra målesensoren
  myPID.Compute();                          // Kjører PID-loopen
  if (myPID.GetMode() == MANUAL) {  // Sjekker om regulator står i manuell
    // Dette er en litt sær løsning hvor jeg gjenbruker pot-en til SP for å stille
    // regulatorutgangen OP direkte når den står i MAN. Løsninga gjør at det ikke 
    // blir bumpless transfer, og denne koden må skrives om når vi skal lage en 
    // skikkelig reguleringssløyfe senere.
    Output = Setpoint;              // Skriver verdien fra pot-en direkte til utgangen
    Setpoint = Input;               // SP følger PV når regulator er i manuell
  }
  analogWrite(CONTROL_LED, Output); // Skriver utgangen fra PID-regulatoren til LED-en
  
  if (digitalRead(MODE_BTN) == LOW) {   // Noen har trykket på knappen for modusskifte 
    // OBS: dette gir ikke bumpless transfer, se kommentar ovenfor.
    if (myPID.GetMode() == AUTOMATIC) { // Hvis nåværende modus er AUTO, bytt til MAN
      myPID.SetMode(MANUAL);
      lcd.setCursor(15,1);
      lcd.print("M");
    } else {                            // Ellers: bytt til AUTO
      myPID.SetMode(AUTOMATIC);
      lcd.setCursor(15,1);
      lcd.print("A");
    }
    delay(500); // Vent litt, så operatøren rekker å slippe fingeren fra knappen
  }
  
  nowTime = millis(); // Sjekker tiden
  if (nowTime - lastMessage > COM_DELAY) { 
    // Om det har gått lang nok tid, gjøres følgende:
    // -> Skriver data til seriellporten
    // -> Leser data fra seriellporten (regulatorparametre)
    // -> Skriver data til LCD-displayet 
    digitalWrite(STATUS_LED, HIGH); // Tenn status-LED
    

    // Seriell-kommunikasjonen skal brukes i en Processing-sketch senere.
    Serial.print(ID);  // Sender navnet på kontroll-loopen først 
    Serial.print(','); // Separerer alle data som sendes med et komma
    Serial.print(Setpoint, DATA_DEC); // SP
    Serial.print(',');                // ,
    Serial.print(Input, DATA_DEC);    // PV
    Serial.print(',');                // , 
    Serial.print(Output, DATA_DEC);   // OP - avslutter med linjeskift
    Serial.print(',');                // , 
    // Avslutter med en kontrollsum og et linjeskift
    // Kode for kontrollsum er ikke ferdig (obs for avrundingsfeil)
    Serial.println((int)((Setpoint+Input)/(1+Output)));
    
    // Skriver oppdaterte verdier til LCD-display
    printToLCD(4,0,Setpoint);
    printToLCD(4,1,Input);
    printToLCD(13,0,Output);
    
    // Sjekk om det er innkommende data og prosesser dette
    if (Serial.available() > 0) {
      for (int x=0; x<4; x++) { 
        switch (x) {
          case 0:
            Kp = Serial.parseFloat(); 
            break;
          case 1:
            Ki = Serial.parseFloat(); 
            break;
          case 2:
            Kd = Serial.parseFloat(); 
            break;
          case 3:
            for (int y = Serial.available(); y == 0; y--) {
              Serial.read(); // Fjerner søppeldata som eventuelt er igjen 
            }
            break; 
        }
      }
      myPID.SetTunings(Kp, Ki, Kd); // Setter nye PID-parametre
    }
    lastMessage = nowTime; // Oppdater tidsstempel
  } 
}

/* Funsjon for å skrive verdier til LCD.
 * For tall som er mindre enn 100 legges det til en null foran tallet
 * For tall som er mindre enn 10 legges det til to nuller foran tallet
 * Gjør dette for å sikre at det ikke henger igjen noen tall på skjermen fra tidligere
*/
void printToLCD(int col, int row, double val) {
  lcd.setCursor(col, row);
  if (val < 10) { 
    lcd.print("00");
  } else if (val >= 10 && val < 100) {
    lcd.print("0");
  }
  lcd.print(val,0);
}

/*
* Førsteordens digitalt filter
*
*/
float digFilter(float lastVal, float nowVal, float filtVal) {
  // Om vi ikke har noen god måling fra før, returnerer vi nå-verdien
  if (isnan(lastVal)) return nowVal;
  // Om det er noe galt med nå-verdien, returnerer vi forrige verdi
  if (isnan(nowVal)) return lastVal;
  // Bruk filterparameter for å beregne den filtrerte verdien
  return filtVal * lastVal + (1 - filtVal) * nowVal;
}
