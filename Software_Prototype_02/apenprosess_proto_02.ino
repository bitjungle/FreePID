/*************************************************************************
 * Implementering av PID-regulator for ÅpenProsess.no - Prototype #02
 * Copyright (C) 2014 BITJUNGLE Rune Mathisen
 * Denne koden er lisensiert under en GPLv3-lisens. 
 * Se http://www.gnu.org/licenses/gpl-3.0.html 
 * 
 * For mer informasjon om hva denne koden gjør,
 * se http://xn--penprosess-05a.no/category/simulator/prototype02/
 * 
 * PID-bilbioteksfunksjonene er laget av Brett Beauregard
 * Se https://github.com/br3ttb/Arduino-PID-Library/
 * 
 *************************************************************************/

#include <PID_v1.h>
#include <LiquidCrystal.h>

// Navnet på regulator-loopen vår, må samsvare med P&ID.
const char ID[8] = "FIC-101";

// IO-pinnene som brukes på arduino mikrokontrolleren.
const int SENSOR_PIN = 2;    // Inngangen som målesensoren er koblet til.
const int OUTPUT_PIN = 3;    // Utgangen som pådragsorganet er koblet til.
// I tillegg kommer pinner som brukes av LCD keypad shield. Se lenger ned i koden.

// Noen tidskonstanter som brukes i regulatoren.
const int SAMPLE_RATE = 1;     // Setter hvor ofte PID-loopen skal kjøre (millisekunder).
const int COM_DELAY = 1000;    // Hvor ofte vi sjekker/sender serielldata (millisekunder). 
const int DATA_DELAY = 1000;   // Hvor ofte vi henter sensordata (millisekunder). 
const int REFRESH_RATE = 1000; // Hvor ofte vi oppdaterer operatørskjerm (millisekunder).

// Formattering av data.
const int DATA_DEC = 2;        // Hvor mange desimaler vi skal bruke i data som sendes ut.

// Tuning-parametre for PID-regulatoren, kan justeres mens regulatoren er i drift.
float Kp=0.8;   // Startverdi for proporsjonal-forsterkning.
float Ki=100;   // Startverdi for integral-forsterkning.
float Kd=0.001; // Startverdi for differensial-forsterkning.

// Globale variabler for SP, PV og OP
float sensorFilterParam = 0.50; // Parameter som brukes til filtrering av måling
double Setpoint, Input, Output; // Variabler for intern lagring av data

// Setter opp PID-regulatoren
// Input er er-verdien (PV)
// Output er pådragssignalet (OP)
// Setpoint er skal-verdien (SP)
PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);

// Variabler for å holde rede på tiden
unsigned long lastMessage = 0; // Tidspunkt for siste utskrift av data
unsigned long lastDataFetch = 0;

// I denne applikasjonen bruker vi et LCD1602 keypad shield til operatørskjerm.
// Sjekk at alle pinner er definert riktig. Disse er for http://dx.com/p/118059 
// DATA4_PIN = 4, DATA5_PIN = 5, DATA6_PIN = 6, DATA7_PIN = 7
// RESET_PIN = 8, ENABLE_PIN = 9, BACKLIGHT_PIN = 10, KEYPAD_PIN = A0
LiquidCrystal lcd(8, 9, 4, 5, 6, 7);

// Definerer operatørknappene, disse brukes for å finne ut hvilken knapp som trykkes inn.
const int BUTTON_NONE = 5;
const int BUTTON_SELECT = 4;
const int BUTTON_LEFT = 3;
const int BUTTON_DOWN = 2;
const int BUTTON_UP = 1;
const int BUTTON_RIGHT = 0;
int button = BUTTON_NONE; // Holder rede på hvilken knapp som er trykket inn, default er ingen knapp.
int fadeValue = 255;      // Variabel for instilling av lysstyrke på operatørskjerm.
int lastScreenUpdate = 0; // Holder rede på når displayet ble sist oppdatert.
int currentScreenMode = 0;// Hvilket skjermbilde er det som vises akkurat nå?
const int SCREENMODE_DEFAULT = 0; // Standardskjermen (SP, PV, OP, MODE).
const int SCREENMODE_CONFIG = 1;  // Konfigureringsskjermen (ikke implementert ennå).
const int SCREENMODE_ALARM = 2;   // Konfigureringsskjermen (ikke implementert ennå).

//============================== INTERRUPT-FUNKSJON =============================
volatile int sensorCounter;       // Vi teller pulser fra flow-sensoren.
void rpm () { sensorCounter++; }  // Funksjonen som trigges av et interrupt.

//============================== SETUP ==========================================
void setup() {
  pinMode(SENSOR_PIN, INPUT);      // Initialiserer sensorpinnen som input.
  attachInterrupt(0, rpm, RISING); // Kobler til en interrupt som trigges på et stigende signal.
  sensorCounter = 0;               // Setter telleverket til null.
  lcd.begin(16, 2);       // Starter LCD
  defaultDisplayLayout(); // Setter opp operatørskjerm med de faste tekstene
  myPID.SetMode(MANUAL);  // Setter regulatoren i standard modus (manuell)
  lcd.setCursor(15,1);
  lcd.print("M");         // Skriver regulatormodus til operatørskjerm
  myPID.SetSampleTime(SAMPLE_RATE); // Setter samplingrate
  Serial.begin(9600);     // Starter seriellkommunikasjon
  lastMessage = millis(); // Tidsstempel
}

//============================== LOOP ===========================================
void loop(){ 
  // Aktiverer interrupts
  sei();
   // Sjekker tiden akkurat nå.
  unsigned long nowTime = millis();
  // Sjekker hvor lenge det har gått siden siste innhenting av data.
  int timeElapsedSinceDataFetched = nowTime - lastDataFetch; 
  // Om forhåndsinnstilt tid har medgått, eller om millis() har hatt en overflow, henter vi nye data.
  if (timeElapsedSinceDataFetched > DATA_DELAY || timeElapsedSinceDataFetched < 0) { 
    cli(); // Deaktiverer interrupts for å gjøre opptelling og beregning
    // Beregningen nedenfor må vurderes. Det kan ha gått mer enn ett sekund siden forrige beregning
    // selv om DATA_DELAY er satt til 1000. Noen kan ha trykket på en knapp eller gjort noe annet
    // som introduserer en forsinkelse. Mulig at datafiltreringen glatter ut disse feilene.
    // Mengdeberegning for sensoren FS03 - http://dx.com/p/217267 
    // Pulsfrekvens (Hz) = 7,5 * Q (liter/min)
    // Q = (pulsfrekvens x 60) / 7,5
    float rawData = sensorCounter * 60 / 7.5;
    Input = digFilter(Input, rawData, sensorFilterParam);
    sensorCounter = 0;	// Nullstiller telleverket.
    lastDataFetch = millis();
  }

  // Sjekker om regulator står i manuell. Om den gjør det, skal settpunkt følge input-signalet
  int currentControllerMode = myPID.GetMode();
  if (currentControllerMode == MANUAL) Setpoint = Input;

  // Kjører PID-loopen  
  myPID.Compute();
  // Skriver utgangen fra regulatoren til output-pinnen
  analogWrite(OUTPUT_PIN, Output);
  
  button = readButton();
  switch (button) {
    case 0://RIGHT
      fadeDisplay(5); // Skal skrives om for å bytte mellom ulike skjermbilder
      break; 
    case 1://UP - Øker settpunktet (eller utgangssignalet når regulator står i manuell)
      if (currentControllerMode == AUTOMATIC) Setpoint++;
      else Output++;// Øke med en enhet
      delay(200);// Vent litt, så operatøren rekker å slippe fingeren fra knappen
      break;
    case 2://DOWN - Minker settpunktet (eller utgangssignalet når regulator står i manuell)
      if (currentControllerMode == AUTOMATIC) Setpoint--;
      else Output--;// Minke med en enhet
      delay(200);// Vent litt, så operatøren rekker å slippe fingeren fra knappen
      break;
    case 3://LEFT
      fadeDisplay(-5); // Skal skrives om for å bytte mellom ulike skjermbilder
      break;
    case 4://SELECT
      if (currentControllerMode == AUTOMATIC) {
        // Hvis nåværende modus er AUTO, bytt til MAN
        myPID.SetMode(MANUAL);
      } else {
        // Ellers: bytt til AUTO
        myPID.SetMode(AUTOMATIC);
      }
      delay(200); // Vent litt, så operatøren rekker å slippe fingeren fra knappen
      break;
    case 5: // Ingen knappetrykk er registrert, vis den valgte skjermen.
      if (currentScreenMode != SCREENMODE_DEFAULT 
          && nowTime - lastScreenUpdate > REFRESH_RATE) {
        defaultDisplayLayout();
        lastScreenUpdate = millis();
      }
      break;
  }
  
  if (nowTime - lastMessage > COM_DELAY) { 
    // Om det har gått lang nok tid, gjøres følgende:
    // -> Henter data fra inputsensor
    // -> Skriver data til seriellporten
    // -> Leser data fra seriellporten (regulatorparametre)
    // -> Skriver data til LCD-displayet     

    writeDataToSerial(Setpoint, Input, Output);    // Skriver nå-data til seriellport
    if (currentScreenMode == SCREENMODE_DEFAULT) { // Om skjermen står i standardmodus...
      writeDataToLCD(Setpoint, Input, Output, currentControllerMode);// ...skriver vi nå-data
    }
    
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


/*
* Førsteordens digitalt filter
*
*/
float digFilter(float lastVal, float nowVal, float filtVal) {
  // Om vi ikke har noen god måling fra før, returnerer vi nå-verdien
  if (isnan(lastVal)) return nowVal;
  // Om det er noe galt med nå-verdien, returnerer vi forrige verdi
  else if (isnan(nowVal)) return lastVal;
  // Bruk filterparameter for å beregne den filtrerte verdien
  else return filtVal * lastVal + (1 - filtVal) * nowVal;
}


/*
* Avlesing av knapper på LCD-display
* På LCD Keypad-shieldet som brukes her er knappene koblet med en 
* voltage divider på analog-pinne 0.
*/
int readButton() {
  int buttonVoltage = analogRead(A0);
  int buttonPressed;
  if (buttonVoltage > 800) buttonPressed = BUTTON_NONE;
  else if (buttonVoltage > 500) buttonPressed = BUTTON_SELECT;
  else if (buttonVoltage > 400) buttonPressed = BUTTON_LEFT;
  else if (buttonVoltage > 250) buttonPressed = BUTTON_DOWN;
  else if (buttonVoltage > 90) buttonPressed = BUTTON_UP;
  else buttonPressed = BUTTON_RIGHT;
  return buttonPressed;
}


/*
* Still lysnivå på LCD-display
* Bruk +5 eller -5 på fadeIncrement
* På LCD-panelet som brukes her styres baklys over IO-pinne 10.
*/
void fadeDisplay(int fadeIncrement) {
  fadeValue = fadeValue + fadeIncrement;
  if (fadeValue < 5) fadeValue = 0;
  if (fadeValue > 254) fadeValue = 255;
  analogWrite(10, fadeValue);
  delay (100);
}


/*
* Skriv standard layout til operatørskjerm
*
*/
void defaultDisplayLayout() {
  lcd.clear();     // Sletter skjerminnhold  
  lcd.setCursor(0,0);
  lcd.print("SP: ");
  lcd.setCursor(0,1);
  lcd.print("PV: ");
  lcd.setCursor(9,0);
  lcd.print("OP: ");
  lcd.setCursor(9,1);
  lcd.print("MODE: ");
  currentScreenMode = SCREENMODE_DEFAULT;
}


/*
* Skriver nå-verdier for SP, PV og OP til operatørskjerm
*
*/
void writeDataToLCD(double sp, double pv, double op, int mode) {
  numberToLCD(4, 0, sp);
  numberToLCD(4, 1, pv);
  numberToLCD(12, 0, map(op, 0, 255, 0, 100)); // Skalerer visningen av utgangssignalet  
  lcd.setCursor(15,1);
  if (mode == AUTOMATIC) lcd.print("A");
  else lcd.print("M");
}


/* 
 * Funsjon for å skrive verdier til LCD.
 * For tall som er mindre enn 10 legges det til tre blank foran tallet
 * For tall som er mindre enn 100 legges det til to blank foran tallet
 * For tall som er mindre enn 1000 legges det til en blank foran tallet
 * Gjør dette for å sikre at det ikke henger igjen noen tall på skjermen fra tidligere
*/
void numberToLCD(int col, int row, double val) {
  lcd.setCursor(col, row);
  if (val < 10) { 
    lcd.print("   ");
  } else if (val >= 10 && val < 100) {
    lcd.print("  ");
  } else if (val >= 100 && val < 1000) {
    lcd.print(" ");
  } else {
    //ikke gjør noe
  }
  lcd.print(val,0);
}


/*
* Seriell-kommunikasjonen skal brukes i en Processing-sketch senere.
*
*/
void writeDataToSerial(double sp, double pv, double op) {
  Serial.print(ID);  // Sender navnet på kontroll-loopen først 
  Serial.print(","); // Separerer alle data som sendes med et komma
  Serial.print(sp, DATA_DEC); // Setpoint
  Serial.print(",");          // ,
  Serial.print(pv, DATA_DEC); // Input
  Serial.print(",");          // , 
  Serial.print(op, DATA_DEC); // Output
  Serial.print(",");          // , 
  // Avslutter med en kontrollsum og et linjeskift
  Serial.println(Setpoint + Input + Output);
}
