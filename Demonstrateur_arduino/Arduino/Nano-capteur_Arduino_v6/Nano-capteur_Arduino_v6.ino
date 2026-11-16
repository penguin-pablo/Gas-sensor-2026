/*
==============================================
============ Projet Nano-Capteur =============
==============================================
Authors: Yoann LAI KOUN SING, Artur NERSISYAN, Lucas BOURDARIE
Last update: 09/01/2026

Commentaire : 
  Ce programme mesure deux tensions : une au niveau du capteur de gaz et une au niveau du capteur de température.
  Il convertit ces tensions en valeur de résistance pour le capteur de gaz, et de températue pour le capteur de température.
  Il transmet via bluetooth ces informations à une application codée sur MIT App Inventor
  Il reçoit via bluetooth des consignes pour ajuster son gain.
*/


//====================================================
//==================== Libraries =====================
//====================================================

#include <SPI.h>             // Library for SPI to control Digital Potentiometer
#include <SoftwareSerial.h>  // Library for HC-05 bluetooth module
#include <stdlib.h>
#include <time.h>
#include <string.h>
#include <stddef.h>


//====================================================
//==================== Declaration ===================
//====================================================

// _____ Capteur _____
#define pinCapteur A0
int Rin_Capteur = 0;                //Résistance nominale du capteur caractérisé en sortie d'usine.
float Resistance_CapteurGaz = 0.0;  //Mesure de la résistance du capteur.

// _____ Temperature _____
#define pinTemp A1
float Temperature = 0.0;  // [K]
const int R5 = 220;       // [ohm]   --CONSTANTE À INITIALISER--
int Rtemp = 0.0;          // Résistance du capteur de température.  --Calculé--
const float Vcc_Temp = 5.0;
const float rho_300K = 27 * (0.00000001);                          // résistivité nominale à 300K.
const float R_300K = 100.0;                                              // ohm résistance nominale à 300K.  --CONSTANTE À INITIALISER--
const float alpha_T = (600.0 - 200.0) / ((62.0 - 15.0) * (0.00000001));  // [K.ohm-1.m-1] Calcul de la pente pour conversion rho->T entre 200K et 600K.
const float beta_T = 500.0 - (alpha_T * 50.0 * (0.00000001));            // [K] Calcul de l'ordonnée à l'origine pour conversion rho->T entre 200K et 600K.

// _____ Digital Potentiometer _____
#define pinPot_SCK 13
#define pinPot_SDI 11
const byte pinCS_P1 = 10;  // | pin de commande des potentiomètres digitaux P1, P2, P3.
const byte pinCS_P2 = 6;   // |
const byte pinCS_P3 = 5;   // |
const byte Tab_pinCS[3] = { pinCS_P1, pinCS_P2, pinCS_P3 };

const byte address_P1 = 0x11;  // | Adresse numérique par défaut du potentiomètre.
const byte address_P2 = 0x11;  // |
const byte address_P3 = 0x11;  // |

const float rAB_P1 = 48600.0;  // [K] Résistance maximale du potentimètre digital P1.
const float rAB_P2 = 46200.0;  // [K] Résistance maximale du potentimètre digital P2.
const float rAB_P3 = 47000.0;  // [K] Résistance maximale du potentimètre digital P3.

const float rWiper_P1 = 124.0;  // [K] Résistance de offset du potentimètre digital P1.
const float rWiper_P2 = 309.0;  // [K] Résistance de offset du potentimètre digital P2.
const float rWiper_P3 = 125.0;  // [K] Résistance de offset du potentimètre digital P3.

const int maxPositions = 255;

const byte addressShutdown_P1 = 0x21;
const byte addressShutdown_P2 = 0x21;
const byte addressShutdown_P3 = 0x21;

volatile float R_P1 = 0.0;  // [K] Résisitance effective du potentiomètre P1.  --Calculée--
volatile float R_P2 = 0.0;  // [K] Résisitance effective du potentiomètre P2.  --Calculée--
volatile float R_P3 = 0.0;  // [K] Résisitance effective du potentiomètre P3.  --Calculée--

// _____ Bluetooth _____
#define pinBT_TXD 3
#define pinBT_RXD 2
SoftwareSerial MyBT(pinBT_RXD, pinBT_TXD);
volatile float DataToSend = 0.0;
String DataReceived = "";  //Taille implicite de 32 octets suffisant pour recevoir le message bluetooth.

// _____ Other Parameters _____
#define baudrate 9600
const int DeltaTime = 200;
const int Res1 = 10000;  // [ohm] Résistance Res1 du circuit.
const int Res4 = 1000;   // [ohm] Résistance Res2 du circuit.
float Vcc = 20;
unsigned long currentTime;
unsigned long previousTime = 0;
#define TRUE 1
#define FALSE 0
struct Tab_Pot {
  float gain;
  int pos1;
  int pos2;
  int pos3;
};
struct Tab_Pot Pot_Mode_Init = { 1.0, 61, 231, 194 };
struct Tab_Pot Pot_Mode_A = { 29.999, 157, 0, 103 };   //  Gain: 30 -- À DEFINIR --
struct Tab_Pot Pot_Mode_B = { 59.999, 75, 0, 168 };    //  Gain: 60 -- À DEFINIR --
struct Tab_Pot Pot_Mode_C = { 89.999, 229, 0, 183 };   //  Gain: 90 -- À DEFINIR --
struct Tab_Pot Pot_Mode_D = { 119.999, 204, 0, 248 };  //  Gain: 120 -- À DEFINIR --
struct Tab_Pot Pot_Mode_E = { 127.367, 255, 0, 255 };  //  Gain: 127.367 MAX -- À DEFINIR --

bool TakeAndSendMeasure = FALSE;




//====================================================
//====================== Setup =======================
//====================================================
void setup() {
  Serial.begin(baudrate);
  Serial.println("===========================================================");
  Serial.println("----- Programme Nano-Capteur -----");
  // _____ Bluetooth _____
  MyBT.begin(9600);
  InitBluetooth();
  // _____ Digital Potentiometer _____
  Set_DigitalPotentiometer(address_P1, pinCS_P1, 0, rAB_P1, rWiper_P1);  // Initialisation R_P1
  Set_DigitalPotentiometer(address_P2, pinCS_P2, 0, rAB_P2, rWiper_P2);  // Initialisation R_P2
  Set_DigitalPotentiometer(address_P3, pinCS_P3, 0, rAB_P3, rWiper_P3);  // Initialisation R_P3
  Set_Potards(Pot_Mode_Init.pos1, Pot_Mode_Init.pos2, Pot_Mode_Init.pos3);
  // _____ Capteur _____
  Init_Capteur();
  // _____ Température _____
  Init_temperature();
  //
  Serial.println("-- Initialisation Finished");
  Serial.println("----- Loop Begin -----");

  // Serial.print("Gain: ");
  // Serial.println(Compute_Actual_Gain());

  // Serial.print("R_P1: ");
  // Serial.println(R_P1);
  // Serial.print("R_P2: ");
  // Serial.println(R_P2);
  // Serial.print("R_P3: ");
  // Serial.println(R_P3);
}


//====================================================
//==================== Main Loop =====================
//====================================================
void loop() {
  //delay(1000);

  if (TakeAndSendMeasure != FALSE) {
    // Lecture Capteur de Gaz.
    Resistance_CapteurGaz = Read_Capteur();
    // Lecture de la température.
    Temperature = Read_Temperature();
    // Affichage
    Serial.print("Résistance du Capteur: ");
    Serial.print(Resistance_CapteurGaz);
    Serial.print(" ; ");
    Serial.print("Température: ");
    Serial.println(Temperature);
    // Envoi BT
    BT_Sending(Compute_Actual_Gain(), Resistance_CapteurGaz, Temperature);
  }

  //Réception BT
  BT_Received_Message();
  delay(1000);
}



//====================================================
//==================== Functions =====================
//====================================================


//=========================== Capteur ===========================
void Init_Capteur() {
  pinMode(pinCapteur, INPUT);
  Serial.println("-- Capteur Resistance :  OK");
}

// Read_Capteur(): Mesure la tension de sortie de l'AOP, convertit, et renvoie la résistance.
float Read_Capteur() {
  float Resultat = 0.0;
  // float Gain = 0.0 ;
  // float VE = 0.0 ;
  // float VS = digitalRead(pinCapteur);
  // Gain = (1+ R_P3/R_P2) * ( R_P1 / (Res1 + R_P1) ) ;        // À VERIFIER
  // VE = VS / Gain ;
  // Resultat = (VE)*( Res1 / (Vcc-VE) ) ;                   // [ohm]
  float VS = analogRead(pinCapteur) * 5.0 / 1024.0;
  if ( (R_P2 > 0) && (VS != 0) ) {
    Resultat = ((1 + (R_P3 / R_P2)) * (Res1 * Vcc) / VS) - R_P1 - Res1;  //[ohm]
  } else {
    Resultat = 0;
  }
  Resultat = roundf(Resultat);  // [ohm]
  return Resultat;
}

//========================= Temperature =========================
void Init_temperature() {
  pinMode(pinTemp, INPUT);
  Serial.println("-- Capteur Temperature : OK");
}

// Read_Temperature(): Calcul de la température avec l'abbaque.           // À VOIR
float Read_Temperature() {
  float Tension_Temp = analogRead(pinTemp)*Vcc_Temp / 1024.0;
  // Résistance du capteur de température (serpentin d'Al) déduite d'un pont diviseur de tension.
  Rtemp = (Tension_Temp * R5) / (Vcc_Temp - Tension_Temp);
  // Résistivité du serpentin d'Al déduite à T° actuelle.
  float rho_T = Rtemp * rho_300K / R_300K;
  // Déduction de la T° actuelle avec linéarisation de la courbe résistivité-température de l'abaque.
  Temperature = (alpha_T * rho_T) + beta_T-273.15;  // [°C]
  Temperature = roundf(Temperature);         // [°C]
  return Temperature;
}

//==================== Digital Potentiometer ====================
// Set_DigitalPotentiometer : Initialise le potentiomètre digital donné.
/* Arguments :  - addr: addresse | - pinCS: pin de contrôle | - pos: position A-X | 
                  - rAB: résistance entre A-B | - rWiper: résistance offset | - R_pot: résistance Pn.
  */
void Set_DigitalPotentiometer(int addr, byte pinCS, int pos, float rAB, float rWiper) {
  pinMode(pinCS, OUTPUT);
  digitalWrite(pinCS, HIGH);
  SPI.begin();
  setPotWiper(addr, pinCS, pos, rAB, rWiper);
  Serial.print("-- Potentiometer P");
  Serial.print(Detect_Potentiometer(pinCS));
  Serial.println(":  OK");
}
// setPotWiper : paramètre la résistance du potentiomètre désigné par pinCS.
/* Arguments :  - addr: addresse | - pinCS: pin de contrôle | - pos: position A-X | 
                  - rAB: résistance entre A-B | - rWiper: résistance offset | - R_pot: résistance Pn.
  */
void setPotWiper(int addr, byte pinCS, int pos, float rAB, float rWiper) {
  pos = constrain(pos, 0, 255);  // limit wiper setting to range of 0 to 255
  digitalWrite(pinCS, LOW);      // select chip
  SPI.transfer(addr);            // configure target pot with wiper position
  SPI.transfer(pos);
  digitalWrite(pinCS, HIGH);  // de-select chip

  // print pot resistance between wiper and B terminal
  float R_pot = ((rAB * pos) / maxPositions) + rWiper;
  // Actualisation de la valeur de résistance du potentiomètre P... .
  Changing_Resistance_Pot(Detect_Potentiometer(pinCS), R_pot);
}

// Detect_Potentiometer : Renvoie le numéro associé au potentiomètre 1, 2 ou 3 selon pinCS.
// For setPotWiper only
int Detect_Potentiometer(byte pinCS) {
  int Resultat_Pot = 0;
  for (int i = 0; i < sizeof(Tab_pinCS); i++) {
    if (pinCS == Tab_pinCS[i]) {
      Resultat_Pot = i;
    }
  }
  return Resultat_Pot + 1;
}

// Changing_Resistance_Pot : Change la résistance du potentiomètre donné.
// For setPotWiper only
void Changing_Resistance_Pot(int Pot, float Resistance) {
  switch (Pot) {
    case 1:
      R_P1 = Resistance;
      break;
    case 2:
      R_P2 = Resistance;
      break;
    case 3:
      R_P3 = Resistance;
      break;
    default:
      break;
  }
}


// Set_Potards : positionne les potards 1, 2, 3 respectivement aux positions pos1, pos2, pos3.
// Arguments :  - pos1: position du potentiomètre 1 | - pos2: position du potentiomètre 2 | - pos3: position du potentiomètre 3.
void Set_Potards(int pos1, int pos2, int pos3) {
  setPotWiper(address_P1, pinCS_P1, pos1, rAB_P1, rWiper_P1);
  setPotWiper(address_P2, pinCS_P2, pos2, rAB_P2, rWiper_P2);
  setPotWiper(address_P3, pinCS_P3, pos3, rAB_P3, rWiper_P3);
}


// Research_Gain : Recherche des positions pos1, pos2, pos3, pour avoir un gain le plus proche de la consigne.
// Arguments: Gain_voulu: consigne de gain.
struct Tab_Pot Research_Gain(int Gain_voulu) {
  Gain_voulu = constrain(Gain_voulu, 0, 501);
  struct Tab_Pot Resultat_Gain = { 0.0, 0, 0, 0 };  // {Gain, pos1, pos2, pos3}
  float Resistance1 = 0.0;
  float Resistance2 = 0.0;
  float Resistance3 = 0.0;
  float Gain_calcule = 0.0;
  float Resultat = 0.0;

  for (int i = 1; i < 255; i += 10) {  // position du potard 1
    Resistance1 = ((rAB_P1 * i) / maxPositions) + rWiper_P1;
    for (int j = 1; j < 255; j += 10) {  // position du potard 2
      Resistance2 = ((rAB_P2 * j) / maxPositions) + rWiper_P2;
      for (int k = 1; k < 255; k += 10) {  // position du potard 3
        Resistance3 = ((rAB_P3 * k) / maxPositions) + rWiper_P3;
        Gain_calcule = (1 + Resistance3 / Resistance2) * (Resistance1 / (Res1 + Resistance1));

        if (abs(Gain_voulu - Gain_calcule) < abs(Gain_voulu - Resultat_Gain.gain)) {
          Resultat_Gain.gain = Gain_calcule;
          Resultat_Gain.pos1 = i;
          Resultat_Gain.pos2 = j;
          Resultat_Gain.pos3 = k;
        } else {
          NULL;
        }
      }
    }
  }
  return Resultat_Gain;
}

// Compute_Actual_Gain calcul le gain actuel.
float Compute_Actual_Gain() {
  float Gain = 0.0;
  // Calcul du Gain suivant la formule associée au circuit électronique.
  if (R_P2 > 0){
    Gain = (1 + (R_P3 / R_P2)) * (R_P1 / (Res1 + R_P1));  // Gain total de Res1 à Res4.
    // Gain = (1+ (R_P3/R_P2)) ;                                    // Gain de l'AOP uniquement: de R_P1 à Res4.
  }
  else {
    Gain = 0.0 ;
  }
  Gain = roundf(Gain);
  return Gain;
}



//==================== Function for Bluetooth ====================
// Initialise the Bluetooth
void InitBluetooth() {
  pinMode(pinBT_RXD, INPUT);
  pinMode(pinBT_TXD, OUTPUT);
  MyBT.begin(baudrate);
  Serial.println("-- Bluetooth :        OK");
}

//Function for decoding received message
/*
  Structure du message reçu (en octet):
  Mode du Gain + Valeur du Gain
  |     1      |       3       |
  1      -> Mode du Gain (M, A, B, C): -M: Manuel ; -A: Gain A ; -B: Gain B ; -C: Gain C.
  2 à 4  -> Gain de 1 à 500.
  */
  // Vraie lecture du message reçu par bluetooth
void BT_Received_Message(){
  char Message[32]={0};
  if (MyBT.available() > 0){
    int i=0;
    while (MyBT.available() > 0 && i < sizeof(Message)) { 
        Message[i++] = MyBT.read();
        delay(4);
    }
    Serial.print("ReceivedMessage: ");
    Serial.println(Message);
  }

  char sendMesure = Message[0] ;
  char Mode = Message[1] ;
  // int Valeur_Gain_a = atoi(*Message[1]);
  // int Valeur_Gain_b = atoi(*Message[2]);
  // int Valeur_Gain_c = atoi(*Message[3]);
  // int Valeur_Gain = Valeur_Gain_c*100 + Valeur_Gain_b*10 + Valeur_Gain_a ;

  // Flag pour l'envoi ou non de mesures
  switch (sendMesure){
    case 'm':           // Envoi de mesures
      TakeAndSendMeasure = TRUE ;
      break;
    case 'e':           // Pas d'envoi de mesures
      TakeAndSendMeasure = FALSE ;

      switch (Mode){
        // case 'M':
        //   struct Tab_Pot Tab_Gain = Research_Gain(Valeur_Gain) ;
        //   Set_Potards(Tab_Gain.pos1, Tab_Gain.pos2, Tab_Gain.pos3);
        //   break;
        case 'A':       // Mode A est appliqué, voir Pot_Mode_A
          Set_Potards(Pot_Mode_A.pos1, Pot_Mode_A.pos2, Pot_Mode_A.pos3);
          break;
        case 'B':       // Mode B est appliqué, voir Pot_Mode_B
          Set_Potards(Pot_Mode_B.pos1, Pot_Mode_B.pos2, Pot_Mode_B.pos3);
          break;
        case 'C':       // Mode C est appliqué, voir Pot_Mode_C
          Set_Potards(Pot_Mode_C.pos1, Pot_Mode_C.pos2, Pot_Mode_C.pos3);
          break;
        case 'D':       // Mode D est appliqué, voir Pot_Mode_D
          Set_Potards(Pot_Mode_D.pos1, Pot_Mode_D.pos2, Pot_Mode_D.pos3);
          break;
        case 'E':       // Mode E est appliqué, voir Pot_Mode_E
          Set_Potards(Pot_Mode_E.pos1, Pot_Mode_E.pos2, Pot_Mode_E.pos3);
          break;
        default:
          break;
      }
      break;

    default:
      TakeAndSendMeasure = FALSE ;
      break;
  }

}







//Function for encoding send message
/*
  Structure du message envoyé (en octet): 35 octets
  Valeur du gain + ; + Valeur résistance + ; + Valeur température + ;
  | 1 |     3    | 1 |        10         | 1 |       3            | 1 |
  */
void BT_Sending(float Valeur_Gain, float Valeur_Res, float Valeur_Temp) {
  String Message;
  String Valeur_Gain_char = String(Valeur_Gain);
  String Valeur_Res_char = String(Valeur_Res);
  String Valeur_Temp_char = String(Valeur_Temp);
  int Longueur = sizeof(Valeur_Gain_char) + sizeof(Valeur_Res_char) + sizeof(Valeur_Temp_char) + 4 + 3;
  // Serial.println(Longueur);

  Message = Valeur_Gain_char + ";" + Valeur_Res_char + ";" + Valeur_Temp_char + ";";
  char MessageToSend[Longueur];
  for (int i = 0; i < Longueur; i++) {
    MessageToSend[i] = Message[i];
  }

  MyBT.write(MessageToSend);
  Serial.print("MessageToSend: ");
  Serial.println(MessageToSend);
}
