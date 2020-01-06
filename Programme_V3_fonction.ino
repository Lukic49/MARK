/*
 * Programme dédié au Robot MARK (N°3)
 * Le robot rempli tous les champs du cahier des charges à l'exception de la détection de la vitesse max des roues
 * Le programme a été réalisé par Luc Veyriol et Lucas Raoult
 */
//Ajout des fichiers sources (provenance des fonctions)
#include "Ultrasonic.h"
#include "rgb_lcd.h"
#include "Grove_I2C_Motor_Driver.h"
#include "Encoder.h"
#include "SparkFunLSM6DS3.h"  //Librairy de l'accéléromètre

#define COEF_TPS 1.290
#define COEF_DIST 1.
#define CIRC_ROUE 2*3.1459*3  //3cm = rayon d'une roue
#define CORRECTION_MOTEUR 28 
#define CURRENT 0.150 //On suppose le courant conssommée par les moteurs constant = 150mA //En réalité lecourant varie entre 100 et 200 mA sur un sol
#define TENSION_ECH 7.00/143.00                //Tension mis à l'échelle GAIN inclu 143 = signal retourné par A0 non modifié 
//*****Déclaration des variables non matériel
volatile short int start = 0;
volatile short int tempo = 0, tempo2 = 0; //tempos de l'interruption de débordement
volatile short int test_20cm = 0;
volatile short int mode_motor;

//*****Définition des équipements et déclaration de leurs variables associées*****
//Ultrasons
Ultrasonic us_front(8); 
Ultrasonic us_right(10);  
Ultrasonic us_left(2);
volatile int dist_us_right, dist_us_left, dist_us_front;

//Encoder
Encoder knobLeft(18, 29);
Encoder knobRight(19, 27);
volatile long newLeft, newRight;
volatile int positionLeft  = -999;
volatile int positionRight = -999;
volatile int compt_dist;
volatile int distance_parcours;
//A REVOIR
volatile float vit_max_droite;
volatile float test_vit_max_droite;
volatile float vit_max_gauche;
volatile float test_vit_max_gauche;

//Afficheur LCD
rgb_lcd lcd;
volatile  int ecran = 0;
volatile unsigned long chrono, t0, t1;

//Divseur de tension
volatile float voltage, energie;

//Joystick
volatile int joystick_X;
volatile int joystick_Y;


//------------FONCTIONS D'INTERRUPTION PAR TIMER2----------------
ISR(TIMER2_OVF_vect){ //Timer2 = 8bits --> 256: valeurs 0 --> 255   freq=16MHz  T=625µs
    tempo++;  //A CHAQUE TOP ATTEINT --> INCRÉMENTATION DU COMPTEUR = TEMPORISATION ENTRE 2 MESURES
    if(tempo >= 30 ){  // QUAND LE TOP A ÉTÉ ATTEINT 30 fois --> ON AFFECTE LES MESURES DES ULTRASONS À LEUR VARIABLE RESPECTIVE! Limitation des mesures
        //*****CAPTEURS ULTRASONS*****
        dist_us_front = us_front.MeasureInCentimeters(); //variable globale "dist_us_front" prend la valeur de la mesure en cm de l'US de front
        dist_us_left = us_left.MeasureInCentimeters();  //............................................................................... de gauche
        dist_us_right = us_right.MeasureInCentimeters(); //............................................................................... de droite
        joystick_X = analogRead(A3);  //Lecture analogique du joystick --> axe x + appuie
        joystick_Y = analogRead(A2);  //Lecture analogique du joystick --> axe y
        voltage = analogRead(A0) * TENSION_ECH ; //voltage reçoit le signal du diviseur de tension mis à l'échelle part rapport au 7V délivré par la batterie
        tempo = 0;  //Remise à 0 du compteur
    }
}

void configTimer2(){  //configuration du Timer 2
    TCCR2A = 0; //choix mode: WGM2 = 0, WGM1 = 0, WGM0 = 0 -->  0 = mode normal 
    TCCR2B = (1<<CS22)+(1<<CS21); //CHOIX DU PRÉDIVISEUR = 256 
    TIMSK2 |= (1<<TOIE2); //TOIE2: "Timer/Counter2 Overflow Interrupt Enable" --> Rend possible l'interrution de débordement
}

//------------------- GESTION MARCHE / ARRET DU ROBOT ------------------------
void marche_arret(){
    if (analogRead(A2) == 1023 && start == 0){    //ORDRE DE MARCHE
        start = 1;
        t0 = millis();
        _delay_ms(200);
    }
    if (analogRead(A2) == 1023 && start == 1){    //ORDRE D'ARRÊT
        start = 0;
        t1 = millis();                  //enregistrement du compteur "millis" à l'appui sur STOP
        _delay_ms(200);
    }
    if (start == 1){
       t1 = millis();                   //enregistrement du compteur "millis" à l'appui sur START
    }
    chrono = (t1 - t0) * COEF_TPS / 1000 ;     //COEF_TPS = facteur d'echelle - chrono en seconde 
    if (chrono == 600){           //10 minutes de parcours --> STOP
        start = 0 ;
    }
}
//------------------- GESTION MOTEUR START --------------------------------
void gest_motors_start(){
    switch(mode_motor){
    case 0:
        Motor.speed(MOTOR1,100);  //vitesse (-100 ; +100)
        Motor.speed(MOTOR2, 100);   // Ajustement du moteur droit par une correction = -28
        break;
    case 1: //VIRE A DROITE
        Motor.speed(MOTOR1,100);  //MOTOR1 coté gauche
        Motor.speed(MOTOR2,50);  //MOTOR2 coté droite //60 trop lent 50 ok
        break;
    case 2: // VIRE A GAUCHE
        Motor.speed(MOTOR1,50);
        Motor.speed(MOTOR2,100);
        break;
    case 3: //VIRAGE SERRE A GAUCHE
        Motor.speed(MOTOR1,-75);  //MOTOR1 coté gauche
        Motor.speed(MOTOR2,100);  //MOTOR2 coté droite
        break;
    case 4: // VIRAGE SERRE A DROITE
        Motor.speed(MOTOR1,100);
        Motor.speed(MOTOR2,0);
        break;
    case 5:
        start = 0;
        break;
    }
}

//------------------- GESTION MOTEUR STOP --------------------------------
void gest_motors_stop(){
    Motor.stop(MOTOR1);
    Motor.stop(MOTOR2);
    if(joystick_Y > 650){   //Remise à 0 des données
        t0 = 0; t1 = 0; //chrono = t0 - t1
        //vit_max_droite = 0;
        //vit_max_gauche = 0;
        knobLeft.write(0);
        knobRight.write(0);
        compt_dist = 0;
        energie = 0;
    }
}

//------------------Detection capteurs US à moins de 20 cm ---------------
void nb_dist20(){
    if ( (((dist_us_front < 20) || (dist_us_right < 20) || (dist_us_left < 20)) and test_20cm == 0) and start == 1){  // or dist_us_left < 20 or dist_us_right < 20)
       compt_dist = compt_dist + 1;                                                            // or dist_us_front < 20
       test_20cm = 1;
    }
    else if (dist_us_front > 20 and dist_us_right > 20 and dist_us_left > 20 and test_20cm == 1 and start == 1){   //dist_us_left < 20 and dist_us_right < 20 and
        test_20cm = 0;
    }
}

//------------------- Nombre de tour de chaque Roue -------------------
void calcul_tour_roue(){
    newLeft = knobLeft.read();  //variable globale "newLeft" prend la valeur de la mesure en cm de l'US de front
    newRight = knobRight.read();  //variable globale "newRight" prend la valeur de la mesure en cm de l'US de front
    if( newLeft != positionLeft || newRight != positionRight ) {
        positionLeft = newLeft / 2500;  //2500 est la valeur retournée par l'encodeur de newLeft/newRight pour 1 tour de roue
        positionRight = newRight / 2500;
    }
    distance_parcours = (positionLeft + positionRight)* CIRC_ROUE / 2;
}

//------------------- GESTION DES DETECTION ULTRASON "LEFT"/"RIGHT"/"FRONT" --------------------------------
void test_US(){
    if( (dist_us_right <= 60) && (dist_us_right >= 50) ){ //entre 55 et 60 cm
        mode_motor = 0;
    }
    if(dist_us_right > 60){
        mode_motor = 1;
    }
    if(dist_us_right < 50){
        mode_motor = 2;
    }
    if( (dist_us_front < 70 && dist_us_left > 100) ){ //condition rajoutée
        mode_motor = 3;
    }
    if( (dist_us_right > 250) && (dist_us_front > 170) ){
        mode_motor = 4;
    }
    if( (dist_us_right < 90) && (dist_us_left < 90) && (dist_us_front < 75) ){
        mode_motor = 5;
    }
}

//------------------- AFFICHAGE ECRAN L'ECRAN --------------------------------
void affichage_LCD(){
//Gestion des changements d'écran
  if (joystick_X > 700){ //GESTION CHANGEMENT D'ÉCRAN QUAND JOYSTICK VIRE A GAUCHE
      ecran = ecran + 1;
      lcd.clear();
      _delay_ms(200);
  }
   if (joystick_X < 400){  //GESTION CHANGEMENT D'ÉCRAN QUAND JOYSTICK VIRE A DROITE
      if(ecran == 0){
          lcd.clear();
          ecran = 6;
          _delay_ms(200);
      }
      else{
          ecran = ecran - 1;
          lcd.clear();
          _delay_ms(200);
      }
  }
 
//Séléction de l'écran selon la valeur de la variable 'ecran'
  switch (ecran){
      case 0:
          lcd.setCursor(0,0);
          lcd.print("ETAT DU ROBOT :");
          lcd.setCursor(0,1);
          if(start == 1){ 
              lcd.print("Marche");  
          }
          else{  
              lcd.print("Arret ");
          }
          break;
      case 1:
          lcd.setCursor(0,0);   
          lcd.print("VITESSE MAX :");
          lcd.setCursor(0,1);
          lcd.print("R :");
          lcd.print(vit_max_droite);
          lcd.setCursor(9,1);
          lcd.print("L :");
          lcd.print(vit_max_gauche);
          break;
      case 2:
          lcd.setCursor(0,0); 
          lcd.print("NBR TOUR ROUE :");
          lcd.setCursor(0,1);
          lcd.print("L :");
          lcd.print(positionLeft);
          lcd.print("   ");
          lcd.setCursor(9,1);
          lcd.print("R :");
          lcd.print(positionRight);
          lcd.print("   ");
          break;
      case 3:
          lcd.setCursor(0,0);
          lcd.print("Dist parcourue:");
          lcd.setCursor(0,1);
          lcd.print(distance_parcours); 
          lcd.print(" cm    ");
          break;
      case 4:
          lcd.setCursor(0,0);
          lcd.print("COMPT DIST < 20");
          lcd.setCursor(0,1);
          lcd.print(compt_dist);
          lcd.print("  ");
          break;
      case 5:
          lcd.setCursor(0,0);
          lcd.print("chrono : ");
          lcd.print(chrono);   //en seconde
          lcd.print(" s  ");
          break;
      case 6:
          lcd.setCursor(0,0);
          lcd.print("Energie = ");   //en seconde /1000
          lcd.print(energie);   //U * I * t = Energie (J) 
          lcd.print(" J ");
          lcd.setCursor(0,1);
          lcd.print(voltage);   //Tension en sortie de batterie 
          lcd.print(" V "); 
          break;
      default :
          ecran = 0;
          break;
  }
}

void setup() {
  pinMode(A0,INPUT);  //DÉCLARATION DE LA PIN ANALOGIQUE "A0" EN ENTRÉE --> relevé de signal sur diviseur de tension
  pinMode(A2,INPUT);  //DÉCLARATION DE LA PIN ANALOGIQUE "A2" EN ENTRÉE --> appuie sur JOYSTICK
  pinMode(A3,INPUT);  //DÉCLARATION DE LA PIN ANALOGIQUE "A3" EN ENTRÉE --> mouvement JOYSTICK
  lcd.begin(16, 2);   //INITIALISATION DE LA TAILLE DE L'AFFICHEUR
  configTimer2();   //APPEL DE LA FONCTION "CONFIGTIMER2"
  sei();            //AUTORISE LES INTERRUPTIONS A SE DÉLENCHER
}

void loop() {
  //Déclaration des variables locales à la "affichage_LCD()
  //sum_voltage = sum_voltage + (voltage[i] / 1000); //tension en mV
  //sum_voltage = (sum_voltage * 3 * 4980 / 1023.00); //set gain to 3
  energie = voltage * CURRENT * chrono;  //gain de 10 / CURRENT = 150mA / "/3600" --> Wh
  
  affichage_LCD();
  marche_arret();
  test_US();
  calcul_tour_roue();
  nb_dist20();
  
  if( start == 1 ){
      gest_motors_start();
  }else{
      gest_motors_stop();
  }

}

/* 
//------------------- Calcul Vitesse MAX de chaque roue ---------------
  if (test_vit_max_gauche > vit_max_gauche){
    vit_max_gauche = test_vit_max_gauche;
  }

  if (test_vit_max_droite > vit_max_droite){
    vit_max_droite = test_vit_max_droite;
  }
*/
