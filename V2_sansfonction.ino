//Ajout des fichiers sources (provenance des fonctions)
#include "Ultrasonic.h"
#include "Grove_LED_Bar.h"
#include "rgb_lcd.h"
#include "Grove_I2C_Motor_Driver.h"
#include "Encoder.h"
#include "SparkFunLSM6DS3.h"

#define COEF_TPS 1.290
#define CORRECTION_MOTEUR 28

//*****Déclaration des variables non matériel
volatile  int start = 0;
volatile int tempo = 0; //tempo de l'interruption de débordement
volatile int test_20cm = 0 ;
volatile int mode_motor;
volatile int i = 0; //indice de la tension mesurée

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
volatile long positionLeft  = -999;
volatile long positionRight = -999;
volatile int compt_dist ;

//Afficheur LCD
rgb_lcd lcd;
volatile  int ecran = 0;
volatile unsigned long chrono, t0, t1;

//Divseur de tension
volatile long sum_voltage = 0, voltage[100];


//*****FONCTIONS D'INTERRUPTION PAR TIMER2*****
ISR(TIMER2_OVF_vect){ //Timer2 = 8bits --> 256: valeurs 0 --> 255   freq=16MHz  T=625µs
  tempo++;  //A CHAQUE TOP ATTEINT --> INCRÉMENTATION DU COMPTEUR = TEMPORISATION ENTRE 2 MESURES
  if(tempo > 30){  // QUAND LE TOP A ÉTÉ ATTEINT 30 fois --> ON AFFECTE LES MESURES DES ULTRASONS À LEUR VARIABLE RESPECTIVE! Limitation des mesures
    //*****CAPTEURS ULTRASONS*****
    dist_us_front = us_front.MeasureInCentimeters(); //déclaration d'une variable globale: prend la valeur de la mesure en cm de l'US de front
    dist_us_left = us_left.MeasureInCentimeters();  //............................................................................... de gauche
    dist_us_right = us_right.MeasureInCentimeters(); //............................................................................... de droite
    voltage[i] = analogRead(A0);
    
    i++;
    if(i == 100){
      i = 0;
    }
    tempo = 0;  //Remise à 0 du compteur
  }
}

void configTimer2(){  //configuration du Timer 2
  TCCR2A = 0; //choix mode: WGM2 = 0, WGM1 = 0, WGM0 = 0 -->  0 = mode normal 
  TCCR2B = (1<<CS22)+(1<<CS21); //CHOIX DU PRÉDIVISEUR = 256 
  TIMSK2 |= (1<<TOIE2); //TOIE2: "Timer/Counter2 Overflow Interrupt Enable" --> Rend possible l'interrution de débordement
}

void setup() {
  pinMode(A2,INPUT);  //DÉCLARATION DE LA PIN ANALOGIQUE "A2" EN ENTRÉE --> appuie sur JOYSTICK
  pinMode(A3,INPUT);  //DÉCLARATION DE LA PIN ANALOGIQUE "A3" EN ENTRÉE --> mouvement JOYSTICK
  lcd.begin(16, 2);   //INITIALISATION DE LA TAILLE DE L'AFFICHEUR
  configTimer2();   //APPEL DE LA FONCTION "CONFIGTIMER2"
  sei();            //AUTORISE LES INTERRUPTIONS A SE DÉLENCHER
}

void loop() {
//------------------- DECLARATION DE VARIABLE --------------------------------  
  volatile long newLeft, newRight;
  volatile float vit_max_droite;
  volatile float test_vit_max_droite;
  volatile float vit_max_gauche;
  volatile float test_vit_max_gauche;
  sum_voltage = sum_voltage + (voltage[i] / 1000); //tension en mV
  sum_voltage = (sum_voltage * 3 * 4980 / 1023.00); //set gain to 3
  //int k =  100 - dist_us_right; //variable proportionnelle k>0 --> proche mur droit -->
  
//------------------- AFFICHAGE ECRAN L'ECRAN --------------------------------
  if (analogRead(A3) > 700 ){ //GESTION CHANGEMENT D'ÉCRAN QUAND JOYSTICK VIRE A GAUCHE
    ecran = ecran + 1;
    lcd.clear();
    _delay_ms(200);
  }
   if (analogRead(A3) < 400 ){  //GESTION CHANGEMENT D'ÉCRAN QUAND JOYSTICK VIRE A DROITE
    if(ecran == 0){
      lcd.clear();
      ecran = 8;
      _delay_ms(200);
    }else{
      ecran = ecran-1;
      lcd.clear();
      _delay_ms(200);
    }
  }
  switch (ecran){
    case 0:
      lcd.setCursor(0,0);
      lcd.print("ETAT DU ROBOT :");
      lcd.setCursor(0,1);
      if(start == 1) lcd.print("Marche");  
      else  lcd.print("Arret ");
        
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
      lcd.print("R :");
      lcd.print(positionLeft);
      lcd.setCursor(9,1);
      lcd.print("L :");
      lcd.print(positionRight);
      break;
    case 3:
      lcd.setCursor(0,0);
      lcd.print("Dist parcourue:");
      lcd.setCursor(0,1);
      lcd.print((positionLeft + positionRight) * 19 / 2);
      break;
    case 4:
      lcd.clear(); //à tester
      lcd.setCursor(6,0);
      lcd.print(dist_us_front);
      lcd.setCursor(0,1);
      lcd.print(dist_us_left);
      lcd.setCursor(13,1);
      lcd.print(dist_us_right);
      break;
    case 5: 
      lcd.setCursor(0,0);
      lcd.print("COMPT DIST < 20");
      lcd.setCursor(0,1);
      lcd.print(compt_dist);
      break;  
    case 6 :
      lcd.setCursor(0,0);
      lcd.print("ENE CONS ");
      break;
    case 7:
      lcd.setCursor(0,0);
      lcd.print("chrono : ");
      lcd.print(chrono/1000);   //en seconde /1000
      lcd.print(" s  ");
      break; 
    case 8:
      lcd.setCursor(0,0);
      lcd.print("U(t) = ");   //en seconde /1000
      lcd.print(voltage[i]);   //en seconde /1000
      lcd.print(" mV   ");
      break;
    default :
      ecran = 0;
      break;
  }

//------------------- MARCHE / ARRET DU ROBOT ------------------------
  if (analogRead(A2) == 1023 && start == 0){    //ORDRE DE MARCHE
    start = 1;
    t0 = millis();
    _delay_ms(200);
  }
 
  if (analogRead(A2) == 1023 && start == 1){    //ORDRE D'ARRÊT
    start = 0;
    t1 = millis();
    _delay_ms(200);
  }
  
  if (start == 1){
    t1 = millis();
  }
 chrono = (t1 - t0) * COEF_TPS ;     //facteur d'echelle (millis

  if (chrono/1000 == 600){    //10 minutes de parcours --> STOP
    start = 0 ;
  }
  if( start == 1 ){
    if( (dist_us_right <= 60) && (dist_us_right >= 55) ){ //entre 55 et 60 cm
      mode_motor = 0;
    }
    if(dist_us_right > 60){
      mode_motor = 1;
    }
    if(dist_us_right < 55){
      mode_motor = 2;
    }
    if( (dist_us_front < 70 && dist_us_left > 100) ){ //condition rajoutée
      mode_motor = 3;
    }
    if( (dist_us_right > 250) && (dist_us_front > 170) ){
      mode_motor = 4;
    }
    if( (dist_us_right < 90) && (dist_us_left < 90) && (dist_us_front < 75) )
    {
      mode_motor = 5;
    }

    switch(mode_motor){
    case 0:
        Motor.speed(MOTOR1,100);  //vitesse (-100 ; +100)
        Motor.speed(MOTOR2, 100 - CORRECTION_MOTEUR);
        break;
    case 1: //TOURNE A DROITE
        Motor.speed(MOTOR1,100);  //MOTOR1 coté gauche
        Motor.speed(MOTOR2,50);  //MOTOR2 coté droite //60 trop lent 50 ok
        break;
    case 2: // TOURNE A GAUCHE
        Motor.speed(MOTOR1,80);
        Motor.speed(MOTOR2,100);
        break;
    case 3: //VIRAGE SERRE A GAUCHE
        Motor.speed(MOTOR1,-50);  //MOTOR1 coté gauche
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
  }else{
      Motor.stop(MOTOR1);
      Motor.stop(MOTOR2);
  }

//------------------- Nombre de tour de chaque Roue -------------------
    newLeft = knobLeft.read();
    newRight = knobRight.read();
  if (newLeft != positionLeft || newRight != positionRight) {
    positionLeft = newLeft / 3600;
    positionRight = newRight / 3600;
  }

  if ( (((dist_us_front < 20) || (dist_us_right < 20) || (dist_us_left < 20)) and test_20cm == 0) and start == 1){  // or dist_us_left < 20 or dist_us_right < 20)
   compt_dist = compt_dist + 1;                                                            // or dist_us_front < 20
   test_20cm = 1;
  }
  else if (dist_us_front > 20 and dist_us_right > 20 and dist_us_left > 20 and test_20cm == 1 and start == 1){   //dist_us_left < 20 and dist_us_right < 20 and
     test_20cm = 0;
    }

/* 
//------------------- Calcul Vitesse MAX de chaque roue ---------------
  if (test_vit_max_gauche > vit_max_gauche){
    vit_max_gauche = test_vit_max_gauche;
  }

  if (test_vit_max_droite > vit_max_droite){
    vit_max_droite = test_vit_max_droite;
  }
  
//------------------- Compter distance < 20 cm ------------------------
*/
}
