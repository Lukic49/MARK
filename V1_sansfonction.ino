//Ajout des fichiers sources (provenance des fonctions)
#include "Ultrasonic.h"
#include "Grove_LED_Bar.h"
#include "rgb_lcd.h"
#include "Grove_I2C_Motor_Driver.h"
#include "Encoder.h"
#include "SparkFunLSM6DS3.h"

//*****Définition des équipements et déclaration de leurs variables associées*****
//Ultrasons
Ultrasonic us_front(8); 
Ultrasonic us_right(10);  
Ultrasonic us_left(2);
volatile int dist_us_right, dist_us_left, dist_us_front;
volatile int mode;
volatile  int ecran = 0;
volatile  int start = 0;
volatile int tour_roue_droite = 0;
volatile int tour_roue_gauche = 0;
volatile int tempo = 0;
volatile int test = 0 ;
volatile int compt_dist ;

//Encoder
Encoder knobLeft(18, 29);
Encoder knobRight(19, 27);
long newLeft, newRight;
long positionLeft  = -999;
long positionRight = -999;
unsigned long duration_stop, reel_time=0, t0, t1, t2;
long i = 0;


//Afficheur LCD
rgb_lcd lcd;

//*****FONCTIONS D'INTERRUPTION PAR TIMER2*****
ISR(TIMER2_OVF_vect){
  tempo++;  //A CHAQUE TOP ATTEINT --> INCRÉMENTATION DU COMPTEUR = TEMPORISATION ENTRE 2 MESURES
  if(tempo > 30){  // QUAND LE TOP A ÉTÉ ATTEINT 30 fois --> ON AFFECTE LES MESURES DES ULTRASONS À LEUR VARIABLE RESPECTIVE
    //*****CAPTEURS ULTRASONS*****
    dist_us_front = us_front.MeasureInCentimeters(); //déclaration d'une variable globale: prend la valeur de la mesure en cm de l'US de front
    dist_us_left = us_left.MeasureInCentimeters();  //............................................................................... de gauche
    dist_us_right = us_right.MeasureInCentimeters(); //............................................................................... de droite
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
  long newLeft, newRight;
  float vit_max_droite;
  float test_vit_max_droite;
  float vit_max_gauche;
  float test_vit_max_gauche;
  float temps;
  const byte infrared = 6;
  int k =  100 - dist_us_right; //variable proportionnelle k>0 --> proche mur droit --> 
  
//------------------- AFFICHAGE ECRAN L'ECRAN --------------------------------
  if (analogRead(A3) > 700 ){ //GESTION CHANGEMENT D'ÉCRAN QUAND JOYSTICK VIRE A GAUCHE
    ecran = ecran+1;
    lcd.clear();
    _delay_ms(200);
  }

   if (analogRead(A3) < 400 ){  //GESTION CHANGEMENT D'ÉCRAN QUAND JOYSTICK VIRE A DROITE
    ecran = ecran-1;
    lcd.clear();
    _delay_ms(200);
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
      lcd.print("Distance parcourue:");
      lcd.setCursor(0,1);
      lcd.print((positionLeft+positionRight)*19 /2);
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
      lcd.print("ENE CONS");
      break;
    case 7:
      lcd.setCursor(0,0);
      lcd.print(reel_time/1000);
      lcd.print(" s");
      /*lcd.setCursor(9,0);
      lcd.print(t1/1000);
      lcd.print(" s");*/
      lcd.setCursor(5,1);
      lcd.print(duration_stop/1000);
      lcd.print(" s");
      break; 
    default :
      ecran = 0 ;
      break;
  }

//------------------- MARCHE / ARRET DU ROBOT ------------------------
  if (analogRead(A2) == 1023 && start == 0){
    start=1;
    t0 = millis() - 200;
    reel_time = millis() - t0 - duration_stop;
    _delay_ms(200);
  }
  if (analogRead(A2) == 1023 && start == 1){
    start = 0;
    t1 = millis() - 200;  //-200ms de la tempo dans condition
    duration_stop = t1 - t0;
    _delay_ms(200);
  }
  if (start == 1){
    if( (dist_us_right <= 60) && (dist_us_right >= 50)  ){//ok
      mode = 0;
    }
    if( (dist_us_right > 60) ){
      mode = 1;
    }
    if(dist_us_right < 50){
      mode = 2;
    }
    if(dist_us_front < 70){
      mode = 3;
    }
    if( (dist_us_right > 250) && (dist_us_front > 170) ){
      mode = 4;
    }

    switch(mode){
    case 0:
        Motor.speed(MOTOR1,100);  //vitesse (-100 ; +100)
        Motor.speed(MOTOR2,100);
        break;
    case 1:
        Motor.speed(MOTOR1,100);  //MOTOR1 coté gauche
        Motor.speed(MOTOR2,85);  //MOTOR2 coté droite
        break;
    case 2:
        Motor.speed(MOTOR1,85);
        Motor.speed(MOTOR2,100);
        break;
    case 3:
        Motor.speed(MOTOR1,-50);  //MOTOR1 coté gauche
        Motor.speed(MOTOR2,100);  //MOTOR2 coté droite
        break;
    case 4:
        Motor.speed(MOTOR1,100);
        Motor.speed(MOTOR2,-50);
        break;
    default: 
        Motor.stop(MOTOR1);
        Motor.stop(MOTOR2);
        break;
    }
  }else{
      Motor.stop(MOTOR1);
      Motor.stop(MOTOR2);
  }

 
//DETECTION US + 1er virage
 
//------------------- Nombre de tour de chaque Roue -------------------
    newLeft = knobLeft.read();
    newRight = knobRight.read();
  if (newLeft != positionLeft || newRight != positionRight) {
    positionLeft = newLeft / 3600;
    positionRight = newRight / 3600;
  }

/* 
//------------------- Calcul Vitesse MAX de chaque roue ---------------

  if (test_vit_max_gauche > vit_max_gauche){
    vit_max_gauche = test_vit_max_gauche;
  }

  if (test_vit_max_droite > vit_max_droite){
    vit_max_droite = test_vit_max_droite;
  }

//------------------- Calcul Distance Parcourue -----------------------
  
  if (temps > 10){
    start=0;
  }
  
//------------------- Calcul TEMPS ------------------------------------



//------------------- Compter distance < 20 cm ------------------------
*/

  if ( (((dist_us_front < 20) || (dist_us_right < 20) || (dist_us_left < 20)) and test == 0) and start == 1){  // or dist_us_left < 20 or dist_us_right < 20)
   compt_dist = compt_dist + 1;                                                                // or dist_us_front < 20
   test = 1;
  }
  else{
    if (dist_us_front > 20 and dist_us_right > 20 and dist_us_left > 20 and test == 1 and start == 1){   //dist_us_left < 20 and dist_us_right < 20 and
     test = 0;
    }
  }


//------------------- detection infrarouge ------------------------
}
