//AJOUTS DES BIBLIOTEQUES CONTENANT L'ENSEMBLE DES FONCTIONS ASSOCIEES AUX ÉQUIPEMENTS DU ROBOT MARK ET DE L'ARDUINO
#include <Encoder.h>
#include <Wire.h>
#include "Grove_I2C_Motor_Driver.h" //appelé par "Motor."
#include "Ultrasonic.h" //appelé par Ultrasonic puis distingué pour les différents US avec "us_front(8)", "us_right(10)", "us_left(2)"
#include "rgb_lcd.h"  //appelé par rgb_lcd puis abrégé avec "lcd"

#define SPEED 50
#define SPEED_MAX 100
#define SPEED_HIGH 75
#define SPEED_LOW 25
#define CORRECTION
#define LARGEUR_MURS 140 //largeur en cm du couloir à la position initiale


// ÉTIQUETTAGE DE L'ADRESSE DE LA COMMUNICATION I2C (COMPRIS ENTRE 0x00 et 0x0F)
#define I2C_ADDRESS 0x0f

//LCD declaration
rgb_lcd lcd;
const int colorR = 100;
const int colorG = 100;
const int colorB = 200;

//DÉCLARATION DES ULTRASONS EXISTANTS x3 + VARIABLE(S) ASSOCIÉE(S)
Ultrasonic us_front(8); //ultrason "front" est conneccté à la PIN 8 des E/S digitales de l'ARDUINO 
Ultrasonic us_right(10);  //........"right"........................PIN 10..........................
Ultrasonic us_left(2);  //........."left".......................PIN 2..........................
volatile int dist_us_right, dist_us_left, dist_us_front, diff_us;

//DECLARATION DE(S) VARIABLE(S) ASSOCIÉE(S) AU JOYSTICK
volatile int etat_joystick_click;

//FONCTION GERANT LES INTERRUPTIONS
ISR(TIMER2_OVF_vect){           //acquisitions
//*****CAPTEURS ULTRASONS*****
dist_us_front = us_front.MeasureInCentimeters(); //déclaration d'une variable globale: prend la valeur de la mesure en cm de l'US de front
dist_us_left = us_left.MeasureInCentimeters();  //............................................................................... de gauche
dist_us_right = us_right.MeasureInCentimeters(); //............................................................................... de droite

//*****JOYSTICK*****
etat_joystick_click = run_stop();  //etat du joystick 1 = marche, 0 = arrêt
}

bool run_stop(){  //GESTION DU MARCHE/ARRET VIA LE CLICK DU JOYSTICK
  bool marche;
    if(analogRead(A9) == 1024){
      marche = 1;
    }
    else{  //VOIR CE QUI SE PASSE EN CAS DE MOUVEMENT DU JOYSTICK (CLICK OK)
      marche = 0;
    }
  return marche;
}

void traj_droite(){ //CORRECTION TRAJECTOIRE POUR AVANCER DROIT
  bool sens;
  if((diff_us > 0) && (dist_us_front > 60)){  //ROBOT PLUS A GAUCHE
    Motor.speed(MOTOR1,SPEED + diff_us);
    Motor.speed(MOTOR2,SPEED - diff_us);
    sens = 1;
  }
  else if((diff_us < 0) && (dist_us_front > 60)){ //ROBOT PLUS A DROITE
    Motor.speed(MOTOR1,SPEED - diff_us);
    Motor.speed(MOTOR2,SPEED + diff_us);
    sens = 0;
  }//A TESTER DOUTEUX...
  else if (dist_us_front < 60){ //CAS: ROTATION TROP GRANDE --> DETECTION "us_front" ALORS CONTRE-BRAQUE
        if (sens == 1){
          Motor.speed(MOTOR1,SPEED - diff_us);
          Motor.speed(MOTOR2,SPEED + diff_us);
        }else{
          Motor.speed(MOTOR1,SPEED + diff_us);
          Motor.speed(MOTOR2,SPEED - diff_us);
        }
  }
}
void virage_diff(int speed_right, int speed_left){
  
}

void virage_rotation_robot(int speed_right, int speed_left){
            Motor.speed(MOTOR1,SPEED + diff_us);
            Motor.speed(MOTOR2,SPEED - diff_us);
}

int test_fin(){
  
}

void setup() {
  Serial.begin(9600); //début de la communication série fonction "serial" définie dans la bibliotèque "wire" à 9600baups
  
  Motor.begin(I2C_ADDRESS); //initialisation du module motor driver par I2C à 9600bps
  Motor.stop(MOTOR1); //consigne du moteur 1 à zéro
  Motor.stop(MOTOR2); //consigne du moteur 2 à zéro
  
  lcd.begin(16, 2); //initialisation de la commnicaton du module LCD nombre de colonne = 16, nombre de ligne = 2
  lcd.setRGB(colorR, colorG, colorB);
  lcd.setCursor(0, 0);
}

void loop(){
  diff_us = ( (dist_us_right - dist_us_left) * 100 / LARGEUR_MURS );
  
  if(etat_joystick_click){ //condition dépendant du click joystick "Marche/Arrêt" (!= ON/OFF)
    //*****test affichage*****
    lcd.setCursor(6, 0);  //POSTIONNEMENT DU CURSEUR
    lcd.print(dist_us_front); //AFFICHAGE DU CONTENU DE LA VARIABLE "dist_us_front"
    lcd.print(" cm  "); //AFFICHAGE DES CARACTERES " cm  "
    lcd.setCursor(0, 1);
    lcd.print(dist_us_left);
    lcd.print(" cm  ");
    lcd.setCursor(10, 1);
    lcd.print(dist_us_right);
    lcd.print(" cm  ");
    //*****gestion des fonctions*****
    traj_droite();
    delay(1000);
    }
}
