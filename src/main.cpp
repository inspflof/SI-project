#include <Arduino.h>
#include <Adafruit_MotorShield.h>
#include <Stepper.h>
#include <Servo.h>
#include <AccelStepper.h>
#include <Wire.h>

// ================== CONSTANTES ==================
const int STEPS = 4096;   // 28BYJ-48

const int PAS_PAR_CASE_X = 500;
const int PAS_PAR_CASE_Y = STEPS;
const int PAS_Z = STEPS; // avance/recul prise voiture

// ================== MOTEUR X ==================
Adafruit_MotorShield AFMS2 = Adafruit_MotorShield(0x61);
Adafruit_StepperMotor *motorX = AFMS2.getStepper(200, 1);

void stepX() {
  motorX->onestep(FORWARD, DOUBLE);
}

void stepXBack() {
  motorX->onestep(BACKWARD, DOUBLE);
}

AccelStepper stepperX(stepX, stepXBack);

// ================== MOTEUR Y ==================
Adafruit_MotorShield AFMS1 = Adafruit_MotorShield(0x60);
Adafruit_StepperMotor *motorY = AFMS1.getStepper(STEPS, 1);
Adafruit_StepperMotor *motorY2 = AFMS1.getStepper(STEPS, 2);

void stepY1() {
  motorY->onestep(FORWARD, DOUBLE);
}

void stepY1Back() {
  motorY->onestep(BACKWARD, DOUBLE);
}

void stepY2() {
  motorY2->onestep(BACKWARD, DOUBLE); // inversé
}

void stepY2Back() {
  motorY2->onestep(FORWARD, DOUBLE); // inversé
}

AccelStepper stepperY1(stepY1, stepY1Back);
AccelStepper stepperY2(stepY2, stepY2Back);

// ================== Capteurs fin de course =============

#define ENDSTOP_X A2
#define ENDSTOP_Y A3
#define ENDSTOP_Z A4

// ================== MOTEUR Z ==================
#define IN1Z 10
#define IN2Z 11
#define IN3Z 12
#define IN4Z 13
Stepper motorZ(STEPS, IN1Z, IN3Z, IN2Z, IN4Z);

// ================== SERVO ======================

#define SERVOZ1 A0
#define SERVOZ2 A1

Servo servoZ1;
Servo servoZ2;

// ================== POSITION ACTUELLE ==================
int posX = 0;
int posY = 0;

// ================== DEPLACEMENTS ==================
void moveX(int targetX) {
  int delta = targetX - posX;
  if (delta == 0) return;

  long steps = delta * PAS_PAR_CASE_X;

  stepperX.move(-steps);

  while (stepperX.distanceToGo() != 0) {
    stepperX.run();
  }

  posX = targetX;
}

void homeX() {
  Serial.println("Homing X...");

  // Sécurité timeout
  unsigned long start = millis();

  // On force une vitesse lente vers le ENDSTOP
  stepperX.setMaxSpeed(50);
  stepperX.setAcceleration(200);

  // Mouvement continu vers le home (direction BACKWARD)
  stepperX.moveTo(100000000);

  while (digitalRead(ENDSTOP_X) == HIGH) {

    // arrêt si timeout
    if (millis() - start > 15000) {
      Serial.println("ERREUR: fin de course X non detecte");
      break;
    }

    stepperX.run();
  }

  Serial.println("Endstop X atteint");

  // STOP
  stepperX.setCurrentPosition(0);

  delay(100);

  // Petit recul pour relâcher le switch
  stepperX.move(-20);

  while (stepperX.distanceToGo() != 0) {
    stepperX.run();
  }

  // Reset position logique
  stepperX.setCurrentPosition(0);
  posX = 0;

  Serial.println("Homing X terminé");
}

void moveY(int targetY) {
  int delta = targetY - posY;
  if (delta == 0) return;

  long steps = delta * PAS_PAR_CASE_Y;

  stepperY1.move(steps);
  stepperY2.move(steps);

  while (stepperY1.distanceToGo() != 0) {
    stepperY1.run();
    stepperY2.run();
  }

  posY = targetY;
}

void homeY() {
  Serial.println("Homing Y (AccelStepper)...");

  // Sécurité timeout
  unsigned long start = millis();

  // On force une vitesse lente vers le ENDSTOP
  stepperY1.setMaxSpeed(400);
  stepperY1.setAcceleration(200);

  stepperY2.setMaxSpeed(400);
  stepperY2.setAcceleration(200);

  // Mouvement continu vers le home (direction BACKWARD)
  stepperY1.moveTo(-100000000);
  stepperY2.moveTo(-100000000);

  while (digitalRead(ENDSTOP_Y) == HIGH) {

    // arrêt si timeout
    if (millis() - start > 50000) {
      Serial.println("ERREUR: fin de course Y non detecte");
      break;
    }

    stepperY1.run();
    stepperY2.run();
  }

  Serial.println("Endstop Y atteint");

  // STOP
  stepperY1.setCurrentPosition(0);
  stepperY2.setCurrentPosition(0);

  delay(100);

  // Petit recul pour relâcher le switch
  stepperY1.move(50);
  stepperY2.move(50);

  while (stepperY1.distanceToGo() != 0) {
    stepperY1.run();
    stepperY2.run();
  }

  // Reset position logique
  stepperY1.setCurrentPosition(0);
  stepperY2.setCurrentPosition(0);
  posY = 0;

  Serial.println("Homing Y terminé");
}

void moveTo(int x, int y) {
  moveY(y); // hauteur d'abord
  moveX(x);
}

void homeZ() {
  Serial.println("Homing Z...");

  unsigned long start = millis();
  // Aller vers le fin de course
  while (digitalRead(ENDSTOP_Z) == HIGH) {
    Serial.println("Homing Z en cours...");
    motorZ.step(1);  // recule (à adapter si sens inversé)

    // sécurité timeout
    if (millis() - start > 10000) {
      Serial.println("ERREUR: fin de course Z non detecte");
      return;
    }
  }

  Serial.println("Fin de course Z atteint");

  // Petit dégagement (important pour ne pas rester appuyé)
  for (int i = 0; i < 2700; i++) {
    motorZ.step(-1);
  }

  Serial.println("Z calibre");
}
// ================== PRISE VOITURE ==================
void takeCar() {
  Serial.println("Prise voiture");
  // motorZ.step(PAS_Z);
  servoZ1.write(90);
  delay(500);
  motorZ.step(2700); 
  delay(500);
  servoZ1.write(180);
  delay(500);
  motorZ.step(-2700); 

  Serial.println("Voiture prise");
  delay(2000);
}

void dropCar() {
  Serial.println("Depot voiture");
  motorZ.step(-PAS_Z);
}

// ================== LOGIQUE PRINCIPALE ==================
void fetchCar(int x, int y) {
  if (x < 0 || x > 2 || y < 0 || y > 2) {
    Serial.println("Coordonnees invalides");
    return;
  }

  Serial.print("Aller a la place ");
  Serial.print(x);
  Serial.print(",");
  Serial.println(y);

  moveTo(x, y);
  takeCar();

  Serial.println("Retour a (0,0)");
  moveTo(0, 0);
  dropCar();

  Serial.println("Operation terminee");
}

// ================== SETUP ==================
void setup() {
  Wire.begin();
  Wire.setClock(400000); // 400 kHz au lieu de 100 kHz
  Serial.begin(115200);

  while (!Serial);
  
  pinMode(ENDSTOP_X, INPUT_PULLUP);
  pinMode(ENDSTOP_Y, INPUT_PULLUP);
  pinMode(ENDSTOP_Z, INPUT_PULLUP);

  servoZ1.attach(SERVOZ1);
  servoZ2.attach(SERVOZ2);

  servoZ1.write(90);
  
  motorZ.setSpeed(10);
  
  
  if (!AFMS1.begin()) {
    while (1) {
      Serial.println("Shield 1 non detecte");
      delay(1000);
    }
  }

  if (!AFMS2.begin()) {
    while (1) {
      Serial.println("Shield 2 non detecte");
      delay(1000);
    }
  }

  stepperX.setMaxSpeed(1000);
  stepperX.setAcceleration(300);
  
  stepperY1.setMaxSpeed(1000);
  stepperY1.setAcceleration(300);

  stepperY2.setMaxSpeed(1000);
  stepperY2.setAcceleration(300);

  // motorX->setSpeed(300);  // 10 rpm
  motorY->setSpeed(300);
  motorY2->setSpeed(300);
  
  Serial.println("JE VAIS A LA MAISON");
  // homeZ();
  homeY();
  homeX();

  Serial.println("Parking automatique prêt");
}

// ================== LOOP ==================
void loop() {
  delay(5000);

  Serial.println("TEST X");

  moveX(1);
  delay(2000);

  moveX(0);
  delay(2000);

  Serial.println("TEST Y");

  moveY(1);
  delay(2000);

  moveY(0);
  delay(2000);
}


