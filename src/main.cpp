#include <Arduino.h>
#include <Adafruit_MotorShield.h>
#include <Stepper.h>
#include <Servo.h>

Adafruit_MotorShield AFMS = Adafruit_MotorShield();
// ================== CONSTANTES ==================
const int STEPS = 4096;   // 28BYJ-48

const int PAS_PAR_CASE_X = STEPS;
const int PAS_PAR_CASE_Y = STEPS;
const int PAS_Z = STEPS; // avance/recul prise voiture

// ================== MOTEUR X ==================
// Adafruit_StepperMotor *motorX = AFMS.getStepper(200, 3);

// ================== MOTEUR Y ==================
Adafruit_StepperMotor *motorY = AFMS.getStepper(STEPS, 1);
Adafruit_StepperMotor *motorY2 = AFMS.getStepper(STEPS, 2);

// ================== Capteurs fin de course =============

#define ENDSTOP_X 2
#define ENDSTOP_Y 3
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
  int steps = delta * PAS_PAR_CASE_X;
  Serial.println("deplacement X");
  Serial.println(steps);
  // if(delta > 0){
  //   motorX->step(steps, FORWARD, DOUBLE);
  // } else if (delta < 0) {
  //   motorX->step(steps, BACKWARD, DOUBLE);
  // }
  posX = targetX;
}

void moveY(int targetY) {
  int delta = targetY - posY;
  if (delta == 0) return;

  unsigned long totalSteps = abs(delta) * (unsigned long)PAS_PAR_CASE_Y;
  
  // Direction pour le moteur Y1
  uint8_t dir1 = (delta > 0) ? FORWARD : BACKWARD;
  // Direction opposée pour le moteur Y2
  uint8_t dir2 = (delta > 0) ? BACKWARD : FORWARD; 

  Serial.println("Deplacement Y synchrone (miroir)...");

  for (unsigned long i = 0; i < totalSteps; i++) {
    motorY->onestep(dir1, DOUBLE);
    motorY2->onestep(dir2, DOUBLE); // Le moteur 2 tourne à l'envers du moteur 1
    
    delayMicroseconds(800); // Ajuste ce délai si les moteurs forcent
  }

  delay(2000);

  posY = targetY;
}

void homeY() {
  Serial.println("Homing Y...");

  unsigned long start = millis();
  while (digitalRead(ENDSTOP_Y) == HIGH) {
    if (millis() - start > 10000) { // 10 sec timeout
      Serial.println("ERREUR: fin de course Y non detecte");
      break;
    }
    motorY->onestep(BACKWARD, DOUBLE);
    motorY2->onestep(FORWARD, DOUBLE); // miroir
    delayMicroseconds(800);
  }

  // petit recul
  for (int i = 0; i < 50; i++) {
    motorY->onestep(FORWARD, DOUBLE);
    motorY2->onestep(BACKWARD, DOUBLE);
    delayMicroseconds(800);
  }

  Serial.println("Fin de course Y atteint");

  posY = 0;
}

void homeX() {
  Serial.println("Homing X...");

  unsigned long start = millis();
  while (digitalRead(ENDSTOP_X) == HIGH) {
    if (millis() - start > 10000) { // 10 sec timeout
      Serial.println("ERREUR: fin de course X non detecte");
      break;
    }
    // motorX->onestep(BACKWARD, DOUBLE);
    delayMicroseconds(800);
  }

  // petit recul
  for (int i = 0; i < 50; i++) {
    // motorX->onestep(FORWARD, DOUBLE);
    delayMicroseconds(800);
  }

  Serial.println("Fin de course X atteint");

  posX = 0;
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
  servoZ1.write(180);
  
  delay(2000);
  // motorZ.step(-PAS_Z);
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
  pinMode(ENDSTOP_X, INPUT_PULLUP);
  pinMode(ENDSTOP_Y, INPUT_PULLUP);
  pinMode(ENDSTOP_Z, INPUT_PULLUP);

  servoZ1.attach(SERVOZ1);
  servoZ2.attach(SERVOZ2);

  servoZ1.write(90);

  Serial.begin(115200);

  while (!Serial);
  Serial.println("Stepper test!");

  // if (!AFMS.begin()) {         // create with the default frequency 1.6KHz
  // // if (!AFMS.begin(1000)) {  // OR with a different frequency, say 1KHz
  //   while (1) {
  //     Serial.println("Could not find Motor Shield. Check wiring.");
  //     delay(2000);
  //   }
  // }
  // Serial.println("Motor Shield found.");

  // // motorX->setSpeed(300);  // 10 rpm
  // motorY->setSpeed(300);
  // motorY2->setSpeed(300);
  
  motorZ.setSpeed(10);

  // homeY();
  // homeX();
  // homeZ();

  Serial.println("Parking automatique prêt");
}

// ================== LOOP ==================
void loop() {
  Serial.println("test ok");
  // Exemple : aller chercher la voiture en (2,1)
  // fetchCar(2, 1);
  // fetchCar(0, 0);

  
  Serial.println("test home Z");
  takeCar();
  

  delay(5000);
}


