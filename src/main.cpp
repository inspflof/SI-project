#include "USBCDC.h"
#include <Arduino.h>
#include <Adafruit_MotorShield.h>
#include <Stepper.h>
#include <Servo.h>
#include <AccelStepper.h>
#include <Wire.h>
#include <WiFi.h>
#include <HTTPClient.h>
#include <ArduinoJson.h>

// ================== RESEAU ========================

const char* ssid = "Flo";
const char* password = "JambonBeurre";
const char* apiUrl = "https://api.si-project.devflo.freeddns.org/api/command/todo";

unsigned long lastCheckTime = 0;
const unsigned long checkInterval = 5000;

// ================== CONSTANTES ==================
const int STEPS = 4096;

const int PAS_PAR_CASE_X = 640;
const int PAS_PAR_CASE_Y = 2600;
const int PAS_Z = 4096; // avance/recul prise voiture

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

AccelStepper stepperZ(AccelStepper::FULL4WIRE, IN1Z, IN3Z, IN2Z, IN4Z);

// ================== SERVO ======================

#define SERVOZ1 17
#define SERVOZ2 18

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

  // Paramètres doux (comme Y)
  stepperX.setMaxSpeed(400);
  stepperX.setAcceleration(150);

  stepperX.move(steps);

  while (stepperX.distanceToGo() != 0) {
    stepperX.run();
  }

  motorX->release();  // économie + silence

  posX = targetX;
}
void homeX() {
  Serial.println("Homing X...");

  unsigned long start = millis();

  // Mouvement doux
  stepperX.setMaxSpeed(200);
  stepperX.setAcceleration(100);

  // Aller vers le endstop
  stepperX.moveTo(-100000000);

  while (digitalRead(ENDSTOP_X) == HIGH) {
    if (millis() - start > 15000) {
      Serial.println("ERREUR: fin de course X non detecte");
      break;
    }

    delay(2);

    if(digitalRead(ENDSTOP_X) == HIGH) {
      stepperX.run();
    }
  }

  Serial.println("Endstop X atteint");

  // Stop
  stepperX.setCurrentPosition(0);

  delay(100);

  // Petit recul doux
  stepperX.setMaxSpeed(150);
  stepperX.setAcceleration(100);

  stepperX.move(50);

  while (stepperX.distanceToGo() != 0) {
    stepperX.run();
  }

  // Reset position
  stepperX.setCurrentPosition(0);
  posX = 0;

  motorX->release();

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

  motorY->release();
  motorY2->release();

  posY = targetY;
}

void homeY() {
  Serial.println("Homing Y...");

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

    if(digitalRead(ENDSTOP_Y) == HIGH) {
      stepperY1.run();
      stepperY2.run();
    }
  }

  Serial.println("Endstop Y atteint");

  // STOP
  stepperY1.setCurrentPosition(0);
  stepperY2.setCurrentPosition(0);

  delay(100);

  // Petit recul pour relâcher le switch
  stepperY1.move(100);
  stepperY2.move(100);

  while (stepperY1.distanceToGo() != 0) {
    stepperY1.run();
    stepperY2.run();
  }

  // Reset position logique
  stepperY1.setCurrentPosition(0);
  stepperY2.setCurrentPosition(0);
  posY = 0;

  motorY->release();
  motorY2->release();

  Serial.println("Homing Y terminé");
}

void moveTo(int x, int y) {
  moveY(y); // hauteur d'abord
  moveX(x);
}

void homeZ() {
  Serial.println("Homing Z...");

  unsigned long start = millis();

  // Paramètres doux pour le homing
  stepperZ.setMaxSpeed(300);
  stepperZ.setAcceleration(200);

  // Mouvement vers le endstop
  stepperZ.moveTo(-100000);

  while (digitalRead(ENDSTOP_Z) == HIGH) {
    if(digitalRead(ENDSTOP_Z) == HIGH) {
      stepperZ.run();
    }

    // sécurité timeout
    if (millis() - start > 10000) {
      Serial.println("ERREUR: fin de course Z non detecte");
      return;
    }
  }

  Serial.println("Fin de course Z atteint");

  // Stop et reset position
  stepperZ.setCurrentPosition(0);

  delay(100);

  // Petit recul pour libérer le switch
  stepperZ.move((PAS_Z/2)+ 500);

  while (stepperZ.distanceToGo() != 0) {
    stepperZ.run();
  }

  // Reset position logique
  stepperZ.setCurrentPosition(0);

  Serial.println("Z calibre");
}

void moveZ(long steps) {
  stepperZ.move(steps);

  while (stepperZ.distanceToGo() != 0) {
    stepperZ.run();
  }
}

// ================== PRISE VOITURE ==================
void takeCar() {
  Serial.println("Prise voiture");

  servoZ1.write(0);
  delay(500);

  moveZ(PAS_Z/2);
  delay(200);

  servoZ1.write(90);
  delay(500);

  moveZ(-PAS_Z); 
  delay(200);

  servoZ1.write(0);
  delay(500);

  moveZ(PAS_Z);
  delay(200);

  servoZ1.write(90);
  delay(500);

  moveZ(-PAS_Z); 
  delay(200);

  servoZ1.write(0);
  delay(500);

  moveZ(PAS_Z);
  delay(200);

  servoZ1.write(90);
  delay(500);

  moveZ((-PAS_Z)/2);

  Serial.println("Voiture prise");
  delay(2000);
}

void dropCar() {
  Serial.println("Depot voiture");

  servoZ1.write(0);
  delay(500);

  moveZ(-(PAS_Z/2));
  delay(200);

  servoZ1.write(90);
  delay(500);

  moveZ(PAS_Z);
  delay(200);

  servoZ1.write(0);
  delay(500);

  moveZ(-PAS_Z); 
  delay(200);

  servoZ1.write(90);
  delay(500);

  moveZ(PAS_Z-2000);
  delay(200);

  servoZ1.write(0);
  delay(500);

  moveZ(-PAS_Z+2000);
  servoZ1.write(90);

  moveZ((PAS_Z/2)+1000);
  delay(500);
  servoZ1.write(0);
  delay(200);

  moveZ(-1000);
  
  Serial.println("Voiture déposée");
  delay(2000);
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
  Serial.print(y);

  moveTo(x, y);
  takeCar();

  Serial.println("Retour a (0,0)");
  moveTo(0, 0);
  dropCar();

  Serial.println("Operation terminee");
}

void defetcheCar(int x, int y) {
  if (x < 0 || x > 2 || y < 0 || y > 2) {
    Serial.println("Coordonnees invalides");
    return;
  }

  moveTo(0, 0);
  Serial.println("Aller au garage");

  takeCar();

  moveTo(x, y);
  Serial.print("Aller a la place ");
  Serial.print(x);
  Serial.print(",");
  Serial.print(y);
  dropCar();

}

// ================== MISE À JOUR DU STATUT (PUT) ==================

/**
 * Envoie une requête PUT pour mettre à jour le statut de la commande sur le serveur.
 * Format : x-www-form-urlencoded (status=DONE)
 */
void updateCommandStatus(int commandId, String newStatus) {
  if (WiFi.status() == WL_CONNECTED) {
    HTTPClient http;
    String updateUrl = "https://api.si-project.devflo.freeddns.org/api/command/" + String(commandId);
    
    http.begin(updateUrl);
    http.addHeader("Content-Type", "application/x-www-form-urlencoded");

    String httpRequestData = "status=" + newStatus;
    
    Serial.print("Mise à jour statut (");
    Serial.print(newStatus);
    Serial.println(") pour ID : " + String(commandId));

    int httpResponseCode = http.PUT(httpRequestData);

    if (httpResponseCode > 0) {
      Serial.print("Réponse serveur : ");
      Serial.println(httpResponseCode);
    } else {
      Serial.print("Erreur lors du PUT : ");
      // CORRECTION ICI : errorToString au lieu de errorString
      Serial.println(http.errorToString(httpResponseCode).c_str());
    }

    http.end();
  }
}

// ================== API ====================

void setupWiFi() {
  Serial.print("Connexion au WiFi : ");
  Serial.println(ssid);
  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nWiFi connecté !");
  Serial.print("Adresse IP : ");
  Serial.println(WiFi.localIP());
}

void getCommandFromAPI() {
  if (WiFi.status() == WL_CONNECTED) {
    HTTPClient http;
    http.begin(apiUrl);
    
    int httpCode = http.GET();

    if (httpCode == 200) {
      String payload = http.getString();
      
      // CORRECTION ICI : Utilisation de JsonDocument au lieu de StaticJsonDocument
      JsonDocument doc; 
      DeserializationError error = deserializeJson(doc, payload);

      if (!error) {
        int commandId = doc["id"];
        const char* type = doc["type"];      
        const char* status = doc["status"];  
        int targetX = doc["slot"]["x"];      
        int targetY = doc["slot"]["y"];      

        if (String(status) == "PENDING") {
          updateCommandStatus(commandId, "PROCESSING");

          if (String(type) == "PARK") {
            defetcheCar(targetX, targetY); 
            updateCommandStatus(commandId, "DONE");
          } 
          else if (String(type) == "RETRIEVE") {
            fetchCar(targetX, targetY); 
            updateCommandStatus(commandId, "DONE");
          }
        }
      } else {
        Serial.print("Erreur JSON : ");
        Serial.println(error.f_str());
      }
    }
    http.end();
  }
}

// ================== SETUP ==================

void setup() {
  Wire.begin();
  Wire.setClock(400000); // 400 kHz au lieu de 100 kHz
  Serial.begin(115200);

  while (!Serial);

  setupWiFi();
  
  pinMode(ENDSTOP_X, INPUT_PULLUP);
  pinMode(ENDSTOP_Y, INPUT_PULLUP);
  pinMode(ENDSTOP_Z, INPUT_PULLUP);

  pinMode(IN1Z, OUTPUT),
  pinMode(IN2Z, OUTPUT),
  pinMode(IN3Z, OUTPUT),
  pinMode(IN4Z, OUTPUT),

  servoZ1.attach(SERVOZ1);
  servoZ2.attach(SERVOZ2);

  servoZ1.write(0);
  
  stepperZ.setMaxSpeed(200);      // à ajuster
  stepperZ.setAcceleration(100);  // important pour le couple
  
  // if (!AFMS1.begin()) {
  //   while (1) {
  //     Serial.println("Shield 1 non detecte");
  //     delay(1000);
  //   }
  // }

  // if (!AFMS2.begin()) {
  //   while (1) {
  //     Serial.println("Shield 2 non detecte");
  //     delay(1000);
  //   }
  // }

  stepperX.setMaxSpeed(1000);
  stepperX.setAcceleration(300);
  
  stepperY1.setMaxSpeed(1000);
  stepperY1.setAcceleration(300);

  stepperY2.setMaxSpeed(1000);
  stepperY2.setAcceleration(300);

  motorY->setSpeed(300);
  motorY2->setSpeed(300);
  
  // Serial.println("Homing...");
  // homeY();
  // homeX();
  // homeZ();

  Serial.println("Parking automatique prêt");
}

// ================== LOOP ==================
void loop() {
  unsigned long currentMillis = millis();
  
  if (currentMillis - lastCheckTime >= checkInterval) {
    lastCheckTime = currentMillis;
    Serial.println("Vérification de l'API...");
    getCommandFromAPI();
  }
}