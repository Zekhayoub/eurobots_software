#include <Wire.h>
#include <Adafruit_MotorShield.h>

// Initialisation du shield moteur et des moteurs
Adafruit_MotorShield AFMS = Adafruit_MotorShield();
Adafruit_DCMotor *motorLeft = AFMS.getMotor(1);
Adafruit_DCMotor *motorRight = AFMS.getMotor(2);

// ================================================================
//                           Connexions capteurs
// ================================================================

#define CPT_LEFT 3                 // Capteur infrarouge gauche
#define CPT_RIGHT 6                // Capteur infrarouge droit
#define CPT_VOID 7                // Capteur frontal (vide devant)
#define CPT_US_RIGHT_TRIG_PIN 10  // Trigger capteur ultrason droit
#define CPT_US_RIGHT_ECHO_PIN 8   // Echo capteur ultrason droit
#define CPT_US_LEFT_TRIG_PIN 13   // Trigger capteur ultrason gauche
#define CPT_US_LEFT_ECHO_PIN 11   // Echo capteur ultrason gauche

// ================================================================
//                           Paramètres
// ================================================================

volatile int speed = 55;                // Vitesse de base du robot
const bool motOff = false;             // Permet d’éteindre les moteurs via une condition
const int stopAfterSec = 15;           // Arrêt automatique après 15 secondes
const int delayStartSec = 5;           // Délai initial avant de démarrer (en secondes)
const float obstacleThreshold = 25.0;  // Seuil de détection obstacle (cm)
const int ultrasonicInterval = 200;    // Intervalle entre deux mesures (ms)

// Temps pour effectuer le contournement (~90°)
const int stopTime = 1000;
const int turnLeftTime = 500;
const int forwardTime = 1500;
const int turnRightTime = 500;
const int realignTime = 2000;

int maxTime = (stopAfterSec * 1000) + (delayStartSec * 1000);
float obstacleRightDistance = 0.0;
float obstacleLeftDistance = 0.0;
unsigned long startTime;
unsigned long lastUltrasonicCheck = 0;
unsigned long avoidStartTime = 0;
bool currentSensor = false;

// ================================================================
//                           États FSM
// ================================================================

enum State {
  IDLE,             // Attente initiale
  RUNNING,          // Suivi de ligne
  AVOID_OBSTACLE,   // Contournement obstacle
  REALIGN_LINE,     // Recherche de ligne après évitement
  STOPPED           // Arrêt
};

State currentState = IDLE;
int avoidStep = 0; // Étape du contournement

// ================================================================
//                           Setup
// ================================================================

void setup() {
  Serial.begin(9600);

  // Définition des pins capteurs
  pinMode(CPT_LEFT, INPUT);
  pinMode(CPT_RIGHT, INPUT);
  pinMode(CPT_VOID, INPUT);
  pinMode(CPT_US_RIGHT_TRIG_PIN, OUTPUT);
  pinMode(CPT_US_RIGHT_ECHO_PIN, INPUT);
  pinMode(CPT_US_LEFT_TRIG_PIN, OUTPUT);
  pinMode(CPT_US_LEFT_ECHO_PIN, INPUT);

  AFMS.begin();            // Initialisation du shield moteur
  startTime = millis();    // Sauvegarde du temps de démarrage
}

// ================================================================
//                           Boucle principale (FSM)
// ================================================================

void loop() {
  unsigned long elapsedTime = millis() - startTime;
  unsigned long currentTime = millis();

  // Mesure alternée des capteurs à ultrasons toutes les 200 ms
  if (currentTime - lastUltrasonicCheck >= ultrasonicInterval) {
    lastUltrasonicCheck = currentTime;

    if (currentSensor) {
      obstacleLeftDistance = readUltrasonic(CPT_US_LEFT_TRIG_PIN, CPT_US_LEFT_ECHO_PIN);
    } else {
      obstacleRightDistance = readUltrasonic(CPT_US_RIGHT_TRIG_PIN, CPT_US_RIGHT_ECHO_PIN);
    }

    currentSensor = !currentSensor;
  }

  // Machine à états
  switch (currentState) {
    case IDLE:
      if (elapsedTime >= delayStartSec * 1000) {
        currentState = RUNNING;
      }
      break;

    case RUNNING:
      if (obstacleDetected()) {
        avoidStartTime = millis();
        avoidStep = 0;
        currentState = AVOID_OBSTACLE;
      } else if (motOff || elapsedTime >= maxTime) {
        currentState = STOPPED;
      } else {
        moveRobot();
      }
      break;

    case AVOID_OBSTACLE:
      avoidObstacle();
      break;

    case REALIGN_LINE:
      realignToLine();
      break;

    case STOPPED:
      stopMotors();
      break;
  }
}

// ================================================================
//                           Fonctions principales
// ================================================================

// Vérifie si un obstacle est détecté par les capteurs à ultrasons ou le capteur frontal
bool obstacleDetected() {
  return (digitalRead(CPT_VOID) || 
          (obstacleRightDistance <= obstacleThreshold && obstacleRightDistance != 0) ||
          (obstacleLeftDistance <= obstacleThreshold && obstacleLeftDistance != 0));
}

// Fait avancer le robot en fonction des capteurs de ligne
void moveRobot() {
  bool left = digitalRead(CPT_LEFT);   
  bool right = digitalRead(CPT_RIGHT);

  if (!left && right) {
    motorLeft->run(RELEASE);
    motorRight->setSpeed(speed);
    motorRight->run(FORWARD);
  } else if (left && !right) {
    motorRight->run(RELEASE);
    motorLeft->setSpeed(speed);
    motorLeft->run(FORWARD);
  } else {
    motorLeft->setSpeed(speed);
    motorRight->setSpeed(speed);
    motorLeft->run(FORWARD);
    motorRight->run(FORWARD);
  }
}

// Arrête les deux moteurs
void stopMotors() {
  motorLeft->run(RELEASE);
  motorRight->run(RELEASE);
}

// Effectue le contournement d’un obstacle en 4 étapes
void avoidObstacle() {
  unsigned long elapsed = millis() - avoidStartTime;

  switch (avoidStep) {
    case 0:
      Serial.println("Obstacle détecté - Pause");
      stopMotors();
      if (elapsed >= stopTime) {
        avoidStep = 1;
        avoidStartTime = millis();
      }
      break;

    case 1:
      Serial.println("Tourner à gauche (90°)");
      motorLeft->run(BACKWARD);
      motorRight->run(FORWARD);
      if (elapsed >= turnLeftTime) {
        avoidStep = 2;
        avoidStartTime = millis();
      }
      break;

    case 2:
      Serial.println("Avancer droit");
      motorLeft->run(FORWARD);
      motorRight->run(FORWARD);
      if (elapsed >= forwardTime) {
        avoidStep = 3;
        avoidStartTime = millis();
      }
      break;

    case 3:
      Serial.println("Tourner à droite (90°)");
      motorLeft->run(FORWARD);
      motorRight->run(BACKWARD);
      if (elapsed >= turnRightTime) {
        currentState = REALIGN_LINE;
        avoidStartTime = millis();
      }
      break;
  }
}

// Recherche les lignes blanches après avoir contourné un obstacle
void realignToLine() {
  unsigned long elapsed = millis() - avoidStartTime;
  Serial.println("Recherche de la ligne...");
  moveRobot(); 

  if (elapsed >= realignTime || digitalRead(CPT_LEFT) || digitalRead(CPT_RIGHT)) {
    Serial.println("Ligne retrouvée !");
    currentState = RUNNING;
  }
}

// Effectue une mesure de distance avec un capteur ultrasonique (en cm)
float readUltrasonic(int trigPin, int echoPin){
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  long duration = pulseIn(echoPin, HIGH, 4000);
  return (duration * .0343) / 2; 
}
