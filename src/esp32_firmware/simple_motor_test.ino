// =================================================================
//  Simple X-Motor Test (ohne AccelStepper)
// =================================================================
//
//  Einfacher Test für X-Achse Motor:
//  - Sendet direkte STEP/DIR Pulse
//  - Nutzt die gleichen Pins wie das Hauptprogramm
//  - Bewegt Motor in beide Richtungen
//  - Serial Output für Debugging

// X-Achse Pins (aus dem Hauptprogramm)
const int X_STEP_PIN = 23;
const int X_DIR_PIN = 22;
const int X_ENA_PIN = 19;

// Test-Parameter
const int STEPS_TO_MOVE = 1000;      // Anzahl Schritte pro Bewegung
const int PULSE_WIDTH_US = 5;        // STEP-Puls Breite in Mikrosekunden
const int STEP_DELAY_US = 1000;      // Pause zwischen Schritten (1ms = 1000 steps/s)

void setup() {
  Serial.begin(115200);
  Serial.println("=== Simple X-Motor Test ===");
  
  // Pins als Output konfigurieren
  pinMode(X_STEP_PIN, OUTPUT);
  pinMode(X_DIR_PIN, OUTPUT);
  pinMode(X_ENA_PIN, OUTPUT);
  
  // Motor aktivieren (LOW = Enable)
  digitalWrite(X_ENA_PIN, LOW);
  Serial.println("Motor enabled (ENA = LOW)");
  
  // Startposition
  digitalWrite(X_STEP_PIN, LOW);
  digitalWrite(X_DIR_PIN, LOW);
  
  Serial.println("Warte 2 Sekunden...");
  delay(2000);
  
  Serial.println("Test startet!");
}

void loop() {
  // Test 1: Vorwärts bewegen
  Serial.print("Bewege ");
  Serial.print(STEPS_TO_MOVE);
  Serial.println(" Schritte VORWÄRTS...");
  
  digitalWrite(X_DIR_PIN, LOW);  // Richtung: Vorwärts
  delay(1);  // DIR Setup Zeit
  
  moveSteps(STEPS_TO_MOVE);
  
  Serial.println("Pause 3 Sekunden...");
  delay(3000);
  
  // Test 2: Rückwärts bewegen
  Serial.print("Bewege ");
  Serial.print(STEPS_TO_MOVE);
  Serial.println(" Schritte RÜCKWÄRTS...");
  
  digitalWrite(X_DIR_PIN, HIGH); // Richtung: Rückwärts
  delay(1);  // DIR Setup Zeit
  
  moveSteps(STEPS_TO_MOVE);
  
  Serial.println("Pause 5 Sekunden...");
  delay(5000);
  
  Serial.println("--- Nächster Zyklus ---");
}

void moveSteps(int steps) {
  for (int i = 0; i < steps; i++) {
    // STEP-Puls erzeugen
    digitalWrite(X_STEP_PIN, HIGH);
    delayMicroseconds(PULSE_WIDTH_US);
    digitalWrite(X_STEP_PIN, LOW);
    delayMicroseconds(STEP_DELAY_US);
    
    // Fortschritt alle 100 Schritte
    if ((i + 1) % 100 == 0) {
      Serial.print("Schritt ");
      Serial.print(i + 1);
      Serial.print("/");
      Serial.println(steps);
    }
  }
  Serial.println("Bewegung abgeschlossen.");
}
