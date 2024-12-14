#include <Servo.h>  // Εισάγουμε τη βιβλιοθήκη για το Servo

// Ορίζουμε τις θύρες για το servo και το μοτέρ
const int servoPin = 9;      // Θύρα του servo
const int motorPin = 6;      // Θύρα του μοτέρ (PWM pin)
const int motorDirPin = 7;   // Θύρα κατεύθυνσης μοτέρ (για να ελέγξουμε την κατεύθυνση)

// Δημιουργούμε το αντικείμενο για το servo
Servo myServo;

void setup() {
  // Αρχικοποιούμε τη σειριακή επικοινωνία για να μπορούμε να διαβάζουμε δεδομένα από τον υπολογιστή
  Serial.begin(9600);

  // Ρυθμίζουμε το servo
  myServo.attach(servoPin);

  // Ορίζουμε τις θύρες του μοτέρ ως εξόδους
  pinMode(motorPin, OUTPUT);
  pinMode(motorDirPin, OUTPUT);

  // Μήνυμα για να ενημερώνει ότι το σύστημα είναι έτοιμο
  Serial.println("Arduino Serial Command Receiver Ready!");
}

void loop() {
  // Ελέγχουμε αν υπάρχουν δεδομένα στη σειριακή θύρα
  if (Serial.available() > 0) {
    // Διαβάζουμε ολόκληρη την εισερχόμενη συμβολοσειρά
    String input = Serial.readString();

    // Αφαιρούμε το τέλος της συμβολοσειράς (το "\n" που μπορεί να έχει προστεθεί)
    input.trim();

    // Εμφανίζουμε τη συμβολοσειρά που διαβάστηκε για debugging
    Serial.print("Received string: ");
    Serial.println(input);

    // Αν η συμβολοσειρά περιέχει ':' (δηλαδή έχει δύο εντολές)
    if (input.indexOf(':') != -1) {
      // Διαχωρίζουμε την είσοδο στα δύο μέρη: για το Servo και το μοτέρ
      int colonIndex = input.indexOf(':');
      String servoCommand = input.substring(1, colonIndex);  // Πρώτο τμήμα για το Servo
      String motorCommand = input.substring(colonIndex + 1, input.length() - 1);  // Δεύτερο τμήμα για το μοτέρ

      // Εμφανίζουμε τα τμήματα για debugging
      Serial.print("Servo Command: ");
      Serial.println(servoCommand);
      Serial.print("Motor Command: ");
      Serial.println(motorCommand);

      // Επεξεργασία της πρώτης εντολής (Servo)
      int servoInput = servoCommand.toInt();  // Μετατροπή της τιμής για το Servo σε ακέραιο

      // Υπολογίζουμε την γωνία του servo (αν η τιμή 50 να είναι στη μέση, 0-100 να χαρτογραφείται σε 0-180 μοίρες)
      int servoAngle = map(servoInput, 0, 100, 0, 180);  // Μετατροπή τιμής 0-100 σε γωνία 0-180

      // Ρυθμίζουμε τη γωνία του servo
      myServo.write(servoAngle);  
      Serial.print("Servo angle set to: ");
      Serial.println(servoAngle);

      // Επεξεργασία της δεύτερης εντολής (Motor)
      int motorInput = motorCommand.toInt();  // Μετατροπή της τιμής για το μοτέρ σε ακέραιο

      // Υπολογίζουμε την ταχύτητα του μοτέρ και την κατεύθυνση
      int motorSpeed = map(motorInput, 0, 100, -255, 255);  // Μετατροπή τιμής 0-100 σε ταχύτητα PWM -255 έως 255

      if (motorSpeed < 0) {
        // Αν η τιμή είναι μικρότερη από 0, το μοτέρ περιστρέφεται αντίστροφα
        digitalWrite(motorDirPin, LOW);  // Θέτουμε την κατεύθυνση του μοτέρ στο αντίθετο
        analogWrite(motorPin, abs(motorSpeed));  // Ρυθμίζουμε την ταχύτητα με την απόλυτη τιμή
      } else {
        // Αν η τιμή είναι μεγαλύτερη από 0, το μοτέρ περιστρέφεται κανονικά
        digitalWrite(motorDirPin, HIGH);  // Θέτουμε την κατεύθυνση του μοτέρ στη σωστή κατεύθυνση
        analogWrite(motorPin, motorSpeed);  // Ρυθμίζουμε την ταχύτητα
      }

      Serial.print("Motor speed set to: ");
      Serial.println(motorSpeed);
    } else {
      Serial.println("Invalid format. Please use #(0):(100)# format.");
    }
  }
}
