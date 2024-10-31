#include <Stepper.h>

const int stepPerRevolution = 200;
const int stepPin = 3;
const int dirPin = 2;

Stepper myStepper(stepPerRevolution, stepPin, dirPin);
int directionSpeed;
int handOpen;

void setup() {
  Serial.begin(115200);
  pinMode(stepPin, OUTPUT);
  pinMode(dirPin, OUTPUT);
}

void loop() {
  if (Serial.available() > 0) {
    String data = Serial.readStringUntil('x'); // Promijenjeno u karakter
    int commaIndex = data.indexOf(',');

    if (commaIndex > 0) { // Provjera ako postoji zarez
      String handOpenStr = data.substring(0, commaIndex);
      String directionSpeedStr = data.substring(commaIndex + 1);

      handOpen = handOpenStr.toInt();
      directionSpeed = directionSpeedStr.toInt();

      // Postavljanje brzine sa ograničenjem
      myStepper.setSpeed(min(100 * abs(directionSpeed), 1000)); // Postavljanje maksimalne brzine
    }
  }

  if (handOpen == 1) {
    if (directionSpeed > 0) {
      myStepper.step(stepPerRevolution / 10); // Manji broj koraka za stalno kretanje
    } else if (directionSpeed < 0) {
      myStepper.step(-stepPerRevolution / 10);
    }
  } else {
    // Kada handOpen nije aktivan, ne pokreći motor
    myStepper.step(0);
  }
}
