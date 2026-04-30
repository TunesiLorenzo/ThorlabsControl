const int pinAnalog = A0;

const float VREF = 3.3;   // Référence externe
const int ADC_MAX = 1023;

void setup() {
  analogReference(EXTERNAL); // utilise AREF
  Serial.begin(115200);
}

void loop() {
  int raw = analogRead(pinAnalog);

  float voltage = raw * (VREF / ADC_MAX);

  Serial.println(voltage, 5); // meilleure précision
  
  delay(1); // ~100 Hz
}