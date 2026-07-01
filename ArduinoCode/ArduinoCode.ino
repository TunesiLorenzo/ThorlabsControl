const int analogPin = A0;

void setup() {
  Serial.begin(115200);

  // ADC prescaler = 32
  ADCSRA = (ADCSRA & 0b11111000) | 0b101;
}

void loop() {
  uint16_t adcValue = analogRead(analogPin);

  float voltage = adcValue;

  Serial.println(voltage);  // print voltage with 3 decimal places
}