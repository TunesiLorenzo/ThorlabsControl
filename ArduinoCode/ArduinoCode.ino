const int analogPin = A0;

void setup() {
  Serial.begin(230400);

  // ADC prescaler = 32
  ADCSRA = (ADCSRA & 0b11111000) | 0b101;
}

void loop() {
  uint16_t adcValue = analogRead(analogPin);

  // Convert 10-bit ADC, 0-1023, to 8-bit, 0-255
  uint8_t sample = adcValue >> 2;

  Serial.write(sample);
}