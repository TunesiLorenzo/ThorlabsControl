const int analogPin = A0;
const uint8_t ALIGN_BYTE = 0xA5;

void setup() {
  Serial.begin(460800);

  // ADC prescaler = 32
  ADCSRA = (ADCSRA & 0b11111000) | 0b101;
}

void loop() {
  uint16_t value = analogRead(analogPin);

  Serial.write(ALIGN_BYTE);                  // alignment byte
  Serial.write((uint8_t)(value & 0xFF));     // low byte
  Serial.write((uint8_t)(value >> 8));       // high byte
}