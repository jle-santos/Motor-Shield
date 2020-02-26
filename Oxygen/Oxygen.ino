#define O2_SENSOR_PIN A0
#define VREF  5

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200); 
}

void loop() {
  // put your main code here, to run repeatedly:

  Serial.print("Vout = ");
  Serial.print(readO2Vout(O2_SENSOR_PIN));
  Serial.print(" V, Concentration of O2 is ");
  Serial.println(readConcentration(O2_SENSOR_PIN));
  delay(1000);
}

float readO2Vout(uint8_t analogPin) {
  // Vout samples are with reference to 5V
  float MeasuredVout = (analogRead(analogPin) * (VREF / 1023.0)) - 0.06;
  return MeasuredVout;
}

float readConcentration(uint8_t analogPin) {
  float o2_concentrationInPercent;

  // Equation derived from linear relationship between O2 and voltage out 
  o2_concentrationInPercent = readO2Vout(analogPin) * 11.0097;

  return o2_concentrationInPercent; 
}
