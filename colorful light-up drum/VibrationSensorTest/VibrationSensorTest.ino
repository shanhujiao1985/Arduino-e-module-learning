int sensorPin=A2;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200); // Initialize serial communication
}

void loop() {
   int sensorValue = analogRead(sensorPin); // Read the value from analog pin A2
  Serial.println(sensorValue);
  delay(50); // Delay for 20 milliseconds
}
