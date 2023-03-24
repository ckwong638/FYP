int outputPin = 4;
int monitorPin = A1;
float pressureSetpoint; 
float voltageValue;
float pressureValue;

void setup() {
  // put your setup code here, to run once:
  pinMode(outputPin, OUTPUT);
  pinMode(monitorPin, INPUT); 
  Serial.begin(9600);

}

void loop() {
  // put your main code here, to run repeatedly:
  analogWrite(outputPin, 20);
  delay(6000);
  analogWrite(outputPin, 30);
  delay(6000);
  analogWrite(outputPin, 35);
  delay(6000);
  
  // Serial.print("Enter pressure setpoint in MPa (0.005-0.1): ");
  
  // if (Serial.available() > 0) {
  //   pressureSetpoint = Serial.parseFloat();

  //   // if (Serial.read() == '\n');

  // }
  // // pressureSetpoint = Serial.parseFloat();
  // Serial.println(pressureSetpoint);

  // int pwmValue = map(pressureSetpoint, 0.005, 0.1, 0, 255);
  // analogWrite(outputPin, pwmValue);
  // voltageValue = (analogRead(monitorPin) / 1023.0) * 4.0 + 1.0;
  // pressureValue = (voltageValue - 1.0) * 20.0;
  // Serial.print("Setpoint: ");
  // Serial.print(pressureSetpoint);
  // Serial.print(" MPa, Pressure: ");
  // Serial.print(pressureValue);
  // Serial.print(" MPa, Voltage: ");
  // Serial.print(voltageValue);
  // Serial.println(" V");

  // delay(100); 





}
