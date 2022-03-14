int vibNum = 6;


void setup() {
  // put your setup code here, to run once:
  pinMode(vibNum, OUTPUT);
  pinMode(LED_BUILTIN, OUTPUT);
}

void loop() {
  // put your main code here, to run repeatedly:
  analogWrite(vibNum, 102);
  delay(1000);                       // wait for a second
  analogWrite(vibNum, 150);
  delay(1000);
  analogWrite(vibNum, 202);
  delay(1000); 
  analogWrite(vibNum, 0);
  delay(1000); 
}
