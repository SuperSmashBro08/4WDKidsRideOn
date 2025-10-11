const int LPWM = 15;
const int RPWM = 16;
//test
void setup() {
  pinMode(LPWM, OUTPUT);
  pinMode(RPWM, OUTPUT);
  
  digitalWrite(LPWM, LOW);   // Low side off
  digitalWrite(RPWM, HIGH);  // Force high
}

void loop() {}
