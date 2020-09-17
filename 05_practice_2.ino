#define PIN_LED 7
unsigned int toggle;

void setup() {
  pinMode(PIN_LED, OUTPUT);
  toggle = 1;
}

void loop() {
  digitalWrite(PIN_LED, 0);
  delay(1000);
  for (int i=0; i<=10; i++){
    digitalWrite(PIN_LED, toggle);
    delay(100);
    toggle = 1 - toggle;
  }
  while(1);
}
