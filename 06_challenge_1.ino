#define PIN_LED 7

int p; // p : period ( 100 to 10000, unit : us), 1 ms = 1000 us

void set_period(int period) {
  p = period;
}

void set_duty(int duty) { // duty : 0 to 100 (unit : %)
  int new_p = p / 100;
  digitalWrite(PIN_LED, 0);
  delayMicroseconds(p * duty);
  digitalWrite(PIN_LED, 1);
  delayMicroseconds(new_p * (100 - duty));
}

void setup() {
  pinMode(PIN_LED, OUTPUT);
  Serial.begin(115200);
}

void loop() {
  int running_time = 1000;

  //set_period(100);
  //set_period(1000);
  set_period(10000);

  int d = 0;
  int i = 1;
  unsigned long ts = millis();

  if (p == 10000) {
    running_time = 2000;
  }

  while (millis() - ts <= running_time) {
    set_duty(d);
    d += i;
    if (d <= 0 or d >= 100) {
      i = -i;
    }
  }
  digitalWrite(PIN_LED, 1);
  while (1);


}
