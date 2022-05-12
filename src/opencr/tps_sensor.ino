#define PORT4_SIG1          73
#define PORT4_SIG2          74
#define PORT4_ADC           75
#define OLLO_SLEEP          46

void setup() {
  Serial.begin(115200);

  pinMode(OLLO_SLEEP, OUTPUT);
  digitalWrite(OLLO_SLEEP, HIGH);

  pinMode(PORT4_ADC, INPUT_ANALOG);
  pinMode(PORT4_SIG1, OUTPUT);
  pinMode(PORT4_SIG2, OUTPUT);
}

void loop() {

  if (Serial.available() > 0) {
    String msg = Serial.readString();

    if (msg == "testing"){
      uint32_t analogValue;
      analogValue = analogRead(PORT4_ADC);
      word long vvalue = (4095 - analogValue) * 10000 / analogValue;
      Serial.println(float(vvalue / 1000.0));

    }
  }

}