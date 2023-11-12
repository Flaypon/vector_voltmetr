//Скетч вывода напряжения в последовательный порт

#define ADC_PIN A0

void setup() {
  pinMode(ADC_PIN, INPUT);
  Serial.begin(9600);
}
 
void loop() {
  float val = analogRead(ADC_PIN);
  val = 5*val/1023;                 // перевод в вольты, раскоментировать для вывода в вольтах
  //Serial.println(val,2);          // вывод в порт, второй аргумент — количесвто знаков после запятой
  delay(500);
}
