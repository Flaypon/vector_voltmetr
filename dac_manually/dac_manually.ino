//Скетч ручного управления ЦАП. Для установки напряжения необходимо отправить в порт значение в формате float
#include <Wire.h>

#define MCP4725_ADDRESS 0b01100001      //адрес I2C модуля, 01100 — зарезервировано, далее А2, А1, А0 
#define ADC_PIN A0
#define COEFFICIENT 1

void setVoltage(float);

float voltage = 0;  // сюда пишем напряжение

void setup() {
  Serial.begin(9600);
  Wire.begin();  //стартуем I2C
  pinMode(ADC_PIN, INPUT);
}

void loop() {
  if (Serial.available())         //проверяем порт на наличие данных
  {
    voltage = Serial.parseFloat(); // парсим float
    voltage = constrain(voltage, 0, 5); //ограничиваем от 0 до 5 на случай неправильного ввода
    Serial.print("Новое значение напряжения: ");
    Serial.print(voltage, 3);
    Serial.println("V");
  }
  float temp = analogRead(ADC_PIN) * COEFFICIENT; //пересчитываем значение напряжение в температуру, коэффициент вычислить и прописать в define
  setVoltage(voltage);                            //устанавливаем напряжение на выходе ЦАП
  Serial.print("Temp: ");
  Serial.println(temp, 1);
  Serial.print("Voltage: ");
  Serial.print(voltage, 3);
  Serial.println(" V");
  delay(1000);
}

void setVoltage(float voltage)
{
  uint8_t buffer[3];
  buffer[0] = 0b01000000;            //записываем в buffer0 контрольный байт (010-Sets in Write mode)
  int bits = (4096.0 * voltage) / 5;  //формула для расчета значения напряжения (A0)
  buffer[1] = bits >> 4;              //записываем наиболее значимые биты
  buffer[2] = bits << 4;              //записываем наименее значимые биты
  Wire.beginTransmission(MCP4725_ADDRESS);         // инициалзируем передачу о указанному адресу
  Wire.write(buffer[0]);                   // передаем контрольный байт с помощью протокола I2C 
  Wire.write(buffer[1]);                   // передаем наиболее значимые биты с помощью протокола I2C 
  Wire.write(buffer[2]);                   // передаем наименее значимые биты с помощью протокола I2C
  Wire.endTransmission();                  //окончание передачи
}
