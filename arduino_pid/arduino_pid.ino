// Настройки перед подключением библиотеки
// задать задержку переключения CLK в микросекундах для улучшения связи по длинным проводам
// (для GyverMAX6675)
//#define MAX6675_DELAY 10

// задать скорость SPI в Гц (умолч. 1000000 - 1 МГц) для улучшения связи по длинным проводам
// (для GyverMAX6675_SPI)
//#define MAX6675_SPI_SPEED 300000

#define MCP4725_ADDRESS 0b01100000      //адрес I2C модуля, 01100 — зарезервировано, далее А2, А1, А0 
#define ADC_PIN A0
#define COEFFICIENT 1
#define TARGET 37.0f //значение уставки в попугаях
#define KP 12 //коэффициент P
#define KI 0.4f //коэффициент I
#define KD 60 //коэффициент D
#define DT 50 //период вычисления и регулирования

#include <GyverMAX6675_SPI.h>

#include <Wire.h>


GyverMAX6675_SPI<10> sens;                  // аппаратный SPI

//Использование

//int getTempInt();   // Получить температуру int


void setVoltage(float voltage);

// (вход, установка, п, и, д, период в секундах, мин.выход, макс. выход)
int computePID(float input, float setpoint, float kp, float ki, float kd, float dt, int minOut, int maxOut);

float voltage = 0;  // сюда пишем управляющее напряжение
float temp; // сюда пишем температуру

void setup() {
  Serial.begin(9600);
  Wire.begin();  //стартуем I2C
//  pinMode(ADC_PIN, INPUT);
}

//основной цикл
void loop() {
  if (sens.readTemp()) // Запросить температуру (вернёт true если успешно)
  {
    temp = sens.getTemp();    // Получить температуру float)
  } //else Serial.println("Read temp error");
  voltage = computePID(temp, TARGET, KP, KI, KD, DT, 0, 4.99);
  setVoltage(voltage);                            //устанавливаем напряжение на выходе ЦАП
//  Serial.print("target: ");
  Serial.println(TARGET);
//  Serial.print("Temp: ");
  Serial.println(temp, 2);
//  Serial.print("Voltage: ");
  Serial.println(voltage, 2);
//  Serial.println(" V");
  delay(1000);
}

// (вход, установка, п, и, д, период в секундах, мин.выход, макс. выход)
int computePID(float input, float setpoint, float kp, float ki, float kd, float dt, int minOut, int maxOut) {
  float err = setpoint - input;
  static float integral = 0, prevErr = 0;
  integral = constrain(integral + (float)err * dt * ki, minOut, maxOut);
  float D = (err - prevErr) / dt;
  prevErr = err;
  return constrain(err * kp + integral + D * kd, minOut, maxOut);
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
