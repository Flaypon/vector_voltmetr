#define PT100 //выбираем датчик температуры PT100, NTC или нормализатор MAX6675
// Настройки перед подключением библиотеки
// задать задержку переключения CLK в микросекундах для улучшения связи по длинным проводам
// (для GyverMAX6675)
//#define MAX6675_DELAY 10

// задать скорость SPI в Гц (умолч. 1000000 - 1 МГц) для улучшения связи по длинным проводам
// (для GyverMAX6675_SPI)
//#define MAX6675_SPI_SPEED 300000

#include <GyverPID.h>
#include <GyverMAX6675_SPI.h>
#include <GyverNTC.h>
#include <Adafruit_MAX31865.h>
#include <Wire.h>

#define RREF 430.0 // The value of the Rref resistor. Use 430.0 for PT100 and 4300.0 for PT1000 (Adafruit_MAX31865)
#define RNOMINAL  100.0 // The 'nominal' 0-degrees-C resistance of the sensor (100.0 for PT100, 1000.0 for PT1000)
#define ADC_PIN A0 //сигнал термистора
#define R1_TERMISTOR 10000 //значение R1 на модуле NTC
#define R_TERMISTOR 10000 //сопротивление NTC
#define MCP4725_COLD 0b01100000      //адрес I2C модуля, 01100 — зарезервировано, далее А2, А1, А0 
#define MCP4725_HOT 0b01100001      //адрес I2C модуля, 01100 — зарезервировано, далее А2, А1, А0 
#define MAX_OUT 4.99f
#define TARGET 37.0f //значение уставки в попугаях
#define KP 12 //коэффициент P
#define KI 0.4f //коэффициент I
#define KD 60 //коэффициент D
#define DT 500 //период вычисления и регулирования

// pin - аналоговый пин, к которому подключен термистор
// R - сопротивление балластного резистора, Ом
// B - бета-коэффициент термистора (см. даташит) [число в районе 1000-5000]
// t - базовая температура термистора, градусов Цельсия (см. даташит) [обычно 25 градусов]
// Rt - сопротивление термистора при базовой температуре, Ом (см. даташит)
// res - разрешение АЦП, бит. По умолчанию 10
GyverNTC therm(0, R1_TERMISTOR, 3435, 25, R_TERMISTOR, 10); // коэффициент подобрать
GyverMAX6675_SPI<10> sens;                  // аппаратный SPI
GyverPID regulator(KP, KI, KD, DT); // инициализировать с коэффициентами и dt (в миллисекундах)
Adafruit_MAX31865 thermo = Adafruit_MAX31865(9); // use hardware SPI, just pass in the CS pin

double read_temp_termistor();
void setVoltage(float voltage, int); //функция установки напряжения на ЦАП

// (вход, установка, п, и, д, период в секундах, мин.выход, макс. выход)
int computePID(float input, float setpoint, float kp, float ki, float kd, float dt, int minOut, int maxOut);

float voltage = 0;  // сюда пишем управляющее напряжение
double temp; // сюда пишем температуру

void setup() {
  Serial.begin(9600);
  Wire.begin();  //стартуем I2C
  thermo.begin(MAX31865_3WIRE);  // set to 2WIRE or 4WIRE as necessary
  pinMode(ADC_PIN, INPUT);
  regulator.setDirection(NORMAL); // направление регулирования (NORMAL/REVERSE). ПО УМОЛЧАНИЮ СТОИТ NORMAL
  regulator.setLimits(0, 4.99);    // пределы (ставим для 8 битного ШИМ). ПО УМОЛЧАНИЮ СТОЯТ 0 И 255
  regulator.setpoint = TARGET;        // сообщаем регулятору температуру, которую он должен поддерживать
}

//основной цикл
void loop() {
  #ifdef MAX6675 //если хотим нормализатор MAX6675
  if (sens.readTemp()) // Запросить температуру (вернёт true если успешно)
  {
    temp = sens.getTemp();    // Получить температуру float)
  } else Serial.println("Read temp error");
  #endif
  #ifdef PT100 //если хотим PT100
  if (thermo.readFault()) {
    Serial.print("Fault 0x"); Serial.println(fault, HEX);
    if (fault & MAX31865_FAULT_HIGHTHRESH) {
      Serial.println("RTD High Threshold"); 
    }
    if (fault & MAX31865_FAULT_LOWTHRESH) {
      Serial.println("RTD Low Threshold"); 
    }
    if (fault & MAX31865_FAULT_REFINLOW) {
      Serial.println("REFIN- > 0.85 x Bias"); 
    }
    if (fault & MAX31865_FAULT_REFINHIGH) {
      Serial.println("REFIN- < 0.85 x Bias - FORCE- open"); 
    }
    if (fault & MAX31865_FAULT_RTDINLOW) {
      Serial.println("RTDIN- < 0.85 x Bias - FORCE- open"); 
    }
    if (fault & MAX31865_FAULT_OVUV) {
      Serial.println("Under/Over voltage"); 
    }
    thermo.clearFault();
  }
  else 
  {
    term = thermo.temperature(RNOMINAL, RREF);
    // uint16_t rtd = thermo.readRTD(); //персчёт величины термосопротивления
  }voltage = computePID(temp, TARGET, KP, KI, KD, DT, 0, MAX_OUT);
  #endif
  #ifdef NTC //если хотим NTC
  temp = therm.getTempAverage();
  //temp = therm.getTemp();
  #endif
  // voltage = computePID(temp, TARGET, KP, KI, KD, DT, 0, MAX_OUT);
  voltage = regulator.getResult();   // возвращает новое значение при вызове (если используем свой таймер с периодом dt!)
  // voltage = regulator.getResultTimer(); // возвращает новое значение не ранее, чем через dt миллисекунд (встроенный таймер с периодом dt)
  setVoltage(voltage, MCP4725_HOT);                            //устанавливаем напряжение на выходе ЦАП
  setVoltage(MAX_OUT - voltage, MCP4725_COLD);                            //устанавливаем напряжение на выходе ЦАП
  // Serial.print("target: ");
  // Serial.println(TARGET);
  // Serial.print("Temp: ");
  // Serial.println(temp, 2);
  // Serial.print("Voltage: ");
  // Serial.println(voltage, 2);
  // Serial.println(" V");
  delay(DT);
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

void setVoltage(float voltage, int address)
{
  uint8_t buffer[3];
  buffer[0] = 0b01000000;            //записываем в buffer0 контрольный байт (010-Sets in Write mode)
  int bits = (4096.0 * voltage) / 5;  //формула для расчета значения напряжения (A0)
  buffer[1] = bits >> 4;              //записываем наиболее значимые биты
  buffer[2] = bits << 4;              //записываем наименее значимые биты
  Wire.beginTransmission(address);         // инициалзируем передачу о указанному адресу
  Wire.write(buffer[0]);                   // передаем контрольный байт с помощью протокола I2C
  Wire.write(buffer[1]);                   // передаем наиболее значимые биты с помощью протокола I2C
  Wire.write(buffer[2]);                   // передаем наименее значимые биты с помощью протокола I2C
  Wire.endTransmission();                  //окончание передачи
}

double read_temp_termistor()
{
  float logR2, R2;
  double temp;
  float R1 = R1_TERMISTOR; // значение R1 на модуле
  double c1 = 0.001129148, c2 = 0.000234125, c3 = 0.0000000876741; //коэффициенты Штейнхарта-Харта для термистора
  int val = analogRead(ADC_PIN);
  R2 = R1 * (1023.0 / (float)val - 1.0); //вычислите сопротивление на термисторе
  logR2 = log(R2);
  temp = (1.0 / (c1 + c2*logR2 + c3*logR2*logR2*logR2)) - 273.15; // температура в Кельвине
  temp = val;
  return temp; 
}