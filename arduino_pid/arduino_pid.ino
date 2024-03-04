#define DEBUG 1

#include <GyverEncoder.h>

#include <Adafruit_ADS1X15.h>

#include <GyverPID.h>
#include <Adafruit_MAX31865.h>
#include <Wire.h>

#define CURRENT_RESISTANCE 0.02 // сопротивлене шунта [Ом]
#define LED_PIN 5 //номер пина со светодиодом
#define MCP4725_COLD 0b01100000  //адрес I2C модуля, 01100 — зарезервировано, далее А2, А1, А0
#define MCP4725_HOT 0b01100001   //адрес I2C модуля, 01100 — зарезервировано, далее А2, А1, А0
#define MAX_OUT_VOLTS 4.99f
#define MAX_OUT_WATTS 15.0f
#define LOW_LIMIT_TEMP 15.0f
#define HIGH_LIMIT_TEMP 40.0F
#define TARGET 37.0f  //значение уставки в попугаях
#define KP 12         //коэффициент P
#define KI 0.4f       //коэффициент I
#define KD 60         //коэффициент D
#define DT 500        //период вычисления и регулирования

// The value of the Rref resistor. Use 430.0 for PT100 and 4300.0 for PT1000
#define RREF      430.0
// The 'nominal' 0-degrees-C resistance of the sensor
// 100.0 for PT100, 1000.0 for PT1000
#define RNOMINAL  100.0

Encoder enc(4, 3, 2, TYPE2);                      // для работы c кнопкой и сразу выбираем тип
Adafruit_ADS1115 ads_voltage;                     // Создание объекта измерителя напряжения для общения через него с модулем
Adafruit_ADS1115 ads_current;                     // Создание объекта измерителя тока для общения через него с модулем
GyverPID regulator(KP, KI, KD, DT);               // инициализировать с коэффициентами и dt (в миллисекундах)
Adafruit_MAX31865 thermo = Adafruit_MAX31865(4);  // use hardware SPI, just pass in the CS pin

void setVoltage(float, int);  //функция установки напряжения на ЦАП
void setWatts(float, int);   //функция установки мощности на пельтье

// (вход, установка, п, и, д, период в секундах, мин.выход, макс. выход)
int computePID(float input, float setpoint, float kp, float ki, float kd, float dt, int minOut, int maxOut);

float watts = 0;  // сюда пишем управляющее напряжение
float temp;      // сюда пишем текущую температуру
float target = LOW_LIMIT_TEMP; //уставка
float adc_voltage_1ch; //дифференциальный измеритель напряжения 1 канал (В)
float adc_voltage_2ch; //дифференциальный измеритель напряжения 2 канал (В)
float adc_current_1ch; //дифференциальный измеритель тока 1 канал (А)
float adc_current_2ch; //дифференциальный измеритель тока 2 канал (А)

void setup() {
  Serial.begin(9600);
  Wire.begin();                  //стартуем I2C
  thermo.begin(MAX31865_3WIRE);  // set to 2WIRE or 4WIRE as necessary
  pinMode(LED_PIN, OUTPUT);
  regulator.setDirection(NORMAL);       // направление регулирования (NORMAL/REVERSE). ПО УМОЛЧАНИЮ СТОИТ NORMAL
  regulator.setLimits(0, 1024);         // пределы (ставим для 8 битного ШИМ). ПО УМОЛЧАНИЮ СТОЯТ 0 И 255
  regulator.setpoint = TARGET;          // сообщаем регулятору температуру, которую он должен поддерживать
  
  ads_voltage.setGain(GAIN_TWOTHIRDS);  // усиление измерителя напряжения 2/3
  ads_current.setGain(GAIN_EIGHT);      // усиление измерителя тока 8
  // ВОЗМОЖНЫЕ ВАРИАНТЫ УСТАНОВКИ КУ:
  // ads.setGain(GAIN_TWOTHIRDS); | 2/3х | +/-6.144V | 1bit = 0.1875mV    |
  // ads.setGain(GAIN_ONE);       | 1х   | +/-4.096V | 1bit = 0.125mV     |
  // ads.setGain(GAIN_TWO);       | 2х   | +/-2.048V | 1bit = 0.0625mV    |
  // ads.setGain(GAIN_FOUR);      | 4х   | +/-1.024V | 1bit = 0.03125mV   |
  // ads.setGain(GAIN_EIGHT);     | 8х   | +/-0.512V | 1bit = 0.015625mV  |
  // ads.setGain(GAIN_SIXTEEN);   | 16х  | +/-0.256V | 1bit = 0.0078125mV |
  ads_voltage.begin(0x49); // Инициализация модуля измерителя напряжения
  ads_current.begin(0x48); // Инициализация модуля измерителя тока
  digitalWrite(LED_PIN, 0);

  start_set_target();
  Serial.println("target, temp");
}

//основной цикл
void loop() {
  //----------------------------------------------------------------------------
  if (enc.isRight()) {
      target += 0.1;
      target = constrain(target, LOW_LIMIT_TEMP, HIGH_LIMIT_TEMP);
#ifdef DEBUG
      Serial.print("Значение уставки: ");
      Serial.println(target, 2);
#endif
    }
    if (enc.isLeft()) {
      target -= 0.1;
      target = constrain(target, LOW_LIMIT_TEMP, HIGH_LIMIT_TEMP);
#ifdef DEBUG
      Serial.print("Значение уставки: ");
      Serial.println(target, 2);
#endif
    }
    if (enc.isFastR()) {
      target += 1.0;
      target = constrain(target, LOW_LIMIT_TEMP, HIGH_LIMIT_TEMP);
#ifndef DEBUG      
      Serial.print("Значение уставки: ");
      Serial.println(target, 2);
#endif
    }
    if (enc.isFastL()) {
      target -= 1.0;
      target = constrain(target, LOW_LIMIT_TEMP, HIGH_LIMIT_TEMP);
#ifdef DEBUG         
      Serial.print("Значение уставки: ");
      Serial.println(target, 2);
#endif
    }
  //----------------------------------------------------------------------------
  uint8_t fault = thermo.readFault();
  if (fault) 
  {
#ifdef DEBUG
    Serial.println("Ошибка чтения температуры");
#endif
    ;
  }
  else temp = thermo.temperature(RNOMINAL, RREF);

  //зажигаем светодиод уставки
  if (abs(TARGET - temp) <= 1) {
    digitalWrite(LED_PIN, 1);
  } else {
    digitalWrite(LED_PIN, 0);
  }
  //-----------------------------основной алгоритм-----------------------------
  // voltage = computePID(temp, TARGET, KP, KI, KD, DT, 0, MAX_OUT);
  // voltage = regulator.getResult();   // возвращает новое значение при вызове (если используем свой таймер с периодом dt!)
  adc_voltage_1ch = ads_voltage.readADC_Differential_0_1() * 0.1875 / 1000;    // получаем напряжение (дифф.) с первого канала измерителя напряжения
  adc_voltage_2ch = ads_voltage.readADC_Differential_2_3() * 0.1875 / 1000;    // получаем напряжение (дифф.) со второго канала измерителя напряжения
  adc_current_1ch = ads_current.readADC_Differential_0_1() * 0.015625 / 1000 / CURRENT_RESISTANCE;  // получаем ток (дифф.) с первого канала измерителя тока
  adc_current_2ch = ads_current.readADC_Differential_2_3() * 0.015625 / 1000 / CURRENT_RESISTANCE;  // получаем ток (дифф.) со второго канала измерителя тока
  watts = regulator.getResultTimer();                                                // возвращает новое значение не ранее, чем через dt миллисекунд (встроенный таймер с периодом dt)
 
  Serial.print(target, 2);
  Serial.print(",");
  Serial.println(temp, 2);
  // delay(DT);
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

void setVoltage(float voltage, int address) {
  uint8_t buffer[3];
  buffer[0] = 0b01000000;             //записываем в buffer0 контрольный байт (010-Sets in Write mode)
  int bits = (4096.0 * voltage) / 5;  //формула для расчета значения напряжения (A0)
  buffer[1] = bits >> 4;              //записываем наиболее значимые биты
  buffer[2] = bits << 4;              //записываем наименее значимые биты
  Wire.beginTransmission(address);    // инициалзируем передачу о указанному адресу
  Wire.write(buffer[0]);              // передаем контрольный байт с помощью протокола I2C
  Wire.write(buffer[1]);              // передаем наиболее значимые биты с помощью протокола I2C
  Wire.write(buffer[2]);              // передаем наименее значимые биты с помощью протокола I2C
  Wire.endTransmission();             //окончание передачи
}

void setWatts(float watts, int address) {
  float voltage;
  setVoltage(voltage, MCP4725_HOT);                                                  //устанавливаем напряжение на выходе ЦАП
  setVoltage(MAX_OUT_VOLTS - voltage, MCP4725_COLD);                                       //устанавливаем напряжение на выходе ЦАП
}

void start_set_target() {
  while (enc.isClick() == false) {
    if (enc.isRight()) {
      target += 0.1;
      target = constrain(target, LOW_LIMIT_TEMP, HIGH_LIMIT_TEMP);
      Serial.print("Значение уставки: ");
      Serial.println(target, 2);
    }
    if (enc.isLeft()) {
      target -= 0.1;
      target = constrain(target, LOW_LIMIT_TEMP, HIGH_LIMIT_TEMP);
      Serial.print("Значение уставки: ");
      Serial.println(target, 2);
    }
    if (enc.isFastR()) {
      target += 1.0;
      target = constrain(target, LOW_LIMIT_TEMP, HIGH_LIMIT_TEMP);
      Serial.print("Значение уставки: ");
      Serial.println(target, 2);
    }
    if (enc.isFastL()) {
      target -= 1.0;
      target = constrain(target, LOW_LIMIT_TEMP, HIGH_LIMIT_TEMP);
      Serial.print("Значение уставки: ");
      Serial.println(target, 2);
    }
  }
}