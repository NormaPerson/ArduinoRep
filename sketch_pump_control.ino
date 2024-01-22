#include <Arduino.h>
#include <Wire.h>
#include "Adafruit_SHT31.h"
#include <EasyBuzzer.h>
#include <Wire.h> 
#include <LiquidCrystal_I2C.h>
#include <EEPROM.h>

#define PUMP_pin 7
#define VENTILATION_pin A3
#define PUMP_CHECK_pin 2
#define hose_diam_mm 19
#define imp_litters 50
#define BUZZER_pin 8
#define SOIL_MOISTURE_pin A6
#define ALARM_STOP_pin 3
#define ALERT_led 9
#define ADDR_I2C_SHT31 0x44
#define LUMINOSITY_pin A0
#define PUMP_OFF         digitalWrite(PUMP_pin, LOW) // выключение реле 1 
#define PUMP_ON          digitalWrite(PUMP_pin, HIGH) // включение реле 1
#define VENTILATION_OFF  digitalWrite(VENTILATION_pin, LOW) // выключение реле 2
#define VENTILATION_ON   digitalWrite(VENTILATION_pin, HIGH) // включение реле 2
#define ALERT_LED_OFF    digitalWrite(ALERT_led, LOW) // выключение красного светодиода
#define ALERT_LED_ON     digitalWrite(ALERT_led, HIGH) // включение красного светодиода
#define BTN_NL 13
#define BTN_PL 4
#define BTN_C 5
#define BTN_R 12
#define CURRENT_DATA_ARRAY_SIZE 5
#define ERRORS_ARRAY_SIZE 4
#define SETTINGS_ARRAY_SIZE 10
#define BTN_PRESS 0
#define FIRST_LIST 5 // граница первого листа настроек для кода (- 1)
#define SECOND_LIST 10 // граница второго листа настроек для кода (- 1)
#define THIRD_LIST 17 // граница третьего листа настроек для кода (- 1)
#define FOURTH_LIST 20 // граница четвёртого листа настроек для кода (- 1)



Adafruit_SHT31 sht31 = Adafruit_SHT31();
LiquidCrystal_I2C lcd(0x38,16,2);

volatile unsigned long cur_time_nl;
volatile unsigned long cur_time_pl;
volatile unsigned long cur_time_c;
volatile unsigned long cur_time_r;
uint8_t ypos = 0;
uint8_t xpos = 0;
uint16_t x = 0;
bool btn_nl, btn_pl, btn_r, btn_c;
uint8_t mode = 0;

uint32_t water_spead = 0;
unsigned long time;
bool flag = false;
unsigned int frequency = 500;
unsigned int beeps = 3;
bool pump_arcive = 0;
uint16_t mean = 0;
unsigned long water_time;
unsigned long time_chsm;
unsigned long time_lum;
uint8_t s1, s2, s3, s4 = 0;
unsigned long clock;
unsigned long clock2;
volatile uint16_t water_speed_ps = 0;
uint16_t litters = 0;


struct GardenTypeDef{ // необходима для хранения общих настроек
  uint16_t Id;
  String str_id = "Identification";
  uint16_t Luminosity;
  uint16_t Temperature;
  uint16_t Humidity;
  uint16_t Irrigation_Strategy;
  uint16_t Litters;
  uint16_t Water_speed_const;
  uint16_t Charge;
  uint16_t connected_pumpId;
  uint16_t Hum_ErrMax = 800;
  uint16_t Hum_ErrLow = 50;
  uint16_t Hum_NormMax = 699;
  uint16_t Hum_NormMin = 580;
};GardenTypeDef gardendata;

struct StatusTypeDef{ // предназначенна для передачи информации
  uint16_t Luminosity;
  uint16_t AirTemperature;
  uint16_t AirHumidity;
  uint16_t SoilHumidity1;
  uint16_t Pump_power = 0;
  uint16_t Vent_power = 0;
  uint16_t SoilHumidity2;
  uint16_t Power_Ok;
  uint16_t Watering_Speed;
  uint16_t Manual_control;
  uint16_t PumpErr;
  uint16_t SoilMoistureErr1;
  uint16_t SoilMoistureErr2;
  uint16_t HumTempErr;
  uint16_t Err_quant;
};StatusTypeDef statusdata;

struct DrawValues{
  const char* name;
  uint16_t* data;
};

struct Sbase{
  const char* name;
};

uint16_t lumArray[10];


Sbase Base[4]{
  {"Current data"}, {"Current Errors"}, {"Settings"}, {"Manual Control"}
};

DrawValues Vals[21] = {
  {"Luminosity", &statusdata.Luminosity}, {"Air Temperature", &statusdata.AirTemperature},
  {"Air Humidity",&statusdata.AirHumidity}, {"Soil Humidity 1", &statusdata.SoilHumidity1},
  {"Soil Humidity 2", &statusdata.SoilHumidity2},{"Water_Speed", &statusdata.Watering_Speed},
  // 6
  {"Pump Error",&statusdata.PumpErr}, {"Power_Error", &statusdata.Power_Ok},
  {"Soil Moisture Error 1", &statusdata.SoilMoistureErr1},
  {"Soil Moisture Error 2", &statusdata.SoilMoistureErr2}, {"Air Temperature-Humudity Error", &statusdata.HumTempErr}, 
  // 11
  {"Luminosity", &gardendata.Luminosity}, {"Humidity", &gardendata.Humidity}, {"Litters Quantity", &gardendata.Litters},
  {"Hum_ErrMax", &gardendata.Hum_ErrMax}, {"Hum_ErrLow", &gardendata.Hum_ErrLow}, {"Hum_NormMax", &gardendata.Hum_NormMax},
  {"Hum_NormMin", &gardendata.Hum_NormMin},
  // 18
  {"Pump power", &statusdata.Pump_power}, {"Ventilation power", &statusdata.Vent_power}, {"Manual Control", &statusdata.Manual_control}
  //21
};

// bool read_saved_data(data){
//   if (EEPROM.get(0, data)){
//     return 1;
//   }
//   return 0;
// }

// void save_data(data){
//   EEPROM.put(0, gardendata);
// }

bool chatter_btn(int btn_name, unsigned long* cur_time){
  if (digitalRead(btn_name) == BTN_PRESS){
    if (millis() - *cur_time >= 150){
      *cur_time = millis();
      return 1;
    }else{
      return 0;
    }
  }else{
    *cur_time = millis();
    return 0;
  }
}

void draw_SK(uint8_t xpos, uint8_t ypos, uint16_t value){
  switch(xpos){
    case 0:
      lcd.clear();
      lcd.noCursor();
      lcd.print(Base[ypos].name);
      //Serial.println(Base[ypos].name);
      lcd.setCursor(0, 1);
      lcd.cursor();
    break;
    case 1:
      lcd.clear();
      lcd.noCursor();
      lcd.print(Vals[ypos].name);
      lcd.setCursor(0, 1);
      lcd.print(*Vals[ypos].data);
      //Serial.println(Vals[ypos].name);
      //Serial.println(1);
    break;
    case 2:
      lcd.clear();
      lcd.cursor();
      lcd.print(Vals[ypos].name);
      lcd.setCursor(0, 1);
      lcd.print(value);
      //Serial.println(Vals[ypos].name);
      //Serial.println(2);
    break;
    case 3:
      lcd.noCursor();
      *Vals[ypos].data = x;
      lcd.clear();
      //lcd.print("Saving...");
      // EEPROM.put(0, gardendata);
      lcd.clear();
      lcd.print("Saved!");
      Serial.println(3);
    default:
      Serial.println(ypos);
  }
}
// int menu_list[*StatusTypeDef, *GardenTypeDef];
int pump() {
  if (statusdata.Pump_power){
    PUMP_ON;
    unsigned long time_start = millis();
    water_speed();
    if (millis() - time >= 4000){
      if (statusdata.Watering_Speed == 0){
        statusdata.PumpErr = 1;
        PUMP_OFF;
        statusdata.Pump_power = 0;
      }
    }
  if (litters >= gardendata.Litters){
    return;
  }  
  }else{
    time = millis();
    water_time = millis();
    litters = 0;
    PUMP_OFF;
  }
}

void water_speed_count(){
  water_speed_ps ++;
}

// функция water_speed возвращает 2 значения False если не закончила свою работу True если закончила
bool water_speed(){ // перед запуском обновить water_time
  if ((millis() - water_time) >= 1000){
    statusdata.Watering_Speed = water_speed_ps;
    // litters += water_speed_ps * gardendata.Water_speed_const;
    water_time = millis();
    water_speed_ps = 0;
  }
}
// функция check_soil_moisture возвращает 2 значения False если не закончила свою работу, True если закончила
uint8_t check_soil_moisture(uint8_t pin, uint16_t* link) { 
  *link = analogRead(pin);
  return 1;
}

// функция res_soil_moisture обрабатывает занчения влажности почвы для выдачи результата например о необходимости полива или об ошибке
uint8_t res_soil_moisture(uint16_t mean) {
  if (gardendata.Hum_NormMax >= mean && mean >= gardendata.Hum_NormMin) {
    return 0;
  } else if (gardendata.Hum_NormMin > mean && mean > gardendata.Hum_ErrLow) {
    return 1;
  }else if(gardendata.Hum_NormMax <= mean && gardendata.Hum_ErrMax <= mean){
    return 2;
  } else {
    return 3;
  }
}
// функция alarm_stop вызывается при аварийной остановке
void alarm_stop() {
  PUMP_OFF; // выключение реле 1 
  VENTILATION_OFF; // выключение реле 2
  ALERT_LED_ON; // включение красного светодиода
  lcd.clear();
  lcd.print("EMERGENCY STOP");// добавить вывод на экран 
  // добавить сообщение от gsm модуля
  // Добавить писк !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
  while (1){} 
}


uint8_t hum_temp_check(uint8_t addr) {
  if (!sht31.begin(addr)) {  // Set to 0x45 for alternate i2c addr (0x44)
    Serial.println("Couldn't find SHT31");
    statusdata.HumTempErr = 1;
  }else{
  statusdata.AirTemperature = sht31.readTemperature();
  statusdata.AirHumidity = sht31.readHumidity();
  }
  return 1;
  // незабудь выключить нагреватель
  // sht31.heater(1);
}

bool detect_dayTime() {  // пин датчика
  uint16_t mean = 0;
  for (uint8_t i = 0; i < 9; i++) {
    lumArray[i + 1] = lumArray[i];
  }
  lumArray[0] = analogRead(LUMINOSITY_pin);
  for (uint8_t i = 0; i < 10; i++){
    mean += lumArray[i];
  }
  statusdata.Luminosity = mean / 10;
  return 1;
}



// надо построить функцию для построения общего графика данных
bool res_dayTime() {  // пин датчика
  double Sx=0,Sy=0,Sxy=0,Sxx=0,a=0;

  for (uint8_t i = 0; i < 10; i++) {
    Sx += i;
    Sy += lumArray[i];
    Sxy += lumArray[i]*i;
    Sxx += i * i;

    }
  Sx/=10;Sy/=10;Sxy/=10;Sxx/=10;

  a = (Sx*Sy-Sxy)/(Sx*Sx-Sxx);
  return a;
}


void mane(){
  ypos -= btn_nl; //ypos
  ypos += btn_pl; //ypos
  xpos -= btn_r; // ypos
  xpos += btn_c; // ypos


  if (xpos == 0){mode = ypos;}


  switch (mode){
    case 0:
      if (ypos > FIRST_LIST) {ypos = FIRST_LIST;}

      if (xpos > 1) {xpos = 1;}
      break;
    case 1:
      if (xpos > 1) {xpos = 1;}

      if (xpos == 1){
        if (ypos > SECOND_LIST) {ypos = FIRST_LIST + 1;} else if (ypos < FIRST_LIST + 1) {ypos = SECOND_LIST;}
      }
      break;
    case 2:
      if (xpos > 3) {xpos = 1;}
      
      if (xpos == 1){ 
        x = *Vals[ypos].data;
        Serial.println(x);
        if (ypos > THIRD_LIST) {ypos = SECOND_LIST + 1;} else if (ypos < SECOND_LIST + 1) {ypos = THIRD_LIST;};
      }else if (xpos == 2){
        x += btn_nl;
        x -= btn_pl;
        ypos += btn_nl; //ypos
        ypos -= btn_pl; //ypos
      }
      break;
    case 3:
      if (xpos > 3) {xpos = 1;}
      
      if (xpos == 1){ 
        x = *Vals[ypos].data;
        Serial.println(x);
        if (ypos > FOURTH_LIST) {ypos = THIRD_LIST + 1;} else if (ypos < THIRD_LIST + 1) {ypos = FOURTH_LIST;};
      }else if (xpos == 2){
        x += btn_nl;
        x -= btn_pl;
        ypos += btn_nl; //ypos
        ypos -= btn_pl; //ypos
        x = x % 2;
      }
      break;
    default:
      Serial.println(mode);
      mode = 0;
      ypos = 0;
      xpos = 0;
      break;
  }

    Serial.print("BTN Pressed, Mode:");
    Serial.println(mode);
    Serial.println(xpos);
    Serial.println(ypos);
    draw_SK(xpos, ypos, x);
}


void setup() {
  EasyBuzzer.setPin(8);
  lcd.init();                      // initialize the lcd 
  lcd.init();
  lcd.backlight();
  lcd.setCursor(0,0);
  pinMode(BTN_R, INPUT_PULLUP);
  pinMode(BTN_C, INPUT_PULLUP);
  pinMode(BTN_NL, INPUT_PULLUP);
  pinMode(BTN_PL, INPUT_PULLUP);
  pinMode(PUMP_pin, INPUT);
  pinMode(PUMP_CHECK_pin, OUTPUT);
  pinMode(BUZZER_pin, INPUT);
  pinMode(ALARM_STOP_pin, INPUT_PULLUP);
  pinMode(PUMP_CHECK_pin, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(PUMP_CHECK_pin), water_speed_count, RISING);
  attachInterrupt(digitalPinToInterrupt(ALARM_STOP_pin), alarm_stop, LOW);
  Serial.begin(9600);

  if (!sht31.begin(ADDR_I2C_SHT31)) {  // Set to 0x45 for alternate i2c addr (0x44)
    Serial.println("Couldn't find SHT31");
  } else {
    uint16_t x1 = analogRead(LUMINOSITY_pin);
    for (int i = 0; i < 10; i++) {
      lumArray[i] = x1;
    }
  }
  //EasyBuzzer.beep(2700, 1);
  ypos = 0;
  xpos = 0;
  clock = millis();
}


void loop() {
  // начало обновления времени и переменных
  bool x1 = 0, x2 = 0, x3 = 0;
  // water_time = millis();
  time_chsm = millis();
  time_lum = millis();
  btn_r = chatter_btn(BTN_R, &cur_time_r);
  btn_c = chatter_btn(BTN_C, &cur_time_c);
  btn_nl = chatter_btn(BTN_NL, &cur_time_nl);
  btn_pl = chatter_btn(BTN_PL, &cur_time_pl);
  
  // конец обновления
  if (millis() - clock2 >= 1000){
    while ((x1 + x2 + x3) != 3){ // первый цикл, отвечающий за сбор данных (в каждом из них должна присутствовать оброботка кнопок для работы дисплея)
      x1 = check_soil_moisture(SOIL_MOISTURE_pin, &statusdata.SoilHumidity1);
      x3 = hum_temp_check(0x45);
      x2 = detect_dayTime();
    }
    clock2 = millis();
  }
  if ((btn_r + btn_c + btn_nl + btn_pl) || (millis() - clock >= 250)){
    mane();
    clock = millis();
  }

  // while (True){ // второй цикл, отвечающий за оброботку данных (в каждом из них должна присутствовать оброботка кнопок для работы дисплея)
  if (! statusdata.Manual_control){
    switch(res_soil_moisture(statusdata.SoilHumidity1)){
      case 0:
        statusdata.Pump_power = 0;
        statusdata.SoilMoistureErr1 = 0;
        break;
      case 1:
        statusdata.Pump_power = 1;
        statusdata.SoilMoistureErr1 = 0;
        break;
      case 2:
        statusdata.Pump_power = 0;
        statusdata.SoilMoistureErr1 = 0;
      default:
        statusdata.SoilMoistureErr1 = 1;
        break;
    }
  }
  // }

  // EasyBuzzer.update();
  // hum_temp_check(0x45);
  // check_soil_moisture(SOIL_MOISTURE_pin, &statusdata.SoilHumidity1);
  // statusdata.HumTempErr = hum_temp_check(0x45);
  if (time + 10000 >= millis() && sht31.isHeaterEnabled()) {
    sht31.heater(0);
  }
  pump();
  
  // if (pump_active){

  // }
}