// ds18b20
#include <OneWire.h>
#include <DallasTemperature.h>

// i2c
#include <Wire.h>

// lcd1602
#include <LiquidCrystal_I2C.h>

#include <RTClock.h>

#include <EEPROM.h>

///////////////////////////////////////////////////////////////////////
struct MyTimer
{
  MyTimer(int delay = 1000)
    : delay_(delay), prev_(0)
  {
  }

  bool update()
  {
    unsigned long current = millis();
    delta_ -= current - prev_;
    prev_ = current;
    bool update = delta_ < 0;
    if(update)
    {
      delta_ = delay_;
    }
    return update;
  }

  int delay_, delta_;
  unsigned long prev_;
};

///////////////////////////////////////////////////////////////////////
struct SingleMicrosecondTimer
{
  SingleMicrosecondTimer()
    : delay(-1)
    , prevMicrosecond(0)
  {
  }

  void start(long delay_)
  {
    delay = delay_;
    prevMicrosecond = micros();
  }

  bool update()
  {
    if (delay == -1)
      return false;

    unsigned long microsecond = micros();
    unsigned long delta = 0;
    delta = microsecond - prevMicrosecond;
    prevMicrosecond = microsecond;
    delay -= delta;
    if (delay < 0)
    {
      delay = -1;
      prevMicrosecond = 0;
      return true;
    }
    return false;
  }

  long delay;
  unsigned long prevMicrosecond;
};

///////////////////////////////////////////////////////////////////////
struct SingleMillisecondTimer
{
  SingleMillisecondTimer()
    : delay(-1)
    , prev(0)
  {
  }

  void start(long delay_)
  {
    delay = delay_;
    prev = millis();
  }

  bool update()
  {
    if (delay == -1)
      return false;

    unsigned long millisecond = millis();
    unsigned long delta = 0;
    delta = millisecond - prev;
    prev = millisecond;
    delay -= delta;
    if (delay < 0)
    {
      delay = -1;
      prev = 0;
      return true;
    }
    return false;
  }

  long delay;
  unsigned long prev;
};

///////////////////////////////////////////////////////////////////////
struct Button
{
  typedef void (*Handler)(void*);

  void init(const int pinNumber_, Handler down_, Handler up_)
  {
    pinNumber = pinNumber_;
    down = down_;
    up = up_;
    pushed = false;
    userData = 0;
    
    pinMode(pinNumber, INPUT_PULLUP);
    digitalWrite(pinNumber, HIGH);
  }

  void update()
  {
    if (!digitalRead(pinNumber) && !pushed)
    {
      pushed = true;
      if (down)
        down(userData);
    }
    if (digitalRead(pinNumber) && pushed)
    {
      pushed = false;
      if (up)
        up(userData);
    }
  }

  bool isDown()
  {
    return pushed;
  }

  int pinNumber;
  Handler down;
  Handler up;
  bool pushed;
  void* userData;
};

///////////////////////////////////////////////////////////////////////
struct RotaryEncoder
{
  typedef void(*Handler)(void*);

  static void aDown(void* ud)
  {
    RotaryEncoder* re = (RotaryEncoder*)ud;
    if (re->b.isDown() && re->plus)
    {
      re->plus(re->userData);
    }
  }
  static void bDown(void* ud)
  {
    RotaryEncoder* re = (RotaryEncoder*)ud;
    if (re->a.isDown() && re->minus)
    {
      re->minus(re->userData);
    }
  }
  
  void init(int pinA_, int pinB_, Handler plus_, Handler minus_)
  {
    a.init(pinA_, aDown, 0);
    a.userData = this;
    b.init(pinB_, bDown, 0);
    b.userData = this;
    plus = plus_;
    minus = minus_;
    userData = 0;
  }
  
  void update()
  {
    a.update();
    b.update();
  }

  Button a;
  Button b;
  Handler plus;
  Handler minus;
  void* userData;
};

///////////////////////////////////////////////////////////////////////

struct Line
{
  virtual const char* str() = 0;
  virtual bool update() = 0;
};

struct IntTimeLine: Line
{
  IntTimeLine(RTClock* rtc_)
    : rtc(rtc_)
    , showSec(false)
  {
  }
  const char* str()
  {
    struct tm * tx;
    tx = rtc->getTime(NULL);
    snprintf(buffer, sizeof(buffer), "%02d%s%02d %02d.%02d.%04d", 
      tx->tm_hour, showSec ? ":" : " ", tx->tm_min, tx->tm_mday, tx->tm_mon, 1900 + tx->tm_year);
    showSec = !showSec;
    return buffer;
  }

  bool update()
  {
    return timer.update();
  }

  RTClock* rtc;
  bool showSec;
  char buffer[17];
  MyTimer timer;
};


///////////////////////////////////////////////////////////////////////
struct MenuItem
{
  virtual ~MenuItem() {}

  virtual bool Update(){ return true; }  
  virtual bool Print(LiquidCrystal_I2C& lcd){ return true; }  
  virtual bool HandleButton(){ return true; }
  virtual bool HandleEncoderPlus(){ return true; }
  virtual bool HandleEncoderMinus(){ return true; }
};

///////////////////////////////////////////////////////////////////////
struct MenuInfo: MenuItem
{
  enum {
    MESSAGE_AND_LINE,
    MESSAGE_AND_MESSAGE
  };
  MenuInfo(
    const char* message1_ = 0,
    const char* message2_ = 0, 
    Line* line2_ = 0)
    : message1(message1_)
  {
    if (message2_ && !line2_)
    {
      type = MESSAGE_AND_MESSAGE;
      message2 = message2_;
    }
    else if (!message2_ && line2_)
    {
      type = MESSAGE_AND_LINE;
      line2 = line2_;
    }
  }
  
  virtual bool Update()
  {
    bool result = false;
    if (type == MESSAGE_AND_LINE && line2)
      result |= line2->update();
    return result;
  }
  
  virtual bool Print(LiquidCrystal_I2C& lcd)
  {
    if (message1 != 0)
    {
      lcd.setCursor(0, 0);
      lcd.print(message1);
    }
    
    if (type == MESSAGE_AND_MESSAGE && message2 != 0)
    {
      lcd.setCursor(0, 1);
      lcd.print(message2);
    }
    else if (type == MESSAGE_AND_LINE && line2 != 0)
    {
      lcd.setCursor(0, 1);
      lcd.print(line2->str());
    }
  
    return true;
  }
  
  const char* message1;
  union{
    const char* message2;
    Line* line2;
  };
  byte type;
};

///////////////////////////////////////////////////////////////////////
template<typename T>
struct MenuViewer: MenuInfo
{
  MenuViewer(
    const char* message_, 
    T* value_)
    : MenuInfo(message_, 0, 0)
    , value(value_)
  {}

  virtual bool Update()
  {
    return updateTimer.update();
  }
  
  virtual bool Print(LiquidCrystal_I2C& lcd)
  {
    if (message1 == 0)
      return false;
    lcd.setCursor(0, 0);
    lcd.print(message1);
    lcd.setCursor(0, 1);
    lcd.print("= ");
    if (value == 0)
    {
      return false;
    }
    lcd.print(*value);
  }

  MyTimer updateTimer;
  T* value;
};

///////////////////////////////////////////////////////////////////////
// struct MenuItemViewerSymbols: MenuItem
// {
//   MenuItemViewerSymbols& init(
//     MenuItem** current_, 
//     byte min_)
//   {
//     MenuItem::init(0, 0, 0, 0, current_);
//     min = min_;
//     return *this;
//   }

//   virtual void Print(LiquidCrystal_I2C& lcd)
//   {
//     lcd.setCursor(0, 0);    
//     for (byte i = 0; i < 16; ++i)
//       lcd.write(byte(min + i));

//     lcd.setCursor(0, 1);
//     for (byte i = 0; i < 16; ++i)
//       lcd.write(byte(min + 16 + i));
//   }
//   byte min;
// };

///////////////////////////////////////////////////////////////////////
template<typename T>
struct MenuEditor: MenuInfo
{
  typedef void(*Handler)();
  
  MenuEditor(  
    const char* message_, 
    T* value_, 
    T step_,
    T min_,
    T max_, 
    Handler handler_ = 0)
    : MenuInfo(message_, 0, 0)
    , value(value_)
    , step(step_)
    , handler(handler_)
  {
    min = min_;
    max = max_;
    updated = false;
  }

  virtual bool Update()
  {
    bool r = updated;
    updated = false;
    return r;
  }
  
  virtual bool HandleEncoderPlus()
  {
    float m = Multiplier();
    *value = constrain(*value + (step * m), min, max);
    if (handler)
    {
      handler();
    }
    updated = true;
    return true;
  }

  virtual bool HandleEncoderMinus()
  {
    float m = Multiplier();
    *value = constrain(*value - (step * m), min, max);
    if (handler)
    {
      handler();
    }
    updated = true;
    return true;
  }
  
  virtual bool Print(LiquidCrystal_I2C& lcd);

  float Multiplier()
  {
    unsigned long now = millis();
    long d = now - prevEditTime;
    float m = 1;
    if (d < 25)
      m = 100;
    else if (d < 50)
      m = 10;
    else if (d < 100)
      m = 1;
    prevEditTime = now;
    return m;
  }

  MyTimer updateTimer;
  T* value;
  T step;
  T min;
  T max;
  Handler handler;
  bool updated;
  unsigned long prevEditTime;
};

template<>
bool MenuEditor<float>::Print(LiquidCrystal_I2C& lcd)
{
  if (message1 == 0)
    return false;
  lcd.setCursor(0, 0);
  lcd.print(message1);
  lcd.setCursor(0, 1);
  lcd.print("= ");
  if (value == 0)
  {
    return false;
  }
  lcd.print(*value, 3);
}

template<>
bool MenuEditor<bool>::Print(LiquidCrystal_I2C& lcd)
{
  if (message1 == 0)
    return false;
  lcd.setCursor(0, 0);
  lcd.print(message1);
  lcd.setCursor(0, 1);
  lcd.print("= ");
  if (value == 0)
  {
    return false;
  }
  if (*value)
    lcd.print("On");
  else
    lcd.print("Off");
}

template<typename T>
bool MenuEditor<T>::Print(LiquidCrystal_I2C& lcd)
{
  if (message1 == 0)
    return false;
  lcd.setCursor(0, 0);
  lcd.print(message1);
  lcd.setCursor(0, 1);
  lcd.print("= ");
  if (value == 0)
  {
    return false;
  }
  lcd.print(*value);
}
///////////////////////////////////////////////////////////////////////
struct PID
{
  PID(float* Kp, float* Ki, float* Kd)
    : Kp(Kp)
    , Ki(Ki)
    , Kd(Kd)
    , prevIntegral(0)
    , prevError(0)
  {
  }

  float update(float real, float required)
  {
    float error = required - real;

    float result = 0;
    result += (*Kp) * error;

    float integral = prevIntegral + (*Ki) * error;
    prevIntegral = integral;
    result += integral;

    result += (*Kd) * (error - prevError);
    prevError = error;
    
    return result;
  }

  float* Kp;
  float* Ki;
  float* Kd;

  float prevIntegral;
  
  float prevError;
};

///////////////////////////////////////////////////////////////////////
struct PowerControl
{
  PowerControl(int pin_, byte* compare_)
    : pin(pin_)
    , compare(compare_)
  {
    pinMode(pin, OUTPUT);
    digitalWrite(pin, LOW);
  }
  void update()
  {
    counter++;
    if (counter >= 100)
    {
      counter = 0;
      digitalWrite(pin, HIGH);
    }
    if (counter >= *compare)
    {
      digitalWrite(pin, LOW);
    }
  }

  void reset()
  {
    counter = 0;
    digitalWrite(pin, LOW);
  }

  int pin;
  byte counter;
  byte* compare;
};

///////////////////////////////////////////////////////////////////////

const int LcdWidth = 16;
const int LcdHeight = 2;

const int RePin1 = PB10;
const int RePin2 = PB1;
const int ReButtonPin = PB11;

const int ZeroSensorPin = PA4;
const int PumpPin = PA3;
const int HeatPin = PA5;

const int TempPin = PB12;

const int TonePin = PB0;

LiquidCrystal_I2C lcd(0x3F, LcdWidth, LcdHeight);
bool dataChanged = true;

OneWire oneWire(TempPin);
DallasTemperature sensors(&oneWire);
DeviceAddress insideThermometer;
const int resolution = 12;

// Real Time Clock
RTClock rt(RTCSEL_LSE);
IntTimeLine timeLine(&rt);
byte hour = 0;
byte minute = 0;
byte month = 0;
byte day = 0;
int year = 0;

void saveTime();
void loadTime();

MyTimer updateTime_;

// Temperature
float temperature = 0.0;
bool temperatureSensorWork = false;
SingleMillisecondTimer temperatureTimer;

const int zeroOffsetAddr = 0;
int zeroOffset = 0;
int pumpPower = 0;
bool pumpWork = 0;
byte heatPower = 0;

PowerControl heat(HeatPin, &heatPower);
void resetHeat();

bool setTemperatureWork = false;
float setTemperature = 0;

float Kp = 0;
float Ki = 0;
float Kd = 0;

PID heatRegulator(&Kp, &Ki, &Kd);
MyTimer regulatorTimer(100);

float heatOutput = 0.f;

bool reverseEncoder = false;


int loadConfig();
void saveConfig();

MenuInfo Welcome_Info("Hello, Brewer", 0, &timeLine);
MenuInfo Program_Info("Program", 0, 0);    
    MenuInfo Program_back_Info("<-", 0, 0);
    MenuInfo RunProgram_Info("Run program", 0, 0);
    MenuInfo NewProgram_Info("New program", 0, 0);
MenuInfo Manual_Info("Manual Control", 0, 0);    
    MenuInfo Manual_back_Info("<-", 0, 0);
    MenuInfo Temperature_Info("Temperature", 0, 0);    
        MenuViewer<float> TemperatureViewer_Viewer("Temperature", &temperature);
    MenuInfo TempSensorWork_Info("Temp Sensor Work", 0, 0);    
        MenuViewer<bool> TempSensorWorkViewer_Viewer("Temp Sensor Work", &temperatureSensorWork);
    MenuInfo Pump_Info("Pump", 0, 0);    
        MenuEditor<bool> PumpEditor_Editor("Pump", &pumpWork, 1, 0, 1);
    MenuInfo Heat_Info("Heat Power", 0, 0);    
        MenuEditor<byte> HeatEditor_Editor("Heat Power", &heatPower, 2, 0, 100, resetHeat);
    MenuInfo SetTemp_Info("Set temperature", 0, 0);    
        MenuInfo SetTemp_back_Info("<-", 0, 0);
        MenuInfo SetTempOn_Info("On/Off", 0, 0);    
            MenuEditor<bool> SetTempOn_Editor("On/Off", &setTemperatureWork, 1, 0, 1);
        MenuInfo SetTemperature_Info("Set temperature", 0, 0);    
            MenuEditor<float> SetTemperature_Editor("Set temperature", &setTemperature, 5, 0, 110, saveConfig);
MenuInfo Settings_Info("Settings", 0, 0);    
    MenuInfo Settings_back_Info("<-", 0, 0);
    MenuInfo SetTime_Info("Set Time", 0, 0);    
        MenuInfo SetTime_back_Info("<-", 0, 0);
        MenuInfo Year_Info("Year", 0, 0);    
            MenuEditor<int> YearEditor_Editor("Year", &year, 1, 1970, 2100, saveTime);
        MenuInfo Month_Info("Month", 0, 0);    
            MenuEditor<byte> MonthEditor_Editor("Month", &month, 1, 1, 12, saveTime);
        MenuInfo Day_Info("Day", 0, 0);    
            MenuEditor<byte> DayEditor_Editor("Day", &day, 1, 0, 31, saveTime);
        MenuInfo Hour_Info("Hour", 0, 0);    
            MenuEditor<byte> HourEditor_Editor("Hour", &hour, 1, 0, 23, saveTime);
        MenuInfo Minute_Info("Minute", 0, 0);    
            MenuEditor<byte> MinuteEditor_Editor("Minute", &minute, 1, 0, 59, saveTime);
    MenuInfo Kp_Info("Kp", 0, 0);    
        MenuEditor<float> KpEditor_Editor("Kp", &Kp, 0.1, 0, 100, saveConfig);
    MenuInfo Ki_Info("Ki", 0, 0);    
        MenuEditor<float> KiEditor_Editor("Ki", &Ki, 0.001, 0, 100, saveConfig);
    MenuInfo Kd_Info("Kd", 0, 0);    
        MenuEditor<float> KdEditor_Editor("Kd", &Kd, 0.001, 0, 100, saveConfig);
    MenuInfo ReverseEnc_Info("Reverse Encoder", 0, 0);    
        MenuEditor<bool> ReverseEnc_Editor("Reverse Encoder", &reverseEncoder, 1, 0, 1, saveConfig);

MenuItem* current = &Welcome_Info;

Button button;
RotaryEncoder encoder;

void downHandler(void*);
void plus(void*);
void minus(void*);
void minusHandler(void*);
void plusHandler(void*);

void setupMenu();
void setupTemperature();

void zeroCross();

void timeoutPump();
void timeoutHeat();

void setupEEPROM();

void setup()
{
  Serial.begin(115200);

  button.init(ReButtonPin, downHandler, 0);
  encoder.init(RePin1, RePin2, plusHandler, minusHandler);

  lcd.begin();

  setupTemperature();
  sensors.requestTemperatures();

  loadTime();

  tone(TonePin, 1000, 200);
  delay(200);
  noTone(TonePin);

  loadConfig();

  pinMode(ZeroSensorPin, INPUT);
  attachInterrupt(ZeroSensorPin, zeroCross, RISING);

  pinMode(PumpPin, OUTPUT);
  digitalWrite(PumpPin, LOW);
}

void loop()
{  
  button.update();
  encoder.update();

  if (updateTime_.update())
  {
    tm* tx = rt.getTime(NULL);

    if (tx->tm_min != minute)
    {
      loadTime();
    }
  }

  if (pumpWork)
  {
    digitalWrite(PumpPin, HIGH);
  }
  else
  {
    digitalWrite(PumpPin, LOW);
  }

  if (temperatureTimer.update())
  {
    temperature = sensors.getTempC(insideThermometer);

    Serial.print(temperature); Serial.print(" ");
    Serial.print(setTemperature); Serial.print(" ");
    Serial.print(heatOutput); Serial.print(" ");
    Serial.print(heatPower); Serial.print(" ");
    Serial.println("");

    temperatureTimer.start(750 / (1 << (12 - resolution)));
    
    sensors.requestTemperatures();
  }

  if (setTemperatureWork && regulatorTimer.update())
  {
    heatOutput = heatRegulator.update(temperature, setTemperature);
    int ho = int(heatOutput);
    heatPower = constrain(ho, 0, 100);
  }

  if (current)
    dataChanged |= current->Update();

  if (dataChanged)
  {
    lcd.clear();

    if (current)
      current->Print(lcd);
    
    dataChanged = false;
 }
}

void zeroCross()
{
  heat.update();
}

void setupTemperature()
{
  sensors.begin();

  if (!sensors.getAddress(insideThermometer, 0))
  {
    temperatureSensorWork = false;
    return;
  }
  temperatureSensorWork = true;

  sensors.setWaitForConversion(false);
  sensors.requestTemperatures();
  temperatureTimer.start(750 / (1 << (12 - resolution)));

  sensors.setResolution(insideThermometer, resolution);
}

void resetHeat()
{
  heat.reset();
}

void downHandler(void*)
{
// button handler
do{
// if (current == &Welcome_Info) {}
if (current == &Program_Info) { current = &Program_back_Info; break; }    
    if (current == &Program_back_Info) { current = &Program_Info; break; }
    if (current == &RunProgram_Info) { current = &Program_Info; break; }
    if (current == &NewProgram_Info) { current = &Program_Info; break; }
if (current == &Manual_Info) { current = &Manual_back_Info; break; }    
    if (current == &Manual_back_Info) { current = &Manual_Info; break; }
    if (current == &Temperature_Info) { current = &TemperatureViewer_Viewer; break; }    
        if (current == &TemperatureViewer_Viewer) { current = &Temperature_Info; break; }
    if (current == &TempSensorWork_Info) { current = &TempSensorWorkViewer_Viewer; break; }    
        if (current == &TempSensorWorkViewer_Viewer) { current = &TempSensorWork_Info; break; }
    if (current == &Pump_Info) { current = &PumpEditor_Editor; break; }    
        if (current == &PumpEditor_Editor) { current = &Pump_Info; break; }
    if (current == &Heat_Info) { current = &HeatEditor_Editor; break; }    
        if (current == &HeatEditor_Editor) { current = &Heat_Info; break; }
    if (current == &SetTemp_Info) { current = &SetTemp_back_Info; break; }    
        if (current == &SetTemp_back_Info) { current = &SetTemp_Info; break; }
        if (current == &SetTempOn_Info) { current = &SetTempOn_Editor; break; }    
            if (current == &SetTempOn_Editor) { current = &SetTempOn_Info; break; }
        if (current == &SetTemperature_Info) { current = &SetTemperature_Editor; break; }    
            if (current == &SetTemperature_Editor) { current = &SetTemperature_Info; break; }
if (current == &Settings_Info) { current = &Settings_back_Info; break; }    
    if (current == &Settings_back_Info) { current = &Settings_Info; break; }
    if (current == &SetTime_Info) { current = &SetTime_back_Info; break; }    
        if (current == &SetTime_back_Info) { current = &SetTime_Info; break; }
        if (current == &Year_Info) { current = &YearEditor_Editor; break; }    
            if (current == &YearEditor_Editor) { current = &Year_Info; break; }
        if (current == &Month_Info) { current = &MonthEditor_Editor; break; }    
            if (current == &MonthEditor_Editor) { current = &Month_Info; break; }
        if (current == &Day_Info) { current = &DayEditor_Editor; break; }    
            if (current == &DayEditor_Editor) { current = &Day_Info; break; }
        if (current == &Hour_Info) { current = &HourEditor_Editor; break; }    
            if (current == &HourEditor_Editor) { current = &Hour_Info; break; }
        if (current == &Minute_Info) { current = &MinuteEditor_Editor; break; }    
            if (current == &MinuteEditor_Editor) { current = &Minute_Info; break; }
    if (current == &Kp_Info) { current = &KpEditor_Editor; break; }    
        if (current == &KpEditor_Editor) { current = &Kp_Info; break; }
    if (current == &Ki_Info) { current = &KiEditor_Editor; break; }    
        if (current == &KiEditor_Editor) { current = &Ki_Info; break; }
    if (current == &Kd_Info) { current = &KdEditor_Editor; break; }    
        if (current == &KdEditor_Editor) { current = &Kd_Info; break; }
    if (current == &ReverseEnc_Info) { current = &ReverseEnc_Editor; break; }    
        if (current == &ReverseEnc_Editor) { current = &ReverseEnc_Info; break; }
}while(false);
  if (current)
      current->HandleButton();
    dataChanged = true;
}

void plus(void*)
{
// up handler
do{
if (current == &Welcome_Info) { current = &Program_Info; break; }
if (current == &Program_Info) { current = &Manual_Info; break; }    
    if (current == &Program_back_Info) { current = &RunProgram_Info; break; }
    if (current == &RunProgram_Info) { current = &NewProgram_Info; break; }
    if (current == &NewProgram_Info) { break; }
if (current == &Manual_Info) { current = &Settings_Info; break; }    
    if (current == &Manual_back_Info) { current = &Temperature_Info; break; }
    if (current == &Temperature_Info) { current = &TempSensorWork_Info; break; }    
        if (current == &TemperatureViewer_Viewer) { break; }
    if (current == &TempSensorWork_Info) { current = &Pump_Info; break; }    
        if (current == &TempSensorWorkViewer_Viewer) { break; }
    if (current == &Pump_Info) { current = &Heat_Info; break; }    
        if (current == &PumpEditor_Editor) { break; }
    if (current == &Heat_Info) { current = &SetTemp_Info; break; }    
        if (current == &HeatEditor_Editor) { break; }
    if (current == &SetTemp_Info) { break; }    
        if (current == &SetTemp_back_Info) { current = &SetTempOn_Info; break; }
        if (current == &SetTempOn_Info) { current = &SetTemperature_Info; break; }    
            if (current == &SetTempOn_Editor) { break; }
        if (current == &SetTemperature_Info) { break; }    
            if (current == &SetTemperature_Editor) { break; }
if (current == &Settings_Info) { break; }    
    if (current == &Settings_back_Info) { current = &SetTime_Info; break; }
    if (current == &SetTime_Info) { current = &Kp_Info; break; }    
        if (current == &SetTime_back_Info) { current = &Year_Info; break; }
        if (current == &Year_Info) { current = &Month_Info; break; }    
            if (current == &YearEditor_Editor) { break; }
        if (current == &Month_Info) { current = &Day_Info; break; }    
            if (current == &MonthEditor_Editor) { break; }
        if (current == &Day_Info) { current = &Hour_Info; break; }    
            if (current == &DayEditor_Editor) { break; }
        if (current == &Hour_Info) { current = &Minute_Info; break; }    
            if (current == &HourEditor_Editor) { break; }
        if (current == &Minute_Info) { break; }    
            if (current == &MinuteEditor_Editor) { break; }
    if (current == &Kp_Info) { current = &Ki_Info; break; }    
        if (current == &KpEditor_Editor) { break; }
    if (current == &Ki_Info) { current = &Kd_Info; break; }    
        if (current == &KiEditor_Editor) { break; }
    if (current == &Kd_Info) { current = &ReverseEnc_Info; break; }    
        if (current == &KdEditor_Editor) { break; }
    if (current == &ReverseEnc_Info) { break; }    
        if (current == &ReverseEnc_Editor) { break; }
}while(false);
  if (current)
      current->HandleEncoderPlus();
  dataChanged = true;
}


void minus(void*)
{
// down handler
do{
if (current == &Welcome_Info) { break; }
if (current == &Program_Info) { current = &Welcome_Info; break; }    
    if (current == &Program_back_Info) { break; }
    if (current == &RunProgram_Info) { current = &Program_back_Info; break; }
    if (current == &NewProgram_Info) { current = &RunProgram_Info; break; }
if (current == &Manual_Info) { current = &Program_Info; break; }    
    if (current == &Manual_back_Info) { break; }
    if (current == &Temperature_Info) { current = &Manual_back_Info; break; }    
        if (current == &TemperatureViewer_Viewer) { break; }
    if (current == &TempSensorWork_Info) { current = &Temperature_Info; break; }    
        if (current == &TempSensorWorkViewer_Viewer) { break; }
    if (current == &Pump_Info) { current = &TempSensorWork_Info; break; }    
        if (current == &PumpEditor_Editor) { break; }
    if (current == &Heat_Info) { current = &Pump_Info; break; }    
        if (current == &HeatEditor_Editor) { break; }
    if (current == &SetTemp_Info) { current = &Heat_Info; break; }    
        if (current == &SetTemp_back_Info) { break; }
        if (current == &SetTempOn_Info) { current = &SetTemp_back_Info; break; }    
            if (current == &SetTempOn_Editor) { break; }
        if (current == &SetTemperature_Info) { current = &SetTempOn_Info; break; }    
            if (current == &SetTemperature_Editor) { break; }
if (current == &Settings_Info) { current = &Manual_Info; break; }    
    if (current == &Settings_back_Info) { break; }
    if (current == &SetTime_Info) { current = &Settings_back_Info; break; }    
        if (current == &SetTime_back_Info) { break; }
        if (current == &Year_Info) { current = &SetTime_back_Info; break; }    
            if (current == &YearEditor_Editor) { break; }
        if (current == &Month_Info) { current = &Year_Info; break; }    
            if (current == &MonthEditor_Editor) { break; }
        if (current == &Day_Info) { current = &Month_Info; break; }    
            if (current == &DayEditor_Editor) { break; }
        if (current == &Hour_Info) { current = &Day_Info; break; }    
            if (current == &HourEditor_Editor) { break; }
        if (current == &Minute_Info) { current = &Hour_Info; break; }    
            if (current == &MinuteEditor_Editor) { break; }
    if (current == &Kp_Info) { current = &SetTime_Info; break; }    
        if (current == &KpEditor_Editor) { break; }
    if (current == &Ki_Info) { current = &Kp_Info; break; }    
        if (current == &KiEditor_Editor) { break; }
    if (current == &Kd_Info) { current = &Ki_Info; break; }    
        if (current == &KdEditor_Editor) { break; }
    if (current == &ReverseEnc_Info) { current = &Kd_Info; break; }    
        if (current == &ReverseEnc_Editor) { break; }
}while(false);
  if (current)
      current->HandleEncoderMinus();
  dataChanged = true;
}

void minusHandler(void*)
{
  if (reverseEncoder)
    plus(0);
  else
    minus(0);
}

void plusHandler(void*)
{
  if (reverseEncoder)
    minus(0);
  else
    plus(0);
}

void saveTime()
{
    struct tm * tx;
    tx = rt.getTime(NULL);

    tx->tm_min  = minute;
    tx->tm_hour = hour;

    tx->tm_mday = day;
    tx->tm_mon  = month;
    tx->tm_year = year - 1900;
    
    rt.setTime(tx);
}
void loadTime()
{
    struct tm * tx;
    tx = rt.getTime(NULL);

    minute  = tx->tm_min;
    hour    = tx->tm_hour;

    day     = tx->tm_mday;
    month   = tx->tm_mon;
    year    = 1900 + tx->tm_year;
}

#define CONFIG_START 32
 
typedef struct
{
  float kp;
  float ki;
  float kd;
  bool re;
  float st;  
} configuration_type;
 
int loadConfig() {
  configuration_type CONFIGURATION;
  for (unsigned int i=0; i<sizeof(CONFIGURATION); i++)
  {
    *((char*)&CONFIGURATION + i) = EEPROM.read(CONFIG_START + i);
  }
  Kp = CONFIGURATION.kp;
  Ki = CONFIGURATION.ki;
  Kd = CONFIGURATION.kd;
  reverseEncoder = CONFIGURATION.re;
  setTemperature = CONFIGURATION.st;
  
  Serial.println("configuration loaded");
  return 1;
}
 
void saveConfig() {
  configuration_type CONFIGURATION = {
    Kp,
    Ki,
    Kd,
    reverseEncoder,
    setTemperature,
  };
  
  for (unsigned int i=0; i<sizeof(CONFIGURATION); i++)
  {
    EEPROM.write(CONFIG_START + i, *((char*)&CONFIGURATION + i));
  }
}

