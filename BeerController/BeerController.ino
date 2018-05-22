// ds18b20
#include <OneWire.h>
#include <DallasTemperature.h>

// i2c
#include <Wire.h>

// lcd1602
#include <LiquidCrystal_I2C.h>

#include <DS1302.h>

#include <RTClock.h>

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

struct TimeLine: Line
{
  TimeLine(DS1302* rtc_)
    : rtc(rtc_)
    , showSec(false)
  {
  }
  const char* str()
  {
    Time t = rtc->time();
    snprintf(buffer, sizeof(buffer), "%02d%s%02d %02d.%02d.%04d", 
      t.hr, showSec ? ":" : " ", t.min, t.date, t.mon, t.yr);
    showSec = !showSec;
    return buffer;
  }

  bool update()
  {
    return timer.update();
  }

  DS1302* rtc;
  bool showSec;
  char buffer[17];
  MyTimer timer;
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
struct MenuStack;

struct MenuItem
{
  virtual ~MenuItem() {}

  virtual bool Update()
  {
    return false;
  }
  
  virtual bool Print(LiquidCrystal_I2C& lcd)
  {
    return false;
  }
  
  virtual bool HandleButton(MenuStack* stack)
  {
    return false;
  }
  
  virtual bool HandleEncoderPlus()
  {
    return false;
  }
  
  virtual bool HandleEncoderMinus()
  {
    return false;
  }  
};

struct MenuStack: MenuItem
{
  static const byte MAX = 10;

  static byte MenuSize(MenuItem** menu)
  {
    byte size = 0;
    while(menu[size] != 0)
      size++;
    return size;
  }

  MenuStack(MenuItem** menu)
    : t(0)
  {
    menus[t] = menu;
    poses[t] = 0;
    sizes[t] = MenuSize(menu);
  }

  void Push(MenuItem** menu)
  {
    t++;
    menus[t] = menu;
    poses[t] = 0;
    sizes[t] = MenuSize(menu);
  }
  void Pop()
  {
    t--;
  }

  virtual bool Update()
  {
    return menus[t][poses[t]]->Update();
  }
  
  virtual bool Print(LiquidCrystal_I2C& lcd)
  {
    return menus[t][poses[t]]->Print(lcd);
  }
  
  virtual bool HandleButton(MenuStack* stack)
  {
    return menus[t][poses[t]]->HandleButton(stack);
  }
  
  virtual bool HandleEncoderPlus()
  {
    if (menus[t][poses[t]]->HandleEncoderPlus())
      return true;
    poses[t] = constrain(poses[t] + 1, 0, sizes[t] - 1);
    return false;
  }
  
  virtual bool HandleEncoderMinus()
  {
    if (menus[t][poses[t]]->HandleEncoderMinus())
      return true;
    poses[t] = constrain(poses[t] - 1, 0, sizes[t] - 1);
    return false;
  }
  
  MenuItem** menus[MAX];
  byte poses[MAX];
  byte sizes[MAX];
  byte t;
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
    Line* line2_ = 0,
    MenuItem** list_ = 0)
    : message1(message1_)
    , list(list_)
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
  
  virtual bool HandleButton(MenuStack* stack)
  {
    if (list)
      stack->Push(list);
    else
      stack->Pop();
    return false;
  }
  
  virtual bool HandleEncoderPlus()
  {
    return false;
  }
  
  virtual bool HandleEncoderMinus()
  {
    return false;
  }

  const char* message1;
  union{
    const char* message2;
    Line* line2;
  };
  byte type;
  MenuItem** list;
};

///////////////////////////////////////////////////////////////////////
template<typename T>
struct MenuViewer: MenuInfo
{
  MenuViewer(
    const char* message_, 
    T* value_)
    : MenuInfo(message_, 0, 0, 0)
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
    : MenuInfo(message_, 0, 0, 0)
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
    *value = constrain(*value + step, min, max);
    if (handler)
    {
      handler();
    }
    updated = true;
    return true;
  }

  virtual bool HandleEncoderMinus()
  {
    *value = constrain(*value - step, min, max);
    if (handler)
    {
      handler();
    }
    updated = true;
    return true;
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
  T step;
  T min;
  T max;
  Handler handler;
  bool updated;
};

///////////////////////////////////////////////////////////////////////
struct PID
{
  PID(float Kp, float Ki, float Kd)
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
    result += Kp * error;

    float integral = prevIntegral + Ki * error;
    prevIntegral = integral;
    result += integral;

    result += Kd * (error - prevError);
    prevError = error;
    
    return result;
  }

  float Kp;
  float Ki;
  float Kd;

  float prevIntegral;
  
  float prevError;
};

///////////////////////////////////////////////////////////////////////
struct PowerControl
{
  void update()
  {
//    counter++;
//    if (counter > )
//    {
//      counter = 0;
//    }
  }
  
  byte counter;
  byte compare;
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

const int zeroOffsetAddr = 0;
int zeroOffset = 0;
int pumpPower = 0;
int heatPower = 0;

SingleMicrosecondTimer zeroOffsetTimer;
SingleMicrosecondTimer pumpPowerTimer;
SingleMicrosecondTimer heatPowerTimer;

float Kp = 0;
float Ki = 0;
float Kd = 0;

MyTimer tempTimer(1000);

  MenuInfo                    welcome("Hello, Brewer", 0, &timeLine, 0);
  MenuInfo                    program("Program", 0, 0, 0);

    MenuInfo                    controlBack("<-", 0, 0, 0);
      
      MenuViewer<float>           tempViewer("Temperature", &temperature);
    MenuItem*                   temperatureInfo_[] = {&tempViewer, 0};
    MenuInfo                    temperatureInfo("Temperature", 0, 0, temperatureInfo_);

      MenuViewer<bool>            temperatureSensorWorkViewer("Temp Sensor Work", &temperatureSensorWork);
    MenuItem*                   temperatureSensorWorkInfo_[] = {&temperatureSensorWorkViewer, 0};
    MenuInfo                    temperatureSensorWorkInfo("Temp Sensor Work", 0, 0, temperatureSensorWorkInfo_);

      MenuEditor<int>             pumpPowerEditor("Pump Power", &pumpPower, 2, 0, 100);
    MenuItem*                   pumpPowerInfo_[] = {&pumpPowerEditor, 0};
    MenuInfo                    pumpPowerInfo("Pump Power", 0, 0, pumpPowerInfo_);

      MenuEditor<int>             heatPowerEditor("Heat Power", &heatPower, 2, 0, 100);
    MenuItem*                   heatPowerInfo_[] = {&heatPowerEditor, 0};
    MenuInfo                    heatPowerInfo("Heat Power", 0, 0, heatPowerInfo_); 

      MenuEditor<int>             zeroOffsetEditor("Zero Offset", &zeroOffset, 1, 0, 100);
    MenuItem*                   zeroOffsetInfo_[] = {&zeroOffsetEditor, 0};
    MenuInfo                    zeroOffsetInfo("Zero Offset", 0, 0, zeroOffsetInfo_); 

  MenuItem*                   control_[] = {&controlBack, &temperatureInfo, &temperatureSensorWorkInfo, &pumpPowerInfo, &heatPowerInfo, &zeroOffsetInfo, 0};
  MenuInfo                    control("Manual Control", 0, 0, control_);

    MenuInfo                    backSettings("<-", 0, 0, 0);
      MenuInfo                    timeSetBack("<-", 0, 0, 0);
      
        MenuEditor<byte>            minuteEditor("Minute", &minute, 1, 0, 59, saveTime);
      MenuItem*                   minuteInfo_[] = {&minuteEditor, 0};
      MenuInfo                    minuteInfo("Minute", 0, 0, minuteInfo_);
        
        MenuEditor<byte>            hourEditor("Hour", &hour, 1, 0, 23, saveTime);
      MenuItem*                   hourInfo_[] = {&hourEditor, 0};
      MenuInfo                    hourInfo("Hour", 0, 0, hourInfo_);

        MenuEditor<byte>            dayEditor("Day", &day, 1, 0, 31, saveTime);
      MenuItem*                   dayInfo_[] = {&dayEditor, 0};
      MenuInfo                    dayInfo("Day", 0, 0, dayInfo_);

        MenuEditor<byte>            monthEditor("Month", &month, 1, 1, 12, saveTime);
      MenuItem*                   monthInfo_[] = {&monthEditor, 0};
      MenuInfo                    monthInfo("Month", 0, 0, monthInfo_);
        
        MenuEditor<int>             yearEditor("Year", &year, 1, 2000, 2100, saveTime);
      MenuItem*                   yearInfo_[] = {&yearEditor, 0};
      MenuInfo                    yearInfo("Year", 0, 0, yearInfo_);
      
    MenuItem*                   timeSet_[] = {&timeSetBack, &minuteInfo, &hourInfo, &dayInfo, &monthInfo, &yearInfo, 0};
    MenuInfo                    timeSet("Set Time", 0, 0, timeSet_);

      MenuEditor<float>           KpSetEditor("Kp", &Kp, 0.1, 0, 100);
    MenuItem*                   KpSetInfo_[] = {&KpSetEditor, 0};
    MenuInfo                    KpSetInfo("Kp", 0, 0, KpSetInfo_);
    
      MenuEditor<float>           KiSetEditor("Ki", &Ki, 0.1, 0, 100);
    MenuItem*                   KiSetInfo_[] = {&KiSetEditor, 0};
    MenuInfo                    KiSetInfo("Ki", 0, 0, KiSetInfo_);
    
      MenuEditor<float>           KdSetEditor("Kd", &Kd, 0.1, 0, 100);
    MenuItem*                   KdSetInfo_[] = {&KdSetEditor, 0};
    MenuInfo                    KdSetInfo("Kd", 0, 0, KdSetInfo_);
    
  MenuItem*                   settings_[] = {&backSettings, &timeSet, &KpSetInfo, &KiSetInfo, &KdSetInfo, 0};
  MenuInfo                    settings("Settings", 0, 0, settings_);

  MenuItem* root_[] = {&welcome, &program, &control, &settings, 0};
  MenuStack stack(root_);
  

Button button;
RotaryEncoder encoder;

void downHandler(void*);
void plus(void*);
void minus(void*);

void setupMenu();
void setupTemperature();

void zeroCross();

void timeoutPump();
void timeoutHeat();

void setup()
{
  Serial.begin(115200);

  button.init(ReButtonPin, downHandler, 0);
  encoder.init(RePin1, RePin2, plus, minus);

  lcd.begin();

  setupTemperature();

  loadTime();
  
  tone(TonePin, 1000);
  delay(100);
  noTone(TonePin);

  pinMode(ZeroSensorPin, INPUT);
  attachInterrupt(ZeroSensorPin, zeroCross, RISING);

  pinMode(HeatPin, OUTPUT);
  digitalWrite(HeatPin, LOW);

  pinMode(PumpPin, OUTPUT);
  digitalWrite(PumpPin, LOW);

  pinMode(PC13, OUTPUT);
  digitalWrite(PC13, LOW);
}

void loop()
{  
  button.update();
  encoder.update();

  if (updateTime_.update())
  {
    struct tm * tx;
    tx = rt.getTime(NULL);

    if (tx->tm_min != minute)
    {
      loadTime();
    }
  }

  if (tempTimer.update())
  {
//    sensors.requestTemperatures();
//    temperature = sensors.getTempC(insideThermometer);
  }

  if(zeroOffsetTimer.update())
  {
    if (pumpPower > 0)
    {
      digitalWrite(PumpPin, HIGH);
    }
    if (heatPower > 0)
    {
      digitalWrite(HeatPin, HIGH);
    }
    pumpPowerTimer.start(map(pumpPower, 0, 100, 0, 10000));
    heatPowerTimer.start(map(heatPower, 0, 100, 0, 10000));
  }

  if (heatPowerTimer.update())
  {
    digitalWrite(HeatPin, LOW);
  }

  if (pumpPowerTimer.update())
  {
    digitalWrite(PumpPin, LOW);
  }

  dataChanged |= stack.Update();

  if (dataChanged)
  {
    lcd.clear();

    stack.Print(lcd);
    
    dataChanged = false;
 }
}

void zeroCross()
{
  zeroOffsetTimer.start(zeroOffset * 100);
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
  
  sensors.setResolution(insideThermometer, 12);
}

void downHandler(void*)
{
  stack.HandleButton(&stack);
  dataChanged = true;
}

void plus(void*)
{
  stack.HandleEncoderPlus();
  dataChanged = true;
}

void minus(void*)
{
  stack.HandleEncoderMinus();  
  dataChanged = true;
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

