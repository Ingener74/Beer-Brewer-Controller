// ds18b20
#include <OneWire.h>
#include <DallasTemperature.h>

// i2c
#include <Wire.h>

// lcd1602
#include <LiquidCrystal_I2C.h>

#include <DS1302.h>

#include "pitches.h"

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
    
    pinMode(INPUT_PULLUP, pinNumber);
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

struct MenuItem
{
  MenuItem()
  {
    message1 = 0;
    message2 = 0;

    line1 = 0;
    line2 = 0;

    current = 0;
    
    parent = 0;  
    next = 0;
    prev = 0;  
    child = 0;
  }

  MenuItem& init(
    const char* message_, 
    MenuItem** current_)
  {
    return init(message_, 0, 0, 0, current_);
  }
  
  MenuItem& init(
    const char* message1_, 
    const char* message2_, 
    MenuItem** current_)
  {
    return init(message1_, message2_, 0, 0, current_);
  }

  MenuItem& init(
    const char* message1_, 
    Line* line2_, 
    MenuItem** current_)
  {
    return init(message1_, 0, 0, line2_, current_);
  }

  MenuItem& init(
    Line* line1_, 
    const char* message2_, 
    MenuItem** current_)
  {
    return init(0, message2_, line1_, 0, current_);
  }

  MenuItem& init(
    Line* line1_, 
    Line* line2_, 
    MenuItem** current_)
  {
    return init(0, 0, line1_, line2_, current_);
  }

  MenuItem& init(
    const char* message1_, 
    const char* message2_, 
    Line* line1_,
    Line* line2_,
    MenuItem** current_)
  {
    message1 = message1_;
    message2 = message2_;
    line1 = line1_;
    line2 = line2_;
    current = current_;
    return *this;
  }

  MenuItem& Parent(MenuItem* parent_){ parent = parent_; return *this; }
  MenuItem& Next(MenuItem* next_){ next = next_; return *this; }
  MenuItem& Prev(MenuItem* prev_){ prev = prev_; return *this; }
  MenuItem& Child(MenuItem* child_){ child = child_; return *this; }

  virtual bool Update()
  {
    bool result = false;
    if (line1)
      result |= line1->update();
    if (line2)
      result |= line2->update();
    return result;
  }

  virtual void Print(LiquidCrystal_I2C& lcd)
  {
    if (message1 != 0)
    {
      lcd.setCursor(0, 0);
      lcd.print(message1);
    }
    else if (line1 != 0)
    {
      lcd.setCursor(0, 0);
      lcd.print(line1->str());
    }
    
    if (message2 != 0)
    {
      lcd.setCursor(0, 1);
      lcd.print(message2);
    }
    else if (line2 != 0)
    {
      lcd.setCursor(0, 1);
      lcd.print(line2->str());
    }
  }

  virtual void HandleButton()
  {
    if (parent)
      *current = parent;
    if (child)
      *current = child;
  }

  virtual void HandleEncoderPlus()
  {
    if (next)
      *current = next;
  }

  virtual void HandleEncoderMinus()
  {
    if (prev)
      *current = prev;
  }

  const char* message1;
  const char* message2;
  Line* line1;
  Line* line2;

  MenuItem** current;
  
  MenuItem* parent;
  MenuItem* next;
  MenuItem* prev;
  MenuItem* child;
};

///////////////////////////////////////////////////////////////////////
template<typename T>
struct MenuItemViewer: MenuItem
{
  MenuItemViewer(): MenuItem() {}

  MenuItemViewer& init(
    const char* message_, 
    MenuItem** current_,
    T* value_)
  {
    MenuItem::init(message_, 0, 0, 0, current_);
    value = value_;
    return *this;
  }

  MenuItemViewer& setValue(T* value_)
  {
    value = value_;
    return *this;
  }
  
  virtual bool Update()
  {
    if (updateTimer.update())
    {
      return true;
    }
    return false;
  }
  
  virtual void Print(LiquidCrystal_I2C& lcd)
  {
    if (message1 == 0)
      return;
    lcd.setCursor(0, 0);
    lcd.print(message1);
    lcd.setCursor(0, 1);
    lcd.print("= ");
    if (value == 0)
    {
      return;
    }
    lcd.print(*value);
  }

  MyTimer updateTimer;
  T* value;
};

///////////////////////////////////////////////////////////////////////
struct MenuItemViewerSymbols: MenuItem
{
  MenuItemViewerSymbols& init(
    MenuItem** current_, 
    byte min_)
  {
    MenuItem::init(0, 0, 0, 0, current_);
    min = min_;
    return *this;
  }

  virtual void Print(LiquidCrystal_I2C& lcd)
  {
    lcd.setCursor(0, 0);    
    for (byte i = 0; i < 16; ++i)
      lcd.write(byte(min + i));

    lcd.setCursor(0, 1);
    for (byte i = 0; i < 16; ++i)
      lcd.write(byte(min + 16 + i));
  }
  byte min;
};

///////////////////////////////////////////////////////////////////////
template<typename T>
struct MenuItemEditor: MenuItem
{
  MenuItemEditor(): MenuItem() {}

  MenuItemEditor& init(
    const char* message_, 
    MenuItem** current_,
    T* value_, 
    T step_,
    T min_,
    T max_)
  {
    MenuItem::init(message_, 0, 0, 0, current_);
    value = value_;
    step = step_;
    min = min_;
    max = max_;
    return *this;
  }

  MenuItemEditor& setValue(T* value_)
  {
    value = value_;
    return *this;
  }
  
  virtual bool Update()
  {
    if (updateTimer.update())
    {
      return true;
    }
    return false;
  }
  
  virtual void HandleEncoderPlus()
  {
    *value = constrain(*value + step, min, max);
  }

  virtual void HandleEncoderMinus()
  {
    *value = constrain(*value - step, min, max);
  }
  
  virtual void Print(LiquidCrystal_I2C& lcd)
  {
    if (message1 == 0)
      return;
    lcd.setCursor(0, 0);
    lcd.print(message1);
    lcd.setCursor(0, 1);
    lcd.print("= ");
    if (value == 0)
    {
      return;
    }
    lcd.print(*value);
  }

  MyTimer updateTimer;
  T* value;
  T step;
  T min;
  T max;
};

///////////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////////

const int LcdWidth = 16;
const int LcdHeight = 2;

const int RePin1 = 4;
const int RePin2 = 5;
const int ReButtonPin = 6;

const int ZeroSensorPin = 2;
const int PumpPin = 7;
const int HeatPin = 8;
const int TempPin = 9;

const int RtcRstPin = 11;
const int RtcClkPin = 10;
const int RtcDatPin = 12;

const int TonePin = 3;

LiquidCrystal_I2C lcd(0x3F, LcdWidth, LcdHeight);
bool dataChanged = true;

OneWire oneWire(TempPin);
DallasTemperature sensors(&oneWire);
DeviceAddress insideThermometer;

DS1302 rtc(RtcRstPin, RtcDatPin, RtcClkPin);
TimeLine timeLine(&rtc);

float temperature = 0.0;
bool temperatureSensorWork = false;
int temperatureSensorResolution = 0;

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

MenuItem* current = 0;

  MenuItem                    welcome;
  MenuItem                    program;
  MenuItem                    control;
  MenuItem                    settings;
    MenuItem                    backFromSettings;
    MenuItem                    timeItem;
      
    MenuItem                    pumpPowerItem;
      MenuItemEditor<int>         pumpPowerEditor;
    MenuItem                    heatPowerItem;
      MenuItemEditor<int>         heatPowerEditor;
    MenuItem                    zeroOffsetItem;
      MenuItemEditor<int>         zeroOffsetEditor;
    MenuItem                    KpSet;
      MenuItemEditor<float>       KpSetEditor;
    MenuItem                    KiSet;
      MenuItemEditor<float>       KiSetEditor;
    MenuItem                    KdSet;
      MenuItemEditor<float>       KdSetEditor;
    MenuItem                    temperatureItem;
      MenuItemViewer<float>       temperatureViewer;
    MenuItem                    temperatureSensorWorkItem;
      MenuItemViewer<bool>        temperatureSensorWorkViewer;
    MenuItem                    temperatureSensorResolutionItem;
      MenuItemViewer<int>         temperatureSensorResolutionViewer;
    MenuItem                    symbolsItem;
      MenuItem                    symbolsBack;
      MenuItemViewerSymbols       symbols1;
      MenuItemViewerSymbols       symbols2;
      MenuItemViewerSymbols       symbols3;
      MenuItemViewerSymbols       symbols4;
      MenuItemViewerSymbols       symbols5;
      MenuItemViewerSymbols       symbols6;
      MenuItemViewerSymbols       symbols7;
      MenuItemViewerSymbols       symbols8;


Button button;
RotaryEncoder encoder;

void downHandler(void*);
void plus(void*);
void minus(void*);

void setupMenu();
void setupTemperature();

void playMelody(int pin, int melody[]);

void zeroCross();

void setupRussianSymbols();

void setup()
{
  Serial.begin(9600);

  button.init(ReButtonPin, downHandler, 0);
  encoder.init(RePin1, RePin2, plus, minus);

  setupTemperature();
  setupMenu();

  pinMode(TonePin, OUTPUT);

  rtc.writeProtect(false);
  rtc.halt(false);

  lcd.begin();

  tone(TonePin, 1000);
  delay(100);
  noTone(TonePin);

  pinMode(ZeroSensorPin, INPUT);
  attachInterrupt(digitalPinToInterrupt(ZeroSensorPin), zeroCross, RISING);

  pinMode(HeatPin, OUTPUT);
  digitalWrite(HeatPin, LOW);

  pinMode(PumpPin, OUTPUT);
  digitalWrite(PumpPin, LOW);
}

void loop()
{  
  button.update();
  encoder.update();

  if (tempTimer.update())
  {
    sensors.requestTemperatures();
    temperature = sensors.getTempC(insideThermometer);
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

  if (current != 0)
    dataChanged |= current->Update();

  if (dataChanged)
  {
    lcd.clear();
    
    if (current != 0)
    {
      current->Print(lcd);
    }
    
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
  temperatureSensorResolution = sensors.getResolution(insideThermometer);
}

void setupMenu()
{
  current = &welcome;                    
                    
  welcome.init                                 ("Hello, Brewer", &timeLine, &current).Next(&program);

  program.init                                 ("Program",              &current).Next(&control).Prev(&welcome);
  
  control.init                                 ("Control",              &current).Next(&settings).Prev(&program);
  
  settings.init                                ("Settings",             &current).Prev(&control).Child(&pumpPowerItem);
    backFromSettings.init                      ("<-",                   &current).Next(&pumpPowerItem).Parent(&settings);
    
    pumpPowerItem.init                         ("Pump power",           &current).Next(&heatPowerItem).Prev(&backFromSettings).Child(&pumpPowerEditor);
      pumpPowerEditor.init                     ("Pump power",           &current, &pumpPower, 3, 0, 100).Parent(&pumpPowerItem);
    heatPowerItem.init                         ("Heat power",           &current).Next(&zeroOffsetItem).Prev(&pumpPowerItem).Child(&heatPowerEditor);
      heatPowerEditor.init                     ("Heat power",           &current, &heatPower, 5, 0, 100).Parent(&heatPowerItem);
    zeroOffsetItem.init                        ("Zero offset",          &current).Next(&KpSet).Prev(&heatPowerItem).Child(&zeroOffsetEditor);
      zeroOffsetEditor.init                    ("Zero offset",          &current, &zeroOffset, 2, 0, 100).Parent(&zeroOffsetItem);
    KpSet.init                                 ("K proportional",       &current).Next(&KiSet).Prev(&zeroOffsetItem);
    KiSet.init                                 ("K integral",           &current).Next(&KdSet).Prev(&KpSet);
    KdSet.init                                 ("K differential",       &current).Next(&temperatureItem).Prev(&KiSet);
    
    temperatureItem.init                       ("Temperature",          &current).Next(&temperatureSensorWorkItem).Prev(&KdSet).Child(&temperatureViewer);
      temperatureViewer.init                   ("Temperature",          &current, &temperature).Parent(&temperatureItem);
    
    temperatureSensorWorkItem.init             ("Temp Sensor Work",     &current).Next(&temperatureSensorResolutionItem).Prev(&temperatureItem).Child(&temperatureSensorWorkViewer);
      temperatureSensorWorkViewer.init         ("Temp Sensor Work",     &current, &temperatureSensorWork).Parent(&temperatureSensorWorkItem);
    
    temperatureSensorResolutionItem.init       ("Temp Sensor Res",      &current).Next(&symbolsItem).Prev(&temperatureSensorWorkItem).Child(&temperatureSensorResolutionViewer);
      temperatureSensorResolutionViewer.init   ("Temp Sensor Res",      &current, &temperatureSensorResolution).Parent(&temperatureSensorResolutionItem);

    symbolsItem.init                           ("Symbols",              &current).Prev(&temperatureSensorResolutionItem).Child(&symbols1);
      symbolsBack.init                         ("<-",                   &current).Next(&symbols1).Parent(&symbolsItem);
      symbols1.init                            (                        &current, 32 * 0).Next(&symbols2).Prev(&symbolsBack);
      symbols2.init                            (                        &current, 32 * 1).Next(&symbols3).Prev(&symbols1);
      symbols3.init                            (                        &current, 32 * 2).Next(&symbols4).Prev(&symbols2);
      symbols4.init                            (                        &current, 32 * 3).Next(&symbols5).Prev(&symbols3);
      symbols5.init                            (                        &current, 32 * 4).Next(&symbols6).Prev(&symbols4);
      symbols6.init                            (                        &current, 32 * 5).Next(&symbols7).Prev(&symbols5);
      symbols7.init                            (                        &current, 32 * 6).Next(&symbols8).Prev(&symbols6);
      symbols8.init                            (                        &current, 32 * 7).Prev(&symbols7);
}

void downHandler(void*)
{
  if (current != 0)
    current->HandleButton();
  dataChanged = true;
}

void plus(void*)
{
  if (current != 0)
    current->HandleEncoderPlus();
  dataChanged = true;
}

void minus(void*)
{
  if (current != 0)
    current->HandleEncoderMinus();
  dataChanged = true;
}

int speed=90;  //higher value, slower notes
void playMelody(int pin, int melody[])
{
  for (int thisNote = 0; melody[thisNote*2] != END; thisNote++)
  {
    int noteDuration = speed * melody[thisNote*2 + 1];
    
    tone(pin, melody[thisNote*2],noteDuration*.95);
    
    delay(noteDuration);

    noTone(pin);
  }
}

