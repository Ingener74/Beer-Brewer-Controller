#include <OneWire.h>
#include <DallasTemperature.h>

#include <Wire.h>

#include <LiquidCrystal_I2C.h>

#include <RTClock.h>

#include <PID_v1.h>

// https://www.youtube.com/watch?v=IenFIIMIbyk - cool menu

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
/*
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
*/
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

  void init(const int pinNumber_, Handler down_, Handler up_, byte* bounceDelay = 0)
  {
    pinNumber = pinNumber_;
    down = down_;
    up = up_;
    pushed = false;
    userData = 0;
    _bounceDelay = bounceDelay;
    
    pinMode(pinNumber, INPUT_PULLUP);
    digitalWrite(pinNumber, HIGH);
  }

  void update()
  {
    if (!digitalRead(pinNumber) && !pushed)
    {
      pushed = true;
      if (_bounceDelay == 0)
      {
        if (down)
          down(userData);
      }
      else
      {
        _bounceTimer.start(*_bounceDelay);
      }
    }
    if (digitalRead(pinNumber) && pushed)
    {
      pushed = false;
      if (up)
        up(userData);
    }
    if (_bounceDelay != 0 && pushed && _bounceTimer.update() && down)
    {
      down(userData);
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
  byte* _bounceDelay;
  SingleMillisecondTimer _bounceTimer;
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
  
  void init(int pinA_, int pinB_, Handler plus_, Handler minus_, byte* bounceDelay = 0)
  {
    a.init(pinA_, aDown, 0, bounceDelay);
    a.userData = this;
    b.init(pinB_, bDown, 0, bounceDelay);
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
  virtual const char* str() {}
  virtual void print(LiquidCrystal_I2C& lcd) {}
  virtual bool update() {}
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
      const char* s = line2->str();
      if (s)
      {
        lcd.print(s);
      }
      line2->print(lcd);
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
    Handler handler_ = 0,
    Line* line2 = 0)
    : MenuInfo(message_, 0, line2)
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
  lcd.print(*value, 4);
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
  if (line2)
  {
    lcd.print(line2->str());
  }
  else
  {
    lcd.print("= ");
    if (value == 0)
    {
      return false;
    }
    lcd.print(*value);
  }
}

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
struct Updater
{
  virtual void update() = 0;
};

template<typename T>
struct Var: Updater
{
  typedef void (*Handler)(void*, T*);
  Var(const T& defaultValue = {}, Handler handler = 0)
    : value(defaultValue)
    , prevValue(defaultValue)
    , _handler(handler)
  {
  }
  void update()
  {
    if (value != prevValue && _handler)
    {
      _handler(this, &value);
      prevValue = value;
    }
  }

  T* ptr()
  {
    return &value;
  }

  T value;
  T prevValue;
  Handler _handler;
};

///////////////////////////////////////////////////////////////////////
const int NumberOfPrograms = 20;
const int MaxNumberOfSteps = 20;

struct ProgramStep
{
  byte opCode;
  union 
  {
    byte temperature;
    byte minutes;
  };
};

struct Interpreter
{
  enum Command
  {
    NOP,        // 0
    HEAT,       // 1
    COLD,       // 2
    WAIT,       // 3
    PUMP_ON ,   // 4
    PUMP_OFF,   // 5
    OP_CODE_COUNT,
  };

  static const char* operations[OP_CODE_COUNT];

  Interpreter(
    ProgramStep* programs,
    byte* program,
    byte* step,
    double* temperature,
    double* setTemperature,
    bool* heatOnOff,
    bool* pumpOnOff,
    bool* runProgram,
    tm* currentTime
    )
    : _programs(programs)
    , _step(step)
    , _program(program)
    , _temperature(temperature)
    , _setTemperature(setTemperature)
    , _heatOnOff(heatOnOff)
    , _pumpOnOff(pumpOnOff)
    , _runProgram(runProgram)
    , _currentTime(currentTime)
  {
  }
  
  void update()
  {
    switch (_programs[*_program * MaxNumberOfSteps + *_step].opCode)
    {
      case NOP:
      {
        next();
        break;      
      }
      case HEAT:
      {
        float t = _programs[*_program * MaxNumberOfSteps + *_step].temperature;
        (*_heatOnOff) = true;
        (*_setTemperature) = t;
        if ((*_temperature) >= t)
        {
          next();
        }
        break;
      }
      case COLD:
      {
        float t = _programs[*_program * MaxNumberOfSteps + *_step].temperature;
        (*_heatOnOff) = true;
        if ((*_temperature) < t)
        {
          next();
        }
        break;
      }
      case WAIT:
      {
        if (
          _currentTime->tm_min  == _waitTime.tm_min  &&
          _currentTime->tm_hour == _waitTime.tm_hour &&
          _currentTime->tm_mday == _waitTime.tm_mday &&
          _currentTime->tm_mon  == _waitTime.tm_mon  &&
          _currentTime->tm_year == _waitTime.tm_year &&
          _currentTime->tm_sec  == _waitTime.tm_sec
        ) {
          next();
        }
        break;
      }
      case PUMP_ON:
      {
        (*_pumpOnOff) = true;
        next();
        break;
      }
      case PUMP_OFF:
      {
        (*_pumpOnOff) = false;
        next();
        break;
      }
    }
  }

  void next()
  {
    (*_step)++;
    if (_programs[*_program * MaxNumberOfSteps + *_step].opCode == WAIT)
    {
      _waitTime = *_currentTime;
      _waitTime.tm_min += _programs[*_program * MaxNumberOfSteps + *_step].minutes;
      mktime(&_waitTime);
    }
    if ((*_step) >= MaxNumberOfSteps)
    {
      (*_runProgram) = false;
    }
  }

  ProgramStep* _programs;
  byte* _step;
  byte* _program;

  double* _temperature;
  double* _setTemperature;
  bool* _heatOnOff;
  bool* _pumpOnOff;
  tm* _currentTime;
  tm _waitTime;

  bool* _runProgram;
};

const char* Interpreter::operations[OP_CODE_COUNT] = {
  "Nop", 
  "Heat", 
  "Cold", 
  "Wait", 
  "Pump On", 
  "Pump Off", 
};

///////////////////////////////////////////////////////////////////////
struct OperationsLine: Line
{
  OperationsLine(byte* operation)
    : _operation(operation)
  {
  }

  const char* str()
  {
    snprintf(buffer, sizeof(buffer), "%s", Interpreter::operations[*_operation]);
    return buffer;
  }

  bool update()
  {
    return timer.update();
  }

  byte* _operation;
  char buffer[17];
  MyTimer timer;  
};

///////////////////////////////////////////////////////////////////////
struct RunProgramLine: Line
{
  RunProgramLine(
    ProgramStep* programs,
    bool* runProgram,
    byte* program,
    byte* step,
    double* temperature,
    tm* currentTime,
    tm* nextTime,
    byte* heatPower,
    bool* pumpOnOff
    )
    : _programs(programs)
    , _runProgram(runProgram)
    , _program(program)
    , _step(step)
    , _temperature(temperature)
    , _currentTime(currentTime)
    , _nextTime(nextTime)
    , _heatPower(heatPower)
    , _pumpOnOff(pumpOnOff)
  {
  }

  const char* str()
  {
    return 0;
  }

  void updateBuffer()
  {
    char opcode[16];
    memset(opcode, 0, sizeof(opcode));
    snprintf(opcode, sizeof(opcode), "%s", Interpreter::operations[_programs[*_program * MaxNumberOfSteps + *_step].opCode]);

    char progPrm[32];
    memset(progPrm, 0, sizeof(progPrm));
    if (_programs[*_program * MaxNumberOfSteps + *_step].opCode == Interpreter::WAIT)
      snprintf(progPrm, sizeof(progPrm), "(%d min)%02d:%02d -> %02d:%02d, Temp %.1f%cC", 
        _programs[*_program * MaxNumberOfSteps + *_step].minutes, _currentTime->tm_hour, _currentTime->tm_min, _nextTime->tm_hour, _nextTime->tm_min, *_temperature, 0xDF);
    else if (_programs[*_program * MaxNumberOfSteps + *_step].opCode == Interpreter::HEAT)
      snprintf(progPrm, sizeof(progPrm), "%.1f%cC -> %d%cC", *_temperature, 0xDF, _programs[*_program * MaxNumberOfSteps + *_step].temperature, 0xDF);
    else if (_programs[*_program * MaxNumberOfSteps + *_step].opCode == Interpreter::COLD)
      snprintf(progPrm, sizeof(progPrm), "%d%cC <- %.1f%cC", _programs[*_program * MaxNumberOfSteps + *_step].temperature, 0xDF, *_temperature, 0xDF);

    char onOff[4];
    memset(onOff, 0, sizeof(onOff));
    snprintf(onOff, sizeof(onOff), "%s", (*_runProgram) ? "On" : "Off");

    char pumpOnOff[4];
    memset(pumpOnOff, 0, sizeof(pumpOnOff));
    snprintf(pumpOnOff, sizeof(pumpOnOff), "%s", (*_pumpOnOff) ? "On" : "Off");

    snprintf(buffer, sizeof(buffer), "   Pr %d(%s), S %d, %s, %s, Heat power %d, Pump %s   ", *_program, onOff, *_step, opcode, progPrm, * _heatPower, pumpOnOff);
  }

  void print(LiquidCrystal_I2C& lcd) {
    updateBuffer();
    for (byte i = 0; i < 16; ++i)
    {
      lcd.setCursor(i, 1);
      lcd.write(buffer[cp + i]);
    }
  }

  bool update()
  {
    bool u = timer.update();
    if (u)
    {
      cp++;
      if (cp >= size() - 16)
      {
        cp = 0;
      }
    }
    return u;
  }
  
  byte size()
  {
    for (uint8 i = 0; i < sizeof(buffer); ++i)
    {
      if (buffer[i] == 0)
      {
        return i;
      }
    }
  }
  
  char buffer[128];
  byte bufferStart;
  MyTimer timer;

  byte cp;

  ProgramStep* _programs;
  bool* _runProgram;
  byte* _program;
  byte* _step;
  double* _temperature;
  tm* _currentTime;
  tm* _nextTime;
  byte* _heatPower;
  bool* _pumpOnOff;
};

///////////////////////////////////////////////////////////////////////
struct ViewProgramLine: Line
{
  ViewProgramLine(
    ProgramStep* programs,
    byte* program
    )
    : _programs(programs)
    , _program(program)
  {
  }

  void updateBuffer()
  {
    char steps[MaxNumberOfSteps][32];
    for (uint8 i = 0; i < MaxNumberOfSteps; ++i)
    {
      memset(steps[i], 0, sizeof(steps[i]));
      if (false) {}
      else if (_programs[*_program * MaxNumberOfSteps + i].opCode == Interpreter::WAIT)
        snprintf(&steps[i][0], sizeof(steps[i]), "%d:Wait %d min", i, _programs[*_program * MaxNumberOfSteps + i].minutes);
      else if (_programs[*_program * MaxNumberOfSteps + i].opCode == Interpreter::HEAT)
        snprintf(&steps[i][0], sizeof(steps[i]), "%d:Heat %d%cC", i, _programs[*_program * MaxNumberOfSteps + i].temperature, 0xDF);
      else if (_programs[*_program * MaxNumberOfSteps + i].opCode == Interpreter::COLD)
        snprintf(&steps[i][0], sizeof(steps[i]), "%d:Cold %d%cC", i, _programs[*_program * MaxNumberOfSteps + i].temperature, 0xDF);
      else if (_programs[*_program * MaxNumberOfSteps + i].opCode == Interpreter::PUMP_ON)
        snprintf(&steps[i][0], sizeof(steps[i]), "%d:Pump On", i);
      else if (_programs[*_program * MaxNumberOfSteps + i].opCode == Interpreter::PUMP_OFF)
        snprintf(&steps[i][0], sizeof(steps[i]), "%d:Pump Off", i);
      else if (_programs[*_program * MaxNumberOfSteps + i].opCode == Interpreter::NOP)
        snprintf(&steps[i][0], sizeof(steps[i]), "%d", i);
    }

    snprintf(buffer, sizeof(buffer), "   %s->%s->%s->%s->%s->%s->%s->%s->%s->%s->%s->%s->%s->%s->%s->%s->%s->%s->%s->%s->   ", 
      steps[ 0], steps[ 1], steps[ 2], steps[ 3], steps[ 4], 
      steps[ 5], steps[ 6], steps[ 7], steps[ 8], steps[ 9], 
      steps[10], steps[11], steps[12], steps[13], steps[14], 
      steps[15], steps[16], steps[17], steps[18], steps[19]
      );
  }

  void print(LiquidCrystal_I2C& lcd) {
    updateBuffer();
    for (byte i = 0; i < 16; ++i)
    {
      lcd.setCursor(i, 1);
      lcd.write(buffer[cp + i]);
    }
  }

  bool update()
  {
    bool u = timer.update();
    if (u)
    {
      cp++;
      if (cp >= size() - 16)
      {
        cp = 0;
      }
    }
    return u;
  }

  byte size()
  {
    for (uint16 i = 0; i < 512; ++i)
    {
      if (buffer[i] == 0)
      {
        return i;
      }
    }
  }
  
  char buffer[512];
  uint16 bufferStart;
  MyTimer timer;

  byte cp;

  ProgramStep* _programs;
  byte* _program;
};

///////////////////////////////////////////////////////////////////////

const int LcdWidth = 16;
const int LcdHeight = 2;

const int RePin1 = PB8;
const int RePin2 = PB1;
const int ReButtonPin = PB9;

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
double temperature = 0.0;
bool temperatureSensorWork = false;
SingleMillisecondTimer temperatureTimer;

bool pumpWork = 0;
byte heatPower = 0;
PowerControl heat(HeatPin, &heatPower);

void resetHeat();

void onSetTempWorkUpdatet(void*, bool*);
Var<bool> setTempWork{false, onSetTempWorkUpdatet};
double setTemperature = 0;

float Kp = 0;
float Ki = 0;
float Kd = 0;

void checkAndUpdatePID();

double heatOutput = 0.f;

PID heatReg(&temperature, &heatOutput, &setTemperature, Kp, Ki, Kd, DIRECT);
MyTimer regulatorTimer(100);


bool reverseEncoder = false;

void onProgram(void*, byte*);
Var<byte> program(0, onProgram);

void onProgramStep(void*, byte*);
Var<byte> programStep(0, onProgramStep);
void onOperation(void*, byte*);
Var<byte> operation(0, onOperation);
ProgramStep programs[NumberOfPrograms * MaxNumberOfSteps];
void onRunProgram(void*,bool* value);
Var<bool> runProgram(false, onRunProgram);
tm currentTime;
Interpreter interpreter(
  programs, 
  program.ptr(), 
  programStep.ptr(), 
  &temperature, 
  &setTemperature, 
  setTempWork.ptr(), 
  &pumpWork, 
  runProgram.ptr(), 
  &currentTime);

RunProgramLine runProgramLine(programs, runProgram.ptr(), program.ptr(), programStep.ptr(), &temperature, &currentTime, &interpreter._waitTime, &heatPower, &pumpWork);
ViewProgramLine viewProgramLine(programs, program.ptr());

OperationsLine opCodeLine(operation.ptr());

void onClearProgram(void*, bool* value);
Var<bool> clearProgram(false, onClearProgram);

void onClearAllProgram(void*, bool* value);
Var<bool> clearAllProgram(false, onClearAllProgram);

void onProgramTempUpdate(void*, byte* value);
Var<byte> programTemp(20, onProgramTempUpdate);

void onProgramMinutesUpdate(void*, byte* value);
Var<byte> programMinutes(0, onProgramMinutesUpdate);

Var<byte> bounceDelay(1);

void onSavePrograms(void*, bool*);
Var<bool> savePrograms(false, onSavePrograms);

void onSaveSettings(void*, bool*);
Var<bool> saveSettings(false, onSaveSettings);

void onOperation(void*, byte*)
{
  programs[program.value * MaxNumberOfSteps + programStep.value].opCode = operation.value;
}

void onProgramTempUpdate(void*, byte* value)
{
  programs[program.value * MaxNumberOfSteps + programStep.value].temperature = programTemp.value;
}

void onProgramMinutesUpdate(void*, byte* value)
{
  programs[program.value * MaxNumberOfSteps + programStep.value].minutes = programMinutes.value;
}

void onProgramStep(void*, byte* s)
{
  programTemp.value = programs[program.value * MaxNumberOfSteps + programStep.value].temperature;
  programMinutes.value = programs[program.value * MaxNumberOfSteps + programStep.value].minutes;
  operation.value = programs[program.value * MaxNumberOfSteps + programStep.value].opCode;
  runProgramLine.cp = 0;
}

      /*
      { "id": "Test", "type": "info", "args": "\"Test\", 0, 0", "menu": [
        { "id": "Test", "type": "edit", "ctype": "bool", "args": "\"Test\", testBool.ptr(), 1, 0, 1" }
      ]},
      */
void onTest(void*, bool*);
Var<bool> testBool(false, onTest);

Updater* updaters[] = {
  &program,
  &setTempWork,
  &programStep,
  &operation,
  &runProgram,
  &clearProgram,
  &clearAllProgram,
  &programTemp,
  &programMinutes,
  &bounceDelay,
  &savePrograms,
  &saveSettings,
  &testBool,  
  0
};

void loadConfig();
void saveConfig();

#include "Menu.h"

MenuItem* current = &Welcome_Info;

Button button;
RotaryEncoder encoder;

HardWire i2c(2, I2C_FAST_MODE);

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

void updateAll();
void updateCurrentTime();

void setup()
{
  Serial.begin(115200);

  i2c.begin();  
  loadConfig();

  button.init(ReButtonPin, downHandler, 0, bounceDelay.ptr());
  encoder.init(RePin1, RePin2, plusHandler, minusHandler, bounceDelay.ptr());

  lcd.begin();

  setupTemperature();
  sensors.requestTemperatures();

  heatReg.SetOutputLimits(0, 100);
  heatReg.SetMode(AUTOMATIC);

  loadTime();

  tone(TonePin, 1000, 200);
  delay(200);
  noTone(TonePin);

  pinMode(ZeroSensorPin, INPUT);
  attachInterrupt(ZeroSensorPin, zeroCross, RISING);

  pinMode(PumpPin, OUTPUT);
  digitalWrite(PumpPin, LOW);

  onProgramStep(0, 0);
}

void loop()
{  
  updateAll();
  checkAndUpdatePID();

  updateCurrentTime();

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
    Serial.print(heatPower); Serial.print(" ");
    Serial.println("");

    temperatureTimer.start(750 / (1 << (12 - resolution)));
    
    sensors.requestTemperatures();
  }

  if (setTempWork.value && regulatorTimer.update())
  {
    heatReg.Compute();
    heatPower = byte(heatOutput);
  }

  if (runProgram.value)
  {
    interpreter.update();
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

void zeroCross() {
  heat.update();
}

void setupTemperature() {
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

void resetHeat() {
  heat.reset();
}

void downHandler(void*) {
  #include "BtnHandlers.h"
  if (current)
      current->HandleButton();
    dataChanged = true;
}

void plus(void*) {
  #include "UpHandlers.h"
  if (current)
      current->HandleEncoderPlus();
  dataChanged = true;
}


void minus(void*) {
  #include "DwnHandlers.h"
  if (current)
      current->HandleEncoderMinus();
  dataChanged = true;
}

void minusHandler(void*) {
  if (reverseEncoder)
    plus(0);
  else
    minus(0);
}

void plusHandler(void*) {
  if (reverseEncoder)
    minus(0);
  else
    plus(0);
}

void saveTime() {
    tm* tx;
    tx = rt.getTime(NULL);

    tx->tm_min  = minute;
    tx->tm_hour = hour;

    tx->tm_mday = day;
    tx->tm_mon  = month;
    tx->tm_year = year - 1900;
    
    rt.setTime(tx);
}

void loadTime() {
    tm* tx;
    tx = rt.getTime(NULL);

    minute  = tx->tm_min;
    hour    = tx->tm_hour;

    day     = tx->tm_mday;
    month   = tx->tm_mon;
    year    = 1900 + tx->tm_year;
}

void checkAndUpdatePID() {
  const double EPS = 1e-5;
  if (
    fabs(Kp - heatReg.GetKp()) > EPS || 
    fabs(Ki - heatReg.GetKi()) > EPS || 
    fabs(Kd - heatReg.GetKd()) > EPS)
  {
    heatReg.SetTunings(Kp, Ki, Kd);
  }
}

void updateAll() {
  for (byte i = 0;; ++i)
  {
    Updater* u = updaters[i];
    if (u == 0)
      break;
    u->update();
  }
}

void onSetTempWorkUpdatet(void*, bool* value) {
  if (*value)
  {
  }
  else
  {
    heatPower = 0;
    heatOutput = 0;
  }
}

void onRunProgram(void*,bool* value) {
  programStep.value = 0;
  if (*value)
  {
    programStep.value = 0;
  }
  else
  {
    setTempWork.value = false;
    pumpWork = false;

    tone(TonePin, 1000, 200);
    delay(100);
    tone(TonePin, 500, 200);
    delay(100);
    tone(TonePin, 1000, 200);
    delay(100);
    tone(TonePin, 500, 200);
    delay(100);
    tone(TonePin, 1000, 200);
    delay(100);
    noTone(TonePin);
  }
}

void onClearProgram(void*, bool* value) {
  for (byte s = 0; s < MaxNumberOfSteps; ++s)
  {
    programs[program.value * MaxNumberOfSteps + s].temperature = 0;
    programs[program.value * MaxNumberOfSteps + s].opCode = 0;
  }
  clearProgram.value = false;
  clearProgram.prevValue = false;
  dataChanged = true;
}

void onClearAllProgram(void*, bool* value) {
  for (byte p = 0; p < NumberOfPrograms; ++p)
  {
    for (byte s = 0; s < MaxNumberOfSteps; ++s)
    {
      programs[p * MaxNumberOfSteps + s].temperature = 0;
      programs[p * MaxNumberOfSteps + s].opCode = 0;
    }    
  }
  clearAllProgram.value = false;
  clearAllProgram.prevValue = false;
  dataChanged = true;
  saveConfigWithPrograms();
}

void writeEeprom(int deviceaddress, byte eeaddress, byte data) {
  i2c.beginTransmission(deviceaddress);
  i2c.write(eeaddress);
  i2c.write(data);
  i2c.endTransmission();
  delay(5);
}
 
byte readEeprom(int deviceaddress, byte eeaddress) {
  byte rdata = 0xFF;
  i2c.beginTransmission(deviceaddress);
  i2c.write(eeaddress);
  i2c.endTransmission();
 
  i2c.requestFrom(deviceaddress, 1);
 
  if (i2c.available()) rdata = i2c.read();
  return rdata;
}

void writeData(int address, byte data) {
  int device = address / 256;
  int addr = address % 256;
  writeEeprom(0x50 + device, addr, data); 
}

byte readData(int address) {
  int device = address / 256;
  int addr = address % 256;
  return readEeprom(0x50 + device, addr);
}

template<typename V> void Save(unsigned int& offset, const V& value) {
  for (unsigned int i = 0; i < sizeof(value); ++i)
    writeData(offset + i, *((char*)&value + i));
  offset += sizeof(value);
}

template<typename V> void Load(unsigned int& offset, V& v) {
  for (unsigned int i = 0; i < sizeof(v); ++i)
    *((char*)&v + i) = readData(offset + i);
  offset += sizeof(v);
}

void saveConfigDetail (bool savePrograms) {
  unsigned int offset = 0;
  Save(offset, Kp);
  Save(offset, Ki);
  Save(offset, Kd);
  Save(offset, reverseEncoder);
  Save(offset, setTemperature);
  Save(offset, bounceDelay.value);
  Save(offset, program.value);
  if (savePrograms)
  {
    Save(offset, programs);
  }
}

void saveConfig() {
  saveConfigDetail(false);
}

void saveConfigWithPrograms() {
  saveConfigDetail(true);
}

void loadConfig() {
  unsigned int offset = 0;
  Load(offset, Kp); 
  Load(offset, Ki); 
  Load(offset, Kd); 
  Load(offset, reverseEncoder); 
  Load(offset, setTemperature); 
  Load(offset, bounceDelay.value); 
  Load(offset, program.value);
  Load(offset, programs);

  checkAndUpdatePID();
}

void onTest(void*, bool* value)
{
}

void onSavePrograms(void*, bool*) {
  saveConfigWithPrograms();
  savePrograms.value = false;
  savePrograms.prevValue = false;
  dataChanged = true;
}

void onSaveSettings(void*, bool*) {
  saveConfig();
  saveSettings.value = false;
  saveSettings.prevValue = false;
  dataChanged = true;
}

void updateCurrentTime() {
  currentTime = *rt.getTime(NULL);
}

void onProgram(void*, byte*) {
  viewProgramLine.cp = 0;
}
