
MenuInfo Welcome_Info("Hello, Brewer", 0, &timeLine);
MenuInfo Program_Info("Program", 0, 0);
MenuInfo Manual_Info("Manual Control", 0, 0);    
    MenuInfo Manual_back_Info("<-", 0, 0);
    MenuInfo Temperature_Info("Temperature", 0, 0);    
        MenuViewer<float> TemperatureViewer_Viewer("Temperature", &temperature);
    MenuInfo TempSensorWork_Info("Temp Sensor Work", 0, 0);    
        MenuViewer<bool> TempSensorWorkViewer_Viewer("Temp Sensor Work", &temperatureSensorWork);
    MenuInfo Pump_Info("Pump Power", 0, 0);    
        MenuEditor<bool> PumpEditor_Editor("Pump", &pumpWork, 1, 0, 1);
    MenuInfo Heat_Info("Heat Power", 0, 0);    
        MenuEditor<int> HeatEditor_Editor("Heat Power", &heatPower, 2, 0, 100);
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
        MenuEditor<float> KpEditor_Editor("Kp", &Kp, 0.1, 0, 100);
    MenuInfo Ki_Info("Ki", 0, 0);    
        MenuEditor<float> KiEditor_Editor("Ki", &Ki, 0.1, 0, 100);
    MenuInfo Kd_Info("Kd", 0, 0);    
        MenuEditor<float> KdEditor_Editor("Kd", &Kd, 0.1, 0, 100);
    MenuInfo ReverseEnc_Info("Reverse Encoder", 0, 0);    
        MenuEditor<bool> ReverseEnc_Editor("Reverse Encoder", &reverseEncoder, 1, 0, 1);

// button handler
do{
// if (current == &Welcome_Info) {}
// if (current == &Program_Info) {}
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
if (current == &Settings_Info) { current = &Settings_back_Info; break; }    
    if (current == &Settings_back_Info) { current = &Settings_Info; break; }
    if (current == &SetTime_Info) { current = &Settings_back_Info; break; }    
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

// up handler
do{
if (current == &Welcome_Info) { current = &Program_Info; break; }
if (current == &Program_Info) { current = &Manual_Info; break; }
if (current == &Manual_Info) { current = &Settings_Info; break; }    
    if (current == &Manual_back_Info) { current = &Temperature_Info; break; }
    if (current == &Temperature_Info) { current = &TempSensorWork_Info; break; }    
        if (current == &TemperatureViewer_Viewer) { break; }
    if (current == &TempSensorWork_Info) { current = &Pump_Info; break; }    
        if (current == &TempSensorWorkViewer_Viewer) { break; }
    if (current == &Pump_Info) { current = &Heat_Info; break; }    
        if (current == &PumpEditor_Editor) { break; }
    if (current == &Heat_Info) { break; }    
        if (current == &HeatEditor_Editor) { break; }
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

// down handler
do{
if (current == &Welcome_Info) { break; }
if (current == &Program_Info) { current = &Welcome_Info; break; }
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