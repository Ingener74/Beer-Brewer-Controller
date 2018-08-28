
// up handler
do{
if (current == &Welcome_Info) { current = &Program_Info; break; }
if (current == &Program_Info) { current = &Manual_Info; break; }    
    if (current == &Program_back_Info) { current = &SelectProgram_Info; break; }
    if (current == &SelectProgram_Info) { current = &EditProgram_Info; break; }    
        if (current == &SelectProgram_Editor) { break; }
    if (current == &EditProgram_Info) { current = &RunProgram_Info; break; }    
        if (current == &EditProgram_back_Info) { current = &SelectStep_Info; break; }
        if (current == &SelectStep_Info) { current = &SelectOperation_Info; break; }    
            if (current == &SelectStep_Editor) { break; }
        if (current == &SelectOperation_Info) { current = &EditTemperature_Info; break; }    
            if (current == &SelectOperation_Editor) { break; }
        if (current == &EditTemperature_Info) { current = &EditMinutes_Info; break; }    
            if (current == &EditTemperature_Editor) { break; }
        if (current == &EditMinutes_Info) { current = &SavePrograms_Info; break; }    
            if (current == &EditMinutes_Editor) { break; }
        if (current == &SavePrograms_Info) { break; }    
            if (current == &SavePrograms_Editor) { break; }
    if (current == &RunProgram_Info) { current = &ClearProgram_Info; break; }    
        if (current == &RunProgram_Editor) { break; }
    if (current == &ClearProgram_Info) { current = &ClearAllProgram_Info; break; }    
        if (current == &ClearProgram_Editor) { break; }
    if (current == &ClearAllProgram_Info) { break; }    
        if (current == &ClearAllProgram_Editor) { break; }
if (current == &Manual_Info) { current = &Settings_Info; break; }    
    if (current == &Manual_back_Info) { current = &Temperature_Info; break; }
    if (current == &Temperature_Info) { current = &TempSensorWork_Info; break; }    
        if (current == &TemperatureViewer_Viewer) { break; }
    if (current == &TempSensorWork_Info) { current = &Pump_Info; break; }    
        if (current == &TempSensorWorkViewer_Viewer) { break; }
    if (current == &Pump_Info) { current = &Heat_Info; break; }    
        if (current == &Pump_Editor) { break; }
    if (current == &Heat_Info) { current = &SetTemp_Info; break; }    
        if (current == &Heat_Editor) { break; }
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
            if (current == &Year_Editor) { break; }
        if (current == &Month_Info) { current = &Day_Info; break; }    
            if (current == &Month_Editor) { break; }
        if (current == &Day_Info) { current = &Hour_Info; break; }    
            if (current == &Day_Editor) { break; }
        if (current == &Hour_Info) { current = &Minute_Info; break; }    
            if (current == &Hour_Editor) { break; }
        if (current == &Minute_Info) { break; }    
            if (current == &Minute_Editor) { break; }
    if (current == &Kp_Info) { current = &Ki_Info; break; }    
        if (current == &Kp_Editor) { break; }
    if (current == &Ki_Info) { current = &Kd_Info; break; }    
        if (current == &Ki_Editor) { break; }
    if (current == &Kd_Info) { current = &ReverseEnc_Info; break; }    
        if (current == &Kd_Editor) { break; }
    if (current == &ReverseEnc_Info) { current = &BounceDelay_Info; break; }    
        if (current == &ReverseEnc_Editor) { break; }
    if (current == &BounceDelay_Info) { current = &SaveSettings_Info; break; }    
        if (current == &BounceDelay_Editor) { break; }
    if (current == &SaveSettings_Info) { break; }    
        if (current == &SaveSettings_Editor) { break; }
}while(false);