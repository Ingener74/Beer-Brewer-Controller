
MenuInfo Welcome_Info("Hello, Brewer", 0, &timeLine);
MenuInfo Program_Info("Program", 0, 0);    
    MenuInfo Program_back_Info("<-", 0, 0);
    MenuInfo SelectProgram_Info("Select program", 0, &viewProgramLine);    
        MenuEditor<byte> SelectProgram_Editor("Select program", program.ptr(), 1, 0, NumberOfPrograms - 1);
    MenuInfo EditProgram_Info("Edit Program", 0, 0);    
        MenuInfo EditProgram_back_Info("<-", 0, 0);
        MenuInfo SelectStep_Info("Select Step", 0, 0);    
            MenuEditor<byte> SelectStep_Editor("Select Step", programStep.ptr(), 1, 0, MaxNumberOfSteps - 1);
        MenuInfo SelectOperation_Info("Select Operation", 0, 0);    
            MenuEditor<byte> SelectOperation_Editor("Select Operation", operation.ptr(), 1, 0, Interpreter::OP_CODE_COUNT - 1, 0, &opCodeLine);
        MenuInfo EditTemperature_Info("Temperature", 0, 0);    
            MenuEditor<byte> EditTemperature_Editor("Temperature", programTemp.ptr(), 1, 0, 110);
        MenuInfo EditMinutes_Info("Minutes", 0, 0);    
            MenuEditor<byte> EditMinutes_Editor("Minutes", programMinutes.ptr(), 1, 0, 255);
        MenuInfo SavePrograms_Info("Save programs", 0, 0);    
            MenuEditor<bool> SavePrograms_Editor("Save programs", savePrograms.ptr(), 1, 0, 1);
    MenuInfo RunProgram_Info("Run program", 0, &runProgramLine);    
        MenuEditor<bool> RunProgram_Editor("Run program", runProgram.ptr(), 1, 0, 1);
    MenuInfo ClearProgram_Info("Clear program", 0, 0);    
        MenuEditor<bool> ClearProgram_Editor("Clear program", clearProgram.ptr(), 1, 0, 1);
    MenuInfo ClearAllProgram_Info("Clear all program", 0, 0);    
        MenuEditor<bool> ClearAllProgram_Editor("Clear all program", clearAllProgram.ptr(), 1, 0, 1);
MenuInfo Manual_Info("Manual Control", 0, 0);    
    MenuInfo Manual_back_Info("<-", 0, 0);
    MenuInfo Temperature_Info("Temperature", 0, 0);    
        MenuViewer<double> TemperatureViewer_Viewer("Temperature", &temperature);
    MenuInfo TempSensorWork_Info("Temp Sensor Work", 0, 0);    
        MenuViewer<bool> TempSensorWorkViewer_Viewer("Temp Sensor Work", &temperatureSensorWork);
    MenuInfo Pump_Info("Pump", 0, 0);    
        MenuEditor<bool> Pump_Editor("Pump", &pumpWork, 1, 0, 1);
    MenuInfo Heat_Info("Heat Power", 0, 0);    
        MenuEditor<byte> Heat_Editor("Heat Power", &heatPower, 2, 0, 100, resetHeat);
    MenuInfo SetTemp_Info("Set temperature", 0, 0);    
        MenuInfo SetTemp_back_Info("<-", 0, 0);
        MenuInfo SetTempOn_Info("On/Off", 0, 0);    
            MenuEditor<bool> SetTempOn_Editor("On/Off", setTempWork.ptr(), 1, 0, 1);
        MenuInfo SetTemperature_Info("Set temperature", 0, 0);    
            MenuEditor<double> SetTemperature_Editor("Set temperature", &setTemperature, 5, 0, 110);
MenuInfo Settings_Info("Settings", 0, 0);    
    MenuInfo Settings_back_Info("<-", 0, 0);
    MenuInfo SetTime_Info("Set Time", 0, 0);    
        MenuInfo SetTime_back_Info("<-", 0, 0);
        MenuInfo Year_Info("Year", 0, 0);    
            MenuEditor<int> Year_Editor("Year", &year, 1, 1970, 2100, saveTime);
        MenuInfo Month_Info("Month", 0, 0);    
            MenuEditor<byte> Month_Editor("Month", &month, 1, 1, 12, saveTime);
        MenuInfo Day_Info("Day", 0, 0);    
            MenuEditor<byte> Day_Editor("Day", &day, 1, 0, 31, saveTime);
        MenuInfo Hour_Info("Hour", 0, 0);    
            MenuEditor<byte> Hour_Editor("Hour", &hour, 1, 0, 23, saveTime);
        MenuInfo Minute_Info("Minute", 0, 0);    
            MenuEditor<byte> Minute_Editor("Minute", &minute, 1, 0, 59, saveTime);
    MenuInfo Kp_Info("Kp", 0, 0);    
        MenuEditor<float> Kp_Editor("Kp", &Kp, 0.1, 0, 100);
    MenuInfo Ki_Info("Ki", 0, 0);    
        MenuEditor<float> Ki_Editor("Ki", &Ki, 0.0001, 0, 100);
    MenuInfo Kd_Info("Kd", 0, 0);    
        MenuEditor<float> Kd_Editor("Kd", &Kd, 0.001, 0, 100);
    MenuInfo ReverseEnc_Info("Reverse Encoder", 0, 0);    
        MenuEditor<bool> ReverseEnc_Editor("Reverse Encoder", &reverseEncoder, 1, 0, 1);
    MenuInfo BounceDelay_Info("Bounce delay", 0, 0);    
        MenuEditor<byte> BounceDelay_Editor("Bounce delay", bounceDelay.ptr(), 1, 0, 10);
    MenuInfo SaveSettings_Info("Save settings", 0, 0);    
        MenuEditor<bool> SaveSettings_Editor("Save settings", saveSettings.ptr(), 1, 0, 1);