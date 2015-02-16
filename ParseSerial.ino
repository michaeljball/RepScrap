
/***************************************************************************************************************
*
*  ParseSerialData() allows the modification of the PID parameters as well as selecting new target 
*  
***************************************************************************************************************/

void ParseSerialData()
{

  char *p = inData;                // The data to be parsed
  char *str;                       // Temp store for each data chunk
  int count = 0;                   // Id ref for each chunk
    
  while ((str = strtok_r(p, ",", &p)) != NULL)    // Loop through the data and seperate it into chunks at each "," delimeter
  { 
    inParse[count] = str;      // Add chunk to array  
    count++;      
  }

  if(count > 1)     // If the data has two values then..  
  {
    // Define value 1 as a Command identifier
    char *Command = inParse[0];
    // Define value 2 as a Parameter value
    char *Parameter = inParse[1];

  Serial.print("CMD,"); Serial.print(Command); Serial.print(","); Serial.print(Parameter); Serial.print(","); Serial.println("0");
    
    // Call the relevant identified Commandtion  
    switch(*Command)
    {

      case 'p':                                                         // Set 'Proportional'
        KpX = atof(Parameter);       
        KpY = atof(Parameter);       
        Serial.print("KpX = "); Serial.print(KpX); Serial.print("\n\r");
        lcd.setCursor(0, 3);
        lcd.print("KpX = "); lcd.print(KpX); lcd.print("    ");
        break;

      case 'i':                                                         // Set "Integral'
        KiX = atof(Parameter);       
        KiY = atof(Parameter);       
        Serial.print("KiX = "); Serial.print(KiX); Serial.print("\n\r");
        lcd.setCursor(0, 3);
        lcd.print("KiX = "); lcd.print(KiX); lcd.print("    ");
        break;

      case 'd':                                                         // Set 'Derivative'
        KdX = atof(Parameter);       
        KdY = atof(Parameter);       
        Serial.print("KdX = "); Serial.print(KdX); Serial.print("\n\r");
        lcd.setCursor(0, 3);
        lcd.print("KdX = "); lcd.print(KdX); lcd.print("    ");
        break;

      case 'n':                                                         // New Setpoint
        XaxisSetpoint = atof(Parameter);       
        YaxisSetpoint = atof(Parameter);       
        Serial.print("XaxisSetpoint = "); Serial.print(XaxisSetpoint); Serial.print("\n\r");
        lcd.setCursor(0, 3);
        lcd.print("Setpoint = "); lcd.print(XaxisSetpoint); lcd.print("    ");
        break;

      case 's':                                                         // Set MAX Speed
        Spd = atoi(Parameter);   
        XaxisPID.SetOutputLimits(0-Spd,Spd);           
        YaxisPID.SetOutputLimits(0-Spd,Spd);           
    
        Serial.print("Max Spd = "); Serial.print(Spd); Serial.print("\n\r");
        lcd.setCursor(0, 3);
        lcd.print("Spd = "); lcd.print(Spd); lcd.print("    ");
        break;

      case 'r':                                                         // Randomize Target
        mode = 0;   
        Serial.print("Mode = Random \n\r");
        lcd.setCursor(0, 3);
        lcd.print("Mode = Random         "); 
        break;

      case 'l':                                                         // Randomize Target
        mode = 1;   
        XaxisSetpoint = 100;         
        linearStep = atoi(Parameter);   
        Serial.print("Mode = Linear Step ="); Serial.print(linearStep); Serial.print("\n\r");
        lcd.setCursor(0, 3);
        lcd.print("Linear Step = "); lcd.print(linearStep);
        break;

     }    
  }
  
}


void SerialEvent2() 
{
  while (Serial.available() && stringComplete == false)    // Read while we have data
  {
    char inChar = Serial.read();             // Read a character
    inData[chindex] = inChar;                  // Store it in char array
    chindex++;                                 // Increment where to write next  
    inString += inChar;                      // Also add it to string storage just in case, not used yet :)
    
    if (inChar == '\n' || inChar == '\r')                      // Check for termination character
    {
      chindex = 0;
      stringComplete = true;
    }
  }
}

