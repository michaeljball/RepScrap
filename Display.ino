
void updateDisplay() {            // update LCD readout
    if (doOutput){
      doOutput=false;

   // For diagnostic  comment out later
    Serial.print("X, "); Serial.print(XaxisSetpoint);Serial.print(", "); Serial.print(XaxisPos);Serial.print(", "); Serial.print(XaxisSpd);
    Serial.print(", "); Serial.print(KpX); Serial.print(", "); Serial.print(KiX); Serial.print(", "); Serial.print(KdX);
    Serial.print(", "); Serial.print(millis()-timeToAcquire);Serial.print("\r\n");
    Serial.print("Y, "); Serial.print(YaxisSetpoint);Serial.print(", "); Serial.print(YaxisPos);Serial.print(", "); Serial.print(YaxisSpd);
    Serial.print(", "); Serial.print(KpY); Serial.print(", "); Serial.print(KiY); Serial.print(", "); Serial.print(KdY);
    Serial.print(", "); Serial.print(millis()-timeToAcquire);Serial.print("\r\n");
      
      lcd.setCursor(0, 0);
      lcd.print("XTarget "); lcd.print(XaxisSetpoint,2);lcd.print("       ");
      lcd.setCursor(0, 1);
      sprintf(disbuffer, "%6d", rtX);

      lcd.print("XPosition "); lcd.print(disbuffer); 
      
      lcd.setCursor(0, 2);
      digitalClockDisplay(); 
          
    }
}

void displaySetpoints() {
   Serial.print("XaxisSetpoint = "); Serial.print(XaxisSetpoint); Serial.print("\n\r");
   Serial.print("YaxisSetpoint = "); Serial.print(YaxisSetpoint); Serial.print("\n\r");
   lcd.setCursor(0, 3);
   lcd.print("X = "); lcd.print(XaxisSetpoint); lcd.print(", ");
   lcd.print("Y = "); lcd.print(YaxisSetpoint); 
}


void digitalClockDisplay() {
  // digital clock display of the time
  lcd.print(hour());
  printDigits(minute());
  printDigits(second());
  lcd.print(" ");
  lcd.print(month());
  lcd.print(" ");
  lcd.print(day());
  lcd.print(" ");
  lcd.print(year());
}


void printDigits(int digits){
// utility function for digital clock display: prints preceding colon and leading 0
  lcd.print(":");
  if(digits < 10) lcd.print('0');
  lcd.print(digits);
}

