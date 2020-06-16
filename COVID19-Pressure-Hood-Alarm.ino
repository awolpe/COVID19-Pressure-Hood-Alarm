#include "pHoodAlarm.h"

void setup() {
  
  
  Initialize_Alarm();
  Initialize_HMI()

  

  // Start BME sensor
  if (!bme.begin()) {
    Display(255); // Display missing sensor view on LCD
    PRINTS("Could not find a valid BME680 sensor, check wiring!\n");
    while (!bme.begin()) {
      delay(1000);
      PRINTS(".");
    }
    Display(254); // Display sensor detected view on LCD
    delay(1000); // Wait a sec for user to see notification
  }

  //Zero Pressure Sensors
  if (zeroOnBoot) Zero_Sensors();

  

  
  
  digitalWrite(goodLED, true);
  for(int i=0; i<5; i++){
    digitalWrite(alarmPin, true);
    delay(100);
    digitalWrite(alarmPin, false);
    delay(100);
  }

}

void loop() {

  if (! bme.performReading()) {
    Serial.println("BME failed to perform reading :(");
    return;
  }

//  if (Serial.available() > 0) {
//
//    String CMD = "";
//    CMD = Serial.readStringUntil("\n");
//    CMD.trim();
//    CMD.toLowerCase();
//    if (CMD == "zero") Zero_Sensors();
//    PRINTS("----------------------------------------------\n\n");
//  }

  if(zeroSensors_Flag) {
    Zero_Sensors();
    zeroSensors_Flag = false;
  }

  pGauge_BME = ( ( bme.pressure / 100.0 ) * hPa_to_cmH20 ) - pOffset_BME; //Get pressure reading and convert to cmH20, and subtract offset for gauge pressure
  float Vout = ( ads.readADC_Differential_0_1() * ads_gain );
  pGauge_MPX = ( ( Vout - 2.5 ) * ( hPa_to_cmH20 * 10 )) - pOffset_MPX;

  MeasFreq();

  PRINT( "BME Pressure = ", String( pGauge_BME, 3 ) );
  PRINTS(" cmH2O \n");
  PRINT( "MPX Pressure = ", String( pGauge_MPX, 2 ) );
  PRINTS(" cmH2O\n");

  PRINT( "MPX (ADC) = ", String( Vout, 2 ) );
  PRINTS( " Vdc\n\n" );

  PRINT( "Temperature = ", String( bme.temperature, 3 ) );
  PRINTS( " C \n" );

  PRINT( "Humidity = ", String( bme.humidity, 1 ) );
  PRINTS(" % \n");

  PRINT( "Freq = ", String( freq, 2 ) );
  PRINTS( "Hz \n" );
  PRINT( "Avg Freq = ", String( avgFreq, 2 ) );
  PRINTS( "Hz \r\n" );

#if DEBUG == 0
  Serial.print( String(pGauge_BME, 3) + ',' + String(bme.temperature, 1) + ',' + String(bme.humidity, 1) + ',' );
  Serial.print( String(pGauge_MPX, 3) + ',' + String(Vout, 3)  + ',' /* + String(freq, 2) + ',' + String(avgFreq) + ','*/ );
  Serial.print( String(pOffset_BME, 3) + ',' + String(pOffset_MPX, 3) + "\r\n" );
#endif


  bool enableSet;
  EEPROM.get(9, enableSet);
  if (enableSet && AlarmCheck() ) {
    Alarm = true;
    digitalWrite(goodLED, false);
    digitalWrite(alarmPin, true);
    Display(253);
  }
  else {
    digitalWrite(goodLED, true);
    digitalWrite(alarmPin, false);
    // Display data on LCD screen
    Display(UI_Index);
  }

  // Keep loop time as deterministic as possible to 500ms iterations
  unsigned long dt = millis() - millisLast;
  PRINT( "dt = ", String(dt) );
  PRINTS(" ms \n\n");
  if (500 > dt) delay(500 - dt);
  millisLast = millis();


}
