#include "PinChangeInterrupt.h"
#include <Wire.h>
#include <SPI.h>
#include <Adafruit_Sensor.h>
#include "Adafruit_BME680.h"
#include <Adafruit_ADS1015.h>
#include <EEPROM.h>
#include <AltSoftSerial.h>



// Enables serial output of data needed for debugging only
#define DEBUG 0

// Define debugging functions
#if DEBUG
// If debugging is on, these functions print the passed data to serial output
#define PRINTS(s)   { Serial.print(F(s)); }                  // Prints static text only. The 'F()' functions stores the string in non-volitile memory to save RAM
#define PRINT(s,v)  { Serial.print(F(s)); Serial.print(v); } // Prints static text, followed by any dyncamic text passed in runtime. The 'F()' function for static text saves RAM 
#else
// If debugging is off, these functions will do nothing
#define PRINTS(s)
#define PRINT(s,v)
#endif

// Define pin numbers for interrupts
#define leftBtnPin  A0
#define midBtnPin  A1
#define rightBtnPin  A2
#define alarmPin  2 // Next version should be pin 3
#define flowSensorPin  3
#define WDPin  4
#define goodLED 8


// LCD Display
#define NUM_VIEWS 6 // Total number of unique views that can be display 
AltSoftSerial  OpenLCD; // Using the Alt Software Serial library to comm. with LCD. Most efficient alternative to hardware serial.
volatile byte displayIndex = 0; // Index that indicates what view to display on LCD
volatile bool Selected = false; // False = button presses will change screen views, True = button presses will change selection value\

// uS timestamp of when the button was pushed last (for debouncing)
volatile unsigned long lastPush_mBtn = 0;
volatile unsigned long lastPush_lBtn = 0;
volatile unsigned long lastPush_rBtn = 0;

// Flow sensor
#define FREQ_BUFFER_SIZE 60 // Size of the buffer/array to store frequency measurements to calculate average freq
float freq = 0, avgFreq = 0, freqBuffer[FREQ_BUFFER_SIZE];
int bufferIndex = 0; // Variable to keep track of which index to replace in the buffer. (Circular Buffer)

// BME Sensor
Adafruit_BME680 bme;
float pOffset_BME = 984.9506; // BME pressure, intialiized to past measurement as approx barometric pressure in Montrose, CA (cmH2O).
float pGauge_BME = 0; //Initialize variable for gauge pressure measurement (cmH2O)
//MPX Sensor
float pOffset_MPX = 0; // MPX sensor offset (initialized to approx default value from past measurement)(cmH2O)
float pGauge_MPX = 0; // Initialize variable for gauge pressure measurement (cmH2O)
Adafruit_ADS1115 ads; // Driver instance for ADS1115 ADC, used to measure analog voltage signal of MPX sensor

// Scaling/conversion factors
const float hPa_to_cmH20 = 1.01971621; // Coefficient to convert pressure from the BME's native hPa units to cmH20
const int ads_gainSetting = GAIN_ONE; // ADS1115  @ +/- 4.096V gain (16-bit results)
float ads_gain;

// Stores the time (in ms) that the last loop iteration finished
unsigned long millisLast = 0;

// Alarm settings
volatile bool Alarm = false;
volatile bool alarmEnable = false;
volatile float pressAlarm_Lo = 4; //cmH2O
volatile float pressAlarm_Hi = 20; //cmH2O

bool zeroOnBoot = true;
volatile bool zeroSensors_Flag = false;

void setup() {

  // Set pin modes
  pinMode(flowSensorPin,  INPUT);
  pinMode(rightBtnPin,    INPUT);
  pinMode(leftBtnPin,     INPUT);
  pinMode(alarmPin,       OUTPUT);
  pinMode(WDPin,          OUTPUT);
  pinMode(goodLED,        OUTPUT);

  digitalWrite(alarmPin, false);
  digitalWrite(WDPin, true);

  //Start communication with LCD
  OpenLCD.begin(9600); // Initialize LCD
  OpenLCD.write('|'); // Put LCD into setting mode
  OpenLCD.write(0x0F); // Set Baud to 19200( 0x0B=2400,0x0B=2400,0x0B=2400,0x0B=2400,0x0B=2400,0x0B=2400,
  OpenLCD.flush();
  
  OpenLCD.begin(19200); // Initialize LCD
  OpenLCD.write('|'); // Put LCD into setting mode
  OpenLCD.write(24); // Send contrast command
  OpenLCD.write(2); // Send contrast command

  // Start serial communication with host for debugging/develoment
  Serial.begin(57600);

  byte ConfigExistFlag;
  EEPROM.get(0, ConfigExistFlag);
  if ( ConfigExistFlag != 0x22 ) {
    EEPROM.put(1, pressAlarm_Lo);
    EEPROM.put(5, pressAlarm_Hi);
    EEPROM.put(9, alarmEnable);
    EEPROM.put(13, pOffset_BME);
    EEPROM.put(17, pGauge_MPX);
    EEPROM.put(18, zeroOnBoot);
    EEPROM.put(0, 0x22);
  }
  else {
    EEPROM.get(1, pressAlarm_Lo);
    EEPROM.get(5, pressAlarm_Hi);
    EEPROM.get(9, alarmEnable);
    EEPROM.get(13, pOffset_BME);
    EEPROM.get(17, pGauge_MPX);
    EEPROM.get(18, zeroOnBoot);
  }

  // Attach interrupts for buttons
  attachPCINT(digitalPinToPCINT(leftBtnPin),  LeftBtnPress , RISING);
  attachPCINT(digitalPinToPCINT(midBtnPin),   MidBtnPress,   RISING);
  attachPCINT(digitalPinToPCINT(rightBtnPin), RightBtnPress, RISING);


  // Apply  ADC gain setting
  ads.setGain(GAIN_ONE); // 1x gain   +/- 4.096V  1 bit = 0.125mV
  
  // Other gain options:
  // ads.setGain(GAIN_TWOTHIRDS);  // 2/3x gain +/- 6.144V  1 bit = 0.1875mV
  // ads.setGain(GAIN_TWO);        // 2x gain   +/- 2.048V  1 bit = 0.0625mV
  // ads.setGain(GAIN_FOUR);       // 4x gain   +/- 1.024V  1 bit = 0.03125mV
  // ads.setGain(GAIN_EIGHT);      // 8x gain   +/- 0.512V  1 bit = 0.015625mV
  // ads.setGain(GAIN_SIXTEEN);    // 16x gain  +/- 0.256V  1 bit = 0.0078125mV

  // Start ADS1115 ADC
  ads.begin();

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

  // Show default view
  Display(displayIndex);

  //Apply BME oversampling settings
  bme.setTemperatureOversampling(BME680_OS_4X);
  bme.setHumidityOversampling(BME680_OS_2X);
  bme.setPressureOversampling(BME680_OS_4X);
  bme.setIIRFilterSize(BME680_FILTER_SIZE_3);
  bme.setGasHeater(0, 0); // Turn off gas sensor
  
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
  Serial.print( String(pGauge_MPX, 3) + ',' + String(Vout, 3) + ',' + String(freq, 2) + ',' + String(avgFreq) + ',' );
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
    Display(displayIndex);
  }

  // Keep loop time as deterministic as possible to 500ms iterations
  unsigned long dt = millis() - millisLast;
  PRINT( "dt = ", String(dt) );
  PRINTS(" ms \n\n");
  if (500 > dt) delay(500 - dt);
  millisLast = millis();


}


/*================================================================================
  Function: printLCD
  Description: This function will print out characters to the display. Row 1 and
  Row 2 strings are set in the "rows" array, respectively. If any row is more than
  16 characters, the string will be trimmed. If a row is less than 16 char, the row
  will be padded with spaces. The rows are then concat. into one string, and printed
  out to the display, resulting in a perfect fit.
  //  ================================================================================*/
bool printLCD (String rows[]) {

  // flag to return, for each row, to indicate if the string
  // length is too long to be printed in one row.
  // If so, it will be trimmed to 16 characters
  bool rows_overflow[2] = { false, false };

  // Store the passed string character length of both rows
  int rows_length[2] = { rows[0].length(), rows[1].length() } ;


  // Set the overflow flags for each row
  // trim the string if greater than 16 characters
  // pad the string with spaces if less than 16 characters
  for (int i = 0; i < 2; i++) {
    if (rows_length[i] > 16) {
      rows_overflow[i] = true;
      rows[i] = rows[i].substring(0, 16);
    }
    else {
      rows_overflow[i] = false;
      for (int j = rows_length[i]; j < 16; j++) {
        rows[i] += " ";
      } // end nested for loop
    } //end else
  } //end top level for loop


  OpenLCD.write('|'); //Setting character
  OpenLCD.write('-'); //Clear display

  OpenLCD.print(rows[0] + rows[1]); //Print first row characters to LCD

  // Logical OR the overflow flags to indicate if a character overflow was detected and corrected
  return (rows_overflow[0] || rows_overflow[1]);
}


/*================================================================================
  Function: MeasureFreq
  Description: This function will calculate the flow sensor pulse train output
  frequence by measuring the HIGH and LOW times of the signal. It will also
  calculate the average frequency. The number of samples to average is determined
  by the FREQ_BUFFER_SIZE
  ================================================================================*/
void MeasFreq() {

  unsigned long ontime  = pulseIn(flowSensorPin, HIGH, 50000);
  unsigned long offtime = pulseIn(flowSensorPin, LOW,  50000);

  unsigned long period  = ontime + offtime;
  if ( period == 0 ) freq = 0;
  else freq = 1000000.0 / period;

  freqBuffer[bufferIndex] = freq;

  if ( bufferIndex == FREQ_BUFFER_SIZE - 1 ) bufferIndex = 0;
  else bufferIndex++;

  float sum = 0;
  for (int i = 0; i < FREQ_BUFFER_SIZE; i++) sum += freqBuffer[i];
  avgFreq = ( sum ) / FREQ_BUFFER_SIZE;

}



/*================================================================================
  Function: LeftBtnPress (called via pin change interrupt)
  Description: This function will decrement/toggle the selected value if the current
  item is acively selected. If nothing is selected, the View Index is decremented
  instead (i.e. go to the next screen to the left). Triggered by left button press
  ================================================================================*/

void LeftBtnPress() {
  unsigned long t = micros();
  if (t - lastPush_lBtn >= 10000) {
    lastPush_lBtn = t;
    if ( Selected == true ) {
      switch (displayIndex) {
        case 1: pressAlarm_Lo -= 0.5; break;
        case 2: 
          if(pressAlarm_Hi != pressAlarm_Lo) pressAlarm_Hi -= 0.5; 
          break;
        case 3: alarmEnable = !alarmEnable; break;
        case 4: zeroOnBoot = !zeroOnBoot; break;
      }
    }
    else  Decrement_ViewIndex();
  }
}



/*================================================================================
  Function: MidBtnPress (called via pin change interrupt)
  Description: This function will select the currently displayed variable, if it is
  selectable. When, deselecting, the variable is saved. If an alarm exists, pushing
  this button will reset the alarm.
  ================================================================================*/

void MidBtnPress() {
  unsigned long t = micros();
  if (t - lastPush_mBtn >= 10000) {
    lastPush_mBtn = micros();
    if (Alarm) {
      Alarm = false;
      displayIndex = 3;
      alarmEnable = true;
      EEPROM.put(9, false);
      Selected = true;
    }
    else if (Selected) {
      Selected = false;
      switch (displayIndex) {
        case 1:   EEPROM.put(1, pressAlarm_Lo); break;
        case 2:   EEPROM.put(5, pressAlarm_Hi); break;
        case 3: case 252:  EEPROM.put(9, alarmEnable);   break;
        case 4: EEPROM.put(18, zeroOnBoot); break;
      }
    }
    else {
      switch (displayIndex){
        case 1: case 2: case 3: case 4:
          Selected = true; 
          break;
        case 5:
          zeroSensors_Flag = true;
          displayIndex = 0;
          break;
        default: Selected = false;
      }
      
    }
  }
}

/*================================================================================
  Function: RightBtnPress (called via pin change interrupt)
  Description: This function will increment/toggle the selected value if the current
  item is acively selected. If nothing is selected, the View Index is incremented
  instead (i.e. go to the next screen to the right). Triggered by right button press
  ================================================================================*/

void RightBtnPress() {
  unsigned long t = micros();
  if (t - lastPush_rBtn >= 10000) {
    lastPush_rBtn = micros();
    if ( Selected == true ) {
      switch (displayIndex) {
        case 1: 
          if(pressAlarm_Lo != pressAlarm_Hi) pressAlarm_Lo += 0.5; 
          break;
        case 2: pressAlarm_Hi += 0.5; break;
        case 3: alarmEnable = !alarmEnable; break;
        case 4: zeroOnBoot = !zeroOnBoot; break;
      }
    }
    else Increment_ViewIndex();
  }
}

/*================================================================================
  Function: Display
  Description: This function will display a data view on the LCD screen. Which view
  is displayed is determined by the displayIndex value
  ================================================================================*/
void Display(byte index) {

  String lcdRows[2] = { "", "" };
  String tempStr = "";

  switch (index) {

    case 0: // Pressure Sensor Measurements

      tempStr = String(pGauge_BME, 1);
      if (tempStr == "-0.0") tempStr = "0.0";
      lcdRows[0] = "BME: " + tempStr + " cmH2O";

      tempStr = String(pGauge_MPX, 1);
      if (tempStr == "-0.0") tempStr = "0.0";
      lcdRows[1]  = "MPX: " + tempStr + " cmH2O";
      break;

    case 1: // Low Pressure Alarm Limits
      lcdRows[0] = "Press. Lo Limit:";
      lcdRows[1] = String(pressAlarm_Lo, 1) + " cmH2O";
      if (Selected) lcdRows[1] = "*" + lcdRows[1];
      break;

    case 2: // High Pressure Alarm Limits
      lcdRows[0] = "Press. Hi Limit:";
      lcdRows[1] = String(pressAlarm_Hi, 1) + " cmH2O";
      if (Selected) lcdRows[1] = "*" + lcdRows[1];
      break;

    case 3: // Alarm ON/OFF
      lcdRows[0] = "Enable Alarm?";
      if (Selected) lcdRows[1] = "*";
      if (alarmEnable) lcdRows[1] +=  "YES";
      else lcdRows[1] += "NO";
      break;
      
    case 4: 
      lcdRows[0] = "Zero Sensors";
      lcdRows[1] = "on Boot? ";
      if (Selected) lcdRows[1] += "*";
      if (zeroOnBoot) lcdRows[1] += "YES";
      else lcdRows[1] += "NO";
      break;
      
    case 5: // Zero Pressure Sensors?
      lcdRows[0] = "Press Select to";
      lcdRows[1] = "Re-Zero Sensors";
      break;
      
      
    case 253: // ALARM
      lcdRows[0] = "ALARM!!!";
      lcdRows[1] = "LIMIT EXCEEDED!!";
      break;

    case 254: // Sensor detected notification
      lcdRows[0] = "Sensor Detected!";
      break;

    case 255: // Sensor NOT detected notification
      lcdRows[0] = "Pressure Sensor";
      lcdRows[1] = "Not Detected";
      break;

    default: break;
  }
  delay(1);
  printLCD(lcdRows);

}

bool AlarmCheck() {
  float loLimit, hiLimit;
  bool  enable;
  EEPROM.get(1, loLimit);
  EEPROM.get(5, hiLimit);
  EEPROM.get(9, enable);

  if (enable) {
    if    (  ( pGauge_BME <= loLimit || pGauge_MPX <= loLimit )
             || ( pGauge_BME >= hiLimit || pGauge_MPX >= hiLimit ) ) return true;
    else return false;
  }
}


void Increment_ViewIndex() {
  if ( displayIndex == NUM_VIEWS - 1 )  displayIndex = 0;
  else displayIndex++;
}

void Decrement_ViewIndex() {
  if ( displayIndex == 0 )  displayIndex = NUM_VIEWS - 1;
  else displayIndex--;
}

void Zero_Sensors() {
  bme.performReading();
  pOffset_BME = (bme.pressure / 100.0) * hPa_to_cmH20;
  
  float voltADC = ( ads.readADC_Differential_0_1() * ads_gain);
  pOffset_MPX = ( (voltADC - 2.5) * ( hPa_to_cmH20 * 10 ) ); // sensor in units of kpa (= hpa x 10 )
  
  EEPROM.put(13, pOffset_BME); 
  EEPROM.put(17, pGauge_MPX);
  
  PRINTS("=====================ZERO=====================\n");
  PRINT("BME Offset value = ", String( pOffset_BME, 3 ) + " cmH2O\n");
  PRINT("MPX Offset value = ", String( pOffset_MPX, 3 ) + " cmH2O\n");
  PRINTS("==============================================\n\n");
}
