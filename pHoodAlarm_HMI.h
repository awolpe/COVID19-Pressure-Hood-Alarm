#include <PinChangeInterrupt.h>
#include "pHoodAlarm_Display.h"

// Define pin numbers for interrupts
#define leftBtnPin  A0
#define midBtnPin  A1
#define rightBtnPin  A2


// uS timestamp of when the button was pushed last (for debouncing)
volatile unsigned long lastPush_mBtn = 0;
volatile unsigned long lastPush_lBtn = 0;
volatile unsigned long lastPush_rBtn = 0;


// Global vars to store settings while being modified by user via HMI 
volatile bool Alarm = false;
volatile bool alarmEnable = false;
volatile float pressAlarm_Lo = 4; //cmH2O
volatile float pressAlarm_Hi = 20; //cmH2O
volatile bool zeroSensors_Flag = false; // global var is a flag that indicated when user selects "Zero Sensors" via HMI (flag is set via an interrupt method)

/* This is the Global var that indicates whether an item is currently selected, in which case left/right button
  presses will modify the selected item's value (i.e. the respective global variable where the setting). When 
  an item is deselected (var 'Selected'  value changes from TRUE to FALSE), then the setting is saved to EEPROM. */
volatile bool Selected = false; 
// Selected = False ---> Displayed item IS NOT selected, so left/right button presses will change the displayed UI Panel, 
// Selected = True  ---> Displayed item IS actively selected, so left/right button presses will change setting value  




void Initialize_HMI() {
  
  
  pinMode(leftBtnPin,  INPUT);
  pinMode(midBtnPin,   INPUT);
  pinMode(rightBtnPin, INPUT);


  // Attach interrupts for buttons
  attachPCINT(digitalPinToPCINT(leftBtnPin),  LeftBtnPress , RISING);
  attachPCINT(digitalPinToPCINT(midBtnPin),   MidBtnPress,   RISING);
  attachPCINT(digitalPinToPCINT(rightBtnPin), RightBtnPress, RISING);

  // Show default view
  Display(UI_Index);
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
      switch (UI_Index) {
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
      UI_Index = 3;
      alarmEnable = true;
      EEPROM.put(9, false);
      Selected = true;
    }
    else if (Selected) {
      Selected = false;
      switch (UI_Index) {
        case 1:   EEPROM.put(1, pressAlarm_Lo); break;
        case 2:   EEPROM.put(5, pressAlarm_Hi); break;
        case 3: case 252:  EEPROM.put(9, alarmEnable);   break;
        case 4: EEPROM.put(18, zeroOnBoot); break;
      }
    }
    else {
      switch (UI_Index){
        case 1: case 2: case 3: case 4:
          Selected = true; 
          break;
        case 5:
          zeroSensors_Flag = true;
          UI_Index = 0;
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
      switch (UI_Index) {
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
  is displayed is determined by the UI_Index value
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

/*================================================================================
  Function: Increment_UIIndex
  Description: This function will display a data view on the LCD screen. Which view
  is displayed is determined by the UI_Index value
  ================================================================================*/
void UI_Next() {
  if ( UI_Index == UI_Panles_Count - 1 )  UI_Index = 0;
  else UI_Index++;
}

void UI_Previous() {
  if ( UI_Index == 0 )  UI_Index = UI_Panles_Count - 1;
  else UI_Index--;
}
