#include "pHoodAlarm_Sensors.h"
#include "pHoodAlarm_HMI.h"
#include "pHoodAlarm_Debug.h"
#include "pHoodAlarm_Config.h"



#if DEBUG //If debugging is being used, then the DEBUG functions will begin Serial comm.
#else Serial.begin(57600);
#endif

#define alarmPin  2 // Next version should be pin 3
#define WDPin  4
#define goodLED 8

// Total number of unique views that can be display 
#define UI_Panles_Count 6 

// Stores the time (in ms) that the last loop iteration finished
unsigned long millisLast = 0;

bool zeroOnBoot = true;

void Initialize_Alarm() {
  pinMode(WDPin,          OUTPUT);
  pinMode(goodLED,        OUTPUT);
  pinMode(alarmPin,       OUTPUT);
  
  digitalWrite(alarmPin, false);
  digitalWrite(WDPin, true);
  
  GetAlarmConfig();
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
