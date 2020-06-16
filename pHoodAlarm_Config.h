#include <EEPROM.h>


enum cfgName {
  pressAlarm_Lo = 1,
  pressAlarm_Hi = 5,
  alarmEnable = 9,
  pOffset_BME = 13,
  pGauge_MPX = 17,
  zeroOnBoot = 18
};

bool GetAlarmConfig() {
  
  // Falg var that stores value of memory address '0' 
  // if its value = 0x22, then previous settings exist in EEPROM
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
    return false; //Settings did not exist in EEPROM, defaults saved and loaded in memory
  }
  else {
    EEPROM.get(1, pressAlarm_Lo);
    EEPROM.get(5, pressAlarm_Hi);
    EEPROM.get(9, alarmEnable);
    EEPROM.get(13, pOffset_BME);
    EEPROM.get(17, pGauge_MPX);
    EEPROM.get(18, zeroOnBoot);
    return true; //Settings did not exist in EEPROM, defaults saved and loaded in memory
  }
}




float GetConfig(cfgName adr) {
  float val;
  EEPROM.get( ( (int)adr ), val);
  return val;
}

int GetConfig(cfgName adr) {
  int val;
  EEPROM.get( ( (int)adr ), val);
  return val;
}

bool GetConfig(cfgName adr) {
  bool val;
  EEPROM.get( ( (int)adr ), val);
  return val;
}


void PutConfig(cfgName adr, float val) {
  EEPROM.put( ( (int)adr ), val);
}

void PutConfig(cfgName adr, int val) {
  EEPROM.put( ( (int)adr ), val);
}

void PutConfig(cfgName adr, bool val) {
  bool val;
  EEPROM.put( ( (int)adr ), val);
  return val;
}
