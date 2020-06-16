#include <Wire.h>
#include <SPI.h>
#include <Adafruit_Sensor.h>
#include "Adafruit_BME680.h"
#include <Adafruit_ADS1015.h>
#include "pHoodAlarm_Debug.h"

#define flowSensorPin  3


class pHoodSensors
{
  private:
    // Flow sensor
    static const int FREQ_BUFFER_SIZE = 60; // Size of the buffer/array to store frequency measurements to calculate average freq
    float freq = 0, avgFreq = 0, freqBuffer[FREQ_BUFFER_SIZE];
    int Flow_Flow_bufferIndex = 0; // Variable to keep track of which index to replace in the buffer. (Circular Buffer)
    
    // BME Sensor
    Adafruit_BME680 bme;
    
    //ADS1115 ADC for MXP Sensor measurements
    Adafruit_ADS1115 ads; // Driver instance for ADS1115 ADC, used to measure analog voltage signal of MPX sensor
    
    // Scaling/conversion factors
    const float hPa_to_cmH20 = 1.01971621; // Coefficient to convert pressure from the BME's native hPa units to cmH20
    const int ads_gainSetting = GAIN_ONE; // ADS1115  @ +/- 4.096V gain (16-bit results)


  public:
  
    float pOffset_MPX = 0; // MPX sensor offset (initialized to approx default value from past measurement)(cmH2O)
    float pGauge_MPX = 0; // Initialize variable for gauge pressure measurement (cmH2O)
    float pOffset_BME = 984.9506; // BME pressure, intialiized to past measurement as approx barometric pressure in Montrose, CA (cmH2O).
    float pGauge_BME = 0; //Initialize variable for gauge pressure measurement (cmH2O)
    
    
    pHoodSensors(){  
    // Apply  ADC gain setting
    ads.setGain(ads_gainSetting); // 1x gain   +/- 4.096V  1 bit = 0.125mV
    
    // Set pin modes
    pinMode(flowSensorPin,  INPUT);
    
    // Other gain options:
    // ads.setGain(GAIN_TWOTHIRDS);  // 2/3x gain +/- 6.144V  1 bit = 0.1875mV
    // ads.setGain(GAIN_TWO);        // 2x gain   +/- 2.048V  1 bit = 0.0625mV
    // ads.setGain(GAIN_FOUR);       // 4x gain   +/- 1.024V  1 bit = 0.03125mV
    // ads.setGain(GAIN_EIGHT);      // 8x gain   +/- 0.512V  1 bit = 0.015625mV
    // ads.setGain(GAIN_SIXTEEN);    // 16x gain  +/- 0.256V  1 bit = 0.0078125mV
    }


    bool begin(){
      if (!bme.begin() || !ads.begin()) return false;
    
      //Apply BME oversampling and filter settings
      //IMPORTANT! Temp meas must be on for compensation on other measurements! Better temp accuracy == more accurate compensation
      bme.setTemperatureOversampling(BME680_OS_4X); 
      bme.setHumidityOversampling(BME680_OS_2X);
      bme.setPressureOversampling(BME680_OS_4X);
      bme.setIIRFilterSize(BME680_FILTER_SIZE_3);
      
      // Turn off gas sensor. No eCO2 or TVOC measurements required for this applications purpose
      bme.setGasHeater(0, 0); 
      
    }


    void ZeroAll() {
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
};










/*================================================================================
  Function: Flow_MeasFreq
  Description: This function will calculate the flow sensor pulse train output
  frequence by measuring the HIGH and LOW times of the signal. It will also
  calculate the average frequency. The number of samples to average is determined
  by the FREQ_BUFFER_SIZE
  ================================================================================
void pHoodSensors::Flow_MeasFreq() {

  unsigned long ontime  = pulseIn(flowSensorPin, HIGH, 50000);
  unsigned long offtime = pulseIn(flowSensorPin, LOW,  50000);

  unsigned long period  = ontime + offtime;
  if ( period == 0 ) freq = 0;
  else freq = 1000000.0 / period;

  freqBuffer[Flow_bufferIndex] = freq;

  if ( Flow_bufferIndex == FREQ_BUFFER_SIZE - 1 ) Flow_bufferIndex = 0;
  else Flow_bufferIndex++;

  float sum = 0;
  for (int i = 0; i < FREQ_BUFFER_SIZE; i++) sum += freqBuffer[i];
  avgFreq = ( sum ) / FREQ_BUFFER_SIZE;

}
*/
