#include <Arduino.h>

#include <Adafruit_ADS1X15.h>
#include "DFRobot_GP8403.h"


Adafruit_ADS1015 ADS1;     /* Use this for the 12-bit version */

#define SDA 26
#define SCL 14

#define DAC2_ADDR 0x59
#define DAC1_ADDR 0x58

DFRobot_GP8403 dac1(&Wire, DAC1_ADDR);
DFRobot_GP8403 dac2(&Wire, DAC2_ADDR);


void initADC(void);
void initDAC(void);
void readLocalADCVoltage(void);
uint16_t adctomv(uint16_t adc);
uint16_t adctoua(uint16_t adc);


float scaleFactor;       // (R1 + R2) / R2
float referenceVoltage;  // ADC reference voltage
int adcResolution;       // ADC resolution
float resistorValue;     //Shunt resistor value in Ohms


void setup() 
{
  Serial.begin(115200);

  delay(1000);

  initADC();
  initDAC();
}

void loop() 
{
  
  readLocalADCVoltage();
  Serial.println("====================================================");
  delay(1000);

}

void readLocalADCVoltage(void)
{
  Serial.print("Analog In Voltage: ");
  for(int i = 0; i < 4; i++)
  {
    uint16_t tADCvalue = ADS1.readADC_SingleEnded(i);
    Serial.print("AI" + String(i) + ":" + String(adctomv(tADCvalue)) + "mV ");
  }
  Serial.println();
  Serial.flush();
}

void initADC(void)
{
  Wire.begin(SDA,SCL);

 if(ADS1.begin(0x48))//4A
  {

    scaleFactor = 50.2 / 10.0;  // (R1 + R2) / R2
    referenceVoltage = 2048.0;  // Reference voltage in mV
    adcResolution = 2048;       // resolution 11-bit
    resistorValue = 100;        //Shunt resistor value in Ohms

    ADS1.setGain(GAIN_TWO);

    Serial.println("ADC_HW_V3 init OK");
  }
  else
  {
    
    while (1)
    {
      Serial.println("ADC not initialized");
      delay(1000);
    }
    
  }

}

void initDAC(void)
{
  while(dac2.begin()!=0)
  {
    Serial.println("DAC2 not initialized");
    delay(100);
  }
  delay(100);
  while(dac1.begin()!=0)
  {
    Serial.println("DAC1 not initialized");
    delay(100);
  }
  Serial.println("DAC init OK");

  //Set DAC output range
  dac1.setDACOutRange(dac1.eOutputRange10V);

  dac2.setDACOutRange(dac2.eOutputRange10V);


  dac1.setDACOutVoltage(1000, 0);
  dac1.setDACOutVoltage(2000, 1);
  dac2.setDACOutVoltage(3000, 0);
  dac2.setDACOutVoltage(4000, 1);

}

uint16_t adctomv(uint16_t adc)
{
  return (adc * referenceVoltage / adcResolution) * scaleFactor;
}

uint16_t adctoua(uint16_t adc)
{
  float voltage_mV = (adc * referenceVoltage) / adcResolution;
  uint16_t current_uA = (voltage_mV * 1000) / resistorValue;
  return current_uA;
}
