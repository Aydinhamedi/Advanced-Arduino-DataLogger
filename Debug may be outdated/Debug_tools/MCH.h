#include "Debug_config.h"
#ifdef MCH_A
  //include/define...
  #include "HardwareSerial.h"
  #include "Arduino_DebugUtils.h"
  #include "WString.h"
  #include "Arduino.h"
  #define timer_start I_Past_time = millis(); Serial.println(F("\nMCI_RT_D begin{"));
  #define timer_end I_RT_D = (millis() - I_Past_time) - 33; Serial.println("\nMCI_RT_D end: " + String(I_RT_D) + String("ms}"));
  #define UDVC(a) String(a)
  #define BP1 BRP1_ECF();
  #define BP2 BRP2_ECF();
  #define BP3 BRP3_ECF();
  #define BP4 BRP4_ECF();
  #ifdef Arduino_DebugUtils 
    #define ADU_I Debug.setDebugOutputStream(&Serial); Debug.setDebugLevel(Arduino_DebugUtils_MODE);  Debug.timestampOn(); 
  #endif
//dat
  uint32_t I_Past_time;
  uint32_t I_RT_D;
//func
int freeRam(void) {
  extern int __heap_start, *__brkval; 
  int v; 
  return (int) &v - (__brkval == 0 ? (int) &__heap_start : (int) __brkval); 
}
bool freeRam_internal(unsigned long i = 0, bool a = SRAM_ERROR) 
{
  Serial.println(F("\n---------------------------"));
  Serial.print(F("freeSram: "));
  Serial.print(freeRam());
  Serial.println(F(" byte"));
  Serial.print(F("REC: "));
  Serial.println(i);
  if (freeRam() < SRAM_Limit && a == true)
  {
    Serial.println(F("!!!ERROR LOW SRAM!!!"));  
    Serial.println(F("---------------------------"));
    return true;
  }
  Serial.println(F("---------------------------"));
  return false;
}
  void DebugVal(String Name, DebugVal_VDT Val) 
  {
    Serial.print(Name);
    Serial.print(F(": "));
    Serial.println(Val);
  }
  void CCL(CCL_VDT i = 0)
  {
    Serial.print(F("!!![CL: "));
    Serial.print(i);
    Serial.println(F(" ]!!!"));
  }
  void runDS1()
  {
    Serial.println(F("\nDS1 begin{"));
    DS1
    Serial.println(F("\nDS1 END}"));
  }
  void runDS2()
  {
    Serial.println(F("\nDS2 begin{"));
    DS2
    Serial.println(F("\nDS2 END}"));
  }
  void runDS3()
  {
    Serial.println(F("\nDS3 begin{"));
    DS3
    Serial.println(F("\nDS3 END}"));
  }
  void runDS4()
  {
    Serial.println(F("\nDS4 begin{"));
    DS4
    Serial.println(F("\nDS4 END}"));
  }
  void BRP1_ECF()
  {
    Serial.println("\nBrake point_S(1)>>>");
    BP_EC1
    Serial.println("INPUT any val to continue...");    
    while (Serial.available() == 0) 
    {delay(10);}
    while (Serial.available() > 0) 
    {Serial.read();}
    Serial.println("\nBrake point_E(1)<<<");
  }  
  void BRP2_ECF()
  {
    Serial.println("\nBrake point_S(2)>>>");
    BP_EC2
    Serial.println("INPUT any val to continue...");    
    while (Serial.available() == 0) 
    {delay(10);}
    while (Serial.available() > 0) 
    {Serial.read();}
    Serial.println("\nBrake point_E(2)<<<");
  }  
  void BRP3_ECF()
  {
    Serial.println("\nBrake point_S(3)>>>");
    BP_EC3
    Serial.println("INPUT any val to continue...");    
    while (Serial.available() == 0) 
    {delay(10);}
    while (Serial.available() > 0) 
    {Serial.read();}
    Serial.println("\nBrake point_E(3)<<<");
  }  
  void BRP4_ECF()
  {
    Serial.println("\nBrake point_S(4)>>>");
    BP_EC4
    Serial.println("INPUT any val to continue...");    
    while (Serial.available() == 0) 
    {delay(10);}
    while (Serial.available() > 0) 
    {Serial.read();}
    Serial.println("\nBrake point_E(4)<<<");
  }  
#endif