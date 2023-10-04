//include/define
#include <Arduino.h>
#include "Main_code.h"
#include "Debug.h"
#include "Debug_config.h"
#define DRTS1 Serial.println(F("\nDRTS1-----------------------")); Debug_RTS1(); Serial.println(F("DRTS1_END-------------------"));
#define DRTS2 Serial.println(F("\nDRTS2-----------------------")); Debug_RTS2(); Serial.println(F("DRTS2_END-------------------"));
#define DRTS3 Serial.println(F("\nDRTS3-----------------------")); Debug_RTS3(); Serial.println(F("DRTS3_END-------------------"));
#define T_MC_start Past_time = millis(); Serial.println(F("\nRT_D begin------------------"));
#define T_MC_End RT_D = (millis() - Past_time) - 33; Serial.println("\nRT_D end: " + String(RT_D) + String("ms-------------"));
//data
bool v;
uint8_t fcode;
uint32_t Count = 1;
uint32_t Past_time;
uint32_t RT_D;
//main code
void setup() 
{
  if(Serial_R){while(!Serial);}
  Board_info();
  //DRTS
  DRTS_P_L1
  //----
  Serial.println(F("\n!!!---------------------!!!"));
  Serial.println(F("Running setupDebug..."));
  Serial.println(F("!!!---------------------!!!"));
  #if T_M == 1 
  T_MC_start 
  #endif
  Serial.println(F("\nSetupS-----------------------")); 
  fcode = setupDebug();
  Serial.println(F("\nSetupE-----------------------")); 
  Serial.println("\nsetupDebug finished with code: " + String(fcode));
  #if T_M == 1 
  T_MC_End
  #endif
  //DRTS
  DRTS_P_L2
  //----
  print_freeRam();
  Serial.println(F("\n!!!---------------------!!!"));
  Serial.println(F("Running loop..."));
  Serial.println(F("!!!---------------------!!!"));
  do
  {
    #if T_M == 1 
    T_MC_start 
    #endif
    Serial.println(F("\nLoopS-----------------------")); 
    fcode = loopDebug();
    Serial.println(F("\nLoopE-----------------------")); 
    #if T_M == 1 
    T_MC_End
    #endif
    if (Auto_SRAM_M) 
    {
      v = print_freeRam(Count);
      Count++;
    }
    //DRTS
    DRTS_P_L3
    //----
    Serial.println(F("\n«««LS+-+-+-+-+-+-+-+-+-+-+-+-+-+LS»»»\n"));
  }
  while (fcode == 151 && v == false && (Count < Auto_STOP_C || Auto_STOP_C == 0));
  Serial.println("\nloopDebug finished with code: " + String(fcode));
  //DRTS
  DRTS_P_L4
  //----
  print_freeRam();
  Serial.println(F("\n!!!debug code end!!!"));
}
void loop() {}
