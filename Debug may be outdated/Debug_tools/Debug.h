//include
#include <Arduino.h>
#include <WString.h>
#include "ArduinoBoardManager.h"
#include "HardwareSerial.h"
#include "Debug_config.h"
ArduinoBoardManager arduino_board = ArduinoBoardManager();
//------------------comdata
//-------------------------
int freeRam_MCH(void) {
  extern int __heap_start, *__brkval; 
  int v; 
  return (int) &v - (__brkval == 0 ? (int) &__heap_start : (int) __brkval); 
}
bool print_freeRam(unsigned long i = 0, bool a = SRAM_ERROR) 
{
  Serial.println(F("\n---------------------------"));
  Serial.print(F("freeSram: "));
  Serial.print(freeRam_MCH());
  Serial.println(F(" byte"));
  Serial.print(F("REC: "));
  Serial.println(i);
  if (freeRam_MCH() < SRAM_Limit && a == true)
  {
    Serial.println(F("!!!ERROR LOW SRAM!!!"));  
    Serial.println(F("---------------------------"));
    return true;
  }
  Serial.println(F("---------------------------"));
  return false;
}
void Board_info(int Serial_speed = Defult_Serial_speed)
{  
  Serial.begin(Serial_speed);
  Serial.println(F(Ver));
  Serial.println(F("Board info-----------------"));
  Serial.print(F("CPU name: "));
  Serial.println(arduino_board.CPU_NAME);
  //-----------------------------------
  Serial.print(F("CPU architecture: "));
  Serial.print(arduino_board.NUM_BITS);
  Serial.println(F("-bit"));  
  //-----------------------------------
  Serial.print(F("CPU speed: "));
  Serial.print(F_CPU / 1000000);
  Serial.println(F("Mhz"));
  //-----------------------------------
  Serial.print(F("CPU SRAM: "));
  Serial.print(arduino_board.SRAM_SIZE / 1000);
  Serial.println(F("k"));
  //-----------------------------------
  Serial.print(F("CPU FLASH: "));
  Serial.print(arduino_board.FLASH_SIZE / 1000);
  Serial.println(F("k"));
  //-----------------------------------
  Serial.print(F("CPU EEPROM: "));
  Serial.print(arduino_board.EEPROM_SIZE / 1000);
  Serial.println(F("k"));
  Serial.println(F("----------------------------"));
}