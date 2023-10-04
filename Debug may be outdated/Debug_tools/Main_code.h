//Write your code here>>>
//DEFL
#include "Arduino.h"
#include "Arduino_DebugUtils.h"
#include "MCH.h"
//include/define
#include <Arduino.h>
#include <ArduinoJson.h>
#include "SoftwareSerial.h"
#include <avr/pgmspace.h>
#include <RokkitHash.h>
#include <EEPROM.h> 
#include <SD.h>
#include <SPI.h>
#include <Adafruit_SHT31.h>
#include <Adafruit_GFX.h>   
#include <Adafruit_ST7735.h> 
#include <Wire.h>
#include <ds3231.h>
#include <BMP180advanced.h>
#include <math.h>
//macros----------------
#define Version Firmware_Version " B[" __DATE__ "] -" + String(Firmware_class)
#define Version_DX Firmware_Version "-" + String(Firmware_class) + " [" + SAVE_DATA_TYPE + "]\n\nbuild: " __DATE__ \
"\n\ndisplay style: \n\n" + String(DSN[display_style])
#define PIEZO_PIN 7
#define B_RX 11
#define B_TX 12
#define TFT_CS 10
#define TFT_RST 9  
#define TFT_DC 8
#define Minute (uint32_t)60000
#define SEC (uint32_t)1000
#define ATAT 30000
#define chipSelect 6
#define MX_Mi_Day 0
#define MX_Mi_Month 1
#define MX_Mi_year 2
#define MX_Mi_Main 3
#define BCVS "[DAT_LOGGER]" 
//Settings
#define Firmware_Version "V1.4.6"
#define Debug 0 // 0 off / +1 on
#define display_style 2
#define style_name {"[Debug]","[Simple]","[Simple(2)]"}
#define DELAY_SMTUL (uint32_t)1200 //in sec
#define D_DELAY_LOG 1 //in Minute (log interval)
#define PIEZO_MODE 1 // 0 off / 1 on
#define MAX_MIN_UPDATE_MODE 0 // 0 for in sync with logging time (for better charts) / 1 for in sync with code update (for better data) {[!!!only on main file!!!]}
#define EVENT_LOG_Detail_mode 0 // 0 only Errors / 1 high detail (uses a lot of storage space)
#define noise_filtering_Mode_DEF 0 // noise_filtering_Mode_DEF
#define EEPROM_Saddress 1 // EEPROM starting address 
#define EEPROM_SAVING 1 // 0 off / 1 on
#define SAVE_DATA_TYPE String("csv") // csv or txt
#define text_data_separator "\t"
#define Sensor_Active_OFFSET_H 0 // Sensor_Active_OFFSET_H
#define Sensor_Active_OFFSET_T 0 // Sensor_Active_OFFSET_T
#define heater_temp_offset -1.5
#define EEPROM_CLEAR 0 // 0 / 1
//TIME Settings
#define NEW_TIME_SET 0 // 0 / 1
#define NEW_DAY 21
#define NEW_MON 7
#define NEW_YEAR 2023
#define NEW_S 30
#define NEW_M 24
#define NEW_H 13
//noise_filtering
//S1
#define BMP180_SC 180
#define BMP180_LI 10
#define BMP180_HI 10
/*
ULtra:
#define BMP180_SC 180
#define BMP180_LI 10
#define BMP180_HI 10
Normal:
#define BMP180_SC 100
#define BMP180_LI 10
#define BMP180_HI 10
*/
//S2
#define SHT31_SC 120
#define SHT31_LI 8
#define SHT31_HI 8
/*
ULtra:
#define BMP180_SC 120
#define BMP180_LI 10
#define BMP180_HI 10
Normal:
#define BMP180_SC 80
#define BMP180_LI 10
#define BMP180_HI 10
*/
//Data
//float
float AVT_D;
float AVH_D;
float AVT_D_TEMP;
float AVH_D_TEMP;
float TEMPH;
float TEMPT;
float TEMPH2;
float TEMPT2;
float humidity; 
float RAW_humidity;
float temperature;
float RAW_temperature;
float Pressure_hPa;
float RAW_Pressure_hPa;
float DELAY_LOG = D_DELAY_LOG;
float SAF_T;
//int/long/...
uint8_t ERROR_C;
uint8_t year;
uint8_t mon;
uint8_t day;
uint8_t OLD_DAY;
uint8_t OLD_MON;
uint8_t OLD_YEAR;
uint32_t AV_C;
uint32_t day_C;
uint32_t VCH, S_VCH, VCD_LEN;
//bool
bool START = true;
bool START_L_EX = true;
bool BS_SHT31;      
bool BS_SD_CARD;     
bool BS_BMP_180;     
bool SD_EVENT_FILE_EX = true;
bool ignore_ERROR;
bool SD_MAIN_FILE_EX;
bool SHT31_heater;
bool HAM;
bool USB_Debug;
bool noise_filtering_Mode = noise_filtering_Mode_DEF;
//other DT
struct Max_min_REC{
   float MAX_T_REC = -100.00;
   float MIN_T_REC = 100.00;
   float MAX_H_REC = 0;
   float MIN_H_REC = 100.00;
   float TD_PD;
   float HD_PD;
};
char BLS_M = 'N';
char Firmware_class = 's';
Max_min_REC MAX_MIN_RT[4];
struct ts t; 
String config_B = "";
const String txt = String("txt");
const String csv = String("csv");
const String DSN[5] = style_name;
const char RAW_DATA[] =
  "RSD\n"
  "{\n"
  "wH8RdS86tS5gKj3E"
  "Fp4NzY2jLq6BvXc9"
  "tS9MxK6qLp8bHf7D"
  "cJ7dGh6fHt2Km5R"
  "yP5kXn9bVr2LcQ8J"
  "zC2mLp7fTg9DdE5"
  "sR3gNk6tVq8yHf4"
  "bH9nTc2vZj6Lx7Y"
  "mF8tKj2nGc7Dp5S"
  "vL7gHd9sT6fRq3N"
  "wH8RdS86tS5gKj3E"
  "Fp4NzY2jLq6BvXc9"
  "tS9MxK6qLp8bHf7D"
  "cJ7dGh6fHt2Km5R"
  "yP5kXn9bVr2LcQ8J"
  "zC2mLp7fTg9DdE5"
  "sR3gNk6tVq8yHf4"
  "bH9nTc2vZj6Lx7Y"
  "mF8tKj2nGc7Dp5S"
  "vL7gHd9sT6fRq3N"
  "}";
const char DSinfo[] =
  "============================\n"
  "      24H DATA SUMMARY      \n"
  "============================";
const char BL_help_tmp[] =
  "command table\n"
  "    help\n"
  "    EL_Info\n"
  "    SET_LOG.INTERVAL:\n"
  "    CONST_SAVE\n"
  "    RESET_DEF\n"
  "    SOFT_RESET\n"
  "    NFM_ON\n"
  "    NFM_OFF\n"
  "    ignore_ERROR_ON\n"
  "    ignore_ERROR_OFF\n"
  "    Data_stream_on\n"
  "    Data_stream_off\n"
  "    Data_stream_DB\n"
  "    SHT31_heater_ON\n"
  "    SHT31_heater_OFF";
String ELinfo =
  "---------------------------------\n"
  "Version: " + String(Version) + "\n\n"
  "Settings>>>\n"
  "Debug: " + String(Debug) + " (+1 ON / 0 OFF)\n"
  "display_style: " + String(DSN[display_style]) + "\n"
  "DELAY_SMTUL: " + String(DELAY_SMTUL) + "s\n"
  "DELAY_LOG: " + String(DELAY_LOG) + "m\n"
  "Sensor_Active_OFFSET_T: " + String(Sensor_Active_OFFSET_T) + "C\n"
  "heater_temp_offset: " + String(heater_temp_offset) + "C\n"
  "Sensor_Active_OFFSET_H: " + String(Sensor_Active_OFFSET_H) + "RH%\n"
  "noise_filtering_Mode: " + String(noise_filtering_Mode) + " (1 ON / 0 OFF)\n"
  "SHT31_heater: " + String(SHT31_heater) + " (1 ON / 0 OFF)\n"
  "MAX_MIN_UPDATE_MODE: " + String(MAX_MIN_UPDATE_MODE) + " (1 SCU / 0 SL)\n"
  "EVENT_LOG_Detail_mode: " + String(EVENT_LOG_Detail_mode) + " (1 HIGH / 0 LOW)\n"
  "EEPROM_Saddress: " + String(EEPROM_Saddress) + "\n"
  "EEPROM_SAVING: " + String(EEPROM_SAVING) + " (1 ON / 0 OFF)\n"
  "SAVE_DATA_TYPE: " + String(SAVE_DATA_TYPE) + "\n"
  "---------------------------------";
const String FIX_RAW =
  "sd card is not       working properly  try\nchanging the sd card";  
char VCD[] = __DATE__ __TIME__;
//other
SoftwareSerial bluetooth(B_RX, B_TX);
BMP180advanced bmp180(BMP180_ULTRAHIGHRES); 
Adafruit_ST7735 tft = Adafruit_ST7735(TFT_CS, TFT_DC, TFT_RST);
Adafruit_SHT31 TH_S = Adafruit_SHT31();
File SD_CARD;
Sd2Card card;
//warning
#if NEW_TIME_SET == 1
  #warning  "TIME SET IS ON|------------------------------"
#endif
#if EEPROM_CLEAR == 1
  #warning  "EEPROM_CLEAR IS ON|------------------------------"
#endif
//func
bool millisDelay(unsigned long Delay, int TimerS, int Sdelay = 0, bool RESET_CT = false, bool ATA = false) 
{
  static uint32_t TIME_DELAY[5];
  static uint32_t TIME_OVF;
  delay(Sdelay);
  #if Debug >= 2
    DebugVal("ATA: ", String(ATA));
    DebugVal("RESET_CT: ", String(RESET_CT));
    DebugVal("Sdelay: ", String(Sdelay));
    DebugVal("TimerS: ", String(TimerS));
    DebugVal("Delay: ", String(Delay));
    DebugVal("TIME_OVF: ", String(TIME_OVF));
    DebugVal("TIME_DELAY: ", String(TIME_DELAY[TimerS]));
  #endif
  if (TIME_OVF > millis())  //if OVF
  {
    TIME_DELAY[TimerS] = (millis() + Delay);
  }
  TIME_OVF = millis();                 //RESET OVF Val
  if (TIME_DELAY[TimerS] <= millis())  // TIMER true / RESET
  {
    TIME_DELAY[TimerS] = (millis() + Delay);
    SMART_EX:
    return true;
  } 
  else if (RESET_CT)  // TIMER RESET
  {
    TIME_DELAY[TimerS] = (millis() + Delay);
  }
  if (TIME_DELAY[TimerS] <= (millis() + ATAT) && ATA)
  {goto SMART_EX;}
  return false;
}
//----------------------
void printMD_bl(String startDate, String endDate, String filename)
{
  bluetooth.println(F("scan starts in 10 sec..."));
  delay(10000);
  bluetooth.println(F("scaning..."));
  bool printData = false;
  SD_CARD = SD.open(filename);
  if (SD_CARD) 
  {
    tft.fillScreen(ST7735_BLACK); 
    tft.setTextColor(ST7735_CYAN, ST7735_BLACK);
    tft.setCursor(0, 5);
    tft.setTextSize(2);                           
    tft.print("Scaning \n" + filename +"\nfile...");
    tft.setTextColor(ST7735_WHITE, ST7735_BLACK);
    tft.setTextSize(1);
    uint32_t Index_L = 0;
    String date = "";
    String line = "";
    while (SD_CARD.available()) 
    { 
      Index_L++;
      line = SD_CARD.readStringUntil('\n'); 
      line.trim();
      startDate.trim();
      endDate.trim();
      uint8_t FCI = line.indexOf(','); 
      date = line.substring(FCI + 1, line.indexOf(',', FCI + 1)); 
      if(millisDelay(500, 4))
      {
        tft.setCursor(0, 60);
        tft.println("StartDate: " + startDate);
        tft.setCursor(0, 73);
        tft.println("EndDate: " + endDate);
        tft.setCursor(0, 86);
        tft.println("CSD: " + date + "         ");
        tft.setCursor(0, 99);
        tft.println("Datefound: " + String(printData));
        tft.setCursor(0, 112);
        tft.println("Index_L: " + String(Index_L));
      }
      if (date.equals(startDate))
      {printData = true;}
      if (printData)
      {
        bluetooth.println("/*" + line + "*/");
      }
      if (date.equals(endDate) || date.equals(""))
      {break;}
    }
    SD_CARD.close(); // close the file
    if (!printData)
    {
      bluetooth.println(F("Error finding the date"));
      tft.fillScreen(ST7735_BLACK); 
      tft.setTextColor(ST7735_RED, ST7735_BLACK);
      tft.setCursor(0, 5);
      tft.println(F("Error finding the    date"));
      delay(4000);
      ST7735_TAB_Main();
    }
  } 
  else 
  {
    bluetooth.println(F("error opening the file"));
    tft.fillScreen(ST7735_BLACK); 
    tft.setTextColor(ST7735_RED, ST7735_BLACK);
    tft.setCursor(0, 5);
    tft.println(F("error     opening   the file"));
    delay(4000);
    ST7735_TAB_Main();
  }
  ST7735_TAB_Main();
}
//----------------------
void version_control()
{
  //CVH
  VCD_LEN = strlen(VCD);
  VCH = rokkit(VCD, VCD_LEN);
  //M_PROC
  EEPROM.get((EEPROM_Saddress + sizeof(float)) * 5 + sizeof(bool), S_VCH);
  if(S_VCH != VCH)
  {
    Firmware_class = 'b';
  }
  EEPROM.put((EEPROM_Saddress + sizeof(float)) * 5 + sizeof(bool), VCH);
  #if Debug >= 2
    DebugVal("VCD_LEN: ", String(VCD_LEN));
    DebugVal("VCD: ", String(VCD));
    DebugVal("VCH: ", String(VCH));
    DebugVal("S_VCH: ", String(S_VCH));
  #endif
}
//----------------------
void ST7735_TAB_ERROR(String ER = "???", String FIX = "N/A", bool RTFTS = true)
{
  if (!ignore_ERROR)
  {
    tft.setTextSize(1);
    tft.fillScreen(ST7735_BLACK);
    tft.setTextColor(ST7735_RED, ST7735_BLACK);
    tft.setCursor(0, 5);
    tft.print("ERROR: \n" + String(ER) + "\n\nFIX:\n" + String(FIX));
    PIEZO(PIEZO_PIN,'E',1000);
    delay(5000);
    if(RTFTS)
    {
      tft.fillScreen(ST7735_BLACK); 
      ST7735_TAB_Main();
    }
  }
}
//----------------------
void ST7735_TAB_Warning(String ER = "???", int Wdelay = 5000)
{
  if (!ignore_ERROR)
  {
    tft.setTextSize(1);
    tft.fillScreen(ST7735_BLACK);
    tft.setTextColor(ST7735_YELLOW, ST7735_BLACK);
    tft.setCursor(0, 5);
    tft.print("Warning: \n" + String(ER));
    PIEZO(PIEZO_PIN,'I',1000);
    delay(Wdelay);
    tft.fillScreen(ST7735_BLACK); 
    ST7735_TAB_Main();
  }
}
//----------------------
void ST7735_TAB_Boot() {
  tft.fillScreen(ST7735_BLACK); 
  tft.setTextColor(ST7735_RED, ST7735_BLACK);
  tft.setCursor(0, 5);
  tft.setTextSize(2);                           
  tft.print("Firmware:");
  tft.setCursor(0, 27);
  tft.setTextColor(ST7735_WHITE, ST7735_BLACK);
  tft.setTextSize(1);
  tft.print(Version_DX);             
  delay(5000);
}
//----------------------
void ST7735_TAB_Main() 
{ //TODO make new style
  tft.fillScreen(ST7735_BLACK);
  //style (0)------------------------------------------------
  #if display_style == 0
  //Time/date
  tft.setTextSize(1);                           
  tft.setTextColor(ST7735_WHITE, ST7735_BLACK);
  tft.setCursor(0, 5);
  tft.print(F("TIME ="));
  tft.setCursor(0, 20);
  tft.print(F("DATE ="));
  //Sys Data
  tft.setTextSize(1);
  tft.setTextColor(ST7735_WHITE, ST7735_BLACK);
  tft.setCursor(0, 35);
  tft.print(F("T-AVPRD ="));
  tft.setCursor(0, 50);
  tft.print(F("H-AVPRD ="));
  tft.setCursor(0, 65);
  tft.print(F("PROC ="));
  //THP data: H = 105 / T = 125 / Pressure_hPa = 145
  tft.drawFastHLine(0, 93, tft.width(), ST7735_WHITE);
  tft.setTextSize(1);
  tft.setTextColor(ST7735_BLUE, ST7735_BLACK);
  tft.setCursor(0, 105);
  tft.print(F("HUMIDITY ="));

  tft.setTextColor(ST7735_RED, ST7735_BLACK);
  tft.setCursor(0, 125);
  tft.print(F("TEMPERATURE ="));

  tft.setTextColor(ST7735_GREEN, ST7735_BLACK);
  tft.setCursor(0, 145);
  tft.print(F("PRESSURE ="));
  //style (1)------------------------------------------------
  #elif display_style == 1
  tft.setTextSize(1);                           
  tft.setTextColor(ST7735_WHITE, ST7735_BLACK);
  tft.setCursor(0, 10);
  tft.print(F("TIME ="));
  tft.setCursor(0, 25);
  tft.print(F("DATE ="));
  tft.drawFastHLine(0, 50,  tft.width(), ST7735_BLUE);   
  tft.drawFastHLine(0, 102,  tft.width(), ST7735_BLUE);         
  tft.setTextColor(ST7735_GREEN, ST7735_BLACK);     
  tft.setCursor(25, 61);            
  tft.print(F("TEMPERATURE ="));
  tft.setTextColor(ST7735_YELLOW, ST7735_BLACK); 
  tft.setCursor(34, 113);             
  tft.print(F("HUMIDITY ="));
  //style (2)------------------------------------------------
  #elif display_style == 2
  tft.setTextSize(1);
  tft.setTextColor(ST7735_WHITE, ST7735_BLACK);
  tft.setCursor(0, 2);
  tft.print(F("TIME ="));
  tft.setCursor(0, 17);
  tft.print(F("DATE ="));    
  tft.drawFastHLine(0, 30,  tft.width(), ST7735_WHITE);                   
  tft.drawFastHLine(0, 76,  tft.width(), ST7735_WHITE); 
  tft.drawFastHLine(0, 122,  tft.width(), ST7735_WHITE);  
  tft.setTextColor(ST7735_RED, ST7735_BLACK);    
  tft.setCursor(25, 39);            
  tft.print(F("TEMPERATURE ="));
  tft.setTextColor(ST7735_CYAN, ST7735_BLACK);  
  tft.setCursor(34, 85);             
  tft.print(F("HUMIDITY ="));
  tft.setTextColor(ST7735_GREEN, ST7735_BLACK); 
  tft.setCursor(34, 131);          
  tft.print(F("PRESSURE ="));
  tft.setTextSize(2);   
  //end of Style
  #endif
}
//----------------------
void ST7735_TAB_Main_data(String PROC = "Wait_\\   ") 
{ 
  #if Debug >= 2
    DebugVal("PROC: ", String(PROC));
  #endif
  //style (0)------------------------------------------------
  #if display_style == 0 //TODO make new style
  char buffer[8];
  //Time/date
  tft.setTextSize(1);
  tft.setTextColor(ST7735_WHITE, ST7735_BLACK);
  tft.setCursor(40, 5);
  sprintf(buffer, "%i:%i:%i   ", t.hour, t.min, t.sec);
  tft.print(buffer);
  tft.setCursor(40, 20);
  sprintf(buffer, "%i/%i/%i   ", t.mday, t.mon, t.year);
  tft.print(buffer);
  //Sys Data
  tft.setTextSize(1);
  tft.setTextColor(ST7735_WHITE, ST7735_BLACK);
  tft.setCursor(58, 35);
  tft.print(String(AVT_D) + "C    ");
  tft.setCursor(58, 50);
  tft.print(String(AVH_D) + "%    ");
  tft.setCursor(36, 65);
  tft.setTextColor(ST7735_GREEN, ST7735_BLACK);
  tft.print(String(" ") + PROC + String("  "));
  //THP data: H = 105 / T = 125 / Pressure_hPa = 145
  tft.setTextSize(1);
  tft.setTextColor(ST7735_BLUE, ST7735_BLACK);
  tft.setCursor(64, 105);
  tft.print(humidity);
  tft.print(F("%"));

  tft.setTextColor(ST7735_RED, ST7735_BLACK);
  tft.setCursor(83, 125);
  tft.print(temperature);
  tft.print(F("C"));

  tft.setTextColor(ST7735_GREEN, ST7735_BLACK);
  tft.setCursor(64, 145);
  tft.print(Pressure_hPa);
  tft.print(F("hpa "));
  //style (1)------------------------------------------------
  #elif display_style == 1
  char buffer[8];
  //Time/date
  tft.setTextSize(1);
  tft.setTextColor(ST7735_WHITE, ST7735_BLACK);
  tft.setCursor(40, 10);
  sprintf(buffer, "%i:%i:%i   ", t.hour, t.min, t.sec);
  tft.print(buffer);
  tft.setCursor(40, 25);
  sprintf(buffer, "%i/%i/%i   ", t.mday, t.mon, t.year);
  tft.print(buffer);
  //data
  tft.setTextSize(2);
  tft.setTextColor(ST7735_RED, ST7735_BLACK); 
  tft.setCursor(29, 78);
  tft.print(String(temperature));
  tft.drawCircle(90, 80, 2, ST7735_RED); 
  tft.setCursor(93, 78);
  tft.print(F("C"));
  tft.setTextColor(ST7735_CYAN, ST7735_BLACK);  
  tft.setCursor(29, 130);
  tft.print(String(humidity) + "%");
  //style (2)------------------------------------------------
  #elif display_style == 2
  char buffer[8];
  //Time/date
  tft.setTextSize(1);
  tft.setTextColor(ST7735_WHITE, ST7735_BLACK);
  tft.setCursor(40, 2);
  sprintf(buffer, "%i:%i:%i   ", t.hour, t.min, t.sec);
  tft.print(buffer);
  tft.setCursor(40, 17);
  sprintf(buffer, "%i/%i/%i   ", t.mday, t.mon, t.year);
  tft.print(buffer);
  //data
  tft.setTextSize(2);   
  tft.setTextColor(ST7735_YELLOW, ST7735_BLACK);  
  tft.setCursor(25, 53);
  tft.print(String(temperature) + "C" );
  tft.setTextColor(ST7735_MAGENTA, ST7735_BLACK);  
  tft.setCursor(25, 99);
  tft.print(String(humidity) + "% ");
  tft.setTextColor(0xFD00, ST7735_BLACK);  
  tft.setCursor(11, 145);
  tft.print(String(Pressure_hPa) + "hpa ");
  //end of Style
  #endif
}
//----------------------
void ELinfo_U()
{
  ELinfo =
    "---------------------------------\n"
    "Version: " + String(Version) + "\n\n"
    "Settings>>>\n"
    "Debug: " + String(Debug) + " (+1 ON / 0 OFF)\n"
    "display_style: " + String(DSN[display_style]) + "\n"
    "DELAY_SMTUL: " + String(DELAY_SMTUL) + "s\n"
    "DELAY_LOG: " + String(DELAY_LOG) + "m\n"
    "Sensor_Active_OFFSET_T: " + String(Sensor_Active_OFFSET_T) + "C\n"
    "heater_temp_offset: " + String(heater_temp_offset) + "C\n"
    "Sensor_Active_OFFSET_H: " + String(Sensor_Active_OFFSET_H) + "RH%\n"
    "noise_filtering_Mode: " + String(noise_filtering_Mode) + " (1 ON / 0 OFF)\n"
    "SHT31_heater: " + String(SHT31_heater) + " (1 ON / 0 OFF)\n"
    "MAX_MIN_UPDATE_MODE: " + String(MAX_MIN_UPDATE_MODE) + " (1 SCU / 0 SL)\n"
    "EVENT_LOG_Detail_mode: " + String(EVENT_LOG_Detail_mode) + " (1 HIGH / 0 LOW)\n"
    "EEPROM_Saddress: " + String(EEPROM_Saddress) + "\n"
    "EEPROM_SAVING: " + String(EEPROM_SAVING) + " (1 ON / 0 OFF)\n"
    "SAVE_DATA_TYPE: " + String(SAVE_DATA_TYPE) + "\n"
    "---------------------------------";
}
//----------------------
void(* resetFunc) (void) = 0; 
//----------------------
void PIEZO(int pin, char mode, long ONTIME)// mode E : ERROR , W : interrupts2 , I : interrupts
{
  if (PIEZO_MODE)
  {
    unsigned long NEWTIME = (millis() + ONTIME);
    unsigned long OLDTIME = millis();
    delay(10);
    if (mode == 'E')
    {
      while (NEWTIME > millis() && millis() > OLDTIME)
      {
        tone(pin,2500,40);
        delay(100);
        tone(pin,2500,40);
        delay(400);
      }
    }
    else if (mode == 'W')
    {
      while (NEWTIME > millis() && millis() > OLDTIME)
      {
        tone(pin,2000,40);
        delay(200);
      }
    }
    else if (mode == 'I')
    {
      while (NEWTIME > millis() && millis() > OLDTIME)
      {
        tone(pin,1000,100);
        delay(1000);
      }
    }
  }
}
//----------------------
void DAY_LOG()
{ 
  bool print_DAYF_H = SD.exists(F("DAY.txt"));
  SD_CARD = SD.open(F("DAY.txt"), FILE_WRITE);
  if (!print_DAYF_H){SD_CARD.println(DSinfo);}
  DS3231_get(&t);
  SD_CARD.print("\n");
  SD_CARD.print("#----- DATE: " + String(t.mday) + "/" + String(t.mon) + "/" + String(t.year) + " -----#\n");
  SD_CARD.print("temp range: (" + String(MAX_MIN_RT[MX_Mi_Day].MIN_T_REC) + "°C|" + String(MAX_MIN_RT[MX_Mi_Day].MAX_T_REC) + "°C)\n");
  SD_CARD.print("humidity range: (" + String(MAX_MIN_RT[MX_Mi_Day].MIN_H_REC) + "RH%|" + String(MAX_MIN_RT[MX_Mi_Day].MAX_H_REC) + "RH%)\n");
  SD_CARD.print("AV temp: " + String(AVT_D) + "°C (based on " + String(AV_C) + " data points)\n");
  SD_CARD.print("AV humidity: " + String(AVH_D) + "RH% (based on " + String(AV_C) + " data points)\n");
  SD_CARD.print("Temp difference from previous day: " + String(MAX_MIN_RT[MX_Mi_Day].TD_PD) + "°C\n");
  SD_CARD.print("Humidity difference from previous day: " + String(MAX_MIN_RT[MX_Mi_Day].HD_PD) + "RH%\n");
  SD_CARD.close();
}
//----------------------
void Bluetooth_PRINT() 
{
  if (BLS_M == 'N')
  {
    DS3231_get(&t);
    bluetooth.print("\n");
    bluetooth.print("#----- DATE & TIME: " + String(t.mday) + "/" + String(t.mon) + "/" + String(t.year) + " | "
    + String(t.hour) + ":" + String(t.min) + ":" + String(t.sec) + " -----#\n");
    bluetooth.print("temp: " + String(temperature) + "°C\n");
    bluetooth.print("humidity: " + String(humidity) + "RH%\n");
    bluetooth.print("temp range: (" + String(MAX_MIN_RT[MX_Mi_Day].MIN_T_REC) + "°C|" + String(MAX_MIN_RT[MX_Mi_Day].MAX_T_REC) + "°C)\n");
    bluetooth.print("humidity range: (" + String(MAX_MIN_RT[MX_Mi_Day].MIN_H_REC) + "RH%|" + String(MAX_MIN_RT[MX_Mi_Day].MAX_H_REC) + "RH%)\n");
    bluetooth.print("AV temp: " + String(AVT_D) + "°C (based on " + String(AV_C) + " data points)\n");
    bluetooth.print("AV humidity: " + String(AVH_D) + "RH% (based on " + String(AV_C) + " data points)\n");
    bluetooth.print("Temp difference from previous day: " + String(MAX_MIN_RT[MX_Mi_Day].TD_PD) + "°C\n");
    bluetooth.print("Humidity difference from previous day: " + String(MAX_MIN_RT[MX_Mi_Day].HD_PD) + "RH%\n");
  }
  else if (BLS_M == 'S')
  {
    bluetooth.print(F("/*")); //for suport of serial studio
    bluetooth.print(t.hour);
    bluetooth.print(F(":"));
    bluetooth.print(t.min);
    bluetooth.print(F(":"));
    bluetooth.print(t.sec);
    bluetooth.print(F(","));
    
    bluetooth.print(t.mday);
    bluetooth.print(F("/"));
    bluetooth.print(t.mon);
    bluetooth.print(F("/"));
    bluetooth.print(t.year);
    bluetooth.print(F(","));

    bluetooth.print(Pressure_hPa); 
    #if Debug >= 1
      bluetooth.print(F(","));
      bluetooth.print(RAW_Pressure_hPa);
    #endif
    bluetooth.print(F(","));
    bluetooth.print(temperature);  
    #if Debug >= 1
      bluetooth.print(F(",")); 
      bluetooth.print(RAW_temperature);
    #endif
    bluetooth.print(F(",")); 
    bluetooth.print(MAX_MIN_RT[MX_Mi_Month].MAX_T_REC); 
    bluetooth.print(F(",")); 
    bluetooth.print(MAX_MIN_RT[MX_Mi_Month].MIN_T_REC); 
    bluetooth.print(F(",")); 
    bluetooth.print(humidity); 
    #if Debug >= 1
      bluetooth.print(F(",")); 
      bluetooth.print(RAW_humidity);
    #endif
    bluetooth.print(F(",")); 
    bluetooth.print(MAX_MIN_RT[MX_Mi_Month].MAX_H_REC); 
    bluetooth.print(F(",")); 
    bluetooth.print(MAX_MIN_RT[MX_Mi_Month].MIN_H_REC); 
    bluetooth.print(F(",")); 
    bluetooth.print(MAX_MIN_RT[MX_Mi_Main].TD_PD); 
    bluetooth.print(F(",")); 
    bluetooth.print(MAX_MIN_RT[MX_Mi_Main].HD_PD); 
    bluetooth.print(F(","));
    bluetooth.print(BS_SHT31);
    bluetooth.print(F(","));
    bluetooth.print(HAM);
    bluetooth.print(F(","));
    bluetooth.print(BS_BMP_180);
    bluetooth.print(F(","));
    bluetooth.print(ERROR_C);
    bluetooth.println(F("*/")); //for suport of serial studio
  }
}
//----------------------
void ELOG(String ELOG_INPUT, bool DATA_HEADER, bool OVER_WRITE = false, bool HM = false)
{
  #if Debug >= 3
    DebugVal("ELOG_INPUT: ", String(ELOG_INPUT));
    DebugVal("DATA_HEADER: ", String(DATA_HEADER));
    DebugVal("OVER_WRITE: ", String(OVER_WRITE));
    DebugVal("HM: ", String(HM));
  #endif
  SD_CARD = SD.open(F("Event.log"), FILE_WRITE);
  if (!HM)
  {
    if (SD_EVENT_FILE_EX == true or OVER_WRITE == true)
    {
      if (DATA_HEADER)
      {
        SD_CARD.print(F("|"));
        SD_CARD.print(t.hour);
        SD_CARD.print(F(":"));
        SD_CARD.print(t.min);
        SD_CARD.print(F("."));
        SD_CARD.print(t.sec);
        SD_CARD.print(F("|"));
        
        SD_CARD.print(t.mday);
        SD_CARD.print(F("/"));
        SD_CARD.print(t.mon);
        SD_CARD.print(F("/"));
        SD_CARD.print(t.year);
        SD_CARD.print(F("|"));

        SD_CARD.print(F("DATA: "));
      } 
      SD_CARD.println(ELOG_INPUT);
      SD_CARD.close();
    }
  }
  else
  {
    DS3231_get(&t);
    SD_CARD.print(F("\n"));
    SD_CARD.print(F("|"));
    SD_CARD.print(t.hour);
    SD_CARD.print(F(":"));
    SD_CARD.print(t.min);
    SD_CARD.print(F("."));
    SD_CARD.print(t.sec);
    SD_CARD.print(F("|"));
    
    SD_CARD.print(t.mday);
    SD_CARD.print(F("/"));
    SD_CARD.print(t.mon);
    SD_CARD.print(F("/"));
    SD_CARD.print(t.year);
    SD_CARD.print(F("|"));
    
    SD_CARD.print(F("WARNING: "));
    SD_CARD.println(ELOG_INPUT);
    SD_CARD.print(F("\n"));
    SD_CARD.close();     
  }  
}
//----------------------
void RAW_TEST()
{
  if (BS_SD_CARD)
  {
    if (SD.exists(F("R_A_W_T")))
    {SD.remove(F("R_A_W_T"));}
    //RAW 
    SD_CARD = SD.open(F("R_A_W_T"), FILE_WRITE);
    SD_CARD.print(RAW_DATA);
    SD_CARD.close();
    SD_CARD = SD.open(F("R_A_W_T"), FILE_READ);
    String TEMP_R = SD_CARD.readString();
    SD_CARD.close();
    //RAW check
    if (!TEMP_R.equals(RAW_DATA))
    {
      ELOG(F("RAW(read and write) TEST failed!!!"), true, false, true); //may not work 
      ST7735_TAB_ERROR(F("RAW(read and write)  TEST failed!!!"), FIX_RAW);
      SD.remove(F("R_A_W_T"));
    }
  }
}
//----------------------
void CSV_H()
{
  SD_CARD.print(F("TIME"));
  SD_CARD.print(F(","));  
  SD_CARD.print(F("DATE_T:(DD/MM/YY)"));
  SD_CARD.print(F(",")); 
  SD_CARD.print(F("airPressure (hpa)"));
  #if Debug >= 1
    SD_CARD.print(F(","));
    SD_CARD.print(F("RAW airPressure (hpa)"));
  #endif
  SD_CARD.print(F(",")); 
  SD_CARD.print(F("temperature (C)"));
  #if Debug >= 1
    SD_CARD.print(F(",")); 
    SD_CARD.print(F("RAW temperature (C)"));
  #endif
  SD_CARD.print(F(",")); 
  SD_CARD.print(F("max temperature (C)"));
  SD_CARD.print(F(",")); 
  SD_CARD.print(F("min temperature (C)"));
  SD_CARD.print(F(",")); 
  SD_CARD.print(F("humidity (RH%)"));
  #if Debug >= 1
    SD_CARD.print(F(",")); 
    SD_CARD.print(F("RAW humidity (RH%)"));
  #endif
  SD_CARD.print(F(",")); 
  SD_CARD.print(F("max humidity (RH%)"));
  SD_CARD.print(F(",")); 
  SD_CARD.print(F("min humidity (RH%)"));
  SD_CARD.print(F(",")); 
  SD_CARD.print(F("TD (C)"));
  SD_CARD.print(F(",")); 
  SD_CARD.print(F("HD (RH%)"));
  SD_CARD.print(F(",")); 
  SD_CARD.print(F("SHT31 state (1/0)"));
  SD_CARD.print(F(",")); 
  SD_CARD.print(F("SHT31 heater state (1/0)"));
  SD_CARD.print(F(",")); 
  SD_CARD.print(F("BMP 180 state (1/0)"));
  SD_CARD.print(F(",")); 
  SD_CARD.println(F("ERROR_C"));
}
//----------------------
void TXT_LOG_STRUCT()
{
  SD_CARD.print(t.hour);
  SD_CARD.print(F(":"));
  SD_CARD.print(t.min);
  SD_CARD.print(F(":"));
  SD_CARD.print(t.sec);
  SD_CARD.print(text_data_separator);

  SD_CARD.print(t.mday);
  SD_CARD.print(F("/"));
  SD_CARD.print(t.mon);
  SD_CARD.print(F("/"));
  SD_CARD.print(t.year);
  SD_CARD.print(text_data_separator);
  
  SD_CARD.print(Pressure_hPa); 
  #if Debug >= 1
    SD_CARD.print(text_data_separator); 
    SD_CARD.print(RAW_Pressure_hPa);
  #endif
  SD_CARD.print(text_data_separator);
  SD_CARD.print(temperature);  
  #if Debug >= 1
    SD_CARD.print(text_data_separator); 
    SD_CARD.print(RAW_temperature);
  #endif
  SD_CARD.print(text_data_separator); 
  SD_CARD.print(MAX_MIN_RT[MX_Mi_Main].MAX_T_REC); 
  SD_CARD.print(text_data_separator); 
  SD_CARD.print(MAX_MIN_RT[MX_Mi_Main].MIN_T_REC); 
  SD_CARD.print(text_data_separator); 
  SD_CARD.print(humidity); 
  #if Debug >= 1
    SD_CARD.print(text_data_separator); 
    SD_CARD.print(RAW_humidity);
  #endif
  SD_CARD.print(text_data_separator); 
  SD_CARD.print(MAX_MIN_RT[MX_Mi_Main].MAX_H_REC); 
  SD_CARD.print(text_data_separator); 
  SD_CARD.print(MAX_MIN_RT[MX_Mi_Main].MIN_H_REC); 
  SD_CARD.print(text_data_separator); 
  SD_CARD.print(MAX_MIN_RT[MX_Mi_Main].TD_PD); 
  SD_CARD.print(text_data_separator); 
  SD_CARD.print(MAX_MIN_RT[MX_Mi_Main].HD_PD); 
  SD_CARD.print(text_data_separator);
  SD_CARD.print(BS_SHT31);
  SD_CARD.print(text_data_separator);
  SD_CARD.print(HAM);
  SD_CARD.print(text_data_separator);
  SD_CARD.print(BS_BMP_180);
  SD_CARD.print(text_data_separator);
  SD_CARD.println(ERROR_C);
}
//----------------------
void CSV_LOG_STRUCT()
{
  SD_CARD.print(t.hour);
  SD_CARD.print(F(":"));
  SD_CARD.print(t.min);
  SD_CARD.print(F(":"));
  SD_CARD.print(t.sec);
  SD_CARD.print(F(","));
  
  SD_CARD.print(t.mday);
  SD_CARD.print(F("/"));
  SD_CARD.print(t.mon);
  SD_CARD.print(F("/"));
  SD_CARD.print(t.year);
  SD_CARD.print(F(","));

  SD_CARD.print(Pressure_hPa); 
  #if Debug >= 1
    SD_CARD.print(F(","));
    SD_CARD.print(RAW_Pressure_hPa);
  #endif
  SD_CARD.print(F(","));
  SD_CARD.print(temperature);  
  #if Debug >= 1
    SD_CARD.print(F(",")); 
    SD_CARD.print(RAW_temperature);
  #endif
  SD_CARD.print(F(",")); 
  SD_CARD.print(MAX_MIN_RT[MX_Mi_Main].MAX_T_REC); 
  SD_CARD.print(F(",")); 
  SD_CARD.print(MAX_MIN_RT[MX_Mi_Main].MIN_T_REC); 
  SD_CARD.print(F(",")); 
  SD_CARD.print(humidity); 
  #if Debug >= 1
    SD_CARD.print(F(",")); 
    SD_CARD.print(RAW_humidity);
  #endif
  SD_CARD.print(F(",")); 
  SD_CARD.print(MAX_MIN_RT[MX_Mi_Main].MAX_H_REC); 
  SD_CARD.print(F(",")); 
  SD_CARD.print(MAX_MIN_RT[MX_Mi_Main].MIN_H_REC); 
  SD_CARD.print(F(",")); 
  SD_CARD.print(MAX_MIN_RT[MX_Mi_Main].TD_PD); 
  SD_CARD.print(F(",")); 
  SD_CARD.print(MAX_MIN_RT[MX_Mi_Main].HD_PD); 
  SD_CARD.print(F(","));
  SD_CARD.print(BS_SHT31);
  SD_CARD.print(F(","));
  SD_CARD.print(HAM);
  SD_CARD.print(F(","));
  SD_CARD.print(BS_BMP_180);
  SD_CARD.print(F(","));
  SD_CARD.println(ERROR_C);
}
//----------------------
bool SHT31_TEST() 
{
  uint16_t stat = TH_S.readStatus();
  Serial.println(stat, HEX);
  #if Debug >= 2
    DebugVal("SHT31_TEST SC: ", String(stat));
  #endif
  //TODO: find what is the problem (bit 15 and bit 12 of the status register are set. Bit 15 indicates a write data checksum error and bit 12 indicates a temperature tracking alert)
  // if (stat & 0x8000 || stat & 0x4000 || stat & 0x1000 || stat & 0x0800 || stat & 0x0080)
  // {
  //   return false;
  // }
  // return true;
  return TH_S.begin();
}
//----------------------
bool BMP180_TEST() 
{
  #if Debug >= 2
    DebugVal("BMP180_TEST DI: ", String(bmp180.readDeviceID()));
  #endif
  if(bmp180.readDeviceID() == false) return false; 
  return true; 
}
//----------------------
bool SD_TEST() 
{
  return SD.begin(SPI_FULL_SPEED, chipSelect);
}
//----------------------
void update_airPressure() 
{
  if (noise_filtering_Mode)
  {
    RAW_Pressure_hPa = bmp180.getPressure_hPa();
    if (EVENT_LOG_Detail_mode == 1)
    {ELOG(F("BMP180 NF_ON DATA UPDATE..."), true);}
    //noise_filtering
    float filterArray[BMP180_SC]; 
    for (uint8_t sample = 0; sample < BMP180_SC; sample++) 
    {
      filterArray[sample] = bmp180.getPressure_hPa();
      delay(500);
    }
    for (uint8_t i = 0; i < (BMP180_SC - 1); i++) {
      for (uint8_t j = i + 1; j < BMP180_SC; j++) {
        if (filterArray[i] > filterArray[j]) {
          float swap = filterArray[i];
          filterArray[i] = filterArray[j];
          filterArray[j] = swap;
        }
      }
    }
    float sum = 0;
    for (uint8_t sample = BMP180_LI; sample < (BMP180_SC - BMP180_HI); sample++) {
      sum += filterArray[sample];
    }
    Pressure_hPa = sum / ((BMP180_SC - BMP180_HI) - BMP180_LI);
  }
  else
  {
    if (EVENT_LOG_Detail_mode == 1)
    {ELOG(F("BMP180 NF_OFF DATA UPDATE..."), true);}
    Pressure_hPa = bmp180.getPressure_hPa();
    RAW_Pressure_hPa = Pressure_hPa;
  }
}
//----------------------
void update_SHT31_data()
{ 
  if (temperature < 32 && humidity > 75 && SHT31_heater)
  {
    TH_S.heater(true);
  }
  else
  {
    TH_S.heater(false);
  }
  if (TH_S.isHeaterEnabled())
  {
    ELOG("SHT31 heater ON...", true, false, true);
    ST7735_TAB_Warning("SHT31 heater ON...", 0);
    HAM = true;
    SAF_T = heater_temp_offset;
  }
  else if (HAM)
  {
    ELOG("SHT31 heater OFF...", true, false, true);
    ST7735_TAB_Warning("SHT31 heater OFF...", 0); 
    HAM = false;
    SAF_T = 0;
  }
  #if Debug >= 2
    DebugVal("SHT31 isHeaterEnabled: ", String(TH_S.isHeaterEnabled()));
  #endif
  if (noise_filtering_Mode)
  {
    RAW_temperature = TH_S.readTemperature();
    RAW_humidity = TH_S.readHumidity();
    if (EVENT_LOG_Detail_mode == 1)
    {ELOG(F("SHT31 NF_ON DATA UPDATE..."), true);}
    //noise_filtering [humidity]
    float filterArray[SHT31_SC]; 
    float filterArray2[SHT31_SC]; 
    for (uint8_t sample = 0; sample < SHT31_SC; sample++) 
    {
      filterArray[sample] = TH_S.readHumidity();
      filterArray2[sample] = TH_S.readTemperature();
      delay(500);
    }
    for (uint8_t i = 0; i < (SHT31_SC - 1); i++) {
      for (uint8_t j = i + 1; j < SHT31_SC; j++) {
        if (filterArray[i] > filterArray[j]) {
          float swap = filterArray[i];
          filterArray[i] = filterArray[j];
          filterArray[j] = swap;
        }
      }
    }
    float sum = 0;
    for (uint8_t sample = SHT31_LI; sample < (SHT31_SC - SHT31_HI); sample++) {
      sum += filterArray[sample];
    }
    humidity = (sum / ((SHT31_SC - SHT31_HI) - SHT31_LI)) + Sensor_Active_OFFSET_H;
    //noise_filtering [Temperature]
    for (uint8_t i = 0; i < (SHT31_SC - 1); i++) {
      for (uint8_t j = i + 1; j < SHT31_SC; j++) {
        if (filterArray2[i] > filterArray2[j]) {
          float swap = filterArray2[i];
          filterArray2[i] = filterArray2[j];
          filterArray2[j] = swap;
        }
      }
    }
    sum = 0;
    for (uint8_t sample = SHT31_LI; sample < (SHT31_SC - SHT31_HI); sample++) {
      sum += filterArray2[sample];
    }
    temperature = (sum / ((SHT31_SC - SHT31_HI) - SHT31_LI)) + Sensor_Active_OFFSET_T + SAF_T; 
  }
  else
  {
    if (EVENT_LOG_Detail_mode == 1)
    {ELOG(F("SHT31 NF_OFF DATA UPDATE..."), true);}
    temperature = TH_S.readTemperature() + Sensor_Active_OFFSET_T + SAF_T;
    humidity = TH_S.readHumidity() + Sensor_Active_OFFSET_H;
    RAW_temperature = temperature;
    RAW_humidity = humidity;
  }
}
//main code
int setupDebug()
{
  ADU_I
  //Starting------------
  SD.begin(SPI_FULL_SPEED, chipSelect); //SD
  Serial.begin(9600); //Serial
  bluetooth.begin(38400); //Bluetooth
  tft.initR(INITR_BLACKTAB); //Dysplay
  tft.setRotation(90); //Dysplay
  tft.fillScreen(ST7735_BLACK); //Dysplay
  Wire.begin(); //i2c
  TH_S.begin(); //SHT31
  bmp180.begin(); //BMP180
  DS3231_init(DS3231_CONTROL_INTCN); //DS3231
  version_control();
  Serial.println(ELinfo); 
  delay(10);  
  if (NEW_TIME_SET == 1) //NEW_TIME
  {
    t.hour=NEW_H; 
    t.min=NEW_M;
    t.sec=NEW_S;
    t.mday=NEW_DAY;
    t.mon=NEW_MON;
    t.year=NEW_YEAR;
    DS3231_set(t);
    ST7735_TAB_ERROR(F("NEW TIME SET IS ON"), "N/A", false);
    Serial.println(F("STOP"));
    while (true) {PIEZO(PIEZO_PIN,'E',5000);} // STOP
  }
  if (EEPROM_CLEAR == 1) //EEPROM_CLEAR
  {
    tft.fillScreen(ST7735_BLACK); 
    tft.setTextColor(ST7735_RED, ST7735_BLACK);
    tft.setCursor(0, 5);
    tft.setTextSize(2);                           
    tft.print("CLEARING  EEPROM:");
    tft.setTextColor(ST7735_WHITE, ST7735_BLACK);
    tft.setTextSize(2);
    for (int i = 0 ; i < EEPROM.length() ; i++) 
    {
      tft.setCursor(0, 43);
      tft.println(String(i) + "/" + String(EEPROM.length()) + "       ");
      EEPROM.write(i, 0);
    }
    ST7735_TAB_ERROR(F("EEPROM_CLEAR IS ON"));
    Serial.println(F("STOP"));
    while (true) {PIEZO(PIEZO_PIN,'E',5000);} // STOP
  }
  PIEZO(PIEZO_PIN,'I',1000);
  //Check--------------->>>
  //display
  ST7735_TAB_Boot();
  if(Serial)
  {
    tft.fillScreen(ST7735_BLACK);
    tft.setTextColor(ST7735_RED, ST7735_BLACK);
    tft.setTextSize(2);
    tft.setCursor(0, 5);
    tft.println("USB Serial\nDetected!");
    tft.setTextSize(1);
    tft.println("\nSerial Debug is ON");
    PIEZO(PIEZO_PIN,'E',2000);
    USB_Debug = true;
    delay(10000);
    tft.fillScreen(ST7735_BLACK); 
  }
  #if Debug >= 1
    ST7735_TAB_Warning("Debug mode is on!\nDebug class: " + String(Debug));
  #endif
  //Data_type_error
  if (!SAVE_DATA_TYPE.equals(csv) && !SAVE_DATA_TYPE.equals(txt))
  {
    ELOG(F("DATA TYPE ERROR/CODE STOP!!!"), true);
    Serial.println(F("DATA TYPE ERROR"));
    ST7735_TAB_ERROR(F("LOG DATA TYPE !=(csv/txt)"));
    Serial.println(F("STOP"));
    while (true) {PIEZO(PIEZO_PIN,'E',5000);} // STOP
  }
  //SD_EVENT_FILE_EX----------
  if (SD.exists(F("EVENT.LOG"))) 
  {
    SD_EVENT_FILE_EX = true;
  } 
  else 
  {
    SD_EVENT_FILE_EX = false;

    if (EEPROM_SAVING == 1) //EEPROM_RESET
    {
      Serial.println(F("EEPROM RESET DONE"));
      EEPROM.put(EEPROM_Saddress, -100.00);
      EEPROM.put((EEPROM_Saddress + sizeof(float)), 100.00);

      EEPROM.put((EEPROM_Saddress + sizeof(float)) * 2, 0.00);
      EEPROM.put((EEPROM_Saddress + sizeof(float)) * 3, 100.00);
      PIEZO(PIEZO_PIN,'W',1000);
    }
  }
  if (EEPROM_SAVING == 1) //EEPROM_READ
  {
    EEPROM.get(EEPROM_Saddress, MAX_MIN_RT[MX_Mi_Main].MAX_T_REC);
    EEPROM.get((EEPROM_Saddress + sizeof(float)), MAX_MIN_RT[MX_Mi_Main].MIN_T_REC);

    EEPROM.get((EEPROM_Saddress + sizeof(float)) * 2, MAX_MIN_RT[MX_Mi_Main].MAX_H_REC);
    EEPROM.get((EEPROM_Saddress + sizeof(float)) * 3, MAX_MIN_RT[MX_Mi_Main].MIN_H_REC);

    EEPROM.get((EEPROM_Saddress + sizeof(float)) * 4, DELAY_LOG);
    noise_filtering_Mode = EEPROM.read((EEPROM_Saddress + sizeof(float)) * 5);
    SHT31_heater = EEPROM.read(((EEPROM_Saddress + sizeof(float)) * 5) + sizeof(bool));
  }
  ELinfo_U();
  ELOG(F("[!!!_START_!!!]"), true, true, true);
  ELOG(ELinfo, false, true);
  if (SD.exists(F("EVENT.LOG"))) 
  {
    SD_EVENT_FILE_EX = true; 
  } 
  else 
  {
    SD_EVENT_FILE_EX = false;
  }
  Serial.println(F("setup done--------------"));   
  //END----
  return 1;
}
int loopDebug()
{
  //ELOG
  if (EVENT_LOG_Detail_mode == 1)
  {ELOG(F("LOOP-Start>>>"), true);}
  //display struct
  if (millisDelay(DELAY_SMTUL * SEC, 2)) 
  {
    ST7735_TAB_Main();
  }
  //ERROR CHECKING
  if (EVENT_LOG_Detail_mode == 1)
  {ELOG(F("ERROR CHECKING..."), true);}
  //BMP180
  BS_BMP_180 = BMP180_TEST();
  //SHT31
  BS_SHT31 = SHT31_TEST();
  //SD
  BS_SD_CARD = SD_TEST();
  RAW_TEST();
  //ERROR reporting
  if (!BS_BMP_180) 
  {
    ST7735_TAB_ERROR(F("BMP180.ERROR!!!"));
    ELOG(F("BMP180.ERROR!!!"), true, false, true);
    ERROR_C ++;
  } 
  if (!BS_SHT31) 
  {
    ST7735_TAB_ERROR(F("SHT31.ERROR!!!"));
    ELOG(F("SHT31.ERROR!!!"), true, false, true);
    ERROR_C ++;
  } 
  if (!BS_SD_CARD)
  {
    ST7735_TAB_Warning("Cant find a working  sd card!!!");
  }
  //EEPROM management
  if (BS_SD_CARD == true && !(SD.exists(F("Event.log"))))
  {
    ELOG(F("[!!!_START_!!!]"), true, true, true);
    ELOG(ELinfo, false, true);
    if (EEPROM_SAVING == 1) //EEPROM_RESET
    {
      EEPROM.put(EEPROM_Saddress, -100.00);
      EEPROM.put((EEPROM_Saddress + sizeof(float)), 100.00);

      EEPROM.put((EEPROM_Saddress + sizeof(float)) * 2, 0.00);
      EEPROM.put((EEPROM_Saddress + sizeof(float)) * 3, 100.00);
      PIEZO(PIEZO_PIN,'W',1000);
      if (EVENT_LOG_Detail_mode == 1)
      {ELOG(F("EEPROM RESET..."), true);}
    }
    SD_EVENT_FILE_EX = true;
  }
  else if (BS_SD_CARD == true && SD_EVENT_FILE_EX == false && SD.exists(F("Event.log")))
  {
    ELOG(F("[!!!_START_!!!]"), true, true, true);
    ELOG(ELinfo, false, true);
    PIEZO(PIEZO_PIN,'I', 1000);
    SD_EVENT_FILE_EX = true;
  }
  else if (BS_SD_CARD == false)
  {
    SD_EVENT_FILE_EX = false;    
  }
  //file checking
  SD_MAIN_FILE_EX = SD.exists(F("main.csv"));
  //DATA UPDATE
  if (EVENT_LOG_Detail_mode == 1)
  {ELOG(F("DATA UPDATE..."), true);}
  ST7735_TAB_Main_data(F("dat_U...    "));
  if(BS_SHT31)
  {
    update_SHT31_data();
  }
  if(BS_BMP_180)
  {
    update_airPressure(); 
  }
  if (bluetooth.availableForWrite() == 0)
  {Bluetooth_PRINT();}
  ST7735_TAB_Main_data();
  //bluetooth data read
  //geting_data--------- 
  /*command table
    help
    EL_Info
    SET_LOG.INTERVAL:
    CONST_SAVE
    RESET_DEF
    SOFT_RESET
    NFM_ON
    NFM_OFF
    ignore_ERROR_ON
    ignore_ERROR_OFF
    Data_stream_on
    Data_stream_off
    SHT31_heater_ON
    SHT31_heater_OFF
  */
  while (bluetooth.available())
  {
    char character = bluetooth.read();
    config_B.concat(character);
    ST7735_TAB_Main_data(F("bl_IN raed..."));
  }
  if (!config_B.equals(""))
  {
    #if Debug >= 2
      DebugVal("config_B BL: ", String(config_B));
    #endif
    if (config_B.substring(0, 17).equals(F("SET_LOG.INTERVAL:")))
    {
      ST7735_TAB_Main_data(F("bl_SI        "));
      DELAY_LOG = config_B.substring(17).toFloat();
      bluetooth.print(F("DELAY_LOG:"));
      bluetooth.println(DELAY_LOG);
      PIEZO(PIEZO_PIN, 'I', 1000);
      ELinfo_U();
      ELOG("DELAY_LOG>>>:" + String(DELAY_LOG), true, false, true);
    }
    else if (config_B.substring(0, 10).equals(F("CONST_SAVE")))
    {
      ST7735_TAB_Main_data(F("bl_SI        "));
      bluetooth.println(F("CONST_SAVE..."));
      EEPROM.put((EEPROM_Saddress + sizeof(float)) * 4, DELAY_LOG);
      EEPROM.update((EEPROM_Saddress + sizeof(float)) * 5, noise_filtering_Mode);
      EEPROM.update(((EEPROM_Saddress + sizeof(float)) * 5) + sizeof(bool), SHT31_heater);
      PIEZO(PIEZO_PIN, 'I', 1000);
      ELOG(F("CONST_SAVE..."), true, false, true);
    }
    else if (config_B.substring(0, 6).equals(F("NFM_ON")))
    {
      ST7735_TAB_Main_data(F("bl_SI        "));
      bluetooth.println(F("NFM_ON..."));
      noise_filtering_Mode = true;
      PIEZO(PIEZO_PIN, 'I', 1000);
      ELOG(F("NFM_ON..."), true, false, true);
    }
    else if (config_B.substring(0, 7).equals(F("NFM_OFF")))
    {
      ST7735_TAB_Main_data(F("bl_SI        "));
      bluetooth.println(F("NFM_OFF..."));
      noise_filtering_Mode = false;
      PIEZO(PIEZO_PIN, 'I', 1000);
      ELOG(F("NFM_OFF..."), true, false, true);
    }
    else if (config_B.substring(0, 9).equals(F("RESET_DEF")))
    {
      ST7735_TAB_Main_data(F("bl_SI        "));
      bluetooth.println(F("RESET_DEF..."));
      EEPROM.put((EEPROM_Saddress + sizeof(float)) * 4, D_DELAY_LOG);
      EEPROM.update((EEPROM_Saddress + sizeof(float)) * 5, false);
      EEPROM.update(((EEPROM_Saddress + sizeof(float)) * 5) + sizeof(bool), false);
      DELAY_LOG = D_DELAY_LOG;
      ELinfo_U();
      PIEZO(PIEZO_PIN, 'I', 1000);
      ELOG(F("RESET_DEF..."), true, false, true);
    }
    else if (config_B.substring(0, 10).equals(F("SOFT_RESET")))
    {
      ST7735_TAB_Main_data(F("SOFT_RESET..."));
      bluetooth.println(F("SOFT_RESET..."));
      PIEZO(PIEZO_PIN, 'E', 1000);
      ELOG(F("SOFT_RESET..."), true, false, true);
      resetFunc();
    }
    else if (config_B.substring(0, 7).equals(F("EL_Info")))
    {
      ST7735_TAB_Main_data(F("bl_BL        "));
      bluetooth.println(ELinfo);
    }
    else if (config_B.substring(0, 4).equals(F("BCVS")))
    {
      ST7735_TAB_Main_data(F("bl_BL        "));
      bluetooth.println(BCVS);
    }
    else if (config_B.substring(0, 10).equals(F("mode.dat-R")))
    {
      ST7735_TAB_Main_data(F("bl_SI        "));
      PIEZO(PIEZO_PIN, 'E', 1000);
      ELOG(F("mode.dat-R"), true, false, true);
      BLS_M = 'R';
    }
    else if (config_B.substring(0, 11).equals(F("mode.normal")))
    {
      ST7735_TAB_Main_data(F("bl_SI        "));
      PIEZO(PIEZO_PIN, 'E', 1000);
      ELOG(F("mode.dat-R"), true, false, true);
      BLS_M = 'N';
    }
    else if (config_B.substring(0, 15).equals(F("ignore_ERROR_ON")))
    {
      bluetooth.println(F("ignore_ERROR_ON..."));
      ST7735_TAB_Main_data(F("bl_SI        "));
      PIEZO(PIEZO_PIN, 'E', 1000);
      ELOG(F("ignore_ERROR_ON"), true, false, true);
      ignore_ERROR = true;
    }
    else if (config_B.substring(0, 16).equals(F("ignore_ERROR_OFF")))
    {
      bluetooth.println(F("ignore_ERROR_OFF..."));
      ST7735_TAB_Main_data(F("bl_SI        "));
      PIEZO(PIEZO_PIN, 'E', 1000);
      ELOG(F("ignore_ERROR_OFF"), true, false, true);
      ignore_ERROR = false;
    }
    else if (config_B.substring(0, 15).equals(F("SHT31_heater_ON")))
    {
      bluetooth.println(F("SHT31_heater_ON..."));
      ST7735_TAB_Main_data(F("bl_SI        "));
      PIEZO(PIEZO_PIN, 'E', 1000);
      ELOG(F("SHT31_heater_ON"), true, false, true);
      SHT31_heater = true;
    }
    else if (config_B.substring(0, 16).equals(F("SHT31_heater_OFF")))
    {
      bluetooth.println(F("SHT31_heater_OFF..."));
      ST7735_TAB_Main_data(F("bl_SI        "));
      PIEZO(PIEZO_PIN, 'E', 1000);
      ELOG(F("SHT31_heater_OFF"), true, false, true);
      SHT31_heater = false;
    }
    else if (config_B.substring(0, 4).equals(F("help")))
    {
      bluetooth.println(BL_help_tmp);
    }
    else if (config_B.substring(0, 14).equals(F("Data_stream_on")))
    {
      bluetooth.println(F("Data_stream_on..."));
      BLS_M = 'S';
    }
    else if (config_B.substring(0, 15).equals(F("Data_stream_off")))
    {
      bluetooth.println(F("Data_stream_off..."));
      BLS_M = 'N';
    }
    else if (config_B.substring(0, 14).equals(F("Data_stream_DB")))
    {
      bluetooth.println(F("Enter Esc for escape"));
      bluetooth.println(F("Enter start date (DD/MM/YYYY): "));
      while (bluetooth.available() == 0) 
      {delay(10);}
      String startDate = bluetooth.readStringUntil('\n');
      if (startDate.equals("Esc")) goto BLR_Esc;
      bluetooth.println(F("Enter end date (DD/MM/YYYY): "));
      while (bluetooth.available() == 0) 
      {delay(10);}
      String endDate = bluetooth.readStringUntil('\n');
      if (endDate.equals("Esc")) goto BLR_Esc;
      printMD_bl(startDate, endDate, "main.csv");
    }
    else
    {
      bluetooth.println(F("ERROR: command N/A"));
    }
  }
  BLR_Esc:
  config_B = "";
  //DAY_MATH
  DS3231_get(&t);
  if (t.mday != OLD_DAY)
  {
    TEMPH2 = (AVH_D - TEMPH2);
    TEMPT2 = (AVT_D - TEMPT2);
    MAX_MIN_RT[MX_Mi_Day].HD_PD = TEMPH2;
    MAX_MIN_RT[MX_Mi_Day].TD_PD = TEMPT2;
    TEMPH2 = AVH_D;
    TEMPT2 = AVT_D;
    day_C++;
  }
  if (temperature > MAX_MIN_RT[MX_Mi_Day].MAX_T_REC){
    MAX_MIN_RT[MX_Mi_Day].MAX_T_REC = temperature;
  }
  if (temperature < MAX_MIN_RT[MX_Mi_Day].MIN_T_REC){ 
    MAX_MIN_RT[MX_Mi_Day].MIN_T_REC = temperature;
  }
  if (humidity > MAX_MIN_RT[MX_Mi_Day].MAX_H_REC){
    MAX_MIN_RT[MX_Mi_Day].MAX_H_REC = humidity;
  }
  if (humidity < MAX_MIN_RT[MX_Mi_Day].MIN_H_REC){
     MAX_MIN_RT[MX_Mi_Day].MIN_H_REC = humidity;
  } 
  AV_C = (AV_C + 1);
  AVT_D_TEMP = (temperature + AVT_D_TEMP);
  AVH_D_TEMP = (humidity + AVH_D_TEMP);
  AVT_D = AVT_D_TEMP / AV_C;
  AVH_D = AVH_D_TEMP / AV_C;
  if (START == true || day_C == 2){
    MAX_MIN_RT[MX_Mi_Day].HD_PD = 0;
    MAX_MIN_RT[MX_Mi_Day].TD_PD = 0;
  } 
  if (START == false && t.mday != OLD_DAY){DAY_LOG();}
  if (t.mday != OLD_DAY)
  {
    MAX_MIN_RT[MX_Mi_Day].MAX_T_REC = -100.00;
    MAX_MIN_RT[MX_Mi_Day].MIN_T_REC = 100.00;
    MAX_MIN_RT[MX_Mi_Day].MAX_H_REC = 0;
    MAX_MIN_RT[MX_Mi_Day].MIN_H_REC = 100.00;
    AVT_D_TEMP = 0;
    AVH_D_TEMP = 0;
    AVT_D = 0;
    AVH_D = 0;
    MAX_MIN_RT[MX_Mi_Day].TD_PD = 0;
    MAX_MIN_RT[MX_Mi_Day].HD_PD = 0;
    AV_C = 0;
  }
  //Month_MATH
  if (t.mon != OLD_MON)
  {
    MAX_MIN_RT[MX_Mi_Month].MAX_T_REC = -100.00;
    MAX_MIN_RT[MX_Mi_Month].MIN_T_REC = 100.00;
    MAX_MIN_RT[MX_Mi_Month].MAX_H_REC = 0;
    MAX_MIN_RT[MX_Mi_Month].MIN_H_REC = 100.00;
  }
  if (temperature > MAX_MIN_RT[MX_Mi_Month].MAX_T_REC){
    MAX_MIN_RT[MX_Mi_Month].MAX_T_REC = temperature;
  }
  if (temperature < MAX_MIN_RT[MX_Mi_Month].MIN_T_REC){ 
    MAX_MIN_RT[MX_Mi_Month].MIN_T_REC = temperature;
  }
  if (humidity > MAX_MIN_RT[MX_Mi_Month].MAX_H_REC){
    MAX_MIN_RT[MX_Mi_Month].MAX_H_REC = humidity;
  }
  if (humidity < MAX_MIN_RT[MX_Mi_Month].MIN_H_REC){
     MAX_MIN_RT[MX_Mi_Month].MIN_H_REC = humidity;
  }  
  //Year_MATH
  if (t.year != OLD_YEAR)
  {
    MAX_MIN_RT[MX_Mi_year].MAX_T_REC = -100.00;
    MAX_MIN_RT[MX_Mi_year].MIN_T_REC = 100.00;
    MAX_MIN_RT[MX_Mi_year].MAX_H_REC = 0;
    MAX_MIN_RT[MX_Mi_year].MIN_H_REC = 100.00;
  }
  if (temperature > MAX_MIN_RT[MX_Mi_year].MAX_T_REC){
    MAX_MIN_RT[MX_Mi_year].MAX_T_REC = temperature;
  }
  if (temperature < MAX_MIN_RT[MX_Mi_year].MIN_T_REC){ 
    MAX_MIN_RT[MX_Mi_year].MIN_T_REC = temperature;
  }
  if (humidity > MAX_MIN_RT[MX_Mi_year].MAX_H_REC){
    MAX_MIN_RT[MX_Mi_year].MAX_H_REC = humidity;
  }
  if (humidity < MAX_MIN_RT[MX_Mi_year].MIN_H_REC){
     MAX_MIN_RT[MX_Mi_year].MIN_H_REC = humidity;
  }  
  //SET_O::D/M/Y
  OLD_DAY = t.mday;
  OLD_MON = t.mon;
  OLD_YEAR = t.year;
  //max/min(M1)----------------
  if (MAX_MIN_UPDATE_MODE == 1)
  {
    if (temperature > MAX_MIN_RT[MX_Mi_Main].MAX_T_REC){
      MAX_MIN_RT[MX_Mi_Main].MAX_T_REC = temperature;
    }
    if (temperature < MAX_MIN_RT[MX_Mi_Main].MIN_T_REC){
      MAX_MIN_RT[MX_Mi_Main].MIN_T_REC = temperature;
    }
    if (humidity > MAX_MIN_RT[MX_Mi_Main].MAX_H_REC){
      MAX_MIN_RT[MX_Mi_Main].MAX_H_REC = humidity;
    }
    if (humidity < MAX_MIN_RT[MX_Mi_Main].MIN_H_REC){
      MAX_MIN_RT[MX_Mi_Main].MIN_H_REC = humidity;
    }
  } 
  //LOGGING DATA
  if (millisDelay((DELAY_LOG * Minute), 3, 0, false, noise_filtering_Mode) && BS_SD_CARD)
  { 
    //math--------------
    //TD/HD
    TEMPH = (humidity - TEMPH);
    TEMPT = (temperature - TEMPT);
    MAX_MIN_RT[MX_Mi_Main].HD_PD = TEMPH;
    MAX_MIN_RT[MX_Mi_Main].TD_PD = TEMPT;
    TEMPH = humidity;
    TEMPT = temperature;
    if (START_L_EX == true){
      MAX_MIN_RT[MX_Mi_Main].HD_PD = 0;
      MAX_MIN_RT[MX_Mi_Main].TD_PD = 0;
    }
    //max/min(M0)
    if (MAX_MIN_UPDATE_MODE == 0)
    {
      if (temperature > MAX_MIN_RT[MX_Mi_Main].MAX_T_REC){
        MAX_MIN_RT[MX_Mi_Main].MAX_T_REC = temperature;
      }
      if (temperature < MAX_MIN_RT[MX_Mi_Main].MIN_T_REC){
        MAX_MIN_RT[MX_Mi_Main].MIN_T_REC = temperature;
      }
      if (humidity > MAX_MIN_RT[MX_Mi_Main].MAX_H_REC){
        MAX_MIN_RT[MX_Mi_Main].MAX_H_REC = humidity;
      }
      if (humidity < MAX_MIN_RT[MX_Mi_Main].MIN_H_REC){
        MAX_MIN_RT[MX_Mi_Main].MIN_H_REC = humidity;
      }
    }   
    //EEPROM_SAVING
    if (EEPROM_SAVING == 1)
    {
      EEPROM.put(EEPROM_Saddress, MAX_MIN_RT[MX_Mi_Main].MAX_T_REC);
      EEPROM.put((EEPROM_Saddress + sizeof(float)), MAX_MIN_RT[MX_Mi_Main].MIN_T_REC);

      EEPROM.put((EEPROM_Saddress + sizeof(float)) * 2, MAX_MIN_RT[MX_Mi_Main].MAX_H_REC);
      EEPROM.put((EEPROM_Saddress + sizeof(float)) * 3, MAX_MIN_RT[MX_Mi_Main].MIN_H_REC);
    }
    //main logging
    ELOG(F("LOGGING DATA..."), true);
    if (BS_SD_CARD == true)
    {
      SD.mkdir(F("month"));
      SD.mkdir(F("year"));
    }  
    if(SAVE_DATA_TYPE.equals(csv)) // CSV
    {
      DS3231_get(&t);
      //main log
      SD_CARD = SD.open(F("main.csv"), FILE_WRITE);
      if (SD_MAIN_FILE_EX == false)
      {CSV_H();}//printing the CSV header
      CSV_LOG_STRUCT();
      SD_CARD.close(); 
      //mon log
      SD_CARD = SD.open("month/" + String(t.mon) + "_" + String(t.year) + ".csv", FILE_WRITE);
      if (SD_MAIN_FILE_EX == false)
      {CSV_H();}//printing the CSV header
      CSV_LOG_STRUCT();
      SD_CARD.close(); 
      //year log
      SD_CARD = SD.open("year/" + String(t.year) + ".csv", FILE_WRITE);
      if (SD_MAIN_FILE_EX == false)
      {CSV_H();}//printing the CSV header
      CSV_LOG_STRUCT();
      SD_CARD.close(); 
    }
    else if (SAVE_DATA_TYPE.equals(txt)) //TXT
    {
      DS3231_get(&t);
      //main
      SD_CARD = SD.open(F("main.txt"), FILE_WRITE);
      TXT_LOG_STRUCT();
      SD_CARD.close(); 
      //year log
      SD_CARD = SD.open("year/" + String(t.year) + ".txt", FILE_WRITE);
      TXT_LOG_STRUCT();
      SD_CARD.close(); 
      //mon log
      SD_CARD = SD.open("month/" + String(t.mon) + "_" + String(t.year) + ".txt", FILE_WRITE);
      TXT_LOG_STRUCT();
      SD_CARD.close(); 
    }
    START_L_EX = false;
  }
  //LCD data PRINT------
  ST7735_TAB_Main_data();
  //SERIAL PRINT DATE/TIME
  Serial.print(F("Date : "));
  Serial.print(t.mday);
  Serial.print(F("/"));
  Serial.print(t.mon);
  Serial.print(F("/"));
  Serial.print(t.year);
  Serial.print(F("\t Hour : "));
  Serial.print(t.hour);
  Serial.print(F(":"));
  Serial.print(t.min);
  Serial.print(F("."));
  Serial.println(t.sec);
  Serial.println(F("-------------------------------------------"));  
  if (EVENT_LOG_Detail_mode == 1)
  {ELOG(F("LOOP-End<<<"), true);}
  START = false;
  //END----
  return 151; //151 = loop
}
//DRTS-----------------
void Debug_RTS1()
{
}
void Debug_RTS2()
{ 
}
void Debug_RTS3()
{
}