#define Ver "Debug tools V_0.8.3"
#define Defult_Serial_speed 9600
#define SRAM_Limit 1300
#define SRAM_ERROR 1
#define Auto_SRAM_M 1
#define T_M 1
#define Serial_R 1
#define Auto_STOP_C 5  // 0 for inf
#define MCH_A
#define Arduino_DebugUtils 
#define Arduino_DebugUtils_MODE DBG_VERBOSE  
#define CCL_VDT uint8_t         //byte/String/char/float/...
#define DebugVal_VDT bool  //byte/String/char/float/...
//DRTS_P---------
#define DRTS_P_L1 
#define DRTS_P_L2
#define DRTS_P_L3
#define DRTS_P_L4
//DS-------------
#define DS1
#define DS2
#define DS3
#define DS4
//SABP-----------
#define BP_EC1
#define BP_EC2
#define BP_EC3
#define BP_EC4
//---------------
/*
DRTS_P (Debug runtime state package) instructions:

DRTS_P_L1 [for Startup]
DRTS_P_L2 [for after SetupDebug]
DRTS_P_L3 [for LoopDebug (runs with each LoopDebug end)]
DRTS_P_L4 [for loopDebug finish]

Elements::

T_MC_start [timer start]
T_MC_End   [timer end]
DRTS1 [func (DRTS1) run]
DRTS2 [func (DRTS2) run]
DRTS3 [func (DRTS3) run]

EXAMPLE::

T_MC_start DRTS1 T_MC_End

T_M (time monitoring): 1/0

!!!------------------------!!!
(MCH) code tools:

[!!!Warning Thise only work if MCH_A is defined in Main_code.h!!!]

GENERAL:

DebugVal(String Name, DebugVal_VDT Val);
CCL(CCL_VDT i);
timer_start
timer_end
freeRam_internal();
freeRam();
runDS1();
runDS2();
runDS3();
runDS4();

BRAKE POINT:

BP1
BP2
BP3
BP4

Arduino_DebugUtils:

[!!!Warning you need to put ADU_I macro in setupDebug!!!]

https://github.com/arduino-libraries/Arduino_DebugUtils

you can add more in MCH.h file
*/