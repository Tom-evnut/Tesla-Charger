/*
  Tesla Gen 2 Charger Control Program
  2017-2018
  T de Bree
  D.Maguire
  Additional work by C. Kidder
  Runs on OpenSource Logic board V2 in Gen2 charger. Commands all modules.
*/

#include <can_common.h>
#include <due_can.h>
#include <due_wire.h>
#include <Wire_EEPROM.h>
#include <DueTimer.h>
#include "config.h"
#include <rtc_clock.h>  ///https://github.com/MarkusLange/Arduino-Due-RTC-Library
#include <PID_v1.h>

#define Serial SerialUSB

#define SUPC_KEY (0xA5)

template<class T> inline Print &operator<<(Print &obj, T arg) {
  obj.print(arg);
  return obj;
}

int firmware = 230822;

//////////////Set the hardware version/////////////////

//int hardware = BoardV5;
int hardware = BoardV4;

RTC_clock rtc_clock(XTAL);

int watchdogTime = 8000;

char *daynames[] = { "Mon", "Tue", "Wed", "Thu", "Fri", "Sat", "Sun" };

//////////////Setup PID for DC current regulation/////////////////

#define PIN_INPUT 0
#define PIN_OUTPUT 3

//Define Variables we'll be connecting to
double Setpoint, Input, Output;

//Specify the links and initial tuning parameters
double Kp = 2, Ki = 5, Kd = 1;
PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);


//*********GENERAL VARIABLE   DATA ******************
int evsedebug = 1;  // 1 = show Proximity status and Pilot current limmits
int debug = 1;      // 1 = show phase module CAN feedback


uint16_t curset = 0;
int setting = 1;
int incomingByte = 0;
int state;
unsigned long sleeptime, slavetimeout, tlast, tcan, tboot, droptime = 0;
bool dropsignal = 0;
bool bChargerEnabled;

float DCLim = 0;
int L1 = 0;

//**************Sleep Variables****************

//*********EVSE VARIABLE   DATA ******************
byte Proximity = 0;
uint16_t ACvoltIN = 240;  // AC input voltage 240VAC for EU/UK and 110VAC for US
//proximity status values for type 1
#define Unconnected 0  // 3.3V
#define Buttonpress 1  // 2.3V
#define Connected 2    // 1.35V

volatile uint32_t pilottimer = 0;
volatile uint16_t timehigh, duration = 0;
volatile uint16_t accurlim = 0;
volatile int dutycycle = 0;

uint16_t cablelim = 0;  // Type 2 cable current limit

//*********Single or Three Phase Config VARIABLE   DATA ******************

//proximity status values
#define Singlephase 0  // all parrallel on one phase Type 1
#define Threephase 1   // one module per phase Type 2

//*********Charger Control VARIABLE   DATA ******************
bool Vlimmode = true;  // Set charges to voltage limit mode
uint16_t modulelimcur, dcaclim = 0;
uint32_t maxaccur = 16000;  //maximum AC current in mA
uint16_t maxdccur = 45000;  //max DC current outputin mA
int activemodules, slavechargerenable = 0;
bool LockOut = false;     //lockout on termination voltage reached. Reset by evse plug recycle.
uint16_t LockOutCnt = 0;  // lockout counter


//*********Feedback from charge VARIABLE   DATA ******************
uint16_t dcvolt[3] = { 0, 0, 0 };  //1 = 1V
uint16_t dccur[3] = { 0, 0, 0 };
uint16_t totdccur = 0;             //1 = 0.005Amp
uint16_t acvolt[3] = { 0, 0, 0 };  //1 = 1V
uint16_t accur[3] = { 0, 0, 0 };   //1 = 0.06666 Amp
long acpower = 0;
byte inlettarg[3] = { 0, 0, 0 };                    //inlet target temperatures, should be used to command cooling.
byte curtemplim[3] = { 0, 0, 0 };                   //current limit due to temperature
byte templeg[2][3] = { { 0, 0, 0 }, { 0, 0, 0 } };  //temperatures reported back
bool ACpres[3] = { 0, 0, 0 };                       //AC present detection on the modules
bool ModEn[3] = { 0, 0, 0 };                        //Module enable feedback on the modules
bool ModFlt[3] = { 0, 0, 0 };                       //module fault feedback
byte ModStat[3] = { 0, 0, 0 };                      //Module Status
int newframe = 0;

ChargerParams parameters;

//*********DCDC Messages VARIABLE   DATA ******************
bool dcdcenable = 0;  // turn on can messages for the DCDC.

//*********Charger Messages VARIABLE   DATA ******************
int ControlID = 0x300;
int StatusID = 0x410;
unsigned long ElconID = 0x18FF50E5;
unsigned long ElconControlID = 0x1806E5F4;


int candebug = 0;
int menuload = 0;

// this function has to be present, otherwise watchdog won't work
void watchdogSetup(void) {
  // do what you want here
}

void setup() {
  pmc_set_writeprotect(false);
  pmc_mck_set_prescaler(16);

  // 12MHz / 64 * 14 = 2.625MHz //  96 = 110 << 4 = /64
  // 12MHz / 32 * 14 = 5.25 MHz //  80 = 101 << 4 = /32
  // 12MHz / 16 * 14 = 10.5 MHz //  64 = 100 << 4 = /16
  // 12MHz /  8 * 14 = 21   MHz //  48 = 011 << 4 = /8
  // 12MHz /  4 * 14 = 42   MHz //  32 = 010 << 4 = /4
  // 12MHz /  2 * 14 = 84   MHz //  16 = 001 << 4 = /2 (default)

  pmc_disable_periph_clk(22);  // TWI/I2C bus 0 (i.MX6 controlling)
  pmc_disable_periph_clk(23);  // TWI/I2C bus 1
  pmc_disable_periph_clk(24);  // SPI0
  pmc_disable_periph_clk(25);  // SPI1
  pmc_disable_periph_clk(26);  // SSC (I2S digital audio, N/C)

  pmc_disable_periph_clk(41);  // random number generator
  pmc_disable_periph_clk(42);  // ethernet MAC - N/C


  Serial.begin(115200);  //Initialize our USB port which will always be redefined as SerialUSB to use the Native USB port tied directly to the SAM3X processor.

  Timer3.attachInterrupt(Ext_msgs).start(90000);  // charger messages every 100ms

  attachInterrupt(EVSE_PILOT, Pilotread, CHANGE);
  watchdogEnable(watchdogTime);
  Wire.begin();
  EEPROM.read(0, parameters);
  if (parameters.version != EEPROM_VERSION) {
    parameters.version = EEPROM_VERSION;
    parameters.currReq = 0;            //max input limit per module 1500 = 1A
    parameters.enabledChargers = 123;  // enable per phase - 123 is all phases - 3 is just phase 3
    parameters.can0Speed = 500000;
    parameters.can1Speed = 250000;
    parameters.mainsRelay = 48;
    parameters.voltSet = 32000;           //1 = 0.01V
    parameters.tVolt = 34000;             //1 = 0.01V
    parameters.autoEnableCharger = 0;     //disable auto start, proximity and pilot control
    parameters.canControl = 0;            //0 disabled can control, 1 master, 3 slave
    parameters.dcdcsetpoint = 14000;      //voltage setpoint for dcdc in mv
    parameters.phaseconfig = Threephase;  //AC input configuration
    parameters.type = 2;                  //Socket type1 or 2
    parameters.sleeptimeout = 30000;      // mS before entering sleep
    EEPROM.write(0, parameters);
  }

  ////rtc clock start///////

  rtc_clock.init();

  //rtc_clock.set_time(19, 35, 0);
  //rtc_clock.set_date(16, 11, 2018);

  // Initialize CAN ports
  if (Can1.begin(parameters.can1Speed, 255))  //can1 external bus
  {
    Serial.println("Using CAN1 - initialization completed.\n");
  } else Serial.println("CAN1 initialization (sync) ERROR\n");


  // Initialize CAN0
  if (Can0.begin(parameters.can0Speed, 255))  //can0 charger modules
  {
    Serial.println("Using CAN0 - initialization completed.\n");
  } else Serial.println("CAN0 initialization (sync) ERROR\n");

  int filter;
  //extended
  for (filter = 0; filter < 3; filter++) {
    Can0.setRXFilter(filter, 0, 0, true);
    Can1.setRXFilter(filter, 0, 0, true);
  }
  //standard
  for (int filter = 3; filter < 7; filter++) {
    Can0.setRXFilter(filter, 0, 0, false);
    Can1.setRXFilter(filter, 0, 0, false);
  }
  ///////////////////CHARGER ENABLE AND ACTIVATE LINES///////////////////////////////////

  //////////////////////////////////////////////////////////////////////////////////////

  ///////////////////CHARGER ENABLE AND ACTIVATE LINES///////////////////////////////////
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(CHARGER1_ENABLE, OUTPUT);    //CHG1 ENABLE
  pinMode(CHARGER2_ENABLE, OUTPUT);    //CHG2 ENABLE
  pinMode(CHARGER3_ENABLE, OUTPUT);    //CHG3 ENABLE
  pinMode(CHARGER1_ACTIVATE, OUTPUT);  //CHG1 ACTIVATE
  pinMode(CHARGER2_ACTIVATE, OUTPUT);  //CHG2 ACTIVATE
  pinMode(CHARGER3_ACTIVATE, OUTPUT);  //CHG3 ACTIVATE

  digitalWrite(CHARGER1_ACTIVATE, LOW);  //chargeph1 deactivate
  digitalWrite(CHARGER2_ACTIVATE, LOW);  //chargeph2 deactivate
  digitalWrite(CHARGER3_ACTIVATE, LOW);  //chargeph3 deactivate
  digitalWrite(CHARGER1_ENABLE, LOW);    //disable phase 1 power module
  digitalWrite(CHARGER2_ENABLE, LOW);    //disable phase 2 power module
  digitalWrite(CHARGER3_ENABLE, LOW);    //disable phase 3 power module
  //////////////////////////////////////////////////////////////////////////////////////


  pinMode(DIG_IN_1, INPUT);  //IP1
  pinMode(DIG_IN_2, INPUT);  //IP2
  //////////////////////////////////////////////////////////////////////////////////////

  //////////////DIGITAL OUTPUTS MAPPED TO X046. 10 PIN CONNECTOR ON LEFT//////////////////////////////////////////
  pinMode(DIG_OUT_1, OUTPUT);      //OP1 - X046 PIN 6
  pinMode(DIG_OUT_2, OUTPUT);      //OP2
  pinMode(DIG_OUT_3, OUTPUT);      //OP2
  pinMode(DIG_OUT_4, OUTPUT);      //OP3
  pinMode(EVSE_ACTIVATE, OUTPUT);  //pull Pilot to 6V

  digitalWrite(EVSE_ACTIVATE, LOW);
  digitalWrite(DIG_OUT_1, LOW);  //MAINS OFF
  digitalWrite(DIG_OUT_2, LOW);  //AC OFF when in manual mode.
  digitalWrite(DIG_OUT_3, LOW);
  digitalWrite(DIG_OUT_4, LOW);
  ///////////////////////////////////////////////////////////////////////////////////////

  ///////////////Setup Sleep Mode Wake Ups////////////////////////////////////////////////////////////////////////

  uint16_t tmp = ((1 << 12) | (1 << 5) | (1 << 1) | 1);  // Wake up on CAN1 RX AND D19 AND D60 AND CAN0 RX
  SUPC->SUPC_WUIR = tmp;                                 // above sources, low-> high transition
  SUPC->SUPC_WUMR = 0;                                   // try no debouncing; 1 << 12 for 3 cycle

  ///////////////////////////////////////////////////////////////////////////////////////
  dcaclim = maxaccur;
  bChargerEnabled = false;  //are we supposed to command the charger to charge?
  //

  myPID.SetMode(AUTOMATIC);  //start PID

  sleeptime = millis();  //update timer
}

void loop() {
  CAN_FRAME incoming;

  if (Can0.available()) {
    Can0.read(incoming);
    candecode(incoming);
    sleeptime = millis();  //update timer
  }

  if (Can1.available()) {
    //Serial.println();
    //Serial.print("can 1 data");
    Can1.read(incoming);
    canextdecode(incoming);
    sleeptime = millis();  //update timer
  }

  if (Serial.available()) {
    menu();
    sleeptime = millis();  //update timer
  }

  if (parameters.canControl > 1) {
    if (state != 0) {
      if (millis() - tcan > 2000) {
        state = 0;
        Serial.println();
        Serial.println("CAN time-out");
      }
    }
  }

  if (digitalRead(DIG_IN_1) == LOW) {
    state = 0;
  }

  switch (state) {
    case 0:  //Charger off
      //Serial.println();
      //Serial.println("Enter State 0");
      //Serial.println();
      if (bChargerEnabled == true) {
        bChargerEnabled = false;
      }
      digitalWrite(DIG_OUT_1, LOW);  //MAINS OFF\
      digitalWrite(EVSE_ACTIVATE, LOW);
      //digitalWrite(DIG_OUT_2, LOW);//AC OFF when in manual mode.
      digitalWrite(CHARGER1_ACTIVATE, LOW);  //chargeph1 deactivate
      digitalWrite(CHARGER2_ACTIVATE, LOW);  //chargeph2 deactivate
      digitalWrite(CHARGER3_ACTIVATE, LOW);  //chargeph3 deactivate
      digitalWrite(CHARGER1_ENABLE, LOW);    //disable phase 1 power module
      digitalWrite(CHARGER2_ENABLE, LOW);    //disable phase 2 power module
      digitalWrite(CHARGER3_ENABLE, LOW);    //disable phase 3 power module
      for (int y; y < 3; y++) {
        dcvolt[y] = 0;
        dccur[y] = 0;
        totdccur = 0;  //1 = 0.005Amp
        acvolt[y] = 0;
        accur[y] = 0;
      }
      break;

    case 1:                  //Charger on
      sleeptime = millis();  //update timer
      if (digitalRead(DIG_IN_1) == HIGH) {
        if (bChargerEnabled == false) {
          bChargerEnabled = true;
          switch (parameters.enabledChargers) {
            case 1:
              digitalWrite(CHARGER1_ACTIVATE, HIGH);
              activemodules = 1;
              break;
            case 2:
              digitalWrite(CHARGER2_ACTIVATE, HIGH);
              activemodules = 1;
              break;
            case 3:
              digitalWrite(CHARGER3_ACTIVATE, HIGH);
              activemodules = 1;
              break;
            case 12:
              digitalWrite(CHARGER1_ACTIVATE, HIGH);
              digitalWrite(CHARGER2_ACTIVATE, HIGH);
              activemodules = 2;
              break;
            case 13:
              digitalWrite(CHARGER1_ACTIVATE, HIGH);
              digitalWrite(CHARGER3_ACTIVATE, HIGH);
              activemodules = 2;
              break;
            case 123:
              digitalWrite(CHARGER1_ACTIVATE, HIGH);
              digitalWrite(CHARGER2_ACTIVATE, HIGH);
              digitalWrite(CHARGER3_ACTIVATE, HIGH);
              activemodules = 3;
              break;
            case 23:
              digitalWrite(CHARGER2_ACTIVATE, HIGH);
              digitalWrite(CHARGER3_ACTIVATE, HIGH);
              activemodules = 2;
              break;
            default:
              // if nothing else matches, do the default
              // default is optional
              break;
          }
          delay(100);
          digitalWrite(DIG_OUT_1, HIGH);  //MAINS ON
          digitalWrite(EVSE_ACTIVATE, HIGH);
          digitalWrite(DIG_OUT_2, HIGH);  //AC on in manual mode
        }
      } else {
        bChargerEnabled = false;
        state = 0;
      }
      break;

    case 2:
      switch (parameters.enabledChargers) {
        case 1:
          digitalWrite(CHARGER1_ENABLE, HIGH);  //enable phase 1 power module
          break;
        case 2:
          digitalWrite(CHARGER2_ENABLE, HIGH);  //enable phase 2 power module
          break;
        case 3:
          digitalWrite(CHARGER3_ENABLE, HIGH);  //enable phase 3 power module
          break;
        case 12:
          digitalWrite(CHARGER1_ENABLE, HIGH);  //enable phase 1 power module
          digitalWrite(CHARGER2_ENABLE, HIGH);  //enable phase 2 power module
          break;
        case 13:
          digitalWrite(CHARGER1_ENABLE, HIGH);  //enable phase 1 power module
          digitalWrite(CHARGER3_ENABLE, HIGH);  //enable phase 3 power module
          break;
        case 123:
          digitalWrite(CHARGER1_ENABLE, HIGH);  //enable phase 1 power module
          digitalWrite(CHARGER2_ENABLE, HIGH);  //enable phase 2 power module
          digitalWrite(CHARGER3_ENABLE, HIGH);  //enable phase 3 power module
          break;
        case 23:
          digitalWrite(CHARGER2_ENABLE, HIGH);  //enable phase 2 power module
          digitalWrite(CHARGER3_ENABLE, HIGH);  //enable phase 3 power module
          break;

        default:
          // if nothing else matches, do the default
          break;
      }
      if (tboot < (millis() - 1000))  //delay in ms before moving to state 1.
      {
        state = 1;
      }

    default:
      // if nothing else matches, do the default
      break;
  }
  if (tlast < (millis() - 500)) {
    tlast = millis();
    evseread();
    //autoShutdown();
    watchdogReset();
    if (debug != 0) {
      if (hardware == BoardV5) {
        if (millis() - sleeptime > parameters.sleeptimeout - 10000) {
          Serial.println();
          Serial.println();
          Serial.println();
          Serial.print("!!!!! Sleep in: ");
          Serial.print((sleeptime - (millis() - parameters.sleeptimeout)) * 0.001);
          Serial.print(" S");
        }
      }
      Serial.println();
      Serial.print(millis());
      Serial.print(" State: ");
      Serial.print(state);
      Serial.print(" Phases : ");
      Serial.print(parameters.phaseconfig);
      Serial.print(" Modules Active : ");
      Serial.print(activemodules);
      if (bChargerEnabled) {
        Serial.print(" ON  ");
      } else {
        Serial.print(" OFF ");
      }
      if (digitalRead(DIG_IN_1) == HIGH) {
        Serial.print(" D1 H");
      } else {
        Serial.print(" D1 L");
      }
      if (digitalRead(DIG_OUT_1) == HIGH) {
        Serial.print(" O1 H");
      } else {
        Serial.print(" O1 L");
      }
      if (digitalRead(DIG_OUT_2) == HIGH) {
        Serial.print(" O2 H");
      } else {
        Serial.print(" O2 L");
      }
      Serial.print(" Droptime : ");
      Serial.print(droptime);
      if (bChargerEnabled) {
        Serial.println();
        for (int x = 0; x < 3; x++) {
          Serial.print("  Phase ");
          Serial.print(x + 1);
          Serial.print(" Feebback //  AC present: ");
          Serial.print(ACpres[x]);
          Serial.print("  AC volt: ");
          Serial.print(acvolt[x]);
          Serial.print("  AC cur: ");
          Serial.print((accur[x] * 0.06666), 2);
          //Serial.print("  ");
          ///Serial.print(accur[x], HEX); ///not needed since current fixed
          Serial.print("  DC volt: ");
          Serial.print(dcvolt[x]);
          Serial.print("  DC cur: ");
          Serial.print(dccur[x], 2);
          Serial.print("  Inlet Targ: ");
          Serial.print(inlettarg[x]);
          Serial.print("  Temp Lim Cur: ");
          Serial.print(curtemplim[x]);
          Serial.print("  ");
          Serial.print(templeg[0][x]);
          Serial.print("  ");
          Serial.print(templeg[1][x]);
          Serial.print(" EN:");
          Serial.print(ModEn[x]);
          Serial.print(" Flt:");
          Serial.print(ModFlt[x]);
          Serial.print(" Stat:");
          Serial.print(ModStat[x], BIN);
          Serial.println();
        }
      } else {
        Serial.println();
        Serial.print("Modules Turned OFF");
        Serial.println();
      }
    }
    if (debug == 1) {
      if (evsedebug != 0) {
        Serial.println();
        Serial.print("  Proximity Status : ");
        switch (Proximity) {
          case Unconnected:
            Serial.print("Unconnected");
            break;
          case Buttonpress:
            Serial.print("Button Pressed");
            break;
          case Connected:
            Serial.print("Connected");
            break;
        }
        Serial.print(" AC limit : ");
        Serial.print(accurlim);
        Serial.print(" /Cable Limit: ");
        Serial.print(cablelim);
        Serial.print(" /Module Cur Request: ");
        Serial.print(modulelimcur / 1.5, 0);
        Serial.print(" /DC total Cur:");
        Serial.print(totdccur, 2);
        Serial.print(" /DC Setpoint:");
        Serial.print(parameters.voltSet * 0.01, 0);
        Serial.print(" /DC Lim: ");
        Serial.print(maxdccur);
      }
    }
  }
  //DCcurrentlimit();
  DClimCalc();
  ACcurrentlimit();

  //resetFaults();

  //EVSE automatic control

  if (parameters.autoEnableCharger == 1) {
    if (Proximity == Connected)  //check if plugged in
    {
      //digitalWrite(EVSE_ACTIVATE, HIGH);//pull pilot low to indicate ready - NOT WORKING freezes PWM reading
      if (accurlim > 1400)  // one amp or more active modules
      {
        if (state == 0) {
          if (digitalRead(DIG_IN_1) == HIGH) {
            state = 2;  // initialize modules
            tboot = millis();
          }
        }
      }
      digitalWrite(DIG_OUT_2, HIGH);  //enable AC present indication
    } else                            // unplugged or buton pressed stop charging
    {
      if (dropsignal == 0) {
        droptime == millis();
        dropsignal == 1;
      } else {
        if (millis() - millis() < 1000) {
          state = 0;
          digitalWrite(DIG_OUT_2, LOW);  //disable AC present indication
          dropsignal == 0;
        }
      }
    }
  }
  if (hardware == BoardV5)  ///Ensure to only sleep on boards that can wake up
  {
    if (millis() - sleeptime > parameters.sleeptimeout && menuload != 1) {
      Serial.println();
      Serial.println();
      Serial.print("Entering Sleep");
      delay(10);
      pmc_enable_backupmode();
    }
  }


}  //end of loop

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//If a charger power module is running and faults out we want to reset and go again otherwise charger can just sit there and not finish a charge.
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void resetFaults() {


  if ((bChargerEnabled == true) && (ACpres[0] == true) && (ModFlt[0] == true) && ((parameters.enabledChargers == 1) || (parameters.enabledChargers == 12) || (parameters.enabledChargers == 13) || (parameters.enabledChargers == 123))) {
    //if these conditions are met then phase one is enabled, has ac present and has entered a fault state so we want to reset.
    state = 0;
    //digitalWrite(DIG_OUT_2, LOW); //disable AC present indication;
    //  digitalWrite(EVSE_ACTIVATE, LOW);
  }
  if ((bChargerEnabled == true) && (ACpres[1] == true) && (ModFlt[1] == true) && ((parameters.enabledChargers == 2) || (parameters.enabledChargers == 12) || (parameters.enabledChargers == 23) || (parameters.enabledChargers == 123))) {
    //if these conditions are met then phase two is enabled, has ac present and has entered a fault state so we want to reset.
    state = 0;
    //digitalWrite(DIG_OUT_2, LOW); //disable AC present indication;
    //digitalWrite(EVSE_ACTIVATE, LOW);
  }
  if ((bChargerEnabled == true) && (ACpres[2] == true) && (ModFlt[2] == true) && ((parameters.enabledChargers == 3) || (parameters.enabledChargers == 13) || (parameters.enabledChargers == 23) || (parameters.enabledChargers == 123))) {
    //if these conditions are met then phase three is enabled, has ac present and has entered a fault state so we want to reset.
    state = 0;
    //digitalWrite(DIG_OUT_2, LOW); //disable AC present indication;
    //digitalWrite(EVSE_ACTIVATE, LOW);
  }
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//If the HV voltage exceeds the tVolt setpoint we want to shut down the charger and not re enable until the charge plug
//is removed and re connected. For now we just read the voltage on phase module one.
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void autoShutdown() {
  if ((bChargerEnabled == true) && (LockOut == false))  //if charger is running and we are not locked out ...
  {
    if (dcvolt[0] > (parameters.tVolt * 0.01))  //and if we exceed tVolt...
    {
      LockOutCnt++;  //increment the lockout counter
      //      LockOut=true; //lockout and shutdown
      //    state=0;
    } else {
      LockOutCnt = 0;  //other wise we reset the lockout counter
    }
  }
  if (Proximity == Unconnected && (parameters.autoEnableCharger == 1)) LockOut = false;  //re set the lockout flag when the evse plug is pulled only if we are in evse mode.

  if (LockOutCnt > 10) {
    state = 0;       //if we are above our shutdown targer for 10 consecutive counts we lockout
    LockOut = true;  //lockout and shutdown
    LockOutCnt = 0;
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


void candecode(CAN_FRAME &frame) {
  int x = 0;
  switch (frame.id) {
    case 0x217:  //phase 1 Status message
      ModStat[0] = frame.data.bytes[0];
      break;

    case 0x219:  //phase 2 Status message
      ModStat[1] = frame.data.bytes[0];
      break;

    case 0x21B:  //phase 3 Status message
      ModStat[2] = frame.data.bytes[0];
      break;

    case 0x24B:  //phase 3 temp message 2
      curtemplim[2] = frame.data.bytes[0] * 0.234375;
      newframe = newframe | 1;
      break;

    case 0x23B:  //phase 3 temp message 1
      templeg[0][2] = frame.data.bytes[0] - 40;
      templeg[1][2] = frame.data.bytes[1] - 40;
      inlettarg[2] = frame.data.bytes[5] - 40;
      newframe = newframe | 1;
      break;

    case 0x239:  //phase 2 temp message 1
      templeg[0][1] = frame.data.bytes[0] - 40;
      templeg[1][1] = frame.data.bytes[1] - 40;
      inlettarg[1] = frame.data.bytes[5] - 40;
      newframe = newframe | 1;
      break;
    case 0x249:  //phase 2 temp message 2
      curtemplim[1] = frame.data.bytes[0] * 0.234375;
      newframe = newframe | 1;
      break;

    case 0x237:  //phase 1 temp message 1
      templeg[0][0] = frame.data.bytes[0] - 40;
      templeg[1][0] = frame.data.bytes[1] - 40;
      inlettarg[0] = frame.data.bytes[5] - 40;
      newframe = newframe | 1;
      break;
    case 0x247:  //phase 2 temp message 2
      curtemplim[0] = frame.data.bytes[0] * 0.234375;
      newframe = newframe | 1;
      break;

    case 0x207:  //phase 2 msg 0x209. phase 3 msg 0x20B
      acvolt[0] = frame.data.bytes[1];
      accur[0] = uint16_t((frame.data.bytes[6] & 0x03) * 256 + frame.data.bytes[5]) >> 1;
      x = frame.data.bytes[1];  // & 12;
      if (x > 0x46)             //say 0x46 = 70V
      {
        ACpres[0] = true;
      } else {
        ACpres[0] = false;
      }
      x = frame.data.bytes[2] & 0x02;  //was 40
      if (x != 0) {
        ModEn[0] = true;
      } else {
        ModEn[0] = false;
      }
      x = frame.data.bytes[2] & 0x04;  //was 20
      if (x != 0) {
        ModFlt[0] = true;
      } else {
        ModFlt[0] = false;
      }
      newframe = newframe | 1;
      break;
    case 0x209:  //phase 2 msg 0x209. phase 3 msg 0x20B
      acvolt[1] = frame.data.bytes[1];
      accur[1] = uint16_t((frame.data.bytes[6] & 0x03) * 256 + frame.data.bytes[5]) >> 1;
      x = frame.data.bytes[1];  // & 12;
      if (x > 0x46)             //say 0x46 = 70V)
      {
        ACpres[1] = true;
      } else {
        ACpres[1] = false;
      }
      x = frame.data.bytes[2] & 0x02;  //was 40
      if (x != 0) {
        ModEn[1] = true;
      } else {
        ModEn[1] = false;
      }
      x = frame.data.bytes[2] & 0x04;  //was 20
      if (x != 0) {
        ModFlt[1] = true;
      } else {
        ModFlt[1] = false;
      }
      newframe = newframe | 1;
      break;
    case 0x20B:  //phase 2 msg 0x209. phase 3 msg 0x20B
      acvolt[2] = frame.data.bytes[1];
      accur[2] = uint16_t((frame.data.bytes[6] & 0x03) * 256 + frame.data.bytes[5]) >> 1;
      x = frame.data.bytes[1];  // & 12;
      if (x > 0x46)             //say 0x46 = 70V)
      {
        ACpres[2] = true;
      } else {
        ACpres[2] = false;
      }
      x = frame.data.bytes[2] & 0x02;  //was 40
      if (x != 0) {
        ModEn[2] = true;
      } else {
        ModEn[2] = false;
      }
      x = frame.data.bytes[2] & 0x04;  //was 20
      if (x != 0) {
        ModFlt[2] = true;
      } else {
        ModFlt[2] = false;
      }
      newframe = newframe | 1;
      break;
    case 0x227:  //dc feedback. Phase 1 measured DC battery current and voltage Charger phase 2 msg : 0x229. Charger phase 3 mesg : 0x22B
      dccur[0] = ((frame.data.bytes[5] << 8) + frame.data.bytes[4]) * 0.000839233;
      dcvolt[0] = ((frame.data.bytes[3] << 8) + frame.data.bytes[2]) * 0.0105286;  //we left shift 8 bits to make a 16bit uint.
      newframe = newframe | 2;
      break;
    case 0x229:                                                                     //dc feedback. Phase 2 measured DC battery current and voltage Charger phase 2 msg : 0x229. Charger phase 3 mesg : 0x22B
      dccur[1] = ((frame.data.bytes[5] << 8) + frame.data.bytes[4]) * 0.000839233;  //* 0.000839233 convert in rest of code
      dcvolt[1] = ((frame.data.bytes[3] << 8) + frame.data.bytes[2]) * 0.0105286;   //we left shift 8 bits to make a 16bit uint.
      newframe = newframe | 2;
      break;
    case 0x22B:                                                                     //dc feedback. Phase 3 measured DC battery current and voltage Charger phase 2 msg : 0x229. Charger phase 3 mesg : 0x22B
      dccur[2] = ((frame.data.bytes[5] << 8) + frame.data.bytes[4]) * 0.000839233;  //* 0.000839233 convert in rest of code
      dcvolt[2] = ((frame.data.bytes[3] << 8) + frame.data.bytes[2]) * 0.01052856;  //we left shift 8 bits to make a 16bit uint.
      newframe = newframe | 2;
      break;

    default:
      // if nothing else matches, do the default
      break;
  }
}
void Charger_msgs() {
  CAN_FRAME outframe;  //A structured variable according to due_can library for transmitting CAN data.
  /////////////////////This msg addresses all modules/////////////////////////////////////////////////
  outframe.id = 0x045c;                                   // Set our transmission address ID
  outframe.length = 8;                                    // Data payload 8 bytes
  outframe.extended = 0;                                  // Extended addresses - 0=11-bit 1=29bit
  outframe.rtr = 0;                                       //No request
  outframe.data.bytes[0] = lowByte(parameters.voltSet);   //Voltage setpoint
  outframe.data.bytes[1] = highByte(parameters.voltSet);  //Voltage setpoint
  outframe.data.bytes[2] = 0x14;
  if (bChargerEnabled) {
    outframe.data.bytes[3] = 0x2e;
  } else outframe.data.bytes[3] = 0x0e;
  outframe.data.bytes[4] = 0x00;
  outframe.data.bytes[5] = 0x00;
  outframe.data.bytes[6] = 0x90;
  outframe.data.bytes[7] = 0x8c;
  Can0.sendFrame(outframe);
  //////////////////////////////////////////////////////////////////////////////////////////////////////

  //////////////////////////////////////Phase 1 command message////////////////////////////////////////
  outframe.id = 0x042c;   // Set our transmission address ID
  outframe.length = 8;    // Data payload 8 bytes
  outframe.extended = 0;  // Extended addresses - 0=11-bit 1=29bit
  outframe.rtr = 0;       //No request
  outframe.data.bytes[0] = 0x42;
  outframe.data.bytes[2] = lowByte(modulelimcur);   //AC Current setpoint
  outframe.data.bytes[3] = highByte(modulelimcur);  //AC Current setpoint
  if (bChargerEnabled) {
    outframe.data.bytes[1] = 0xBB;
    outframe.data.bytes[4] = 0xFE;
  } else {
    outframe.data.bytes[1] = lowByte(uint16_t(ACvoltIN / 1.2));
    outframe.data.bytes[4] = 0x64;
  }
  outframe.data.bytes[5] = 0x00;
  outframe.data.bytes[6] = 0x00;
  outframe.data.bytes[7] = 0x00;
  Can0.sendFrame(outframe);
  //////////////////////////////Phase 2 command message//////////////////////////////////////////////
  outframe.id = 0x43c;  //phase 2 and 3 are copies of phase 1 so no need to set them up again
  Can0.sendFrame(outframe);
  ///////////////////////////////Phase 3 command message/////////////////////////////////////////////
  outframe.id = 0x44c;
  Can0.sendFrame(outframe);

  ///////////Static Frame every 100ms///////////////////////////////////////////////////////////////////
  outframe.id = 0x368;    // Set our transmission address ID
  outframe.length = 8;    // Data payload 8 bytes
  outframe.extended = 0;  // Extended addresses - 0=11-bit 1=29bit
  outframe.rtr = 0;       //No request
  outframe.data.bytes[0] = 0x03;
  outframe.data.bytes[1] = 0x49;
  outframe.data.bytes[2] = 0x29;
  outframe.data.bytes[3] = 0x11;
  outframe.data.bytes[4] = 0x00;
  outframe.data.bytes[5] = 0x0c;
  outframe.data.bytes[6] = 0x40;
  outframe.data.bytes[7] = 0xff;
  Can0.sendFrame(outframe);
}

void Ext_msgs() {
  /*////////////////////////////////////////////////////////////////////////////////////////////////////////
    External CAN
    ////////////////////////////////////////////////////////////////////////////////////////////////////////*/
  CAN_FRAME outframe;  //A structured variable according to due_can library for transmitting CAN data.

  uint16_t y, z = 0;
  outframe.id = StatusID;
  if (parameters.canControl == 3) {
    outframe.id = StatusID + 1;
  }
  outframe.length = 8;    // Data payload 8 bytes
  outframe.extended = 0;  // Extended addresses - 0=11-bit 1=29bit
  outframe.rtr = 0;       //No request
  outframe.data.bytes[0] = 0x00;
  for (int x = 0; x < 3; x++) {
    y = y + dcvolt[x];
  }
  outframe.data.bytes[0] = y / 3;

  if (parameters.phaseconfig == Singlephase) {
    for (int x = 0; x < 3; x++) {
      z = z + (accur[x] * 66.66);
    }
  } else {
    z = accur[2] * 66.66;
  }

  outframe.data.bytes[1] = lowByte(z);
  outframe.data.bytes[2] = highByte(z);

  outframe.data.bytes[3] = lowByte(uint16_t(totdccur));   //0.005Amp
  outframe.data.bytes[4] = highByte(uint16_t(totdccur));  //0.005Amp
  outframe.data.bytes[5] = lowByte(uint16_t(modulelimcur * 0.66666));
  outframe.data.bytes[6] = highByte(uint16_t(modulelimcur * 0.66666));
  outframe.data.bytes[7] = 0x00;
  outframe.data.bytes[7] = Proximity << 6;
  outframe.data.bytes[7] = outframe.data.bytes[7] || (parameters.type << 4);
  Can1.sendFrame(outframe);

  /////////Elcon Message////////////

  outframe.id = ElconID;
  if (parameters.canControl == 3) {
    outframe.id = ElconID + 1;
  }
  outframe.id = ElconID;
  outframe.length = 8;    // Data payload 8 bytes
  outframe.extended = 1;  // Extended addresses - 0=11-bit 1=29bit
  outframe.rtr = 0;       //No request
  outframe.data.bytes[0] = highByte(y * 10 / 3);
  outframe.data.bytes[1] = lowByte(y * 10 / 3);
  outframe.data.bytes[2] = highByte(uint16_t(totdccur * 5));  //0.005Amp conv to 0.1
  outframe.data.bytes[3] = lowByte(uint16_t(totdccur * 5));   //0.005Amp conv to 0.1
  outframe.data.bytes[4] = 0x00;
  if (state != 0) {
    outframe.data.bytes[4] = 0x08;
  }
  outframe.data.bytes[5] = 0x00;
  outframe.data.bytes[6] = 0x00;
  outframe.data.bytes[7] = 0x00;
  Can1.sendFrame(outframe);

  ///DCDC CAN//////////////////////////////////////////////////////////////////////
  if (dcdcenable) {
    outframe.id = 0x3D8;
    outframe.length = 3;    // Data payload 8 bytes
    outframe.extended = 0;  // Extended addresses - 0=11-bit 1=29bit
    outframe.rtr = 0;       //No request

    outframe.data.bytes[0] = highByte(uint16_t((parameters.dcdcsetpoint - 9000) / 68.359375) << 6);
    outframe.data.bytes[1] = lowByte(uint16_t((parameters.dcdcsetpoint - 9000) / 68.359375) << 6);

    outframe.data.bytes[1] = outframe.data.bytes[1] | 0x20;
    outframe.data.bytes[2] = 0x00;
    Can1.sendFrame(outframe);
  }

  ////////////////////////////////////////////////////////////////////

  if (parameters.canControl == 1) {
    outframe.id = ControlID;
    outframe.length = 8;    // Data payload 8 bytes
    outframe.extended = 0;  // Extended addresses - 0=11-bit 1=29bit
    outframe.rtr = 0;       //No request

    outframe.data.bytes[0] = 0;

    if (state != 0) {
      if (slavechargerenable == 1) {
        outframe.data.bytes[0] = 0x01;
      }
    }

    outframe.data.bytes[1] = highByte(parameters.voltSet);
    outframe.data.bytes[2] = lowByte(parameters.voltSet);
    outframe.data.bytes[3] = highByte(maxdccur);
    outframe.data.bytes[4] = lowByte(maxdccur);
    outframe.data.bytes[5] = highByte(modulelimcur);
    outframe.data.bytes[6] = lowByte(modulelimcur);
    outframe.data.bytes[7] = 0;

    Can1.sendFrame(outframe);
  }

  if (state != 0) {
    Charger_msgs();
  }
}

void evseread() {
  uint16_t val = 0;
  val = analogRead(EVSE_PROX);  // read the input pin
  if (parameters.type == 2) {
    if (val > 950) {
      Proximity = Unconnected;
      cablelim = 0;
    } else {
      Proximity = Connected;
      if (val < 950 && val > 800) {
        cablelim = 13000;
      }
      if (val < 800 && val > 700) {
        cablelim = 20000;
      }
      if (val < 600 && val > 450) {
        cablelim = 32000;
      }
      if (val < 400 && val > 250) {
        cablelim = 63000;
      }
    }
  }

  if (parameters.type == 1) {
    if (val > 800) {
      Proximity = Unconnected;
    } else {
      if (val > 550) {
        Proximity = Buttonpress;
      } else {
        Proximity = Connected;
      }
    }
  }
}

void Pilotread() {
  Pilotcalc();
}

void Pilotcalc() {
  if (digitalRead(EVSE_PILOT) == HIGH) {
    duration = micros() - pilottimer;
    pilottimer = micros();
  } else {
    accurlim = (micros() - pilottimer) * 100 / duration * 600;  //Calculate the duty cycle then multiply by 600 to get mA current limit
  }
}

void ACcurrentlimit() {
  if (parameters.autoEnableCharger == 1) {
    if (micros() - pilottimer > 1200)  //too big a gap in pilot signal kills means signal error or disconnected so no current allowed.
    {
      accurlim = 0;
    }

    if (parameters.phaseconfig == 0) {
      modulelimcur = (accurlim / 3) * 1.5;  // all module parallel, sharing AC input current
    } else {
      modulelimcur = accurlim * 1.5;  // one module per phase, EVSE current limit is per phase
    }
  } else {
    if (parameters.phaseconfig == 0) {
      modulelimcur = (parameters.currReq / 3);  // all module parallel, sharing AC input current
    } else {
      modulelimcur = parameters.currReq;
    }
  }
  if (parameters.canControl == 1 | parameters.canControl == 2) {
    if (accurlim * 1.5 > (16000 * 1.5))  //enable second charger if current available >15A
    {
      modulelimcur = modulelimcur * 0.5;
      slavechargerenable = 1;
    } else {
      slavechargerenable = 0;
    }
  } else if (parameters.canControl == 4) {
    if (parameters.phaseconfig == 0) {
      modulelimcur = (accurlim / 3) * 1.5;  // all module parallel, sharing AC input current
    } else {
      modulelimcur = accurlim * 1.5;  // one module per phase, EVSE current limit is per phase
    }

    if (maxdccur == 0) {
      modulelimcur = 0;
    }
  }

  /*
  if (parameters.phaseconfig == 1) {
    if (modulelimcur > (dcaclim * 1.5))  //if more current then max per module or limited by DC output current
    {
      modulelimcur = (dcaclim * 1.5);
    }
    if (modulelimcur > parameters.currReq)  //if evse allows more current then set in parameters limit it
    {
      modulelimcur = parameters.currReq;
    }
  }
  if (parameters.phaseconfig == 0) {
    if (modulelimcur * activemodules > (dcaclim * 1.5))  //if more current then max per module or limited by DC output current
    {
      modulelimcur = (dcaclim * 1.5 / activemodules);
    }
    if (modulelimcur > (parameters.currReq / activemodules))  //if evse allows more current then set in parameters limit it
    {
      modulelimcur = (parameters.currReq / activemodules);
    }
  }
*/
  if (parameters.phaseconfig != 0 && parameters.phaseconfig != 1) {
    modulelimcur = 0;
  }
}

void DCcurrentlimit() {
  totdccur = 0;
  for (int x = 0; x < 3; x++) {
    totdccur = totdccur + (dccur[x] * 0.1678466);
  }
  Input = totdccur;
  Setpoint = maxdccur;
  myPID.Compute();

  dcaclim = Output;
}

void DClimCalc() {
  DCLim = dcvolt[L1] * maxdccur;
  dcaclim = float(DCLim / acvolt[L1]);
}

void canextdecode(CAN_FRAME &frame) {
  int x = 0;
  if (parameters.canControl == 2) {
    if (ElconControlID == frame.id)  //Charge Control message
    {
      parameters.voltSet = ((frame.data.bytes[0] << 8) + frame.data.bytes[1]) * 0.1;
      maxdccur = (frame.data.bytes[2] << 8) + frame.data.bytes[3];

      if (frame.data.bytes[4] & 0x01 == 0) {
        if (state == 0) {
          state = 2;
          tboot = millis();
        }
      } else {
        state = 0;
      }
      if (debug == 1) {
        if (candebug == 1) {
          Serial.println();
          Serial.print(state);
          Serial.print(" ");
          Serial.print(parameters.voltSet);
          Serial.print(" ");
          Serial.print(maxdccur);
          Serial.print(" A ");
          Serial.println();
        }
        tcan = millis();
      }
    }
  }

  if (parameters.canControl == 3) {
    if (ControlID == frame.id)  //Charge Control message
    {
      if (frame.data.bytes[0] & 0x01 == 1) {
        if (state == 0) {
          state = 2;
          tboot = millis();
        }
      } else {
        if (millis() - slavetimeout > 1000) {
          slavetimeout = millis();
        }
        if (millis() - slavetimeout > 500) {
          state = 0;
        }
      }
      parameters.voltSet = (frame.data.bytes[1] << 8) + frame.data.bytes[2];
      maxdccur = ((frame.data.bytes[3] << 8) + frame.data.bytes[4]);
      modulelimcur = (frame.data.bytes[5] << 8) + frame.data.bytes[6];
      if (debug == 1) {
        if (candebug == 1) {
          Serial.println();
          Serial.print(state);
          Serial.print(" ");
          Serial.print(parameters.voltSet);
          Serial.print(" ");
          Serial.print(modulelimcur);
          Serial.println();
        }
      }
      tcan = millis();
    }
  }

  if (parameters.canControl == 4) {  //TH Special
    if (frame.id == 0x357)           //Charge Control message
    {
      accurlim = ((frame.data.bytes[2] << 8) + frame.data.bytes[1]) * 1000;  //convert to mA
      tcan = millis();

      if (frame.data.bytes[3] & 0x03 > 0) {
        if (state == 0) {
          state = 2;
          tboot = millis();
        }
      } else {
        state = 0;
      }
    }
    if (frame.id == 0x351)  //BMS Pack Limit message
    {
      parameters.voltSet = ((frame.data.bytes[1] << 8) + frame.data.bytes[0]) * 10;  //convert to 0.01 V
      maxdccur = ((frame.data.bytes[3] << 8) + frame.data.bytes[2]) * 0.1;           //convert to 1 A
      tcan = millis();
    }

    if (frame.id == 0x3A1)  //BMS Pack Limit message
    {
      parameters.voltSet = ((frame.data.bytes[1] << 8) + frame.data.bytes[0]) * 10;  //convert to 0.01 V
      maxdccur = ((frame.data.bytes[3] << 8) + frame.data.bytes[2]) * 0.1;           //convert to 1 A
      tcan = millis();
    }
  }
}

void menu() {
  incomingByte = Serial.read();  // read the incoming byte:
  if (menuload == 1) {
    switch (incomingByte) {
      case 'q':  //q for quit
        debug = 1;
        menuload = 0;
        break;

      case 'a':  //a for auto enable
        candebug++;
        if (candebug > 1) {
          candebug = 0;
        }
        menuload = 0;
        incomingByte = 'd';
        break;

      case 'b':  //a for auto enable
        evsedebug++;
        if (evsedebug > 1) {
          evsedebug = 0;
        }
        menuload = 0;
        incomingByte = 'd';
        break;


      case '1':  //a for auto enable
        parameters.autoEnableCharger++;
        if (parameters.autoEnableCharger > 1) {
          parameters.autoEnableCharger = 0;
        }
        menuload = 0;
        incomingByte = 'd';
        break;

      case '2':  //e for enabling chargers followed by numbers to indicate which ones to run
        if (Serial.available() > 0) {
          parameters.enabledChargers = Serial.parseInt();
          menuload = 0;
          incomingByte = 'd';
        }
        break;

      case '3':  //a for can control enable
        if (Serial.available() > 0) {
          parameters.canControl = Serial.parseInt();
          if (parameters.canControl > 4) {
            parameters.canControl = 0;
          }
          menuload = 0;
          incomingByte = 'd';
        }
        break;

      case '4':  //t for type
        if (Serial.available() > 0) {
          parameters.type = Serial.parseInt();
          if (parameters.type > 2) {
            parameters.type = 2;
          }
          if (parameters.type == 0) {
            parameters.type = 2;
          }
          menuload = 0;
          incomingByte = 'd';
        }
        break;

      case '5':  //a for can control enable
        if (Serial.available() > 0) {
          parameters.phaseconfig = Serial.parseInt() - 1;
          if (parameters.phaseconfig == 2) {
            parameters.phaseconfig = 1;
          }
          if (parameters.phaseconfig == 0) {
            parameters.phaseconfig = 0;
          }
          menuload = 0;
          incomingByte = 'd';
        }
        break;
      case '6':  //v for voltage setting in whole numbers
        if (Serial.available() > 0) {
          parameters.voltSet = (Serial.parseInt() * 100);
          menuload = 0;
          incomingByte = 'd';
        }
        break;

      case '7':  //c for current setting in whole numbers
        if (Serial.available() > 0) {
          parameters.currReq = (Serial.parseInt() * 1500);
          menuload = 0;
          incomingByte = 'd';
        }
        break;

      case '8':  //c for current setting in whole numbers
        if (Serial.available() > 0) {
          parameters.can0Speed = long(Serial.parseInt() * 1000);
          Can1.begin(parameters.can0Speed);
          menuload = 0;
          incomingByte = 'd';
        }
        break;

      case '9':  //c for current setting in whole numbers
        if (Serial.available() > 0) {
          parameters.can1Speed = long(Serial.parseInt() * 1000);

          Can1.begin(parameters.can1Speed);
          menuload = 0;
          incomingByte = 'd';
        }
        break;
      case 'c':  //c for time
        if (Serial.available() > 0) {
          rtc_clock.set_time(Serial.parseInt(), Serial.parseInt(), Serial.parseInt());
          menuload = 0;
          incomingByte = 'd';
        }
        break;
      case 'd':  //c for time
        if (Serial.available() > 0) {
          rtc_clock.set_date(Serial.parseInt(), Serial.parseInt(), Serial.parseInt());
          menuload = 0;
          incomingByte = 'd';
        }
        break;
      case 's':  //t for termaintion voltage setting in whole numbers
        if (Serial.available() > 0) {
          parameters.sleeptimeout = (Serial.parseInt() * 1000);
          menuload = 0;
          incomingByte = 'd';
        }
        break;
      case 't':  //t for termaintion voltage setting in whole numbers
        if (Serial.available() > 0) {
          parameters.tVolt = (Serial.parseInt() * 100);
          menuload = 0;
          incomingByte = 'd';
        }
        break;
    }
  }

  if (menuload == 0) {
    switch (incomingByte) {
      case 's':  //s for start AND stop
        if (Serial.available() > 0) {
          setting = 1;
          digitalWrite(LED_BUILTIN, HIGH);
          if (Serial.parseInt() == 1) {
            if (state == 0) {
              state = 2;  // initialize modules
              tboot = millis();
            }
          }
          if (Serial.parseInt() == 0) {
            if (state == 1) {
              state = 0;  // initialize modules
            }
          }
        }

      case 'q':  //q for quit
        EEPROM.write(0, parameters);
        debug = 1;
        menuload = 0;
        sleeptime = millis();
        break;

      case 'd':  //d for display
        debug = 0;
        menuload = 1;
        Serial.println();
        Serial.println();
        Serial.println();
        Serial.println("Zero-EV Tesla Charger");
        Serial.println();
        Serial.print("Firmware : ");
        Serial.print(firmware);
        Serial.println();
        Serial.println("Settings Menu");
        Serial.print("1 - Auto Enable : ");
        if (parameters.autoEnableCharger == 1) {
          Serial.println("ON");
        } else {
          Serial.println("OFF");
        }
        Serial.print("2 - Modules Enabled : ");
        Serial.println(parameters.enabledChargers);
        Serial.print("3 - Can Mode : ");
        if (parameters.canControl == 0) {
          Serial.println(" Off ");
        }
        if (parameters.canControl == 1) {
          Serial.println(" Master ");
        }
        if (parameters.canControl == 2) {
          Serial.println(" Master Elcon ");
        }
        if (parameters.canControl == 3) {
          Serial.println(" Slave ");
        }
        if (parameters.canControl == 4) {
          Serial.println(" TH Special ");
        }
        Serial.print("4 - Port Type : ");
        Serial.println(parameters.type);
        Serial.print("5 - Phase Wiring : ");
        Serial.println(parameters.phaseconfig + 1);
        Serial.print("6 - DC Charge Voltage : ");
        Serial.print(parameters.voltSet * 0.01, 0);
        Serial.println("V");
        Serial.print("7 - AC Current Limit : ");
        Serial.print(parameters.currReq / 1500);
        Serial.println("A");
        Serial.print("8 - CAN0 Speed : ");
        Serial.println(parameters.can0Speed * 0.001, 0);
        Serial.print("9 - CAN1 Speed : ");
        Serial.println(parameters.can1Speed * 0.001, 0);
        Serial.print("a - Can Debug : ");
        if (candebug == 1) {
          Serial.println("ON");
        } else {
          Serial.println("OFF");
        }
        Serial.print("b - EVSE Debug : ");
        if (evsedebug == 1) {
          Serial.println("ON");
        } else {
          Serial.println("OFF");
        }
        /*
                Serial.print("c - time : ");
                Serial.print(rtc_clock.get_hours());
                Serial.print(":");
                Serial.print(rtc_clock.get_minutes());
                Serial.print(":");
                Serial.println(rtc_clock.get_seconds());
                Serial.print("d - date : ");
                Serial.print(daynames[rtc_clock.get_day_of_week() - 1]);
                Serial.print(": ");
                Serial.print(rtc_clock.get_days());
                Serial.print(".");
                Serial.print(rtc_clock.get_months());
                Serial.print(".");
                Serial.println(rtc_clock.get_years());
        */
        Serial.print("s - Sleep Time Out : ");
        Serial.print(parameters.sleeptimeout * 0.001, 0);
        Serial.println(" S");
        Serial.print("t - termination voltage : ");
        Serial.print(parameters.tVolt * 0.01, 0);
        Serial.println("V");
        Serial.println("q - To Quit Menu");
        break;
    }
  }
}
