/*
  Copyright (c) 2019 Simp ECO Engineering
  Permission is hereby granted, free of charge, to any person obtaining
  a copy of this software and associated documentation files (the
  "Software"), to deal in the Software without restriction, including
  without limitation the rights to use, copy, modify, merge, publish,
  distribute, sublicense, and/or sell copies of the Software, and to
  permit persons to whom the Software is furnished to do so, subject to
  the following conditions:
  The above copyright notice and this permission notice shall be included
  in all copies or substantial portions of the Software.

  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
  EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
  MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
  IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY
  CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT,
  TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE
  SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

#include "BMSModuleManager.h"
#include <Arduino.h>
#include "config.h"
#include "SerialConsole.h"
#include "Logger.h"
#include <ADC.h> //https://github.com/pedvide/ADC
#include <EEPROM.h>
#include <FlexCAN.h> //https://github.com/collin80/FlexCAN_Library
#include <SPI.h>
#include <Filters.h>                    //https://github.com/JonHub/Filters
#include "Serial_CAN_Module_TeensyS3.h" //https://github.com/tomdebree/Serial_CAN_Teensy

#define RESTART_ADDR 0xE000ED0C
#define READ_RESTART() (*(volatile uint32_t *)RESTART_ADDR)
#define WRITE_RESTART(val) ((*(volatile uint32_t *)RESTART_ADDR) = (val))
#define CPU_REBOOT WRITE_RESTART(0x5FA0004)

Serial_CAN can;
BMSModuleManager bms;
SerialConsole console;
EEPROMSettings settings;

/////Version Identifier/////////
int firmver = 21010501;

//Curent filter//
float filterFrequency = 5.0;
FilterOnePole lowpassFilter(LOWPASS, filterFrequency);

//Simple BMS V2 wiring//
const int ACUR2 = A0; // current 1
const int ACUR1 = A1; // current 2
const int IN1 = 17;   // input 1 - high active
const int IN2 = 16;   // input 2- high active
const int IN3 = 18;   // input 1 - high active
const int IN4 = 19;   // input 2- high active
const int OUT1 = 11;  // output 1 - high active
const int OUT2 = 12;  // output 1 - high active
const int OUT3 = 20;  // output 1 - high active
const int OUT4 = 21;  // output 1 - high active
const int OUT5 = 22;  // output 1 - high active
const int OUT6 = 23;  // output 1 - high active
const int OUT7 = 5;   // output 1 - high active
const int OUT8 = 6;   // output 1 - high active
const int led = 13;
const int BMBfault = 11;

byte bmsstatus = 0;
//bms status values
#define Boot 0
#define Ready 1
#define Drive 2
#define Charge 3
#define Precharge 4
#define Error 5
//

//Current sensor values
#define Undefined 0
#define Analoguedual 1
#define Canbus 2
#define Analoguesing 3
//
//Charger Types
#define Relay Charger 0
#define BrusaNLG5 1
#define ChevyVolt 2
#define Eltek 3
#define Elcon 4
#define Victron 5
#define Coda 6
//

int Discharge;
int ErrorReason = 0;

//variables for output control
int pulltime = 1000;
int contctrl, contstat = 0; //1 = out 5 high 2 = out 6 high 3 = both high
unsigned long conttimer1, conttimer2, conttimer3, Pretimer, Pretimer1 = 0;
uint16_t pwmfreq = 10000; //pwm frequency

int pwmcurmax = 50;    //Max current to be shown with pwm
int pwmcurmid = 50;    //Mid point for pwm dutycycle based on current
int16_t pwmcurmin = 0; //DONOT fill in, calculated later based on other values

//variables for VE driect bus comms
char *myStrings[] = {"V", "14674", "I", "0", "CE", "-1", "SOC", "800", "TTG", "-1", "Alarm", "OFF", "Relay", "OFF", "AR", "0", "BMV", "600S", "FW", "212", "H1", "-3", "H2", "-3", "H3", "0", "H4", "0", "H5", "0", "H6", "-7", "H7", "13180", "H8", "14774", "H9", "137", "H10", "0", "H11", "0", "H12", "0"};

//variables for VE can
uint16_t chargevoltage = 49100; //max charge voltage in mv
int chargecurrent;
uint16_t disvoltage = 42000; // max discharge voltage in mv
int discurrent;

uint16_t SOH = 100; // SOH place holder

unsigned char alarm[4], warning[4] = {0, 0, 0, 0};
unsigned char mes[8] = {0, 0, 0, 0, 0, 0, 0, 0};
unsigned char bmsname[8] = {'S', 'I', 'M', 'P', ' ', 'B', 'M', 'S'};
unsigned char bmsmanu[8] = {'T', 'O', 'M', ' ', 'D', 'E', ' ', 'B'};
long unsigned int rxId;
unsigned char len = 0;
byte rxBuf[8];
char msgString[128]; // Array to store serial string
uint32_t inbox;
signed long CANmilliamps;

int Charged = 0;

//struct can_frame canMsg;
//MCP2515 CAN1(10); //set CS pin for can controlelr

//variables for current calulation
int value;
float currentact, RawCur;
float ampsecond;
unsigned long lasttime;
unsigned long loopTimeMain, looptime, UnderTime, looptime1, cleartime, loopTimeBalance = 0; //ms
int currentsense = 14;
int sensor = 1;

//running average
const int RunningAverageCount = 16;
float RunningAverageBuffer[RunningAverageCount];
int NextRunningAverage;

//Variables for SOC calc
int SOC = 100; //State of Charge
int SOCset = 0;
int SOCtest = 0;

///charger variables
int maxac1 = 16;          //Shore power 16A per charger
int maxac2 = 10;          //Generator Charging
int chargerid1 = 0x618;   //bulk chargers
int chargerid2 = 0x638;   //finishing charger
float chargerendbulk = 0; //V before Charge Voltage to turn off the bulk charger/s
float chargerend = 0;     //V before Charge Voltage to turn off the finishing charger/s
int chargertoggle = 0;
int ncharger = 1; // number of chargers
int maxchargepercent = 100;

//serial canbus expansion
unsigned long id = 0;
unsigned char dta[8];
int mescycl = 0;

//variables
int outputstate = 0;
int incomingByte = 0;
int storagemode = 0;
int x = 0;
bool balancecells = false;
int cellspresent = 0;

//Display Control
bool displaySleep = false;

//Debugging modes//////////////////
int debug = 1;
int inputcheck = 0;     //read digital inputs
int outputcheck = 0;    //check outputs
int candebug = 0;       //view can frames
int CanDebugSerial = 0; //view can frames
int gaugedebug = 0;
int debugCur = 0;
int CSVdebug = 0;
int menuload = 0;
int debugdigits = 3; //amount of digits behind decimal for voltage reading

ADC *adc = new ADC(); // adc object
void loadSettings()
{
  Logger::console(0, "Resetting to factory defaults");
  settings.version = EEPROM_VERSION;
  settings.checksum = 2;
  settings.canSpeed = 125000;
  settings.batteryID = 0x01; //in the future should be 0xFF to force it to ask for an address
  settings.OverVSetpoint = 4.2f;
  settings.UnderVSetpoint = 3.0f;
  settings.ChargeVsetpoint = 4.1f;
  settings.ChargeHys = 0.2f; // voltage drop required for charger to kick back on
  settings.WarnOff = 0.1f;   //voltage offset to raise a warning
  settings.DischVsetpoint = 3.2f;
  settings.CellGap = 0.2f; //max delta between high and low cell
  settings.OverTSetpoint = 65.0f;
  settings.UnderTSetpoint = -10.0f;
  settings.ChargeTSetpoint = 0.0f;
  settings.DisTSetpoint = 40.0f;
  settings.WarnToff = 5.0f;  //temp offset before raising warning
  settings.IgnoreTemp = 0;   // 0 - use both sensors, 1 or 2 only use that sensor
  settings.IgnoreVolt = 0.5; //
  settings.balanceVoltage = 3.9f;
  settings.balanceHyst = 0.04f;
  settings.logLevel = 2;
  settings.CAP = 100;              //battery size in Ah
  settings.Pstrings = 1;           // strings in parallel used to divide voltage of pack
  settings.Scells = 12;            //Cells in series
  settings.StoreVsetpoint = 3.8;   // V storage mode charge max
  settings.discurrentmax = 300;    // max discharge current in 0.1A
  settings.DisTaper = 0.3f;        //V offset to bring in discharge taper to Zero Amps at settings.DischVsetpoint
  settings.chargecurrentmax = 300; //max charge current in 0.1A
  settings.chargecurrentend = 50;  //end charge current in 0.1A
  settings.socvolt[0] = 3100;      //Voltage and SOC curve for voltage based SOC calc
  settings.socvolt[1] = 10;        //Voltage and SOC curve for voltage based SOC calc
  settings.socvolt[2] = 4100;      //Voltage and SOC curve for voltage based SOC calc
  settings.socvolt[3] = 90;        //Voltage and SOC curve for voltage based SOC calc
  settings.invertcur = 0;          //Invert current sensor direction
  settings.cursens = 2;
  settings.voltsoc = 0;          //SOC purely voltage based
  settings.Pretime = 5000;       //ms of precharge time
  settings.conthold = 50;        //holding duty cycle for contactor 0-255
  settings.Precurrent = 1000;    //ma before closing main contator
  settings.convhigh = 58;        // mV/A current sensor high range channel
  settings.convlow = 643;        // mV/A current sensor low range channel
  settings.changecur = 20000;    //mA change overpoint
  settings.offset1 = 1750;       //mV mid point of channel 1
  settings.offset2 = 1750;       //mV mid point of channel 2
  settings.gaugelow = 50;        //empty fuel gauge pwm
  settings.gaugehigh = 255;      //full fuel gauge pwm
  settings.ESSmode = 0;          //activate ESS mode
  settings.ncur = 1;             //number of multiples to use for current measurement
  settings.chargertype = 2;      // 1 - Brusa NLG5xx 2 - Volt charger 0 -No Charger
  settings.chargerspd = 100;     //ms per message
  settings.UnderDur = 5000;      //ms of allowed undervoltage before throwing open stopping discharge.
  settings.CurDead = 5;          // mV of dead band on current sensor
  settings.ChargerDirect = 1;    //1 - charger is always connected to HV battery // 0 - Charger is behind the contactors
  settings.disp = 1;             // 1 - display is used 0 - mirror serial data onto serial bus
  settings.SerialCan = 1;        // 1- serial can adapter used 0- Not used
  settings.SerialCanSpeed = 500; //serial can adapter speed
  settings.DCDCreq = 140;        //requested DCDC voltage output in 0.1V
  settings.SerialCanBaud = 19200;
}

CAN_message_t msg;
CAN_message_t inMsg;
CAN_filter_t filter;

uint32_t lastUpdate;

void setup()
{
  delay(4000); //just for easy debugging. It takes a few seconds for USB to come up properly on most OS's
  //pinMode(ACUR1, INPUT);//Not required for Analogue Pins
  //pinMode(ACUR2, INPUT);
  pinMode(IN1, INPUT);
  pinMode(IN2, INPUT);
  pinMode(IN3, INPUT);
  pinMode(IN4, INPUT);
  pinMode(OUT1, OUTPUT); // drive contactor
  pinMode(OUT2, OUTPUT); // precharge
  pinMode(OUT3, OUTPUT); // charge relay
  pinMode(OUT4, OUTPUT); // Negative contactor
  pinMode(OUT5, OUTPUT); // pwm driver output
  pinMode(OUT6, OUTPUT); // pwm driver output
  pinMode(OUT7, OUTPUT); // pwm driver output
  pinMode(OUT8, OUTPUT); // pwm driver output
  pinMode(led, OUTPUT);

  analogWriteFrequency(OUT5, pwmfreq);
  analogWriteFrequency(OUT6, pwmfreq);
  analogWriteFrequency(OUT7, pwmfreq);
  analogWriteFrequency(OUT8, pwmfreq);

  Can0.begin(125000);

  //set filters for standard
  for (int i = 0; i < 8; i++)
  {
    Can0.getFilter(filter, i);
    filter.flags.extended = 0;
    Can0.setFilter(filter, i);
  }
  //set filters for extended
  for (int i = 9; i < 13; i++)
  {
    Can0.getFilter(filter, i);
    filter.flags.extended = 1;
    Can0.setFilter(filter, i);
  }

  //if using enable pins on a transceiver they need to be set on

  adc->adc0->setAveraging(16);  // set number of averages
  adc->adc0->setResolution(16); // set bits of resolution
  adc->adc0->setConversionSpeed(ADC_CONVERSION_SPEED::MED_SPEED);
  adc->adc0->setSamplingSpeed(ADC_SAMPLING_SPEED::MED_SPEED);
  adc->adc0->startContinuous(ACUR1);

  SERIALCONSOLE.begin(115200);
  SERIALCONSOLE.println("Starting up!");
  SERIALCONSOLE.println("SimpBMS V2 Volt-Ampera");

  Serial2.begin(115200);
  Serial3.begin(9600);

  // Display reason the Teensy was last reset
  Serial.println();
  Serial.println("Reason for last Reset: ");

  if (RCM_SRS1 & RCM_SRS1_SACKERR)
    Serial.println("Sop Mode Acknowledge Error Reset");
  if (RCM_SRS1 & RCM_SRS1_MDM_AP)
    Serial.println("MDM-AP Reset");
  if (RCM_SRS1 & RCM_SRS1_SW)
    Serial.println("Software Reset"); // reboot with SCB_AIRCR = 0x05FA0004
  if (RCM_SRS1 & RCM_SRS1_LOCKUP)
    Serial.println("Core Lockup Event Reset");
  if (RCM_SRS0 & RCM_SRS0_POR)
    Serial.println("Power-on Reset"); // removed / applied power
  if (RCM_SRS0 & RCM_SRS0_PIN)
    Serial.println("External Pin Reset"); // Reboot with software download
  if (RCM_SRS0 & RCM_SRS0_WDOG)
    Serial.println("Watchdog(COP) Reset"); // WDT timed out
  if (RCM_SRS0 & RCM_SRS0_LOC)
    Serial.println("Loss of External Clock Reset");
  if (RCM_SRS0 & RCM_SRS0_LOL)
    Serial.println("Loss of Lock in PLL Reset");
  if (RCM_SRS0 & RCM_SRS0_LVD)
    Serial.println("Low-voltage Detect Reset");
  Serial.println();
  ///////////////////

  // enable WDT
  noInterrupts();                 // don't allow interrupts while setting up WDOG
  WDOG_UNLOCK = WDOG_UNLOCK_SEQ1; // unlock access to WDOG registers
  WDOG_UNLOCK = WDOG_UNLOCK_SEQ2;
  delayMicroseconds(1); // Need to wait a bit..

  WDOG_TOVALH = 0x1000;
  WDOG_TOVALL = 0x0000;
  WDOG_PRESC = 0;
  WDOG_STCTRLH |= WDOG_STCTRLH_ALLOWUPDATE |
                  WDOG_STCTRLH_WDOGEN | WDOG_STCTRLH_WAITEN |
                  WDOG_STCTRLH_STOPEN | WDOG_STCTRLH_CLKSRC;
  interrupts();
  /////////////////

  //VE.begin(19200); //Victron VE direct bus

  SERIALCONSOLE.println("Started serial interface");

  EEPROM.get(0, settings);
  if (settings.version != EEPROM_VERSION)
  {
    loadSettings();
  }
  canSerial.begin(settings.SerialCanBaud); //Expansion serial bus
  Logger::setLoglevel(Logger::Off);        //Debug = 0, Info = 1, Warn = 2, Error = 3, Off = 4

  lastUpdate = 0;

  digitalWrite(led, HIGH);
  bms.setPstrings(settings.Pstrings);
  bms.setSensors(settings.IgnoreTemp, settings.IgnoreVolt);

  ///precharge timer kickers
  Pretimer = millis();
  Pretimer1 = millis();
}

void loop()
{
  loopTimeMain = millis(); // get current loop time

  while (Can0.available())
  {
    canread();
  }

  if (SERIALCONSOLE.available() > 0)
  {
    menu();
  }

  if (settings.SerialCan == 1)
  {
    SerialCanRecieve();
  }

  dashDisplayListen(); // listen for info on NextionDisplay

  // MAIN STATE MACHINE
  if (outputcheck != 1)
  {
    contcon();
    if (settings.ESSmode == 1)
    {
      if (bmsstatus != Error)
      {
        contctrl = contctrl | 4; //turn on negative contactor

        if (digitalRead(IN1) == LOW) //IN1 OFF
        {
          if (storagemode == 1)
          {
            storagemode = 0;
          }
        }
        else
        {
          if (storagemode == 0)
          {
            storagemode = 1;
          }
        }
        if (bms.getAvgCellVolt() > settings.balanceVoltage && bms.getHighCellVolt() > bms.getLowCellVolt() + settings.balanceHyst)
        {
          balancecells = true;
        }
        else
        {
          balancecells = false;
        }

        //Pretimer + settings.Pretime > millis();

        if (storagemode == 1)
        {
          if (bms.getHighCellVolt() > settings.StoreVsetpoint)
          {
            digitalWrite(OUT3, LOW); //turn off charger
            contctrl = contctrl & 253;
            Pretimer = millis();
            Charged = 1;
            SOCcharged(100);
          }
          else
          {
            if (Charged == 1)
            {
              if (bms.getHighCellVolt() < (settings.StoreVsetpoint - settings.ChargeHys))
              {
                Charged = 0;
                digitalWrite(OUT3, HIGH); //turn on charger
                if (Pretimer + settings.Pretime < millis())
                {
                  contctrl = contctrl | 2;
                  Pretimer = 0;
                }
              }
            }
            else
            {
              digitalWrite(OUT3, HIGH); //turn on charger
              if (Pretimer + settings.Pretime < millis())
              {
                contctrl = contctrl | 2;
                Pretimer = 0;
              }
            }
          }
        }
        else
        {
          if (bms.getHighCellVolt() > settings.OverVSetpoint || bms.getHighCellVolt() > getChargeVSetpoint())
          {
            digitalWrite(OUT3, LOW); //turn off charger
            contctrl = contctrl & 253;
            Pretimer = millis();
            Charged = 1;
            SOCcharged(100);
          }
          else
          {
            if (Charged == 1)
            {
              if (bms.getHighCellVolt() < (getChargeVSetpoint() - settings.ChargeHys))
              {
                Charged = 0;
                digitalWrite(OUT3, HIGH); //turn on charger
                if (Pretimer + settings.Pretime < millis())
                {
                  // Serial.println();
                  //Serial.print(Pretimer);
                  contctrl = contctrl | 2;
                }
              }
            }
            else
            {
              digitalWrite(OUT3, HIGH); //turn on charger
              if (Pretimer + settings.Pretime < millis())
              {
                // Serial.println();
                //Serial.print(Pretimer);
                contctrl = contctrl | 2;
              }
            }
          }
        }
        if (bms.getLowCellVolt() < settings.UnderVSetpoint || bms.getLowCellVolt() < settings.DischVsetpoint)
        {
          digitalWrite(OUT1, LOW); //turn off discharge
          contctrl = contctrl & 254;
          Pretimer1 = millis();
        }
        else
        {
          digitalWrite(OUT1, HIGH); //turn on discharge
          if (Pretimer1 + settings.Pretime < millis())
          {
            contctrl = contctrl | 1;
          }
        }

        if (SOCset == 1)
        {
          if (bms.getLowCellVolt() < settings.UnderVSetpoint || bms.getHighCellVolt() > settings.OverVSetpoint || bms.getHighTemperature() > settings.OverTSetpoint)
          {
            digitalWrite(OUT2, HIGH); //trip breaker
          }
          else
          {
            digitalWrite(OUT2, LOW); //trip breaker
          }
        }
      }
      else
      {
        /*
          digitalWrite(OUT2, HIGH);//trip breaker
          Discharge = 0;
          digitalWrite(OUT4, LOW);
          digitalWrite(OUT3, LOW);//turn off charger
          digitalWrite(OUT2, LOW);
          digitalWrite(OUT1, LOW);//turn off discharge
          contctrl = 0; //turn off out 5 and 6
        */
        if (SOCset == 1)
        {
          if (bms.getLowCellVolt() < settings.UnderVSetpoint || bms.getHighCellVolt() > settings.OverVSetpoint || bms.getHighTemperature() > settings.OverTSetpoint)
          {
            digitalWrite(OUT2, HIGH); //trip breaker
          }
          else
          {
            digitalWrite(OUT2, LOW); //trip breaker
          }
        }
      }
      //pwmcomms();
    }
    else
    {
      switch (bmsstatus)
      {
      case (Boot):
        Discharge = 0;
        digitalWrite(OUT4, LOW);
        digitalWrite(OUT3, LOW); //turn off charger
        digitalWrite(OUT2, LOW);
        digitalWrite(OUT1, LOW); //turn off discharge
        contctrl = 0;
        bmsstatus = Ready;
        break;

      case (Ready):
        Discharge = 0;
        digitalWrite(OUT4, LOW);
        digitalWrite(OUT3, LOW); //turn off charger
        digitalWrite(OUT2, LOW);
        digitalWrite(OUT1, LOW); //turn off discharge
        contctrl = 0;            //turn off out 5 and 6
        //accurlim = 0;
        if (bms.getAvgCellVolt() > settings.balanceVoltage)
        {
          if (bms.getHighCellVolt() - bms.getLowCellVolt() > (settings.balanceHyst * 2.0)) // start balancing at hyst value
          {
            balancecells = true;
          }
          else if (bms.getHighCellVolt() - bms.getLowCellVolt() <= settings.balanceHyst) // stop balancing at half hyst
          {
            balancecells = false;
          }
        }
        else
        {
          balancecells = false;
        }

        if (digitalRead(IN3) == HIGH && !balancecells && bms.getHighTemperature() < (settings.OverTSetpoint - settings.WarnToff)) //detect AC present for charging and check not balancing
        {
          if (settings.ChargerDirect == 1)
          {
            bmsstatus = Charge;
          }
          else
          {
            bmsstatus = Precharge;
            Pretimer = millis();
          }
        }

        if (digitalRead(IN1) == HIGH) //detect IN1 ON
        {
          balancecells = false; // stop balancing
          bmsstatus = Precharge;
          Pretimer = millis();
        }

        break;

      case (Precharge):
        Discharge = 0;
        Prechargecon();
        break;

      case (Drive):
        Discharge = 1;
        //accurlim = 0;
        if (digitalRead(IN1) == LOW) //IN1 OFF
        {
          bmsstatus = Ready;
        }
        if (digitalRead(IN3) == HIGH) //detect AC present for charging
        {
          bmsstatus = Charge;
        }

        break;

      case (Charge):
        Discharge = 0;
        balancecells = false;
        if (settings.ChargerDirect == 1)
        {
          digitalWrite(OUT4, LOW);
          digitalWrite(OUT2, LOW);
          digitalWrite(OUT1, LOW); //turn off discharge
          contctrl = 0;            //turn off out 5 and 6
        }
        /*
            if (digitalRead(IN2) == HIGH)
            {
            chargecurrentlimit = true;
            }
            else
            {
            chargecurrentlimit = false;
            }
          */
        digitalWrite(OUT3, HIGH); //enable charger
        if (bms.getAvgCellVolt() > settings.balanceVoltage && bms.getHighCellVolt() - bms.getLowCellVolt() > (settings.balanceHyst * 2.0))
        {
          balancecells = true;
          bmsstatus = Ready;
        }

        // RESET Charge AH
        if (bms.getHighCellVolt() > getChargeVSetpoint() || bms.getHighTemperature() > settings.OverTSetpoint)
        {
          if (bms.getAvgCellVolt() > (getChargeVSetpoint() - settings.balanceHyst))
          {
            SOCcharged(100);
          }
          else
          {
            SOCcharged(95);
          }
          bmsstatus = Ready;
        }

        if (digitalRead(IN3) == LOW) //detect AC not present for charging
        {
          bmsstatus = Ready;
        }

        break;

      case (Error):
        Discharge = 0;
        digitalWrite(OUT4, LOW);
        digitalWrite(OUT3, LOW); //turn off charger
        digitalWrite(OUT2, LOW);
        digitalWrite(OUT1, LOW); //turn off discharge
        contctrl = 0;            //turn off out 5 and 6
        /*
        if (digitalRead(IN3) == HIGH) //detect AC present for charging
        {
          bmsstatus = Charge;
        }
          */
        if (bms.getLowCellVolt() > settings.UnderVSetpoint && bms.getHighCellVolt() < settings.OverVSetpoint)
        {
          bmsstatus = Ready;
        }
        break;
      }
    }
    if (settings.cursens == Analoguedual || settings.cursens == Analoguesing)
    {
      // if (bmsstatus == Ready && settings.ESSmode == 0) // powersave only in EV Mode
      //   currentact = 0.0f;
      // else
        getcurrent();
    }
  }

  // main loop 1000ms
  if (loopTimeMain - looptime >= 1000) // process sequence 1sec
  {
    looptime = loopTimeMain; // reset loop time
    bms.getAllVoltTemp();

    //UV  check
    if (settings.ESSmode == 1)
    {
      if (SOCset != 0)
      {
        if (bms.getLowCellVolt() < settings.UnderVSetpoint || bms.getHighCellVolt() < settings.UnderVSetpoint)
        {
          if (debug != 0)
          {
            SERIALCONSOLE.println("  ");
            SERIALCONSOLE.print("   !!! Undervoltage Fault !!!");
            SERIALCONSOLE.println("  ");
          }
          bmsstatus = Error;
          ErrorReason = 1;
        }
      }
    }
    else //In 'vehicle' mode
    {
      if (bms.getLowCellVolt() < settings.UnderVSetpoint || bms.getHighCellVolt() < settings.UnderVSetpoint)
      {
        if (UnderTime > millis()) //check is last time not undervoltage is longer thatn UnderDur ago
        {
          bmsstatus = Error;
          ErrorReason = 2;
        }
      }
      else
      {
        UnderTime = millis() + settings.UnderDur;
      }
    }

    if (debug != 0)
    {
      printbmsstat();
      bms.printPackDetails(debugdigits, 0);
    }
    if (CSVdebug != 0)
    {
      bms.printAllCSV(millis(), currentact, SOC);
    }
    if (inputcheck != 0)
    {
      inputdebug();
    }

    if (outputcheck != 0)
    {
      outputdebug();
    }
    else
    {
      gaugeupdate();
    }

    updateSOC();
    currentlimit();

    VEcan();

    if (!balancecells)
      requestBICMdata(); // request data here only if not balancing.

    if (cellspresent == 0 && SOCset == 1)
    {
      cellspresent = bms.seriescells();
      bms.setSensors(settings.IgnoreTemp, settings.IgnoreVolt);
    }
    else
    {
      if (cellspresent != bms.seriescells() || cellspresent != (settings.Scells * settings.Pstrings)) //detect a fault in cells detected
      {
        SERIALCONSOLE.println("  ");
        SERIALCONSOLE.print("   !!! Series Cells Fault !!!");
        SERIALCONSOLE.println("  ");
        bmsstatus = Error;
        ErrorReason = 3;
      }
    }
    alarmupdate();
    if (CSVdebug != 1)
    {
      dashupdate();
    }

    resetwdog();
  }

  // can loop 200ms
  if (loopTimeMain - loopTimeBalance >= 200)
  {
    loopTimeBalance = loopTimeMain;           // reset loop time
    if (balancecells && loopTimeMain > 15000) // delay balancing
    {
      sendBalanceCommands();
    }
  }

  if (millis() - cleartime > 5000)
  {
    /*
      //bms.clearmodules(); // Not functional
      if (bms.checkcomms())
      {
      //no missing modules
      SERIALCONSOLE.println("  ");
      SERIALCONSOLE.print(" ALL OK NO MODULE MISSING :) ");
      SERIALCONSOLE.println("  ");
      if (  bmsstatus == Error)
      {
        bmsstatus = Boot;
      }
      }
      else
      {
      //missing module
      SERIALCONSOLE.println("  ");
      SERIALCONSOLE.print("   !!! MODULE MISSING !!!");
      SERIALCONSOLE.println("  ");
      //bmsstatus = Error;
      ErrorReason = 4;
      }
    */
    cleartime = millis();
  }

  if (loopTimeMain - looptime1 > settings.chargerspd)
  {
    looptime1 = loopTimeMain;
    if (settings.ESSmode == 1)
    {
      chargercomms();
      if (settings.SerialCan == 1)
      {
        CanSerial();
      }
    }
    else
    {
      if (settings.SerialCan == 1)
      {
        CanSerial();
      }
      else
      {
        if (bmsstatus == Charge)
        {
          chargercomms();
        }
      }
    }
  }
}

void alarmupdate()
{
  alarm[0] = 0x00;
  if (settings.OverVSetpoint < bms.getHighCellVolt())
  {
    alarm[0] = 0x04;
  }
  if (bms.getLowCellVolt() < settings.UnderVSetpoint)
  {
    alarm[0] |= 0x10;
  }
  if (bms.getHighTemperature() > settings.OverTSetpoint)
  {
    alarm[0] |= 0x40;
  }
  alarm[1] = 0;
  if (bms.getLowTemperature() < settings.UnderTSetpoint)
  {
    alarm[1] = 0x01;
  }
  alarm[3] = 0;
  if ((bms.getHighCellVolt() - bms.getLowCellVolt()) > settings.CellGap)
  {
    alarm[3] = 0x01;
  }

  ///warnings///
  warning[0] = 0;

  if (bms.getHighCellVolt() > (settings.OverVSetpoint - settings.WarnOff))
  {
    warning[0] = 0x04;
  }
  if (bms.getLowCellVolt() < (settings.UnderVSetpoint + settings.WarnOff))
  {
    warning[0] |= 0x10;
  }

  if (bms.getHighTemperature() > (settings.OverTSetpoint - settings.WarnToff))
  {
    warning[0] |= 0x40;
  }
  warning[1] = 0;
  if (bms.getLowTemperature() < (settings.UnderTSetpoint + settings.WarnToff))
  {
    warning[1] = 0x01;
  }
}

void gaugeupdate()
{
  if (gaugedebug == 1)
  {
    SOCtest = SOCtest + 10;
    if (SOCtest > 1000)
    {
      SOCtest = 0;
    }
    analogWrite(OUT8, map(SOCtest * 0.1, 0, 100, settings.gaugelow, settings.gaugehigh));

    SERIALCONSOLE.println("  ");
    SERIALCONSOLE.print("SOC : ");
    SERIALCONSOLE.print(SOCtest * 0.1);
    SERIALCONSOLE.print("  fuel pwm : ");
    SERIALCONSOLE.print(map(SOCtest * 0.1, 0, 100, settings.gaugelow, settings.gaugehigh));
    SERIALCONSOLE.println("  ");
  }
  if (gaugedebug == 2)
  {
    SOCtest = 0;
    analogWrite(OUT8, map(SOCtest * 0.1, 0, 100, settings.gaugelow, settings.gaugehigh));
  }
  if (gaugedebug == 3)
  {
    SOCtest = 1000;
    analogWrite(OUT8, map(SOCtest * 0.1, 0, 100, settings.gaugelow, settings.gaugehigh));
  }
  if (gaugedebug == 0)
  {
    analogWrite(OUT8, map(SOC, 0, 100, settings.gaugelow, settings.gaugehigh));
  }
}

void printbmsstat()
{
  SERIALCONSOLE.println();
  SERIALCONSOLE.println();
  SERIALCONSOLE.println();
  SERIALCONSOLE.print("BMS Status : ");
  if (settings.ESSmode == 1)
  {
    SERIALCONSOLE.print("ESS Mode ");

    if (bms.getLowCellVolt() < settings.UnderVSetpoint)
    {
      SERIALCONSOLE.print(": UnderVoltage ");
    }
    if (bms.getHighCellVolt() > settings.OverVSetpoint)
    {
      SERIALCONSOLE.print(": OverVoltage ");
    }
    if ((bms.getHighCellVolt() - bms.getLowCellVolt()) > settings.CellGap)
    {
      SERIALCONSOLE.print(": Cell Imbalance ");
    }
    if (bms.getAvgTemperature() > settings.OverTSetpoint)
    {
      SERIALCONSOLE.print(": Over Temp ");
    }
    if (bms.getAvgTemperature() < settings.UnderTSetpoint)
    {
      SERIALCONSOLE.print(": Under Temp ");
    }
    if (storagemode == 1)
    {
      if (bms.getLowCellVolt() > settings.StoreVsetpoint)
      {
        SERIALCONSOLE.print(": OverVoltage Storage ");
        SERIALCONSOLE.print(": UNhappy:");
      }
      else
      {
        SERIALCONSOLE.print(": Happy ");
      }
    }
    else
    {
      if (bms.getLowCellVolt() > settings.UnderVSetpoint && bms.getHighCellVolt() < settings.OverVSetpoint)
      {

        if (bmsstatus == Error)
        {
          SERIALCONSOLE.print(": UNhappy:");
        }
        else
        {
          SERIALCONSOLE.print(": Happy ");
        }
      }
    }
  }
  else
  {
    SERIALCONSOLE.print(bmsstatus);
    switch (bmsstatus)
    {
    case (Boot):
      SERIALCONSOLE.print(" Boot ");
      break;

    case (Ready):
      SERIALCONSOLE.print(" Ready ");
      break;

    case (Precharge):
      SERIALCONSOLE.print(" Precharge ");
      break;

    case (Drive):
      SERIALCONSOLE.print(" Drive ");
      break;

    case (Charge):
      SERIALCONSOLE.print(" Charge ");
      break;

    case (Error):
      SERIALCONSOLE.print(" Error ");
      SERIALCONSOLE.print(ErrorReason);
      break;
    }
  }
  SERIALCONSOLE.print("  ");
  if (digitalRead(IN3) == HIGH)
  {
    SERIALCONSOLE.print("| AC Present |");
  }
  if (digitalRead(IN1) == HIGH)
  {
    SERIALCONSOLE.print("| IN1 ON |");
  }
  if (balancecells)
  {
    SERIALCONSOLE.print("| Balancing Active");
  }
  SERIALCONSOLE.print("  ");
  SERIALCONSOLE.print(cellspresent);
  SERIALCONSOLE.println();
  SERIALCONSOLE.print("Out:");
  SERIALCONSOLE.print(digitalRead(OUT1));
  SERIALCONSOLE.print(digitalRead(OUT2));
  SERIALCONSOLE.print(digitalRead(OUT3));
  SERIALCONSOLE.print(digitalRead(OUT4));
  SERIALCONSOLE.print(" Cont:");
  if ((contstat & 1) == 1)
  {
    SERIALCONSOLE.print("1");
  }
  else
  {
    SERIALCONSOLE.print("0");
  }
  if ((contstat & 2) == 2)
  {
    SERIALCONSOLE.print("1");
  }
  else
  {
    SERIALCONSOLE.print("0");
  }
  if ((contstat & 4) == 4)
  {
    SERIALCONSOLE.print("1");
  }
  else
  {
    SERIALCONSOLE.print("0");
  }
  if ((contstat & 8) == 8)
  {
    SERIALCONSOLE.print("1");
  }
  else
  {
    SERIALCONSOLE.print("0");
  }
  SERIALCONSOLE.print(" In:");
  SERIALCONSOLE.print(digitalRead(IN1));
  SERIALCONSOLE.print(digitalRead(IN2));
  SERIALCONSOLE.print(digitalRead(IN3));
  SERIALCONSOLE.print(digitalRead(IN4));
}

void getcurrent()
{
  if (settings.cursens == Analoguedual || settings.cursens == Analoguesing)
  {
    if (settings.cursens == Analoguedual)
    {
      if (fabs(currentact) < settings.changecur)
      {
        sensor = 1;
        adc->adc0->startContinuous(ACUR1);
      }
      else
      {
        sensor = 2;
        adc->adc0->startContinuous(ACUR2);
      }
    }
    else
    {
      sensor = 1;
      adc->adc0->startContinuous(ACUR1);
    }
    if (sensor == 1)
    {
      if (debugCur != 0)
      {
        SERIALCONSOLE.println();
        if (settings.cursens == Analoguedual)
        {
          SERIALCONSOLE.print("Low Range: ");
        }
        else
        {
          SERIALCONSOLE.print("Single In: ");
        }
        SERIALCONSOLE.print("Value ADC0: ");
      }
      value = (uint16_t)adc->adc0->analogReadContinuous(); // the unsigned is necessary for 16 bits, otherwise values larger than 3.3/2 V are negative!
      if (debugCur != 0)
      {
        SERIALCONSOLE.print(value * 3300 / adc->adc0->getMaxValue()); //- settings.offset1)
        SERIALCONSOLE.print(" ");
        SERIALCONSOLE.print(settings.offset1);
      }
      RawCur = int16_t((value * 3300 / adc->adc0->getMaxValue()) - settings.offset1) / (settings.convlow * 0.0000066);

      if (abs((int16_t(value * 3300 / adc->adc0->getMaxValue()) - settings.offset1)) < settings.CurDead)
      {
        RawCur = 0;
      }
      if (debugCur != 0)
      {
        SERIALCONSOLE.print("  ");
        SERIALCONSOLE.print(int16_t(value * 3300 / adc->adc0->getMaxValue()) - settings.offset1);
        SERIALCONSOLE.print("  ");
        SERIALCONSOLE.print(RawCur);
        SERIALCONSOLE.print(" mA");
        SERIALCONSOLE.print("  ");
      }
    }
    else
    {
      if (debugCur != 0)
      {
        SERIALCONSOLE.println();
        SERIALCONSOLE.print("High Range: ");
        SERIALCONSOLE.print("Value ADC0: ");
      }
      value = (uint16_t)adc->adc0->analogReadContinuous(); // the unsigned is necessary for 16 bits, otherwise values larger than 3.3/2 V are negative!
      if (debugCur != 0)
      {
        SERIALCONSOLE.print(value * 3300 / adc->adc0->getMaxValue()); //- settings.offset2)
        SERIALCONSOLE.print("  ");
        SERIALCONSOLE.print(settings.offset2);
      }
      RawCur = int16_t((value * 3300 / adc->adc0->getMaxValue()) - settings.offset2) / (settings.convhigh * 0.0000066);
      if (value < 100 || value > (adc->adc0->getMaxValue() - 100))
      {
        RawCur = 0;
      }
      if (debugCur != 0)
      {
        SERIALCONSOLE.print("  ");
        SERIALCONSOLE.print((float(value * 3300 / adc->adc0->getMaxValue()) - settings.offset2));
        SERIALCONSOLE.print("  ");
        SERIALCONSOLE.print(RawCur);
        SERIALCONSOLE.print("mA");
        SERIALCONSOLE.print("  ");
      }
    }
  }

  if (settings.invertcur == 1)
  {
    RawCur = RawCur * -1;
  }

  lowpassFilter.input(RawCur);
  if (debugCur != 0)
  {
    SERIALCONSOLE.print(lowpassFilter.output());
    SERIALCONSOLE.print(" | ");
    SERIALCONSOLE.print(settings.changecur);
    SERIALCONSOLE.print(" | ");
  }

  currentact = lowpassFilter.output();

  if (debugCur != 0)
  {
    SERIALCONSOLE.print(currentact);
    SERIALCONSOLE.print("mA  ");
  }

  if (settings.cursens == Analoguedual)
  {
    if (sensor == 1)
    {
      if (fabs(currentact) > 500)
      {
        ampsecond = ampsecond + ((currentact * (millis() - lasttime) / 1000) / 1000);
        lasttime = millis();
      }
      else
      {
        lasttime = millis();
      }
    }
    if (sensor == 2)
    {
      if (fabs(currentact) > settings.changecur)
      {
        ampsecond = ampsecond + ((currentact * (millis() - lasttime) / 1000) / 1000);
        lasttime = millis();
      }
      else
      {
        lasttime = millis();
      }
    }
  }
  else
  {
    if (fabs(currentact) > 500)
    {
      ampsecond = ampsecond + ((currentact * (millis() - lasttime) / 1000) / 1000);
      lasttime = millis();
    }
    else
    {
      lasttime = millis();
    }
  }
  currentact = settings.ncur * currentact;
  RawCur = 0;
  /*
    AverageCurrentTotal = AverageCurrentTotal - RunningAverageBuffer[NextRunningAverage];

    RunningAverageBuffer[NextRunningAverage] = currentact;

    if (debugCur != 0)
    {
      SERIALCONSOLE.print(" | ");
      SERIALCONSOLE.print(AverageCurrentTotal);
      SERIALCONSOLE.print(" | ");
      SERIALCONSOLE.print(RunningAverageBuffer[NextRunningAverage]);
      SERIALCONSOLE.print(" | ");
    }
    AverageCurrentTotal = AverageCurrentTotal + RunningAverageBuffer[NextRunningAverage];
    if (debugCur != 0)
    {
      SERIALCONSOLE.print(" | ");
      SERIALCONSOLE.print(AverageCurrentTotal);
      SERIALCONSOLE.print(" | ");
    }

    NextRunningAverage = NextRunningAverage + 1;

    if (NextRunningAverage > RunningAverageCount)
    {
      NextRunningAverage = 0;
    }

    AverageCurrent = AverageCurrentTotal / (RunningAverageCount + 1);

    if (debugCur != 0)
    {
      SERIALCONSOLE.print(AverageCurrent);
      SERIALCONSOLE.print(" | ");
      SERIALCONSOLE.print(AverageCurrentTotal);
      SERIALCONSOLE.print(" | ");
      SERIALCONSOLE.print(NextRunningAverage);
    }
  */
}

void updateSOC()
{
  if (SOCset == 0)
  {
    if (millis() > 9000)
    {
      bms.setSensors(settings.IgnoreTemp, settings.IgnoreVolt);
    }
    if (millis() > 10000)
    {
      SOC = map(uint16_t(bms.getAvgCellVolt() * 1000), settings.socvolt[0], settings.socvolt[2], settings.socvolt[1], settings.socvolt[3]);

      ampsecond = (SOC * settings.CAP * settings.Pstrings * 10) / 0.27777777777778;
      SOCset = 1;
      if (debug != 0)
      {
        SERIALCONSOLE.println("  ");
        SERIALCONSOLE.println("//////////////////////////////////////// SOC SET ////////////////////////////////////////");
      }
    }
  }
  if (settings.voltsoc == 1)
  {
    SOC = map(uint16_t(bms.getAvgCellVolt() * 1000), settings.socvolt[0], settings.socvolt[2], settings.socvolt[1], settings.socvolt[3]);

    ampsecond = (SOC * settings.CAP * settings.Pstrings * 10) / 0.27777777777778;
  }
  SOC = ((ampsecond * 0.27777777777778) / (settings.CAP * settings.Pstrings * 1000)) * 100;
  if (SOC >= 100)
  {
    ampsecond = (settings.CAP * settings.Pstrings * 1000) / 0.27777777777778; //reset to full, dependant on given capacity. Need to improve with auto correction for capcity.
    SOC = 100;
  }

  if (SOC < 0)
  {
    SOC = 0; //reset SOC this way the can messages remain in range for other devices. Ampseconds will keep counting.
  }

  if (debug != 0)
  {
    if (settings.cursens == Analoguedual)
    {
      if (sensor == 1)
      {
        SERIALCONSOLE.print("Low Range ");
      }
      else
      {
        SERIALCONSOLE.print("High Range");
      }
    }
    if (settings.cursens == Analoguesing)
    {
      SERIALCONSOLE.print("Analogue Single ");
    }
    if (settings.cursens == Canbus)
    {
      SERIALCONSOLE.print("CANbus ");
    }
    SERIALCONSOLE.print("  ");
    SERIALCONSOLE.print(currentact);
    SERIALCONSOLE.print("mA");
    SERIALCONSOLE.print("  ");
    SERIALCONSOLE.print(SOC);
    SERIALCONSOLE.print("% SOC ");
    SERIALCONSOLE.print(ampsecond * 0.27777777777778, 2);
    SERIALCONSOLE.println("mAh");
  }
}

void SOCcharged(int percent)
{
  SOC = percent * (maxchargepercent / 100);
  ampsecond = (percent / 100 * settings.CAP * settings.Pstrings * 1000) / 0.27777777777778; //reset to full, dependant on given capacity. Need to improve with auto correction for capcity.
}

void Prechargecon()
{
  if (digitalRead(IN1) == HIGH || digitalRead(IN3) == HIGH) //detect IN1 ON or AC present
  {
    digitalWrite(OUT4, HIGH); //Negative Contactor Close
    contctrl = 2;
    if (millis() - Pretimer < settings.Pretime || fabs(currentact) > settings.Precurrent)
    {
      digitalWrite(OUT2, HIGH); //precharge
    }
    else //close main contactor
    {
      digitalWrite(OUT1, HIGH); //Positive Contactor Close
      contctrl = 3;
      if (settings.ChargerDirect == 1)
      {
        bmsstatus = Drive;
      }
      else
      {
        if (digitalRead(IN1) == HIGH)
        {
          bmsstatus = Drive;
        }
        if (digitalRead(IN3) == HIGH)
        {
          bmsstatus = Charge;
        }
      }
      digitalWrite(OUT2, LOW);
    }
  }
  else
  {
    digitalWrite(OUT1, LOW);
    digitalWrite(OUT2, LOW);
    digitalWrite(OUT4, LOW);
    bmsstatus = Ready;
    contctrl = 0;
  }
}

void contcon()
{
  if (contctrl != contstat) //check for contactor request change
  {
    if ((contctrl & 1) == 0)
    {
      analogWrite(OUT5, 0);
      contstat = contstat & 254;
    }
    if ((contctrl & 2) == 0)
    {
      analogWrite(OUT6, 0);
      contstat = contstat & 253;
    }
    if ((contctrl & 4) == 0)
    {
      analogWrite(OUT7, 0);
      contstat = contstat & 251;
    }

    if ((contctrl & 1) == 1)
    {
      if ((contstat & 1) != 1)
      {
        if (conttimer1 == 0)
        {
          analogWrite(OUT5, 255);
          conttimer1 = millis() + pulltime;
        }
        if (conttimer1 < millis())
        {
          analogWrite(OUT5, settings.conthold);
          contstat = contstat | 1;
          conttimer1 = 0;
        }
      }
    }

    if ((contctrl & 2) == 2)
    {
      if ((contstat & 2) != 2)
      {
        if (conttimer2 == 0)
        {
          if (debug != 0)
          {
            Serial.println();
            Serial.println("pull in OUT6");
          }
          analogWrite(OUT6, 255);
          conttimer2 = millis() + pulltime;
        }
        if (conttimer2 < millis())
        {
          analogWrite(OUT6, settings.conthold);
          contstat = contstat | 2;
          conttimer2 = 0;
        }
      }
    }
    if ((contctrl & 4) == 4)
    {
      if ((contstat & 4) != 4)
      {
        if (conttimer3 == 0)
        {
          if (debug != 0)
          {
            Serial.println();
            Serial.println("pull in OUT7");
          }
          analogWrite(OUT7, 255);
          conttimer3 = millis() + pulltime;
        }
        if (conttimer3 < millis())
        {
          analogWrite(OUT7, settings.conthold);
          contstat = contstat | 4;
          conttimer3 = 0;
        }
      }
    }
    /*
       SERIALCONSOLE.print(conttimer);
       SERIALCONSOLE.print("  ");
       SERIALCONSOLE.print(contctrl);
       SERIALCONSOLE.print("  ");
       SERIALCONSOLE.print(contstat);
       SERIALCONSOLE.println("  ");
    */
  }
  if (contctrl == 0)
  {
    analogWrite(OUT5, 0);
    analogWrite(OUT6, 0);
  }
}

void calcur()
{
  adc->adc0->startContinuous(ACUR1);
  sensor = 1;
  x = 0;
  SERIALCONSOLE.print(" Calibrating Current Offset ::::: ");
  while (x < 20)
  {
    settings.offset1 = settings.offset1 + ((uint16_t)adc->adc0->analogReadContinuous() * 3300 / adc->adc0->getMaxValue());
    SERIALCONSOLE.print(".");
    delay(100);
    x++;
  }
  settings.offset1 = settings.offset1 / 21;
  SERIALCONSOLE.print(settings.offset1);
  SERIALCONSOLE.print(" current offset 1 calibrated ");
  SERIALCONSOLE.println("  ");
  x = 0;
  adc->startContinuous(ACUR2, ADC_0);
  sensor = 2;
  SERIALCONSOLE.print(" Calibrating Current Offset ::::: ");
  while (x < 20)
  {
    settings.offset2 = settings.offset2 + ((uint16_t)adc->adc0->analogReadContinuous() * 3300 / adc->adc0->getMaxValue());
    SERIALCONSOLE.print(".");
    delay(100);
    x++;
  }
  settings.offset2 = settings.offset2 / 21;
  SERIALCONSOLE.print(settings.offset2);
  SERIALCONSOLE.print(" current offset 2 calibrated ");
  SERIALCONSOLE.println("  ");
}
void VEcan() //communication with Victron system over CAN
{
  msg.id = 0x351;
  msg.len = 8;
  if (storagemode == 0)
  {
    msg.buf[0] = lowByte(uint16_t((getChargeVSetpoint() * settings.Scells) * 10));
    msg.buf[1] = highByte(uint16_t((getChargeVSetpoint() * settings.Scells) * 10));
  }
  else
  {
    msg.buf[0] = lowByte(uint16_t((settings.StoreVsetpoint * settings.Scells) * 10));
    msg.buf[1] = highByte(uint16_t((settings.StoreVsetpoint * settings.Scells) * 10));
  }
  msg.buf[2] = lowByte(chargecurrent);
  msg.buf[3] = highByte(chargecurrent);
  msg.buf[4] = lowByte(discurrent);
  msg.buf[5] = highByte(discurrent);
  msg.buf[6] = lowByte(uint16_t((settings.DischVsetpoint * settings.Scells) * 10));
  msg.buf[7] = highByte(uint16_t((settings.DischVsetpoint * settings.Scells) * 10));
  Can0.write(msg);

  msg.id = 0x355;
  msg.len = 8;
  msg.buf[0] = lowByte(SOC);
  msg.buf[1] = highByte(SOC);
  msg.buf[2] = lowByte(SOH);
  msg.buf[3] = highByte(SOH);
  msg.buf[4] = lowByte(SOC * 10);
  msg.buf[5] = highByte(SOC * 10);
  msg.buf[6] = 0;
  msg.buf[7] = 0;
  Can0.write(msg);

  msg.id = 0x356;
  msg.len = 8;
  msg.buf[0] = lowByte(uint16_t(bms.getPackVoltage() * 100));
  msg.buf[1] = highByte(uint16_t(bms.getPackVoltage() * 100));
  msg.buf[2] = lowByte(long(currentact / 100));
  msg.buf[3] = highByte(long(currentact / 100));
  msg.buf[4] = lowByte(int16_t(bms.getAvgTemperature() * 10));
  msg.buf[5] = highByte(int16_t(bms.getAvgTemperature() * 10));
  msg.buf[6] = 0;
  msg.buf[7] = 0;
  Can0.write(msg);

  delay(2);
  msg.id = 0x35A;
  msg.len = 8;
  msg.buf[0] = alarm[0];   //High temp  Low Voltage | High Voltage
  msg.buf[1] = alarm[1];   // High Discharge Current | Low Temperature
  msg.buf[2] = alarm[2];   //Internal Failure | High Charge current
  msg.buf[3] = alarm[3];   // Cell Imbalance
  msg.buf[4] = warning[0]; //High temp  Low Voltage | High Voltage
  msg.buf[5] = warning[1]; // High Discharge Current | Low Temperature
  msg.buf[6] = warning[2]; //Internal Failure | High Charge current
  msg.buf[7] = warning[3]; // Cell Imbalance
  Can0.write(msg);

  msg.id = 0x35E;
  msg.len = 8;
  msg.buf[0] = bmsname[0];
  msg.buf[1] = bmsname[1];
  msg.buf[2] = bmsname[2];
  msg.buf[3] = bmsname[3];
  msg.buf[4] = bmsname[4];
  msg.buf[5] = bmsname[5];
  msg.buf[6] = bmsname[6];
  msg.buf[7] = bmsname[7];
  Can0.write(msg);

  delay(2);
  msg.id = 0x370;
  msg.len = 8;
  msg.buf[0] = bmsmanu[0];
  msg.buf[1] = bmsmanu[1];
  msg.buf[2] = bmsmanu[2];
  msg.buf[3] = bmsmanu[3];
  msg.buf[4] = bmsmanu[4];
  msg.buf[5] = bmsmanu[5];
  msg.buf[6] = bmsmanu[6];
  msg.buf[7] = bmsmanu[7];
  Can0.write(msg);
}

void sendBalanceCommands() // send CAN commands to balance cells
{
  sendcommand();
  bms.balanceCells();
}

void requestBICMdata()
{
  sendcommand();

  for (int c = 0; c < 8; c++)
  {
    msg.buf[c] = 0;
  }
  msg.id = 0x300;
  msg.len = 8;
  Can0.write(msg);

  for (int c = 0; c < 8; c++)
  {
    msg.buf[c] = 0;
  }
  msg.id = 0x310;
  msg.len = 5;
  Can0.write(msg);
}

void BMVmessage() //communication with the Victron Color Control System over VEdirect
{
  lasttime = millis();
  x = 0;
  VE.write(13);
  VE.write(10);
  VE.write(myStrings[0]);
  VE.write(9);
  VE.print(bms.getPackVoltage() * 1000, 0);
  VE.write(13);
  VE.write(10);
  VE.write(myStrings[2]);
  VE.write(9);
  VE.print(currentact);
  VE.write(13);
  VE.write(10);
  VE.write(myStrings[4]);
  VE.write(9);
  VE.print(ampsecond * 0.27777777777778, 0); //consumed ah
  VE.write(13);
  VE.write(10);
  VE.write(myStrings[6]);
  VE.write(9);
  VE.print(SOC * 10); //SOC
  x = 8;
  while (x < 20)
  {
    VE.write(13);
    VE.write(10);
    VE.write(myStrings[x]);
    x++;
    VE.write(9);
    VE.write(myStrings[x]);
    x++;
  }
  VE.write(13);
  VE.write(10);
  VE.write("Checksum");
  VE.write(9);
  VE.write(0x50); //0x59
  delay(10);

  while (x < 44)
  {
    VE.write(13);
    VE.write(10);
    VE.write(myStrings[x]);
    x++;
    VE.write(9);
    VE.write(myStrings[x]);
    x++;
  }
  /*
    VE.write(13);
    VE.write(10);
    VE.write(myStrings[32]);
    VE.write(9);
    VE.print(bms.getLowVoltage()*1000,0);
    VE.write(13);
    VE.write(10);
    VE.write(myStrings[34]);
    VE.write(9);
    VE.print(bms.getHighVoltage()*1000,0);
    x=36;

    while(x < 43)
    {
     VE.write(13);
     VE.write(10);
     VE.write(myStrings[x]);
     x ++;
     VE.write(9);
     VE.write(myStrings[x]);
     x ++;
    }
  */
  VE.write(13);
  VE.write(10);
  VE.write("Checksum");
  VE.write(9);
  VE.write(231);
}

// Settings menu
void menu()
{

  incomingByte = Serial.read(); // read the incoming byte:
  if (menuload == 4)
  {
    switch (incomingByte)
    {

    case '1':
      menuload = 1;
      candebug = !candebug;
      incomingByte = 'd';
      break;

    case '2':
      menuload = 1;
      debugCur = !debugCur;
      incomingByte = 'd';
      break;

    case '3':
      menuload = 1;
      outputcheck = !outputcheck;
      if (outputcheck == 0)
      {
        contctrl = 0;
        digitalWrite(OUT1, LOW);
        digitalWrite(OUT2, LOW);
        digitalWrite(OUT3, LOW);
        digitalWrite(OUT4, LOW);
      }
      incomingByte = 'd';
      break;

    case '4':
      menuload = 1;
      inputcheck = !inputcheck;
      incomingByte = 'd';
      break;

    case '5':
      menuload = 1;
      settings.ESSmode = !settings.ESSmode;
      incomingByte = 'd';
      break;

    case '6':
      menuload = 1;
      cellspresent = bms.seriescells();
      incomingByte = 'd';
      break;

    case '7':
      menuload = 1;
      gaugedebug = !gaugedebug;
      incomingByte = 'd';
      break;

    case '8':
      menuload = 1;
      CSVdebug = !CSVdebug;
      incomingByte = 'd';
      break;

    case '9':
      menuload = 1;
      if (Serial.available() > 0)
      {
        debugdigits = Serial.parseInt();
      }
      if (debugdigits > 4)
      {
        debugdigits = 2;
      }
      incomingByte = 'd';
      break;

    case '0':
      menuload = 1;
      settings.disp = !settings.disp;
      incomingByte = 'd';
      break;

    case 'a':
      menuload = 1;
      CanDebugSerial = !CanDebugSerial;
      incomingByte = 'd';
      break;

    case 113: //q for quite menu

      menuload = 0;
      incomingByte = 115;
      break;

    default:
      // if nothing else matches, do the default
      // default is optional
      break;
    }
  }

  if (menuload == 2)
  {
    switch (incomingByte)
    {

    case 99: //c for calibrate zero offset

      calcur();
      break;

    case '1':
      menuload = 1;
      settings.invertcur = !settings.invertcur;
      incomingByte = 'c';
      break;

    case '2':
      menuload = 1;
      settings.voltsoc = !settings.voltsoc;
      incomingByte = 'c';
      break;

    case '3':
      menuload = 1;
      if (Serial.available() > 0)
      {
        settings.ncur = Serial.parseInt();
      }
      menuload = 1;
      incomingByte = 'c';
      break;

    case '8':
      menuload = 1;
      if (Serial.available() > 0)
      {
        settings.changecur = Serial.parseInt();
      }
      menuload = 1;
      incomingByte = 'c';
      break;

    case '4':
      menuload = 1;
      if (Serial.available() > 0)
      {
        settings.convlow = Serial.parseInt();
      }
      incomingByte = 'c';
      break;

    case '5':
      menuload = 1;
      if (Serial.available() > 0)
      {
        settings.convhigh = Serial.parseInt();
      }
      incomingByte = 'c';
      break;

    case '6':
      menuload = 1;
      if (Serial.available() > 0)
      {
        settings.CurDead = Serial.parseInt();
      }
      incomingByte = 'c';
      break;

    case 113: //q for quite menu

      menuload = 0;
      incomingByte = 115;
      break;

    case 115: //s for switch sensor
      settings.cursens++;
      if (settings.cursens > 3)
      {
        settings.cursens = 0;
      }
      /*
          if (settings.cursens == Analoguedual)
          {
            settings.cursens = Canbus;
            SERIALCONSOLE.println("  ");
            SERIALCONSOLE.print(" CANbus Current Sensor ");
            SERIALCONSOLE.println("  ");
          }
          else
          {
            settings.cursens = Analoguedual;
            SERIALCONSOLE.println("  ");
            SERIALCONSOLE.print(" Analogue Current Sensor ");
            SERIALCONSOLE.println("  ");
          }
        */
      menuload = 1;
      incomingByte = 'c';
      break;

    default:
      // if nothing else matches, do the default
      // default is optional
      break;
    }
  }

  if (menuload == 9)
  {
    switch (incomingByte)
    {
    case '1':
      menuload = 1;
      settings.SerialCan = !settings.SerialCan;
      if (settings.SerialCan > 1)
      {
        settings.SerialCan = 0;
      }
      incomingByte = 'x';
      break;

    case '2':
      menuload = 1;
      if (Serial.available() > 0)
      {
        SetSerialCan(Serial.parseInt());
      }
      incomingByte = 'x';
      break;

    case '3':
      if (Serial.available() > 0)
      {
        settings.DCDCreq = Serial.parseInt();
        menuload = 1;
        incomingByte = 'x';
      }
      break;

    case '4':
      menuload = 1;
      if (Serial.available() > 0)
      {
        SetSerialBaud(Serial.parseInt() * 100);
      }
      incomingByte = 'x';
      break;

    case 113: //q to go back to main menu
      menuload = 0;
      incomingByte = 115;
      break;
    }
  }

  if (menuload == 8)
  {
    switch (incomingByte)
    {
    case '1': //e dispaly settings
      if (Serial.available() > 0)
      {
        settings.IgnoreTemp = Serial.parseInt();
      }
      if (settings.IgnoreTemp > 2)
      {
        settings.IgnoreTemp = 0;
      }
      bms.setSensors(settings.IgnoreTemp, settings.IgnoreVolt);
      menuload = 1;
      incomingByte = 'i';
      break;

    case '2':
      if (Serial.available() > 0)
      {
        settings.IgnoreVolt = Serial.parseInt();
        settings.IgnoreVolt = settings.IgnoreVolt * 0.001;
        bms.setSensors(settings.IgnoreTemp, settings.IgnoreVolt);
        // Serial.println(settings.IgnoreVolt);
        menuload = 1;
        incomingByte = 'i';
      }
      break;

    case 113: //q to go back to main menu

      menuload = 0;
      incomingByte = 115;
      break;
    }
  }

  if (menuload == 7)
  {
    switch (incomingByte)
    {
    case '1':
      if (Serial.available() > 0)
      {
        settings.WarnOff = Serial.parseInt();
        settings.WarnOff = settings.WarnOff * 0.001;
        menuload = 1;
        incomingByte = 'a';
      }
      break;

    case '2':
      if (Serial.available() > 0)
      {
        settings.CellGap = Serial.parseInt();
        settings.CellGap = settings.CellGap * 0.001;
        menuload = 1;
        incomingByte = 'a';
      }
      break;

    case '3':
      if (Serial.available() > 0)
      {
        settings.WarnToff = Serial.parseInt();
        menuload = 1;
        incomingByte = 'a';
      }
      break;

    case '4':
      if (Serial.available() > 0)
      {
        settings.UnderDur = Serial.parseInt();
        menuload = 1;
        incomingByte = 'a';
      }
      break;

    case 113: //q to go back to main menu
      menuload = 0;
      incomingByte = 115;
      break;
    }
  }

  if (menuload == 6) //Charging settings
  {
    switch (incomingByte)
    {

    case 113: //q to go back to main menu

      menuload = 0;
      incomingByte = 115;
      break;

    case '1':
      if (Serial.available() > 0)
      {
        settings.ChargeVsetpoint = Serial.parseInt();
        settings.ChargeVsetpoint = settings.ChargeVsetpoint / 1000;
        menuload = 1;
        incomingByte = 'e';
      }
      break;

    case '2':
      if (Serial.available() > 0)
      {
        settings.ChargeHys = Serial.parseInt();
        settings.ChargeHys = settings.ChargeHys / 1000;
        menuload = 1;
        incomingByte = 'e';
      }
      break;

    case '4':
      if (Serial.available() > 0)
      {
        settings.chargecurrentend = Serial.parseInt() * 10;
        menuload = 1;
        incomingByte = 'e';
      }
      break;

    case '3':
      if (Serial.available() > 0)
      {
        settings.chargecurrentmax = Serial.parseInt() * 10;
        menuload = 1;
        incomingByte = 'e';
      }
      break;

    case '5': //1 Over Voltage Setpoint
      settings.chargertype = settings.chargertype + 1;
      if (settings.chargertype > 6)
      {
        settings.chargertype = 0;
      }
      menuload = 1;
      incomingByte = 'e';
      break;

    case '6':
      if (Serial.available() > 0)
      {
        settings.chargerspd = Serial.parseInt();
        menuload = 1;
        incomingByte = 'e';
      }
      break;

    case '7':
      if (settings.ChargerDirect == 1)
      {
        settings.ChargerDirect = 0;
      }
      else
      {
        settings.ChargerDirect = 1;
      }
      menuload = 1;
      incomingByte = 'e';
      break;
    }
  }

  if (menuload == 5)
  {
    switch (incomingByte)
    {
    case '1':
      if (Serial.available() > 0)
      {
        settings.Pretime = Serial.parseInt();
        menuload = 1;
        incomingByte = 'k';
      }
      break;

    case '2':
      if (Serial.available() > 0)
      {
        settings.Precurrent = Serial.parseInt();
        menuload = 1;
        incomingByte = 'k';
      }
      break;

    case '3':
      if (Serial.available() > 0)
      {
        settings.conthold = Serial.parseInt();
        menuload = 1;
        incomingByte = 'k';
      }
      break;

    case '4':
      if (Serial.available() > 0)
      {
        settings.gaugelow = Serial.parseInt();
        gaugedebug = 2;
        gaugeupdate();
        menuload = 1;
        incomingByte = 'k';
      }
      break;

    case '5':
      if (Serial.available() > 0)
      {
        settings.gaugehigh = Serial.parseInt();
        gaugedebug = 3;
        gaugeupdate();
        menuload = 1;
        incomingByte = 'k';
      }
      break;

    case 113: //q to go back to main menu
      gaugedebug = 0;
      menuload = 0;
      incomingByte = 115;
      break;
    }
  }

  if (menuload == 3)
  {
    switch (incomingByte)
    {
    case 113: //q to go back to main menu

      menuload = 0;
      incomingByte = 115;
      break;

    case 'f': //f factory settings
      loadSettings();
      SERIALCONSOLE.println("  ");
      SERIALCONSOLE.println("  ");
      SERIALCONSOLE.println("  ");
      SERIALCONSOLE.println(" Coded Settings Loaded ");
      SERIALCONSOLE.println("  ");
      menuload = 1;
      incomingByte = 'b';
      break;

    case 114: //r for reset
      SOCset = 0;
      SERIALCONSOLE.println("  ");
      SERIALCONSOLE.print(" mAh Reset ");
      SERIALCONSOLE.println("  ");
      menuload = 1;
      incomingByte = 'b';
      break;

    case '1': //1 Over Voltage Setpoint
      if (Serial.available() > 0)
      {
        settings.OverVSetpoint = Serial.parseInt();
        settings.OverVSetpoint = settings.OverVSetpoint / 1000;
        menuload = 1;
        incomingByte = 'b';
      }
      break;

    case 'g':
      if (Serial.available() > 0)
      {
        settings.StoreVsetpoint = Serial.parseInt();
        settings.StoreVsetpoint = settings.StoreVsetpoint / 1000;
        menuload = 1;
        incomingByte = 'b';
      }

    case 'h':
      if (Serial.available() > 0)
      {
        settings.DisTaper = Serial.parseInt();
        settings.DisTaper = settings.DisTaper / 1000;
        menuload = 1;
        incomingByte = 'b';
      }

    case 'b':
      if (Serial.available() > 0)
      {
        settings.socvolt[0] = Serial.parseInt();
        menuload = 1;
        incomingByte = 'b';
      }
      break;

    case 'c':
      if (Serial.available() > 0)
      {
        settings.socvolt[1] = Serial.parseInt();
        menuload = 1;
        incomingByte = 'b';
      }
      break;

    case 'd':
      if (Serial.available() > 0)
      {
        settings.socvolt[2] = Serial.parseInt();
        menuload = 1;
        incomingByte = 'b';
      }
      break;

    case 'e':
      if (Serial.available() > 0)
      {
        settings.socvolt[3] = Serial.parseInt();
        menuload = 1;
        incomingByte = 'b';
      }
      break;

    case '9': //Discharge Voltage Setpoint
      if (Serial.available() > 0)
      {
        settings.DischVsetpoint = Serial.parseInt();
        settings.DischVsetpoint = settings.DischVsetpoint / 1000;
        menuload = 1;
        incomingByte = 'b';
      }
      break;

    case '0': //c Pstrings
      if (Serial.available() > 0)
      {
        settings.Pstrings = Serial.parseInt();
        menuload = 1;
        incomingByte = 'b';
        bms.setPstrings(settings.Pstrings);
      }
      break;

    case 'a': //
      if (Serial.available() > 0)
      {
        settings.Scells = Serial.parseInt();
        menuload = 1;
        incomingByte = 'b';
      }
      break;

    case '2': //2 Under Voltage Setpoint
      if (Serial.available() > 0)
      {
        settings.UnderVSetpoint = Serial.parseInt();
        settings.UnderVSetpoint = settings.UnderVSetpoint / 1000;
        menuload = 1;
        incomingByte = 'b';
      }
      break;

    case '3': //3 Over Temperature Setpoint
      if (Serial.available() > 0)
      {
        settings.OverTSetpoint = Serial.parseInt();
        menuload = 1;
        incomingByte = 'b';
      }
      break;

    case '4': //4 Udner Temperature Setpoint
      if (Serial.available() > 0)
      {
        settings.UnderTSetpoint = Serial.parseInt();
        menuload = 1;
        incomingByte = 'b';
      }
      break;

    case '5': //5 Balance Voltage Setpoint
      if (Serial.available() > 0)
      {
        settings.balanceVoltage = Serial.parseInt();
        settings.balanceVoltage = settings.balanceVoltage / 1000;
        menuload = 1;
        incomingByte = 'b';
      }
      break;

    case '6': //6 Balance Voltage Hystersis
      if (Serial.available() > 0)
      {
        settings.balanceHyst = Serial.parseInt();
        settings.balanceHyst = settings.balanceHyst / 1000;
        menuload = 1;
        incomingByte = 'b';
      }
      break;

    case '7': //7 Battery Capacity inAh
      if (Serial.available() > 0)
      {
        settings.CAP = Serial.parseInt();
        menuload = 1;
        incomingByte = 'b';
      }
      break;

    case '8': // discurrent in A
      if (Serial.available() > 0)
      {
        settings.discurrentmax = Serial.parseInt() * 10;
        menuload = 1;
        incomingByte = 'b';
      }
      break;
    }
  }

  if (menuload == 1)
  {
    switch (incomingByte)
    {
    case 'R': //restart
      CPU_REBOOT;
      break;

    case 'x': //Expansion Settings
      while (Serial.available())
      {
        Serial.read();
      }
      SERIALCONSOLE.println();
      SERIALCONSOLE.println();
      SERIALCONSOLE.println();
      SERIALCONSOLE.println();
      SERIALCONSOLE.println();
      SERIALCONSOLE.println("Expansion Settings");
      SERIALCONSOLE.println();
      SERIALCONSOLE.println();
      SERIALCONSOLE.print("1 - Serial Port Function:");
      if (settings.SerialCan == 0)
      {
        SERIALCONSOLE.println("None");
      }
      else
      {
        SERIALCONSOLE.println("Can Bus Expansion");
        SERIALCONSOLE.print("2 - Serial Can Speed:");
        SERIALCONSOLE.print(settings.SerialCanSpeed);
        SERIALCONSOLE.println(" kbps");
        SERIALCONSOLE.print("3 - Volt DCDC request:");
        SERIALCONSOLE.print(settings.DCDCreq * 0.1, 1);
        SERIALCONSOLE.println(" V");
        SERIALCONSOLE.print("4 - Serial Baud Rate ");
        SERIALCONSOLE.print(settings.SerialCanBaud);
        SERIALCONSOLE.println(" kbps");
      }
      SERIALCONSOLE.println("q - Go back to menu");
      menuload = 9;
      break;

    case 'i': //Ignore Value Settings
      while (Serial.available())
      {
        Serial.read();
      }
      SERIALCONSOLE.println();
      SERIALCONSOLE.println();
      SERIALCONSOLE.println();
      SERIALCONSOLE.println();
      SERIALCONSOLE.println();
      SERIALCONSOLE.println("Ignore Value Settings");
      SERIALCONSOLE.print("1 - Temp Sensor Setting:");
      SERIALCONSOLE.println(settings.IgnoreTemp);
      SERIALCONSOLE.print("2 - Voltage Under Which To Ignore Cells:");
      SERIALCONSOLE.print(settings.IgnoreVolt * 1000, 0);
      SERIALCONSOLE.println("mV");
      SERIALCONSOLE.println("q - Go back to menu");
      menuload = 8;
      break;

    case 'e': //Charging settings
      while (Serial.available())
      {
        Serial.read();
      }
      SERIALCONSOLE.println();
      SERIALCONSOLE.println();
      SERIALCONSOLE.println();
      SERIALCONSOLE.println();
      SERIALCONSOLE.println();
      SERIALCONSOLE.println("Charging Settings");
      SERIALCONSOLE.print("1 - Cell Charge Voltage Limit Setpoint: ");
      SERIALCONSOLE.print(settings.ChargeVsetpoint * 1000, 0);
      SERIALCONSOLE.println("mV");
      SERIALCONSOLE.print("2 - Charge Hystersis: ");
      SERIALCONSOLE.print(settings.ChargeHys * 1000, 0);
      SERIALCONSOLE.println("mV");
      if (settings.chargertype > 0)
      {
        SERIALCONSOLE.print("3 - Pack Max Charge Current: ");
        SERIALCONSOLE.print(settings.chargecurrentmax * 0.1);
        SERIALCONSOLE.println("A");
        SERIALCONSOLE.print("4- Pack End of Charge Current: ");
        SERIALCONSOLE.print(settings.chargecurrentend * 0.1);
        SERIALCONSOLE.println("A");
      }
      SERIALCONSOLE.print("5- Charger Type: ");
      switch (settings.chargertype)
      {
      case 0:
        SERIALCONSOLE.print("Relay Control");
        break;
      case 1:
        SERIALCONSOLE.print("Brusa NLG5xx");
        break;
      case 2:
        SERIALCONSOLE.print("Volt Charger");
        break;
      case 3:
        SERIALCONSOLE.print("Eltek Charger");
        break;
      case 4:
        SERIALCONSOLE.print("Elcon Charger");
        break;
      case 5:
        SERIALCONSOLE.print("Victron/SMA");
        break;
      case 6:
        SERIALCONSOLE.print("Coda");
        break;
      }
      SERIALCONSOLE.println();
      if (settings.chargertype > 0)
      {
        SERIALCONSOLE.print("6- Charger Can Msg Spd: ");
        SERIALCONSOLE.print(settings.chargerspd);
        SERIALCONSOLE.println("mS");
        SERIALCONSOLE.println();
      }
      /*
          SERIALCONSOLE.print("7- Can Speed:");
          SERIALCONSOLE.print(settings.canSpeed/1000);
          SERIALCONSOLE.println("kbps");
        */
      SERIALCONSOLE.print("7 - Charger HV Connection: ");
      switch (settings.ChargerDirect)
      {
      case 0:
        SERIALCONSOLE.print(" Behind Contactors");
        break;
      case 1:
        SERIALCONSOLE.print("Direct To Battery HV");
        break;
      }
      SERIALCONSOLE.println();
      SERIALCONSOLE.println("q - Go back to menu");
      menuload = 6;
      break;

    case 'a': //Alarm and Warning settings
      while (Serial.available())
      {
        Serial.read();
      }
      SERIALCONSOLE.println();
      SERIALCONSOLE.println();
      SERIALCONSOLE.println();
      SERIALCONSOLE.println();
      SERIALCONSOLE.println();
      SERIALCONSOLE.println("Alarm and Warning Settings Menu");
      SERIALCONSOLE.print("1 - Voltage Warning Offset: ");
      SERIALCONSOLE.print(settings.WarnOff * 1000, 0);
      SERIALCONSOLE.println("mV");
      SERIALCONSOLE.print("2 - Cell Voltage Difference Alarm: ");
      SERIALCONSOLE.print(settings.CellGap * 1000, 0);
      SERIALCONSOLE.println("mV");
      SERIALCONSOLE.print("3 - Temp Warning Offset: ");
      SERIALCONSOLE.print(settings.WarnToff);
      SERIALCONSOLE.println(" C");
      SERIALCONSOLE.print("4 - Temp Warning Offset: ");
      SERIALCONSOLE.print(settings.UnderDur);
      SERIALCONSOLE.println(" mS");
      menuload = 7;
      break;

    case 'k': //contactor settings
      while (Serial.available())
      {
        Serial.read();
      }
      SERIALCONSOLE.println();
      SERIALCONSOLE.println();
      SERIALCONSOLE.println();
      SERIALCONSOLE.println();
      SERIALCONSOLE.println();
      SERIALCONSOLE.println("Contactor and Gauge Settings Menu");
      SERIALCONSOLE.print("1 - PreCharge Timer: ");
      SERIALCONSOLE.print(settings.Pretime);
      SERIALCONSOLE.println("mS");
      SERIALCONSOLE.print("2 - PreCharge Finish Current: ");
      SERIALCONSOLE.print(settings.Precurrent);
      SERIALCONSOLE.println(" mA");
      SERIALCONSOLE.print("3 - PWM contactor Hold 0-255 :");
      SERIALCONSOLE.println(settings.conthold);
      SERIALCONSOLE.print("4 - PWM for Gauge Low 0-255 :");
      SERIALCONSOLE.println(settings.gaugelow);
      SERIALCONSOLE.print("5 - PWM for Gauge High 0-255 :");
      SERIALCONSOLE.println(settings.gaugehigh);
      menuload = 5;
      break;

    case 113:                  //q to go back to main menu
      EEPROM.put(0, settings); //save all change to eeprom
      menuload = 0;
      debug = 1;
      break;
    case 'd': //d for debug settings
      while (Serial.available())
      {
        Serial.read();
      }
      SERIALCONSOLE.println();
      SERIALCONSOLE.println();
      SERIALCONSOLE.println();
      SERIALCONSOLE.println();
      SERIALCONSOLE.println();
      SERIALCONSOLE.println("Debug Settings Menu");
      SERIALCONSOLE.println("Toggle on/off");
      SERIALCONSOLE.print("1 - Can Debug :");
      SERIALCONSOLE.println(candebug);
      SERIALCONSOLE.print("2 - Current Debug :");
      SERIALCONSOLE.println(debugCur);
      SERIALCONSOLE.print("3 - Output Check :");
      SERIALCONSOLE.println(outputcheck);
      SERIALCONSOLE.print("4 - Input Check :");
      SERIALCONSOLE.println(inputcheck);
      SERIALCONSOLE.print("5 - ESS mode :");
      SERIALCONSOLE.println(settings.ESSmode);
      SERIALCONSOLE.print("6 - Cells Present Reset :");
      SERIALCONSOLE.println(cellspresent);
      SERIALCONSOLE.print("7 - Gauge Debug :");
      SERIALCONSOLE.println(gaugedebug);
      SERIALCONSOLE.print("8 - CSV Output :");
      SERIALCONSOLE.println(CSVdebug);
      SERIALCONSOLE.print("9 - Decimal Places to Show :");
      SERIALCONSOLE.println(debugdigits);
      SERIALCONSOLE.print("0 - Display or Serial Mirror :");
      if (settings.disp == 1)
      {
        SERIALCONSOLE.println(" Display Data");
      }
      else
      {
        SERIALCONSOLE.println(" Serial Mirror");
      }
      SERIALCONSOLE.print("a - Can Debug Serial:");
      SERIALCONSOLE.println(CanDebugSerial);
      SERIALCONSOLE.println("q - Go back to menu");
      menuload = 4;
      break;

    case 99: //c for calibrate zero offset
      while (Serial.available())
      {
        Serial.read();
      }
      SERIALCONSOLE.println();
      SERIALCONSOLE.println();
      SERIALCONSOLE.println();
      SERIALCONSOLE.println();
      SERIALCONSOLE.println();
      SERIALCONSOLE.println("Current Sensor Calibration Menu");
      SERIALCONSOLE.println("c - To calibrate sensor offset");
      SERIALCONSOLE.print("s - Current Sensor Type : ");
      switch (settings.cursens)
      {
      case Analoguedual:
        SERIALCONSOLE.println(" Analogue Dual Current Sensor ");
        break;
      case Analoguesing:
        SERIALCONSOLE.println(" Analogue Single Current Sensor ");
        break;
      case Canbus:
        SERIALCONSOLE.println(" Canbus Current Sensor ");
        break;
      default:
        SERIALCONSOLE.println("Undefined");
        break;
      }
      SERIALCONSOLE.print("1 - invert current :");
      SERIALCONSOLE.println(settings.invertcur);
      SERIALCONSOLE.print("2 - Pure Voltage based SOC :");
      SERIALCONSOLE.println(settings.voltsoc);
      SERIALCONSOLE.print("3 - Current Multiplication :");
      SERIALCONSOLE.println(settings.ncur);
      if (settings.cursens == Analoguesing || settings.cursens == Analoguedual)
      {
        SERIALCONSOLE.print("4 - Analogue Low Range Conv:");
        SERIALCONSOLE.print(settings.convlow * 0.1, 1);
        SERIALCONSOLE.println(" mV/A");
      }
      if (settings.cursens == Analoguedual)
      {
        SERIALCONSOLE.print("5 - Analogue High Range Conv:");
        SERIALCONSOLE.print(settings.convhigh * 0.1, 1);
        SERIALCONSOLE.println(" mV/A");
      }
      if (settings.cursens == Analoguesing || settings.cursens == Analoguedual)
      {
        SERIALCONSOLE.print("6 - Current Sensor Deadband:");
        SERIALCONSOLE.print(settings.CurDead);
        SERIALCONSOLE.println(" mV");
      }
      if (settings.cursens == Analoguedual)
      {

        SERIALCONSOLE.print("8 - Current Channel ChangeOver:");
        SERIALCONSOLE.print(settings.changecur * 0.001);
        SERIALCONSOLE.println(" A");
      }

      SERIALCONSOLE.println("q - Go back to menu");
      menuload = 2;
      break;

    case 98: //c for calibrate zero offset
      while (Serial.available())
      {
        Serial.read();
      }
      SERIALCONSOLE.println();
      SERIALCONSOLE.println();
      SERIALCONSOLE.println();
      SERIALCONSOLE.println();
      SERIALCONSOLE.println();
      SERIALCONSOLE.println("Battery Settings Menu");
      SERIALCONSOLE.println("r - Reset AH counter");
      SERIALCONSOLE.println("f - Reset to Coded Settings");
      SERIALCONSOLE.println("q - Go back to menu");
      SERIALCONSOLE.println();
      SERIALCONSOLE.println();
      SERIALCONSOLE.print("1 - Cell Over Voltage Setpoint: ");
      SERIALCONSOLE.print(settings.OverVSetpoint * 1000, 0);
      SERIALCONSOLE.print("mV");
      SERIALCONSOLE.println("  ");
      SERIALCONSOLE.print("2 - Cell Under Voltage Setpoint: ");
      SERIALCONSOLE.print(settings.UnderVSetpoint * 1000, 0);
      SERIALCONSOLE.print("mV");
      SERIALCONSOLE.println("  ");
      SERIALCONSOLE.print("3 - Over Temperature Setpoint: ");
      SERIALCONSOLE.print(settings.OverTSetpoint);
      SERIALCONSOLE.print("C");
      SERIALCONSOLE.println("  ");
      SERIALCONSOLE.print("4 - Under Temperature Setpoint: ");
      SERIALCONSOLE.print(settings.UnderTSetpoint);
      SERIALCONSOLE.print("C");
      SERIALCONSOLE.println("  ");
      SERIALCONSOLE.print("5 - Cell Balance Voltage Setpoint: ");
      SERIALCONSOLE.print(settings.balanceVoltage * 1000, 0);
      SERIALCONSOLE.print("mV");
      SERIALCONSOLE.println("  ");
      SERIALCONSOLE.print("6 - Balance Voltage Hystersis: ");
      SERIALCONSOLE.print(settings.balanceHyst * 1000, 0);
      SERIALCONSOLE.print("mV");
      SERIALCONSOLE.println("  ");
      SERIALCONSOLE.print("7 - Ah Battery Capacity: ");
      SERIALCONSOLE.print(settings.CAP);
      SERIALCONSOLE.print("Ah");
      SERIALCONSOLE.println("  ");
      SERIALCONSOLE.print("8 - Pack Max Discharge: ");
      SERIALCONSOLE.print(settings.discurrentmax * 0.1);
      SERIALCONSOLE.print("A");
      SERIALCONSOLE.println("  ");
      SERIALCONSOLE.print("9 - Cell Discharge Voltage Limit Setpoint: ");
      SERIALCONSOLE.print(settings.DischVsetpoint * 1000, 0);
      SERIALCONSOLE.print("mV");
      SERIALCONSOLE.println("  ");
      SERIALCONSOLE.print("0 - Slave strings in parallel: ");
      SERIALCONSOLE.print(settings.Pstrings);
      SERIALCONSOLE.println("  ");
      SERIALCONSOLE.print("a - Cells in Series per String: ");
      SERIALCONSOLE.print(settings.Scells);
      SERIALCONSOLE.println("  ");
      SERIALCONSOLE.print("b - setpoint 1: ");
      SERIALCONSOLE.print(settings.socvolt[0]);
      SERIALCONSOLE.print("mV");
      SERIALCONSOLE.println("  ");
      SERIALCONSOLE.print("c - SOC setpoint 1:");
      SERIALCONSOLE.print(settings.socvolt[1]);
      SERIALCONSOLE.print("%");
      SERIALCONSOLE.println("  ");
      SERIALCONSOLE.print("d - setpoint 2: ");
      SERIALCONSOLE.print(settings.socvolt[2]);
      SERIALCONSOLE.print("mV");
      SERIALCONSOLE.println("  ");
      SERIALCONSOLE.print("e - SOC setpoint 2: ");
      SERIALCONSOLE.print(settings.socvolt[3]);
      SERIALCONSOLE.print("%");
      SERIALCONSOLE.println("  ");
      SERIALCONSOLE.print("g - Storage Setpoint: ");
      SERIALCONSOLE.print(settings.StoreVsetpoint * 1000, 0);
      SERIALCONSOLE.print("mV");
      SERIALCONSOLE.println("  ");
      SERIALCONSOLE.print("h - Discharge Current Taper Offset: ");
      SERIALCONSOLE.print(settings.DisTaper * 1000, 0);
      SERIALCONSOLE.print("mV");
      SERIALCONSOLE.println("  ");

      SERIALCONSOLE.println();
      menuload = 3;
      break;

    default:
      // if nothing else matches, do the default
      // default is optional
      break;
    }
  }

  if (incomingByte == 115 & menuload == 0)
  {
    SERIALCONSOLE.println();
    SERIALCONSOLE.println("MENU");
    SERIALCONSOLE.println("Debugging Paused");
    SERIALCONSOLE.print("Firmware Version : ");
    SERIALCONSOLE.println(firmver);
    SERIALCONSOLE.println("b - Battery Settings");
    SERIALCONSOLE.println("a - Alarm and Warning Settings");
    SERIALCONSOLE.println("e - Charging Settings");
    SERIALCONSOLE.println("c - Current Sensor Calibration");
    SERIALCONSOLE.println("k - Contactor and Gauge Settings");
    SERIALCONSOLE.println("i - Ignore Value Settings");
    SERIALCONSOLE.println("x - Expansion Settings");
    SERIALCONSOLE.println("d - Debug Settings");
    SERIALCONSOLE.println("R - Restart BMS");
    SERIALCONSOLE.println("q - exit menu");
    debug = 0;
    menuload = 1;
  }
}

void canread()
{
  Can0.read(inMsg);
  // Read data: len = data length, buf = data byte(s)
  switch (inMsg.id)
  {
  case 0x3c2:
    CAB300();
    break;

  default:
    break;
  }

  if (inMsg.id >= 0x460 && inMsg.id < 0x480) //do volt magic if ids are ones identified to be modules
  {
    //DISABLE debugging otherwise message ids take over window
    //Serial.println(inMsg.id, HEX);
    bms.decodecan(inMsg); //do volt magic if ids are ones identified to be modules
  }
  if (inMsg.id >= 0x7E0 && inMsg.id < 0x7F0) //do volt magic if ids are ones identified to be modules
  {
    bms.decodecan(inMsg); //do volt magic if ids are ones identified to be modules
  }
  if (debug == 1)
  {
    if (candebug == 1)
    {
      Serial.print(millis());
      if ((inMsg.id & 0x80000000) == 0x80000000) // Determine if ID is standard (11 bits) or extended (29 bits)
        sprintf(msgString, "Extended ID: 0x%.8lX  DLC: %1d  Data:", (inMsg.id & 0x1FFFFFFF), inMsg.len);
      else
        sprintf(msgString, ",0x%.3lX,false,%1d", inMsg.id, inMsg.len);

      Serial.print(msgString);

      if ((inMsg.id & 0x40000000) == 0x40000000)
      { // Determine if message is a remote request frame.
        sprintf(msgString, " REMOTE REQUEST FRAME");
        Serial.print(msgString);
      }
      else
      {
        for (byte i = 0; i < inMsg.len; i++)
        {
          sprintf(msgString, ", 0x%.2X", inMsg.buf[i]);
          Serial.print(msgString);
        }
      }

      Serial.println();
    }
  }
}

void CAB300()
{
  for (int i = 0; i < 4; i++)
  {
    inbox = (inbox << 8) | inMsg.buf[i];
  }
  CANmilliamps = inbox;
  if (CANmilliamps > 0x80000000)
  {
    CANmilliamps -= 0x80000000;
  }
  else
  {
    CANmilliamps = (0x80000000 - CANmilliamps) * -1;
  }
  if (settings.cursens == Canbus)
  {
    RawCur = CANmilliamps;
    getcurrent();
  }
  if (candebug == 1)
  {
    Serial.println();
    Serial.print(CANmilliamps);
    Serial.print("mA ");
  }
}

float getChargeVSetpoint()
{
  return settings.ChargeVsetpoint * (maxchargepercent / 100);
}

void currentlimit()
{
  if (bmsstatus == Error)
  {
    discurrent = 0;
    chargecurrent = 0;
  }
  /*
    settings.PulseCh = 600; //Peak Charge current in 0.1A
    settings.PulseChDur = 5000; //Ms of discharge pulse derating
    settings.PulseDi = 600; //Peak Charge current in 0.1A
    settings.PulseDiDur = 5000; //Ms of discharge pulse derating
  */
  else
  {

    ///Start at no derating///
    discurrent = settings.discurrentmax;
    chargecurrent = settings.chargecurrentmax;

    ///////All hard limits to into zeros
    if (bms.getLowTemperature() < settings.UnderTSetpoint)
    {
      //discurrent = 0; Request Daniel
      chargecurrent = 0;
    }
    if (bms.getHighTemperature() > settings.OverTSetpoint)
    {
      discurrent = 0;
      chargecurrent = 0;
    }
    if (bms.getHighCellVolt() > settings.OverVSetpoint)
    {
      chargecurrent = 0;
    }
    if (bms.getHighCellVolt() > settings.OverVSetpoint)
    {
      chargecurrent = 0;
    }
    if (bms.getLowCellVolt() < settings.UnderVSetpoint || bms.getLowCellVolt() < settings.DischVsetpoint)
    {
      discurrent = 0;
    }

    //Modifying discharge current///

    if (discurrent > 0)
    {
      //Temperature based///

      if (bms.getLowTemperature() > settings.DisTSetpoint)
      {
        discurrent = discurrent - map(bms.getLowTemperature(), settings.DisTSetpoint, settings.OverTSetpoint, 0, settings.discurrentmax);
      }
      //Voltagee based///
      if (bms.getLowCellVolt() > settings.UnderVSetpoint || bms.getLowCellVolt() > settings.DischVsetpoint)
      {
        if (bms.getLowCellVolt() < (settings.DischVsetpoint + settings.DisTaper))
        {
          discurrent = discurrent - map(bms.getLowCellVolt(), settings.DischVsetpoint, (settings.DischVsetpoint + settings.DisTaper), settings.discurrentmax, 0);
        }
      }
    }

    //Modifying Charge current///
    if (chargecurrent > 0)
    {
      //Temperature based///
      if (bms.getHighTemperature() < settings.ChargeTSetpoint)
      {
        chargecurrent = chargecurrent - map(bms.getHighTemperature(), settings.UnderTSetpoint, settings.ChargeTSetpoint, settings.chargecurrentmax, 0);
      }
      //Voltagee based///
      if (storagemode == 1)
      {
        if (bms.getHighCellVolt() > (settings.StoreVsetpoint - settings.ChargeHys))
        {
          chargecurrent = chargecurrent - map(bms.getHighCellVolt(), (settings.StoreVsetpoint - settings.ChargeHys), settings.StoreVsetpoint, settings.chargecurrentend, settings.chargecurrentmax);
        }
      }
      else
      {
        if (bms.getHighCellVolt() > (settings.ChargeVsetpoint - settings.ChargeHys))
        {
          chargecurrent = chargecurrent - map(bms.getHighCellVolt(), (settings.ChargeVsetpoint - settings.ChargeHys), settings.ChargeVsetpoint, settings.chargecurrentend, settings.chargecurrentmax);
        }
      }
    }
  }
  ///No negative currents///

  if (discurrent < 0)
  {
    discurrent = 0;
  }
  if (chargecurrent < 0)
  {
    chargecurrent = 0;
  }
}

void inputdebug()
{
  Serial.println();
  Serial.print("Input: ");
  if (digitalRead(IN1))
  {
    Serial.print("1 ON  ");
  }
  else
  {
    Serial.print("1 OFF ");
  }
  if (digitalRead(IN3))
  {
    Serial.print("2 ON  ");
  }
  else
  {
    Serial.print("2 OFF ");
  }
  if (digitalRead(IN3))
  {
    Serial.print("3 ON  ");
  }
  else
  {
    Serial.print("3 OFF ");
  }
  if (digitalRead(IN4))
  {
    Serial.print("4 ON  ");
  }
  else
  {
    Serial.print("4 OFF ");
  }
  Serial.println();
}

void outputdebug()
{
  if (outputstate < 5)
  {
    digitalWrite(OUT1, HIGH);
    digitalWrite(OUT2, HIGH);
    digitalWrite(OUT3, HIGH);
    digitalWrite(OUT4, HIGH);
    analogWrite(OUT5, 255);
    analogWrite(OUT6, 255);
    analogWrite(OUT7, 255);
    analogWrite(OUT8, 255);
    outputstate++;
  }
  else
  {
    digitalWrite(OUT1, LOW);
    digitalWrite(OUT2, LOW);
    digitalWrite(OUT3, LOW);
    digitalWrite(OUT4, LOW);
    analogWrite(OUT5, 0);
    analogWrite(OUT6, 0);
    analogWrite(OUT7, 0);
    analogWrite(OUT8, 0);
    outputstate++;
  }
  if (outputstate > 10)
  {
    outputstate = 0;
  }
}

void sendcommand() // send BICM trigger message
{
  msg.id = 0x200;
  msg.len = 3;
  msg.buf[0] = 0x02;
  msg.buf[1] = 0x00;
  msg.buf[2] = 0x00;
  Can0.write(msg);
}

void resetwdog()
{
  noInterrupts(); //   No - reset WDT
  WDOG_REFRESH = 0xA602;
  WDOG_REFRESH = 0xB480;
  interrupts();
}

void pwmcomms()
{
  int p = 0;
  p = map((currentact * 0.001), pwmcurmin, pwmcurmax, 50, 255);
  analogWrite(OUT7, p);
  /*
    Serial.println();
      Serial.print(p*100/255);
      Serial.print(" OUT8 ");
  */

  if (bms.getLowCellVolt() < settings.UnderVSetpoint)
  {
    analogWrite(OUT7, 255); //12V to 10V converter 1.5V
  }
  else
  {
    p = map(SOC, 0, 100, 220, 50);
    analogWrite(OUT8, p); //2V to 10V converter 1.5-10V
  }
  /*
      Serial.println();
      Serial.print(p);
      Serial.print(" OUT7 ");
  */
}

void dashDisplayListen() // simple protocol from https://seithan.com/nextion-projects/send-values-store-them-on-arduino/
{
  if (Serial2.available() > 2)
  {                                   // Read if more then 2 bytes come (we always send more than 2 <#> <len> <cmd> <id>
    char start_char = Serial2.read(); // Create a local variable (start_char) read and store the first byte on it
    if (start_char == '#')
    {                               // And when we find the character #
      uint8_t len = Serial2.read(); // Create local variable (len) / read and store the value of the second byte
                                    // <len> is the lenght (number of bytes following)
      unsigned long tmr_1 = millis();
      boolean cmd_found = true;

      while (Serial2.available() < len)
      { // Waiting for all the bytes that we declare with <len> to arrive
        if ((millis() - tmr_1) > 100)
        {                    // Waiting... But not forever......
          cmd_found = false; // tmr_1 a timer to avoid the stack in the while loop if there is not any bytes on Serial
          break;
        }
        delay(1); // Delay for nothing delete it if you want
      }

      if (cmd_found == true)
      {                               // So..., A command is found (bytes in Serial buffer egual more than len)
        uint8_t cmd = Serial2.read(); // Create local variable (cmd). Read and store the next byte. This is the command group
        switch (cmd)
        {
        case 0x01: // command to change charge amps
          if (Serial2.read() == 0x00)
            settings.chargecurrentmax -= 10;
          else
            settings.chargecurrentmax += 10;
          // Should we store this to EEPROM? probably overkill
          Serial2.print("champ.val=");
          Serial2.print(settings.chargecurrentmax);
          dashEndCommand();

          break;

        case 0x02: // command to change max charge %
          if (Serial2.read() == 0x00)
          {
            maxchargepercent--;
          }
          else
          {
            maxchargepercent++;
            if (maxchargepercent > 100)
            {
              maxchargepercent = 100;
            }
          }
          Serial2.print("maxch.val=");
          Serial2.print(maxchargepercent);
          dashEndCommand();

          break;

        case 0x03: // command to REBOOT BMS
          if (Serial2.read() == 0x00)
          {
            CPU_REBOOT;
          }

          break;

        case 0x04: // command to RESET AH
          if (Serial2.read() == 0x00)
          {
            SOCset = 0;
          }

          break;
        }
      }
    }
  }
}

void dashEndCommand()
{
  Serial2.write(0xff); // We always have to send this three lines after each command sent to the nextion display.
  Serial2.write(0xff);
  Serial2.write(0xff);
}

void dashupdate()
{
  if (bmsstatus == Ready && !displaySleep) { 
    Serial2.print("sleep=1");
    dashEndCommand();
    displaySleep = true;
  }
  else if (bmsstatus != Ready && displaySleep) {
    Serial2.print("sleep=0");
    dashEndCommand();
    displaySleep = false;
  }

  Serial2.print("page0.bmsstat.val=");
  Serial2.print(bmsstatus);
  dashEndCommand();
  Serial2.print("soc.val=");
  Serial2.print(SOC);
  dashEndCommand();
  Serial2.print("current.val=");
  Serial2.print(currentact / 100, 0);
  dashEndCommand();
  Serial2.print("temp.val=");
  Serial2.print(bms.getAvgTemperature(), 0);
  dashEndCommand();
  Serial2.print("templow.val=");
  Serial2.print(bms.getLowTemperature(), 0);
  dashEndCommand();
  Serial2.print("temphigh.val=");
  Serial2.print(bms.getHighTemperature(), 0);
  dashEndCommand();
  Serial2.print("volt.val=");
  Serial2.print(bms.getPackVoltage() * 10, 0);
  dashEndCommand();
  Serial2.print("lowcell.val=");
  Serial2.print(bms.getLowCellVolt() * 1000, 0);
  dashEndCommand();
  Serial2.print("highcell.val=");
  Serial2.print(bms.getHighCellVolt() * 1000, 0);
  dashEndCommand();
  Serial2.print("celldelta.val=");
  Serial2.print(bms.getHighCellVolt() * 1000 - bms.getLowCellVolt() * 1000, 0);
  dashEndCommand();
  Serial2.print("cellbal.val=");
  Serial2.print(balancecells ? 1 : 0);
  dashEndCommand();
  Serial2.print("firm.val=");
  Serial2.print(firmver);
  dashEndCommand();
  Serial2.print("champ.val=");
  Serial2.print(settings.chargecurrentmax);
  dashEndCommand();
  Serial2.print("maxch.val=");
  Serial2.print(maxchargepercent);
  dashEndCommand();
}

void chargercomms()
{
  if (settings.chargertype == Elcon)
  {
    msg.id = 0x1806E5F4; //broadcast to all Elteks
    msg.len = 8;
    msg.ext = 1;
    msg.buf[0] = highByte(uint16_t(getChargeVSetpoint() * settings.Scells * 10));
    msg.buf[1] = lowByte(uint16_t(getChargeVSetpoint() * settings.Scells * 10));
    msg.buf[2] = highByte(chargecurrent / ncharger);
    msg.buf[3] = lowByte(chargecurrent / ncharger);
    msg.buf[4] = 0x00;
    msg.buf[5] = 0x00;
    msg.buf[6] = 0x00;
    msg.buf[7] = 0x00;

    Can0.write(msg);
    msg.ext = 0;
  }

  if (settings.chargertype == Eltek)
  {
    msg.id = 0x2FF; //broadcast to all Elteks
    msg.len = 7;
    msg.buf[0] = 0x01;
    msg.buf[1] = lowByte(1000);
    msg.buf[2] = highByte(1000);
    msg.buf[3] = lowByte(uint16_t(getChargeVSetpoint() * settings.Scells * 10));
    msg.buf[4] = highByte(uint16_t(getChargeVSetpoint() * settings.Scells * 10));
    msg.buf[5] = lowByte(chargecurrent / ncharger);
    msg.buf[6] = highByte(chargecurrent / ncharger);

    Can0.write(msg);
  }
  if (settings.chargertype == BrusaNLG5)
  {
    msg.id = chargerid1;
    msg.len = 7;
    msg.buf[0] = 0x80;
    /*
      if (chargertoggle == 0)
      {
      msg.buf[0] = 0x80;
      chargertoggle++;
      }
      else
      {
      msg.buf[0] = 0xC0;
      chargertoggle = 0;
      }
    */
    if (digitalRead(IN2) == LOW) //Gen OFF
    {
      msg.buf[1] = highByte(maxac1 * 10);
      msg.buf[2] = lowByte(maxac1 * 10);
    }
    else
    {
      msg.buf[1] = highByte(maxac2 * 10);
      msg.buf[2] = lowByte(maxac2 * 10);
    }
    msg.buf[5] = highByte(chargecurrent / ncharger);
    msg.buf[6] = lowByte(chargecurrent / ncharger);
    msg.buf[3] = highByte(uint16_t(((getChargeVSetpoint() * settings.Scells) - chargerendbulk) * 10));
    msg.buf[4] = lowByte(uint16_t(((getChargeVSetpoint() * settings.Scells) - chargerendbulk) * 10));
    Can0.write(msg);

    delay(2);

    msg.id = chargerid2;
    msg.len = 7;
    msg.buf[0] = 0x80;
    if (digitalRead(IN2) == LOW) //Gen OFF
    {
      msg.buf[1] = highByte(maxac1 * 10);
      msg.buf[2] = lowByte(maxac1 * 10);
    }
    else
    {
      msg.buf[1] = highByte(maxac2 * 10);
      msg.buf[2] = lowByte(maxac2 * 10);
    }
    msg.buf[3] = highByte(uint16_t(((getChargeVSetpoint() * settings.Scells) - chargerend) * 10));
    msg.buf[4] = lowByte(uint16_t(((getChargeVSetpoint() * settings.Scells) - chargerend) * 10));
    msg.buf[5] = highByte(chargecurrent / ncharger);
    msg.buf[6] = lowByte(chargecurrent / ncharger);
    Can0.write(msg);
  }
  if (settings.chargertype == ChevyVolt)
  {
    msg.id = 0x30E;
    msg.len = 1;
    msg.buf[0] = 0x02; //only HV charging , 0x03 hv and 12V charging
    Can0.write(msg);

    msg.id = 0x304;
    msg.len = 4;
    msg.buf[0] = 0x40; //fixed
    if ((chargecurrent * 2) > 255)
    {
      msg.buf[1] = 255;
    }
    else
    {
      msg.buf[1] = (chargecurrent * 2);
    }
    if ((getChargeVSetpoint() * settings.Scells) > 200)
    {
      msg.buf[2] = highByte(uint16_t((getChargeVSetpoint() * settings.Scells) * 2));
      msg.buf[3] = lowByte(uint16_t((getChargeVSetpoint() * settings.Scells) * 2));
    }
    else
    {
      msg.buf[2] = highByte(400);
      msg.buf[3] = lowByte(400);
    }
    Can0.write(msg);
  }
}

void SerialCanRecieve()
{
  if (can.recv(&id, dta))
  {
    if (CanDebugSerial == 1)
    {
      Serial.print("GET DATA FROM ID: ");
      Serial.println(id, HEX);
      for (int i = 0; i < 8; i++)
      {
        //Serial.print("0x");
        Serial.print(dta[i]);
        Serial.print('\t');
      }
      Serial.println();
    }
  }
}

void SetSerialCan(int Speed)
{
  switch (Speed)
  {
  case 500:
    if (can.canRate(CAN_RATE_500))
    {
      Serial.println("set can rate ok");
      settings.SerialCanSpeed = 500;
    }
    else
    {
      Serial.println("set can rate fail");
    }
    break;

  case 250:
    if (can.canRate(CAN_RATE_250))
    {
      Serial.println("set can rate ok");
      settings.SerialCanSpeed = 250;
    }
    else
    {
      Serial.println("set can rate fail");
    }
    break;

  default:
    Serial.println("Wrong CAN Speed");
    // if nothing else matches, do the default
    // default is optional
    break;
  }
}

/*
  value          0     1     2     3     4
  baud rate(b/s)  9600  19200 38400 57600 115200
*/

void SetSerialBaud(uint32_t Speed)
{
  Serial.println(Speed);
  switch (Speed)
  {
  case 9600:
    can.baudRate(0);
    settings.SerialCanBaud = 9600;
    canSerial.flush();
    canSerial.begin(9600);
    can.exitSettingMode();
    break;

  case 19200:
    can.baudRate(1);
    settings.SerialCanBaud = 19200;
    canSerial.flush();
    canSerial.begin(19200);
    can.exitSettingMode();
    break;

  case 38400:
    can.baudRate(2);
    settings.SerialCanBaud = 38400;
    canSerial.flush();
    canSerial.begin(38400);
    can.exitSettingMode();
    break;

  case 115200:
    can.baudRate(4);
    settings.SerialCanBaud = 115200;
    canSerial.flush();
    canSerial.begin(115200);
    can.exitSettingMode();
    break;

  default:
    Serial.println("Wrong Baud Rate");
    // if nothing else matches, do the default
    // default is optional
    break;
  }
}

void CanSerial() //communication with Victron system over CAN
{
  if (bmsstatus == Charge)
  {
    if (settings.chargertype == Elcon)
    {
      if (mescycl == 0)
      {
        dta[0] = highByte(uint16_t(getChargeVSetpoint() * settings.Scells * 10));
        dta[1] = lowByte(uint16_t(getChargeVSetpoint() * settings.Scells * 10));
        dta[2] = highByte(chargecurrent / ncharger);
        dta[3] = lowByte(chargecurrent / ncharger);
        dta[4] = 0x00;
        dta[5] = 0x00;
        dta[6] = 0x00;
        dta[7] = 0x00;

        can.send(0x1806E5F4, 1, 0, 8, dta);
      }
    }

    if (settings.chargertype == Eltek)
    {
      if (mescycl == 0)
      {
        dta[0] = 0x01;
        dta[1] = lowByte(1000);
        dta[2] = highByte(1000);
        dta[3] = lowByte(uint16_t(getChargeVSetpoint() * settings.Scells * 10));
        dta[4] = highByte(uint16_t(getChargeVSetpoint() * settings.Scells * 10));
        dta[5] = lowByte(chargecurrent / ncharger);
        dta[6] = highByte(chargecurrent / ncharger);

        can.send(0x2FF, 0, 0, 7, dta);
      }
    }
    if (settings.chargertype == BrusaNLG5)
    {
      if (mescycl == 0)
      {
        dta[0] = 0x80;
        /*
          if (chargertoggle == 0)
          {
          dta[0] = 0x80;
          chargertoggle++;
          }
          else
          {
          dta[0] = 0xC0;
          chargertoggle = 0;
          }
        */
        if (digitalRead(IN2) == LOW) //Gen OFF
        {
          dta[1] = highByte(maxac1 * 10);
          dta[2] = lowByte(maxac1 * 10);
        }
        else
        {
          dta[1] = highByte(maxac2 * 10);
          dta[2] = lowByte(maxac2 * 10);
        }
        dta[5] = highByte(chargecurrent / ncharger);
        dta[6] = lowByte(chargecurrent / ncharger);
        dta[3] = highByte(uint16_t(((getChargeVSetpoint() * settings.Scells) - chargerendbulk) * 10));
        dta[4] = lowByte(uint16_t(((getChargeVSetpoint() * settings.Scells) - chargerendbulk) * 10));
        can.send(chargerid1, 0, 0, 7, dta);
      }
      if (mescycl == 1)
      {

        dta[0] = 0x80;
        if (digitalRead(IN2) == LOW) //Gen OFF
        {
          dta[1] = highByte(maxac1 * 10);
          dta[2] = lowByte(maxac1 * 10);
        }
        else
        {
          dta[1] = highByte(maxac2 * 10);
          dta[2] = lowByte(maxac2 * 10);
        }
        dta[3] = highByte(uint16_t(((getChargeVSetpoint() * settings.Scells) - chargerend) * 10));
        dta[4] = lowByte(uint16_t(((getChargeVSetpoint() * settings.Scells) - chargerend) * 10));
        dta[5] = highByte(chargecurrent / ncharger);
        dta[6] = lowByte(chargecurrent / ncharger);
        can.send(chargerid2, 0, 0, 7, dta);
      }
    }

    if (settings.chargertype == ChevyVolt)
    {
      if (mescycl == 0)
      {
        dta[0] = 0x02; //only HV charging , 0x03 hv and 12V charging
        dta[1] = 0x00;
        dta[2] = 0x00;
        dta[3] = 0x00;
        can.send(0x30E, 0, 0, 4, dta);
      }
      if (mescycl == 1)
      {
        dta[0] = 0x40; //fixed
        if ((chargecurrent * 2) > 255)
        {
          dta[1] = 255;
        }
        else
        {
          dta[1] = (chargecurrent * 2);
        }
        if ((getChargeVSetpoint() * settings.Scells) > 200)
        {
          dta[2] = highByte(uint16_t((getChargeVSetpoint() * settings.Scells) * 2));
          dta[3] = lowByte(uint16_t((getChargeVSetpoint() * settings.Scells) * 2));
        }
        else
        {
          dta[2] = highByte(400);
          dta[3] = lowByte(400);
        }
        can.send(0x304, 0, 0, 4, dta);
      }
    }

    if (settings.chargertype == Coda)
    {
      // Data is big endian (MSB, LSB)
      // Voltage scaling is value * 10
      msg.id = 0x050;
      msg.len = 8;
      msg.buf[0] = 0x00;
      msg.buf[1] = 0xDC;
      if ((getChargeVSetpoint() * settings.Scells) > 200)
      {
        msg.buf[2] = highByte(uint16_t((getChargeVSetpoint() * settings.Scells) * 10));
        msg.buf[3] = lowByte(uint16_t((getChargeVSetpoint() * settings.Scells) * 10));
      }
      else
      {
        // Voltage minimum is 200V -> 200 * 10 = 2000 -> 0x7D0
        msg.buf[2] = 0x07;
        msg.buf[3] = 0xD0;
      }
      msg.buf[4] = 0x00;
      if ((getChargeVSetpoint() * settings.Scells) * chargecurrent < 3300)
      {
        msg.buf[5] = highByte(uint16_t(((getChargeVSetpoint() * settings.Scells) * chargecurrent) / 240));
        msg.buf[6] = lowByte(uint16_t(((getChargeVSetpoint() * settings.Scells) * chargecurrent) / 240));
      }
      else //15 A AC limit
      {
        msg.buf[5] = 0x00;
        msg.buf[6] = 0x96;
      }
      msg.buf[7] = 0x01; //HV charging
      Can0.write(msg);
    }
  }

  if (mescycl == 2)
  {
    if (settings.DCDCreq > 0)
    {
      dta[0] = 0xA0;
      dta[1] = settings.DCDCreq * 1.27;

      can.send(0x1D4, 0, 0, 2, dta);
    }
  }
  mescycl++;
  if (mescycl > 2)
  {
    mescycl = 0;
  }
}

void btUpdate()
{
  if (settings.disp == 1)
  {
    Serial3.print("SOC=  ");
    Serial3.print(SOC);
    Serial3.println("");
    Serial3.println("");
    Serial3.print("Temp=  ");
    Serial3.print((bms.getAvgTemperature() * 1.8) + 32, 1);
    Serial3.println("");
    Serial3.println("");
    Serial3.print("Pack Volts=  ");
    Serial3.print(bms.getPackVoltage(), 2);
    Serial3.println("");
    Serial3.println("");
    Serial3.print("Low Cell=  ");
    Serial3.print(bms.getLowCellVolt(), 3);
    Serial3.println("");
    Serial3.println("");
    Serial3.print("High Cell=  ");
    Serial3.print(bms.getHighCellVolt(), 3);
    Serial3.println("");
    Serial3.println("");
    Serial3.print("Cell Delta=  ");
    Serial3.println((bms.getHighCellVolt() - bms.getLowCellVolt()), 3);
    Serial3.print("Charger=  ");
    if (digitalRead(OUT8) == HIGH)
    {
      Serial3.print(" ON ");
    }
    else {
      Serial3.print(" OFF ");
    }
    Serial3.println("");
    Serial3.print("Balancing=  ");
    if (balancecells == 1)
    {
      Serial3.print(" ON ");
    }
    else {
      Serial3.print(" OFF ");
    }
    Serial3.println("");
    Serial3.println("--------------------");
    Serial3.println("--------------------");
  }
  else
  {
    Serial3.println(".");
    Serial3.println(".");
    Serial3.println(".");
    Serial3.print("BMS Status : ");


    Serial3.print(bmsstatus);
    switch (bmsstatus)
    {
    case (Boot):
      Serial3.print(" Boot ");
      break;

    case (Ready):
      Serial3.print(" Ready ");
      break;


    case (Drive):
      Serial3.print(" Drive ");
      break;

    case (Charge):
      Serial3.print(" Charge ");
      break;

    case (Error):
      Serial3.print(" Error ");
      break;
    }

    Serial3.print("  ");

    if (digitalRead(IN1) == HIGH)
    {
      Serial3.print("| IN1 ON |");
    }
    if (balancecells == 1)
    {
      Serial3.print("|Balancing Active");
    }
    Serial3.print("  ");
    Serial3.print(cellspresent);
    Serial3.println(".");
    Serial3.print("Out:");
    Serial3.print(digitalRead(OUT8));

    Serial3.print(" In:");
    Serial3.print(digitalRead(IN1));
    bms.printPackDetails(debugdigits, 1);

    // Spacer for BT terminal
    Serial3.println(".");
    Serial3.println(".");
    Serial3.println(".");
    Serial3.println(".");
    Serial3.println(".");

}
}
