#include <Arduino.h>
#include <EEPROM.h>
#include <avr/wdt.h>

#define TEMP_TARGET_LIMIT_PANIC_OFF 99         // when something goes wrong:
#define TEMP_TARGET_LIMIT_PANIC_OFF_DEFAULT 50 // keep it lower since that is the temperature in the chip...

#define TEMP_TARGET_LIMIT_UPPER 80 // ABS Melts at like 80c so it makes sense to limit the maximum temperature to that
#define TEMP_TARGET_LIMIT_LOWER 0
#define TEMP_TARGET_DEFAULT 50
#define FAN_TARGET_LIMIT_UPPER 99
#define FAN_TARGET_LIMIT_LOWER 20 // TODO: find out how low it can go.
#define FAN_TARGET_DEFAULT 99
#define TIME_TARGET_DEFAULT 0 // No time limit
#define IGNORE_TRIGGER_DEFAULT true
#define TRIGGER_TYPE_DEFAULT true // invert per default, 1 when led on
#define TRIGGER_MODE_DEFAULT false

#define AMNT_MENU 13
#define MENU_INTERNAL_TEMP_NTC MP + 0  // NTC Temperature in front of the heater
#define MENU_INTERNAL_TEMP MP + 1      // Chip Temperature
#define MENU_TEMPERATURE_TARGET MP + 2 // Target Temperature which the heater should reach / be under
#define MENU_FAN_TARGET MP + 3         // Fan Speed, + or - changes variable
#define MENU_TIME MP + 4               // Time menu, + or - changes Time (which gets saved in eeprom for next time)
#define MENU_LOCK MP + 5               // Lock all inputs until restart, + triggers.
#define MENU_IGNORE_TRIGGER MP + 6     // Ignore External trigger input, + or - changes variable
#define MENU_SHOW_TRIGGER MP + 7       // Show External trigger input is on
#define MENU_TRIGGER_TYPE MP + 8       // Set External trigger active low or high, + or - changes variable
#define MENU_PANIC_TEMP MP + 9         // Set Panic temperature at which the heater should be turned off and the device needs a restart
#define MENU_RESET_EEPROM MP + 10      // Resets the eeprom on + press
#define MENU_SAVE_EEPROM MP + 11       // Saves values on + press
#define MENU_REMAINING_TIME MP + 12    // Shows remaining time, + or - changes by 1H
#define MENU_TIMER_ON MP + 13          // Shows if timer is on, + or - starts (1) / stops (0)
#define MP 150                         // Menu Prefix number (F)
#define TIME_POWEROFF 5000             // time after which the device should turn off when power on remote held
#define TIME_POWEROFF_DELAY 5000       // time to skip inputs after poweroff

#define FAN 6
#define HEATER_TRIAC 5
#define IR A0
#define SEG_0_COM 15
#define SEG_1_COM 8

#define BUTTON_COM 14
#define BUTTON_NEG G_7S
#define BUTTON_POS B_7S
#define BUTTON_MENU F_7S

#define TEMP_EXT_TRIG 0

#define DECODE_NEC
#include <IRremote.hpp>

#define A_7S 4
#define B_7S 16
#define C_7S LED_BUILTIN_TX // A1 in sch
#define D_7S 9
#define E_7S LED_BUILTIN_RX // A2 in sch
#define F_7S 7
#define G_7S 2

#define NTC A3

byte menu = MP;
int number = 0;

int chiptemp = 0;
int ntctemp = 0;
int targetTemperature = TEMP_TARGET_DEFAULT;
int fanTarget = FAN_TARGET_DEFAULT;

bool ison = true; // keep on by default
bool is_locked = false;

int temp_target_limit_panic = TEMP_TARGET_LIMIT_PANIC_OFF_DEFAULT;
int ignore_trigger = IGNORE_TRIGGER_DEFAULT;
int trigger_type = TRIGGER_TYPE_DEFAULT;
unsigned long time_target = TIME_TARGET_DEFAULT;
unsigned long time_stop = 0;
bool timer_on = false;

void ReceiveCompleteCallbackHandler();
volatile bool sIRDataJustReceived = false;

struct EEPromData // theoretically all variables can be directly used with that struct to save some ram... nevermind, there is still more then enough.
{
  int targetTemperature;
  int fanTarget;
  bool ison;
  int temp_target_limit_panic;
  int ignore_trigger;
  int trigger_type;
  unsigned long time_target;
};
unsigned long crc = ~0L;
EEPromData eepromdata;

float chipTemp(float raw)
{
  const float chipTempOffset = -142.0;
  const float chipTempCoeff = .558;
  return ((raw * chipTempCoeff) + chipTempOffset);
}

int getADC()
{
  ADCSRA |= _BV(ADSC); // start conversion
  while ((ADCSRA & _BV(ADSC)))
    ;                  // Wait until conversion is finished
  ADCSRA |= _BV(ADSC); // start conversion
  while ((ADCSRA & _BV(ADSC)))
    ; // Wait until conversion is finished
  return (ADCL | (ADCH << 8));
}

int ntcTempRaw(void)
{
  ADCSRB = 0;
  ADMUX = _BV(REFS0) | _BV(MUX2); // Set internal V reference, 3
  ADCSRA = _BV(ADEN) | 7;         // Enable AD and start conversion
                                  // ADCSRA &= ~(_BV(ADATE) | _BV(ADIE));
  delayMicroseconds(1000);        // it doesn't seem to like lower values...

  return getADC();
}

int chipTempRaw(void)
{
  ADCSRA = _BV(ADEN) | 7;
  ADMUX = _BV(REFS1) | _BV(REFS0) | 7;
  ADCSRB = 0x20;           // ref  24.6
  delayMicroseconds(1000); // it doesn't seem to like lower values...
  return getADC();
}

int mapfanspeed(byte val)
{
  return map(val, 0, FAN_TARGET_LIMIT_UPPER, 0, 255);
}

unsigned long eeprom_crc(void)
{

  const unsigned long crc_table[16] = {

      0x00000000, 0x1db71064, 0x3b6e20c8, 0x26d930ac,
      0x76dc4190, 0x6b6b51f4, 0x4db26158, 0x5005713c,
      0xedb88320, 0xf00f9344, 0xd6d6a3e8, 0xcb61b38c,
      0x9b64c2b0, 0x86d3d2d4, 0xa00ae278, 0xbdbdf21c

  };

  unsigned long pcrc = ~0L;

  for (unsigned int index = 0; index < sizeof(EEPromData); ++index)
  {
    pcrc = crc_table[(pcrc ^ EEPROM[index]) & 0x0f] ^ (pcrc >> 4);
    pcrc = crc_table[(pcrc ^ (EEPROM[index] >> 4)) & 0x0f] ^ (pcrc >> 4);
    pcrc = ~pcrc;
  }

  return pcrc;
}

EEPromData get_eeprom_data()
{
  EEPromData data;
  EEPROM.get(0, data);
  EEPROM.get(EEPROM.length() - sizeof(crc), crc);
  return data;
}

void write_eeprom(bool update_var)
{
  if (update_var)
  {
    eepromdata.targetTemperature = targetTemperature;
    eepromdata.fanTarget = fanTarget;
    eepromdata.ison = ison;
    eepromdata.temp_target_limit_panic = temp_target_limit_panic;
    eepromdata.ignore_trigger = ignore_trigger;
    eepromdata.trigger_type = trigger_type;
    eepromdata.time_target = time_target;
  }
  EEPROM.put(0, eepromdata);
  crc = eeprom_crc();
  EEPROM.put(EEPROM.length() - sizeof(crc), crc);
}

void reset_eeprom()
{
  eepromdata.targetTemperature = TEMP_TARGET_DEFAULT;
  eepromdata.fanTarget = FAN_TARGET_DEFAULT;
  eepromdata.ison = true;
  eepromdata.temp_target_limit_panic = TEMP_TARGET_LIMIT_PANIC_OFF_DEFAULT;
  eepromdata.ignore_trigger = IGNORE_TRIGGER_DEFAULT;
  eepromdata.trigger_type = TRIGGER_TYPE_DEFAULT;
  eepromdata.time_target = 0;
  write_eeprom(false);
}

void load_eeprom()
{
  eepromdata = get_eeprom_data();
  targetTemperature = eepromdata.targetTemperature;
  fanTarget = eepromdata.fanTarget;
  ison = eepromdata.ison;
  temp_target_limit_panic = eepromdata.temp_target_limit_panic;
  ignore_trigger = eepromdata.ignore_trigger;
  trigger_type = eepromdata.trigger_type;
  time_target = eepromdata.time_target;
}

bool crcValid()
{
  unsigned long pcrc = eeprom_crc();
  Serial.println(pcrc);
  Serial.println(crc);
  if (pcrc != crc)
    return false;
  return true;
}

void setup()
{
  Serial.begin(115200);
  pinMode(FAN, OUTPUT);
  pinMode(HEATER_TRIAC, OUTPUT);
  pinMode(SEG_0_COM, OUTPUT);
  pinMode(SEG_1_COM, OUTPUT);
  pinMode(BUTTON_COM, INPUT);

  pinMode(TEMP_EXT_TRIG, INPUT_PULLUP);
  pinMode(NTC, INPUT);

  pinMode(A_7S, OUTPUT);
  pinMode(B_7S, OUTPUT);
  pinMode(C_7S, OUTPUT);
  pinMode(D_7S, OUTPUT);
  pinMode(E_7S, OUTPUT);
  pinMode(F_7S, OUTPUT);
  pinMode(G_7S, OUTPUT);

  IrReceiver.begin(IR);
  IrReceiver.registerReceiveCompleteCallback(ReceiveCompleteCallbackHandler);

  // while (!Serial.available())
  ;

  // eepromdata = get_eeprom_data();
  //  reset_eeprom();
  load_eeprom();

  if (!crcValid())
  {
    Serial.println("Crc invalid, reset.");
    Serial.println(eeprom_crc());
    Serial.println(crc);
    reset_eeprom();
  }
  analogWrite(FAN, mapfanspeed(fanTarget));
  wdt_reset();
  wdt_enable(WDTO_2S);
}

#pragma region SevenSegment
void reset_segment()
{
  digitalWrite(A_7S, LOW);
  digitalWrite(B_7S, LOW);
  digitalWrite(C_7S, LOW);
  digitalWrite(D_7S, LOW);
  digitalWrite(E_7S, LOW);
  digitalWrite(F_7S, LOW);
  digitalWrite(G_7S, LOW);
}

void drive_segment(bool segment, byte digit)
{
  reset_segment();
  if (!segment)
  {
    pinMode(SEG_0_COM, OUTPUT);
    pinMode(SEG_1_COM, INPUT);
    // digitalWrite(SEG_0_COM, HIGH);
    // digitalWrite(SEG_1_COM, LOW);
  }
  else
  {
    pinMode(SEG_0_COM, INPUT);
    pinMode(SEG_1_COM, OUTPUT);
    // digitalWrite(SEG_0_COM, LOW);
    // digitalWrite(SEG_1_COM, HIGH);
  }
  switch (digit)
  {
  case 0:
    digitalWrite(A_7S, HIGH);
    digitalWrite(B_7S, HIGH);
    digitalWrite(C_7S, HIGH);
    digitalWrite(D_7S, HIGH);
    digitalWrite(E_7S, HIGH);
    digitalWrite(F_7S, HIGH);
    break;
  case 1:
    digitalWrite(B_7S, HIGH);
    digitalWrite(C_7S, HIGH);
    break;
  case 2:
    digitalWrite(A_7S, HIGH);
    digitalWrite(B_7S, HIGH);
    digitalWrite(G_7S, HIGH);
    digitalWrite(D_7S, HIGH);
    digitalWrite(E_7S, HIGH);
    break;
  case 3:
    digitalWrite(A_7S, HIGH);
    digitalWrite(B_7S, HIGH);
    digitalWrite(C_7S, HIGH);
    digitalWrite(G_7S, HIGH);
    digitalWrite(D_7S, HIGH);
    break;
  case 4:
    digitalWrite(B_7S, HIGH);
    digitalWrite(C_7S, HIGH);
    digitalWrite(F_7S, HIGH);
    digitalWrite(G_7S, HIGH);
    break;
  case 5:
    digitalWrite(A_7S, HIGH);
    digitalWrite(C_7S, HIGH);
    digitalWrite(D_7S, HIGH);
    digitalWrite(F_7S, HIGH);
    digitalWrite(G_7S, HIGH);
    break;
  case 6:
    digitalWrite(A_7S, HIGH);
    digitalWrite(C_7S, HIGH);
    digitalWrite(D_7S, HIGH);
    digitalWrite(E_7S, HIGH);
    digitalWrite(F_7S, HIGH);
    digitalWrite(G_7S, HIGH);
    break;
  case 7:
    digitalWrite(A_7S, HIGH);
    digitalWrite(B_7S, HIGH);
    digitalWrite(C_7S, HIGH);
    break;
  case 8:
    digitalWrite(A_7S, HIGH);
    digitalWrite(B_7S, HIGH);
    digitalWrite(C_7S, HIGH);
    digitalWrite(D_7S, HIGH);
    digitalWrite(E_7S, HIGH);
    digitalWrite(F_7S, HIGH);
    digitalWrite(G_7S, HIGH);
    break;
  case 9:
    digitalWrite(A_7S, HIGH);
    digitalWrite(B_7S, HIGH);
    digitalWrite(C_7S, HIGH);
    digitalWrite(D_7S, HIGH);
    digitalWrite(F_7S, HIGH);
    digitalWrite(G_7S, HIGH);
    break;
  case 10:
    digitalWrite(A_7S, HIGH);
    digitalWrite(B_7S, HIGH);
    digitalWrite(C_7S, HIGH);
    digitalWrite(D_7S, HIGH);
    digitalWrite(E_7S, HIGH);
    digitalWrite(G_7S, HIGH);
    break;
  case 11:
    digitalWrite(C_7S, HIGH);
    digitalWrite(D_7S, HIGH);
    digitalWrite(E_7S, HIGH);
    digitalWrite(F_7S, HIGH);
    digitalWrite(G_7S, HIGH);
    break;
  case 12:
    digitalWrite(A_7S, HIGH);
    digitalWrite(D_7S, HIGH);
    digitalWrite(E_7S, HIGH);
    digitalWrite(F_7S, HIGH);
    break;
  case 13:
    digitalWrite(B_7S, HIGH);
    digitalWrite(C_7S, HIGH);
    digitalWrite(D_7S, HIGH);
    digitalWrite(E_7S, HIGH);
    digitalWrite(G_7S, HIGH);
    break;
  case 14:
    digitalWrite(A_7S, HIGH);
    digitalWrite(D_7S, HIGH);
    digitalWrite(E_7S, HIGH);
    digitalWrite(F_7S, HIGH);
    digitalWrite(G_7S, HIGH);
    break;
  case 15:
    digitalWrite(A_7S, HIGH);
    digitalWrite(E_7S, HIGH);
    digitalWrite(F_7S, HIGH);
    digitalWrite(G_7S, HIGH);
    break;
  case 16: //-
    digitalWrite(G_7S, HIGH);
    break;
  }
}

void display_number(int number)
{
  // Serial.println(number);
  /*if (number >= 100)
  {
    drive_segment(0, 14);
    drive_segment(1, 14);
    return;
  }*/
  if (number < 0)
  {
    drive_segment(0, 16);
    drive_segment(1, abs(number) % 10);
    return;
  }
  drive_segment(0, number / 10);
  drive_segment(1, number % 10);
}

#pragma endregion

#pragma region ButtonHandling
byte read_button()
{ // 1=- 2=+ 4=M
  byte ret = 0;
  pinMode(SEG_0_COM, INPUT);
  pinMode(SEG_1_COM, INPUT);
  reset_segment();
  pinMode(BUTTON_COM, OUTPUT);
  pinMode(BUTTON_MENU, INPUT_PULLUP);
  pinMode(BUTTON_NEG, INPUT_PULLUP);
  pinMode(BUTTON_POS, INPUT_PULLUP);
  if (!digitalRead(BUTTON_NEG))
    ret = 1;
  if (!digitalRead(BUTTON_POS))
    ret = ret + 2;
  if (!digitalRead(BUTTON_MENU))
    ret = ret + 4;
  pinMode(BUTTON_COM, INPUT);
  pinMode(BUTTON_MENU, OUTPUT);
  pinMode(BUTTON_NEG, OUTPUT);
  pinMode(BUTTON_POS, OUTPUT);
  // reset_segment();
  return ret;
}
#pragma endregion ButtonHandling

#pragma region MenuHandling
byte lastbutton = 0;
unsigned long lastbuttontime = millis();
unsigned long lastpowerbuttontime = millis();
unsigned long lastmenutime = millis();

void menu_button_handler()
{
  byte button = read_button();
  uint16_t c;
  if (sIRDataJustReceived && IrReceiver.decodedIRData.command == 0x46 && millis() - lastpowerbuttontime > TIME_POWEROFF && millis() - lastpowerbuttontime < TIME_POWEROFF + TIME_POWEROFF_DELAY && ison == true)
  { // check if power button is pressed for 1s
    ison = false;
    menu = MP - 1; // decrease menu by one, so that pressing menu/power one time turns it on.
    // lastbuttontime = millis()+100;
    lastmenutime = millis(); //+1000;
    lastpowerbuttontime = millis();
    return;
  }
  if (button == 0 && lastbutton == 0 && sIRDataJustReceived)
  {
    sIRDataJustReceived = false;
    c = IrReceiver.decodedIRData.command;
    if (c == 0x46 || c == 0x44 || c == 0x43 || c == 0x16 || c == 0xd)
    {
      button = c;
    }
  }
  if (button == 0 && millis() - lastbuttontime > 100)
    lastbuttontime = millis(); // Check if buttons released and some debounce time
  if (button != lastbutton)
  {
    lastbutton = button;
    if (button == 4 || button == 0x46)
    {
      if (!ison && millis() - lastpowerbuttontime > TIME_POWEROFF + TIME_POWEROFF_DELAY)
      {
        ison = true;
        if (menu == MP - 1)
          menu++;
        return;
      }
      if (!ison && millis() - lastpowerbuttontime > TIME_POWEROFF)
      {
        menu = MP - 1;
        return;
      }
      if (millis() - lastpowerbuttontime > TIME_POWEROFF)
        lastpowerbuttontime = millis();
      lastmenutime = millis();
      menu++;
      if (menu > MP + AMNT_MENU)
        menu = MP;
      // display_number(menu);
    }
    else if (button == 1 || button == 0xD)
    { // -
      switch (menu)
      {
      case MENU_TEMPERATURE_TARGET:
        if (!(targetTemperature == TEMP_TARGET_LIMIT_LOWER))
          targetTemperature = targetTemperature - 1;
        break;
      case MENU_FAN_TARGET:
        if (!(fanTarget == FAN_TARGET_LIMIT_LOWER))
          fanTarget = fanTarget - 1;
        analogWrite(FAN, mapfanspeed(fanTarget));
        break;
      case MENU_TIME:
        time_target = time_target - 60000;
        break;
      case MENU_IGNORE_TRIGGER:
        ignore_trigger = false;
        break;
      case MENU_TRIGGER_TYPE:
        trigger_type = false;
        break;
      case MENU_PANIC_TEMP:
        if (!(temp_target_limit_panic == 20)) // just in case
          temp_target_limit_panic = temp_target_limit_panic - 1;
        break;
      case MENU_TIMER_ON:
        timer_on = false;
        time_stop = 0;
        break;
      case MENU_REMAINING_TIME:
        if ((time_stop > 60000) && time_stop != 0)
          time_stop = time_stop - 60000;
        break;
      }
    }
    else if (button == 2 || button == 0x16)
    { // +
      switch (menu)
      {
      case MENU_TEMPERATURE_TARGET:
        if (!(targetTemperature == TEMP_TARGET_LIMIT_UPPER))
          targetTemperature = targetTemperature + 1;
        break;
      case MENU_FAN_TARGET:
        if (!(fanTarget == FAN_TARGET_LIMIT_UPPER))
          fanTarget = fanTarget + 1;
        analogWrite(FAN, mapfanspeed(fanTarget));
        break;
      case MENU_TIME:
        time_target = time_target + 60000;
        break;
      case MENU_LOCK:
        is_locked = true;
        menu = MENU_INTERNAL_TEMP_NTC;
        break;
      case MENU_RESET_EEPROM:
        reset_eeprom();
        noInterrupts();
        wdt_enable(WDTO_15MS);
        Serial.println("EEprom reset, bye");
        while (1)
          ;
        break;
      case MENU_IGNORE_TRIGGER:
        ignore_trigger = true;
        break;
      case MENU_TRIGGER_TYPE:
        trigger_type = true;
        break;
      case MENU_PANIC_TEMP:
        if (!(temp_target_limit_panic == TEMP_TARGET_LIMIT_PANIC_OFF))
          temp_target_limit_panic = temp_target_limit_panic + 1;
        break;
      case MENU_SAVE_EEPROM:
        write_eeprom(true);
        menu = MENU_INTERNAL_TEMP_NTC;
        lastmenutime = millis();
        break;
      case MENU_TIMER_ON:
        timer_on = true;
        time_stop = millis() + time_target;
        break;
      case MENU_REMAINING_TIME:
        if (time_stop != 0)
          time_stop = time_stop + 60000;
        break;
      }
    }
    else if (button == 0x44 && ison)
    {
      menu = MENU_REMAINING_TIME;
      lastmenutime = millis();
    }
    else if (button == 0x43 && ison)
    {
      menu = MENU_FAN_TARGET;
      lastmenutime = millis();
    }
  }
}
#pragma endregion MenuHandling

byte trigger_ext_value = HIGH;

unsigned long lastread = millis();

void safety_off(byte error)
{
  digitalWrite(HEATER_TRIAC, LOW);
  digitalWrite(FAN, HIGH);
  while (true)
  {
    display_number(140 + error);
    wdt_reset();
  }
}

const int NTC_Pullup_Resistance = 9500;
const int bCoefficient = 3900; // Beta Coefficient(B25 aus Datenblatt des NTC)
const int ntcNominal = 10000;  // Wiederstand des NTC bei Nominaltemperatur
const int tempNominal = 25;    // Temperatur bei der der NTC den angegebenen Wiederstand hat

float getNTCTemp(int adcValue)
{
  float resistance;
  resistance = 1023.0 / adcValue - 1.0;
  resistance = NTC_Pullup_Resistance / resistance;
  float temp;
  temp = resistance / ntcNominal;       // (R/Ro)
  temp = log(temp);                     // ln(R/Ro)
  temp /= bCoefficient;                 // 1/B * ln(R/Ro)
  temp += 1.0 / (tempNominal + 273.15); // + (1/To)
  temp = 1.0 / temp;                    // Invertieren
  temp -= 273.15;                       // Umwandeln in °C
  return temp;
}

void loop()
{
  //*** Read Values every 500ms
  if (millis() - lastread > 500)
  {
    chiptemp = (int)chipTemp(chipTempRaw());
    ntctemp = getNTCTemp(ntcTempRaw());
    lastread = millis();
  }
  trigger_ext_value = digitalRead(TEMP_EXT_TRIG) ^ trigger_type;

  //*** Safety shutoff
  if (chiptemp > temp_target_limit_panic)
    safety_off(0);
  if (ntctemp > temp_target_limit_panic) // with an pullup the value will be really high if the ntc fails open
    safety_off(1);
  //*** IR
  /*if (sIRDataJustReceived)
  {
    // sIRDataJustReceived = false;
    //  Print a short summary of received data
    IrReceiver.printIRResultShort(&Serial);
    Serial.println();
  }*/

  /*** Timer logic */
  if (timer_on && millis() > time_stop)
  {
    timer_on = false;
    ison = false;
    time_stop = 0;
  }

  //*** Display menu or value
  if (!is_locked)
    menu_button_handler();

  if (millis() - lastmenutime < 1000)
  {
    display_number(menu);
  }
  else
  {
    switch (menu)
    {
    case MENU_INTERNAL_TEMP: // Show internal Temperature
      number = chiptemp;
      break;
    case MENU_SHOW_TRIGGER: // Show external Temperature trigger
      number = trigger_ext_value + 160;
      break;
    case MENU_TEMPERATURE_TARGET: // Show set temperature, + or - should modify
      number = targetTemperature;
      break;
    case MENU_FAN_TARGET: // Show fan speed, + or - should modify
      number = fanTarget;
      break;
    case MENU_TIME: // time
      number = time_target / 60000;
      break;
    case MENU_IGNORE_TRIGGER: //"Ignore external trigger"
      number = ignore_trigger + 160;
      break;
    case MENU_INTERNAL_TEMP_NTC: // Show internal NTC temperature
      number = ntctemp;
      break;
    case MENU_LOCK:
      number = is_locked + 160;
      break;
    case MENU_PANIC_TEMP:
      number = temp_target_limit_panic;
      break;
    case MENU_TRIGGER_TYPE:
      number = trigger_type + 160;
      break;
    case MENU_TIMER_ON:
      number = timer_on + 160;
      break;
    case MENU_REMAINING_TIME:
      if (time_stop != 0)
        number = (time_stop - millis()) / 60000 + 1;
      else
        number = 0;
      break;
    }
    if (!ison && (menu == MP - 1))
    {
      drive_segment(0, 0);
      drive_segment(1, 15);
    }
    else
    {
      display_number(number);
    }
  }
  if ((!ignore_trigger && !trigger_ext_value))
  {
    ison = false; // BUG: Cannot go out of off state once this is triggered. maybe wait 10s after trigger until allowing next trigger?
    menu = MP - 1;
  }

  //*** Regulate, TODO: temperature overshoots by 8 degrees and undershoots by like 5
  if (chiptemp < targetTemperature && ntctemp < targetTemperature && ison)
    digitalWrite(HEATER_TRIAC, HIGH);
  else
    digitalWrite(HEATER_TRIAC, LOW);

  wdt_reset();
  // if (eepromdata.crc != eeprom_crc())
  //   write_eeprom();
}

#pragma region IR_INT
void ReceiveCompleteCallbackHandler()
{
  IrReceiver.decode(); // fill IrReceiver.decodedIRData
  /*
   * Set flag to trigger printing of results in main loop,
   * since printing should not be done in a callback function
   * running in ISR (Interrupt Service Routine) context where interrupts are disabled.
   */
  sIRDataJustReceived = true;

  /*
   * Enable receiving of the next value.
   * !!!Attention!!!
   * After receiving the first mark of the next (repeat) data, 3 variables required for printing are reset/overwritten.
   * - IrReceiver.irparams.rawlen
   * - IrReceiver.irparams.rawbuf[0]
   * - IrReceiver.irparams.OverflowFlag)
   */
  IrReceiver.resume();
}
#pragma endregion IR_INT