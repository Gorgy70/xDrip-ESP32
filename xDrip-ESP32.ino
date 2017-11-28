
#define DEBUG
//#define INT_BLINK_LED
#define EXT_BLINK_LED
#define GSM_MODEM
#define MODEM_SLEEP_DTR
#define PCB_V1
//#define PCB_V2
//#define USE_FREQEST    


#include <SPI.h>
#include <EEPROM.h>
#include <WiFi.h>
#include <HTTPClient.h>
#include <bt.h>
#include <bta_api.h>

#include <esp_gap_ble_api.h>
#include <esp_gatts_api.h>
#include <esp_bt_defs.h>
#include <esp_bt_main.h>
#include <esp_bt_main.h>

#include "cc2500_REG.h"
#include "webform.h"

extern "C" {
uint8_t temprature_sens_read(); 
}

#define GDO0_PIN   19            // Цифровой канал, к которму подключен контакт GD0 платы CC2500
#define LEN_PIN    5             // Цифровой канал, к которму подключен контакт LEN (усилитель слабого сигнала) платы CC2500 (Предыдущее значение 17).
#define BAT_PIN    34            // Аналоговый канал для измерения напряжения питания
#ifdef EXT_BLINK_LED
  #define RED_LED_PIN    16
  #define YELLOW_LED_PIN 17
#endif

#define SCK_PIN   22
#define MISO_PIN  21
#define MOSI_PIN  23
#define SS_PIN    18 // Предыдущее значение 5
#ifdef GSM_MODEM
  #define RX_PIN  25
  #define TX_PIN  26
#ifdef PCB_V1
  #define DTR_PIN 13
#endif
#ifdef PCB_V2
  #define DTR_PIN 14
#endif
  #define RST_PIN 27
#endif

#define NUM_CHANNELS        (4)       // Кол-во проверяемых каналов
#define FIVE_MINUTE         300000    // 5 минут
#define TWO_MINUTE          120000    // 2 минуты
#define WAKEUP_BT_TIME      45000     // Время необходимое для просыпания прибора c BT
#define WAKEUP_NON_BT_TIME  3000      // Время необходимое для просыпания прибора без BT

#define RADIO_BUFFER_LEN 200 // Размер буфера для приема данных от радиомодуля
#ifdef GSM_MODEM
  #define SERIAL_BUFFER_LEN 200 // Размер буфера для приема данных от GSM модема
#endif

#define GSM_DELAY 200         // Задержка между командами модема

// assuming that there is a 10k ohm resistor between BAT+ and BAT_PIN, and a 27k ohm resistor between BAT_PIN and GND
#define  VREF                 3.3 // Опорное напряжение для аналогового входа
#define  VMAX                 4.1  // Максимальное напряжение батареи
#define  VMIN                 3.0  // Минимальное напряжение батареи
#define  R1                   10   // Резистор делителя напряжения между BAT+ и BAT_PIN (кОм)
#define  R2                   27   // Резистор делителя напряжения между BAT_PIN и GND (кОм)
#define  ANALOG_RESOLUTION    4095 // Масимальное значение на аналоговом входе. Ио умолчанию 12 бит 
int      BATTERY_MAXIMUM  =   VMAX*ANALOG_RESOLUTION*R2/(R1+R2)/VREF ; //950 4.2V 1023*4.2*27(27+10)/3.3
int      BATTERY_MINIMUM  =   VMIN*ANALOG_RESOLUTION*R2/(R1+R2)/VREF ; //678 3.0V 1023*3.0*27/(27+10)/3.3

#define my_webservice_url    "http://parakeet.esen.ru/receiver.cgi"
#define my_webservice_reply  "!ACK"
#define my_user_agent        "parakeet-ESP32"
#define my_password_code     "12543"
#define my_gprs_apn   "internet.mts.ru"

#define my_wifi_ssid         "ssid"
#define my_wifi_pwd          "password"

#define GATTS_SERVICE_UUID_XDRIP     0xFFE0  // UID сервиса BLE
#define GATTS_CHAR_UUID_XDRIP        0xFFE1  // UID значения BLE
#define GATTS_NUM_HANDLE_TEST_ON     4       
/* maximum value of a characteristic */
#define GATTS_CHAR_VAL_LEN_MAX 0xFF

// defines the xBridge protocol functional level.  Sent in each packet as the last byte.
#define DEXBRIDGE_PROTO_LEVEL (0x01)

unsigned long dex_tx_id;
char transmitter_id[] = "ABCDE";

IPAddress local_IP(192,168,70,1);
IPAddress gateway(192,168,70,1);
IPAddress subnet(255,255,255,0);
uint8_t   wifi_mac[6];

WiFiServer server(80);
WiFiClient client;

SPIClass SPIclient(VSPI);

#ifdef GSM_MODEM
  #define ERROR_STR "ERROR"
//  SoftwareSerial mySerial(RX_PIN, TX_PIN); // RX, TX
  HardwareSerial mySerial(2);
#endif
  
unsigned long web_server_start_time;

unsigned long packet_received = 0;
unsigned long cc2500_start_time;
int current_channel;
boolean packet_catched;
boolean len_pin_low;
volatile byte gdo0_status;

//byte fOffset[NUM_CHANNELS] = { 0xE4, 0xE3, 0xE2, 0xE2 };
byte fOffset[NUM_CHANNELS] = { 0x00, 0x00, 0x00, 0x00 };
byte nChannels[NUM_CHANNELS] = { 0, 100, 199, 209 };
unsigned long waitTimes[NUM_CHANNELS] = { 0, 600, 600, 600 };

byte sequential_missed_packets = 0;
byte wait_after_time = 100;
unsigned long next_time = 0; // Время ожидания следующего пакета на канале 0
unsigned long catch_time = 0; // Время последнего пойманного пакета (приведенное к пакету на канале 0)

byte misses_until_failure = 2;                                                   //
// after how many missed packets should we just start a nonstop scan?                               //
// a high value is better for conserving batter life if you go out of wixel range a lot             //
// but it could also mean missing packets for MUCH longer periods of time                           //
// a value of zero is best if you dont care at all about battery life                               //

byte wifi_wait_tyme = 100; // Время ожидания соединения WiFi в секундах
byte default_bt_format = 0; // Формат обмена по протколу BlueTooth 0 - None 1 - xDrip, 2 - xBridge
boolean first_start_app = true; // Флаг запуска приложения
unsigned int battery_milivolts;
int battery_percent;
boolean low_battery = false;
unsigned int wake_up_time; // За сколько времени до ожидаемого сигнала надо проснуться

#ifdef GSM_MODEM
  boolean gsm_availible = false; // Доступность связи GSM
  boolean modem_availible = false; // Доступность модема на порту
  boolean internet_availible = false; // Флаг подключения мобильного интернета
  boolean modem_sleeping = false; // Модем находится в режиме сна
  char SerialBuffer[SERIAL_BUFFER_LEN]; // Буффер для работы GSM модемом
#endif

char radio_buff[RADIO_BUFFER_LEN]; // Буффер для чтения данных и прочих нужд

uint16_t ble_gatts_if = ESP_GATT_IF_NONE;
uint16_t ble_conn_id;
uint16_t ble_attr_handle;

esp_gatt_srvc_id_t ble_service;
esp_bt_uuid_t ble_character;
esp_bt_uuid_t ble_descr_uuid;

// Коды ошибок мигают лампочкой в двоичной системе
// 1 (0001) - Нет модключения к WiFi
// 2 (0010) - Облачная служба не отвечает
// 3 (0011) - Облачная служба возвращает ошибку
// 4 (0100) - Неверный CRC в сохраненных настройках. Берем настройки по умолчанию
// 5 (0101) - Не подключен модем.
// 6 (0110) - Нет мобильной связи
// 7 (0111) - Нет мобильного интернета

/* value range of a attribute (characteristic) */
uint8_t attr_str[] = {0x00};
esp_attr_value_t gatts_attr_val =
{
    .attr_max_len = GATTS_CHAR_VAL_LEN_MAX,
    .attr_len     = sizeof(attr_str),
    .attr_value   = attr_str,
};

/* service uuid */
static uint8_t service_uuid128[32] = {
    /* LSB <--------------------------------------------------------------------------------> MSB */
    //first uuid, 16bit, [12],[13] is the value
    0xfb, 0x34, 0x9b, 0x5f, 0x80, 0x00, 0x00, 0x80, 0x00, 0x10, 0x00, 0x00, 0xAB, 0xCD, 0x00, 0x00,
};

static esp_ble_adv_data_t ble_adv_data = {
    .set_scan_rsp = false,
    .include_name = true,
    .include_txpower = true,
    .min_interval = 0x20,
    .max_interval = 0x40,
    .appearance = 0x00,
    .manufacturer_len = 0,
    .p_manufacturer_data =  NULL,
    .service_data_len = 0,
    .p_service_data = NULL,
    .service_uuid_len = 16,
    .p_service_uuid = service_uuid128,
    .flag = (ESP_BLE_ADV_FLAG_GEN_DISC | ESP_BLE_ADV_FLAG_BREDR_NOT_SPT),
};

esp_ble_adv_params_t ble_adv_params;


typedef struct _Dexcom_packet
{
  byte len;
  unsigned long dest_addr;
  unsigned long src_addr;
  byte port;
  byte device_info;
  byte txId;
  unsigned int raw;
  unsigned int filtered;
  byte battery;
  byte unknown;
  byte checksum;
  byte RSSI;
  byte LQI2;
} Dexcom_packet;

Dexcom_packet Pkt;

typedef struct _RawRecord
{
  byte size; //size of the packet.
  byte cmd_code; // code for this data packet.  Always 00 for a Dexcom data packet.
  unsigned long  raw;  //"raw" BGL value.
  unsigned long  filtered; //"filtered" BGL value 
  byte dex_battery;  //battery value
  byte my_battery; //xBridge battery value
  unsigned long  dex_src_id;   //raw TXID of the Dexcom Transmitter
  //int8  RSSI; //RSSI level of the transmitter, used to determine if it is in range.
  //uint8 txid; //ID of this transmission.  Essentially a sequence from 0-63
  byte function; // Byte representing the xBridge code funcitonality.  01 = this level.
} RawRecord;

typedef struct _parakeet_settings
{
  unsigned long dex_tx_id;     //4 bytes
  char http_url[56];
  char password_code[6];
  char wifi_ssid[17];
  char wifi_pwd[18];
  byte bt_format;
  byte use_gsm;
  char gsm_apn[31];
  unsigned long checksum; // needs to be aligned

} parakeet_settings;

parakeet_settings settings;

char SrcNameTable[32] = { '0', '1', '2', '3', '4', '5', '6', '7',
                          '8', '9', 'A', 'B', 'C', 'D', 'E', 'F',
                          'G', 'H', 'J', 'K', 'L', 'M', 'N', 'P',
                          'Q', 'R', 'S', 'T', 'U', 'W', 'X', 'Y'
                        };


void dexcom_src_to_ascii(unsigned long src, char addr[6]) {
  addr[0] = SrcNameTable[(src >> 20) & 0x1F];
  addr[1] = SrcNameTable[(src >> 15) & 0x1F];
  addr[2] = SrcNameTable[(src >> 10) & 0x1F];
  addr[3] = SrcNameTable[(src >> 5) & 0x1F];
  addr[4] = SrcNameTable[(src >> 0) & 0x1F];
  addr[5] = 0;
}


unsigned long getSrcValue(char srcVal) {
  byte i = 0;
  for (i = 0; i < 32; i++) {
    if (SrcNameTable[i] == srcVal) break;
  }
  return i & 0xFF;
}

unsigned long asciiToDexcomSrc(char addr[6]) {
  unsigned long src = 0;
  src |= (getSrcValue(addr[0]) << 20);
  src |= (getSrcValue(addr[1]) << 15);
  src |= (getSrcValue(addr[2]) << 10);
  src |= (getSrcValue(addr[3]) << 5);
  src |= getSrcValue(addr[4]);
  return src;
}

byte bit_reverse_byte (byte in)
{
    byte bRet = 0;
    if (in & 0x01)
        bRet |= 0x80;
    if (in & 0x02)
        bRet |= 0x40;
    if (in & 0x04)
        bRet |= 0x20;
    if (in & 0x08)
        bRet |= 0x10;
    if (in & 0x10)
        bRet |= 0x08;
    if (in & 0x20)
        bRet |= 0x04;
    if (in & 0x40)
        bRet |= 0x02;
    if (in & 0x80)
        bRet |= 0x01;
    return bRet;
}

void bit_reverse_bytes (byte * buf, byte nLen)
{
    byte i = 0;
    for (; i < nLen; i++)
    {
        buf[i] = bit_reverse_byte (buf[i]);
    }
}

unsigned long dex_num_decoder (unsigned int usShortFloat)
{
    unsigned int usReversed = usShortFloat;
    byte usExponent = 0;
    unsigned long usMantissa = 0;
    bit_reverse_bytes ((byte *) & usReversed, 2);
    usExponent = ((usReversed & 0xE000) >> 13);
    usMantissa = (usReversed & 0x1FFF);
    return usMantissa << usExponent;
}

void clearSettings()
{
  memset (&settings, 0, sizeof (settings));
  settings.dex_tx_id = asciiToDexcomSrc (transmitter_id);
  dex_tx_id = settings.dex_tx_id;
  sprintf(settings.http_url, my_webservice_url);
  sprintf(settings.password_code, my_password_code);
  sprintf(settings.wifi_ssid, my_wifi_ssid);
  sprintf(settings.wifi_pwd, my_wifi_pwd);
  sprintf(settings.gsm_apn, my_gprs_apn);
  settings.bt_format = default_bt_format;
  settings.checksum = 0;
}

unsigned long checksum_settings()
{
  char* flash_pointer;
  unsigned long chk = 0x12345678;
  byte i;
  //   flash_pointer = (char*)settings;
  flash_pointer = (char*)&settings;
  for (i = 0; i < sizeof(parakeet_settings) - 4; i++)
  {
    chk += (flash_pointer[i] * (i + 1));
    chk++;
  }
  return chk;
}

#ifdef USE_FREQEST    
void saveOffsetToFlash()
{ 
  byte i;
  int start_pos;

  start_pos = sizeof(parakeet_settings);
  EEPROM.begin(start_pos + 4);
  for (i = 0; i < 4; i++)
  {
    EEPROM.write(start_pos+i,fOffset[i]);
    
  }
  EEPROM.commit();
}

void loadOffsetFromFlash()
{ 
  byte i;
  int start_pos;

  start_pos = sizeof(parakeet_settings);
  EEPROM.begin(start_pos + 4);
  for (i = 0; i < 4; i++)
  {
    fOffset[i] = EEPROM.read(start_pos+i);
  }
  
}
#endif

void saveSettingsToFlash()
{
  char* flash_pointer;
  byte i;

  EEPROM.begin(sizeof(parakeet_settings));
  
  settings.checksum = checksum_settings();
  flash_pointer = (char*)&settings;
  for (i = 0; i < sizeof(parakeet_settings); i++)
  {
    EEPROM.write(i,flash_pointer[i]);
  }
  EEPROM.commit();
//  EEPROM.put(0, settings);
}

void loadSettingsFromFlash()
{
  char* flash_pointer;
  byte i;

  EEPROM.begin(sizeof(parakeet_settings));
  flash_pointer = (char*)&settings;
  for (i = 0; i < sizeof(parakeet_settings); i++)
  {
    flash_pointer[i] = EEPROM.read(i);
  }
  
//  EEPROM.get(0, settings);
  dex_tx_id = settings.dex_tx_id;
  if (settings.checksum != checksum_settings()) {
    clearSettings();
#ifdef INT_BLINK_LED
    blink_sequence("0100");
#endif
#ifdef EXT_BLINK_LED
    blink_sequence_red("0100");
#endif
  }
}

#ifdef EXT_BLINK_LED
void blink_sequence_red(const char *sequence) {
  byte i;

  digitalWrite(YELLOW_LED_PIN, HIGH);
  digitalWrite(RED_LED_PIN, LOW);
  delay(500); 
  for (i = 0; i < strlen(sequence); i++) {
    digitalWrite(RED_LED_PIN, HIGH);
    switch (sequence[i]) {
      case '0': 
        delay(500);
        break;
      case '1': 
        delay(1000);
        break;
      default:
        delay(2000);
        break;
    }
    digitalWrite(RED_LED_PIN, LOW);
    delay(500); 
  }  
  digitalWrite(YELLOW_LED_PIN, LOW);
}

void blink_yellow_led_quarter() {  // Blink quarter seconds
  if ((millis() / 250) % 2) {
    digitalWrite(YELLOW_LED_PIN, HIGH);
  } else
  {
    digitalWrite(YELLOW_LED_PIN, LOW);
  }
}

void blink_yellow_led_half() {  // Blink half seconds
  if ((millis() / 500) % 2) {
    digitalWrite(YELLOW_LED_PIN, HIGH);
  } else
  {
    digitalWrite(YELLOW_LED_PIN, LOW);
  }
}

void blink_red_led_quarter() {  // Blink quarter seconds
  if ((millis() / 250) % 2) {
    digitalWrite(RED_LED_PIN, HIGH);
  } else
  {
    digitalWrite(RED_LED_PIN, LOW);
  }
}

void blink_red_led_quarter2() {  // Blink quarter seconds
  if ((millis() / 250) % 2) {
    digitalWrite(RED_LED_PIN, LOW);
  } else
  {
    digitalWrite(RED_LED_PIN, HIGH);
  }
}

void blink_red_led_half() {  // Blink half seconds
  if ((millis() / 500) % 2) {
    digitalWrite(RED_LED_PIN, HIGH);
  } else
  {
    digitalWrite(RED_LED_PIN, LOW);
  }
}

void blink_red_led_half2() {  // Blink half seconds
  if ((millis() / 500) % 2) {
    digitalWrite(RED_LED_PIN, LOW);
  } else
  {
    digitalWrite(RED_LED_PIN, HIGH);
  }
}
#endif

#ifdef INT_BLINK_LED
void blink_sequence(const char *sequence) {
  byte i;

  digitalWrite(LED_BUILTIN, LOW);
  delay(500); 
  for (i = 0; i < strlen(sequence); i++) {
    digitalWrite(LED_BUILTIN, HIGH);
    switch (sequence[i]) {
      case '0': 
        delay(500);
        break;
      case '1': 
        delay(1000);
        break;
      default:
        delay(2000);
        break;
    }
    digitalWrite(LED_BUILTIN, LOW);
    delay(500); 
  }  
}

void blink_builtin_led_quarter() {  // Blink quarter seconds
  if ((millis() / 250) % 2) {
    digitalWrite(LED_BUILTIN, HIGH);
  } else
  {
    digitalWrite(LED_BUILTIN, LOW);
  }
}

void blink_builtin_led_half() {  // Blink half seconds
  if ((millis() / 500) % 2) {
    digitalWrite(LED_BUILTIN, HIGH);
  } else
  {
    digitalWrite(LED_BUILTIN, LOW);
  }
}
#endif

void WriteReg(char addr, char value) {
  digitalWrite(SS_PIN, LOW);
  while (digitalRead(MISO_PIN) == HIGH) {
  };
  SPIclient.transfer(addr);
  SPIclient.transfer(value);
  digitalWrite(SS_PIN, HIGH);
  //  delay(10);
}

char SendStrobe(char strobe)
{
  digitalWrite(SS_PIN, LOW);

  while (digitalRead(MISO_PIN) == HIGH) {
  };

  char result =  SPIclient.transfer(strobe);
  digitalWrite(SS_PIN, HIGH);
  //  delay(10);
  return result;
}

void init_CC2500() {
//FSCTRL1 and MDMCFG4 have the biggest impact on sensitivity...
   
   WriteReg(PATABLE, 0x00);
//   WriteReg(IOCFG0, 0x01);
   WriteReg(IOCFG0, 0x06);
   WriteReg(PKTLEN, 0xff);
   WriteReg(PKTCTRL1, 0x0C); // CRC_AUTOFLUSH = 1 & APPEND_STATUS = 1
//   WriteReg(PKTCTRL1, 0x04);
   WriteReg(PKTCTRL0, 0x05);
   WriteReg(ADDR, 0x00);
   WriteReg(CHANNR, 0x00);

//   WriteReg(FSCTRL1, 0x0A); 
//   WriteReg(FSCTRL1, 0x0f);  // Intermediate Freq.  Fif = FRef x FREQ_IF / 2^10 = 24000000 * 10/1024 = 234375  for 0x0F = 351562.5
   WriteReg(FSCTRL1, 0x09);  // Intermediate Freq.  Fif = FRef x FREQ_IF / 2^10 = 26000000 * 9/1024 = 228515,625  for 0x0F = 380859,375
   WriteReg(FSCTRL0, 0x00);  
  
   WriteReg(FREQ2, 0x5d);
   WriteReg(FREQ1, 0x44);
   WriteReg(FREQ0, 0xeb);
   
   WriteReg(FREND1, 0xb6);  
   WriteReg(FREND0, 0x10);  

   // Bandwidth
   //0x4a = 406 khz
   //0x5a = 325 khz
   // 300 khz is supposedly what dex uses...
   //0x6a = 271 khz
   //0x7a = 232 khz
//   WriteReg(MDMCFG4, 0x7a); //appear to get better sensitivity
   WriteReg(MDMCFG4, 0x7a); //  Rx Filter BW
   WriteReg(MDMCFG3, 0xf8);
   WriteReg(MDMCFG2, 0x73);
   WriteReg(MDMCFG1, 0x23);
   WriteReg(MDMCFG0, 0x3b);
   
//   WriteReg(DEVIATN, 0x40);
   WriteReg(DEVIATN, 0x00);

   WriteReg(MCSM2, 0x07);
   WriteReg(MCSM1, 0x30);
   WriteReg(MCSM0, 0x18);  
//   WriteReg(FOCCFG, 0x16); //36
   WriteReg(FSCAL3, 0xa9);
   WriteReg(FSCAL2, 0x0a);
   WriteReg(FSCAL1, 0x00);
   WriteReg(FSCAL0, 0x11);
  
   WriteReg(AGCCTRL2, 0x03);  
   WriteReg(AGCCTRL1, 0x00);
   WriteReg(AGCCTRL0, 0x91);
   //
   WriteReg(TEST2, 0x81);
   WriteReg(TEST1, 0x35); 
   WriteReg(TEST0, 0x0b);  
   
//   WriteReg(FOCCFG, 0x0A);    // allow range of +/1 FChan/4 = 375000/4 = 93750.  No CS GATE
   WriteReg(FOCCFG, 0x2A);    // allow range of +/1 FChan/4 = 375000/4 = 93750.  No CS GATE
   WriteReg(BSCFG, 0x6C);
 
}

char ReadReg(char addr) {
  addr = addr + 0x80;
  digitalWrite(SS_PIN, LOW);
  while (digitalRead(MISO_PIN) == HIGH) {
  };
  SPIclient.transfer(addr);
  char y = SPIclient.transfer(0);
  digitalWrite(SS_PIN, HIGH);
  //  delay(10);
  return y;
}

char ReadStatus(char addr) {
  addr = addr + 0xC0;
  digitalWrite(SS_PIN, LOW);
  while (digitalRead(MISO_PIN) == HIGH) {
  };
  SPIclient.transfer(addr);
  char y = SPIclient.transfer(0);
  digitalWrite(SS_PIN, HIGH);
  //  delay(10);
  return y;
}

String urlDecode(const String& text)
{
  String decoded = "";
  char temp[] = "0x00";
  unsigned int len = text.length();
  unsigned int i = 0;
  while (i < len)
  {
    char decodedChar;
    char encodedChar = text.charAt(i++);
    if ((encodedChar == '%') && (i + 1 < len))
    {
      temp[2] = text.charAt(i++);
      temp[3] = text.charAt(i++);

      decodedChar = strtol(temp, NULL, 16);
    }
    else {
      if (encodedChar == '+')
      {
        decodedChar = ' ';
      }
      else {
        decodedChar = encodedChar;  // normal ascii char
      }
    }
    decoded += decodedChar;
  }
  return decoded;
}

String paramByName(const String& param_string, const String& param_name) {
  unsigned int param_index;
  String param = "";

  param_index = param_string.indexOf(param_name + "=");
  if (param_index >= 0) {
    param_index += param_name.length() + 1;
    while (param_index < param_string.length() && param_string.charAt(param_index) != '&') {
      param += param_string.charAt(param_index);
      param_index++ ;
    }
  }
  return urlDecode(param);
}

void handleRoot() {
  char current_id[6];
  char temp[1800];
  char chk1[8];
  char chk2[8];
  char chk3[8];
  char chk4[8];

#ifdef DEBUG
  Serial.println("http server root"); 
#endif
  dexcom_src_to_ascii(settings.dex_tx_id,current_id);
  switch (settings.bt_format) {
    case 0:
      sprintf(chk1,"%s","checked");
      chk2[0] = '\0';
      chk3[0] = '\0';
      break;
    case 1:
      chk1[0] = '\0';
      sprintf(chk2,"%s","checked");
      chk3[0] = '\0';
      break;
    case 2:
      chk1[0] = '\0';
      chk2[0] = '\0';
      sprintf(chk3,"%s","checked");
      break;
    default:  
      chk1[0] = '\0';
      chk2[0] = '\0';
      chk3[0] = '\0';
      break;
  } 
  if (settings.use_gsm == 0) chk4[0] = '\0';
  else sprintf(chk4,"%s","checked");
  sprintf(temp,edit_form,current_id,settings.password_code,settings.http_url,settings.wifi_ssid,settings.wifi_pwd,chk1,chk2,chk3,chk4,settings.gsm_apn);
//  server.send(200, "text/html", temp);
  client.println("HTTP/1.1 200 OK");
  client.println("Content-Type: text/html");  
  client.println("Connection: close");
  client.println();
  client.println("<!DOCTYPE HTML>");
  client.println(temp);
  client.println();
}

void handleNotFound() {
#ifdef DEBUG
  Serial.println("http server not found"); 
#endif
  client.println("HTTP/1.1 404 Not Found");
  client.println("Content-Type: text/plain");
  client.println("Connection: close");
  client.println();
  client.println("not found!");
  client.println();
//  server.send ( 404, "text/plain", "not found!" );
}

void handleSave(const String& param_string) {
  char new_id[6]; 
  String arg1;
  char temp[1400];
  char bt_frmt[8];

  arg1 = paramByName(param_string,"DexcomID");
  arg1.toCharArray(new_id,6);
  settings.dex_tx_id = asciiToDexcomSrc (new_id);
  dex_tx_id = settings.dex_tx_id;
  arg1 = paramByName(param_string,"PasswordCode");
  arg1.toCharArray(settings.password_code,6);
  arg1 = paramByName(param_string,"WebService");
  arg1.toCharArray(settings.http_url,56);
  arg1 = paramByName(param_string,"WiFiSSID");
  arg1.toCharArray(settings.wifi_ssid,16);
  arg1 = paramByName(param_string,"WiFiPwd");
  arg1.toCharArray(settings.wifi_pwd,16);
  arg1 = paramByName(param_string,"BtFormat");
  if (arg1 == "0") {
    settings.bt_format = 0;
    sprintf(bt_frmt,"%s","None");
  }
  else if (arg1 == "1") {   
    settings.bt_format = 1;
    sprintf(bt_frmt,"%s","xDrip");
  } else if (arg1 == "2") {    
    settings.bt_format = 2;
    sprintf(bt_frmt,"%s","xBridge");
  }
  arg1 = paramByName(param_string,"UseGSM");
#ifdef DEBUG
  Serial.print("UseGSM = ");
  Serial.println(arg1);
#endif      
  if (arg1 == "YES") settings.use_gsm = 1;
  else settings.use_gsm = 0;
  arg1 = paramByName(param_string,"APN");
  arg1.toCharArray(settings.gsm_apn,31);
    
  saveSettingsToFlash();
  
  sprintf(temp, "Configuration saved!<br>DexcomID = %s<br>Password Code = %s<br>URL = %s<br>WiFi SSID = %s<br>WiFi Password = %s<br> BlueTooth format: %s<br> Use GSM %d<br> APN = %s",
                new_id,settings.password_code,settings.http_url,settings.wifi_ssid,settings.wifi_pwd,bt_frmt,settings.use_gsm,settings.gsm_apn);
  client.println("HTTP/1.1 200 OK");
  client.println("Content-Type: text/html");  
  client.println("Connection: close");
  client.println();
  client.print("<!DOCTYPE HTML>");
  client.println(temp);
  client.println();

//  server.send ( 200, "text/html",temp );
#ifdef DEBUG
  Serial.println("Configuration saved!");
#endif      
}

void PrepareWebServer() {
/*  
#ifdef DEBUG
  if (!ret) {
    Serial.println("Set IP address error!");    
  } 
  else {
    Serial.print("IP = ");
    Serial.println(WiFi.softAPIP.toString());
  }  
#endif      
*/
  WiFi.softAP("Parakeet");
  WiFi.softAPmacAddress(wifi_mac);
#ifdef DEBUG
  Serial.println("Set SSID Name Done!");    
  Serial.print("WiFi MAC = ");
  Serial.println(WiFi.softAPmacAddress());
#endif      
  bool ret = WiFi.softAPConfig(local_IP, gateway, subnet);
#ifdef DEBUG
  if (!ret) {
    Serial.println("Set IP address error!");    
  } 
  else {
    Serial.println("Set IPAddress Done!");    
  }
#endif      
  server.begin(); 
#ifdef DEBUG
  Serial.println("Server start OK!");    
#endif      
  web_server_start_time = millis();   
}

static void gap_event_handler(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param)
{
    switch (event) {
    case ESP_GAP_BLE_ADV_DATA_SET_COMPLETE_EVT:
        esp_ble_gap_start_advertising(&ble_adv_params);
        break;
    case ESP_GAP_BLE_ADV_DATA_RAW_SET_COMPLETE_EVT:
        esp_ble_gap_start_advertising(&ble_adv_params);
        break;
    case ESP_GAP_BLE_SCAN_RSP_DATA_RAW_SET_COMPLETE_EVT:
        esp_ble_gap_start_advertising(&ble_adv_params);
        break;
#ifdef DEBUG
    case ESP_GAP_BLE_ADV_START_COMPLETE_EVT:
        //advertising start complete event to indicate advertising start successfully or failed
        if (param->adv_start_cmpl.status != ESP_BT_STATUS_SUCCESS) {
            Serial.println("Advertising start failed\n");
        }
        break;
    case ESP_GAP_BLE_ADV_STOP_COMPLETE_EVT:
        if (param->adv_stop_cmpl.status != ESP_BT_STATUS_SUCCESS) {
            Serial.println("Advertising stop failed\n");
        }
        else {
            Serial.println("Stop adv successfully\n");
        }
        break;
#endif      
    default:
        break;
    }
}

static void gatts_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param)
{
    esp_err_t ret;
    byte dexderip_data[20];
    
    switch (event) {
    case ESP_GATTS_REG_EVT: {
#ifdef DEBUG
        printf("REGISTER_APP_EVT, status %d, app_id %d\n", param->reg.status, param->reg.app_id);
#endif      
        esp_ble_gatts_create_service(gatts_if, &ble_service, GATTS_NUM_HANDLE_TEST_ON);
        break;
    }   
    case ESP_GATTS_CREATE_EVT: {
#ifdef DEBUG
        printf("ESP_GATTS_CREATE_EVT, status %d,  service_handle %d\n", param->create.status, param->create.service_handle);
#endif      
        /* 1 service LED and 2 characteristics ON and OFF */
                
        ret = esp_ble_gatts_add_char(param->create.service_handle, &ble_character,
                                     ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
                                     ESP_GATT_CHAR_PROP_BIT_READ | ESP_GATT_CHAR_PROP_BIT_WRITE | ESP_GATT_CHAR_PROP_BIT_NOTIFY | ESP_GATT_CHAR_PROP_BIT_INDICATE,
                                     &gatts_attr_val, NULL);

        Serial.print("Ret = ");
        Serial.println(ret);
        esp_ble_gatts_start_service(param->create.service_handle);
        break;
    }    
    case ESP_GATTS_CONNECT_EVT: {
    /* create service event */
#ifdef DEBUG
        Serial.print("ESP_GATTS_CONNECT_EVT, conn_id = ");
        Serial.print(param->connect.conn_id);
        Serial.print(" gatts_if = ");
        Serial.println(gatts_if);
#endif      
        ble_gatts_if = gatts_if;
        ble_conn_id = param->connect.conn_id;
        esp_ble_conn_update_params_t conn_params = {0};
        memcpy(conn_params.bda, param->connect.remote_bda, sizeof(esp_bd_addr_t));
        /* For the IOS system, please reference the apple official documents about the ble connection parameters restrictions. */
        conn_params.latency = 0;
        conn_params.max_int = 0x50;    // max_int = 0x50*1.25ms = 100ms
        conn_params.min_int = 0x30;    // min_int = 0x30*1.25ms = 60ms
        conn_params.timeout = 1000;    // timeout = 1000*10ms = 10000ms
#ifdef DEBUG
        printf("ESP_GATTS_CONNECT_EVT, conn_id %d, remote %02x:%02x:%02x:%02x:%02x:%02x:, is_conn %d\n",
                 param->connect.conn_id,
                 param->connect.remote_bda[0], param->connect.remote_bda[1], param->connect.remote_bda[2],
                 param->connect.remote_bda[3], param->connect.remote_bda[4], param->connect.remote_bda[5],
                 param->connect.is_connected);
#endif      
        //start sent the update connection parameters to the peer device.
        esp_ble_gap_update_conn_params(&conn_params);
        break;
    }
    case ESP_GATTS_WRITE_EVT: {
#ifdef DEBUG
        printf("ESP_GATTS_WRITE_EVT, conn_id %d, trans_id %d, handle %d\n", param->write.conn_id, param->write.trans_id, param->write.handle);
        Serial.print("data len = ");
        Serial.println(param->write.len);
        Serial.print("data = ");
        Serial.println(*(uint8_t *)param->write.value);
#endif      
        memcpy(dexderip_data,param->write.value,param->write.len);
        if (param->write.len = 0x2 && dexderip_data[0] == 0x2 && dexderip_data[1] == 0xF0) {
#ifdef DEBUG
          Serial.println("Data Acknowledge Packet");
#endif                
        }
        if (param->write.len = 0x6 && dexderip_data[0] == 0x6 && dexderip_data[1] == 0x01) {
          memcpy(&dex_tx_id,&dexderip_data[2],4);
          settings.dex_tx_id = dex_tx_id;
          saveSettingsToFlash();
#ifdef DEBUG
          Serial.println("New TransmitterID Packet");
          Serial.print("Dexcom ID: ");
          Serial.println(dex_tx_id);
#endif    
        }
        if (param->write.need_rsp){
          esp_ble_gatts_send_response(gatts_if, param->write.conn_id, param->write.trans_id, ESP_GATT_OK, NULL);
        }  
        break;
    }    
    case ESP_GATTS_ADD_CHAR_EVT: {
#ifdef DEBUG
        Serial.print("ESP_GATTS_ADD_CHAR_EVT, attr_handle = ");
        Serial.print(param->add_char.attr_handle);
        Serial.print(", status = ");
        Serial.println(param->add_char.status);
#endif      
        ble_attr_handle = param->add_char.attr_handle;
        esp_ble_gatts_add_char_descr(param->add_char.service_handle, &ble_descr_uuid,
                                     ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE, NULL, NULL);
        break;
    }
    case ESP_GATTS_DISCONNECT_EVT: {
        ble_gatts_if = ESP_GATT_IF_NONE;
        ble_conn_id = 0;
        esp_ble_gap_start_advertising(&ble_adv_params);
        break;
    }    
#ifdef DEBUG
    case ESP_GATTS_READ_EVT: {
        printf("ESP_GATTS_READ_EVT, conn_id %d, trans_id %d, handle %d\n", param->read.conn_id, param->read.trans_id, param->read.handle);
        break;
    }
    case ESP_GATTS_EXEC_WRITE_EVT:    {
        printf("ESP_GATTS_EXEC_WRITE_EVT, conn_id %d, trans_id %d, handle %d\n", param->read.conn_id, param->read.trans_id, param->read.handle);
        break;
    }
    case ESP_GATTS_CONF_EVT: {          
        Serial.print("ESP_GATTS_CONF_EVT, status = ");
        Serial.println(param->conf.status);
        break;
    }
    case ESP_GATTS_ADD_CHAR_DESCR_EVT: {      
        printf("ESP_GATTS_ADD_CHAR_DESCR_EVT, conn_id %d, trans_id %d, handle %d\n", param->conf.conn_id, param->read.trans_id, param->read.handle);
        break;
    }
#endif      
    default:
        break;
    }
}

void sendBeacon()
{
  //char array to store the response in.
  unsigned char cmd_response[8];

  if (ble_gatts_if == ESP_GATT_IF_NONE ) {
#ifdef DEBUG
    Serial.println("Not connected");
#endif
    return;
  }
  
  //return if we don't have a connection or if we have already sent a beacon
  cmd_response[0] = 0x07;
  cmd_response[1] = 0xF1;
  memcpy(&cmd_response[2], &settings.dex_tx_id, sizeof(settings.dex_tx_id));
  cmd_response[6] = DEXBRIDGE_PROTO_LEVEL;
  cmd_response[7] = '\0';
  
   esp_err_t ret = esp_ble_gatts_send_indicate(ble_gatts_if, ble_conn_id, ble_attr_handle, cmd_response[0],&cmd_response[0], false); 
#ifdef DEBUG
   if (ret == ESP_OK) {
     Serial.println("Send indicate OK");    
   } 
   else {
    Serial.println("Send indicate fail");
   }  
#endif
}

void PrepareBlueTooth() {
    
    char bt_name[15];

    if (settings.bt_format == 0) return;
    ble_adv_params.adv_int_min        = 0x20;
    ble_adv_params.adv_int_max        = 0x40;
    ble_adv_params.adv_type           = ADV_TYPE_IND;
    ble_adv_params.own_addr_type      = BLE_ADDR_TYPE_PUBLIC;
    ble_adv_params.channel_map        = ADV_CHNL_ALL;
    ble_adv_params.adv_filter_policy = ADV_FILTER_ALLOW_SCAN_ANY_CON_ANY;
    /* initialize profile and characteristic */

    ble_service.is_primary = true;
    ble_service.id.inst_id = 0;
    ble_service.id.uuid.len = ESP_UUID_LEN_16;
    ble_service.id.uuid.uuid.uuid16 = GATTS_SERVICE_UUID_XDRIP;
    
    ble_character.len = ESP_UUID_LEN_16;
    ble_character.uuid.uuid16 = GATTS_CHAR_UUID_XDRIP;

    ble_descr_uuid.len = ESP_UUID_LEN_16;
    ble_descr_uuid.uuid.uuid16 = ESP_GATT_UUID_CHAR_CLIENT_CONFIG;
    
    esp_err_t ret;
    /* initialize BLE and bluedroid */
    btStart();
    ret = esp_bluedroid_init();
    if (ret) {
#ifdef DEBUG
      Serial.println("init bluetooth failed");
#endif      
      return;
    }
    ret = esp_bluedroid_enable();
    if (ret) {
#ifdef DEBUG
      Serial.println("enable bluetooth failed");
#endif      
      return;
    }
    /* set BLE name and broadcast advertising info
    so that the world can see you*/
    if (settings.bt_format == 1) {
      sprintf(bt_name,"%s%02X%02X","xDripESP",wifi_mac[4],wifi_mac[5]);
    } 
    else if (settings.bt_format == 2) {   
      sprintf(bt_name,"%s%02X%02X","xBridgeESP",wifi_mac[4],wifi_mac[5]);
    }
    esp_ble_gap_set_device_name(bt_name);
    esp_ble_gap_config_adv_data(&ble_adv_data);
    /* register callbacks to handle events like register device,
    sending and receiving data */
    esp_ble_gatts_register_callback(gatts_event_handler);
    esp_ble_gap_register_callback(gap_event_handler);
    /* register profiles */
    esp_ble_gatts_app_register(0);
}

#ifdef GSM_MODEM
boolean gsm_command(const char *command, const char *response, int timeout, boolean break_on_error) {
  boolean ret;
  unsigned long timeout_time; 
  int len = strlen (response);
  int len_error = strlen(ERROR_STR);
  int loop = 0;

#ifdef INT_BLINK_LED  
  digitalWrite(LED_BUILTIN, HIGH);
#endif
#ifdef EXT_BLINK_LED  
  digitalWrite(RED_LED_PIN, HIGH);
#endif

  if (len == 0) {
    ret = true;
  } 
  else {
    ret = false;
  }  
//  memset (&SerialBuffer,0,sizeof(SerialBuffer));
  memset (&SerialBuffer[0],0,sizeof(SerialBuffer));
//  memset (&settings, 0, sizeof (settings));
//  mySerial.write(command);
//  mySerial.write("\r\n"); // enter key
  mySerial.println(command);
  timeout_time = timeout;
  timeout_time = millis() + (timeout_time * 1000);
  while (millis() < timeout_time)
  {
    if (mySerial.available()) {
      delayMicroseconds(100);
      SerialBuffer[loop] = mySerial.read();
      loop++;
      if (loop == SERIAL_BUFFER_LEN) loop = 0; // Контролируем переполнение буфера
      if (loop > len) {
        if (strncmp(response,&SerialBuffer[loop-len],len) == 0) {
          ret = true;
          delay(100);
        }
      }  
      if (break_on_error && loop >= len_error && strncmp(ERROR_STR,&SerialBuffer[loop-len_error],len_error) == 0) break;
    } 
    else {
      if (ret) {
        delayMicroseconds(100);
        break;
      }
    }
  }
  SerialBuffer[loop] = '\0';
  while (mySerial.available()) {
    delayMicroseconds(100);
    mySerial.read();
  }
#ifdef DEBUG
  Serial.print("Cmd=");
  Serial.println(command);
  Serial.print("Exp.rep=");
  Serial.println(response);
  Serial.print("Resp=");
  Serial.println(SerialBuffer);
  Serial.print("Res=");
  Serial.println(ret);
#endif
#ifdef INT_BLINK_LED  
  digitalWrite(LED_BUILTIN, LOW);
#endif
#ifdef EXT_BLINK_LED  
  digitalWrite(RED_LED_PIN, LOW);
#endif
  return ret;
}

boolean gsm_command(const char *command, const char *response, int timeout) {
  return gsm_command(command,response,timeout,true);
}

boolean set_gprs_profile() {
  boolean ret;
  
  delay(GSM_DELAY);
  gsm_command("AT+SAPBR=0,1", "OK", 10); // Сбросим настроенный GPRS профиль
  delay(GSM_DELAY);
  ret = gsm_command("AT+SAPBR=3,1,\"Contype\",\"GPRS\"", "OK", 2); // Настраиваем мобильный интернет 2G
  if (ret) {
    sprintf(radio_buff,"AT+SAPBR=3,1,\"APN\",\"%s\"",settings.gsm_apn);   // Точка доступа
    delay(GSM_DELAY);
    ret = gsm_command(radio_buff,"OK",2);
    if (ret) {
      delay(GSM_DELAY);
      ret = gsm_command("AT+SAPBR=1,1", "OK", 90); // Применяем настройки
    }
  }
  return ret;    
}

void set_settings(char *settings_str,char *data, byte idx,byte max_len) {
  byte i1 = idx;  
  byte i2 = 0;

  while (data[i1] == ' ') {
    i1++;
    if (i1 == SERIAL_BUFFER_LEN) return;
  }
  while (isPrintable(data[i1]) and data[i1] != ' ') {
    settings_str[i2] = data[i1];
    i1++;
    if (i1 == SERIAL_BUFFER_LEN) break;
    i2++;
    if (i2 == max_len-1) break;
  }
  settings_str[i2] = '\0';
}

void send_sms(char *phone, char *cmd, char *data) {
#ifdef DEBUG
  Serial.print("SMS to number = ");
  Serial.println(phone);
  Serial.print(cmd);
  Serial.println(data);
#endif
  mySerial.print("AT+CMGS=");
  delay(GSM_DELAY);
  if (gsm_command(phone,">",2)) {
    delay(GSM_DELAY);
    mySerial.print(cmd);
    delay(GSM_DELAY);
    mySerial.print(data);
    delay(GSM_DELAY);
    gsm_command("\x1A","OK",20);
  }
}

void extract_phone_number(char *phone, char *data, byte idx) {
  byte i1 = idx;
  byte i2 = 0;

  while (i1 > 0) {
    if (data[i1] == '"' && data[i1+1] == '+') break;
    i1--;
  }
  if (i1 == 0) return;
  while (data[i1] != ',') {
    phone[i2] = data[i1];
    i1++;
    i2++;
    if (i2 == 14) break;
  }
  phone[i2] = '\0';
}

void read_sms() {
  boolean ret;
  byte i;
  boolean reboot = false;
  char phone_number[15];
  char ascii_trans_id[6];

  delay(GSM_DELAY);
  gsm_command("AT+CMGL=\"REC UNREAD\"" ,"OK",5); // Читаем все новые смс-ки в буфер
  memset(&radio_buff,0,sizeof(radio_buff));
  strcpy(radio_buff,SerialBuffer);
  for (i = 0; i < SERIAL_BUFFER_LEN - 4; i++) {
    if (strncmp("APN ",&radio_buff[i],4) == 0) {
      set_settings(settings.gsm_apn,radio_buff,i+4,32);
      saveSettingsToFlash();
      set_gprs_profile();      
      extract_phone_number(phone_number,radio_buff,i);
      send_sms(phone_number,"APN:",settings.gsm_apn);
    }
    if (strncmp("DEFAULTS",&radio_buff[i],8) == 0) {
      clearSettings();
      saveSettingsToFlash();
      extract_phone_number(phone_number,radio_buff,i);
      send_sms(phone_number,"DEFAULTS:","OK");
    }
    if (strncmp("TRANSMIT ",&radio_buff[i],9) == 0) {
      set_settings(transmitter_id,radio_buff,i+9,6);
      settings.dex_tx_id = asciiToDexcomSrc (transmitter_id);
      dex_tx_id = settings.dex_tx_id;
      saveSettingsToFlash();
      extract_phone_number(phone_number,radio_buff,i);
      send_sms(phone_number,"TRANSMIT:",transmitter_id);
    }
    if (strncmp("HTTP ",&radio_buff[i],5) == 0) {
      set_settings(settings.http_url,radio_buff,i+5,56);
      saveSettingsToFlash();
      extract_phone_number(phone_number,radio_buff,i);
      send_sms(phone_number,"HTTP:",settings.http_url);
    }
    if (strncmp("PWD ",&radio_buff[i],4) == 0) {
      set_settings(settings.password_code,radio_buff,i+4,6);
      saveSettingsToFlash();
      extract_phone_number(phone_number,radio_buff,i);
      send_sms(phone_number,"PWD:",settings.password_code);
    }
    if (strncmp("REBOOT",&radio_buff[i],6) == 0) {
      reboot = true;
      extract_phone_number(phone_number,radio_buff,i);
      send_sms(phone_number,"REBOOT:","OK");
    }
    if (strncmp("SETTINGS",&radio_buff[i],8) == 0) {
      extract_phone_number(phone_number,radio_buff,i);
      dexcom_src_to_ascii(dex_tx_id,ascii_trans_id);
      send_sms(phone_number,"TRANSMIT:",ascii_trans_id);
      send_sms(phone_number,"APN:",settings.gsm_apn);
      send_sms(phone_number,"HTTP:",settings.http_url);
      send_sms(phone_number,"PWD:",settings.password_code);     
    }
  }
  delay(GSM_DELAY);
  gsm_command("AT+CMGDA=\"DEL READ\"","OK",5); // Удалить прочитанные смс-ки
  delay(GSM_DELAY);
  gsm_command("AT+CMGDA=\"DEL SENT\"","OK",5); // Удалить отправленные смс-ки
  if (reboot) ESP.restart();
}


void gsm_wake_up() {
#ifdef MODEM_SLEEP_DTR  
  digitalWrite(DTR_PIN, LOW); // Будим GSM-модем
#else  
  mySerial.write(27);
  delay(GSM_DELAY); 
  gsm_command("AT+CSCLK=0", "OK", 2); // Отключаем на модеме режим сна
#endif  
  delay(GSM_DELAY); 
   // Включаем мигание модема
  if (!gsm_command("AT+CNETLIGHT=1", "OK", 2))
  {
    modem_availible = false;
    init_gsm_modem();
    if (modem_availible) {
      gsm_command("AT+CNETLIGHT=1", "OK", 2);
    } 
    else {
      gsm_availible = false;
      internet_availible = false;
    }
  }
  modem_sleeping = false;
}

void gsm_goto_sleep() {
//  gsm_command("AT+CSCLK=1", "OK", 2); // Переводим модем в режим сна в режиме управления сигналом DTR
  gsm_command("AT+CNETLIGHT=0", "OK", 2); // Отключаем мигание модема
  delay(GSM_DELAY);
#ifdef MODEM_SLEEP_DTR  
  digitalWrite(DTR_PIN, HIGH);
#else  
  gsm_command("AT+CSCLK=2", "OK", 2); // Переводим модем в режим сна 2
  delay(GSM_DELAY);
#endif  
  modem_sleeping = true;
}

void init_base_gsm()
{
//  gsm_command("AT+IPR=9600","OK",2); // Установить скорость порта 9600
//  gsm_command("AT+IFC=0,0","OK",2); 
  gsm_command("ATZ","OK",10); // Установить параметры по умолчанию 
  gsm_command("ATE0","OK", 2); // Выключить эхо 
  gsm_command("AT+CFUN=0", "OK",10); // Отключаем мобильную связь
}

boolean init_gsm_modem()
{
  if (!modem_availible) {
//    gsm_command("AT+IPR=9600","OK",2); // Установить скорость порта 9600
//    gsm_command("AT+IFC=0,0","OK",2); 
    if (!gsm_command("AT","OK",2)) {
      digitalWrite(DTR_PIN, HIGH);
      delay(200);
      digitalWrite(DTR_PIN, LOW);
      if (!gsm_command("AT","OK",2)) {
        mySerial.write(27);
        delay(300);
        if (!gsm_command("AT","OK",2)) {
#ifdef INT_BLINK_LED
          blink_sequence("0101");
#endif
#ifdef EXT_BLINK_LED
          blink_sequence_red("0101");
#endif
          return false;
        }
      }
    }
    modem_availible = true;
  } 
  return true;
}

void init_GSM(boolean sleep_after_init) {
  if (!init_gsm_modem()) return;
  init_base_gsm();
  delay(200);
#ifdef MODEM_SLEEP_DTR  
  gsm_command("AT+CSCLK=1", "OK", 2); // Переводим модем в режим сна в режиме управления сигналом DTR
  delay(200);
#endif  
  if (settings.use_gsm) {
    gsm_availible = gsm_command("AT+CFUN=1", "Call Ready", 30); // Подключаемся к сети  
    if (gsm_availible) {
      delay(GSM_DELAY);
//    gsm_command("AT+CMGF?","OK",10); // Устанавливаем текстовый режим чтения смс
      gsm_command("AT+CPMS?","OK",2);
// Устанавливаем текстовый режим чтения смс
      if (gsm_command("AT+CMGF=1","OK",10)) { 
        read_sms();
      }  
      else {
        gsm_command("AT+CMGL=0","OK",10);
      }
      internet_availible = set_gprs_profile();
      if (!internet_availible) {
#ifdef INT_BLINK_LED
        blink_sequence("0111");
#endif
#ifdef EXT_BLINK_LED
        blink_sequence_red("0111");
#endif
      
      }
    }  
    else {
#ifdef INT_BLINK_LED
      blink_sequence("0110");
#endif
#ifdef EXT_BLINK_LED
      blink_sequence_red("0110");
#endif
    }
  }  
  if (sleep_after_init) {
    gsm_goto_sleep();
  }  
}

void gsm_get_location(char *location) {
  byte i;
  byte i1 = 0;
  byte i2 = 0;
  byte i3 = 0;

  location[0]='\0';
  if (gsm_command("AT+CIPGSMLOC=1,1","OK",15)) {
    if (strlen(SerialBuffer)>16){
      for (i = 0; i < strlen(SerialBuffer); i++) {
        if (SerialBuffer[i] == ',') {
          if (i1 == 0) {
            i1 = i;
          }
          else if (i2 == 0) {
            i2 = i;
          }
          else {
            i3 = i;
            break;
          }
        }
      }
      if (i1 != 0 && i2 != 0 && i3 != 0) {
        strncpy(location,&SerialBuffer[i2+1],i3-i2-1);
        strncpy(&location[i3-i2-1],&SerialBuffer[i1],i2-i1);
        location[i3-i1-1] = '\0';
      }
#ifdef DEBUG
    ;  Serial.print("Location = ");
      Serial.println(location);
#endif
//      if ((longitudeMajor==0)&&(captureBuffer[2]=='-')) longitudeMajor=255;
    }
  }
}

void gsm_get_battery(byte *percent,int *millivolts) {
  byte charging;  
  char *ptr1;

  if (gsm_command("AT+CBC","OK",2)) {
    memset(radio_buff,0,10);
    charging = 0;
    *percent = 0;
    *millivolts = 0;
// Состояние зарядки    
    ptr1 = strchr(SerialBuffer,',');
    if (ptr1 > 0) {
      strncpy(radio_buff,ptr1-1,1);
      charging = atoi(radio_buff);
// Процент зарядки    
      strncpy(radio_buff,ptr1+1,2);
      if (ptr1[3] != ',') {
        radio_buff[2] = ptr1[3];
      }  
      *percent = atoi(radio_buff);
// Напряжение аккумулятора    
      ptr1 = strchr(ptr1+1,',');
      if (ptr1 > 0) {
        strncpy(radio_buff,ptr1+1,4);
        *millivolts = atoi(radio_buff);
      }  
    }
//    sscanf(&SerialBuffer[8],"%d,%d,%d",&charging,percent,millivolts);
#ifdef DEBUG
    sprintf(radio_buff,"Charg=%d",charging);
    Serial.println(radio_buff);
    sprintf(radio_buff,"%=%d",*percent);
    Serial.println(radio_buff);
    sprintf(radio_buff,"mv=%d",*millivolts);
    Serial.println(radio_buff);
#endif
  }  
}
#endif

void setup() {
    
  esp_deep_sleep_wakeup_cause_t wake_up;
#ifdef DEBUG
  byte b1;
#endif

#ifdef DEBUG
  Serial.begin(115200);
  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB port only
  }
#endif

  pinMode(SS_PIN, OUTPUT);
  pinMode(LEN_PIN, OUTPUT);
  pinMode(GDO0_PIN, INPUT);
  pinMode(BAT_PIN, INPUT);
#ifdef GSM_MODEM
  pinMode(DTR_PIN, OUTPUT);
  pinMode(RST_PIN, OUTPUT);
  digitalWrite(RST_PIN, HIGH);
//  mySerial.begin(9600);
  mySerial.begin(9600,SERIAL_8N1,RX_PIN,TX_PIN);
#endif
  
  // initialize digital pin LED_BUILTIN as an output.
#ifdef INT_BLINK_LED
  pinMode(LED_BUILTIN, OUTPUT);
#endif
#ifdef EXT_BLINK_LED
  pinMode(RED_LED_PIN, OUTPUT);
  pinMode(YELLOW_LED_PIN, OUTPUT);
#endif

  loadSettingsFromFlash();
#ifdef DEBUG
  Serial.print("Dexcom ID: ");
  Serial.println(dex_tx_id);
  Serial.print("SCK: ");
  Serial.println(SCK_PIN);
  Serial.print("MISO: ");
  Serial.println(MISO_PIN);
  Serial.print("MOSI: ");
  Serial.println(MOSI_PIN);
  Serial.print("SS: ");
  Serial.println(SS_PIN);
#endif

  SPIclient.begin(SCK_PIN,MISO_PIN,MOSI_PIN,SS_PIN);
  SPIclient.setClockDivider(SPI_CLOCK_DIV2);  // max SPI speed, 1/2 F_CLOCK
  digitalWrite(SS_PIN, HIGH);
  
  wake_up = esp_deep_sleep_get_wakeup_cause();
  if (wake_up == ESP_DEEP_SLEEP_WAKEUP_TIMER) {
#ifdef DEBUG
    Serial.println("I'am wake up by timer");      
#endif
    first_start_app = false;
#ifdef GSM_MODEM
    digitalWrite(DTR_PIN, HIGH); // Модем должен спать
// Считаем что с модемом все хорошо    
    gsm_availible = settings.use_gsm; 
    internet_availible = settings.use_gsm;       
    modem_availible = true; 
    modem_sleeping = true;
#endif
    web_server_start_time = 0;  
    PrepareBlueTooth();
    if (settings.bt_format > 0) {
      wake_up_time = WAKEUP_BT_TIME;        
      delay(wake_up_time - millis() - WAKEUP_NON_BT_TIME); // До следующего сигнала еще долго. Надо подождать.
    }  
    else {
      wake_up_time = WAKEUP_NON_BT_TIME;      
    }
    mesure_battery();   
#ifdef USE_FREQEST    
    loadOffsetFromFlash(); // Считываем текущие смещения частоты и постояной памяти
#endif
  } 
  else {
#ifdef DEBUG
    Serial.println("It's not wake up. It was turn ON!");      
#endif
    first_start_app = true;
    init_CC2500();  // initialise CC2500 registers
#ifdef DEBUG
    Serial.print("CC2500 PARTNUM=");
    b1 = ReadStatus(PARTNUM);
    Serial.println(b1,HEX);
    Serial.print("CC2500 VERSION=");
    b1 = ReadStatus(VERSION);
    Serial.println(b1,HEX);
#endif
#ifdef USE_FREQEST    
    saveOffsetToFlash(); // Сохраняем смещения частоты по умолчанию в постоянной памяти
#endif
    PrepareWebServer();

#ifdef DEBUG
    Serial.println("Wait two minutes or configure device!");
#endif
  }
  current_channel = -1;
  packet_catched = false;
  len_pin_low = true;
  gdo0_status = 0;
  attachInterrupt(digitalPinToInterrupt(GDO0_PIN),gdo0_pin_up , RISING);
}

void swap_channel(unsigned long channel, byte newFSCTRL0) {

  SendStrobe(SIDLE);
  SendStrobe(SFRX);
#ifdef DEBUG
  Serial.print("FSCTRL0 = ");
  Serial.println(newFSCTRL0,HEX);
#endif
#ifdef USE_FREQEST    
  WriteReg(FSCTRL0,newFSCTRL0);
#endif
  WriteReg(CHANNR, channel);
  SendStrobe(SRX);  //RX
  while (ReadStatus(MARCSTATE) != 0x0d) {
    // Подождем пока включится режим приема
  }
}

byte ReadRadioBuffer() {
  byte len;
  byte i;
  byte rxbytes;

  memset (&radio_buff, 0, sizeof (Dexcom_packet));
  len = ReadStatus(RXBYTES);
#ifdef DEBUG
  Serial.print("Bytes in buffer: ");
  Serial.println(len);
#endif
  if (len > 0 && len < 65) {
    for (i = 0; i < len; i++) {
      if (i < sizeof (Dexcom_packet)) {
        radio_buff[i] = ReadReg(RXFIFO);
#ifdef DEBUG
        Serial.print(radio_buff[i],HEX);
        Serial.print("\t");
#endif
      }
    }
    Serial.println();
  }
//  memcpy(&Pkt, &radio_buff[0], sizeof (Dexcom_packet));
  Pkt.len = radio_buff[0];
  memcpy(&Pkt.dest_addr, &radio_buff[1], 4);
  memcpy(&Pkt.src_addr, &radio_buff[5], 4);
  Pkt.port = radio_buff[9];
  Pkt.device_info = radio_buff[10];
  Pkt.txId = radio_buff[11];
  memcpy(&Pkt.raw, &radio_buff[12], 2);
  memcpy(&Pkt.filtered, &radio_buff[14], 2);
  Pkt.battery = radio_buff[16];
  Pkt.unknown = radio_buff[17];
  Pkt.checksum = radio_buff[18];
  Pkt.RSSI = radio_buff[19];
  Pkt.LQI2 = radio_buff[20];
#ifdef DEBUG
  Serial.print("Dexcom ID: ");
  Serial.println(Pkt.src_addr);
#endif
  return len;
}

void mesure_battery() {
  int val;

  val = analogRead(BAT_PIN);
//  val = adc.read(0)  ;
  battery_milivolts = 1000*VREF*val/ANALOG_RESOLUTION;
#ifdef DEBUG
  Serial.print("Analog Read = ");
  Serial.println(val);
  Serial.print("Milivolts = ");
  Serial.println(battery_milivolts);
#endif
  battery_milivolts = battery_milivolts*(10+27)/27;
//  if (val < BATTERY_MINIMUM) val = BATTERY_MINIMUM;
  battery_percent = 100* (val - BATTERY_MINIMUM)/(BATTERY_MAXIMUM - BATTERY_MINIMUM);
  if (battery_percent < 0) battery_percent = 0;
  if (battery_percent > 100) battery_percent = 100;
#ifdef DEBUG
  Serial.print("Battery Milivolts = ");
  Serial.println(battery_milivolts);
  Serial.print("Battery Percent = ");
  Serial.println(battery_percent);
#endif
  if (battery_percent > 0 && battery_percent < 30) low_battery = true;
  else low_battery = false;       
}

byte mesure_temperature() {
  byte tp = temprature_sens_read(); 
  tp = ( tp - 32 )/1.8;
#ifdef DEBUG
  Serial.print("Temprature = ");  
  Serial.println(tp);  
#endif
  return tp;
}

boolean print_wifi_packet() {
  
  HTTPClient http;
  int httpCode;
  String response;
  byte i;
  String request;
  unsigned long ts;
  boolean ret = false;
  
#ifdef DEBUG
  Serial.print(Pkt.len, HEX);
  Serial.print("\t");
  Serial.print(Pkt.dest_addr, HEX);
  Serial.print("\t");
  Serial.print(Pkt.src_addr, HEX);
  Serial.print("\t");
  Serial.print(Pkt.port, HEX);
  Serial.print("\t");
  Serial.print(Pkt.device_info, HEX);
  Serial.print("\t");
  Serial.print(Pkt.txId, HEX);
  Serial.print("\t");
  Serial.print(dex_num_decoder(Pkt.raw));
  Serial.print("\t");
  Serial.print(dex_num_decoder(Pkt.filtered)*2);
  Serial.print("\t");
  Serial.print(Pkt.battery, HEX);
  Serial.print("\t");
  Serial.print(Pkt.unknown, HEX);
  Serial.print("\t");
  Serial.print(Pkt.checksum, HEX);
  Serial.print("\t");
  Serial.print(Pkt.RSSI, HEX);
  Serial.print("\t");
  Serial.print(Pkt.LQI2, HEX);
  Serial.println(" OK");
#endif

  if (strlen(settings.wifi_ssid) == 0) {
#ifdef DEBUG
    Serial.println("WiFi not configred!");
#endif
    return ret;
  }
#ifdef INT_BLINK_LED    
  digitalWrite(LED_BUILTIN, HIGH);
#endif
#ifdef EXT_BLINK_LED    
  digitalWrite(RED_LED_PIN, HIGH);
#endif
    // wait for WiFi connection
  i = 0;  
  WiFi.begin(settings.wifi_ssid,settings.wifi_pwd);
#ifdef DEBUG
    Serial.print("Connecting WiFi: ");
#endif
  while (WiFi.status() != WL_CONNECTED) {
#ifdef DEBUG
    Serial.print(".");
#endif
    delay(500);
    i++;
    if (i == wifi_wait_tyme*2) break;
    if (WiFi.status() == WL_NO_SSID_AVAIL) break;
  }
#ifdef DEBUG
  Serial.println();
#endif
  if((WiFi.status() == WL_CONNECTED)) {
/*    
    sprintf(radio_buff,"%s?rr=%lu&zi=%lu&pc=%s&lv=%lu&lf=%lu&db=%hhu&ts=%lu&bp=%d&bm=%d&ct=%d&gl=%s\" ",my_webservice_url,millis(),dex_tx_id,my_password_code,
                                                                                                        dex_num_decoder(Pkt.raw),dex_num_decoder(Pkt.filtered)*2,
                                                                                                        Pkt.battery,millis()-catch_time,0, 0, 37, "");         
                                                                                                        
    sprintf(radio_buff,"%s?rr=%lu&zi=%lu&pc=%s&lv=%lu&lf=%lu&db=%hhu&ts=%lu&bp=%d&bm=%d&ct=%d",my_webservice_url,millis(),dex_tx_id,my_password_code,
                                                                                                        dex_num_decoder(Pkt.raw),dex_num_decoder(Pkt.filtered)*2,
                                                                                                        Pkt.battery,millis()-catch_time,0, 0, 37);         
*/                       
    byte tp = mesure_temperature();                                                                                 
    ts = millis()-catch_time;  
    request = settings.http_url;                                                                                                      
    request = request + "?rr=" + millis() + "&zi=" + dex_tx_id + "&pc=" + settings.password_code +
              "&lv=" + dex_num_decoder(Pkt.raw) + "&lf=" + dex_num_decoder(Pkt.filtered)*2 + "&db=" + Pkt.battery +
              "&ts=" + ts + "&bp=" + battery_percent + "&bm=" + battery_milivolts + "&ct=" + tp; 
    http.begin(request); //HTTP
#ifdef DEBUG
    Serial.println(request);
#endif
    httpCode = http.GET();
    if(httpCode > 0) {
#ifdef DEBUG
      Serial.print("HTTPCODE = ");
      Serial.println(httpCode);
#endif
      if(httpCode == HTTP_CODE_OK) {
        ret = true;
        response = http.getString();
#ifdef DEBUG
        Serial.print("RESPONSE = ");
        Serial.println(response);
#endif
      } else
      {
#ifdef INT_BLINK_LED    
        blink_sequence("0011");
#endif        
#ifdef EXT_BLINK_LED    
        blink_sequence_red("0011");
#endif        
      }
    } else
    {
#ifdef INT_BLINK_LED    
      blink_sequence("0010");
#endif        
#ifdef EXT_BLINK_LED    
      blink_sequence_red("0010");
#endif        
    }
    WiFi.disconnect(true);
  }
  else {
#ifdef INT_BLINK_LED    
    blink_sequence("0001");
#endif   
#ifdef EXT_BLINK_LED    
    blink_sequence_red("0001");
#endif
#ifdef DEBUG
    Serial.print("WiFi CONNECT ERROR = ");
    Serial.println(WiFi.status());
#endif   
  }
#ifdef INT_BLINK_LED    
  digitalWrite(LED_BUILTIN, LOW);
#endif
#ifdef EXT_BLINK_LED    
  digitalWrite(RED_LED_PIN, LOW);
#endif
  return ret;
}

void print_bt_packet() {
  RawRecord msg;  
  byte msg_len;

  if (settings.bt_format == 0) {
    return;
  }
  if (ble_gatts_if == ESP_GATT_IF_NONE ) {
#ifdef DEBUG
    Serial.println("Not connected");
#endif
    return;
  }
//  sprintf(dex_data,"%lu %d %d",275584,battery,3900);
#ifdef INT_BLINK_LED    
  digitalWrite(LED_BUILTIN, HIGH);
#endif
#ifdef EXT_BLINK_LED    
  digitalWrite(RED_LED_PIN, HIGH);
#endif
  if (settings.bt_format == 1) {
    sprintf(radio_buff,"%lu %d %d\r\n",dex_num_decoder(Pkt.raw),Pkt.battery,battery_milivolts);
    msg_len = strlen(radio_buff);
  }  
  else if (settings.bt_format == 2) { 
    msg.cmd_code = 0x00;
    msg.raw = dex_num_decoder(Pkt.raw);
    msg.filtered = dex_num_decoder(Pkt.filtered)*2;
    msg.dex_battery = Pkt.battery;
    msg.my_battery = battery_percent;
//    msg.my_battery = 0;
    msg.dex_src_id = Pkt.src_addr;
//    msg.size = sizeof(msg);
    msg.size = 17;
    msg.function = DEXBRIDGE_PROTO_LEVEL; // basic functionality, data packet (with ack), TXID packet, beacon packet (also TXID ack).
//    memcpy(&radio_buff, &msg, sizeof(msg));

    radio_buff[0] = msg.size;
    radio_buff[1] = msg.cmd_code;
    memcpy(&radio_buff[2],&msg.raw , 4);
    memcpy(&radio_buff[6],&msg.filtered , 4);
    radio_buff[10] = msg.dex_battery;
    radio_buff[11] = msg.my_battery;
    memcpy(&radio_buff[12],&msg.dex_src_id , 4);
    radio_buff[16] = msg.function;
    msg_len = sizeof(msg);
  }
   esp_err_t ret = esp_ble_gatts_send_indicate(ble_gatts_if, ble_conn_id, ble_attr_handle, msg_len,(uint8_t*)&radio_buff[0], false); 
#ifdef DEBUG
   if (ret == ESP_OK) {
     Serial.println("Send indicate OK");    
   } 
   else {
    Serial.println("Send indicate fail");
   }  
#endif
#ifdef INT_BLINK_LED    
  digitalWrite(LED_BUILTIN, LOW);
#endif
#ifdef EXT_BLINK_LED    
  digitalWrite(RED_LED_PIN, LOW);
#endif
}

#ifdef GSM_MODEM
boolean print_modem_packet() {
  char lastLocation[30];  
  boolean res1;

  gsm_get_location(lastLocation);
//  gsm_get_battery(&batteryPercent, &batteryMillivolts);
  gsm_command("AT+HTTPTERM", "OK", 2); // Завершить сессию на вскяий случай
  gsm_command("AT+HTTPINIT", "OK", 10); // Начинаем http сессию
  gsm_command("AT+HTTPPARA=\"CID\",1", "OK", 2) ;  
  gsm_command("AT+HTTPPARA=\"UA\",\"" my_user_agent "\"", "OK", 2);  // User agent для http запроса
// Адрес сервера паракита
  sprintf(radio_buff,"AT+HTTPPARA=\"URL\",\"%s?rr=%lu&zi=%lu&pc=%s&lv=%lu&lf=%lu&db=%hhu&ts=%lu&bp=%d&bm=%d&ct=%d&gl=%s\" ",settings.http_url,millis(),dex_tx_id,settings.password_code,
                                                                                                                         dex_num_decoder(Pkt.raw),dex_num_decoder(Pkt.filtered)*2,
                                                                                                                         Pkt.battery,millis()-catch_time,battery_percent, battery_milivolts, 
                                                                                                                         37, lastLocation);         
  gsm_command(radio_buff,"OK",2) ;
  res1 = gsm_command("AT+HTTPACTION=0", "+HTTPACTION: 0,200,", 60); // Отправляем запрос на сервер
  gsm_command("AT+HTTPREAD", my_webservice_reply , 20) ;    // Читаем ответ вэб-сервиса
  gsm_command("AT+HTTPTERM", "OK", 2); // Завершаем http сессию
  return res1;
}
#endif

void HandleWebClient() {
  String http_method;  
  String req2;

#ifdef DEBUG
  Serial.println("New client");  //  "Новый клиент"
#endif
  
  while (client.connected()) {
#ifdef EXT_BLINK_LED    
    blink_red_led_half();
#endif
    if (client.available()) {
      String req = client.readStringUntil('\r');
#ifdef DEBUG
      Serial.print("http request = ");  
      Serial.println(req);
#endif
      int addr_start = req.indexOf(' ');
      int addr_end = req.indexOf(' ', addr_start + 1);
      if (addr_start == -1 || addr_end == -1) {
#ifdef DEBUG
        Serial.print("Invalid request: ");
        Serial.println(req);
#endif
        return;
      }
      http_method = req.substring(0,addr_start);
#ifdef DEBUG
      Serial.print("HTTP method = ");
      Serial.println(http_method);
#endif
      if (http_method == "POST") {
        req2 = client.readString();
#ifdef DEBUG
        Serial.print("post request = ");  
        Serial.println(req2);
#endif
      }
      req = req.substring(addr_start + 1, addr_end);
#ifdef DEBUG
      Serial.print("Request: ");
      Serial.println(req);
#endif
      client.flush();
      if (req == "/") {
        handleRoot();
      }
      else if (http_method == "POST" && req == "/save") {
        handleSave(req2);        
      }
      else 
      {
        handleNotFound();
      }
      client.stop();
    }    
  }  
}

void esp32_goto_sleep() {
  unsigned long current_time;
  esp_bluedroid_status_t stat;
  
  stat = esp_bluedroid_get_status();
  if (stat == ESP_BLUEDROID_STATUS_ENABLED) {
    esp_bluedroid_disable();
    delay(500);
    stat = esp_bluedroid_get_status();
  }  
  if (stat == ESP_BLUEDROID_STATUS_INITIALIZED) {
    esp_bluedroid_deinit();
    delay(500);
  }  
  btStop();

#ifdef DEBUG
  Serial.println("Goto sleep!");
  Serial.print("Current time = ");
  Serial.println(millis());
  Serial.print("Next time = ");
  Serial.println(next_time);
#endif
  current_time = millis();
  if (next_time - current_time < wake_up_time) return;
  esp_deep_sleep(( next_time - current_time - wake_up_time)*1000);
  
}

#ifdef GSM_MODEM
void check_sms() {
  if (gsm_availible ) {
#ifdef DEBUG
    Serial.println("gsm_availible. Let's check sms");
    Serial.print("current_channel = ");
    Serial.println(current_channel);
#endif
    if (packet_catched || (current_channel == 0)) {
      if (modem_sleeping) gsm_wake_up(); // Будим GSM-модем
      if (gsm_availible) {    
        read_sms(); // Прочитаем полученные смс-ки
      }  
      gsm_goto_sleep();
    }  
  }    
}
#endif

void loop() {
  unsigned long current_time;
  boolean packet_on_board;
  int old_channel;
  byte packet_len;
  uint8_t freqest;
  int rssi;

// Первые две минуты работает WebServer на адресе 192.168.70.1 для конфигурации устройства
  if (web_server_start_time > 0) {
#ifdef EXT_BLINK_LED    
    blink_red_led_half();
#endif
    client = server.available();
    if (client) {
      HandleWebClient();
    }
    delay(1);
    if ((millis() - web_server_start_time) > TWO_MINUTE && !server.hasClient()) {
      server.stop();
      WiFi.softAPdisconnect(true);
      web_server_start_time = 0;  
#ifdef DEBUG
      Serial.println("Configuration mode is done!");
#endif
      if (settings.bt_format == 0) {
        wake_up_time = WAKEUP_NON_BT_TIME;
      }
      else {
        wake_up_time = WAKEUP_BT_TIME;        
      }
      PrepareBlueTooth();
      delay(500);
      mesure_battery();        
#ifdef GSM_MODEM
      init_GSM(true);
#endif
    }
    return;
  }

// Ловим сигнал от Декскома
  delay(1);
  
  current_time = millis();

  if (len_pin_low) {
    digitalWrite(LEN_PIN, HIGH); // Включаем усилитель слабого сигнала
    len_pin_low = false;
  } // Включаем усилитель слабого сигнала  

  old_channel = current_channel;
  if (current_channel < 0) {
    current_channel = 0;
  }
  if (current_channel > 0 && current_time - cc2500_start_time > waitTimes[current_channel]) {
    current_channel++;
    if (current_channel > 3) {
      current_channel = 0;
#ifdef GSM_MODEM
      check_sms();
#endif
    }
  }  

  if (current_channel != old_channel) {  
    cc2500_start_time = current_time;
    swap_channel(nChannels[current_channel], fOffset[current_channel]);
#ifdef DEBUG
    Serial.print("Chanel = ");
    Serial.print(nChannels[current_channel]);
    Serial.print(" Time = ");
    Serial.println(cc2500_start_time);
#endif
  }
   
#ifdef INT_BLINK_LED
  blink_builtin_led_quarter();
#endif
#ifdef EXT_BLINK_LED
  if (dex_tx_id == 10858926 || dex_tx_id == 0)   // ABCDE
  {
    blink_yellow_led_half();
    if (low_battery) {
      blink_red_led_half2();
    }
  } else
  {
    blink_yellow_led_quarter();
    if (low_battery) {
      blink_red_led_quarter2();
    }
  }  
#endif
  packet_on_board = gdo0_status;
  if (packet_on_board) {
    while (digitalRead(GDO0_PIN) == HIGH) {
    // Идет прием пакета
    }
  }
  
  if (packet_on_board) return; // Нет сигнала от декскома - выходим из цикла

#ifdef INT_BLINK_LED
  digitalWrite(LED_BUILTIN, LOW);
#endif
#ifdef EXT_BLINK_LED
  digitalWrite(YELLOW_LED_PIN, LOW);
  digitalWrite(RED_LED_PIN, LOW);
#endif

  packet_len = ReadRadioBuffer();
  if (packet_len == 0 || packet_len == 21) {
    freqest = ReadStatus(FREQEST);
//    fOffset[current_channel] += freqest;
    fOffset[current_channel] = freqest;
#ifdef USE_FREQEST    
    saveOffsetToFlash(); // Сохраняем смещения частоты по умолчанию в постоянной памяти
#endif
#ifdef DEBUG
    Serial.print("Offset:");
    Serial.println(fOffset[current_channel], HEX);
#endif
  }
  if (Pkt.src_addr == dex_tx_id) {
#ifdef DEBUG
    Serial.print("gdo0_status = ");
    Serial.println(gdo0_status);
    Serial.print("Catched.Ch=");
    Serial.println(nChannels[current_channel]);
    Serial.print("Signal Power = ");
    if (Pkt.RSSI > 127) rssi = (Pkt.RSSI - 256) / 2 - 73;
    else rssi = Pkt.RSSI / 2 - 73;
    Serial.println(rssi);
    Serial.print("Signal Quality = ");
    Serial.println(Pkt.LQI2);
#endif
    catch_time = current_time - 500 * current_channel; // Приводим к каналу 0
    next_time = catch_time + FIVE_MINUTE;
    packet_catched = true;
    digitalWrite(LEN_PIN, LOW); // Выключаем усилитель слабого сигнала
    SendStrobe(SIDLE);  // Переводим приемник в режим ожидания
    SendStrobe(SFRX);  // Очищаем буфер вхоядщих данных
  } 
  else {
    current_channel++;
    if (current_channel > 3) current_channel = 0;
    cc2500_start_time = current_time;
    swap_channel(nChannels[current_channel], fOffset[current_channel]);
#ifdef DEBUG
    Serial.print("Chanel = ");
    Serial.print(nChannels[current_channel]);
    Serial.print(" Time = ");
    Serial.println(cc2500_start_time);
    Serial.print("gdo0_status = ");
    Serial.println(gdo0_status);
#endif
  }
  gdo0_status = 0;
  
  if (packet_catched)
  {
    mesure_battery();
    print_bt_packet();
    delay(500);
    if (!print_wifi_packet () && settings.use_gsm) {
#ifdef GSM_MODEM
      gsm_wake_up(); // Будим GSM-модем
      if (modem_availible) {
        if (!print_modem_packet()) {
          init_GSM(false);
          if (internet_availible) {
            print_modem_packet();
          }  
        }
      }  
#endif      
    }
//  - Отправить пакет по модему, если он не ушел по ВайФай    
  } 
  else if (current_channel == 0) {
    sendBeacon();
    mesure_battery();
  }
#ifdef GSM_MODEM
  check_sms();
#endif
  
  if (packet_catched) {
    esp32_goto_sleep();    
  }
}

void gdo0_pin_up() {
  gdo0_status = 1;  
}

