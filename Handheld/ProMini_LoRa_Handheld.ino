/*
  Arduino PRO MINI 3.3V 8MHz
  RFM96 LoRa
  U-blox NEO-6M
  OLED 0.96" 128x64
  
  Telemetry format:
    $$TT7S,1,15:04:09,47.45485,19.25149,253,0,0,10,34.2*27E1
*/

#include <SPI.h>
#include <math.h>
#include <avr/eeprom.h>

extern "C"{
#include "fonts.h"
}


#define OLED_ADDRESS                0x3C                             // 0x78 - write to 0x3C
#define LORA_RSSI_CONSTANT          -137                             // -137 for RFM9x, -157 for SX127x HF and -164 for SX127x LF
#define LORA_MODE                   0                                // default LoRa mode
#define LORA_FREQUENCY              434.250                          // default LoRa frequency
#define LORA_P_LENGTH               255                              // default LoRa payload length
#define LORA_FREQ_MIN               433.050                          // set according to frequency bands in CZ (410.00 MHz synthesizer frequency range)
#define LORA_FREQ_MAX               434.790                          // set according to frequency bands in CZ (525.00 MHz synthesizer frequency range)


volatile uint8_t pressButton1       = 0;
volatile uint8_t pressButton2       = 0;
volatile unsigned long last_millis1 = 0;
volatile unsigned long last_millis2 = 0;
uint8_t prevButton1                 = 0;
uint8_t prevButton2                 = 0;

uint8_t GPSbuffer[80], GPStime[9], GPSlat[10], GPSlon[11], GPSalt[8], GPSsats[2], GPSfix[1], GPSns[1], GPSew[1];
unsigned long GPS_last_update       = 0;
uint8_t GPShourU                    = 0;
uint8_t GPSminU                     = 0;
uint8_t GPSsecU                     = 0;
float GPSlatF                       = 0.0;
float GPSlonF                       = 0.0;
int32_t GPSaltI                     = 0;
uint8_t GPSsatsU                    = 0;
uint8_t GPS_position                = 0;
uint8_t GPS_time_flag               = 0;
uint8_t GPS_position_flag           = 0;

// NEO-6M (NEO-7M is different)
static uint8_t setGGArate_off[16]   = {0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xF0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFF, 0x23};
static uint8_t setGLLrate_off[16]   = {0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xF0, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x2A};
static uint8_t setGSArate_off[16]   = {0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xF0, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x31};
static uint8_t setGSVrate_off[16]   = {0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xF0, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x02, 0x38};
static uint8_t setRMCrate_off[16]   = {0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xF0, 0x04, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x03, 0x3F};
static uint8_t setVTGrate_off[16]   = {0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xF0, 0x05, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x04, 0x46};
static uint8_t setupPSM_3s[52]      = {0xB5, 0x62, 0x06, 0x3B, 0x2C, 0x00, 0x01, 0x06, 0x00, 0x00, 0x00, 0x90, 0x03, 0x01, 0xB8, 0x0B,
                                       0x00, 0x00, 0x10, 0x27, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x2C, 0x01,
                                       0x00, 0x00, 0x4F, 0xC1, 0x03, 0x00, 0x87, 0x02, 0x00, 0x00, 0xFF, 0x00, 0x00, 0x00, 0x64, 0x40,
                                       0x01, 0x00, 0x6F, 0x4C};
static uint8_t setupPSM_5s[52]      = {0xB5, 0x62, 0x06, 0x3B, 0x2C, 0x00, 0x01, 0x06, 0x00, 0x00, 0x00, 0x98, 0x03, 0x00, 0x88, 0x13,
                                       0x00, 0x00, 0x10, 0x27, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x2C, 0x01,
                                       0x00, 0x00, 0x4F, 0xC1, 0x03, 0x00, 0x86, 0x02, 0x00, 0x00, 0xFE, 0x00, 0x00, 0x00, 0x64, 0x40,
                                       0x01, 0x00, 0x4C, 0xA3};
static uint8_t setPSM[10]           = {0xB5, 0x62, 0x06, 0x11, 0x02, 0x00, 0x08, 0x01, 0x22, 0x92};
static uint8_t setCONT[10]          = {0xB5, 0x62, 0x06, 0x11, 0x02, 0x00, 0x08, 0x00, 0x21, 0x91};
static uint8_t setNAVmode[44]       = {0xB5, 0x62, 0x06, 0x24, 0x24, 0x00, 0xFF, 0xFF, 0x06, 0x03, 0x00, 0x00, 0x00, 0x00, 0x10, 0x27,
                                       0x00, 0x00, 0x05, 0x00, 0xFA, 0x00, 0xFA, 0x00, 0x64, 0x00, 0x2C, 0x01, 0x00, 0x3C, 0x00, 0x00,
                                       0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x52, 0xE8};
static uint8_t requestGPGGA[15]     = "$EIGPQ,GGA*27\r\n";

uint8_t LORA_FIFO[256];
uint8_t LORA_mode                   = LORA_MODE;
uint8_t LORA_explicit_implicit      = 0;
uint8_t LORA_bandwidth              = 0;
uint8_t LORA_error_coding           = 0;
uint8_t LORA_spreading_factor       = 0;
uint8_t LORA_low_data_rate          = 0;
float LORA_frequency                = LORA_FREQUENCY;
float LORA_frequency_error          = 0.0;
int16_t LORA_RSSI_value             = 0;
int16_t LORA_RSSI_packet            = 0;
int8_t LORA_SNR_packet              = 0;
uint16_t LORA_payload_length        = LORA_P_LENGTH;
uint8_t LORA_new_pkt                = 0;
unsigned long LORA_RSSI_update      = 0;

float BW[] = {7.8, 10.4, 15.6, 20.8, 31.25, 41.7, 62.5, 125.0, 250.0, 500.0};
uint8_t EC[] = {0, 5, 6, 7, 8};

uint8_t Bcall[6], Btime[8];
uint32_t BcountU                    = 0;
uint8_t BhourU                      = 0;
uint8_t BminU                       = 0;
uint8_t BsecU                       = 0;
float BlatF                         = 0.0;
float BlonF                         = 0.0;
int32_t BaltI                       = 0;
uint16_t BspeedU                    = 0;
uint16_t BheadU                     = 0;
uint8_t BsatsU                      = 0;
float BinttempF                     = 0.0;
uint8_t B_position                  = 0;
unsigned long B_pkt_millis          = 0;


// SETUP FUNCTION ---------------------------------------------------------------------------------
void setup()
{
  Serial.begin(9600);                                               // Arduino <-> Ublox NEO-6
  I2C_init();                                                       // Arduino <-> OLED

  pinMode(10, OUTPUT);                                              // set up chip select
  digitalWrite(10, HIGH);
  SPI.begin();                                                      // Arduino <-> RFM96W
  
  OLED_init();
  OLED_clear();
  OLED_initial_screen();                                            // show initial screen
  
  pinMode(2, INPUT);                                                  // D2 - External Interrupt for a button
  attachInterrupt(digitalPinToInterrupt(2), buttonPressD2, RISING);   // execute buttonPressD2 when D2 goes HIGH
  pinMode(3, INPUT);                                                  // D3 - External Interrupt for another button
  attachInterrupt(digitalPinToInterrupt(3), buttonPressD3, RISING);   // execute buttonPressD3 when D3 goes HIGH

  GPS_setup_continuous();

  EEPROM_get_settings();

  LORA_power_mode_sleep();                                          // for configuration put the module in SLEEP mode
  LORA_set_mode(LORA_mode);                                         // last mode
  LORA_set_frequency(LORA_frequency);                               // last frequency
  LORA_get_frequency();
  LORA_set_payload_length(LORA_payload_length);                     // last payload length
  LORA_receive_continuous();                                        // start receiving
}


// LOOP FUNCTION ----------------------------------------------------------------------------------
void loop()
{
  unsigned long mil = millis();
  
  // update GPS
  if(mil - GPS_last_update > 3000)
  {
    GPS_send_msg(requestGPGGA, 15);                                       // poll GGA message
    
    while(!Serial.available()) {if(millis() - mil > 800) break;}          // wait for the response, or eventually timeout
    
    uint8_t nB = 0;
    
    while(Serial.available())
    {
      if(nB < 80) GPSbuffer[nB++] = Serial.read();
      else Serial.read();
    }

    if(GPS_parse_GPGGA())
    {
      if(GPS_position_flag) GPS_position = 1;
    
      OLED_update_receiver();
      if(B_position) OLED_update_pkt_age();
      if(B_position && GPS_position) OLED_update_calculations();
    }
    
    GPS_last_update = millis();
  }

  // check USER activity
  if(pressButton1 != prevButton1)                                         // D2 Button pressed
  {
    LORA_power_mode_sleep();
    
    OLED_clear();
    OLED_settings_screen();
    
    while(pressButton1 != 0)                                              // enter settings menu
    {
      if(pressButton1 != prevButton1)                                     // D2 Button pressed
      {
        switch(pressButton1)
        {
          case 0:
            break;
            
          case 1:
            OLED_draw_string_6x8((uint8_t*)" ", 116, 0, 1);               // clear previous arrows
            OLED_draw_string_6x8((uint8_t*)"       ", 86, 2, 7);
            OLED_draw_string_6x8((uint8_t*)"   ", 86, 4, 3);
            OLED_draw_symbol_6x8(1, 116, 0);                              // draw current arrow
            break;
            
          case 2:
            OLED_draw_string_6x8((uint8_t*)" ", 116, 0, 1);               // clear previous arrows
            OLED_draw_string_6x8((uint8_t*)"       ", 86, 2, 7);
            OLED_draw_string_6x8((uint8_t*)"   ", 86, 4, 3);
            OLED_draw_symbol_6x8(1, 86, 2);                               // draw current arrow
            break;
            
          case 3:
            OLED_draw_string_6x8((uint8_t*)" ", 116, 0, 1);               // clear previous arrows
            OLED_draw_string_6x8((uint8_t*)"       ", 86, 2, 7);
            OLED_draw_string_6x8((uint8_t*)"   ", 86, 4, 3);
            OLED_draw_symbol_6x8(1, 92, 2);                               // draw current arrow
            break;
            
          case 4:
            OLED_draw_string_6x8((uint8_t*)" ", 116, 0, 1);               // clear previous arrows
            OLED_draw_string_6x8((uint8_t*)"       ", 86, 2, 7);
            OLED_draw_string_6x8((uint8_t*)"   ", 86, 4, 3);
            OLED_draw_symbol_6x8(1, 98, 2);                               // draw current arrow
            break;
            
          case 5:
            OLED_draw_string_6x8((uint8_t*)" ", 116, 0, 1);               // clear previous arrows
            OLED_draw_string_6x8((uint8_t*)"       ", 86, 2, 7);
            OLED_draw_string_6x8((uint8_t*)"   ", 86, 4, 3);
            OLED_draw_symbol_6x8(1, 110, 2);                              // draw current arrow
            break;
            
          case 6:
            OLED_draw_string_6x8((uint8_t*)" ", 116, 0, 1);               // clear previous arrows
            OLED_draw_string_6x8((uint8_t*)"       ", 86, 2, 7);
            OLED_draw_string_6x8((uint8_t*)"   ", 86, 4, 3);
            OLED_draw_symbol_6x8(1, 116, 2);                              // draw current arrow
            break;
            
          case 7:
            OLED_draw_string_6x8((uint8_t*)" ", 116, 0, 1);               // clear previous arrows
            OLED_draw_string_6x8((uint8_t*)"       ", 86, 2, 7);
            OLED_draw_string_6x8((uint8_t*)"   ", 86, 4, 3);
            OLED_draw_symbol_6x8(1, 122, 2);                              // draw current arrow
            break;
            
          case 8:
            OLED_draw_string_6x8((uint8_t*)" ", 116, 0, 1);               // clear previous arrows
            OLED_draw_string_6x8((uint8_t*)"       ", 86, 2, 7);
            OLED_draw_string_6x8((uint8_t*)"   ", 86, 4, 3);
            OLED_draw_symbol_6x8(1, 86, 4);                               // draw current arrow
            break;
            
          case 9:
            OLED_draw_string_6x8((uint8_t*)" ", 116, 0, 1);               // clear previous arrows
            OLED_draw_string_6x8((uint8_t*)"       ", 86, 2, 7);
            OLED_draw_string_6x8((uint8_t*)"   ", 86, 4, 3);
            OLED_draw_symbol_6x8(1, 92, 4);                               // draw current arrow
            break;
            
          case 10:
            OLED_draw_string_6x8((uint8_t*)" ", 116, 0, 1);               // clear previous arrows
            OLED_draw_string_6x8((uint8_t*)"       ", 86, 2, 7);
            OLED_draw_string_6x8((uint8_t*)"   ", 86, 4, 3);
            OLED_draw_symbol_6x8(1, 98, 4);                               // draw current arrow
            break;
            
          default:
            break;
        }

      prevButton1 = pressButton1;
      }

      if(pressButton2 != prevButton2)                                     // D3 Button pressed
      {
        uint32_t tem1, tem2;
        
        switch(pressButton1)
        {
          case 1:                                                         // LoRa Mode
            LORA_mode = (LORA_mode + 1) % 10;
            OLED_settings_screen();
            break;
            
          case 2:                                                         // LoRa Frequency 100 MHz
            LORA_frequency = LORA_frequency + 100.0;
            if(LORA_frequency >= 1000.0) LORA_frequency -= 1000.0;
            OLED_update_FREQ_PAY();
            break;

          case 3:                                                         // LoRa Frequency 10 MHz
            tem1 = (uint32_t)LORA_frequency / 100 % 10;
            LORA_frequency = LORA_frequency + 10.0;
            tem2 = (uint32_t)LORA_frequency / 100 % 10;
            if(tem2 > tem1) LORA_frequency -= 100.0;
            OLED_update_FREQ_PAY();
            break;

          case 4:                                                         // LoRa Frequency 1 MHz
            tem1 = (uint32_t)LORA_frequency / 10 % 10;
            LORA_frequency = LORA_frequency + 1.0;
            tem2 = (uint32_t)LORA_frequency / 10 % 10;
            if(tem2 > tem1) LORA_frequency -= 10.0;
            OLED_update_FREQ_PAY();
            break;

          case 5:                                                         // LoRa Frequency 0.1 MHz
            tem1 = (uint32_t)LORA_frequency % 10;
            LORA_frequency = LORA_frequency + 0.1;
            tem2 = (uint32_t)LORA_frequency % 10;
            if(tem2 > tem1) LORA_frequency -= 1.0;
            OLED_update_FREQ_PAY();
            break;

          case 6:                                                         // LoRa Frequency 0.01 MHz
            tem1 = (uint32_t)(LORA_frequency * 10.0) % 10;
            LORA_frequency = LORA_frequency + 0.01;
            tem2 = (uint32_t)(LORA_frequency * 10.0) % 10;
            if(tem2 > tem1) LORA_frequency -= 0.1;
            OLED_update_FREQ_PAY();
            break;

          case 7:                                                         // LoRa Frequency 0.001 MHz
            tem1 = (uint32_t)(LORA_frequency * 100.0) % 10;
            LORA_frequency = LORA_frequency + 0.001;
            tem2 = (uint32_t)(LORA_frequency * 100.0) % 10;
            if(tem2 > tem1) LORA_frequency -= 0.01;
            OLED_update_FREQ_PAY();
            break;

          case 8:                                                         // LoRa Payload Length 100 byte
            LORA_payload_length += 100;
            if(LORA_payload_length > 255) LORA_payload_length = LORA_payload_length % 100;
            OLED_update_FREQ_PAY();
            break;

          case 9:                                                         // LoRa Payload Length 10 byte
            tem1 = LORA_payload_length / 100 % 10;
            LORA_payload_length += 10;
            tem2 = LORA_payload_length / 100 % 10;
            if(tem2 > tem1) LORA_payload_length -= 100;
            if(LORA_payload_length > 255) LORA_payload_length = LORA_payload_length % 10 + 200;
            OLED_update_FREQ_PAY();
            break;

          case 10:                                                        // LoRa Payload Length 1 byte
            tem1 = LORA_payload_length / 10 % 10;
            LORA_payload_length += 1;
            tem2 = LORA_payload_length / 10 % 10;
            if(tem2 > tem1) LORA_payload_length -= 10;
            if(LORA_payload_length > 255) LORA_payload_length = 250;
            OLED_update_FREQ_PAY();
            break;
    
          default:
            break;
        }
        
        prevButton2 = pressButton2;
      }
    }

    prevButton1 = 0;
    pressButton1 = 0;
    prevButton2 = 0;
    pressButton2 = 0;

    if(LORA_payload_length > 255) LORA_payload_length = 255;
    if(LORA_payload_length < 1) LORA_payload_length = 1;

    if(LORA_frequency > LORA_FREQ_MAX) LORA_frequency = LORA_FREQUENCY;
    if(LORA_frequency < LORA_FREQ_MIN) LORA_frequency = LORA_FREQUENCY;
    
    LORA_set_mode(LORA_mode);
    LORA_set_frequency(LORA_frequency);
    LORA_set_payload_length(LORA_payload_length);

    EEPROM_save_settings();
    
    B_position = 0;
    
    OLED_clear();
    OLED_initial_screen();
    
    LORA_receive_continuous();
  }

  // check RECEIVER
  LORA_get_packet_and_parse();

  if(LORA_new_pkt)
  {
    B_position = 1;

    LORA_sample_RSSI();
    
    OLED_update_balloon();
    OLED_update_pkt_info();
    OLED_update_pkt_age();
    if(B_position && GPS_position) OLED_update_calculations();
    LORA_new_pkt = 0;
  }

  // update RSSI
  if(millis() > LORA_RSSI_update)
  {
    LORA_sample_RSSI();
    OLED_update_RSSI();
    LORA_RSSI_update = millis() + 500;
  }
}


// EXTERNAL INTERRUPT -----------------------------------------------------------------------------
/*
  D2 interrupt routine.
*/
void buttonPressD2(void)
{
  unsigned long m = millis();

  if(m - last_millis1 > 200)
  {
    pressButton1 = (pressButton1 + 1) % 11;
    last_millis1 = m;
  }
}


/*
  D3 interrupt routine.
*/
void buttonPressD3(void)
{
  unsigned long m = millis();

  if((m - last_millis2 > 200) && (pressButton1 != 0))
  {
    pressButton2 = (pressButton2 + 1);
    last_millis2 = m;
  }
}


// EEPROM functions -------------------------------------------------------------------------------
/*
  0x00    LORA_Mode             0-9
  0x01    LORA_frequency        31:24       uint32_t 434250 kHz
  0x02    LORA_frequency        23:16
  0x03    LORA_frequency        15:8
  0x04    LORA_frequency        7:0
  0x05    LORA_payload_length   0-255
*/


/*
  Retrieves the last LoRa settings from EEPROM.
  Used to remember settings on power-up.
*/
void EEPROM_get_settings(void)
{
  // LoRa Mode
  LORA_mode = eeprom_read_byte((uint8_t*)0x00);
  if(LORA_mode > 9) LORA_mode = LORA_MODE;

  // LoRa Frequency
  uint32_t frq;

  frq = ((uint32_t)eeprom_read_byte((uint8_t*)0x01) << 24);
  frq |= ((uint32_t)eeprom_read_byte((uint8_t*)0x02) << 16);
  frq |= ((uint32_t)eeprom_read_byte((uint8_t*)0x03) << 8);
  frq |= ((uint32_t)eeprom_read_byte((uint8_t*)0x04) << 0);

  LORA_frequency = (float)frq / 1000.0;
  if(LORA_frequency < LORA_FREQ_MIN || LORA_frequency > LORA_FREQ_MAX) LORA_frequency = LORA_FREQUENCY;
  
  // LoRa Payload Length
  LORA_payload_length = eeprom_read_byte((uint8_t*)0x05);
  if(LORA_payload_length < 1 || LORA_payload_length > 255) LORA_payload_length = LORA_P_LENGTH;
}


/*
  Saves the latest LoRa settings to EEPROM.
*/
void EEPROM_save_settings(void)
{
  // LoRa Mode
  eeprom_write_byte((uint8_t*)0x00, LORA_mode);

  // LoRa Frequency
  uint32_t frq;

  frq = (uint32_t)roundf(LORA_frequency * 1000.0);
  eeprom_write_byte((uint8_t*)0x01, (frq >> 24) & 0xFF);
  eeprom_write_byte((uint8_t*)0x02, (frq >> 16) & 0xFF);
  eeprom_write_byte((uint8_t*)0x03, (frq >> 8) & 0xFF);
  eeprom_write_byte((uint8_t*)0x04, (frq >> 0) & 0xFF);

  // LoRa Payload Length
  eeprom_write_byte((uint8_t*)0x05, LORA_payload_length & 0xFF);
}


// I2C functions ----------------------------------------------------------------------------------
/*
  
*/
void I2C_init(void)
{
  TWSR = 0x00;                                      // set the prescaler value to 1
  TWBR = 0x48;                                      // set the division factor for 100kHz clock signal (0x48 -> 16000000/(16+2*72*1)=100000)
  TWCR = (1<<TWEN);                                 // I2C enable
}


/*

*/
void I2C_start(void)
{
  TWCR = (1<<TWINT)|(1<<TWSTA)|(1<<TWEN);           // TWINT - clearing the 'job finished' flag, TWSTA - if the bus is clear become Master, TWEN - I2C enable
  while ((TWCR & (1<<TWINT)) == 0);                 // waiting for the 'job finished' flag
}


/*

*/
void I2C_stop(void)
{
  TWCR = (1<<TWINT)|(1<<TWSTO)|(1<<TWEN);           // TWSTO - generate a Stop condition
}


/*

*/
void I2C_write_byte(uint8_t u8data)
{
  TWDR = u8data;                                    // fill the Data Register
  TWCR = (1<<TWINT)|(1<<TWEN);                      // TWINT - clearing the 'job finished' flag, TWEN - I2C enable
  while ((TWCR & (1<<TWINT)) == 0);                 // waiting for the 'job finished' flag
}


// OLED functions ---------------------------------------------------------------------------------
/*
  keywords:
    SEG (segment) = COL (column) = byte of data (bits represent 8 rows within the column)
    COM = row
    Page = 8 rows of pixels of 128 columns
    Display = 8 pages
*/
void OLED_init(void)
{
  I2C_start();
  I2C_write_byte(OLED_ADDRESS << 1);                // ADDRESS
  I2C_write_byte(0x00);                             // BYTE_CMD_STREAM
  I2C_write_byte(0xAE);                             // DISPLAY_OFF
  I2C_write_byte(0xA8);                             // SET_MUX_RATIO
  I2C_write_byte(0x3F);
  I2C_write_byte(0xD3);                             // SET_DISPLAY_OFFSET
  I2C_write_byte(0x00);
  I2C_write_byte(0x40);                             // SET_DISPLAY_START_LINE
  I2C_write_byte(0xA1);                             // SET_SEGMENT_REMAP
  I2C_write_byte(0xC8);                             // SET_COM_SCAN_MODE
  I2C_write_byte(0xDA);                             // SET_COM_PIN_MAP
  I2C_write_byte(0x12);
  I2C_write_byte(0x81);                             // SET_CONTRAST
  I2C_write_byte(0x7F);
  I2C_write_byte(0xA4);                             // DISPLAY_RAM
  I2C_write_byte(0xA6);                             // DISPLAY NORMAL
  I2C_write_byte(0xD5);                             // SET_DISPLAY_CLK_DIV
  I2C_write_byte(0x80);
  I2C_write_byte(0x8D);                             // SET_CHARGE_PUMP
  I2C_write_byte(0x14);
  I2C_write_byte(0xD9);                             // SET_PRECHARGE
  I2C_write_byte(0x22);
  I2C_write_byte(0xDB);                             // SET_VCOMH_DESELECT
  I2C_write_byte(0x30);
  I2C_write_byte(0x20);                             // SET_MEMORY_ADDR_MODE
  I2C_write_byte(0x00);
  I2C_write_byte(0xAF);                             // DISPLAY_ON
  I2C_stop();
}


/*

*/
void OLED_draw_string_5x7(uint8_t * string, uint8_t column, uint8_t page, uint16_t stringsize)
{
  I2C_start();
  I2C_write_byte(OLED_ADDRESS << 1);                // ADDRESS
  I2C_write_byte(0x00);                             // BYTE_CMD_STREAM
  I2C_write_byte(0x21);                             // SET_COLUMN_ADDRESS
  I2C_write_byte(column);
  I2C_write_byte(0x7F);
  I2C_write_byte(0x22);                             // SET_PAGE_ADDRESS
  I2C_write_byte(page);
  I2C_write_byte(0x07);
  I2C_stop();
  
  I2C_start();
  I2C_write_byte(OLED_ADDRESS << 1);                // ADDRESS
  I2C_write_byte(0x40);                             // BYTE_DATA_STREAM
  
  for(uint8_t i = 0; i < stringsize; i++)
  {
    uint8_t c = *string;
    uint16_t x = (uint16_t)c * 5;
    
    for(uint8_t y = 0; y < 5; y++)
    {
      I2C_write_byte(pgm_read_byte(&font5x7[x]));
      x++;
    }
    
    string++;
  }
  
  I2C_stop();
}


/*

*/
void OLED_draw_string_6x8(uint8_t * string, uint8_t column, uint8_t page, uint16_t stringsize)
{
  I2C_start();
  I2C_write_byte(OLED_ADDRESS << 1);                // ADDRESS
  I2C_write_byte(0x00);                             // BYTE_CMD_STREAM
  I2C_write_byte(0x21);                             // SET_COLUMN_ADDRESS
  I2C_write_byte(column);
  I2C_write_byte(0x7F);
  I2C_write_byte(0x22);                             // SET_PAGE_ADDRESS
  I2C_write_byte(page);
  I2C_write_byte(0x07);
  I2C_stop();
  
  I2C_start();
  I2C_write_byte(OLED_ADDRESS << 1);                // ADDRESS
  I2C_write_byte(0x40);                             // BYTE_DATA_STREAM
  
  for(uint8_t i = 0; i < stringsize; i++)
  {
    uint8_t c = *string - 0x20;
    
    for(uint8_t y = 0; y < 6; y++)
    {
      I2C_write_byte(pgm_read_byte(&font6x8[c][y]));
    }
    
    string++;
  }
  
  I2C_stop();
}


/*

*/
void OLED_draw_string_8x16(uint8_t * string, uint8_t column, uint8_t page, uint16_t stringsize)
{
  uint8_t * stringf = string;
  
  // First Row
  I2C_start();
  I2C_write_byte(OLED_ADDRESS << 1);                // ADDRESS
  I2C_write_byte(0x00);                             // BYTE_CMD_STREAM
  I2C_write_byte(0x21);                             // SET_COLUMN_ADDRESS
  I2C_write_byte(column);
  I2C_write_byte(0x7F);
  I2C_write_byte(0x22);                             // SET_PAGE_ADDRESS
  I2C_write_byte(page);
  I2C_write_byte(0x07);
  I2C_stop();
  
  I2C_start();
  I2C_write_byte(OLED_ADDRESS << 1);                // ADDRESS
  I2C_write_byte(0x40);                             // BYTE_DATA_STREAM
  
  for(uint8_t i = 0; i < stringsize; i++)
  {
    uint8_t c = *stringf - 0x20;
    uint16_t x = (uint16_t)c * 16;
    
    for(uint8_t y = 0; y < 8; y++)
    {
      I2C_write_byte(pgm_read_byte(&font8x16[x]));
      x++;
    }
    
    stringf++;
  }
  
  I2C_stop();

  // Second Row
  I2C_start();
  I2C_write_byte(OLED_ADDRESS << 1);                // ADDRESS
  I2C_write_byte(0x00);                             // BYTE_CMD_STREAM
  I2C_write_byte(0x21);                             // SET_COLUMN_ADDRESS
  I2C_write_byte(column);
  I2C_write_byte(0x7F);
  I2C_write_byte(0x22);                             // SET_PAGE_ADDRESS
  I2C_write_byte(page+1);
  I2C_write_byte(0x07);
  I2C_stop();

  I2C_start();
  I2C_write_byte(OLED_ADDRESS << 1);                // ADDRESS
  I2C_write_byte(0x40);                             // BYTE_DATA_STREAM
  
  for(uint8_t i = 0; i < stringsize; i++)
  {
    uint8_t c = *string - 0x20;
    uint16_t x = (uint16_t)c * 16 + 8;
    
    for(uint8_t y = 0; y < 8; y++)
    {
      I2C_write_byte(pgm_read_byte(&font8x16[x]));
      x++;
    }
    
    string++;
  }
  
  I2C_stop();
}


/*

*/
void OLED_draw_symbol_6x8(uint8_t symbol, uint8_t column, uint8_t page)
{
  I2C_start();
  I2C_write_byte(OLED_ADDRESS << 1);                // ADDRESS
  I2C_write_byte(0x00);                             // BYTE_CMD_STREAM
  I2C_write_byte(0x21);                             // SET_COLUMN_ADDRESS
  I2C_write_byte(column);
  I2C_write_byte(0x7F);
  I2C_write_byte(0x22);                             // SET_PAGE_ADDRESS
  I2C_write_byte(page);
  I2C_write_byte(0x07);
  I2C_stop();
  
  I2C_start();
  I2C_write_byte(OLED_ADDRESS << 1);                // ADDRESS
  I2C_write_byte(0x40);                             // BYTE_DATA_STREAM
  
  for(uint8_t y = 0; y < 6; y++)
  {
    I2C_write_byte(pgm_read_byte(&symbol6x8[symbol][y]));
  }
  
  I2C_stop();
}


/*
  Draws initial screen arrangement.
*/
void OLED_initial_screen(void)
{
  OLED_draw_string_6x8((uint8_t*)" --.---", 0, 1, 7);           // balloon latitude
  OLED_draw_string_6x8((uint8_t*)" ---.---", 42, 1, 8);         // balloon longitude
  OLED_draw_string_6x8((uint8_t*)"-----", 96, 1, 5);            // balloon altitude
  
  OLED_draw_string_6x8((uint8_t*)" --.---", 0, 3, 7);           // receiver latitude
  OLED_draw_string_6x8((uint8_t*)" ---.---", 42, 3, 8);         // receiver longitude
  OLED_draw_string_6x8((uint8_t*)"-----", 96, 3, 5);            // receiver altitude
  
  OLED_draw_string_6x8((uint8_t*)"--------", 0, 5, 8);          // distance between balloon and receiver
  OLED_draw_string_6x8((uint8_t*)"m", 48, 5, 1);                // meters
  OLED_draw_string_6x8((uint8_t*)"---", 60, 5, 3);              // azimuth (from receiver to balloon)
  OLED_draw_string_6x8((uint8_t*)"---", 84, 5, 3);              // elevation (from receiver to balloon)
  OLED_draw_string_6x8((uint8_t*)"---", 108, 5, 3);             // age of the last packet received
  
  OLED_draw_string_6x8((uint8_t*)"--:--:--", 0, 7, 8);          // receiver GPS time
  OLED_draw_string_6x8((uint8_t*)"--.-", 76, 7, 4);             // frequency error in kHz
  OLED_draw_string_6x8((uint8_t*)"---", 54, 7, 3);              // last packet SNR in dB
  OLED_draw_string_6x8((uint8_t*)"----", 104, 7, 4);            // current RSSI in dBm
}


/*

*/
void OLED_settings_screen(void)
{
  uint8_t mode[] = "Mode  ";
  mode[5] = LORA_mode + '0';

  uint32_t fU = (uint32_t)(LORA_frequency * 1000.0);
  uint8_t freq[] = "   .   ";
  freq[0] = fU / 100000 % 10 + '0';
  freq[1] = fU / 10000 % 10 + '0';
  freq[2] = fU / 1000 % 10 + '0';
  freq[4] = fU / 100 % 10 + '0';
  freq[5] = fU / 10 % 10 + '0';
  freq[6] = fU % 10 + '0';

  uint8_t pay[] = "   byte";
  if(LORA_payload_length >= 100) pay[0] = LORA_payload_length / 100 % 10 + '0';
  else pay[0] = ' ';
  if(LORA_payload_length >= 10) pay[1] = LORA_payload_length / 10 % 10 + '0';
  else pay[1] = ' ';
  pay[2] = LORA_payload_length % 10 + '0';
  
  OLED_draw_string_6x8(mode, 86, 1, 6);                         // LoRa mode
  OLED_draw_string_6x8(freq, 86, 3, 7);                         // LoRa frequency
  OLED_draw_string_6x8(pay, 86, 5, 7);                          // LoRa payload length

  switch(LORA_mode)
  {
    case 0:
      OLED_draw_string_6x8((uint8_t*)"explicit", 0, 1, 8);      // header mode
      OLED_draw_string_6x8((uint8_t*)"20.8kHz", 0, 3, 7);       // bandwidth
      OLED_draw_string_6x8((uint8_t*)"4/8", 0, 5, 3);           // error coding
      OLED_draw_string_6x8((uint8_t*)"11", 0, 7, 2);            // spreading factor
      OLED_draw_string_6x8((uint8_t*)"43 b/s   ", 24, 7, 9);    // data rate
      break;
      
    case 1:
      OLED_draw_string_6x8((uint8_t*)"implicit", 0, 1, 8);      // header mode
      OLED_draw_string_6x8((uint8_t*)"20.8kHz", 0, 3, 7);       // bandwidth
      OLED_draw_string_6x8((uint8_t*)"4/5", 0, 5, 3);           // error coding
      OLED_draw_string_6x8((uint8_t*)"6 ", 0, 7, 2);            // spreading factor
      OLED_draw_string_6x8((uint8_t*)"1.5 Kb/s ", 24, 7, 9);    // data rate
      break;

    case 2:
      OLED_draw_string_6x8((uint8_t*)"explicit", 0, 1, 8);      // header mode
      OLED_draw_string_6x8((uint8_t*)"62.5kHz", 0, 3, 7);       // bandwidth
      OLED_draw_string_6x8((uint8_t*)"4/8", 0, 5, 3);           // error coding
      OLED_draw_string_6x8((uint8_t*)"8 ", 0, 7, 2);            // spreading factor
      OLED_draw_string_6x8((uint8_t*)"915 b/s  ", 24, 7, 9);    // data rate
      break;

    case 3:
      OLED_draw_string_6x8((uint8_t*)"explicit", 0, 1, 8);      // header mode
      OLED_draw_string_6x8((uint8_t*)"250kHz ", 0, 3, 7);       // bandwidth
      OLED_draw_string_6x8((uint8_t*)"4/6", 0, 5, 3);           // error coding
      OLED_draw_string_6x8((uint8_t*)"7 ", 0, 7, 2);            // spreading factor
      OLED_draw_string_6x8((uint8_t*)"8.5 Kb/s ", 24, 7, 9);    // data rate
      break;

    case 4:
      OLED_draw_string_6x8((uint8_t*)"implicit", 0, 1, 8);      // header mode
      OLED_draw_string_6x8((uint8_t*)"250kHz ", 0, 3, 7);       // bandwidth
      OLED_draw_string_6x8((uint8_t*)"4/5", 0, 5, 3);           // error coding
      OLED_draw_string_6x8((uint8_t*)"6 ", 0, 7, 2);            // spreading factor
      OLED_draw_string_6x8((uint8_t*)"17.7 Kb/s", 24, 7, 9);    // data rate
      break;

    case 5:
      OLED_draw_string_6x8((uint8_t*)"explicit", 0, 1, 8);      // header mode
      OLED_draw_string_6x8((uint8_t*)"41.7kHz", 0, 3, 7);       // bandwidth
      OLED_draw_string_6x8((uint8_t*)"4/8", 0, 5, 3);           // error coding
      OLED_draw_string_6x8((uint8_t*)"11", 0, 7, 2);            // spreading factor
      OLED_draw_string_6x8((uint8_t*)"104 b/s  ", 24, 7, 9);    // data rate
      break;

    case 6:
      OLED_draw_string_6x8((uint8_t*)"implicit", 0, 1, 8);      // header mode
      OLED_draw_string_6x8((uint8_t*)"41.7kHz", 0, 3, 7);       // bandwidth
      OLED_draw_string_6x8((uint8_t*)"4/5", 0, 5, 3);           // error coding
      OLED_draw_string_6x8((uint8_t*)"6 ", 0, 7, 2);            // spreading factor
      OLED_draw_string_6x8((uint8_t*)"3.0 Kb/s ", 24, 7, 9);    // data rate
      break;

    case 7:
      OLED_draw_string_6x8((uint8_t*)"explicit", 0, 1, 8);      // header mode
      OLED_draw_string_6x8((uint8_t*)"20.8kHz", 0, 3, 7);       // bandwidth
      OLED_draw_string_6x8((uint8_t*)"4/5", 0, 5, 3);           // error coding
      OLED_draw_string_6x8((uint8_t*)"7 ", 0, 7, 2);            // spreading factor
      OLED_draw_string_6x8((uint8_t*)"841 b/s  ", 24, 7, 9);    // data rate
      break;

    case 8:
      OLED_draw_string_6x8((uint8_t*)"implicit", 0, 1, 8);      // header mode
      OLED_draw_string_6x8((uint8_t*)"62.5kHz ", 0, 3, 7);      // bandwidth
      OLED_draw_string_6x8((uint8_t*)"4/5", 0, 5, 3);           // error coding
      OLED_draw_string_6x8((uint8_t*)"6 ", 0, 7, 2);            // spreading factor
      OLED_draw_string_6x8((uint8_t*)"4.4 Kb/s ", 24, 7, 9);    // data rate
      break;

    case 9:
      OLED_draw_string_6x8((uint8_t*)"implicit", 0, 1, 8);      // header mode
      OLED_draw_string_6x8((uint8_t*)"500kHz ", 0, 3, 7);       // bandwidth
      OLED_draw_string_6x8((uint8_t*)"4/5", 0, 5, 3);           // error coding
      OLED_draw_string_6x8((uint8_t*)"6 ", 0, 7, 2);            // spreading factor
      OLED_draw_string_6x8((uint8_t*)"35.5 Kb/s", 24, 7, 9);    // data rate
      break;
      
    default:
      break;
  }
}


/*
  Updates the screen with data from a received packet.
*/
void OLED_update_balloon(void)
{
  uint8_t lat[7], lon[8], alt[5];

  // Latitude
  uint32_t latU;
  if(BlatF < 0.0) latU = 4294967295 - (uint32_t)(BlatF * 1000.0) & 0x7FFFFFFF;
  else latU = (uint32_t)(BlatF * 1000.0);
  
  if(latU >= 10000) lat[1] = latU / 10000 % 10 + '0';
  else lat[1] = ' ';
  lat[2] = latU / 1000 % 10 + '0';
  lat[3] = '.';
  lat[4] = latU / 100 % 10 + '0';
  lat[5] = latU / 10 % 10 + '0';
  lat[6] = latU % 10 + '0';

  if(BlatF < 0.0)
  {
    if(latU >= 10000) lat[0] = '-';
    else {lat[0] = ' '; lat[1] = '-';}
  }
  else
  {
    if(latU >= 10000) lat[0] = ' ';
    else {lat[0] = ' '; lat[1] = ' ';}
  }

  // Longitude
  uint32_t lonU;
  if(BlonF < 0.0) lonU = 4294967295 - (uint32_t)(BlonF * 1000.0) & 0x7FFFFFFF;
  else lonU = (uint32_t)(BlonF * 1000.0);
  
  if(lonU >= 100000) lon[1] = lonU / 100000 % 10 + '0';
  if(lonU >= 10000) lon[2] = lonU / 10000 % 10 + '0';
  lon[3] = lonU / 1000 % 10 + '0';
  lon[4] = '.';
  lon[5] = lonU / 100 % 10 + '0';
  lon[6] = lonU / 10 % 10 + '0';
  lon[7] = lonU % 10 + '0';

  if(BlonF < 0.0)
  {
    if(lonU >= 100000) lon[0] = '-';
    else if(lonU >= 10000) {lon[0] = ' '; lon[1] = '-';}
    else {lon[0] = ' '; lon[1] = ' '; lon[2] = '-';}
  }
  else
  {
    if(lonU >= 100000) lon[0] = ' ';
    else if(lonU >= 10000) {lon[0] = ' '; lon[1] = ' ';}
    else {lon[0] = ' '; lon[1] = ' '; lon[2] = ' ';}
  }

  // Altitude
  if(BaltI <= 0) {alt[0] = ' '; alt[1] = ' '; alt[2] = ' '; alt[3] = ' '; alt[4] = '0';}
  else
  {
    uint32_t altU;
    if(BaltI < 0) altU = 4294967295 - (uint32_t)BaltI & 0x7FFFFFFF;
    else altU = (uint32_t)BaltI;
    
    if(altU >= 10000) alt[0] = altU / 10000 % 10 + '0';
    else alt[0] = ' ';
    if(altU >= 1000) alt[1] = altU / 1000 % 10 + '0';
    else alt[1] = ' ';
    if(altU >= 100) alt[2] = altU / 100 % 10 + '0';
    else alt[2] = ' ';
    if(altU >= 10) alt[3] = altU / 10 % 10 + '0';
    else alt[3] = ' ';
    alt[4] = altU % 10 + '0';
  }
  
  // Draw
  OLED_draw_string_6x8(lat, 0, 1, 7);                           // balloon latitude
  OLED_draw_string_6x8(lon, 42, 1, 8);                          // balloon longitude
  OLED_draw_string_6x8(alt, 96, 1, 5);                          // balloon altitude
}


/*
  Updates the screen with data from GPS.
*/
void OLED_update_receiver(void)
{
  uint8_t lat[7], lon[8], alt[5], tim[8];
  
  // Latitude
  uint32_t latU;
  if(GPSlatF < 0.0) latU = 4294967295 - (uint32_t)(GPSlatF * 1000.0) & 0x7FFFFFFF;
  else latU = (uint32_t)(GPSlatF * 1000.0);
  
  if(latU >= 10000) lat[1] = latU / 10000 % 10 + '0';
  else lat[1] = ' ';
  lat[2] = latU / 1000 % 10 + '0';
  lat[3] = '.';
  lat[4] = latU / 100 % 10 + '0';
  lat[5] = latU / 10 % 10 + '0';
  lat[6] = latU % 10 + '0';

  if(GPSlatF < 0.0)
  {
    if(latU >= 10000) lat[0] = '-';
    else {lat[0] = ' '; lat[1] = '-';}
  }
  else
  {
    if(latU >= 10000) lat[0] = ' ';
    else {lat[0] = ' '; lat[1] = ' ';}
  }
  
  // Longitude
  uint32_t lonU;
  if(GPSlonF < 0.0) lonU = 4294967295 - (uint32_t)(GPSlonF * 1000.0) & 0x7FFFFFFF;
  else lonU = (uint32_t)(GPSlonF * 1000.0);
  
  if(lonU >= 100000) lon[1] = lonU / 100000 % 10 + '0';
  if(lonU >= 10000) lon[2] = lonU / 10000 % 10 + '0';
  lon[3] = lonU / 1000 % 10 + '0';
  lon[4] = '.';
  lon[5] = lonU / 100 % 10 + '0';
  lon[6] = lonU / 10 % 10 + '0';
  lon[7] = lonU % 10 + '0';

  if(GPSlonF < 0.0)
  {
    if(lonU >= 100000) lon[0] = '-';
    else if(lonU >= 10000) {lon[0] = ' '; lon[1] = '-';}
    else {lon[0] = ' '; lon[1] = ' '; lon[2] = '-';}
  }
  else
  {
    if(lonU >= 100000) lon[0] = ' ';
    else if(lonU >= 10000) {lon[0] = ' '; lon[1] = ' ';}
    else {lon[0] = ' '; lon[1] = ' '; lon[2] = ' ';}
  }
  
  // Altitude
  if(GPSaltI <= 0) {alt[0] = ' '; alt[1] = ' '; alt[2] = ' '; alt[3] = ' '; alt[4] = '0';}
  else
  {
    uint32_t altU;
    if(GPSaltI < 0) altU = 4294967295 - (uint32_t)GPSaltI & 0x7FFFFFFF;
    else altU = (uint32_t)GPSaltI;
    
    if(altU >= 10000) alt[0] = altU / 10000 % 10 + '0';
    else alt[0] = ' ';
    if(altU >= 1000) alt[1] = altU / 1000 % 10 + '0';
    else alt[1] = ' ';
    if(altU >= 100) alt[2] = altU / 100 % 10 + '0';
    else alt[2] = ' ';
    if(altU >= 10) alt[3] = altU / 10 % 10 + '0';
    else alt[3] = ' ';
    alt[4] = altU % 10 + '0';
  }
  
  // Time
  tim[0] = GPShourU / 10 % 10 + '0';
  tim[1] = GPShourU % 10 + '0';
  tim[2] = ':';
  tim[3] = GPSminU / 10 % 10 + '0';
  tim[4] = GPSminU % 10 + '0';
  tim[5] = ':';
  tim[6] = GPSsecU / 10 % 10 + '0';
  tim[7] = GPSsecU % 10 + '0';

  if(GPS_position_flag)
  {
    OLED_draw_string_6x8(lat, 0, 3, 7);                         // receiver latitude
    OLED_draw_string_6x8(lon, 42, 3, 8);                        // receiver longitude
    OLED_draw_string_6x8(alt, 96, 3, 5);                        // receiver altitude
  }

  if(GPS_time_flag)
  {
    OLED_draw_string_6x8(tim, 0, 7, 8);                         // receiver GPS time
  }
}


/*
  Updates the screen with data from GPS.
*/
void OLED_update_calculations(void)
{
  uint8_t dist[8], azim[3], elev[3];
  
  // Distance
  float ltB = BlatF / 180.0 * M_PI;
  float lnB = BlonF / 180.0 * M_PI;
  float ltR = GPSlatF / 180.0 * M_PI;
  float lnR = GPSlonF / 180.0 * M_PI;

  float a = sin((ltB - ltR) / 2.0) * sin((ltB - ltR) / 2.0) + cos(ltB) * cos(ltR) * sin((lnB - lnR) / 2.0) * sin((lnB - lnR) / 2.0);
  float c = 2 * atan2(sqrt(a), sqrt(1 - a));
  uint32_t distance = (uint32_t)(6371000.0 * c);

  if(distance >= 10000000) dist[0] = distance / 10000000 % 10 + '0';
  else dist[0] = ' ';
  if(distance >= 1000000) dist[1] = distance / 1000000 % 10 + '0';
  else dist[1] = ' ';
  if(distance >= 100000) dist[2] = distance / 100000 % 10 + '0';
  else dist[2] = ' ';
  if(distance >= 10000) dist[3] = distance / 10000 % 10 + '0';
  else dist[3] = ' ';
  if(distance >= 1000) dist[4] = distance / 1000 % 10 + '0';
  else dist[4] = ' ';
  if(distance >= 100) dist[5] = distance / 100 % 10 + '0';
  else dist[5] = ' ';
  if(distance >= 10) dist[6] = distance / 10 % 10 + '0';
  else dist[6] = ' ';
  dist[7] = distance % 10 + '0';
  
  // Azimuth
  float theta = atan2(sin(lnB - lnR) * cos(ltB), cos(ltR) * sin(ltB) - sin(ltR) * cos(ltB) * cos(lnB - lnR));
  uint16_t azimuth = (uint16_t)fmod(theta / M_PI * 180.0 + 360.0, 360.0);

  if(azimuth >= 100) azim[0] = azimuth / 100 % 10 + '0';
  else azim[0] = ' ';
  if(azimuth >= 10) azim[1] = azimuth / 10 % 10 + '0';
  else azim[1] = ' ';
  azim[2] = azimuth % 10 + '0';

  // Elevation
  float gcdistance = (float)distance;
  float a2 = gcdistance / (2.0 * M_PI * 6371000.0) * (2.0 * M_PI);
  float s = sin(a2) * (6371000.0 + (float)BaltI);
  float c2 = cos(a2) * ((float)BaltI + 6371000.0) - ((float)GPSaltI + 6371000.0);
  float l = sqrt(powf(s, 2.0) + powf(c2, 2.0));
  float y = M_PI / 2.0 - acos((powf(s, 2.0) - powf(c2, 2.0) - powf(l, 2.0)) / (-2.0 * c2 * l));
  float elevation = y / M_PI * 180.0;

  uint8_t el = (uint8_t)fabs(elevation);

  if(el >= 10) elev[1] = el / 10 % 10 + '0';
  elev[2] = el % 10 + '0';
  
  if(elevation < 0.0)
  {
    if(el >= 10)
    {
      elev[0] = '-';
    }
    else
    {
      elev[0] = ' ';
      elev[1] = '-';
    }
    
  }
  else
  {
    if(el >= 10)
    {
      elev[0] = ' ';
    }
    else
    {
      elev[0] = ' ';
      elev[1] = ' ';
    }
  }
  
  OLED_draw_string_6x8(dist, 0, 5, 8);                          // distance between balloon and receiver
  OLED_draw_string_6x8(azim, 60, 5, 3);                         // azimuth (from receiver to balloon)
  OLED_draw_string_6x8(elev, 84, 5, 3);                         // elevation (from receiver to balloon)
}


/*
  
*/
void OLED_update_pkt_age(void)
{
  uint8_t age[3];
  
  // Age
  unsigned long ag = (millis() - B_pkt_millis) / 1000;          // seconds since last packet was received
  
  if(ag < 60)                                                   // ' 1s' to '59s'
  { 
    if(ag >= 10) age[0] = ag / 10 % 10 + '0';
    else age[0] = ' ';
    age[1] = ag % 10 + '0';
    age[2] = 's';
  }
  else if(ag < 3600)                                            // ' 1m' to '59m'
  {
    uint8_t agm = (uint8_t)(ag / 60);
    
    if(agm >= 10) age[0] = agm / 10 % 10 + '0';
    else age[0] = ' ';
    age[1] = agm % 10 + '0';
    age[2] = 'm';
  }
  else                                                          // ' 1h' to '99h'
  {
    uint8_t agh = (uint8_t)(ag / 3600);

    if(agh >= 99)
    {
       age[0] = '9';
       age[1] = '9';
       age[2] = 'h';
    }
    else
    {
      if(agh >= 10) age[0] = agh / 10 % 10 + '0';
      else age[0] = ' ';
      age[1] = agh % 10 + '0';
      age[2] = 'h';
    }
  }
  
  OLED_draw_string_6x8(age, 108, 5, 3);                         // age of the last packet received
}


/*
  
*/
void OLED_update_FREQ_PAY(void)
{
  uint32_t fU = (uint32_t)(LORA_frequency * 1000.0);
  uint8_t freq[] = "   .   ";
  freq[0] = fU / 100000 % 10 + '0';
  freq[1] = fU / 10000 % 10 + '0';
  freq[2] = fU / 1000 % 10 + '0';
  freq[4] = fU / 100 % 10 + '0';
  freq[5] = fU / 10 % 10 + '0';
  freq[6] = fU % 10 + '0';

  uint8_t pay[] = "   byte";
  if(LORA_payload_length >= 100) pay[0] = LORA_payload_length / 100 % 10 + '0';
  else pay[0] = ' ';
  if(LORA_payload_length >= 10) pay[1] = LORA_payload_length / 10 % 10 + '0';
  else pay[1] = ' ';
  pay[2] = LORA_payload_length % 10 + '0';
  
  OLED_draw_string_6x8(freq, 86, 3, 7);                         // LoRa frequency
  OLED_draw_string_6x8(pay, 86, 5, 7);                          // LoRa payload length
}


/*

*/
void OLED_update_pkt_info(void)
{
  uint8_t freqErr[4];
  uint8_t snr[3];
  uint8_t snru;
  int16_t snri;
  uint32_t freqe;

  // Frequency Error
  freqe = (uint32_t)fabs(LORA_frequency_error);

  if(freqe >= 10000) // error >= 10kHz
  {
    if(freqe >= 100000) freqErr[1] = freqe / 100000 % 10 + '0';
    freqErr[2] = freqe / 10000 % 10 + '0';
    freqErr[3] = freqe / 1000 % 10 + '0';
  }
  else    // error < 10kHz
  {
    freqErr[1] = freqe / 1000 % 10 + '0';
    freqErr[2] = '.';
    freqErr[3] = freqe / 100 % 10 + '0';
  }

  if(LORA_frequency_error < 0)
  {
    if(freqe >= 100000) freqErr[0] = '-';
    else if (freqe >= 10000) {freqErr[0] = ' '; freqErr[1] = '-';}
    else freqErr[0] = '-';
  }
  else
  {
    if(freqe >= 100000) freqErr[0] = ' ';
    else if (freqe >= 10000) {freqErr[0] = ' '; freqErr[1] = ' ';}
    else freqErr[0] = ' ';
  }

  // SNR (estimated)
  if(LORA_SNR_packet > 5)
  {
    snri = LORA_RSSI_packet - LORA_RSSI_value;

    if(snri < 0) snru = 128 - (uint8_t)snri & 0x7F;
    else snru = (uint8_t)snri;

    if(snru >= 100) snr[0] = snru / 100 % 10 + '0';
    if(snru >= 10) snr[1] = snru / 10 % 10 + '0';
    snr[2] = snru % 10 + '0';

    if(snri < 0)
    {
      if(snru >= 10) snr[0] = '-';
      else {snr[0] = ' '; snr[1] = '-';}
    }
    else
    {
      if(snru >= 10 && snru < 100) snr[0] = ' ';
      else {snr[0] = ' '; snr[1] = ' ';}
    }
  }
  else
  {
    if(LORA_SNR_packet < 0) snru = 128 - (uint8_t)LORA_SNR_packet & 0x7F;
    else snru = (uint8_t)LORA_SNR_packet;

    if(snru >= 10) snr[1] = snru / 10 % 10 + '0';
    snr[2] = snru % 10 + '0';

    if(LORA_SNR_packet < 0)
    {
      if(snru >= 10) snr[0] = '-';
      else {snr[0] = ' '; snr[1] = '-';}
    }
    else
    {
      if(snru >= 10) snr[0] = ' ';
      else {snr[0] = ' '; snr[1] = ' ';}
    }
  }

  OLED_draw_string_6x8(freqErr, 76, 7, 4);                      // last packet Frequency Error
  OLED_draw_string_6x8(snr, 54, 7, 3);                          // last packet SNR
}


/*

*/
void OLED_update_RSSI(void)
{
  uint8_t rssiA[3];
  uint8_t rssi;
  
  if(LORA_RSSI_value < 0) rssi = 128 - (uint8_t)LORA_RSSI_value & 0x7F;
  else rssi = (uint8_t)LORA_RSSI_value;
  
  if(rssi >= 100) rssiA[1] = rssi / 100 % 10 + '0';
  if(rssi >= 10) rssiA[2] = rssi / 10 % 10 + '0';
  rssiA[3] = rssi % 10 + '0';

  if(LORA_RSSI_value < 0)
  {
    if(rssi >= 100) rssiA[0] = '-';
    else if (rssi >= 10) {rssiA[0] = ' '; rssiA[1] = '-';}
    else {rssiA[0] = ' '; rssiA[1] = ' '; rssiA[2] = '-';}
  }
  else
  {
    if(rssi >= 100) rssiA[0] = ' ';
    else if (rssi >= 10) {rssiA[0] = ' '; rssiA[1] = ' ';}
    else {rssiA[0] = ' '; rssiA[1] = ' '; rssiA[2] = ' ';}
  }

  OLED_draw_string_6x8(rssiA, 104, 7, 4);
}


/*

*/
void OLED_clear(void)
{
  I2C_start();
  I2C_write_byte(OLED_ADDRESS << 1);                // ADDRESS
  I2C_write_byte(0x00);                             // BYTE_CMD_STREAM
  I2C_write_byte(0x21);                             // SET_COLUMN_ADDRESS
  I2C_write_byte(0x00);
  I2C_write_byte(0x7F);
  I2C_write_byte(0x22);                             // SET_PAGE_ADDRESS
  I2C_write_byte(0x00);
  I2C_write_byte(0x07);
  I2C_stop();

  I2C_start();
  I2C_write_byte(OLED_ADDRESS << 1);                // ADDRESS
  I2C_write_byte(0x40);                             // BYTE_DATA_STREAM
  
  for(uint16_t i = 0; i < 1024; i++)
  {
    I2C_write_byte(0x00);
  }
  
  I2C_stop();
}


// GPS functions ----------------------------------------------------------------------------------
/*
  U-blox NEO-6M
    2.7-3.6V
    Crystal
    Active Antenna
    ROM CORE 7.03 (45969) Mar 17 2011 16:18:34
    FW7.03 limitations: only PSM is Cyclic Tracking 1s-3s
    Consumption at 3.3V
      backup        17mA    (active antenna)
      continuous    53mA    (active antenna)
      10Hz rate     62mA    (active antenna)
      PSM 3s        24mA    (active antenna)
    NAV-PVT message doesn't work
      NAV-POSLLH latitude, longitude, altitude
      NAV-SOL fix
      NAV-TIMEUTC time, date
      GGA NMEA
  
  Dynamic Platform Model
    Portable - default, max alt. 12000m, max velocity 310m/s, max vertical velocity 50m/s 
*/


/*

*/
void GPS_send_msg(uint8_t * buff, uint8_t len)
{
  for(uint8_t i = 0; i < len; i++)
  {
    Serial.write(buff[i]);
  }
}


/*
  
*/
void GPS_setup_continuous(void)
{
  GPS_send_msg(setGGArate_off, 16);                 // turn off default NMEA output
  GPS_send_msg(setGLLrate_off, 16);                 // turn off default NMEA output
  GPS_send_msg(setGSArate_off, 16);                 // turn off default NMEA output
  GPS_send_msg(setGSVrate_off, 16);                 // turn off default NMEA output
  GPS_send_msg(setRMCrate_off, 16);                 // turn off default NMEA output
  GPS_send_msg(setVTGrate_off, 16);                 // turn off default NMEA output
  //_delay_ms(100);
  //GPS_send_msg(setNAVmode, 44);                     // set Airborne <1g mode
  _delay_ms(100);
  GPS_send_msg(setCONT, 10);                        // enter Continuous Mode
  _delay_ms(100);
}


/*
  
*/
void GPS_setup_psm(void)
{
  GPS_send_msg(setGGArate_off, 16);                 // turn off default NMEA output
  GPS_send_msg(setGLLrate_off, 16);                 // turn off default NMEA output
  GPS_send_msg(setGSArate_off, 16);                 // turn off default NMEA output
  GPS_send_msg(setGSVrate_off, 16);                 // turn off default NMEA output
  GPS_send_msg(setRMCrate_off, 16);                 // turn off default NMEA output
  GPS_send_msg(setVTGrate_off, 16);                 // turn off default NMEA output
  //_delay_ms(100);
  //GPS_send_msg(setNAVmode, 44);                     // set Airborne <1g mode
  _delay_ms(100);
  GPS_send_msg(setupPSM_3s, 52);                    // setup Cyclic Mode with 5s update period
  GPS_send_msg(setPSM, 10);                         // enter Power Save Mode
  _delay_ms(100);
}


/*
  Example:
    $GPGGA,131036.00,4928.12451,N,01809.05417,E,1,05,1.66,394.8,M,41.2,M,,*5C
*/
uint8_t GPS_parse_GPGGA(void)
{
  uint8_t i;
  
  for(i = 0; i < 9; i++) GPStime[i] = '\0';
  for(i = 0; i < 10; i++) GPSlat[i] = '\0';
  for(i = 0; i < 11; i++) GPSlon[i] = '\0';
  for(i = 0; i < 8; i++) GPSalt[i] = '\0';
  for(i = 0; i < 2; i++) GPSsats[i] = '\0';
  GPSfix[0] = '\0';
  GPSew[0] = '\0';
  GPSns[0] = '\0';
  
  uint8_t j;
  uint8_t k = 1;
  uint8_t l = 0;
  
  for(j = 0; j < 80; j++)
  {
    if(GPSbuffer[0] != '$' || GPSbuffer[3] != 'G' || GPSbuffer[4] != 'G' || GPSbuffer[5] != 'A') return 0;
    
    if(GPSbuffer[j] == ',')
    {
      k++;
      l = 0;
      continue;
    }
    
    switch(k)
    {
      case 2:                                       // Time
        GPStime[l] = GPSbuffer[j];
        l++;
        break;
      case 3:                                       // Latitude
        GPSlat[l] = GPSbuffer[j];
        l++;
        break;
      case 4:                                       // N/S
        GPSns[l] = GPSbuffer[j];
        l++;
        break;
      case 5:                                       // Longitude
        GPSlon[l] = GPSbuffer[j];
        l++;
        break;
      case 6:                                       // E/W
        GPSew[l] = GPSbuffer[j];
        l++;
        break;
      case 7:                                       // Fix
        GPSfix[l] = GPSbuffer[j];
        l++;
        break;
      case 8:                                       // Satelittes
        GPSsats[l] = GPSbuffer[j];
        l++;
        break;
      case 10:                                      // Altitude
        GPSalt[l] = GPSbuffer[j];
        l++;
        break;
      default:
        break;
    }
  }

  GPShourU = 0;
  GPSminU = 0;
  GPSsecU = 0;
  GPSlatF = 0.0;
  GPSlonF = 0.0;
  GPSaltI = 0;
  GPSsatsU = 0;

  if(GPStime[5] != '\0')
  {
    GPShourU = (GPStime[0] - '0') * 10 + (GPStime [1] - '0');
    GPSminU = (GPStime[2] - '0') * 10 + (GPStime [3] - '0');
    GPSsecU = (GPStime[4] - '0') * 10 + (GPStime [5] - '0');
    GPS_time_flag = 1;
  }
  else
  {
    GPS_time_flag = 0;
  }
  
  if(GPSlat[2] != '\0' && GPSlon[2] != '\0' && GPSalt[0] != '\0' && GPSfix[0] == '1')
  {
    uint8_t n, c;

    // Latitude
    uint32_t tlat = 0;
    n = 0;
    
    while(n < 10)
    {
      c = GPSlat[n++];
      if(c == '.') continue;
      tlat *= 10;
      tlat += c - '0';
    }
    
    uint32_t tlatD = tlat % 10000000;
    tlatD = tlatD / 60 * 100;
    GPSlatF = (float)(tlat / 10000000) + (float)tlatD / 10000000.0;
    if(GPSns[0] == 'S') GPSlatF *= -1.0;

    // Longitude
    uint32_t tlon = 0;
    n = 0;
    
    while(n < 11)
    {
      c = GPSlon[n++];
      if(c == '.') continue;
      tlon *= 10;
      tlon += c - '0';
    }

    uint32_t tlonD = tlon % 10000000;
    tlonD = tlonD / 60 * 100;
    GPSlonF = (float)(tlon / 10000000) + (float)tlonD / 10000000.0;
    if(GPSew[0] == 'W') GPSlonF *= -1.0;
    
    // Altitude
    uint8_t neg = 0;
    n = 0;
    c = GPSalt[n++];
    
    while(c != '\0')
    {
      if(n > 8) break;
      if(c == '.') break;
      if(c == '-') {neg = 1; c = GPSalt[n++]; continue;}
      GPSaltI *= 10;
      GPSaltI += c - '0';
      c = GPSalt[n++];
    }
    
    if(neg) GPSaltI *= -1;

    // Satellites
    GPSsatsU = (GPSsats[0] - '0') * 10 + (GPSsats[1] - '0');

    GPS_position_flag = 1;
  }
  else
  {
    GPS_position_flag = 0;
  }

  return 1;
}


// LoRa functions ---------------------------------------------------------------------------------
/*
                         MODE 0       MODE 1      MODE 2      MODE 3      MODE 4      MODE 5      MODE 6      MODE 7      MODE 8      MODE 9
    header mode:         explicit     implicit    explicit    explicit    implicit    explicit    implicit    explicit    implicit    implicit
    bandwidth:           20.8kHz      20.8kHz     62.5kHz     250kHz      250kHz      41.7kHz     41.7kHz     20.8kHz     62.5kHz     500kHz
    error coding:        4/8          4/5         4/8         4/6         4/5         4/8         4/5         4/5         4/5         4/5
    spreading factor:    11           6           8           7           6           11          6           7           6           6
    low data rate:       1            0           0           0           0           0           0           0           0           0

    effective bit rate:  43           1476        915         8509        17738       104         2959        841         4434        35476
    50 byte TX:          11.8s        0.3s        0.5s        0.058s      0.028s      5.1s        0.168s      0.6s        0.112s      0.014s
    256 byte TX:         47.3s        1.4s        2.2s        0.240s      0.115s      19.7s       0.689s      2.4s        0.460s      0.058s


  FREQUENCY BANDS
    MHz     MHz       mW    kHz   % 
    433.05  434.79    10          <10   Czech Republic
    433.05  434.79    1     >250        Czech Republic
    433.05  434.79    10    25          Czech Republic
    868.00  868.60    25          <1    Czech Republic
    869.40  869.65    500   25    <10   Czech Republic
    869.70  870.00    5                 Czech Republic
    434.04  434.79    10    25          UK-OFCOM

  
  PROGRAM FLOW
    select LoRa and OpMode
    setup LoRa mode
    setup LoRa frequency
    select OpMode - TX/RXSINGLE/RXCONTINUOUS
*/


/*
  7-bit address MSB first.
*/
void LORA_register_write(uint8_t reg, uint8_t data)
{
  digitalWrite(10, LOW);                            // select
  SPI.transfer(0x80 | reg);                         // register address
  SPI.transfer(data);                               // data to send
  digitalWrite(10, HIGH);                           // de-select
}


/*
  7-bit address MSB first.
*/
uint8_t LORA_register_read(uint8_t reg)
{
  uint8_t data = 0;
  
  digitalWrite(10, LOW);                            // select
  SPI.transfer(0x7F & reg);                         // register address
  data = SPI.transfer(0x00);                        // data to send
  digitalWrite(10, HIGH);                           // de-select
  
  return data;
}


/*
  To verify communication.
  RFM96W datasheet: 0x11
  SX1276 datasheet: 0x12
*/
uint8_t LORA_get_version(void)
{
  uint8_t data = LORA_register_read(0x42);

  return data;
}


/*
  RegOpMode - 0x01
  (slightly different for LoRa and FSK/OOK)
  
  LongRangeMode (modify only in Sleep mode)
    0 FSK/OOK
    1 LoRa
  ModulationType
    0 FSK
    1 OOK
  LowFrequencyModeOn
    0 High Frequency Mode (access to HF test registers) 
    1 Low Frequency Mode (access to LF test registers)
  Mode
    000  0  Sleep mode                        -switching between LoRa and FSK/OOK in this mode
    001  1  Stadby mode 
    010  2  FS mode TX (FSTx) 
    011  3  Transmitter mode (Tx)             -When activated powers all remaining blocks required for transmit,
                                               ramps the PA, transmits the packet and returns to Standby mode.
    100  4  FS mode RX (FSRx) 
    101  5  Receive continuous (RXCONTINUOUS) -When activated powers all remaining blocks required for reception,
                                               processing all received data until a new user request is made to change operating mode.
    110  6  receive single (RXSINGLE)         -When activated powers all remaining blocks required for reception,
                                               remains in this state until a valid packet has been received and then returns to Standby mode.
    111  7  Channel activity detection (CAD)  -When in CAD mode, the device will check a given channel to detect LoRa preamble signal.
*/
void LORA_set_RegOpMode(uint8_t LongRangeMode, uint8_t ModulationType, uint8_t LowFrequencyModeOn, uint8_t Mode)
{
  uint8_t data = ((LongRangeMode & 0x01) << 7) | ((ModulationType & 0x01) << 5) | ((LowFrequencyModeOn & 0x01) << 3) | (Mode & 0x07);
  LORA_register_write(0x01, data);
}


/*
  Mode 0-8.

  Spreading Factor 6
    Header mode must be implicit.
    0x31[2:0] write to 0b101
    0x37 write to 0x0C

  Signal bandwidth: 
    0000  0  7.8 kHz 
    0001  1  10.4 kHz 
    0010  2  15.6 kHz 
    0011  3  20.8kHz 
    0100  4  31.25 kHz 
    0101  5  41.7 kHz 
    0110  6  62.5 kHz 
    0111  7  125 kHz 
    1000  8  250 kHz 
    1001  9  500 kHz

  Error coding rate 
    001   1  4/5 
    010   2  4/6 
    011   3  4/7 
    100   4  4/8

  0 Explicit Header mode 
  1 Implicit Header mode

  SF rate (expressed as a base-2 logarithm) 
    6   64 chips / symbol 
    7   128 chips / symbol 
    8   256 chips / symbol 
    9   512 chips / symbol 
    10  1024 chips / symbol 
    11  2048 chips / symbol 
    12  4096 chips / symbol

  0 Header indicates CRC off 
  1 Header indicates CRC on

  AgcAutoOn for RX set automatically.
*/
void LORA_set_mode(uint8_t mode)
{
  switch (mode)
  {
    case 0:
      LORA_mode = 0;
      LORA_explicit_implicit = 0;
      LORA_bandwidth = 3;
      LORA_error_coding = 4;
      LORA_spreading_factor = 11;
      LORA_low_data_rate = 1;
      break;
      
    case 1:
      LORA_mode = 1;
      LORA_explicit_implicit = 1;
      LORA_bandwidth = 3;
      LORA_error_coding = 1;
      LORA_spreading_factor = 6;
      LORA_low_data_rate = 0;
      break;

    case 2:
      LORA_mode = 2;
      LORA_explicit_implicit = 0;
      LORA_bandwidth = 6;
      LORA_error_coding = 4;
      LORA_spreading_factor = 8;
      LORA_low_data_rate = 0;
      break;

    case 3:
      LORA_mode = 3;
      LORA_explicit_implicit = 0;
      LORA_bandwidth = 8;
      LORA_error_coding = 2;
      LORA_spreading_factor = 7;
      LORA_low_data_rate = 0;
      break;

    case 4:
      LORA_mode = 4;
      LORA_explicit_implicit = 1;
      LORA_bandwidth = 8;
      LORA_error_coding = 1;
      LORA_spreading_factor = 6;
      LORA_low_data_rate = 0;
      break;

    case 5:
      LORA_mode = 5;
      LORA_explicit_implicit = 0;
      LORA_bandwidth = 5;
      LORA_error_coding = 4;
      LORA_spreading_factor = 11;
      LORA_low_data_rate = 0;
      break;

    case 6:
      LORA_mode = 6;
      LORA_explicit_implicit = 1;
      LORA_bandwidth = 5;
      LORA_error_coding = 1;
      LORA_spreading_factor = 6;
      LORA_low_data_rate = 0;
      break;

    case 7:
      LORA_mode = 7;
      LORA_explicit_implicit = 0;
      LORA_bandwidth = 3;
      LORA_error_coding = 1;
      LORA_spreading_factor = 7;
      LORA_low_data_rate = 0;
      break;

    case 8:
      LORA_mode = 8;
      LORA_explicit_implicit = 1;
      LORA_bandwidth = 6;
      LORA_error_coding = 1;
      LORA_spreading_factor = 6;
      LORA_low_data_rate = 0;
      break;

    case 9:
      LORA_mode = 9;
      LORA_explicit_implicit = 1;
      LORA_bandwidth = 9;
      LORA_error_coding = 1;
      LORA_spreading_factor = 6;
      LORA_low_data_rate = 0;
      break;
      
    default:
      break;
  }

  LORA_register_write(0x1D, (LORA_bandwidth << 4) | (LORA_error_coding << 1) | LORA_explicit_implicit);   // RegModemConfig 1
  LORA_register_write(0x1E, (LORA_spreading_factor << 4) | (0x01 << 2));                                  // RegModemConfig 2
  LORA_register_write(0x26, (LORA_low_data_rate << 3) | 0x04);                                            // RegModemConfig 3
  
  uint8_t reg = LORA_register_read(0x31);
  if(LORA_spreading_factor == 6) reg = (reg & 0xF8) | 0x05;
  else reg = (reg & 0xF8) | 0x03;
  LORA_register_write(0x31, reg);                                                                         // RegDetectOptimize

  if(LORA_spreading_factor == 6) reg = 0x0C;
  else reg = 0x0A;
  LORA_register_write(0x37, reg);                                                                         // RegDetectionThreshold
}


/*
  FREQUENCY
  Modify only in SLEEP or STANDBY.

  fRF = ( Xosc * Frf ) / 2**19

  Xosc = 32000000
  fRF (MHz)
*/
void LORA_set_frequency(float freqMHz)
{
  float Frf = (freqMHz * 524288.0) / 32.0;
  uint32_t Frfvalue = (uint32_t) Frf;

  LORA_register_write(0x06, (Frfvalue >> 16) & 0xFF);
  LORA_register_write(0x07, (Frfvalue >> 8) & 0xFF);
  LORA_register_write(0x08, (Frfvalue >> 0) & 0xFF);
}


/*
  FREQUENCY ERROR (LoRa mode)
  (during receive operation)

  Ferror = (FreqError * 2^24 * BW[kHz]) / (Fxtal * 500)
*/
float LORA_get_frequency_error(float LoRa_bw_khz)
{
  int32_t RegFei = 0;

  RegFei = (((int32_t)LORA_register_read(0x28) & 0x0F) << 16);                                            // RegFeiMsb
  RegFei |= (((int32_t)LORA_register_read(0x29) & 0xFF) << 8);                                            // RegFeiMid
  RegFei |= ((int32_t)LORA_register_read(0x2A) & 0xFF);                                                   // RegFeiLsb

  if(RegFei & 0x80000) RegFei = (RegFei & 0x7FFFF) - 524288;
  else RegFei = RegFei & 0x7FFFF;

  return -((float)RegFei * 0.524288 * (LoRa_bw_khz / 500.0));
}


/*
  RegPaConfig - 0x09

  PaSelect    Mode                                  Power Range     Pout Formula
  0           PA_HF or PA_LF on RFO_HF or RFO_LF    -4 to +15dBm    Pout=Pmax-(15-OutputPower), Pmax=10.8+0.6*MaxPower [dBm]
  1           PA_HP on PA_BOOST, any frequency      +2 to +17dBm    Pout=17-(15-OutputPower) [dBm]

  PA_LF option doesn't seem to transmit anything. Not connected on these boards, perhaps?
  Use only PA_BOOST mode.

  PA_LF   Pmax              
  Pout    0     1     2     3     4     5     6     7
  0       -4.2  -3.6  -3    -2.4  -1.8  -1.2  -0.6  0
  1       -3.2  -2.6  -2    -1.4  -0.8  -0.2  0.4   1
  2       -2.2  -1.6  -1    -0.4  0.2   0.8   1.4   2
  3       -1.2  -0.6  0     0.6   1.2   1.8   2.4   3
  4       -0.2  0.4   1     1.6   2.2   2.8   3.4   4
  5       0.8   1.4   2     2.6   3.2   3.8   4.4   5
  6       1.8   2.4   3     3.6   4.2   4.8   5.4   6
  7       2.8   3.4   4     4.6   5.2   5.8   6.4   7
  8       3.8   4.4   5     5.6   6.2   6.8   7.4   8
  9       4.8   5.4   6     6.6   7.2   7.8   8.4   9
  10      5.8   6.4   7     7.6   8.2   8.8   9.4   10
  11      6.8   7.4   8     8.6   9.2   9.8   10.4  11
  12      7.8   8.4   9     9.6   10.2  10.8  11.4  12
  13      8.8   9.4   10    10.6  11.2  11.8  12.4  13
  14      9.8   10.4  11    11.6  12.2  12.8  13.4  14
  15      10.8  11.4  12    12.6  13.2  13.8  14.4  15  [dBm]

  PA_BOOST 
  Pout  OutputPower
  0     2 dBm
  1     3 dBm
  2     4 dBm
  3     5 dBm
  4     6 dBm
  5     7 dBm
  6     8 dBm
  7     9 dBm
  8     10 dBm
  9     11 dBm
  10    12 dBm
  11    13 dBm
  12    14 dBm
  13    15 dBm
  14    16 dBm
  15    17 dBm
  
  PaSelect
    0 RFO pin. Maximum power of +14 dBm 
    1 PA_BOOST pin. Maximum power of +20 dBm

  MaxPower
    Select max output power: Pmax=10.8+0.6*MaxPower [dBm]

  OutputPower
    Pout=Pmax-(15-OutputPower) if PaSelect = 0 (RFO pins) 
    Pout=17-(15-OutputPower) if PaSelect = 1 (PA_BOOST pin)

  PA_HF and PA_LF are high efficiency amplifiers capable of yielding RF power programmable in 1 dB steps
  from -4 to +14dBm directly into a 50 ohm load with low current consumption. PA_LF covers the lower bands
  (up to 525 MHz), whilst PA_HF will cover the upper bands (from 860 MHz).

  PA_HP (High Power), connected to the PA_BOOST pin, covers all frequency bands that the chip addresses.
  It permits continuous operation at up to +17 dBm and duty cycled operation at up to +20dBm.

  RegOcp - 0x0B

  OcpTrim   IMAX            Imax Formula
  0 to 15   45 to 120 mA    45 + 5*OcpTrim [mA]
  16 to 27  130 to 240 mA   -30 + 10*OcpTrim [mA]
  27+       240 mA          240 mA
  
  The power amplifiers of RFM95/96/97/98(W) are protected against current over supply in adverse RF load
  conditions by the over current protection block. The current limiter value is controlled by the OcpTrim 
  bits in RegOcp.
  Imax sets a limit on the current drain of the Power Amplifier only, hence the maximum current drain of the 
  RFM96/77/78 is equal to Imax + IFS.
  Default 100mA changed to 240mA.
*/
void LORA_set_power(uint8_t PaSelect, uint8_t MaxPower, uint8_t OutputPower)
{
  LORA_register_write(0x0B, 0x3B);
  LORA_register_write(0x09, (PaSelect << 7) | ((MaxPower & 0x07) << 4) | (OutputPower & 0x0F));
}


/*
  RegLna - 0x0C

  LnaGain
    000 reserved 
    001 G1 = highest gain 
    010 G2 = highest gain – 6 dB 
    011 G3 = highest gain – 12 dB 
    100 G4 = highest gain – 24 dB 
    101 G5 = highest gain – 36 dB 
    110 G6 = highest gain – 48 dB 
    111 reserved

  LnaBoostLl
    00 Default LNA current 
    Other Reserved

  LnaBoostHf
    00 Default LNA current 
    11 Boost on, 150% LNA current
*/
void LORA_set_LNA(uint8_t LnaGain, uint8_t LnaBoostLf, uint8_t LnaBoostHf)
{
  uint8_t reg = LORA_register_read(0x26);
  LORA_register_write(0x26, reg & 0b11111011);                                                            // AgcAutoOn = 0, LNA gain set by register LnaGain
  LORA_register_write(0x0C, ((LnaGain & 0x07) << 5) | ((LnaBoostLf & 0x03) << 3) | (LnaBoostHf & 0x03));
}


/*
  In Explicit mode, payload length is part of the Header.
  In Implicit mode, fixed payload length needs to be programmed in both the receiver and transmitter.
  
  Payload length in bytes. A 0 value is not permitted. Maximum is 255 bytes.
*/
void LORA_set_payload_length(uint8_t len)
{
  LORA_register_write(0x22, len);
  LORA_payload_length = len;
}


/*
  
*/
void LORA_power_mode_sleep(void)
{
  LORA_set_RegOpMode(1, 0, 0, 0);                                                                         // SLEEP
}


/*
  
*/
void LORA_power_mode_standby(void)
{
  LORA_set_RegOpMode(1, 0, 0, 1);                                                                         // STANDBY
}


/*
  Gets current RSSI value.
  
  RSSI[dBm] = –164 + RSSI
*/
void LORA_sample_RSSI(void)
{
  LORA_RSSI_value = LORA_RSSI_CONSTANT + (int16_t)LORA_register_read(0x1B);
}


/*
  Get the latest packet's SNR and RSSI.
  
  SNR[dB] = PacketSnr[two's compliment] / 4
  Packet Strength[dBm] = –164 + PacketRssi + (PacketRssi >> 4)              for SNR >= 0
  Packet Strength[dBm] = -164 + PacketRssi + (PacketRssi >> 4) + SNR        for SNR < 0

  "SNR higher than +5dB is meaningless."
  "SNR digs down to -20dB or so with SF12."
  "You just have to consider that SNR is "suffucient" when over 5dB, and then just rely on RSSI."
*/
void LORA_get_packet_SNR(void)
{
  int16_t pktrssi = (int16_t)LORA_register_read(0x1A);                                                    // PacketRssi
  int8_t snr = LORA_register_read(0x19);                                                                  // PacketSnr

  LORA_SNR_packet = snr / 4;                                                                              // dB
  
  if(snr < 0)
  {
    LORA_RSSI_packet = LORA_RSSI_CONSTANT + pktrssi + (pktrssi >> 4) + LORA_SNR_packet;                   // dBm
  }else{
    LORA_RSSI_packet = LORA_RSSI_CONSTANT + pktrssi + (pktrssi >> 4);                                     // dBm
  }
}


/*

*/
void LORA_get_frequency(void)
{
  uint32_t freq;
  
  freq = (uint32_t)LORA_register_read(0x06) << 16;
  freq |= (uint32_t)LORA_register_read(0x07) << 8;
  freq |= (uint32_t)LORA_register_read(0x08);
  
  LORA_frequency = (float)freq * 32.0 / 524288.0;
}


/*
  In continuous receive mode, the modem scans the channel continuously for a preamble. Each time a preamble is detected the modem tracks
  it until the packet is received and then carries on waiting for the next preamble.
  
  If the preamble length exceeds the anticipated value set by the registers RegPreambleMsb and RegPreambleLsb (measured in symbol periods)
  the preamble will be dropped and the search for a preamble restarted. However, this scenario will not be flagged by any interrupt.
  In continuous RX mode, opposite to the single RX mode, the RxTimeout interrupt will never occur and the device will never go in Standby
  mode automatically.
  
  It is also important to note that the demodulated bytes are written in the data buffer memory in the order received. Meaning, the first
  byte of a new packet is written just after the last byte of the preceding packet. The RX modem address pointer is never reset as long as
  this mode is enabled. It is therefore necessary for the companion microcontroller to handle the address pointer to make sure the FIFO
  data buffer is never full.
*/
void LORA_receive_continuous(void)
{
  LORA_set_LNA(1, 0, 0);                                                                                  // LnaGain (G1 = maximum gain)
  
  LORA_register_write(0x12, 0xFF);                                                                        // RegIrqFlags - clear all interrupts
  LORA_register_write(0x0F, 0x00);                                                                        // FifoRxBaseAddr
  LORA_register_write(0x0D, 0x00);                                                                        // FifoAddrPtr

  LORA_set_RegOpMode(1, 0, 0, 5);                                                                         // RXCONTINUOUS
}


/*
  Check whether a packet was received. Parse it. And update all stored variables.
*/
void LORA_get_packet_and_parse(void)
{
  uint8_t reg = LORA_register_read(0x12);
  
  if(reg & 0x40)                                                                                          // RxDone
  {
    if(reg & 0x10)                                                                                        // ValidHeader
    {
      if(!(reg & 0x20))                                                                                   // PayloadCrcError
      {
        reg = LORA_register_read(0x13);                                                                   // RxNbBytes
        LORA_register_write(0x0D, LORA_register_read(0x25) - reg);                                        // set FifoAddrPtr to FifoRxByteAddr - RxNbBytes
        
        for(uint8_t i = 0; i < reg; i++)
        {
          LORA_FIFO[i] = LORA_register_read(0x00);                                                        // read RegFifo
        }
      
        if(reg < 255) LORA_FIFO[reg] = '\0';
        
        if(LORA_FIFO[0] == '$' && LORA_FIFO[1] == '$')                                                    // normal UKHAS telemetry
        {
          for(uint8_t z = 0; z < 6; z++) Bcall[z] = '\0';
          BcountU = 0;
          BhourU = 0;
          BminU = 0;
          BsecU = 0;
          BlatF = 0.0;
          BlonF = 0.0;
          BaltI = 0;
          BspeedU = 0;
          BheadU = 0;
          BsatsU = 0;
          BinttempF = 0.0;
          
          uint8_t field = 0;
          uint8_t n = 0;
          uint32_t latTemp = 0;
          uint32_t lonTemp = 0;
          uint16_t itTemp = 0;
          uint8_t latNeg = 0;
          uint8_t lonNeg = 0;
          uint8_t altNeg = 0;
          uint8_t itNeg = 0;
          
          for(uint8_t y = 2; y < reg; y++)                                                                // step through telemetry array and parse it
          {
            if(LORA_FIFO[y] == ',')
            {
              field++;
              n = 0;
              continue;
            }
            
            switch(field)
            {
              case 0:                                                                                     // callsign
                if(n < 6) Bcall[n++] = LORA_FIFO[y];
                break;
                
              case 1:                                                                                     // count
                BcountU *= 10;
                BcountU += LORA_FIFO[y] - '0';
                break;
                
              case 2:                                                                                     // time
                if(n == 0 || n == 1)
                {
                  BhourU *= 10;
                  BhourU += LORA_FIFO[y] - '0';
                }
                if(n == 3 || n == 4)
                {
                  BminU *= 10;
                  BminU += LORA_FIFO[y] - '0';
                }
                if(n == 6 || n == 7)
                {
                  BsecU *= 10;
                  BsecU += LORA_FIFO[y] - '0';
                }
                Btime[n++] = LORA_FIFO[y];
                break;
                
              case 3:                                                                                     // latitude
                if(n == 0 && LORA_FIFO[y] == '-')
                {
                  latNeg = 1;
                  break;
                }
                if(LORA_FIFO[y] == '.') break;
                latTemp *= 10;
                latTemp += LORA_FIFO[y] - '0';
                break;
                
              case 4:                                                                                     // longitude
                if(n == 0 && LORA_FIFO[y] == '-')
                {
                  lonNeg = 1;
                  break;
                }
                if(LORA_FIFO[y] == '.') break;
                lonTemp *= 10;
                lonTemp += LORA_FIFO[y] - '0';
                break;
                
              case 5:                                                                                     // altitude
                if(n == 0 && LORA_FIFO[y] == '-')
                {
                  altNeg = 1;
                  break;
                }
                BaltI *= 10;
                BaltI += LORA_FIFO[y] - '0';
                break;
                
              case 6:                                                                                     // speed
                BspeedU *= 10;
                BspeedU += LORA_FIFO[y] - '0';
                break;
                
              case 7:                                                                                     // heading
                BheadU *= 10;
                BheadU += LORA_FIFO[y] - '0';
                break;
                
              case 8:                                                                                     // satellites
                BsatsU *= 10;
                BsatsU += LORA_FIFO[y] - '0';
                break;
                
              case 9:                                                                                     // internal temperature
                if(n == 0 && LORA_FIFO[y] == '-')
                {
                  itNeg = 1;
                  break;
                }
                if(LORA_FIFO[y] == '.') break;
                itTemp *= 10;
                itTemp += LORA_FIFO[y] - '0';
                break;
                
              default:                                                                                    // if there are more fields, ignore them
                break;
            }
          }

          BlatF = (float)latTemp / 100000.0;
          BlonF = (float)lonTemp / 100000.0;
          BinttempF = (float)itTemp / 10.0;
          if(latNeg) BlatF *= -1.0;
          if(lonNeg) BlonF *= -1.0;
          if(altNeg) BaltI *= -1;
          if(itNeg) BinttempF *= -1.0;

          B_pkt_millis = millis();

          LORA_new_pkt = 1;                                                                               // signal new packet received and decoded
        }
        else if(LORA_FIFO[0] == 0x66 || LORA_FIFO[0] == 0x67)                                             // SSDV packet
        {
          
        }
        else                                                                                              // other packet
        {
          
        }

        LORA_register_write(0x12, 0xFF);                                                                  // RegIrqFlags - clear all interrupts
        LORA_register_write(0x0F, 0x00);                                                                  // FifoRxBaseAddr
        LORA_register_write(0x0D, 0x00);                                                                  // FifoAddrPtr

        LORA_get_packet_SNR();
        LORA_frequency_error = LORA_get_frequency_error(BW[LORA_bandwidth]);
      }
      else
      {
        LORA_register_write(0x12, 0xFF);                                                                  // RegIrqFlags - clear all interrupts
        LORA_register_write(0x0F, 0x00);                                                                  // FifoRxBaseAddr
        LORA_register_write(0x0D, 0x00);                                                                  // FifoAddrPtr
      }
    }
    else
    {
      LORA_register_write(0x12, 0xFF);                                                                    // RegIrqFlags - clear all interrupts
      LORA_register_write(0x0F, 0x00);                                                                    // FifoRxBaseAddr
      LORA_register_write(0x0D, 0x00);                                                                    // FifoAddrPtr
    }
  }
}













