/*
  Arduino PRO MINI 3.3V 8MHz
  RFM96 LoRa
  U-blox NEO-7M
  OV5640 camera
  AL422B FIFO
  LTC1799 oscillator

  Telemetry format:
    $$TT7L,1,15:04:09,47.45485,19.25149,253,0,0,10,0.0*E7D0
*/

#include <SPI.h>
#include <util/delay.h>
#include <avr/pgmspace.h>

extern "C"
{
  #include "ssdv.h"
  #include "rs8.h"
  #include "OV5640_regs.h"
}


#define LORA_MODE                   4                                // default LoRa mode (0-9)
#define LORA_FREQUENCY              434.250                          // [MHz] default LoRa frequency
#define LORA_POWER                  2                               // [dBm] default output power on PA_BOOST (2-17dBm)
#define LORA_RSSI_CONSTANT          -137                             // -137 for RFM9x, -157 for SX127x HF and -164 for SX127x LF
#define JPEG_RESOLUTION             4                                // 0-8 corresponfing to 320x240, 352x288, 640x480, 800x480, 1024x768, 1280x960, 1600x1200, 2048x1536, 2592x1944
#define JPEG_QUALITY                3                                // 0-16 corresponding to 96.7, 93.7, 87.3, 81.2, 74.8, 68.6, 62.3, 56.2, 50.0, 44.4, 39.9, 36.3, 33.2, 30.7, 28.5, 26.6, 25.8
#define SSDV_QUALITY                4                                // 0-7 corresponding to JPEG quality: 13, 18, 29, 43, 50, 71, 86 and 100
#define SCCB_ADDRESS                0x78                             // OV5640's I2C device address
#define IMG_BUFF_SIZE               32                               // size of the buffer feeding SSDV process (decreased to 32 bytes to save RAM)
#define INTERLEAVE_TELEM            50                               // transmit a telemetry packet every X SSDV packets

//#define SECOND_SLOWER_TRANSMISSION                                   // if second slower transmission is desired, uncomment this
#define LORA_MODE_SLOW              7                                // optional slower transmission
#define LORA_FREQUENCY_SLOW         434.300                          // optional slower transmission
#define LORA_POWER_SLOW             10                               // optional slower transmission

#define AUTOFOCUS                                                    // if autofocus is desired, uncomment this


PROGMEM static const uint8_t callsign[] = "TT7L";                    // maximum of 6 characters


float LORA_frequency                = LORA_FREQUENCY;
uint8_t LORA_mode                   = LORA_MODE;
uint8_t LORA_explicit_implicit      = 0;
uint8_t LORA_bandwidth              = 0;
uint8_t LORA_error_coding           = 0;
uint8_t LORA_spreading_factor       = 0;
uint8_t LORA_low_data_rate          = 0;
uint16_t LORA_payload_length        = 255;
float LORA_frequency_error          = 0.0;
//int16_t LORA_RSSI_value             = 0;
//int16_t LORA_RSSI_packet            = 0;
//int16_t LORA_SNR_packet             = 0;
uint8_t LORA_pwr_out                = LORA_POWER - 2;
uint8_t LORA_pkt[256];

// NEO-7M (NEO-6M is different)
PROGMEM static const uint8_t setGGArate_off[16]   = {0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xF0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x00, 0x24};
PROGMEM static const uint8_t setGLLrate_off[16]   = {0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xF0, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x01, 0x2B};
PROGMEM static const uint8_t setGSArate_off[16]   = {0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xF0, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x02, 0x32};
PROGMEM static const uint8_t setGSVrate_off[16]   = {0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xF0, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x03, 0x39};
PROGMEM static const uint8_t setRMCrate_off[16]   = {0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xF0, 0x04, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x04, 0x40};
PROGMEM static const uint8_t setVTGrate_off[16]   = {0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xF0, 0x05, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x05, 0x47};
PROGMEM static const uint8_t setupPSM_5s[52]      = {0xB5, 0x62, 0x06, 0x3B, 0x2C, 0x00, 0x01, 0x06, 0x00, 0x00, 0x00, 0x98, 0x03, 0x00, 0x88, 0x13,
                                                     0x00, 0x00, 0x10, 0x27, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x2C, 0x01,
                                                     0x00, 0x00, 0x4F, 0xC1, 0x03, 0x00, 0x86, 0x02, 0x00, 0x00, 0xFE, 0x00, 0x00, 0x00, 0x64, 0x40,
                                                     0x01, 0x00, 0x4C, 0xA3};
PROGMEM static const uint8_t setPSM[10]           = {0xB5, 0x62, 0x06, 0x11, 0x02, 0x00, 0x08, 0x01, 0x22, 0x92};
PROGMEM static const uint8_t setCONT[10]          = {0xB5, 0x62, 0x06, 0x11, 0x02, 0x00, 0x08, 0x00, 0x21, 0x91};
PROGMEM static const uint8_t setNAVmode[44]       = {0xB5, 0x62, 0x06, 0x24, 0x24, 0x00, 0xFF, 0xFF, 0x06, 0x03, 0x00, 0x00, 0x00, 0x00, 0x10, 0x27,
                                                     0x00, 0x00, 0x05, 0x00, 0xFA, 0x00, 0xFA, 0x00, 0x64, 0x00, 0x2C, 0x01, 0x00, 0x3C, 0x00, 0x00,
                                                     0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x52, 0xE8};
PROGMEM static const uint8_t requestNAV5[8]       = {0xB5, 0x62, 0x06, 0x24, 0x00, 0x00, 0x2A, 0x84};
PROGMEM static const uint8_t requestPVT[8]        = {0xB5, 0x62, 0x01, 0x07, 0x00, 0x00, 0x08, 0x19};

uint8_t GPShour                     = 0;
uint8_t GPSminute                   = 0;
uint8_t GPSsecond                   = 0;
//uint8_t GPSday                      = 0;
//uint8_t GPSmonth                    = 0;
//uint16_t GPSyear                    = 0;
uint8_t GPSsats                     = 0;
uint8_t GPSfix                      = 0;
uint8_t GPSfix_0107                 = 0;
uint8_t GPSvalidity                 = 0;
uint8_t GPSnavigation               = 0;
uint8_t GPSpowermode                = 0;
uint8_t GPSpowersavemodestate       = 0;
//int32_t GPSgroundspeed              = 0;
//int32_t GPSheading                  = 0;
int32_t GPS_UBX_latitude            = 0;
int32_t GPS_UBX_longitude           = 0;
//float GPS_UBX_latitude_Float        = 0.0;
//float GPS_UBX_longitude_Float       = 0.0;
int32_t GPSaltitude                 = 0;
//uint8_t GPS_buffer[100];

uint8_t pktLen                      = 0;
uint32_t count                      = 0;
uint8_t IMG_res                     = JPEG_RESOLUTION;
uint8_t IMG_quality                 = JPEG_QUALITY;
uint32_t IMG_size                   = 0;
uint32_t IMG_count                  = 1;
uint8_t SSDV_quality                = SSDV_QUALITY;
uint8_t IMG_buff[IMG_BUFF_SIZE];


// SETUP FUNCTION ---------------------------------------------------------------------------------
void setup()
{
  Serial.begin(9600);                                               // Arduino <-> Ublox NEO-7
  I2C_init();                                                       // Arduino <-> OV5640
  
  pinMode(10, OUTPUT);                                              // PB2 set as OUTPUT - SPI Chip Select on D10
  digitalWrite(10, HIGH);                                           // PB2 se HIGH
  SPI.begin();                                                      // Arduino <-> RFM96W
  
  /* SETUP CLOCK SIGNALS OV5640 */
  DDRC |= 0b00001000;                                               // A3 set as OUTPUT - AL RCK pin for manual clocking
  PORTC |= 0b00001000;                                              // A3 set HIGH
  
  /* SETUP IMAGE PINS */
  DDRD &= 0b00000011;                                               // D2-D7 set as INPUT
  DDRB &= 0b11111100;                                               // D8-D9 set as INPUT
  
  /* SETUP AL422 PINS */
  DDRC |= 0b00000111;                                               // A0 set as OUTPUT - /RE pin, A1 set as OUTPUT - WE pin, A2 set as OUTPUT - /RR pin
  PORTC |= 0b00000101;                                              // A0 set HIGH, A2 set HIGH
  PORTC &= 0b11111101;                                              // A1 set LOW
  //                                                                // A6 (ADC6) input - VSYNC and /WR

  /* INITITALIZE AL422 */
  _delay_ms(100);
  PORTC &= 0b11111011;                                              // /RR read reset - set LOW
  _delay_ms(1);
  PORTC |= 0b00000100;                                              // /RR read reset - set HIGH
  //                                                                // /WR automatic - connected to VSYNC
  
  /* SETUP OV5640 PINS */
  ADC_init();                                                       // A6 (ADC6) input - VSYNC and /WR
  //                                                                // PWDN pin hardwired to GND
  //                                                                // HARD RESET pin not connected
  
  OV_setup();                                                       // initialize the camera
  _delay_ms(50);
  OV_resolution_JPEG(IMG_res);
  OV_set_quality(IMG_quality);
  
  GPS_setup_continuous();                                           // setup GPS module
  
  LORA_power_mode_sleep();                                          // for configuration put the module in SLEEP mode
  LORA_set_mode(LORA_mode);                                         // default mode
  LORA_set_frequency(LORA_frequency);                               // default frequency
  LORA_set_payload_length(LORA_payload_length);                     // default payload length
}


// LOOP FUNCTION ----------------------------------------------------------------------------------
void loop()
{
  /* Telemetry Packet */
  pktLen = GPS_get_and_prepare_data(LORA_pkt);

#ifdef SECOND_SLOWER_TRANSMISSION

  uint8_t pktLen_slow = pktLen;                                         // in case fast mode is implicit, slower mode should generaly be explicit

#endif // SECOND_SLOWER_TRANSMISSION
  
  if(LORA_mode == 1 || LORA_mode == 4 || LORA_mode == 6 || LORA_mode == 8 || LORA_mode == 9)
  {
    for(uint16_t i = pktLen; i < 256; i++) {LORA_pkt[i] = ' ';}         // for implicit Mode SSDV
    pktLen = 255;
  }

  LORA_power_mode_standby();
  LORA_transmit_packet(LORA_pkt, pktLen, LORA_mode);
  LORA_power_mode_sleep();

#ifdef SECOND_SLOWER_TRANSMISSION

  LORA_mode = LORA_MODE_SLOW;
  LORA_pwr_out = LORA_POWER_SLOW - 2;
  LORA_frequency = LORA_FREQUENCY_SLOW;
  LORA_set_frequency(LORA_frequency);

  LORA_power_mode_standby();
  LORA_transmit_packet(LORA_pkt, pktLen_slow, LORA_mode);
  LORA_power_mode_sleep();
  
  LORA_mode = LORA_MODE;
  LORA_pwr_out = LORA_POWER - 2;
  LORA_frequency = LORA_FREQUENCY;
  LORA_set_frequency(LORA_frequency);

#endif // SECOND_SLOWER_TRANSMISSION

  /* Image Acquisition */
  uint8_t img;
  
#ifdef AUTOFOCUS

  OV_af_firmware();
  OV_af_focus();

#endif // AUTOFOCUS

  img = AL_check_VSYNC();                                               // check there actualy are images being output
  
  if(img)
  {
    AL_capture_frame();

#ifdef AUTOFOCUS

    OV_af_release();

#endif // AUTOFOCUS
    
    img = AL_check_buffer_for_img();
    img = AL_check_buffer_for_img();                                    // twice to verify (stabilize) the result
  }

#ifdef AUTOFOCUS

  else
  {
    OV_af_release();
  }
  
#endif // AUTOFOCUS

  /* SSDV Process */
  if(img == 1 || img == 2)
  {
    uint8_t ssdvcheck;
    uint8_t imgStr = 1;
    uint8_t c;
    uint8_t jpegStr[2];
    uint16_t SSDVpacket = 0;
    
    AL_read_reset();                                                    // resets the read pointer to address 0
    _delay_us(1);
    
    ssdv_t ssdv;

    /* Image Check with SSDV */
/*    ssdv_enc_init(&ssdv, SSDV_TYPE_NOFEC, callsign, 0, SSDV_quality);
    ssdv_enc_set_buffer(&ssdv, LORA_pkt);
    
    while(1)
    {
      while((c = ssdv_enc_get_packet(&ssdv)) == SSDV_FEED_ME)
      {
        if(imgStr)                                                      // insert the missing bytes at the start of the image
        {
          if(img == 1)
          {
            jpegStr[0] = 0xFF;
            ssdv_enc_feed(&ssdv, jpegStr, 1);
          }
          else  // img == 2
          {
            jpegStr[0] = 0xFF;
            jpegStr[1] = 0xD8;
            ssdv_enc_feed(&ssdv, jpegStr, 2);
          }
          
          imgStr = 0;
          continue;
        }
          
        AL_read_frame_partial(IMG_buff, IMG_BUFF_SIZE);
        ssdv_enc_feed(&ssdv, IMG_buff, IMG_BUFF_SIZE);
      }
      
      if(c == SSDV_EOI)
      {
        ssdvcheck = 1;
        break;
      }
      else if(c != SSDV_OK)
      {
        ssdvcheck = 0;
        break;
      }
    }*/ssdvcheck = 1;

    /* Main SSDV Process */
    if(ssdvcheck)
    {
      imgStr = 1;
      c = 0;
      
      AL_read_reset();                                                  // resets the read pointer to address 0
      _delay_us(1);

      ssdv_enc_init(&ssdv, SSDV_TYPE_NOFEC, callsign, IMG_count++, SSDV_quality);
      ssdv_enc_set_buffer(&ssdv, LORA_pkt);

      while(1)
      {
        /* Interleaved Telemetry */
        if((SSDVpacket % INTERLEAVE_TELEM) == 0)
        {
          pktLen = GPS_get_and_prepare_data(LORA_pkt);

          if(LORA_mode == 1 || LORA_mode == 4 || LORA_mode == 6 || LORA_mode == 8 || LORA_mode == 9)
          {
            for(uint16_t i = pktLen; i < 256; i++) {LORA_pkt[i] = ' ';} // for implicit Mode SSDV
            pktLen = 255;
          }
          
          LORA_power_mode_standby();
          LORA_transmit_packet(LORA_pkt, pktLen, LORA_mode);
          LORA_power_mode_sleep();

#ifdef SECOND_SLOWER_TRANSMISSION

          LORA_mode = LORA_MODE_SLOW;
          LORA_pwr_out = LORA_POWER_SLOW - 2;
          LORA_frequency = LORA_FREQUENCY_SLOW;
          LORA_set_frequency(LORA_frequency);
        
          LORA_power_mode_standby();
          LORA_transmit_packet(LORA_pkt, pktLen_slow, LORA_mode);
          LORA_power_mode_sleep();
          
          LORA_mode = LORA_MODE;
          LORA_pwr_out = LORA_POWER - 2;
          LORA_frequency = LORA_FREQUENCY;
          LORA_set_frequency(LORA_frequency);

#endif // SECOND_SLOWER_TRANSMISSION
        
        }

        /* SSDV Packet */
        while((c = ssdv_enc_get_packet(&ssdv)) == SSDV_FEED_ME)
        {
          if(imgStr)                                                    // insert the missing bytes at the start of the image
          {
            if(img == 1)
            {
              jpegStr[0] = 0xFF;
              ssdv_enc_feed(&ssdv, jpegStr, 1);
            }
            else  // img == 2
            {
              jpegStr[0] = 0xFF;
              jpegStr[1] = 0xD8;
              ssdv_enc_feed(&ssdv, jpegStr, 2);
            }
            
            imgStr = 0;
            continue;
          }
          
          AL_read_frame_partial(IMG_buff, IMG_BUFF_SIZE);
          ssdv_enc_feed(&ssdv, IMG_buff, IMG_BUFF_SIZE);
        }
        
        if(c == SSDV_EOI)
        {
          break;
        }
        else if(c != SSDV_OK)
        {
          break;
        }
  
        for(uint16_t i = 0; i < 256; i++) {LORA_pkt[i] = LORA_pkt[i+1];}
        
        LORA_power_mode_standby();
        LORA_transmit_packet(LORA_pkt, 255, LORA_mode);
        LORA_power_mode_sleep();

        SSDVpacket++;
      }
    }
  }
  else
  {
    _delay_ms(5000);                                                    // in case of failing SSDV delay telemetry transmissions a little
  }
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
  digitalWrite(10, LOW);                            // select - PB2 set LOW
  SPI.transfer(0x80 | reg);                         // register address
  SPI.transfer(data);                               // data to send
  digitalWrite(10, HIGH);                           // de-select - PB2 set HIGH
}


/*
  7-bit address MSB first.
*/
uint8_t LORA_register_read(uint8_t reg)
{
  uint8_t data = 0;

  digitalWrite(10, LOW);                            // select - PB2 set LOW
  SPI.transfer(0x7F & reg);                         // register address
  data = SPI.transfer(0x00);                        // data to send
  digitalWrite(10, HIGH);                           // de-select - PB2 set HIGH
  
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
  
  RSSI[dBm] = –164 + Rssi
*//*
void LORA_sample_RSSI(void)
{
  LORA_RSSI_value = LORA_RSSI_CONSTANT + (int16_t)LORA_register_read(0x1B);                               // Rssi
}
*/

/*
  FIFO can be filled only in STANDBY mode.
*/
void LORA_fill_FIFO(uint8_t * data, uint8_t len)
{
  LORA_register_write(0x0E, 0x00);                                                                        // RegFifoTxBaseAddr
  LORA_register_write(0x0D, 0x00);                                                                        // RegFifoAddrPtr

  PORTB &= 0b11111011;                                                                                    // select - PB2 set LOW
  SPI.transfer(0x80 | 0x00);                                                                              // register address
  for(uint8_t i = 0; i < len; i++) SPI.transfer(data[i]);                                                 // data to send
  PORTB |= 0b00000100;                                                                                    // de-select - PB2 set HIGH
}


/*
  
*/
void LORA_transmit_packet(uint8_t * packet, uint8_t len, uint8_t mode)
{
  LORA_set_mode(mode);
  LORA_register_write(0x22, len);                                                                         // RegPayloadLength
  LORA_payload_length = len;

  LORA_register_write(0x12, 0xFF);                                                                        // RegIrqFlags - clear all interrupts

  LORA_set_frequency(LORA_frequency);
  LORA_set_power(1, 0, LORA_pwr_out);                                                                     // PA_BOOST (+2 to +17dBm, 1dBm steps)
  //LORA_set_power(0, 7, LORA_pwr_out);                                                                     // PA_LF (-4 to +15dBm, finner steps) - PA_LF doesn't work (!)
  LORA_set_LNA(0, 0, 0);                                                                                  // LnaGain = reserved
  
  LORA_set_RegOpMode(1, 0, 0, 1);                                                                         // STANDBY
  
  LORA_fill_FIFO(packet, len);
  LORA_set_RegOpMode(1, 0, 0, 3);                                                                         // TX

  while(!(LORA_register_read(0x12) & 0x08));
  LORA_register_write(0x12, 0x08);                                                                        // clear txDone interrupt
  
  LORA_set_RegOpMode(1, 0, 0, 1);                                                                         // STANDBY
}


// GPS functions ----------------------------------------------------------------------------------
/*
  U-blox NEO-7M
    1.65-3.6V
    Crystal
    Active/Passive Antenna
    ROM CORE 1.00 (59842) Jun 27 2012 17:43:52
    PROTVER 14.00
    Consumption at 3.3V
      backup          11.5mA    22.1mA    (passive/active antenna)
      solution        +3-6mA    +3-6mA
      acquisition     35.5mA    45.8mA    (passive/active antenna)
      continuous      28.5mA    39.1mA    (passive/active antenna)
      10Hz rate       30.3mA    40.5mA    (passive/active antenna)
      solution        +6-10mA   +6-10mA
      PSM 5s          15.0mA    25.7mA    (passive/active antenna)
    NAV-PVT message
  
  Dynamic Platform Model
    Airborne <1g - max alt. 50000m, max velocity 100m/s, max vertical velocity 100m/s

  SBAS
    "It calculates GPS integrity and correction data with RIMS (Ranging and Integrity Monitoring Stations)
    on the ground and uses geostationary satellites to broadcast GPS integrity and correction data
    to GPS users. The correction data is transmitted on the GPS L1 frequency (1575.42 MHz)."
    "By default SBAS is enabled with three prioritized SBAS channels and it will use any received SBAS
    satellites (except for those in test mode) for navigation, ionosphere parameters and corrections."
    "When enabling Power Save Mode, SBAS support can be disabled (UBX-CFG-SBAS) since the receiver
    will be unable to download any SBAS data in this mode."
*/


/*

*/
void GPS_send_msg(uint8_t * buff, uint8_t len)
{
  for(uint8_t i = 0; i < len; i++)
  {
    Serial.write(pgm_read_byte(&buff[i]));
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
  _delay_ms(100);
  GPS_send_msg(setNAVmode, 44);                     // set Airborne <1g mode
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
  _delay_ms(100);
  GPS_send_msg(setNAVmode, 44);                     // set Airborne <1g mode
  _delay_ms(100);
  GPS_send_msg(setupPSM_5s, 52);                    // setup Cyclic Mode with 5s update period
  GPS_send_msg(setPSM, 10);                         // enter Power Save Mode
  _delay_ms(100);
}


/*
  Verifies the checksum of received UBX messages.
*/
uint8_t GPS_verify_checksum(uint8_t *buff, uint8_t len)
{
  uint8_t CRC;
  uint8_t CK_A_real = buff[len-2];
  uint8_t CK_B_real = buff[len-1];
  uint8_t CK_A_comp = 0;
  uint8_t CK_B_comp = 0;
  
  for(uint8_t i = 2; i < len-2; i++)
  {
    CK_A_comp = CK_A_comp + buff[i];
    CK_B_comp = CK_A_comp + CK_B_comp;
  }
  
  if(CK_A_real == CK_A_comp && CK_B_real == CK_B_comp) CRC = 1;
  else CRC = 0;

  return CRC;
}


/*
  UBX-NAV-PVT 0x01 0x07 
    GPSyear                   2016
    GPSmonth                  10
    GPSday                    18
    GPShour                   9
    GPSminute                 14
    GPSsecond                 55
    GPSfix                    3
    GPSpowersavemodestate     0 PSM is not active
                              1 Enabled (an intermediate state before Acquisition state)
                              2 Acquisition
                              3 Tracking
                              4 Power Optimized Tracking
                              5 Inactive
    GPSsats                   7
    GPS_UBX_latitude          494681170
    GPS_UBX_longitude         180910855
    GPS_UBX_latitude_Float    49.4681170
    GPS_UBX_longitude_Float   18.0910855
    GPSaltitude               403
    GPSgroundspeed            10000
    GPSheading                2545641
*/
uint8_t GPS_parse_0107(uint8_t *buff)
{
  if(buff[0] == 0xB5 && buff[1] == 0x62 && buff[2] == 0x01 && buff[3] == 0x07)
  {
    if(GPS_verify_checksum(buff, 92) || GPS_verify_checksum(buff, 100))
    {
      // YEAR, MONTH, DAY
      //GPSyear = (uint16_t)buff[10] | (uint16_t)buff[11] << 8;
      //GPSmonth = buff[12];
      //GPSday = buff[13];
      
      // HOUR, MINUTE, SECONDS
      GPShour = buff[14];
      GPSminute = buff[15];
      GPSsecond = buff[16];
      
      // FIX
      GPSfix = buff[26];
      GPSfix_0107 = buff[27] & 0x01;
      GPSvalidity = buff[17];
      
      // POWER SAVE MODE STATE
      GPSpowersavemodestate = (buff[27] >> 2) & 0x07;
      
      // SATS
      GPSsats = buff[29];
      
      // LONGITUDE, LATITUDE, ALTITUDE ABOVE MEAN SEA LEVEL
      GPS_UBX_longitude = (int32_t)buff[30] | (int32_t)buff[31] << 8 | (int32_t)buff[32] << 16 | (int32_t)buff[33] << 24;
      //GPS_UBX_longitude_Float = (float)GPS_UBX_longitude / 10000000.0;
      
      GPS_UBX_latitude = (int32_t)buff[34] | (int32_t)buff[35] << 8 | (int32_t)buff[36] << 16 | (int32_t)buff[37] << 24;
      //GPS_UBX_latitude_Float = (float)GPS_UBX_latitude / 10000000.0;
      
      GPSaltitude = (int32_t)buff[42] | (int32_t)buff[43] << 8 | (int32_t)buff[44] << 16 | (int32_t)buff[45] << 24;
      GPSaltitude /= 1000;
      
      // GROUND SPEED, HEADING
      //GPSgroundspeed = (int32_t)buff[66] | (int32_t)buff[67] << 8 | (int32_t)buff[68] << 16 | (int32_t)buff[69] << 24;
      //GPSheading = (int32_t)buff[70] | (int32_t)buff[71] << 8 | (int32_t)buff[72] << 16 | (int32_t)buff[73] << 24;;
      
      return 1;
    }
    else
    {
      GPSfix = 0;
      GPSsats = 0;
      return 0;
    }
  }
  else
  {
    GPSfix = 0;
    GPSsats = 0;
    return 0;
  }
}


/*
  UBX-CFG-NAV5 0x06 0x24  
    GPSnavigation   0 portable
                    2 stationary
                    3 pedestrian
                    4 automotive
                    5 sea
                    6 airborne with <1g acceleration
                    7 airborne with <2g acceleration
                    8 airborne with <4g acceleration
                    9 wrist worn watch
*/
uint8_t GPS_parse_0624(uint8_t *buff)
{
  if(buff[0] == 0xB5 && buff[1] == 0x62 && buff[2] == 0x06 && buff[3] == 0x24)
  {
    if(GPS_verify_checksum(buff, 44))
    {
      GPSnavigation = buff[8];
      return 1;
    }
    else
    {
      GPSnavigation = 0;
      return 0;
    }
  }
  else
  {
    GPSnavigation = 0;
    return 0;
  }
}


/*
  Cyclic Redundancy Check function used in CRC16_checksum().
*/
uint16_t crc_xmodem_update(uint16_t crc, uint8_t data)
{
  int i;
  crc = crc ^ ((uint16_t)data << 8);
  
  for (i=0; i<8; i++)
  {
    if (crc & 0x8000) crc = (crc << 1) ^ 0x1021;
    else crc <<= 1;
  }
  
  return crc;
}


/*
  RTTY telemetry CRC calculation. String input. Calculation starts on the data after the initial '$' dollar sings.
*/
uint16_t CRC16_checksum(uint8_t *string, uint32_t len, uint32_t start)
{
  uint32_t i;
  uint16_t crc;
  uint8_t c;
  
  crc = 0xFFFF;
  
  for(i=start; i < len; i++)
  {
    c = string[i];
    crc = crc_xmodem_update(crc, c);
  }
  
  return crc;
}


/*
  Telemetry Format
    $$TT7L,1,15:04:09,47.45485,19.25149,253,0,0,10,0.0*E7D0
*/
uint8_t GPS_construct_telemetry(uint8_t *buff)
{
  uint8_t n = 0;
  uint8_t i = 0;
  
  buff[n++] = '$';
  buff[n++] = '$';

  /* CALLSIGN */
  while(pgm_read_byte(&callsign[i]) != '\0') buff[n++] = pgm_read_byte(&callsign[i++]);
  buff[n++] = ',';

  /* SENTENCE ID */
  if(count >= 1000000000) buff[n++] = count / 1000000000 % 10 + '0';
  if(count >= 100000000) buff[n++] = count / 100000000 % 10 + '0';
  if(count >= 10000000) buff[n++] = count / 10000000 % 10 + '0';
  if(count >= 1000000) buff[n++] = count / 1000000 % 10 + '0';
  if(count >= 100000) buff[n++] = count / 100000 % 10 + '0';
  if(count >= 10000) buff[n++] = count / 10000 % 10 + '0';
  if(count >= 1000) buff[n++] = count / 1000 % 10 + '0';
  if(count >= 100) buff[n++] = count / 100 % 10 + '0';
  if(count >= 10) buff[n++] = count / 10 % 10 + '0';
  buff[n++] = count % 10 + '0';
  buff[n++] = ',';

  /* TIME */
  buff[n++] = GPShour / 10 % 10 + '0';
  buff[n++] = GPShour % 10 + '0';
  buff[n++] = ':';
  buff[n++] = GPSminute / 10 % 10 + '0';
  buff[n++] = GPSminute % 10 + '0';
  buff[n++] = ':';
  buff[n++] = GPSsecond / 10 % 10 + '0';
  buff[n++] = GPSsecond % 10 + '0';
  buff[n++] = ',';

  /* LATITUDE */  
  uint32_t latU;
  if(GPS_UBX_latitude < 0) latU = (4294967295 - (uint32_t)GPS_UBX_latitude & 0x7FFFFFFF) / 100;
  else latU = (uint32_t)GPS_UBX_latitude / 100;

  if(GPS_UBX_latitude < 0) buff[n++] = '-';
  if(latU >= 1000000) buff[n++] = latU / 1000000 % 10 + '0';
  buff[n++] = latU / 100000 % 10 + '0';
  buff[n++] = '.';
  buff[n++] = latU / 10000 % 10 + '0';
  buff[n++] = latU / 1000 % 10 + '0';
  buff[n++] = latU / 100 % 10 + '0';
  buff[n++] = latU / 10 % 10 + '0';
  buff[n++] = latU % 10 + '0';
  buff[n++] = ',';

  /* LONGITUDE */
  uint32_t lonU;
  if(GPS_UBX_longitude < 0) lonU = (4294967295 - (uint32_t)GPS_UBX_longitude & 0x7FFFFFFF) / 100;
  else lonU = (uint32_t)GPS_UBX_longitude / 100;

  if(GPS_UBX_longitude < 0) buff[n++] = '-';
  if(lonU >= 10000000) buff[n++] = lonU / 10000000 % 10 + '0';
  if(lonU >= 1000000) buff[n++] = lonU / 1000000 % 10 + '0';
  buff[n++] = lonU / 100000 % 10 + '0';
  buff[n++] = '.';
  buff[n++] = lonU / 10000 % 10 + '0';
  buff[n++] = lonU / 1000 % 10 + '0';
  buff[n++] = lonU / 100 % 10 + '0';
  buff[n++] = lonU / 10 % 10 + '0';
  buff[n++] = lonU % 10 + '0';
  buff[n++] = ',';
  
  /* ALTITUDE */
  uint32_t altU;
  
  if(GPSaltitude < 0 || GPSaltitude > 99999)
  {
    //for(i = 0; i < 5; i++) {buff[n++] = '0';}
    buff[n++] = '0';
  }
  else
  {    
    if(GPSaltitude < 0) altU = 4294967295 - (uint32_t)GPSaltitude & 0x7FFFFFFF;
    else altU = (uint32_t)GPSaltitude;
    
    if(GPSaltitude >= 10000) buff[n++] = altU / 10000 % 10 + '0';
    if(GPSaltitude >= 1000) buff[n++] = altU / 1000 % 10 + '0';
    if(GPSaltitude >= 100) buff[n++] = altU / 100 % 10 + '0';
    if(GPSaltitude >= 10) buff[n++] = altU / 10 % 10 + '0';
    buff[n++] = altU % 10 + '0';
  }

  buff[n++] = ',';

  /* SPEED */
  //buff[n++] = GPSnavigation % 10 + '0';
  buff[n++] = '0';
  buff[n++] = ',';

  /* DIRECTION */
  buff[n++] = '0';
  buff[n++] = ',';

  /* SATELLITES */
  if(GPSsats >= 10) buff[n++] = GPSsats / 10 % 10 + '0';
  buff[n++] = GPSsats % 10 + '0';
  buff[n++] = ',';

  /* TEMPERATURE */
  buff[n++] = '0';
  buff[n++] = '.';
  buff[n++] = '0';
    
  /* CRC */
  uint8_t ax;
  uint16_t crc = CRC16_checksum(buff, n, 2);

  buff[n++] = '*';

  ax = crc / 4096;
  if(ax > 9) buff[n++] = (ax % 10) + 'A';
  else buff[n++] = ax + '0';
  
  ax = crc / 256 % 16;
  if(ax > 9) buff[n++] = (ax % 10) + 'A';
  else buff[n++] = ax + '0';

  ax = crc / 16 % 16;
  if(ax > 9) buff[n++] = (ax % 10) + 'A';
  else buff[n++] = ax + '0';
  
  ax = crc % 16;
  if(ax > 9) buff[n++] = (ax % 10) + 'A';
  else buff[n++] = ax + '0';
  
  /* NEW LINE */
  buff[n++] = '\n';
  
  return n;
}


/*
  Returns the length of a tx ready telemetry string contained in BUFF.
*/
uint8_t GPS_get_and_prepare_data(uint8_t * buff)
{
  /* GET GPS DATA */
  uint8_t nB = 0;
  unsigned long mil = millis();

  while(Serial.available()) {uint8_t dummy = Serial.read();}       // clear the buffer
  
  GPS_send_msg(requestPVT, 8);                                     // poll UBX-NAV-PVT message

  while(!Serial.available()) {if(millis() - mil > 15000) break;}   // wait for the response, or eventually timeout, timeout should reflect the GPS's PSM period

  while(1)                                                         // receive the message or timeout
  {
    if(Serial.available()) LORA_pkt[nB++] = Serial.read();         // GPS_buffer[100] replaced by LORA_pkt[256]
    
    if(nB >= 92)
    {
      break;
    }
    
    if(millis() - mil > 15500) break;                              // timeout must be longer then previous timeout
  }

  GPS_parse_0107(LORA_pkt);                                        // if parsing succeeds construct new telemetry, else transmit old one again, GPS_buffer[100] replaced by LORA_pkt[256]

  /* CHECK AIRBORNE MODE */
  nB = 0;
  mil = millis();
  
  while(Serial.available()) {uint8_t dummy = Serial.read();}       // clear the buffer

  GPS_send_msg(requestNAV5, 8);                                    // poll UBX-CFG-NAV5 message

  while(!Serial.available()) {if(millis() - mil > 1000) break;}    // wait for the response, or eventually timeout, unlike with NAVIGATION messages GPS responds immediately

  while(1)                                                         // receive the message or timeout
  {
    if(Serial.available()) LORA_pkt[nB++] = Serial.read();         // GPS_buffer[100] replaced by LORA_pkt[256]
    
    if(nB >= 44)
    {
      break;
    }
    
    if(millis() - mil > 1500) break;                               // timeout must be longer then previous timeout
  }
  
  GPS_parse_0624(LORA_pkt);                                        // GPS_buffer[100] replaced by LORA_pkt[256]

  if(GPSnavigation != 6) GPS_send_msg(setNAVmode, 44);             // set Airborne <1g mode

  /* PREPARE PACKET */
  count++;                                                         // record another round
  
  return GPS_construct_telemetry(buff);
}


// ADC --------------------------------------------------------------------------------------------
/*
  AD6 and AD7 pins on Arduino PRO MINI are Analog Input only.
  ADC6 is used to read the state of VSYNC and /WR signal.
*/


/*
  
*/
void ADC_init(void)
{
  ADCSRA = (1 << ADEN) | (1 << ADPS2) | (1 << ADPS0);                 // enable ADC, set prescaler 32 (8000000 / 32 = 250kHz)
  ADMUX = (1 << REFS0) | (1 << ADLAR) | 6;                            // AVCC with external capacitor at AREF pin, left adjust result, select channel ADC6
  _delay_ms(50);                                                      // delay for the voltage to settle
  // one conversion after REF change should be discarded
  ADCSRA |= (1 << ADSC);                                              // start single conversion for selected Analog Input
  while(ADCSRA & (1 << ADSC));                                        // wait for the end of conversion (ADSC = 0)
}


/*
  
*/
uint8_t ADC_sample_ADC6(void)
{
  ADCSRA |= (1 << ADSC);                                              // start single conversion for selected Analog Input
  while(ADCSRA & (1 << ADSC));                                        // wait for the end of conversion (ADSC = 0)
  
  return ADCH;
}


// I2C functions ----------------------------------------------------------------------------------
/*

*/


/*
  SCL frequency = CPU clock frequency / (16 + 2 * TWBR * PrescalerValue)
*/
void I2C_init(void)
{
  TWSR = 0x00;                                      // set the prescaler value to 1
  TWBR = 0x48;                                      // set the division factor for 100kHz clock signal (0x48 -> 8000000/(16+2*72*1)=50000)
  TWCR = (1 << TWEN);                               // I2C enable
}


/*

*/
void I2C_start(void)
{
  TWCR = (1 << TWINT) | (1 << TWSTA) | (1 << TWEN); // TWINT - clearing the 'job finished' flag, TWSTA - if the bus is clear become Master, TWEN - I2C enable
  while ((TWCR & (1 << TWINT)) == 0);               // waiting for the 'job finished' flag
}


/*

*/
void I2C_stop(void)
{
  TWCR = (1 << TWINT) | (1 << TWSTO) | (1 << TWEN); // TWSTO - generate a Stop condition
  while(TWCR & (1 << TWSTO));                       // wait until the bit is cleared
}


/*

*/
void I2C_write_byte(uint8_t u8data)
{
  TWDR = u8data;                                    // fill the Data Register
  TWCR = (1 << TWINT) | (1 << TWEN);                // TWINT - clearing the 'job finished' flag, TWEN - I2C enable
  while ((TWCR & (1 << TWINT)) == 0);               // waiting for the 'job finished' flag
}


/*
  Read byte. Expect to read more.
*/
uint8_t I2C_read_ack(void)
{
  TWCR = (1 << TWINT) | (1 << TWEN) | (1 << TWEA);
  while ((TWCR & (1 << TWINT)) == 0);               // waiting for the 'job finished' flag

  return TWDR;
}


/*
  Read last byte. Stop condition follows.
*/
uint8_t I2C_read_nak(void)
{
  TWCR = (1 << TWINT) | (1 << TWEN);
  while ((TWCR & (1 << TWINT)) == 0);               // waiting for the 'job finished' flag
  
  return TWDR;
}


// OV5640 -----------------------------------------------------------------------------------------
/*
  
*/


/*
  
*/
void OV5640_register_write(uint16_t reg, uint8_t data)
{
  I2C_start();
  I2C_write_byte(SCCB_ADDRESS);                     // device adress
  I2C_write_byte((reg >> 8) & 0xFF);                // register address high byte
  I2C_write_byte((reg >> 0) & 0xFF);                // register address low byte
  I2C_write_byte(data);                             // data byte
  I2C_stop();
}


/*
  
*/
uint8_t OV5640_register_read(uint16_t reg)
{
  uint8_t data = 0;

  I2C_start();
  I2C_write_byte(SCCB_ADDRESS);                     // device adress - write
  I2C_write_byte((reg >> 8) & 0xFF);                // register address high byte
  I2C_write_byte((reg >> 0) & 0xFF);                // register address low byte
  I2C_stop();

  I2C_start();
  I2C_write_byte(SCCB_ADDRESS | 1);                 // device adress - read
  data = I2C_read_nak();                            // read one byte
  I2C_stop();
  
  return data;
}


/*

*/
void OV_update_regs(uint8_t * point, uint32_t n_regs)
{
  uint8_t data;
  uint16_t reg;
  uint32_t y = 0;
  
  for(uint32_t i = 0; i < n_regs; i++)
  {
    reg = ((uint16_t)pgm_read_byte(&point[y++]) << 8);
    reg |= 0xFF & pgm_read_byte(&point[y++]);
    data = pgm_read_byte(&point[y++]);
    
    OV5640_register_write(reg, data);
  }
}


/*
  Source:
    https://github.com/ArduCAM/Arduino/blob/master/ArduCAM/ov5640_regs.h

  XVCLK       20.16MHz

  PLL
    VCO = XVCLK / predivider * multiplier
    
    multiplier                        0x3036  105
    predivider                        0x3037  3
    
    VCO       705.6MHz

  SYSCLK
    pll_clki = VCO / Pll_rdiv / SysDiv * 2 / Bit_div2x    (?)
    SYSCLK = pll_clki / Sclk_rdiv                         (?)
    
    Bit_div2x     10-bit mode         0x3034  5
    SysDiv        sys clk divider     0x3035  3
    Pll_rdiv      pll root divider    0x3037  2
    Sclk_rdiv     sys clk root div    0x3038  2
    
    pll_clki  47.04MHz
    SYSCLK    23.52MHz

  PCLK
    PCLK = pll_clki / Pclk_div                            (?)
    
    Pclk_div      pclk divider        0x3824  4
    
    PCLK    11.76MHz

  PIXEL ARRAY
    ISP   Image Signal Processing block running at SYSCLK       2623x1951
    DVP   Digital Video Port parallel interface (output size)   1024x768
          Total Size (VSYNC frame size - ?)                     3200x2000
          Frame duration (VSYNC active)                         420ms
          Frames per second                                     2.4
*/
void OV_setup(void)
{
  OV_update_regs(OVsetup01, 2);
  _delay_ms(100);
  OV_update_regs(OVsetup02, 272);
  _delay_ms(500);
  OV_update_regs(OVres_2592x1944, 38);
  OV_update_regs(OVres_1024x768, 29);
}


/*
  Source:
    https://github.com/ArduCAM/Arduino/blob/master/ArduCAM/ov5640_regs.h
  
  RESOLUTION:
    0   320x240
    1   352x288
    2   640x480
    3   800x480
    4   1024x768
    5   1280x960
    6   1600x1200
    7   2048x1536
    8   2592x1944

  Physical Pixel Size
    0:X_ADDR_MAX
    0:Y_ADDR_MAX
    the total pixel array size in the sensor
  ISP Input Size
    X_ADDR_ST{0x3800, 0x3801}
    Y_ADDR_ST{0x3802, 0x3803}
    X_ADDR_END{0x3804, 0x3805}
    Y_ADDR_END{0x3806, 0x3807}
    the total pixel data read from pixel array
    the larger ISP input size is, the less maximum frame rate can be reached
  Data Output Size
    X_OUTPUT_SIZE{0x3808, 0x3809}
    Y_OUTPUT_SIZE{0x380A, 0x380B}
    the image output size of OV5640
    it is windowed from ISP input size and is defined by X_OFFSET{0x3810, 0x3811} and Y_OFFSET{0x3812, 0x3813}

  Binning and Subsampling
    Binning mode is usually used for subsampling. During subsampling, information is periodically dropped
    when data is output. When the binning function is ON, voltage levels of adjacent pixels are averaged
    before being sent to the ADC. If the binning function is OFF, the pixels, which are not output,
    are merely skipped. The OV5640 supports 2x2, 1x2, and 2x1 binning.
    0x3821[0]
    Vertical binning will automatically turn on when in vertical-subsampled formats.
*/
void OV_resolution_JPEG(uint8_t resolution)
{
  switch(resolution)
  {
    case 0:
      OV_update_regs(OVres_320x240, 29);
      break;

    case 1:
      OV_update_regs(OVres_352x288, 29);
      break;

    case 2:
      OV_update_regs(OVres_640x480, 29);
      break;

    case 3:
      OV_update_regs(OVres_800x480, 29);
      break;

    case 4:
      OV_update_regs(OVres_1024x768, 29);
      break;

    case 5:
      OV_update_regs(OVres_1280x960, 29);
      break;

    case 6:
      OV_update_regs(OVres_1600x1200, 29);
      break;

    case 7:
      OV_update_regs(OVres_2048x1536, 29);
      break;

    case 8:
      OV_update_regs(OVres_2592x1944, 38);
      break;

    default:
      break;
  }
}


/*
  Source:
    https://github.com/ArduCAM/Arduino/blob/master/ArduCAM/ArduCAM.cpp
  
  Light Mode
    0   Auto
    1   Sunny
    2   Office
    3   Cloudy
    4   Home

  All changes take place in AWB R/G/B Gain registers while enabling manual AWB (Auto White Balance).
*/
void OV_set_light_mode(uint8_t light)
{
  switch(light)
  {
    case 0:
      OV_update_regs(OVlight_auto, 11);
      break;
      
    case 1:
      OV_update_regs(OVlight_sunny, 10);
      break;
      
    case 2:
      OV_update_regs(OVlight_office, 20);
      break;
      
    case 3:
      OV_update_regs(OVlight_cloudy, 10);
      break;
      
    case 4:
      OV_update_regs(OVlight_home, 10);
      break;
      
    default:
      break; 
  }
}


/*
  Source:
    https://github.com/ArduCAM/Arduino/blob/master/ArduCAM/ArduCAM.cpp

  SATURATION
    0   S3
    1   S2
    2   S1
    3   S0
    4   S_1
    5   S_2
    6   S_3

  All changes take place in CMX Y/U/V registers.
  "The main purpose of the Color Matrix (CMX) function is to cancel out crosstalk and convert color space."
*/
void OV_set_saturation(uint8_t saturation)
{
  switch(saturation)
  {
    case 0:
      OV_update_regs(OVsatur_S3, 14);
      break;
    
    case 1:
      OV_update_regs(OVsatur_S2, 14);
      break;
    
    case 2:
      OV_update_regs(OVsatur_S1, 14);
      break;
      
    case 3:
      OV_update_regs(OVsatur_S0, 14);
      break;
      
    case 4:
      OV_update_regs(OVsatur_S_1, 14);
      break;
      
    case 5:
      OV_update_regs(OVsatur_S_2, 14);
      break;
      
    case 6:
      OV_update_regs(OVsatur_S_3, 14);
      break;

    default:
      break;
  }
}


/*
  Source:
    https://github.com/ArduCAM/Arduino/blob/master/ArduCAM/ArduCAM.cpp

  BRIGHTNESS
    0   B4
    1   B3
    2   B2
    3   B1
    4   B0
    5   B_1
    6   B_2
    7   B_3
    8   B_4

  All changes take place in a couple of Special Digital Effects (SDE) registers. Mainly varying 'Y bright for contrast'.
*/
void OV_set_brightness(uint8_t brightness)
{
  switch(brightness)
  {
    case 0:
      OV_update_regs(OVbright_b4, 5);
      break;
      
    case 1:
      OV_update_regs(OVbright_b3, 5);
      break;
      
    case 2:
      OV_update_regs(OVbright_b2, 5);
      break;
    
    case 3:
      OV_update_regs(OVbright_b1, 5); 
      break;
      
    case 4:
      OV_update_regs(OVbright_b0, 5);
      break;
  
    case 5:
      OV_update_regs(OVbright_b_1, 5);
      break;
  
    case 6:
      OV_update_regs(OVbright_b_2, 5);
      break;
  
    case 7:
      OV_update_regs(OVbright_b_3, 5);
      break;
  
    case 8:
      OV_update_regs(OVbright_b_4, 5);
      break;
  
    default:
      break;
  }
}


/*
  Source:
    https://github.com/ArduCAM/Arduino/blob/master/ArduCAM/ArduCAM.cpp

  CONTRAST
    0   C3
    1   C2
    2   C1
    3   C0
    4   C_1
    5   C_2
    6   C_3

  All changes take place in a couple of Special Digital Effects (SDE) registers. Mainly varying 'Y gain for contrast'.
*/
void OV_set_contrast(uint8_t contrast)
{
  switch(contrast)
  {
    case 0:
      OV_update_regs(OVcontr_c3, 5);
      break;
    
    case 1:
      OV_update_regs(OVcontr_c2, 5);
      break;
    
    case 2:
      OV_update_regs(OVcontr_c1, 5);
      break;
  
    case 3:
      OV_update_regs(OVcontr_c0, 5);
      break;
    
    case 4:
      OV_update_regs(OVcontr_c_1, 5);
      break;
      
    case 5:
      OV_update_regs(OVcontr_c_2, 5);
      break;
      
    case 6:
      OV_update_regs(OVcontr_c_3, 5);
      break;
  
    default:
      break;
  }
}


/*
  Source:
    https://github.com/ArduCAM/Arduino/blob/master/ArduCAM/ArduCAM.cpp

  SPECIAL EFFECTS
    0   Normal
    1   Blueish
    2   Reddish
    3   BW
    4   Sepia
    5   Negative
    6   Greenish
    7   Overexposure
    8   Solarize

  All changes take place in Special Digital Effects (SDE) registers.
*/
void OV_set_special_effects(uint8_t special_effect)
{
  switch(special_effect)
  {
    case 0:
      OV_update_regs(OVeffect_normal, 7);
      break;
      
    case 1:
      OV_update_regs(OVeffect_blue, 7);
      break;
      
    case 2:
      OV_update_regs(OVeffect_red, 7);
      break;
      
    case 3:
      OV_update_regs(OVeffect_bw, 7);
      break;
      
    case 4:
      OV_update_regs(OVeffect_sepia, 7);
      break;
    
    case 5:
      OV_update_regs(OVeffect_neg, 7);
      break;
      
    case 6:
      OV_update_regs(OVeffect_green, 7);
      break;
         
    case 7:
      OV_update_regs(OVeffect_overex, 7);
      break;
    
    case 8:
      OV_update_regs(OVeffect_sol, 7);
      break;

    default:
      break;
  }
}


/*
  Source:
    https://github.com/ArduCAM/Arduino/blob/master/ArduCAM/ArduCAM.cpp

  NIGHT MODE
    0   on
    1   off

  "The OV5640 supports long integration time such as 1 frame, 2 frames, 3 frames, 4 frames, 5 frames, 6 frames, 
  7 frames, and 8 frames in dark conditions. This is achieved by slowing down the original frame rate and waiting
  for exposure."
*/
void OV_set_night_mode(uint8_t night_mode)
{
  uint8_t reg_val;
  uint16_t reg = 0x3A00;
  
  switch(night_mode)
  {
    case 0:
      reg_val = OV5640_register_read(reg);
      reg_val = reg_val | 0x04;
      OV5640_register_write(reg, reg_val);
      break;
      
    case 1:
      reg_val = OV5640_register_read(reg);
      reg_val = reg_val & 0xFB;
      OV5640_register_write(reg, reg_val);
      break;
      
    default:
      break;
  }
}


/*
  Source:
    https://github.com/ArduCAM/Arduino/blob/master/ArduCAM/ArduCAM.cpp

  BANDING_FILTER
    0   Off
    1   Manual_50HZ
    2   Manual_60HZ
    3   Auto_Detection

  "To avoid image flickering under a periodic light source, the integration time can be adjusted in steps of integer
  multiples of the period of the light source. This new AEC step system is called the banding filter, suggesting
  that the exposure time is not continuous but falls in some steps."
*/
void OV_set_banding_filter(uint8_t banding_filter)
{
  uint8_t reg_val;
  uint16_t reg = 0x3A00;
  
  switch(banding_filter)
  {
    case 0:
      reg_val = OV5640_register_read(reg);
      reg_val = reg_val & 0xdf;                 // turn off banding filter
      OV5640_register_write(reg, reg_val);
      break;
      
    case 1:
      OV5640_register_write(0x3c00, 04);        // set to 50Hz
      OV5640_register_write(0x3c01, 80);        // manual banding filter
      reg_val = OV5640_register_read(reg);
      reg_val = reg_val | 0x20;                 // turn on banding filter
      OV5640_register_write(reg, reg_val);
      break;
      
    case 2:
      OV5640_register_write(0x3c00, 00);        // set to 60Hz
      OV5640_register_write(0x3c01, 80);        // manual banding filter
      reg_val = OV5640_register_read(reg);
      reg_val = reg_val | 0x20;                 // turn on banding filter
      OV5640_register_write(reg, reg_val);
      break;
      
    case 3:
      OV5640_register_write(0x3c01, 00);        // auto banding filter
      reg_val = OV5640_register_read(reg);
      reg_val = reg_val & 0xdf;                 // turn off banding filter
      OV5640_register_write(reg, reg_val);
      break;
      
    default:
      break;
  }
}


/*
  Source:
    https://github.com/ArduCAM/Arduino/blob/master/ArduCAM/ArduCAM.cpp

  EXPOSURE LEVEL
    0   EV3
    1   EV2
    2   EV1
    3   EV0
    4   EV_1
    5   EV_2
    6   EV_3

  All changes in Auto Exposure Control (AEC) registers.
*/
void OV_set_EV(uint8_t ev)
{
  switch(ev)
  {
    case 0:
      OV_update_regs(OVev_ev3, 6);
      break;
      
    case 1:
      OV_update_regs(OVev_ev2, 6);
      break;
      
    case 2:
      OV_update_regs(OVev_ev1, 6);
      break;
      
    case 3:
      OV_update_regs(OVev_ev0, 6);
      break;
      
    case 4:
      OV_update_regs(OVev_ev_1, 6);
      break;
      
    case 5:
      OV_update_regs(OVev_ev_2, 6);
      break;
      
    case 6:
      OV_update_regs(OVev_ev_3, 6);
      break;

    default:
      break;
  }
}


/*
  QUALITY
    0   0x01  96.7
    1   0x02  93.7
    2   0x04  87.3
    3   0x06  81.2
    4   0x08  74.8
    5   0x0A  68.7
    6   0x0C  62.3  (default)
    7   0x0E  56.2
    8   0x10  50.0
    9   0x12  44.4
    10  0x14  39.9
    11  0x16  36.3
    12  0x18  33.2
    13  0x1A  30.7
    14  0x1C  28.5
    15  0x1E  26.6
    16  0x1F  25.8

  SSDV Quality
    7   100.00
    6   85.95
    5   70.82
    4   49.61
    3   42.71
    2   28.83
    1   18.00
    0   12.76

  0x4407 Bit[5:0] Quantization Scale (range: 0-31).
*/
void OV_set_quality(uint8_t quality)
{
  uint16_t reg = 0x4407;
  
  switch(quality)
  {
    case 0:
      OV5640_register_write(reg, quality+1);
      break;
    case 1:
      OV5640_register_write(reg, quality*2);
      break;
    case 2:
      OV5640_register_write(reg, quality*2);
      break;
    case 3:
      OV5640_register_write(reg, quality*2);
      break;
    case 4:
      OV5640_register_write(reg, quality*2);
      break;
    case 5:
      OV5640_register_write(reg, quality*2);
      break;
    case 6:
      OV5640_register_write(reg, quality*2);
      break;
    case 7:
      OV5640_register_write(reg, quality*2);
      break;
    case 8:
      OV5640_register_write(reg, quality*2);
      break;
    case 9:
      OV5640_register_write(reg, quality*2);
      break;
    case 10:
      OV5640_register_write(reg, quality*2);
      break;
    case 11:
      OV5640_register_write(reg, quality*2);
      break;
    case 12:
      OV5640_register_write(reg, quality*2);
      break;
    case 13:
      OV5640_register_write(reg, quality*2);
      break;
    case 14:
      OV5640_register_write(reg, quality*2);
      break;
    case 15:
      OV5640_register_write(reg, quality*2);
      break;
    case 16:
      OV5640_register_write(reg, quality*2-1);
      break;
    default:
      break;
  }
}


/*
  1. Write firmware to camera (must be done before each image taking).
  2. Send command 'trigger single' by writting 0x03 to 0x3022.
  3. Keep polling 0x3029 status register until it reads 0x10 'focused'.
  4. Pause autofocus by sending command 0x06 to 0x3022.
  4. Take an image (third one?).
  5. Release lens by writing 0x08 to 0x3022.
*/
void OV_af_firmware(void)
{
  OV_update_regs(OVaf_firmware, 3736);
}


/*

*/
uint8_t OV_af_focus(void)
{
  uint8_t reg;
  unsigned long mil = millis();
  
  OV5640_register_write(0x3022, 0x03);    // trigger single autofocus
  
  while(1)
  {
    reg = OV5640_register_read(0x3029);   // Autofocus status register
    if(reg == 0x10) break;                // focused
    if(millis() - mil > 15000) return 0;  // in case it all goes wrong (autofocusing takes about 10s for some reason)
  }

  OV5640_register_write(0x3022, 0x06);    // pause autofocus
  
  _delay_ms(500);
  return 1;
}


/*

*/
void OV_af_release(void)
{
  OV5640_register_write(0x3022, 0x08);    // release lens
}


// AL422 BUFFER -----------------------------------------------------------------------------------
/*
  FIFO:   393,216kB
  Speed:  50MHz (max)

  Apply /WRST and /RRST 0.1ms after power on.
  When /WRST signal is pulled low, the data input address will be set to 0 and the data in the Input Buffer will be flushed into memory cell array.
  When /RRST signal is pulled low, the data output address will be set to 0 and pre-fetch the data from memory cell array to Output Buffer. 
  Data input DI7~DI0 is written into the write register at the WCK input when /WE is pulled low.
  /OE needs to be pulled low for read operations.
  Data output DO7~DO0 is written into the read register at the RCK input when both /RE and /OE are pulled  low.
  The output data is ready after TAC (access time) from the rising edge of the RCK input cycle.
  When /OE is pulled high, the data outputs will be at high impedance stage. The read address pointer still increases synchronously with RCK regardless of the /OE status.
*/


/*
  RCK default high.
  Read enable /RE is deactivated during read reset.
*/
void AL_read_reset(void)
{
  PORTC |= 0b00001000;                    // RCK LOW->HIGH
  PORTC |= 0b00000100;                    // /RR read reset - HIGH
  _delay_us(1);
  PORTC &= 0b11111011;                    // /RR read reset - LOW
  PORTC &= 0b11110111;                    // RCK HIGH->LOW
  _delay_us(1);
  PORTC |= 0b00001000;                    // RCK LOW->HIGH
  _delay_us(1);
  PORTC &= 0b11110111;                    // RCK HIGH->LOW
  _delay_us(1);
  PORTC |= 0b00001000;                    // RCK LOW->HIGH
  _delay_us(1);
  PORTC &= 0b11110111;                    // RCK HIGH->LOW
  PORTC |= 0b00000100;                    // /RR read reset - HIGH
  _delay_us(1);
  PORTC |= 0b00001000;                    // RCK LOW->HIGH
}


/*
  Read VSYNC until start of a new frame.
  Enable and later disable writing to AL422B.
*/
void AL_capture_frame(void)
{
  while(ADC_sample_ADC6() > 128);         // wait while VSYNC HIGH
  while(ADC_sample_ADC6() <= 128);        // wait while VSYNC LOW
  PORTC |= 0b00000010;                    // WE write enable - A1 set HIGH
  while(ADC_sample_ADC6() <= 128);        // wait while VSYNC LOW
  while(ADC_sample_ADC6() > 128);         // wait while VSYNC HIGH
  PORTC &= 0b11111101;                    // WE write disable - A1 set LOW
}


/*
  RETURN
    0   not all markers found
    1   all markers found, image starts with 0xD8 0xFF 0xE0 sequence
    2   all markers found, image starts with 0xFF 0xE0 sequence
  
  The initial 0xFF byte and sometimes the second 0xD8 byte as well don't get sampled, the rest of the image is fine.
  Sometimes the first check returns 1, but a consequent check returns 2!
  
  Time to scan the whole buffer with 1μs delays is 0.786s.

  The size of the found image is saved in a global variable IMG_size.
*/
uint8_t AL_check_buffer_for_img(void)
{
  uint8_t prevByte = 0;
  uint8_t currentByte = 0;
  uint16_t flag = 0;
  IMG_size = 0;
  
  AL_read_reset();                                                              // resets the read pointer to address 0
  _delay_us(1);

  PORTC &= 0b11111110;                                                          // /RE read enable - A0 set LOW
  
  for(uint32_t i = 0; i < 393216; i++)
  {
    PORTC &= 0b11110111;                                                        // RCK HIGH->LOW
    _delay_us(1);
    PORTC |= 0b00001000;                                                        // RCK LOW->HIGH
    _delay_us(1);
    currentByte = (PINB & 0b00000011) | (PIND & 0b11111100);                    // read the state of D7:D0 pins
    
    IMG_size++;

    if(i == 0 && currentByte == 0xD8) flag |= (0x01 << 0);                      // starts with D8FFE0 (not FFE0)
    
    if(prevByte == 0xFF)
    {
      if(currentByte == 0xE0)                           flag |= (0x01 << 1);    // FFE0
      if(currentByte == 0xDB && (flag & 2) == 2)        flag |= (0x01 << 2);    // FFDB
      if(currentByte == 0xDB && (flag & 6) == 6)        flag |= (0x01 << 3);    // FFDB
      if(currentByte == 0xC4 && (flag & 14) == 14)      flag |= (0x01 << 4);    // FFC4
      if(currentByte == 0xC4 && (flag & 30) == 30)      flag |= (0x01 << 5);    // FFC4
      if(currentByte == 0xC4 && (flag & 62) == 62)      flag |= (0x01 << 6);    // FFC4
      if(currentByte == 0xC4 && (flag & 126) == 126)    flag |= (0x01 << 7);    // FFC4
      if(currentByte == 0xC0 && (flag & 254) == 254)    flag |= (0x01 << 8);    // FFC0
      if(currentByte == 0xDA && (flag & 510) == 510)    flag |= (0x01 << 9);    // FFDA
      if(currentByte == 0xD9 && (flag & 1022) == 1022)
      {
        flag |= (0x01 << 10);                                                   // FFD9
        break;
      }
    }

    if(i >= 4 && flag < 2) break;                                               // don't scan the whole buffer, if there is no image start within first bytes
    
    prevByte = currentByte;
  }

  PORTC |= 0b00000001;                                                          // /RE read disable - A0 set HIGH

  AL_read_reset();                                                              // resets the read pointer to address 0

  if (flag == 0x7FF) {IMG_size += 1; return 1;}
  else if(flag == 0x7FE) {IMG_size += 2; return 2;}
  else {IMG_size = 0; return 0;}
}


/*
  RETURN
    0   SSDV process encountered ERROR
    1   SSDV process reached EOI
*/
uint8_t AL_check_img_with_ssdv(uint8_t img)
{
  uint8_t imgStr = 1;
  uint8_t c = 0;

  AL_read_reset();                                                              // resets the read pointer to address 0
  _delay_us(1);
  
  ssdv_t ssdv;

  ssdv_enc_init(&ssdv, SSDV_TYPE_NOFEC, callsign, 0, SSDV_quality);
  ssdv_enc_set_buffer(&ssdv, LORA_pkt);
  
  while(1)
  {
    while((c = ssdv_enc_get_packet(&ssdv)) == SSDV_FEED_ME)
    {
      if(imgStr)                                                                // insert the missing bytes at the start of the image
      {
        uint8_t jpegStr[2];
        
        if(img == 1)
        {
          jpegStr[0] = 0xFF;
          ssdv_enc_feed(&ssdv, jpegStr, 1);
        }
        else  // img == 2
        {
          jpegStr[0] = 0xFF;
          jpegStr[1] = 0xD8;
          ssdv_enc_feed(&ssdv, jpegStr, 2);
        }
        
        imgStr = 0;
        continue;
      }
        
      AL_read_frame_partial(IMG_buff, IMG_BUFF_SIZE);
      ssdv_enc_feed(&ssdv, IMG_buff, IMG_BUFF_SIZE);
    }
    
    if(c == SSDV_EOI)
    {
      return 1;
    }
    else if(c != SSDV_OK)
    {
      return 0;
    }
  }
}


/*
  Reads desired number of bytes from the buffer.
*/
void AL_read_frame_partial(uint8_t *buff, uint32_t len)
{
  uint32_t dat = 0;
  
  PORTC &= 0b11111110;                                                          // /RE read enable - A0 set LOW
  
  for(uint32_t i = 0; i < len; i++)
  {
    PORTC &= 0b11110111;                                                        // RCK HIGH->LOW
    _delay_us(1);
    PORTC |= 0b00001000;                                                        // RCK LOW->HIGH
    _delay_us(1);
    buff[i] = (PINB & 0b00000011) | (PIND & 0b11111100);                        // read the state of D7:D0 pins
  }

  PORTC |= 0b00000001;                                                          // /RE read disable - A0 set HIGH
}


/*
  Check the camera outputs images or timeout.
*/
uint8_t AL_check_VSYNC(void)
{
  unsigned long mil = millis();
  
  while(ADC_sample_ADC6() > 128)          // wait while VSYNC HIGH
  {
    if(millis() - mil > 3000) return 0;
  }

  mil = millis();
  
  while(ADC_sample_ADC6() <= 128)         // wait while VSYNC LOW
  {
    if(millis() - mil > 3000) return 0;
  }
  
  mil = millis();
  
  while(ADC_sample_ADC6() > 128);         // wait while VSYNC HIGH
  {
    if(millis() - mil > 1000) return 0;
  }

  return 1;
}








