/*
  Arduino PRO MINI 3.3V 8MHz
  RFM96W LoRa
  USB to TTL adapter
  Gateway software

  PRO MINI -> GATEWAY protocol
    p: - packet
    t: - telemetry
    j: - ssdv packet
    m: - LoRa mode
    i: - explicit/implicit
    b: - bandwidth
    e: - error coding
    s: - spreading factor
    l: - low data rate
    f: - frequency
    r: - current rssi
    n: - packet SNR
    c: - packet RSSI
    q: - frequency error
    w: - payload length
    o: - RX on/off
    z: - CRC error
*/

#include <SPI.h>


#define LORA_MODE                   0                                // default LoRa mode
#define LORA_FREQUENCY              434.250                          // default LoRa frequency
#define LORA_P_LENGTH               255                              // default LoRa payload length (implicit mode)
#define LORA_RSSI_CONSTANT          -137                             // -137 for RFM9x, -157 for SX127x HF and -164 for SX127x LF


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
unsigned long LORA_RSSI_update      = 0;
uint8_t LORA_received_packet        = 0;                            // 0: none, 1: UKHAS telemetry, 2: SSDV packet, 3: unrecognized packet
uint8_t LORA_rx_on_off              = 0;

float BW[] = {7.8, 10.4, 15.6, 20.8, 31.25, 41.7, 62.5, 125.0, 250.0, 500.0};
uint8_t EC[] = {0, 5, 6, 7, 8};


// SETUP FUNCTION ---------------------------------------------------------------------------------
void setup()
{
  Serial.begin(250000);                                             // Arduino <-> PC (Gateway)

  pinMode(10, OUTPUT);                                              // set up chip select
  digitalWrite(10, HIGH);
  SPI.begin();                                                      // Arduino <-> RFM96W

  LORA_power_mode_sleep();                                          // for configuration put the module in SLEEP mode
  LORA_set_mode(LORA_mode);                                         // default mode
  LORA_set_frequency(LORA_frequency);                               // default frequency
  LORA_set_payload_length(LORA_payload_length);                     // default payload length

  LORA_update_GW_mode();                                            // send initial information to Gateway
  LORA_update_GW_frequency();
  LORA_update_GW_payload_length();
  LORA_update_GW_implicit_explicit();
  LORA_update_GW_bandwidth();
  LORA_update_GW_error_coding();
  LORA_update_GW_spreading_factor();
  LORA_update_GW_low_data_rate();
  LORA_update_GW_rx_on_off();
}


// LOOP FUNCTION ----------------------------------------------------------------------------------
void loop()
{
  // Input Commands
  if(Serial.available())
  {
    uint8_t c = Serial.read();

    if(c == 'o')                                                    // RX ON/OFF
    {
      LORA_rx_on_off = Serial.parseInt();

      if(LORA_rx_on_off) LORA_receive_continuous();
      else LORA_power_mode_standby();

      LORA_update_GW_rx_on_off();
    }

    if(c == 'f')                                                    // FREQUENCY
    {
      LORA_frequency = Serial.parseFloat();
      
      LORA_power_mode_standby();
      LORA_set_frequency(LORA_frequency);

      if(LORA_rx_on_off) LORA_receive_continuous();

      LORA_update_GW_rx_on_off();
      LORA_update_GW_frequency();
    }

    if(c == 'm')                                                    // LORA MODE
    {
      uint8_t mode = Serial.parseInt();
      
      LORA_power_mode_standby();
      LORA_set_mode(mode);
      
      if(LORA_rx_on_off) LORA_receive_continuous();

      LORA_update_GW_rx_on_off();
      LORA_update_GW_mode();
      LORA_update_GW_implicit_explicit();
      LORA_update_GW_bandwidth();
      LORA_update_GW_error_coding();
      LORA_update_GW_spreading_factor();
      LORA_update_GW_low_data_rate();
    }

    if(c == 'l')                                                    // PAYLOAD LENGTH
    {
      LORA_payload_length = Serial.parseInt();
      
      LORA_power_mode_standby();
      LORA_set_payload_length(LORA_payload_length);
      
      if(LORA_rx_on_off) LORA_receive_continuous();

      LORA_update_GW_rx_on_off();
      LORA_update_GW_payload_length();
    }
  }

  // Receive
  uint8_t packetLen = LORA_get_packet();
  
  if(packetLen > 0) LORA_send_packet_to_gateway(LORA_received_packet, packetLen);

  // Update Current RSSI
  if(millis() > LORA_RSSI_update)
  {
    LORA_update_GW_rssi();
    
    LORA_RSSI_update = millis() + 1000;
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
  
  RSSI[dBm] = –164 + Rssi
*/
void LORA_sample_RSSI(void)
{
  LORA_RSSI_value = LORA_RSSI_CONSTANT + (int16_t)LORA_register_read(0x1B);                               // Rssi
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
  Check whether a packet was received. Update all stored variables.
*/
uint8_t LORA_get_packet(void)
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
          LORA_received_packet = 1;

          for(uint8_t i = 0; i < reg; i++)                                                                // in case of large implicit Mode packets, find the end of telemetry string
          {
            if(LORA_FIFO[i] == '\n') reg = i;                                                             // and send only the telemetry string to Gateway
          }
        }
        else if(LORA_FIFO[0] == 0x66 || LORA_FIFO[0] == 0x67)                                             // SSDV packet
        {
          LORA_received_packet = 2;
        }
        else                                                                                              // other packet
        {
          LORA_received_packet = 3;
        }

        LORA_register_write(0x12, 0xFF);                                                                  // RegIrqFlags - clear all interrupts
        LORA_register_write(0x0F, 0x00);                                                                  // FifoRxBaseAddr
        LORA_register_write(0x0D, 0x00);                                                                  // FifoAddrPtr

        return reg;
      }
      else
      {
        reg = LORA_register_read(0x13);                                                                   // RxNbBytes
        LORA_register_write(0x0D, LORA_register_read(0x25) - reg);                                        // set FifoAddrPtr to FifoRxByteAddr - RxNbBytes
        
        for(uint8_t i = 0; i < reg; i++)
        {
          LORA_FIFO[i] = LORA_register_read(0x00);                                                        // read RegFifo
        }
        
        if(reg < 255) LORA_FIFO[reg] = '\0';
        
        LORA_received_packet = 0;                                                                         // CRC error
        
        LORA_register_write(0x12, 0xFF);                                                                  // RegIrqFlags - clear all interrupts
        LORA_register_write(0x0F, 0x00);                                                                  // FifoRxBaseAddr
        LORA_register_write(0x0D, 0x00);                                                                  // FifoAddrPtr

        if(reg > 0) return reg;
        else return 1;
      }
    }
    else
    {
      LORA_register_write(0x12, 0xFF);                                                                    // RegIrqFlags - clear all interrupts
      LORA_register_write(0x0F, 0x00);                                                                    // FifoRxBaseAddr
      LORA_register_write(0x0D, 0x00);                                                                    // FifoAddrPtr

      return 0;
    }
  }

  return 0;
}


/*

*/
void LORA_send_packet_to_gateway(uint8_t type, uint8_t pktLen)
{
  switch(type)
  {
    case 0:
      Serial.print("z:");                                                                                 // signal CRC error
      for(uint8_t i = 0; i < pktLen; i++) Serial.write(LORA_FIFO[i]);
      Serial.print('\n');

      LORA_update_GW_rssi();
      LORA_update_GW_packet_rssi_SNR();
      LORA_update_GW_frequency_error();
      break;
      
    case 1:
      Serial.print("t:");                                                                                 // signal normal UKHAS telemetry
      for(uint8_t i = 0; i < pktLen; i++) Serial.write(LORA_FIFO[i]);
      Serial.print('\n');

      LORA_update_GW_rssi();
      LORA_update_GW_packet_rssi_SNR();
      LORA_update_GW_frequency_error();
      break;
      
    case 2:
      Serial.print("j:");                                                                                 // signal SSDV packet
      Serial.write(0x55);                                                                                 // insert the left out SYNC BYTE
      for(uint8_t i = 0; i < pktLen; i++) Serial.write(LORA_FIFO[i]);
      Serial.print('\n');

      LORA_update_GW_rssi();
      LORA_update_GW_packet_rssi_SNR();
      LORA_update_GW_frequency_error();
      break;
      
    case 3:
      Serial.print("p:");                                                                                 // signal unrecognized packet
      for(uint8_t i = 0; i < pktLen; i++) Serial.write(LORA_FIFO[i]);
      Serial.print('\n');

      LORA_update_GW_rssi();
      LORA_update_GW_packet_rssi_SNR();
      LORA_update_GW_frequency_error();
      break;

    default:
      break;
  }
}


/*
  Update Gateway with the new setting.
*/
void LORA_update_GW_frequency(void)
{
  uint32_t freqB;
  float freqBf;
  
  freqB = (uint32_t)LORA_register_read(0x06) << 16;
  freqB |= (uint32_t)LORA_register_read(0x07) << 8;
  freqB |= (uint32_t)LORA_register_read(0x08);
  
  freqBf = (float)freqB * 32.0 / 524288.0;
  
  Serial.print("f:");
  Serial.print(freqBf, 4);
  Serial.print('\n');
}


/*
  Update Gateway with the new setting.
*/
void LORA_update_GW_mode(void)
{
  Serial.print("m:");
  Serial.print(LORA_mode);
  Serial.print('\n');
}


/*
  Update Gateway with the new setting.
*/
void LORA_update_GW_implicit_explicit(void)
{
  uint8_t ie;
  
  ie = LORA_register_read(0x1D) & 0x01;
  
  Serial.print("i:");
  if(ie) Serial.print("implicit");
  else Serial.print("explicit");
  Serial.print('\n');
}


/*
  Update Gateway with the new setting.
*/
void LORA_update_GW_bandwidth(void)
{
  uint8_t bnd;

  bnd = (LORA_register_read(0x1D) >> 4) & 0x0F;
  
  Serial.print("b:");
  Serial.print(BW[bnd]);
  Serial.print('\n');
}


/*
  Update Gateway with the new setting.
*/
void LORA_update_GW_error_coding(void)
{
  uint8_t ec;

  ec = (LORA_register_read(0x1D) >> 1) & 0x07;

  Serial.print("e:");
  Serial.print("4/");
  Serial.print(EC[ec]);
  Serial.print('\n');
}


/*
  Update Gateway with the new setting.
*/
void LORA_update_GW_spreading_factor(void)
{
  uint8_t sf;
  
  sf = (LORA_register_read(0x1E) >> 4) & 0x0F;
  
  Serial.print("s:");
  Serial.print(sf);
  Serial.print('\n');
}


/*
  Update Gateway with the new setting.
*/
void LORA_update_GW_low_data_rate(void)
{
  uint8_t ldr;
  
  ldr = (LORA_register_read(0x26) >> 3) & 0x01;

  Serial.print("l:");
  Serial.print(ldr);
  Serial.print('\n');
}


/*
  Update Gateway with the new setting.
*/
void LORA_update_GW_payload_length(void)
{
  uint8_t pyld;
  
  pyld = LORA_register_read(0x22);
  
  Serial.print("w:");
  Serial.print(pyld);
  Serial.print('\n');
}


/*
  Update Gateway with the new setting.
*/
void LORA_update_GW_rssi(void)
{
  LORA_sample_RSSI();
  
  Serial.print("r:");
  Serial.print(LORA_RSSI_value);
  Serial.print('\n');
}


/*
  Update Gateway with the new setting.
*/
void LORA_update_GW_packet_rssi_SNR(void)
{
  LORA_get_packet_SNR();

  Serial.print("n:");
  Serial.print(LORA_SNR_packet);
  Serial.print('\n');
  
  Serial.print("c:");
  Serial.print(LORA_RSSI_packet);
  Serial.print('\n');
}


/*
  Update Gateway with the new setting.
*/
void LORA_update_GW_frequency_error(void)
{
  float freqErr;

  freqErr = LORA_get_frequency_error(BW[LORA_bandwidth]) / 1000.0;
  
  Serial.print("q:");
  Serial.print(freqErr, 1);
  Serial.print('\n');
}


/*
  Update Gateway with the new setting.
*/
void LORA_update_GW_rx_on_off(void)
{
  Serial.print("o:");
  Serial.print(LORA_rx_on_off);
  Serial.print('\n');
}









