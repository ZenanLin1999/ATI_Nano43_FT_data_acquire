#include "ADS1256.h"
#include "SPI.h"

#define STATUS 0x00
#define MUX 0x01
#define ADCON 0x02
#define DRATE 0x03
#define IO 0x04

#define CH0 0x08 //Singel-Ended Input 0, Channel 0
#define CH1 0x18 //Singel-Ended Input 1, Channel 1
#define CH2 0x28 //Singel-Ended Input 2, Channel 2
#define CH3 0x38 //Singel-Ended Input 3, Channel 3
#define CH4 0x48 //Singel-Ended Input 4, Channel 4
#define CH5 0x58 //Singel-Ended Input 5, Channel 5
#define CH6 0x68 //Singel-Ended Input 6, Channel 6
#define CH7 0x78 //Singel-Ended Input 7, Channel 7
#define CH10 0x01 //Differential Input 0-1, Channel 10
#define CH11 0x23 //Differential Input 2-3, Channel 11
#define CH12 0x45 //Differential Input 4-5, Channel 12
#define CH13 0x67 //Differential Input 6-7, Channel 13

#define PGA1 0x00
#define PGA2 0x01
#define PGA4 0x02
#define PGA8 0x03
#define PGA16 0x04
#define PGA32 0x05
#define PGA64 0x06

#define SPS30000 0xf0
#define SPS15000 0xe0
#define SPS7500 0xd0
#define SPS3750 0xc0
#define SPS2000 0xb0
#define SPS1000 0xa1
#define SPS500 0x92
#define SPS100 0x82
#define SPS50 0x63
#define SPS10 0x23
#define SPS5 0x13
#define SPS25 0x03  //2.5SPS

#define WAKEUP 0x00
#define RDATA  0x01
#define RDATAC 0x03
#define SDATAC 0x0f
#define RREG 0x10
#define WREG 0x50
#define SELFCAL 0xf0
#define SELFOCAL 0xf1
#define SELFGCAL 0xf2
#define SYSOCAL 0xf3
#define SYSGCAL 0xf4
#define SYNC 0xfc
#define STANDBY 0xfd
#define RESET 0xfe

#define VRef 2.5


// default
// #define PIN_DRDY 15
// #define PIN_CS 16
// #define PIN_RST 17
// #define PIN_SYNC 18

// #define PIN_SCK 12
// #define PIN_DIN 13
// #define PIN_DOUT 14

ADS1256::ADS1256() {
  PIN_RST = 17;
  PIN_DRDY = 15;
  PIN_CS = 16;

  PIN_SCK = 12; 
  PIN_DIN = 13;
  PIN_DOUT = 14;

  PGA = 1;
  SPS = 1000;
  Channel = 0;
}

ADS1256::~ADS1256() {
  SPI.end();
}

bool ADS1256::OpenDevice() {

  if (PIN_DRDY == PIN_CS || PIN_CS == PIN_RST || PIN_DRDY == PIN_RST) {
    return false;
  }
  
  pinMode(PIN_DRDY, INPUT);
  pinMode(PIN_CS, OUTPUT);
  pinMode(PIN_RST, OUTPUT);
  digitalWrite(PIN_RST, HIGH);  //Low to Reset
  
  // SPI.begin();
  SPI.begin(PIN_SCK, PIN_DOUT, PIN_DIN, -1); //sck=-1, miso(DIN)=-1, mosi(DOUT)=-1, ss=-1
  SPI.setBitOrder(MSBFIRST);
  SPI.setClockDivider(SPI_CLOCK_DIV4);
  SPI.setDataMode(SPI_MODE1);

  digitalWrite(PIN_CS, LOW);	//Low to select

  write_register(STATUS, 0x04);        //Auto-Calibration
  write_register(ADCON, PGA1);         //Gain=1
  write_register(DRATE, SPS1000);         //1000SPS
  write_register(IO, 0x00);	          //IO not used
  while (digitalRead(PIN_DRDY) == HIGH);                    //DRDY low to available
  SPI.transfer(SELFCAL);                  //Self-Calibration

  digitalWrite(PIN_CS, HIGH);

  return true;

}

void ADS1256::CloseDevice() {
  SPI.end();
}

float ADS1256::GetAnalog() {

  unsigned long x;
  float y;

  digitalWrite(PIN_CS, LOW);
  switch (Channel) {
    case 0:
      write_register(MUX, CH0);
      break;
    case 1:
      write_register(MUX, CH1);
      break;
    case 2:
      write_register(MUX, CH2);
      break;
    case 3:
      write_register(MUX, CH3);
      break;
    case 4:
      write_register(MUX, CH4);
      break;
    case 5:
      write_register(MUX, CH5);
      break;
    case 6:
      write_register(MUX, CH6);
      break;
    case 7:
      write_register(MUX, CH7);
      break;
    case 10:
      write_register(MUX, CH10);
      break;
    case 11:
      write_register(MUX, CH11);
      break;
    case 12:
      write_register(MUX, CH12);
      break;
    case 13:
      write_register(MUX, CH13);
      break;
    default:
      return 0;
  }
  delayMicroseconds(5);
  SPI.transfer(SYNC);
  delayMicroseconds(5);
  SPI.transfer(WAKEUP);
  delayMicroseconds(5);
  SPI.transfer(RDATA);
  delayMicroseconds(10);

  x = read_data();

  digitalWrite(PIN_CS, HIGH);

  if (x > 0x007fffff) {
    x = 0x1000000 - x;
    y = -2 * VRef * x / 0x007fffff / PGA;
  } else {
    y = 2 * VRef * x / 0x007fffff / PGA;
  }

  return y;

}

float ADS1256::GetAnalogC() {

  long x;
  float y;

  digitalWrite(PIN_CS, LOW);

  while (digitalRead(PIN_DRDY) == HIGH);

  x = read_data();

  digitalWrite(PIN_CS, HIGH);

  if (x > 0x007fffff) {
    x = 0x1000000 - x;
    y = -2 * VRef * x / 0x007fffff / PGA;
  } else {
    y = 2 * VRef * x / 0x007fffff / PGA;
  }

  return y;

}

void ADS1256::StartAnalogC() {

  digitalWrite(PIN_CS, LOW);
  switch (Channel) {
    case 0:
      write_register(MUX, CH0);
      break;
    case 1:
      write_register(MUX, CH1);
      break;
    case 2:
      write_register(MUX, CH2);
      break;
    case 3:
      write_register(MUX, CH3);
      break;
    case 4:
      write_register(MUX, CH4);
      break;
    case 5:
      write_register(MUX, CH5);
      break;
    case 6:
      write_register(MUX, CH6);
      break;
    case 7:
      write_register(MUX, CH7);
      break;
    case 10:
      write_register(MUX, CH10);
      break;
    case 11:
      write_register(MUX, CH11);
      break;
    case 12:
      write_register(MUX, CH12);
      break;
    case 13:
      write_register(MUX, CH13);
      break;
    default:
      return;
  }
  delayMicroseconds(5);
  SPI.transfer(SYNC);
  delayMicroseconds(5);
  SPI.transfer(WAKEUP);
  delayMicroseconds(5);
  SPI.transfer(RDATAC);
  delayMicroseconds(10);

  digitalWrite(PIN_CS, HIGH);

}

void ADS1256::StopAnalogC() {

  digitalWrite(PIN_CS, LOW);
  while (digitalRead(PIN_DRDY) == HIGH);
  SPI.transfer(SDATAC);
  digitalWrite(PIN_CS, HIGH);

}

bool ADS1256::SelectChannel(int ch) {

  bool result = false;
  switch (ch) {
    case 0:
      Channel = ch;
      result = true;
      break;
    case 1:
      Channel = ch;
      result = true;
      break;
    case 2:
      Channel = ch;
      result = true;
      break;
    case 3:
      Channel = ch;
      result = true;
      break;
    case 4:
      Channel = ch;
      result = true;
      break;
    case 5:
      Channel = ch;
      result = true;
      break;
    case 6:
      Channel = ch;
      result = true;
      break;
    case 7:
      Channel = ch;
      result = true;
      break;
    case 10:
      Channel = ch;
      result = true;
      break;
    case 11:
      Channel = ch;
      result = true;
      break;
    case 12:
      Channel = ch;
      result = true;
      break;
    case 13:
      Channel = ch;
      result = true;
      break;
    default:
      result = false;
  }
  return result;

}

bool ADS1256::SetChannel(int ch) {

  bool result = false;
  switch (ch) {
    case 0:
      digitalWrite(PIN_CS, LOW);
      Channel = ch;
      write_register(MUX, CH0);
      digitalWrite(PIN_CS, HIGH);
      delay(1);
      result = true;
      break;
    case 1:
      digitalWrite(PIN_CS, LOW);
      Channel = ch;
      write_register(MUX, CH1);
      digitalWrite(PIN_CS, HIGH);
      delay(1);
      result = true;
      break;
    case 2:
      digitalWrite(PIN_CS, LOW);
      Channel = ch;
      write_register(MUX, CH2);
      digitalWrite(PIN_CS, HIGH);
      delay(1);
      result = true;
      break;
    case 3:
      digitalWrite(PIN_CS, LOW);
      Channel = ch;
      write_register(MUX, CH3);
      digitalWrite(PIN_CS, HIGH);
      delay(1);
      result = true;
      break;
    case 4:
      digitalWrite(PIN_CS, LOW);
      Channel = ch;
      write_register(MUX, CH4);
      digitalWrite(PIN_CS, HIGH);
      delay(1);
      result = true;
      break;
    case 5:
      digitalWrite(PIN_CS, LOW);
      Channel = ch;
      write_register(MUX, CH5);
      digitalWrite(PIN_CS, HIGH);
      delay(1);
      result = true;
      break;
    case 6:
      digitalWrite(PIN_CS, LOW);
      Channel = ch;
      write_register(MUX, CH6);
      digitalWrite(PIN_CS, HIGH);
      delay(1);
      result = true;
      break;
    case 7:
      digitalWrite(PIN_CS, LOW);
      Channel = ch;
      write_register(MUX, CH7);
      digitalWrite(PIN_CS, HIGH);
      delay(1);
      result = true;
      break;
    case 10:
      digitalWrite(PIN_CS, LOW);
      Channel = ch;
      write_register(MUX, CH10);
      digitalWrite(PIN_CS, HIGH);
      delay(1);
      result = true;
      break;
    case 11:
      digitalWrite(PIN_CS, LOW);
      Channel = ch;
      write_register(MUX, CH11);
      digitalWrite(PIN_CS, HIGH);
      delay(1);
      result = true;
      break;
    case 12:
      digitalWrite(PIN_CS, LOW);
      Channel = ch;
      write_register(MUX, CH12);
      digitalWrite(PIN_CS, HIGH);
      delay(1);
      result = true;
      break;
    case 13:
      digitalWrite(PIN_CS, LOW);
      Channel = ch;
      write_register(MUX, CH13);
      digitalWrite(PIN_CS, HIGH);
      delay(1);
      result = true;
      break;
    default:
      result = false;
  }
  return result;

}

bool ADS1256::SetPGA(int pga) {

  bool result = false;
  switch (pga) {
    case 1:
      digitalWrite(PIN_CS, LOW);
      PGA = pga;
      write_register(ADCON, PGA1);
      digitalWrite(PIN_CS, HIGH);
      delay(1);
      result = true;
      break;
    case 2:
      digitalWrite(PIN_CS, LOW);
      PGA = pga;
      write_register(ADCON, PGA2);
      digitalWrite(PIN_CS, HIGH);
      delay(1);
      result = true;
      break;
    case 4:
      digitalWrite(PIN_CS, LOW);
      PGA = pga;
      write_register(ADCON, PGA4);
      digitalWrite(PIN_CS, HIGH);
      delay(1);
      result = true;
      break;
    case 8:
      digitalWrite(PIN_CS, LOW);
      PGA = pga;
      write_register(ADCON, PGA8);
      digitalWrite(PIN_CS, HIGH);
      delay(1);
      result = true;
      break;
    case 16:
      digitalWrite(PIN_CS, LOW);
      PGA = pga;
      write_register(ADCON, PGA16);
      digitalWrite(PIN_CS, HIGH);
      delay(1);
      result = true;
      break;
    case 32:
      digitalWrite(PIN_CS, LOW);
      PGA = pga;
      write_register(ADCON, PGA32);
      digitalWrite(PIN_CS, HIGH);
      delay(1);
      result = true;
      break;
    case 64:
      digitalWrite(PIN_CS, LOW);
      PGA = pga;
      write_register(ADCON, PGA64);
      digitalWrite(PIN_CS, HIGH);
      delay(1);
      result = true;
      break;
    default:
      result = false;
  }
  return result;

}

bool ADS1256::SetSPS(int sps) {

  bool result = false;
  switch (sps) {
    case 30000:
      digitalWrite(PIN_CS, LOW);
      SPS = sps;
      write_register(DRATE, SPS30000);
      digitalWrite(PIN_CS, HIGH);
      delay(1);
      result = true;
      break;
    case 15000:
      digitalWrite(PIN_CS, LOW);
      SPS = sps;
      write_register(DRATE, SPS15000);
      digitalWrite(PIN_CS, HIGH);
      delay(1);
      result = true;
      break;
    case 7500:
      digitalWrite(PIN_CS, LOW);
      SPS = sps;
      write_register(DRATE, SPS7500);
      digitalWrite(PIN_CS, HIGH);
      delay(1);
      result = true;
      break;
    case 3750:
      digitalWrite(PIN_CS, LOW);
      SPS = sps;
      write_register(DRATE, SPS3750);
      digitalWrite(PIN_CS, HIGH);
      delay(1);
      result = true;
      break;
    case 2000:
      digitalWrite(PIN_CS, LOW);
      SPS = sps;
      write_register(DRATE, SPS2000);
      digitalWrite(PIN_CS, HIGH);
      delay(1);
      result = true;
      break;
    case 1000:
      digitalWrite(PIN_CS, LOW);
      SPS = sps;
      write_register(DRATE, SPS1000);
      digitalWrite(PIN_CS, HIGH);
      delay(1);
      result = true;
      break;
    case 500:
      digitalWrite(PIN_CS, LOW);
      SPS = sps;
      write_register(DRATE, SPS500);
      digitalWrite(PIN_CS, HIGH);
      delay(1);
      result = true;
      break;
    case 100:
      digitalWrite(PIN_CS, LOW);
      SPS = sps;
      write_register(DRATE, SPS100);
      digitalWrite(PIN_CS, HIGH);
      delay(1);
      result = true;
      break;
    case 50:
      digitalWrite(PIN_CS, LOW);
      SPS = sps;
      write_register(DRATE, SPS50);
      digitalWrite(PIN_CS, HIGH);
      delay(1);
      result = true;
      break;
    case 10:
      digitalWrite(PIN_CS, LOW);
      SPS = sps;
      write_register(DRATE, SPS10);
      digitalWrite(PIN_CS, HIGH);
      delay(1);
      result = true;
      break;
    case 5:
      digitalWrite(PIN_CS, LOW);
      SPS = sps;
      write_register(DRATE, SPS5);
      digitalWrite(PIN_CS, HIGH);
      delay(1);
      result = true;
      break;
    case 25:
      digitalWrite(PIN_CS, LOW);
      SPS = sps;
      write_register(DRATE, SPS25);
      digitalWrite(PIN_CS, HIGH);
      delay(1);
      result = true;
      break;
    default:
      result = false;
  }
  return result;

}

bool ADS1256::SetPinDRDY(int drdy) {
  if (0 <= drdy < 54) {
    PIN_DRDY = drdy;
    return true;
  }
  else {
    return false;
  }
}

bool ADS1256::SetPinCS(int cs) {
  if (0 <= cs < 54) {
    PIN_CS = cs;
    return true;
  }
  else {
    return false;
  }
}

bool ADS1256::SetPinRST(int rst) {
  if (0 <= rst < 54) {
    PIN_RST = rst;
    return true;
  }
  else {
    return false;
  }
}

bool ADS1256::SetPinSCK(int sck) {
  if (0 <= sck < 54) {
    PIN_SCK = sck;
    return true;
  }
  else {
    return false;
  }
}

bool ADS1256::SetPinDIN(int din) {
  if (0 <= din < 54) {
    PIN_DIN = din;
    return true;
  }
  else {
    return false;
  }
}

bool ADS1256::SetPinDOUT(int dout) {
  if (0 <= dout < 54) {
    PIN_DOUT = dout;
    return true;
  }
  else {
    return false;
  }
}

int ADS1256::GetPGA() {
  return PGA;
}

int ADS1256::GetSPS() {
  return SPS;
}

int ADS1256::GetChannel() {
  return Channel;
}

int ADS1256::GetPINRDY() {
  return PIN_DRDY;
}

int ADS1256::GetPINCS() {
  return PIN_CS;
}

int ADS1256::GetPINRST() {
  return PIN_RST;
}

void ADS1256::ResetHardware() {
  digitalWrite(PIN_RST, LOW);
  delay(1);
  digitalWrite(PIN_RST, HIGH);
}

void ADS1256::ResetSoftware() {
  while (digitalRead(PIN_DRDY) == HIGH);                    //DRDY low to available
  SPI.transfer(RESET);
}

void ADS1256::write_register(unsigned char reg, unsigned char wdata) {
  while (digitalRead(PIN_DRDY) == HIGH);                    //DRDY low to available
  SPI.transfer(WREG | reg);             //select register
  SPI.transfer(0x00);                   //write 1 byte
  SPI.transfer(wdata);                  //write data
}

unsigned long ADS1256::read_data() {
  unsigned long rx_dat[3];
  unsigned long x;
  rx_dat[0] = SPI.transfer(0xff);
  rx_dat[1] = SPI.transfer(0xff);
  rx_dat[2] = SPI.transfer(0xff);
//  x = (unsigned long)rx_dat[0] * 65536 + (unsigned long)rx_dat[1] * 256 + (unsigned long)rx_dat[2];
  x = rx_dat[0] * 65536 + rx_dat[1] * 256 + rx_dat[2];
  return x;
}
