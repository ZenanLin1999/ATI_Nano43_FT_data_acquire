#include <Arduino.h>

class ADS1256
{
  private:
    int PIN_DRDY;
    int PIN_CS;
    int PIN_RST;

    int PIN_SCK;
    int PIN_DIN;
    int PIN_DOUT;

    int PGA;
    int SPS;
    int Channel;
  public:
    ADS1256();
    ~ADS1256();
    bool OpenDevice();
    void CloseDevice();
    float GetAnalog();
    float GetAnalogC();
    void StartAnalogC();
    void StopAnalogC();
    bool SetPGA(int pga);
    bool SetSPS(int sps);
    bool SelectChannel(int ch);
    bool SetChannel(int ch);
    
    bool SetPinDRDY(int drdy);
    bool SetPinCS(int cs);
    bool SetPinRST(int rst);

    bool SetPinSCK(int sck);
    bool SetPinDIN(int din);
    bool SetPinDOUT(int dout);

    int GetPGA();
    int GetSPS();
    int GetChannel();
    int GetPINRDY();
    int GetPINCS();
    int GetPINRST();
    void ResetHardware();
    void ResetSoftware();
  private:
    void write_register(unsigned char reg, unsigned char wdata);
    unsigned long read_data();
};
