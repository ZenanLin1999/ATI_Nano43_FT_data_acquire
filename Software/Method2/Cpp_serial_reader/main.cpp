#include "ATI_Nano43_serial.h"
#include <iostream>


// 0xAA+0xFF+0xF1+NUM(byte)+DATA(低位先发)+SC+AC 转换后 力&力矩数据*1,000
// example:(30 bytes)
// AA FF F1 18 06 00 00 00 F3 FF FF FF 02 00 00 00 29 00 00 00 C3 FF FF FF 0C 00 00 00 9F E7
const uint8_t buff_len = 30;
uint8_t buff[buff_len]; // read buffer
const char *dev  = "/dev/ttyACM0";

Force_Data ATI_Nano43_data;

int main()
{
   // basic serial port setup
   serialPort myserial;
   int nread;
   unsigned char byte_transfer[4]={0};

   cout<<"SerialPort Open!"<<endl;
   myserial.OpenPort(dev);
   myserial.setup(115200,0,8,1,'N');

   while (true)
   {
     // 00-read a frame.
     nread = myserial.readBuffer(buff, buff_len); //

     // 01-only for debug(print raw data)
     for(int i = 0;i<buff_len;i++)
     {
         printf("%02x ", buff[i]);
     }
     printf("\r\n");

     // 03-decode the real ATI force and torch value.
     if(buff[0] == 0xAA && buff[1] == 0xFF && buff[2] == 0xF1)
     {
         // force and torch 六轴数据 decode.
         byte_transfer[0] = buff[4];
         byte_transfer[1] = buff[5];
         byte_transfer[2] = buff[6];
         byte_transfer[3] = buff[7];
         Byte_to_int32(&(ATI_Nano43_data.X_force_t), byte_transfer);
         ATI_Nano43_data.X_force = 1.0f * ATI_Nano43_data.X_force_t / 1000.0f;

         byte_transfer[0] = buff[8];
         byte_transfer[1] = buff[9];
         byte_transfer[2] = buff[10];
         byte_transfer[3] = buff[11];
         Byte_to_int32(&(ATI_Nano43_data.Y_force_t), byte_transfer);
         ATI_Nano43_data.Y_force = 1.0f * ATI_Nano43_data.Y_force_t / 1000.0f;

         byte_transfer[0] = buff[12];
         byte_transfer[1] = buff[13];
         byte_transfer[2] = buff[14];
         byte_transfer[3] = buff[15];
         Byte_to_int32(&(ATI_Nano43_data.Z_force_t), byte_transfer);
         ATI_Nano43_data.Z_force = 1.0f * ATI_Nano43_data.Z_force_t / 1000.0f;

         byte_transfer[0] = buff[16];
         byte_transfer[1] = buff[17];
         byte_transfer[2] = buff[18];
         byte_transfer[3] = buff[19];
         Byte_to_int32(&(ATI_Nano43_data.X_torque_t), byte_transfer);
         ATI_Nano43_data.X_torque = 1.0f * ATI_Nano43_data.X_torque_t / 1000.0f;

         byte_transfer[0] = buff[20];
         byte_transfer[1] = buff[21];
         byte_transfer[2] = buff[22];
         byte_transfer[3] = buff[23];
         Byte_to_int32(&(ATI_Nano43_data.Y_torque_t), byte_transfer);
         ATI_Nano43_data.Y_torque = 1.0f * ATI_Nano43_data.Y_torque_t / 1000.0f;

         byte_transfer[0] = buff[24];
         byte_transfer[1] = buff[25];
         byte_transfer[2] = buff[26];
         byte_transfer[3] = buff[27];
         Byte_to_int32(&(ATI_Nano43_data.Z_torque_t), byte_transfer);
         ATI_Nano43_data.Z_torque = 1.0f * ATI_Nano43_data.Z_torque_t / 1000.0f;
     }

     // 04-only for debug(print acquired data)
     printf("%04f  ", ATI_Nano43_data.X_force);
     printf("%04f  ", ATI_Nano43_data.Y_force);
     printf("%04f  ", ATI_Nano43_data.Z_force);
     printf("%04f  ", ATI_Nano43_data.X_torque);
     printf("%04f  ", ATI_Nano43_data.Y_torque);
     printf("%04f  ", ATI_Nano43_data.Z_torque);
     printf("\r\n");

     // 05-100hz must to match the send rate.
     usleep(10000);
   }
}