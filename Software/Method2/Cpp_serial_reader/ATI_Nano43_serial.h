#ifndef ATI_NANO43_SERIAL_H
#define ATI_NANO43_SERIAL_H

#include     <stdio.h>      /*标准输入输出定义*/
#include     <stdlib.h>     /*标准函数库定义*/

#include     <unistd.h>     /*Unix 标准函数定义*/
#include     <sys/types.h>
#include     <sys/stat.h>
#include     <fcntl.h>      /*文件控制定义*/
#include     <termios.h>    /*PPSIX 终端控制定义*/
#include     <errno.h>      /*错误号定义*/
#include     <stdint.h>

using namespace std;

void Byte_to_int32(int32_t *t, unsigned char byte[]);

/*
要点提示:
1. int32_t 和 unsigned long具有相同的数据结构长度
2. union据类型里的数据存放在相同的物理空间
*/
typedef union
{
    int32_t tdata;
    unsigned long ldata;
}Int32_t_LongType;



// 6轴力传感器结构体
typedef struct __Force_Data_
{
    unsigned int frame_number;

    int32_t X_force_t;
    int32_t Y_force_t;
    int32_t Z_force_t;
    int32_t X_torque_t;
    int32_t Y_torque_t;
    int32_t Z_torque_t;

    float X_force;
    float Y_force;
    float Z_force;
    float X_torque;
    float Y_torque;
    float Z_torque;
}Force_Data;


class serialPort
{
    private:
       int fd;
       struct  termios Opt;
       int speed_arr[15] = {B115200, B38400, B19200, B9600, B4800, B2400, B1200, B300,
                         B38400, B19200, B9600, B4800, B2400, B1200, B300, };
       int name_arr[15] = {115200, 38400,  19200,  9600,  4800,  2400,  1200,  300, 38400,
                         19200,  9600, 4800, 2400, 1200,  300, };

    public:
     serialPort();
     bool OpenPort(const char * dev);
     int setup(int speed,int flow_ctrl,int databits,int stopbits,int parity)  ;
     void set_speed(int speed);
     int set_Parity(int databits,int stopbits,int parity);
     int readBuffer(uint8_t * buffer,int size);
     int writeBuffer(uint8_t * buffer,int size);
     uint8_t getchar();
     void ClosePort();
};

#endif // ATI_NANO43_SERIAL_H
