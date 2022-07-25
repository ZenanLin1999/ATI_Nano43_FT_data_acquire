//TBSI-SSR LinZenan
//功能：使用ADS1256 adc的单端模式读取ATI-Nano43传感器数据并转化为六维力力矩数据，通过串口发送出去。

#include <Arduino.h>
#include <FastLED.h>

#include "ADS1256.h"

#define PIN_DRDY 15
#define PIN_CS 16
#define PIN_RST 17
#define PIN_SYNC 18

#define PIN_SCK 12
#define PIN_DIN 13
#define PIN_DOUT 14

#define SerialDebug false  // set to true to get Serial output for debugging

ADS1256 myads1256;    //ADS1256 ADCShield Class
float tempfloat = 0;
float ATI_Nano43_raw_offset[6]; // ATI传感器偏置电压统计
float ATI_Nano43_raw_data[6]; // ATI传感器原始六路电压数据
float ATI_Nano43_force_data[3]; // ATI传感器转化后的三轴力数据 [Fx, Fy, Fz]
float ATI_Nano43_torch_data[3]; // ATI传感器转化后的三轴力矩数据 [Tx, Ty, Tz]
int32_t ATI_Nano43_force_data_t[3]; // ATI传感器转化后的三轴力数据 [Fx, Fy, Fz]*1000  int32_t
int32_t ATI_Nano43_torch_data_t[3]; // ATI传感器转化后的三轴力矩数据 [Tx, Ty, Tz]*1000  int32_t

// 转换矩阵
float ATI_Nano43_matrix_Fx[6] = {0.01987,  -0.00859,   0.02845,   2.20608,   0.07243,  -2.21747};
float ATI_Nano43_matrix_Fy[6] = {0.00614,  -2.60020,   0.03938,   1.30049,  -0.09713,   1.28913};
float ATI_Nano43_matrix_Fz[6] = {1.50115,  -0.02542,   1.52239,  -0.01504,   1.52065,  -0.00098};
float ATI_Nano43_matrix_Tx[6] = {-0.03928, -11.53765,  22.19625,   5.62507, -22.66068,   5.61595};
float ATI_Nano43_matrix_Ty[6] = {-25.28900,  0.43679,  12.70831,  -9.90027,  12.25530,   9.92669};
float ATI_Nano43_matrix_Tz[6] = {-0.08965, -18.96174,  -0.22897, -18.20473,   0.75572, -18.66394};

// ws2812-led
#define NUM_LEDS 1
#define DATA_PIN 48
CRGB leds[NUM_LEDS];
static bool led_status = true;

// 数据发送相关
unsigned char DataToSend[50]; // send to ANOV7
// cup为小端模式存储，也就是在存储的时候，低位被存在0字节，高位在1字节
#define BYTE0(dwTemp)       (*(char *)(&dwTemp))          // 取出int型变量的低字节
#define BYTE1(dwTemp)       (*((char *)(&dwTemp) + 1))    // 取存储在此变量下一内存字节的内容，高字节
#define BYTE2(dwTemp)       (*((char *)(&dwTemp) + 2))
#define BYTE3(dwTemp)       (*((char *)(&dwTemp) + 3))

// freertos 任务中MUTEX信号锁定义
SemaphoreHandle_t xMutex_datapack = NULL; //创建信号量Handler
TickType_t timeOut = 5; //用于获取信号量的Timeout 5 ticks

// 统计偏置电压
void STA_ATI_Nano43_offset(int times = 100)
{
  // 统计times次，默认100次
  for(int i = 0;i<times;i++)
  {
    myads1256.SetChannel(1);
    delay(1);
    ATI_Nano43_raw_offset[0] = ATI_Nano43_raw_offset[0] + (myads1256.GetAnalog() - 2.5);

    myads1256.SetChannel(2);
    delay(1);
    ATI_Nano43_raw_offset[1] = ATI_Nano43_raw_offset[1] + (myads1256.GetAnalog() - 2.5);

    myads1256.SetChannel(3);
    delay(1);
    ATI_Nano43_raw_offset[2] = ATI_Nano43_raw_offset[2] + (myads1256.GetAnalog() - 2.5);

    myads1256.SetChannel(4);
    delay(1);
    ATI_Nano43_raw_offset[3] =  ATI_Nano43_raw_offset[3] + (myads1256.GetAnalog() - 2.5);

    myads1256.SetChannel(5);
    delay(1);
    ATI_Nano43_raw_offset[4] =  ATI_Nano43_raw_offset[4] + (myads1256.GetAnalog() - 2.5);

    myads1256.SetChannel(6);
    delay(1);
    ATI_Nano43_raw_offset[5] =  ATI_Nano43_raw_offset[5] + (myads1256.GetAnalog() - 2.5);
  }

  // 求取均值
  for(int i = 0;i<6;i++)
  {
    ATI_Nano43_raw_offset[i] = ATI_Nano43_raw_offset[i] / times;
  }

  // 打印均值
  Serial.print("ATI_Nano43_raw_offset voltage is: ");
  for(int i = 0;i < 6;i++)
  {
    Serial.print(ATI_Nano43_raw_offset[i], 6);
    Serial.print(',');
  }
  Serial.println("--"); 
}


// 获取原始电压数据(已经减去均值)
void Get_ATI_Nano43_raw()
{
  myads1256.SetChannel(1);
  delay(1);
  ATI_Nano43_raw_data[0] = myads1256.GetAnalog() - 2.5 - ATI_Nano43_raw_offset[0];

  myads1256.SetChannel(2);
  delay(1);
  ATI_Nano43_raw_data[1] = myads1256.GetAnalog() - 2.5 - ATI_Nano43_raw_offset[1];

  myads1256.SetChannel(3);
  delay(1);
  ATI_Nano43_raw_data[2] = myads1256.GetAnalog() - 2.5 - ATI_Nano43_raw_offset[2];

  myads1256.SetChannel(4);
  delay(1);
  ATI_Nano43_raw_data[3] = myads1256.GetAnalog() - 2.5 - ATI_Nano43_raw_offset[3];

  myads1256.SetChannel(5);
  delay(1);
  ATI_Nano43_raw_data[4] = myads1256.GetAnalog() - 2.5 - ATI_Nano43_raw_offset[4];

  myads1256.SetChannel(6);
  delay(1);
  ATI_Nano43_raw_data[5] = myads1256.GetAnalog() - 2.5 - ATI_Nano43_raw_offset[5];

}


// 原始电压转换为六维力力矩数据
void Transfer_ATI_Nano43_FT()
{
  // clear
  for(int i = 0;i < 3;i++)
  {
    ATI_Nano43_force_data[i] = 0;
    ATI_Nano43_torch_data[i] = 0;
  }
  
  // cal
  for(int i = 0;i < 6;i++)
  {
    // force x y z cal
    ATI_Nano43_force_data[0] = ATI_Nano43_force_data[0] + ATI_Nano43_matrix_Fx[i] * ATI_Nano43_raw_data[i];
    ATI_Nano43_force_data[1] = ATI_Nano43_force_data[1] + ATI_Nano43_matrix_Fy[i] * ATI_Nano43_raw_data[i];
    ATI_Nano43_force_data[2] = ATI_Nano43_force_data[2] + ATI_Nano43_matrix_Fz[i] * ATI_Nano43_raw_data[i];
    
    // torch x y z cal
    ATI_Nano43_torch_data[0] = ATI_Nano43_torch_data[0] + ATI_Nano43_matrix_Tx[i] * ATI_Nano43_raw_data[i];
    ATI_Nano43_torch_data[1] = ATI_Nano43_torch_data[1] + ATI_Nano43_matrix_Ty[i] * ATI_Nano43_raw_data[i];
    ATI_Nano43_torch_data[2] = ATI_Nano43_torch_data[2] + ATI_Nano43_matrix_Tz[i] * ATI_Nano43_raw_data[i];
  }

  // int32_t 处理
  for(int i = 0;i < 3;i++)
  {
    ATI_Nano43_force_data_t[i] = (int32_t)(1000.0 * ATI_Nano43_force_data[i]);
    ATI_Nano43_torch_data_t[i] = (int32_t)(1000.0 * ATI_Nano43_torch_data[i]);
  }

}


// 发送原始电压数据
void Send_ATI_Nano43_raw()
{
  for(int i = 0;i < 6;i++)
  {
    Serial.print(ATI_Nano43_raw_data[i], 6);
    Serial.print(',');
  }
  Serial.println("--"); 
}

// 发送计算得到的力 力矩数据
void Send_ATI_Nano43_FT()
{
  Serial.print("ATI-Nano43 FX, FY, FZ, TX, TY, TZ: ");
  for(int i = 0;i < 3;i++)
  {
    Serial.print(ATI_Nano43_force_data[i], 6);
    Serial.print(',');
  }

  for(int i = 0;i < 3;i++)
  {
    Serial.print(ATI_Nano43_torch_data[i], 6);
    Serial.print(',');
  }
  Serial.println("--");
  
}

//ANOv7上位机波形观察
//每一个协议帧只能最多发送10个数据量(不是byte)
//协议：0xAA+0xFF+0xF1+NUM(byte)+DATA(低位先发)+SC+AC 转换后 力&力矩数据*1,000
void ANOv7_ATI_DATA_short_Send1()
{
  unsigned char Max_data_num = 10;//每一帧最大发送数据量
  unsigned char _cnt_send = 0;//用于记录发送帧发送位数
  unsigned char _cnt_data = 0;//用于记录数据发送位数
  unsigned char SC = 0;//求和检验值
  unsigned char AC = 0;//累求和检验值

  /********************* 协议帧 发送过程（0xF1）  *********************/
  //帧头
  DataToSend[_cnt_send++] = 0xAA;
  DataToSend[_cnt_send++] = 0xFF;
  DataToSend[_cnt_send++] = 0xF1;
  DataToSend[_cnt_send++] = 6*4;//int32发送占四个字节
  // 三维力数据
  for(_cnt_data = 0;_cnt_data < 3;_cnt_data++)
  {
    DataToSend[_cnt_send++] = BYTE0(ATI_Nano43_force_data_t[_cnt_data]);
    DataToSend[_cnt_send++] = BYTE1(ATI_Nano43_force_data_t[_cnt_data]);
    DataToSend[_cnt_send++] = BYTE2(ATI_Nano43_force_data_t[_cnt_data]);
    DataToSend[_cnt_send++] = BYTE3(ATI_Nano43_force_data_t[_cnt_data]);
  }
  // 三维力矩数据
  for(_cnt_data = 0;_cnt_data < 3;_cnt_data++)
  {
    DataToSend[_cnt_send++] = BYTE0(ATI_Nano43_torch_data_t[_cnt_data]);
    DataToSend[_cnt_send++] = BYTE1(ATI_Nano43_torch_data_t[_cnt_data]);
    DataToSend[_cnt_send++] = BYTE2(ATI_Nano43_torch_data_t[_cnt_data]);
    DataToSend[_cnt_send++] = BYTE3(ATI_Nano43_torch_data_t[_cnt_data]);
  }
  //双校验
  for(unsigned char i = 0;i < (6*4+4);i++)
  {
    SC = DataToSend[i] + SC;
    AC = SC + AC;
  }

  DataToSend[_cnt_send++] = SC;
  DataToSend[_cnt_send++] = AC;

  //发送
  for(unsigned char i = 0;i < _cnt_send;i++)
  {
      Serial.write(DataToSend[i]);
  }
  /********************* 协议帧 发送过程（0xF1）  *********************/
}


//LED_Task 任务主体(兼容debug功能)
void LED_Task(void *ptParam) {  
  // 初始化当前tick时刻
  TickType_t xLastWakeTime = xTaskGetTickCount();
  // 定义单周期的tick数量
  const TickType_t xFrequency = 500; // 间隔 500 ticks = 0.5 seconds = 2Hz
  
  for(;;) //使用for更高效
  {
    vTaskDelayUntil(&xLastWakeTime, xFrequency);
    // LED_Task 主任务
    led_status = !led_status;
    if(led_status==true)
    {
      leds[0] = CRGB::Red;
      FastLED.show();
    }
    else
    {
      leds[0] = CRGB::Black;
      FastLED.show();
    }
    // 串口打印传感器信息-for debug
    if(SerialDebug == 1) {
        Send_ATI_Nano43_raw();
        Send_ATI_Nano43_FT();
      }
  }
}


//Acquire_Task 任务主体
void Acquire_Task(void *ptParam) {  
  // 初始化当前tick时刻
  TickType_t xLastWakeTime = xTaskGetTickCount();
  // 定义单周期的tick数量
  const TickType_t xFrequency = 10; // 间隔 10 ticks = 10 ms = 100hz
  
  for(;;) //使用for更高效
  {
    vTaskDelayUntil(&xLastWakeTime, xFrequency);
    
    // 首先查询datapack的Mutex锁是否被占用
    if (xSemaphoreTake(xMutex_datapack, timeOut) == pdPASS) { // 未被占用
      
      // Acquire_Task 主任务
      Get_ATI_Nano43_raw();
      
      xSemaphoreGive(xMutex_datapack); //释放钥匙
    }
    else{
      // 发现被占用 do nothing
    }
  }
}

//Send_Task 任务主体
void Send_Task(void *ptParam) {  
  // 初始化当前tick时刻
  TickType_t xLastWakeTime = xTaskGetTickCount();
  // 定义单周期的tick数量
  const TickType_t xFrequency = 10; // 间隔 10 ticks = 10 ms = 100Hz
  
  for(;;) //使用for更高效
  {
    vTaskDelayUntil(&xLastWakeTime, xFrequency);
    // 首先查询datapack的Mutex锁是否被占用
    if (xSemaphoreTake(xMutex_datapack, timeOut) == pdPASS) { // 未被占用
      
      // Send_Task 主任务 这里主任务必须尽量轻不要占用过多时间！！！(遵守先把数据提取出来，然后立刻释放，数据再发送出去)
      Transfer_ATI_Nano43_FT();
      xSemaphoreGive(xMutex_datapack); //释放钥匙
    }
    else{
      // 发现被占用 do nothing
    }

    // 发送单独放出来，不过多占用MUTEX时间
    ANOv7_ATI_DATA_short_Send1();
  }
}

// freertos任务创建
void freertos_task_create()
{
    xMutex_datapack = xSemaphoreCreateMutex(); //创建MUTEX
    xTaskCreatePinnedToCore(LED_Task, "LED_Blink", 1024 * 8, NULL, 1, NULL, 1);
    xTaskCreatePinnedToCore(Acquire_Task, "Data_Acquisition", 1024 * 8, NULL, 2, NULL, 1);
    vTaskDelay(1000); //让数据采集程序提前先运行一秒获取第一笔数据
    xTaskCreatePinnedToCore(Send_Task, "Data_Send", 1024 * 8, NULL, 2, NULL, 1);
    Serial.println("System has created 3 tasks[LED_Blink-2Hz, Data_Acquisition-100Hz, Datapack_Send-100Hz]");
}


void setup() {
  FastLED.addLeds<WS2812, DATA_PIN, RGB>(leds, NUM_LEDS);  // GRB ordering is typical
  delay(50);
  Serial.begin(115200);

  // reset the sync to start the whole system.
  pinMode( PIN_SYNC, OUTPUT );
  digitalWrite( PIN_SYNC , LOW ); 
  delay(100);
  digitalWrite( PIN_SYNC , HIGH ); 
  
  myads1256.SetPinRST(PIN_RST);
  myads1256.SetPinDRDY(PIN_DRDY);
  myads1256.SetPinCS(PIN_CS);
  myads1256.SetPinSCK(PIN_SCK);
  myads1256.SetPinDIN(PIN_DIN);
  myads1256.SetPinDOUT(PIN_DOUT);

  while (myads1256.OpenDevice() == false);  //Initiallization
  myads1256.SetPGA(1);
  myads1256.SetSPS(1000);
  myads1256.SetChannel(1);
  tempfloat = myads1256.GetAnalog();  //Pre-read one data to clear data register


  // 由于ADS1256读数存在bug, 所以使用led指示复位
  // Turn the LED on, then pause
  leds[0] = CRGB::Red;
  FastLED.show();
  Serial.println("ESP32-S3 WS2812 Green.");
  delay(2000);
  // Now turn the LED off, then pause
  leds[0] = CRGB::Green;
  FastLED.show();
  Serial.println("ESP32-S3 WS2812 Red.");
  delay(2000);
  // Now turn the LED off, then pause
  leds[0] = CRGB::Blue;
  FastLED.show();
  Serial.println("ESP32-S3 WS2812 Blue.");
  delay(1000);
  // Now turn the LED off, then pause
  leds[0] = CRGB::Black;
  FastLED.show();
  Serial.println("ESP32-S3 WS2812 OFF.");
  delay(500);
  
  // 统计offset voltage
  STA_ATI_Nano43_offset(100);

  // 开启freertos
  freertos_task_create();

}

void loop() {

  // Get_ATI_Nano43_raw();
  // Transfer_ATI_Nano43_FT();
  // ANOv7_ATI_DATA_short_Send1();
  // // Send_ATI_Nano43_raw();
  // // Send_ATI_Nano43_FT();
  // delay(500);  
}


// #include <Arduino.h>
// #include "ADS1256.h"

// #define PIN_DRDY 15
// #define PIN_CS 16
// #define PIN_RST 17
// #define PIN_SYNC 18

// #define PIN_SCK 12
// #define PIN_DIN 13
// #define PIN_DOUT 14


// ADS1256 myads1256;    //ADS1256 ADCShield Class
// float tempfloat = 0;
// float datach0 = 0;  //Single-Ended Input 0, Channel 0
// float datach1 = 0;  //Single-Ended Input 1, Channel 1
// float datach2 = 0;  //Single-Ended Input 2, Channel 2
// float datach3 = 0;  //Single-Ended Input 3, Channel 3
// float datach4 = 0;  //Single-Ended Input 4, Channel 4
// float datach5 = 0;  //Single-Ended Input 5, Channel 5
// float datach6 = 0;  //Single-Ended Input 6, Channel 6
// float datach7 = 0;  //Single-Ended Input 7, Channel 7

// void setup() {
//   // put your setup code here, to run once:
//   Serial.begin(115200);

//   pinMode( PIN_SYNC, OUTPUT );
//   digitalWrite( PIN_SYNC , LOW ); 
//   delay(100);
//   digitalWrite( PIN_SYNC , HIGH ); 
  
//   myads1256.SetPinRST(PIN_RST);
//   myads1256.SetPinDRDY(PIN_DRDY);
//   myads1256.SetPinCS(PIN_CS);
//   myads1256.SetPinSCK(PIN_SCK);
//   myads1256.SetPinDIN(PIN_DIN);
//   myads1256.SetPinDOUT(PIN_DOUT);

//   while (myads1256.OpenDevice() == false);  //Initiallization
//   myads1256.SetPGA(1);
//   myads1256.SetSPS(1000);
//   myads1256.SetChannel(1);
//   tempfloat = myads1256.GetAnalog();  //Pre-read one data to clear data register

// }

// void loop() {

//   myads1256.SetChannel(1);
//   delay(1);
//   datach1 = myads1256.GetAnalog() - 2.5;

//   myads1256.SetChannel(2);
//   delay(1);
//   datach2 = myads1256.GetAnalog() - 2.5;

//   myads1256.SetChannel(3);
//   delay(1);
//   datach3 = myads1256.GetAnalog() - 2.5;

//   myads1256.SetChannel(4);
//   delay(1);
//   datach4 = myads1256.GetAnalog() - 2.5;

//   myads1256.SetChannel(5);
//   delay(1);
//   datach5 = myads1256.GetAnalog() - 2.5;

//   myads1256.SetChannel(6);
//   delay(1);
//   datach6 = myads1256.GetAnalog() - 2.5;

//   Serial.print(datach1, 6);
//   Serial.print(',');
//   Serial.print(datach2, 6);
//   Serial.print(',');
//   Serial.print(datach3, 6);
//   Serial.print(',');
//   Serial.print(datach4, 6);
//   Serial.print(',');
//   Serial.print(datach5, 6);
//   Serial.print(',');
//   Serial.println(datach6, 6);

//   delay(50);  
// }

// #include <Arduino.h>

// void setup() {

//   Serial.begin(115200);
//   delay(100);
// }

// void loop() {

//   Serial.println("esp32s3_serial_test.");
//   delay(500);  
// }
