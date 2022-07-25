# #!/usr/bin/python
# # -*-coding: utf-8 -*-
# # author: Zenan-SSR
# # data: 2022年7月23日
# # description: 读取 NI-cDAQ_9174基于NI_9220采集卡的nano43数据并经过转置矩阵转化为六轴力和力矩信息.
# # project: Zenan-TENG压力解耦项目
#
# import nidaqmx
# import numpy as np
#
#
# # 00-6*6 transfer martix
# # download from https://www.ati-ia.com/library/software/ftdigitaldownload/getcalfiles.aspx with serial number FT41118
# transfer_martix = np.array([[0.01987,  -0.00859,   0.02845,   2.20608,   0.07243,  -2.21747],
#                             [0.00614,  -2.60020,   0.03938,   1.30049,  -0.09713,   1.28913],
#                             [1.50115,  -0.02542,   1.52239,  -0.01504,   1.52065,  -0.00098],
#                             [-0.03928, -11.53765,  22.19625,   5.62507, -22.66068,   5.61595],
#                             [-25.28900,   0.43679,  12.70831,  -9.90027,  12.25530,   9.92669],
#                             [-0.08965, -18.96174,  -0.22897, -18.20473,   0.75572, -18.66394]])
#
#
# # 01-get offset of every channel(statistic about 100 times and average them).
# def get_offset_data(sample_num):
#     offset_data = np.zeros(6)
#     with nidaqmx.Task() as task:
#         # here I just connect NI_9220 to the third channel in the NI-cDAQ_9174.
#         # And read the first 6 channels.
#         task.ai_channels.add_ai_voltage_chan("cDAQ9220Mod1/ai0:5")
#         for i in range(sample_num):
#             offset_data = offset_data + np.array(task.read())
#         offset_data = offset_data / sample_num
#     return offset_data
#
#
# # 02-read the first 6 channels of the NI_9220 采集卡
# def read_raw_data(offset_data):
#     with nidaqmx.Task() as task:
#         # here I just connect NI_9220 to the third channel in the NI-cDAQ_9174.
#         # And read the first 6 channels.
#         task.ai_channels.add_ai_voltage_chan("cDAQ9220Mod1/ai0:5")
#         raw_data = np.array(task.read()) - offset_data
#     return raw_data
#
#
# # 03-transfer the raw data to real force and torch.
# def cal_FT(raw_data):
#     real_data = np.dot(transfer_martix, raw_data.T)
#     return real_data
#
#
# if __name__ == '__main__':
#     Nano43_sample_num = 10
#     Nano43_offset_data = get_offset_data(Nano43_sample_num)
#     print(f'Nano43_offset_data = {Nano43_offset_data}')
#
#     count = 0
#     while 1:
#         count = count + 1
#         Nano43_raw_data = read_raw_data(Nano43_offset_data)
#         # print(f'Nano43_raw_data = {Nano43_raw_data}')
#         Nano43_real_data = cal_FT(Nano43_raw_data)
#         print(f'The {count} Nano43_real_data = {Nano43_real_data}')

#!/usr/bin/python
# -*-coding: utf-8 -*-
# author: Zenan-SSR
# data: 2022年7月23日
# description: 读取 NI-cDAQ_9174基于NI_9220采集卡的nano43数据并经过转置矩阵转化为六轴力和力矩信息.
# project: Zenan-TENG压力解耦项目

import nidaqmx
import numpy as np


# 00-6*6 transfer martix
# download from https://www.ati-ia.com/library/software/ftdigitaldownload/getcalfiles.aspx with serial number FT41118
transfer_martix = np.array([[0.01987,  -0.00859,   0.02845,   2.20608,   0.07243,  -2.21747],
                            [0.00614,  -2.60020,   0.03938,   1.30049,  -0.09713,   1.28913],
                            [1.50115,  -0.02542,   1.52239,  -0.01504,   1.52065,  -0.00098],
                            [-0.03928, -11.53765,  22.19625,   5.62507, -22.66068,   5.61595],
                            [-25.28900,   0.43679,  12.70831,  -9.90027,  12.25530,   9.92669],
                            [-0.08965, -18.96174,  -0.22897, -18.20473,   0.75572, -18.66394]])


# 01-get offset of every channel(statistic about 100 times and average them).
def get_offset_data(sample_num):
    offset_data = np.zeros(6)
    with nidaqmx.Task() as task:
        # here I just connect NI_9220 to the third channel in the NI-cDAQ_9174.
        # And read the first 6 channels.
        task.ai_channels.add_ai_voltage_chan("cDAQ9189-1FB2CECMod1/ai0:5")
        for i in range(sample_num):
            offset_data = offset_data + np.array(task.read())
        offset_data = offset_data / sample_num
    return offset_data


# 02-read the first 6 channels of the NI_9220 采集卡
def read_raw_data(offset_data):
    with nidaqmx.Task() as task:
        # here I just connect NI_9220 to the third channel in the NI-cDAQ_9174.
        # And read the first 6 channels.
        task.ai_channels.add_ai_voltage_chan("cDAQ9189-1FB2CECMod1/ai0:5")
        raw_data = np.array(task.read()) - offset_data
    return raw_data


# 03-transfer the raw data to real force and torch.
def cal_FT(raw_data):
    real_data = np.dot(transfer_martix, raw_data.T)
    return real_data


if __name__ == '__main__':
    Nano43_sample_num = 1
    Nano43_offset_data = get_offset_data(Nano43_sample_num)
    print(f'Nano43_offset_data = {Nano43_offset_data}')
    Nano43_offset_data = np.zeros(6)

    count = 0
    while 1:
        count = count + 1
        Nano43_raw_data = read_raw_data(Nano43_offset_data)
        print(f'Nano43_raw_data = {Nano43_raw_data}')
        Nano43_real_data = cal_FT(Nano43_raw_data)
        # print(f'The {count} Nano43_real_data = {Nano43_real_data}')

