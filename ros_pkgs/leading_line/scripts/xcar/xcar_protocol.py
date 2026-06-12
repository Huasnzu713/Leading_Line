#!/usr/bin/env python
# -*- coding: utf-8 -*-
from serial import Serial
import struct
from threading import Thread, Lock, Event

class XcarData:
    # 底板上报的状态量
    pos_x = 0                           # m
    pos_y = 0                           # m
    yaw = 0                             # rad
    speed_x = 0                         # m/s
    speed_y = 0                         # m/s
    angular = 0                         # rad/s
    dist = [0, 0, 0, 0]                 # cm
    bat = 0                             # 0.1V
    temp = 0                            # 0.1℃
    humi = 0                            # %Rh
    pressure = 0                        # Pa
    light = 0                           # lux
    tvoc = 0                            # ppm
    smoke = 0                           # ppm
    servo_status = [0, 0, 0, 0, 0, 0]   # °
    # 3399下发的控制量
    speed_set_x = 0                     # m/s
    speed_set_y = 0                     # m/s
    angular_set = 0                     # rad/s
    servo_ctrl = [0, 0, 0, 0, 0, 0]     # °， -128~127
    servo_ctrl_flag = 0                 # unsigned char
    servo_ctrl_time = 1000

class XcarComm(XcarData):
    xcar_ser = None
    ser_data = None
    data_no = 0
    new_data_flag = 0   # 收到新数据的标志，bit0-bit3分别对应收到type为0x01-0x04的数据
    isRunning = False
    send_lock = None
    recv_event = None
    
    def __init__(self):
        self.xcar_ser = Serial('/dev/ttyXCar', 115200, timeout=0.02)
        self.ser_data = bytearray([])
        self.send_lock = Lock()
        self.recv_event = Event()
        
    def __frame_analyses(self, length):
        '''
        私有方法，禁止外部调用
        解析、处理self.ser_data中的剩余数据
        返回值：剩余数据的长度
        '''
        # 数据帧：帧头(2bytes)-编号(1byte)-数据类型（1byte)-数据长度(1byte)-数据内容(nbytes)-校验和(1byte)
        if length < 6:
            return -10
        # 搜索帧头
        for i in range(0, length-1):
            if self.ser_data[i] == 0x2B and self.ser_data[i+1] == 0xA2:
                self.ser_data = self.ser_data[i:]
                length -= i
                break
        else:
            self.ser_data = self.ser_data[-1:]
            return -1
        # 编号获取
        self.data_no = (self.ser_data[2] + 1) % 255
        # 数据类型
        type = self.ser_data[3]
        if (type & 0x80) == 1:
            self.ser_data = self.ser_data[2:]
            return -2
        # 数据长度
        data_len = self.ser_data[4]
        if length < data_len + 6:
            return -3
        # 校验和
        checksum = 0
        for i in range(0, data_len + 6 - 1):
            checksum += self.ser_data[i]
        else:
            checksum &= 0xFF
        if checksum != self.ser_data[data_len + 5]:
            return -4
        # 数据处理
        self.ser_data = self.ser_data[5:]
        if type == 0x01:    # 里程计状态
            temp = struct.unpack('>iihhhh', self.ser_data[:16])
            self.pos_x = temp[0] / 1000.0
            self.pos_y = temp[1] / 1000.0
            self.yaw = temp[2] / 10000.0
            self.speed_x = temp[3] / 10000.0
            self.speed_y = temp[4] / 10000.0
            self.angular = temp[5] / 10000.0
            self.new_data_flag |= 1
        elif type == 0x02:  # 测距传感器状态
            self.dist = struct.unpack('>hhhh', self.ser_data[:8])
            self.new_data_flag |= 2
        elif type == 0x03:  # 环境传感器状态
            temp = struct.unpack('>Bhhhhhh', self.ser_data[:13])
            self.bat = temp[0]
            self.temp = temp[1]
            self.humi = temp[2]
            self.pressure = temp[3] * 10
            self.light = temp[4]
            self.tvoc = temp[5]
            self.smoke = temp[6]
            self.new_data_flag |= 4
        elif type == 0x04:  # 舵机角度状态
            self.servo_status = struct.unpack('>bbbbbb', self.ser_data[:6])
            self.new_data_flag |= 8
        length -= data_len + 6
        if length > 0:
            self.ser_data = self.ser_data[data_len+1:]
        else:
            self.ser_data = self.ser_data[0:0]
        self.recv_event.set()
        return length
        
    def __xcar_read(self):
        '''
        应在首次调用前执行reset_input_buffer()函数，清空输入数组
        应在线程中不间断地调用本函数（read函数有[自行设置]的阻塞）
        '''
        while self.isRunning:
            self.ser_data.extend(bytearray(self.xcar_ser.read(10000)))
            length = len(self.ser_data)
            while length > 0:
                length = self.__frame_analyses(length)
                #if length < 0:
                #    print "err: %d" % length
        
    def start(self):
        '''
        创建线程，获取并解析串口数据
        '''
        if self.isRunning == False:
            t = Thread(target=self.__xcar_read)     # 创建线程
            t.setDaemon(True)                       # 设置为后台线程，这里默认是False，设置为True之后则主线程不用等待子线程
            self.xcar_ser.reset_input_buffer()
            self.isRunning = True
            t.start()  # 开启线程
        
    def stop(self):
        '''
        停止串口数据的接受与解析
        '''
        if self.isRunning == True:
            self.isRunning = False
        
    def send(self, type):
        '''
        发送指定类型的数据帧到底板，调用前应先设置XcarData中的控制类数据
        '''
        if type < 0x81 or type > 0x84:
            return 0
        self.send_lock.acquire()
        # 数据帧：帧头(2bytes)-编号(1byte)-数据类型（1byte)-数据长度(1byte)-数据内容(nbytes)-校验和(1byte)
        cmd = bytearray([0x2B, 0xA2, self.data_no, type])
        self.data_no = (self.data_no + 1) % 255
        # 根据type决定要打包的数据
        if type == 0x81:
            data_len = 6
            temp = struct.pack('>hhh', int(self.speed_set_x * 1000), int(self.speed_set_y * 1000), int(self.angular_set * 1000))
        elif type == 0x82:
            data_len = 7
            temp = struct.pack('>BbbbbbB', self.servo_ctrl_flag, *(self.servo_ctrl[1:] + [int(self.servo_ctrl_time / 10)]))
        elif type == 0x83:
            data_len = 2
            temp = struct.pack('>bB', self.servo_ctrl[0], int(self.servo_ctrl_time / 10))
        elif type == 0x84:
            data_len = 0
            temp = []
        else:
            self.send_lock.release()
            return 0
        cmd.append(data_len)
        cmd.extend(temp)
        checksum = 0
        for data in cmd:
            checksum += data
        else:
            checksum &= 0xFF
        cmd.append(checksum)
        self.xcar_ser.write(cmd)
        send_len = len(cmd)
        self.send_lock.release()
        return send_len
        
