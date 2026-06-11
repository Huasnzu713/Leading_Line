#!/usr/bin/env python
# -*- coding: utf-8 -*-
from xcar_data import XcarComm
from threading import Thread
import rospy
from tf import TransformBroadcaster
from std_msgs.msg import Int32MultiArray, MultiArrayDimension, Int16MultiArray, Int32
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Range
from geometry_msgs.msg import Twist, TransformStamped
from math import sin, cos, modf

class XcarRos:
    xcar_comm  = None
    pub_tf     = None
    pub_arm    = None
    pub_odom   = None
    pub_sonar  = [None, None, None, None]
    pub_sensor = None
    # 线程相关
    isRunning_pub = False
    isRunning_spin = False
    
    def __init__(self):
        rospy.init_node('xcar_ros')
        self.xcar_comm    = XcarComm()
        self.pub_tf       = TransformBroadcaster()
        self.pub_odom     = rospy.Publisher('/odom', Odometry, queue_size=10)
        self.pub_arm      = rospy.Publisher('/xcar/arm_status', Int32MultiArray, queue_size=10)
        self.pub_sonar[0] = rospy.Publisher('/xcar/sonar1', Range, queue_size=10)
        self.pub_sonar[1] = rospy.Publisher('/xcar/sonar2', Range, queue_size=10)
        self.pub_sonar[2] = rospy.Publisher('/xcar/sonar3', Range, queue_size=10)
        self.pub_sonar[3] = rospy.Publisher('/xcar/sonar4', Range, queue_size=10)
        self.pub_sensor   = rospy.Publisher('/xcar/sensors', Int32MultiArray, queue_size=10)
 
    def start(self):
        if self.isRunning_spin == False:
            rospy.Subscriber('/cmd_vel', Twist, self.cmd_vel_cb, queue_size=10)
            rospy.Subscriber('/xcar/arm', Int16MultiArray, self.xcar_arm_cb, queue_size=10)
            rospy.Subscriber('/xcar/gripper', Int32, self.xcar_gripper_cb, queue_size=10)
            rospy.Subscriber('/xcar/arm_status_req', Int32, self.xcar_armreq_cb, queue_size=10)
            #t_spin = Thread(target=self.ros_spin)     # 创建线程
            #t_spin.setDaemon(False)                       # 设置为后台线程，这里默认是False，设置为True之后则主线程不用等待子线程
            self.isRunning_spin = True
            #t_spin.start()  # 开启线程
        
        if self.isRunning_pub == False:
            t_pub = Thread(target=self.pub_datarecv)    # 创建线程
            t_pub.setDaemon(True)                       # 设置为后台线程，这里默认是False，设置为True之后则主线程不用等待子线程
            self.isRunning_pub = True
            t_pub.start()  # 开启线程
            self.xcar_comm.start()
        
        
    def stop(self):
        if self.isRunning_pub == True:
            self.xcar_comm.stop()
            self.isRunning_pub = False
        
    def ros_spin(self):
        rospy.spin()
    
    def pub_datarecv(self):
        # tf变换
        odom_tf = TransformStamped()
        odom_tf.header.frame_id = "odom"
        odom_tf.child_frame_id = "base_link"
        # 里程计状态
        odom = Odometry()
        odom.header.frame_id = "odom"
        odom.child_frame_id = "base_link"
        # 舵机角度状态
        arm_status = Int32MultiArray()
        arm_status.layout.dim = [MultiArrayDimension("XCAR Arm Status", 1, 6)]
        arm_status.layout.data_offset = 0
        # 测距传感器状态
        sonar = [Range(), Range(), Range(), Range()]
        for item in sonar:
            item.min_range = 0.2
            item.max_range = 7.2
            item.field_of_view = 0.3
            item.radiation_type = Range.ULTRASOUND
        sonar[0].header.frame_id = "sonar1"
        sonar[1].header.frame_id = "sonar2"
        sonar[2].header.frame_id = "sonar3"
        sonar[3].header.frame_id = "sonar4"
        # 环境传感器状态
        sensors = Int32MultiArray()
        sensors.layout.dim = [MultiArrayDimension("XCAR Sensors", 1, 12)]
        sensors.layout.data_offset = 0
        # 线程主体
        while self.isRunning_pub:
            if self.xcar_comm.recv_event.wait(1.0) == False:
                continue
            stamp = rospy.Time.now()
            if self.xcar_comm.new_data_flag & 1:    # 收到底板的里程计状态
                # tf
                odom_tf.header.stamp = stamp
                odom_tf.transform.translation.x = self.xcar_comm.pos_x
                odom_tf.transform.translation.y = self.xcar_comm.pos_y
                odom_tf.transform.rotation.z = sin(self.xcar_comm.yaw / 2)
                odom_tf.transform.rotation.w = cos(self.xcar_comm.yaw / 2)
                self.pub_tf.sendTransformMessage(odom_tf)
                # odom
                odom.header.stamp = stamp
                odom.pose.pose.position.x = self.xcar_comm.pos_x
                odom.pose.pose.position.y = self.xcar_comm.pos_y
                odom.pose.pose.orientation.z = sin(self.xcar_comm.yaw / 2)
                odom.pose.pose.orientation.w = cos(self.xcar_comm.yaw / 2)
                odom.twist.twist.linear.x = self.xcar_comm.speed_x
                odom.twist.twist.linear.y = self.xcar_comm.speed_y
                odom.twist.twist.angular.z = self.xcar_comm.angular
                self.pub_odom.publish(odom)
                self.xcar_comm.new_data_flag &= ~1
            if self.xcar_comm.new_data_flag & 2:    # 收到底板的测距传感器状态
                for i in range(0,4):
                    sonar[i].header.stamp = stamp
                    sonar[i].range = self.xcar_comm.dist[i] / 100.0
                    self.pub_sonar[i].publish(sonar[i])
                self.xcar_comm.new_data_flag &= ~2
            if self.xcar_comm.new_data_flag & 4:    # 收到底板的环境传感器状态
                sensors.data = [self.xcar_comm.bat, self.xcar_comm.temp, self.xcar_comm.humi, 0, self.xcar_comm.pressure, self.xcar_comm.light, self.xcar_comm.tvoc, self.xcar_comm.smoke]
                sensors.data.extend(self.xcar_comm.dist)
                self.pub_sensor.publish(sensors)
                self.xcar_comm.new_data_flag &= ~4
            if self.xcar_comm.new_data_flag & 8:    # 收到底板的舵机角度状态
                arm_status.data = self.xcar_comm.servo_status
                self.pub_arm.publish(arm_status)
                self.xcar_comm.new_data_flag &= ~8
        
    #rostopic pub /cmd_vel geometry_msgs/Twist -r 10 '{linear: {x: 0.2, y: 0, z: 0}, angular: {x: 0, y: 0, z: 0}}'
    def cmd_vel_cb(self, msg):
        if type(msg) == Twist:
            self.xcar_comm.speed_set_x = msg.linear.x
            self.xcar_comm.speed_set_y = msg.linear.y
            self.xcar_comm.angular_set = msg.angular.z
            self.xcar_comm.send(0x81)
            
    def xcar_arm_cb(self, msg):
        if type(msg) == Int16MultiArray:
            if len(msg.data) == 5:
                self.xcar_comm.servo_ctrl_time = 1000
            elif len(msg.data) == 6:
                self.xcar_comm.servo_ctrl_time = msg.data[5]
                if self.xcar_comm.servo_ctrl_time > 2550:
                    self.xcar_comm.servo_ctrl_time = 2550
            else:
                return
            self.xcar_comm.servo_ctrl_flag = 0x3E
            for i in range(0, 5):
                if msg.data[i] > 127:
                    msg.data[i] = 127
                elif msg.data[i] < -128:
                    msg.data[i] = -128
                self.xcar_comm.servo_ctrl[i+1] = msg.data[i]
            self.xcar_comm.send(0x82)
            
    def xcar_gripper_cb(self, msg):
        if type(msg) == Int32:
            self.xcar_comm.servo_ctrl[0] = msg.data
            self.xcar_comm.servo_ctrl_time = 1000
            self.xcar_comm.send(0x83)
            
    def xcar_armreq_cb(self, msg):
        if type(msg) == Int32:
            self.xcar_comm.send(0x84)
            
if __name__ == '__main__':
    xcar = XcarRos()
    xcar.start()
    xcar.ros_spin()
