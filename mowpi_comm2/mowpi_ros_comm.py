#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

import math
from math import pi
from geometry_msgs.msg import Twist, Quaternion, Point, Pose, Vector3, Vector3Stamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Int16, Float32
from sensor_msgs.msg import Imu

import mowpi_comm2.transformations as transformations # for euler to quat
from tf2_ros.transform_broadcaster import TransformBroadcaster
from geometry_msgs.msg import TransformStamped

from my_interfaces.msg import OdomInputs
from my_interfaces.srv import SetInt

import serial
import sys
import numpy as np
import time

from functools import partial #allows more arguments to a callback
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

class ReadLine:
    def __init__(self, s):
        self.buf = bytearray()
        self.s = s
    
    def readline(self):
        i = self.buf.find(b"\n")
        if i >= 0:
            r = self.buf[:i+1]
            self.buf = self.buf[i+1:]
            return r
        count = 0
        while True:
            i = max(1, min(2048, self.s.in_waiting))
            data = self.s.read(i)
            i = data.find(b"\n")
            if i >= 0:
                r = self.buf + data[:i+1]
                self.buf[0:] = data[i+1:]
                return r
            else:
                self.buf.extend(data)
            count += 1
            if count > 100:
                return []

class MicroSerial():

    def __init__(self):
        #self.lock = lock
        # 115200 bits/sec = 14400 bytes/sec = 14 bytes/msec
        # When reading a number in this application, max bytes 7 bytes ("-32767\n")
        # So try timeout = 0.002 sec or 28 bytes per readline()
        self.serial = serial.Serial('/dev/mowpi_feather', baudrate=115200, timeout=0.01) #CHANGE TIMEOUT TO SHORT HERE
        
    def safe_write(self, val_str):
        #self.lock.acquire()
        self.serial.write(val_str.encode())
        #self.lock.release()

    def safe_read(self):
        #self.lock.acquire()
        val_read_str = self.serial.readline()
        #self.lock.release()
        return val_read_str
        
    def flush(self):
        self.serial.flushInput()
        self.serial.flushOutput()

class Controller():
    
    def __init__(self, micro):
        self.micro = micro
        self.speed = 0.0
        self.curv = 0.0

    def write_speed_curv(self, speed_in, curv_in):
        #  A1/1/<speed_byte> <curv_byte>
        if np.isnan(speed_in) or np.isnan(curv_in):
            speed_in = 0
            curv_in = 0
        self.speed = speed_in # m/s
        self.curv = curv_in # rad/sec
        #print('speed, curv', speed_in, curv_in)
        
        raw_speed = int(speed_in*100.0)+120
        raw_omega = int(curv_in*180.0/pi)+120
        speed_byte = bytes([ raw_speed & 0xff]) #-120 to 120 cm/sec
        curv_byte = bytes([ (raw_omega) & 0xff]) # -120 to 120 deg/sec
        #print('speed_byte, curv_byte', speed_byte, curv_byte)
        
        self.micro.safe_write('A1/1/' + str(raw_speed) + '/' + str(raw_omega) + '/')
        return

class MicroBridge(Node):
    def __init__(self):
        super().__init__("mowpi_ros_comm")
        
        self.cmd_sub = self.create_subscription(Twist, "cmd_vel", self.drive_callback, 1)
        self.blade_cmd_sub = self.create_subscription(Int16, "blade_cmd", self.blade_callback, 2)
        self.imu_sub = self.create_subscription(Imu, "imu", self.imu_callback, 2)
        self.yawkf_sub = self.create_subscription(Float32, "yawkf_deg", self.yawkf_callback, 2)
        
        odom_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT,
            history=QoSHistoryPolicy.RMW_QOS_POLICY_HISTORY_KEEP_LAST,
            depth=5
        )
        self.odom_pub = self.create_publisher(Odometry, "odom_raw", qos_profile = odom_qos)
        self.odomIn_pub = self.create_publisher(OdomInputs, "odomInputs",5)

        self.odom_broadcaster = TransformBroadcaster(self)
        self.tfs = TransformStamped()
        self.tfs.header.frame_id = "odom"
        self.tfs.child_frame_id = "base_link"
        self.tfs.transform.translation.z = 0.0
        
        self.laser_tfs = TransformStamped()
        self.laser_tfs.header.frame_id = "base_link"
        self.laser_tfs.child_frame_id = "laser"
        self.laser_tfs.transform.translation.x = 0.2
        self.laser_tfs.transform.translation.y = 0.0
        self.laser_tfs.transform.translation.z = 0.0
        
        self.micro = MicroSerial()
        self.controller = Controller(self.micro)
        self.speed = 0.0
        self.curv = 0.0
        
        now_stamp = self.get_clock().now().to_msg()
        self.prev_time = now_stamp
        self.error_time = now_stamp
        
        self.accx = 0.0
        self.accy = 0.0
        self.gyroz_rad = 0.0

        self.enc_total = 0
        self.roll_rad = 0
        self.pitch_rad = 0
        self.blade_status = -1
        
        self.dist_sum = 0
        self.time_sum = 0
        self.vx = 0.0
        self.stopped_db_limit = 10
        self.stopped_speed = 0.05
        self.stopped_gyro_deg = 5.0
        self.stopped_count = 0

        self.bot_deg_prev = 0.
        self.bot_deg = 0.
        self.micro_bot_deg = 0.
        self.botx = 0.
        self.boty = 0.
        
        self.gyro_sum = 0.0
        self.gyro_count = 0
        
        # NXP on breadboard
        self.gyro_bias_rad = 0.06*pi/180
        self.neg_gyro_scale_factor = 1.005
        self.pos_gyro_scale_factor = 1.001
        
        self.bad_serial_count = 0
        
        self.mow_area_id = 0
        
        # original NXP mounted
        #self.gyro_bias_rad = 0.15*pi/180
        #self.neg_gyro_scale_factor = 1.014
        #self.pos_gyro_scale_factor = 0.997
        
        # micro scale factor, bno
        self.micro_gyro_scale_factor = 0.99 #0.9896
        
        self.timer = self.create_timer(1./20., self.update_odom)
        
        self.get_logger().info("Started Zumo Serial ROS Comm")
        
        time.sleep(0.1)
    
    def dt_to_sec(self, stampA, stampB):
        return stampA.sec + stampA.nanosec * 10**-9 - stampB.sec - stampB.nanosec * 10**-9
                
    def drive_callback(self, data):
        v = data.linear.x
        w = data.angular.z
        max_speed = 1.2
        max_omega = 120*pi/180.0
        
        if v > max_speed:
            v = max_speed
        elif v < -max_speed:
            v = -max_speed
        
        if w > max_omega:
            w = max_omega
        elif w < -max_omega:
            w = -max_omega
        
        self.speed = v
        self.curv = w
        
        self.controller.write_speed_curv(self.speed, self.curv)
            
        #print("enc_total: ")
        #print(self.enc_total)
        #print('roll rad: ',self.roll_rad, ', pitch rad: ',self.pitch_rad)
    
    def blade_callback(self,bld):
        bld_cmd = bld.data
        bld_str = 'A2/8/' + str(bld_cmd) + '/'
        print("pre cmd: ", bld_str)
        if(bld_cmd == 1 and self.blade_status == 0):
            self.ard.safe_write(bld_str)
            self.blade_status = 1
            print("blade cmd: ", bld_str)
        elif(bld_cmd == 0 and self.blade_status != 0):
            self.ard.safe_write(bld_str)
            self.blade_status = 0
            print("blade cmd: ", bld_str)
        
    def imu_callback(self, data):
        self.accx = data.linear_acceleration.x
        self.accy = data.linear_acceleration.y
        self.gyroz_rad = data.angular_velocity.z
        if(self.gyro_count < 100):
            self.gyro_count += 1
            self.gyro_sum += self.gyroz_rad
            if(self.gyro_count == 100):
                #self.gyro_bias_rad = self.gyro_sum / self.gyro_count
                print('gyro bias deg: ', self.gyro_bias_rad*180/pi)
                
    def yawkf_callback(self, msg):
        self.bot_deg = msg.data
        
    def call_set_mow(self, mow_area_id):
        client = self.create_client(SetInt, "set_mow_area")
        while not client.wait_for_service(1.0):
            self.get_logger().warn("Waiting for server set_mow_area")
            
        request = SetInt.Request()
        request.data = mow_area_id
        future = client.call_async(request) #call or call_async
        future.add_done_callback(partial(self.call_set_mow_callback, mow_area=mow_area_id) ) # not a ros feature, just python future feature
                    
    def call_set_mow_callback(self, future, mow_area):
        try:
            response = future.result()
            self.get_logger().info('response: %d' % (response.success))
            if response.success:
                self.mow_area_id = mow_area
                self.get_logger().info('mow area %d' % (self.mow_area_id))
        except Exception as e:
            node.get_logger().error("Service call failed %r" % (e,) )
    
    def update_odom(self):
        t2 = self.get_clock().now().to_msg()
        t1 = self.prev_time
        dt = self.dt_to_sec(t2,t1)
        
        BOT_WIDTH = (28.0 * 2.54 / 100.0) #meters
        COUNTS_PER_METER = 162.52
        
        # Process gyro z
        gyro_thresh_dps = 0.3
        g_bias_dps = self.gyro_bias_rad*180/pi
        MAX_DTHETA_GYRO_deg = 100.0
        
        gyroz_raw_dps = float(self.gyroz_rad) * 180.0 / pi
        
        if(abs(gyroz_raw_dps-g_bias_dps) < gyro_thresh_dps):
            gz_dps = 0
            dtheta_gyro_deg = 0
        else:
            gz_dps = gyroz_raw_dps-g_bias_dps
            sf = self.pos_gyro_scale_factor
            if(gz_dps < 0):
                sf = self.neg_gyro_scale_factor
            dtheta_gyro_deg = gz_dps*dt*sf
            
        # Read encoder delta   
        try: 
            self.micro.safe_write('A3/4/')
            s = self.micro.safe_read()
            if len(s) == 0:
                delta_enc_left = 0
                self.bad_serial_count += 1
            else:
                delta_enc_left = int(s)
            s = self.micro.safe_read()
            if len(s) == 0:
                delta_enc_right = 0
            else:
                delta_enc_right = int(s)
            s = self.micro.safe_read()
            if len(s) == 0:
                print("gyro serial timeout")
                micro_gyro_z_deg = gz_dps
            else:
                micro_gyro_z_deg = float(int(s))/100.
            #s = self.micro.safe_read()
            #micro_delta_yaw_deg = float(int(s))/1000. * self.micro_gyro_scale_factor
        except:
            self.bad_serial_count += 1
            delta_enc_left = 0
            delta_enc_right = 0
            micro_gyro_z_deg = 0.0
            micro_delta_yaw_deg = 0.0
            error_delta_time = self.dt_to_sec(t2, self.error_time)
            if(error_delta_time > 1.0):
                print("enc error")
                print("Unexpected Error:", sys.exc_info()[0] )
        finally:
            a=0
            
        if self.bad_serial_count > 10:
            self.bad_serial_count = 0
            print("Bad Serial Count Limit 10 Reached")
            
        self.micro.flush()
            
        dtheta_micro_gyro_deg = micro_gyro_z_deg * dt * self.micro_gyro_scale_factor
        
        # Update odom
        
        delta_enc_counts = float(delta_enc_left + delta_enc_right)/2.0
        self.enc_total = self.enc_total + delta_enc_counts
        
        dmeters = float(delta_enc_left + delta_enc_right)/2.0 / COUNTS_PER_METER
        
        dtheta_enc_deg = float(delta_enc_right - delta_enc_left) / COUNTS_PER_METER / BOT_WIDTH * 180.0 / pi

        if(abs(dtheta_gyro_deg) > MAX_DTHETA_GYRO_deg):
            print('no gyro')
            dtheta_deg = dtheta_enc_deg
        else:
            #print 'use gyro'
            dtheta_deg = dtheta_gyro_deg
            
        dtheta_deg = dtheta_micro_gyro_deg #intentionally always using micro_gyro_deg vs. dtheta_gyro_deg

        #update bot position
        self.micro_bot_deg = self.micro_bot_deg + dtheta_deg # replaced with yaw_kf.py
        #self.micro_bot_deg = self.micro_bot_deg + micro_delta_yaw_deg
        avg_cos = (np.cos(self.micro_bot_deg*pi/180) + np.cos(self.bot_deg_prev*pi/180) ) / 2
        avg_sin = (np.sin(self.micro_bot_deg*pi/180) + np.sin(self.bot_deg_prev*pi/180) ) / 2
        self.bot_deg_prev = self.micro_bot_deg
        dx = dmeters*avg_cos
        dy = dmeters*avg_sin
        #dx = dmeters*np.cos(self.bot_deg*pi/180)
        #dy = dmeters*np.sin(self.bot_deg*pi/180)
        self.botx = self.botx + dx
        self.boty = self.boty + dy
        #print 'bot x,y,deg: ', self.bot.botx, self.bot.boty, self.bot.bot_deg
        
        # update bot linear x velocity every 150 msec
        # need to use collections deque, then push and pop, moving average
        self.dist_sum = self.dist_sum + dmeters
        self.time_sum = self.time_sum + dt
        if(self.time_sum > 0.15):
            self.vx = self.dist_sum / self.time_sum
            # TODO: PUT THIS IN A FUNCTION
            if abs(self.vx) < self.stopped_speed and abs(micro_gyro_z_deg) < self.stopped_gyro_deg:
                self.stopped_count += 1
                self.micro.safe_write('A3/1/')
                s = self.micro.safe_read()
                if len(s) > 0:
                    mow_area = int(s)
                    if not mow_area == self.mow_area_id:
                        self.call_set_mow(mow_area)
                else:
                    self.stopped_count = 0
            # END FUNCTION
            self.dist_sum = 0
            self.time_sum = 0
        
        odom_quat = transformations.quaternion_from_euler(0, 0, self.micro_bot_deg*pi/180.0)
        
        self.tfs.header.stamp = t2
        self.tfs.transform.translation.x = self.botx
        self.tfs.transform.translation.y = self.boty
        self.tfs.transform.rotation.x = odom_quat[0]
        self.tfs.transform.rotation.y = odom_quat[1]
        self.tfs.transform.rotation.z = odom_quat[2]
        self.tfs.transform.rotation.w = odom_quat[3]
        #self.odom_broadcaster.sendTransform(self.tfs)
        
        odom = Odometry()
        odom.header.stamp = t2
        odom.header.frame_id = "odom"

        # set the position
        #odom.pose.pose = Pose(Point(self.botx, self.boty, 0.), Quaternion(*odom_quat))
        odom.pose.pose.position.x = self.botx
        odom.pose.pose.position.y = self.boty
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation.x = self.tfs.transform.rotation.x
        odom.pose.pose.orientation.y = self.tfs.transform.rotation.y
        odom.pose.pose.orientation.z = self.tfs.transform.rotation.z
        odom.pose.pose.orientation.w = self.tfs.transform.rotation.w

        # set the velocity
        odom.child_frame_id = "base_link"
        if dt > 0:
            gz_dps = dtheta_deg / dt
        else:
            gz_dps = 0
        #odom.twist.twist = Twist(Vector3(self.vx, 0, 0), Vector3(0, 0, gz_dps*pi/180.0))
        odom.twist.twist.linear.x = self.vx
        odom.twist.twist.angular.z = gz_dps*pi/180.0

        # publish the message
        self.odom_pub.publish(odom)
        
        odom_in_msg = OdomInputs()
        odom_in_msg.header.stamp = t2
        odom_in_msg.yaw_deg = dtheta_micro_gyro_deg #self.micro_bot_deg
        odom_in_msg.enc_left = delta_enc_left
        odom_in_msg.enc_right = delta_enc_right
        self.odomIn_pub.publish(odom_in_msg)
        
        ##### USE IMU TO PUBLISH TRANSFORM BETWEEN LASER AND BASE
        accx = self.accx - 0.1 #confirm with turn-around cal on concrete using rqt_plot
        accy = self.accy + 0.13 #confirm with turn-around cal on conrete using rqt_plot
        
        if(abs(accx) < 3 and abs(accy) < 3):
            try:
                roll_rad = math.asin(accy/9.81) + 0.0 
                pitch_rad = -math.asin(accx/9.81) - 0.05 # fixed laser pitch w.r.t. imu
            except:
                roll_rad = self.roll_rad
                pitch_rad = self.pitch_rad
                print('asin error for roll or pitch')
        else:
            roll_rad = self.roll_rad
            pitch_rad = self.pitch_rad
            #print('accx,y above 3 m/s^2')
        
        if abs(roll_rad) < 0.02:
            self.roll_rad = 0.9*self.roll_rad + 0.1*roll_rad
        else:
            self.roll_rad = roll_rad
        # filter if we pitch forward, do not filter if we pitch back
        if pitch_rad - self.pitch_rad > 0.02:
            self.pitch_rad = 0.9*self.pitch_rad + 0.1*pitch_rad
        else:
            self.pitch_rad = pitch_rad
                
        laser_quat = transformations.quaternion_from_euler(self.pitch_rad, self.roll_rad, -pi/2) #- roll, -pitch b/c of 180 deg yaw
        
        self.laser_tfs.header.stamp = t2
        self.laser_tfs.transform.rotation.x = laser_quat[0]
        self.laser_tfs.transform.rotation.y = laser_quat[1]
        self.laser_tfs.transform.rotation.z = laser_quat[2]
        self.laser_tfs.transform.rotation.w = laser_quat[3]
        #self.odom_broadcaster.sendTransform(self.laser_tfs)
        #####
        
        self.prev_time = t2

def main(args=None):
    rclpy.init(args=args)
    node = MicroBridge()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
if __name__ == "__main__":
    main()
