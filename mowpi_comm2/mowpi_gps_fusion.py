#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

import math
from math import pi, cos, sin, atan2
from geometry_msgs.msg import Twist, Quaternion, Point, Pose, Vector3, Vector3Stamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Int16, Float32
from sensor_msgs.msg import Imu

import zumo_serial_comm.transformations as transformations# for euler to quat
from tf2_ros.transform_broadcaster import TransformBroadcaster
from geometry_msgs.msg import TransformStamped

from my_interfaces.msg import OdomInputs
from ublox_msgs.msg import NavRELPOSNED
from sensor_msgs.msg import NavSatFix

import serial
import sys
import numpy as np
import time

class GPSFusion(Node):
    def __init__(self):
        super().__init__("gps_fusion")
        
        self.useRelPose = False
        self.useGPSx = True
        self.useGPSy = True
        self.declare_parameter("lat0", 0.0)
        self.declare_parameter("lon0", 0.0)
        self.declare_parameter("dNcm_dlat", 1.0)
        self.declare_parameter("dEcm_dlon", 1.0)
        
        self.lat0 = self.get_parameter("lat0").value
        self.lon0 = self.get_parameter("lon0").value
        self.dNcm_dlat = self.get_parameter("dNcm_dlat").value
        self.dEcm_dlon = self.get_parameter("dEcm_dlon").value
        
        print("lat0 %.7f" % (self.lat0))
        print("lon0 %.7f" % (self.lon0))
        print("dNcm_dlat %.7f" % (self.dNcm_dlat))
        print("dEcm_dlon %.7f" % (self.dEcm_dlon))
        
        self.imu_sub = self.create_subscription(Imu, "imu", self.imu_callback, 2)
        self.odomIn_sub = self.create_subscription(OdomInputs, "odomInputs", self.odom_callback, 5)
        if self.useRelPose:
            self.gps_sub = self.create_subscription(NavRELPOSNED, "navrelposned", self.gps_callback, 2)
        else:
            self.gps_sub = self.create_subscription(NavSatFix, "fix", self.gps_callback, 2)
        
        self.odom_pub = self.create_publisher(Odometry, "odom", 5)
        
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
        
        self.gps_tfs = TransformStamped()
        self.gps_tfs.header.frame_id = "odom"
        self.gps_tfs.child_frame_id = "gps"
        self.east_ant = 0.
        self.north_ant = 0.
        
        now_stamp = self.get_clock().now().to_msg()
        self.prev_time = now_stamp
        self.error_time = now_stamp
        
        self.accx = 0.0
        self.accy = 0.0
        self.gyroz_rad = 0.0

        self.enc_total = 0
        self.roll_rad = 0.
        self.pitch_rad = 0.
        
        self.dist_sum = 0.
        self.time_sum = 0.
        self.vx = 0.

        self.bot_deg_prev = 0.
        self.micro_bot_deg = 0.
        self.botx = 0.
        self.boty = 0.
        
        self.gyro_sum = 0.0
        self.gyro_count = 0
        
        # NXP
        self.gyro_bias_rad = 0.06*pi/180
        self.neg_gyro_scale_factor = 1.005
        self.pos_gyro_scale_factor = 1.001
        
        # Odom inputs
        self.enc_left = 0.
        self.enc_right = 0.
        self.dtheta_odomInputs = 0.
        
        # GPS odom fusion
        self.gps_initialized = False
        self.ref_cum_dmeters = 0.0
        self.prev_cum_dmeters = 0.0
        self.ref_cum_dtheta_deg = 0.0
        self.init_gps_east_sum = 0.0
        self.init_gps_north_sum = 0.0
        self.init_gps_count = 0
        self.gps_buffer = []
        self.buffer_size = 5
        self.buffer_full = False
        
        #self.ant_local_x = 0.6 # antenna coordinates in base_link frame (from center of rotation)
        self.ant_local_x = 0.0 #0.6 - 5*0.025
        self.ant_local_y = 0.0 #-7*0.025
        
        self.timer = self.create_timer(1./20., self.update_odom)
        
        self.get_logger().info("Started GPS Fusion")
        
        time.sleep(0.1)
    
    def dt_to_sec(self, stampA, stampB):
        return stampA.sec + stampA.nanosec * 10**-9 - stampB.sec - stampB.nanosec * 10**-9
        
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
                
    def odom_callback(self, data):
        self.enc_left += data.enc_left
        self.enc_right += data.enc_right
        self.dtheta_odomInputs += data.yaw_deg
        
    def gps_callback(self, msg):
        #try:
        if self.useRelPose:
            relE = msg.rel_pos_e/100.
            relN = msg.rel_pos_n/100.
        else:
            relE = (msg.longitude - self.lon0) * self.dEcm_dlon / 100.
            relN = (msg.latitude - self.lat0) * self.dNcm_dlat / 100.
        yaw_rad = self.micro_bot_deg*pi/180
        rel_east = relE - self.ant_local_x*cos(yaw_rad) + self.ant_local_y*sin(yaw_rad)
        rel_north = relN - self.ant_local_x*sin(yaw_rad) - self.ant_local_y*cos(yaw_rad)
        self.east_ant = relE
        self.north_ant = relN
        if not self.gps_initialized:
            self.init_gps_east_sum += rel_east
            self.init_gps_north_sum += rel_north
            self.init_gps_count += 1
            if self.init_gps_count >= 10:
                self.botx = self.init_gps_east_sum / self.init_gps_count
                self.boty = self.init_gps_north_sum / self.init_gps_count
                self.gps_initialized = True
        else:
            alpha = 0.97
            if abs(self.dtheta_odomInputs) > 1.5:
                alpha = 0.05
            if self.useGPSx:
                self.botx = self.botx*alpha + rel_east*(1.0-alpha)
            if self.useGPSy:
                self.boty = self.boty*alpha + rel_north*(1.0-alpha)
            if len(self.gps_buffer) == self.buffer_size:
                self.gps_buffer.pop(0)
                self.buffer_full = True
            if self.ref_cum_dmeters - self.prev_cum_dmeters > 0.1:
                self.gps_buffer.append([self.east_ant, self.north_ant])
            #print("cum_dmeters %.1f, cum_dtheta_deg %.1f" % (self.ref_cum_dmeters, self.ref_cum_dtheta_deg) )
            if self.buffer_full and self.ref_cum_dmeters > 0.7 and abs(self.ref_cum_dtheta_deg) < 20.0:
                self.ref_cum_dmeters = 0.0
                self.ref_cum_dtheta_deg = 0.0
                dE = self.gps_buffer[-1][0] - self.gps_buffer[0][0]
                dN = self.gps_buffer[-1][1] - self.gps_buffer[0][1]
                gps_heading_deg = atan2(dN,dE)*180.0/pi
                print("gps_heading_deg %.1f, micro_bot_deg %.1f" % (gps_heading_deg, self.micro_bot_deg) )
                #if True or (abs((gps_heading_deg - self.micro_bot_deg) % 360.0) < 45.0):
                cur_cos = cos(yaw_rad)
                cur_sin = sin(yaw_rad)
                gps_cos = cos(gps_heading_deg*pi/180)
                gps_sin = sin(gps_heading_deg*pi/180)
                net_cos = cur_cos*0.9 + gps_cos*0.1
                net_sin = cur_sin*0.9 + gps_sin*0.1
                self.micro_bot_deg = atan2(net_sin, net_cos)*180/pi
            elif self.ref_cum_dmeters > 1.0:
                self.ref_cum_dmeters = 0.0
                self.ref_cum_dtheta_deg = 0.0
        #except:
        #    print("gps callback error")
                
    
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
            
        # This would come from diff of yaw_deg in odomInputs
        #dtheta_micro_gyro_deg = micro_gyro_z_deg * dt * self.micro_gyro_scale_factor
        
        # Update odom
        delta_enc_left = self.enc_left
        delta_enc_right = self.enc_right
        self.enc_left = 0
        self.enc_right = 0
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
            
        #dtheta_deg = dtheta_micro_gyro_deg #intentionally always using micro_gyro_deg vs. dtheta_gyro_deg
        dtheta_deg = self.dtheta_odomInputs
        self.dtheta_odomInputs = 0.
        #if(delta_enc_left == 0 and delta_enc_right == 0):
        #    dtheta_deg = 0.
        
        self.ref_cum_dmeters += dmeters # reset in gps callback
        self.ref_cum_dtheta_deg += dtheta_deg # reset in gps callback

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
        self.odom_broadcaster.sendTransform(self.tfs)
        
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
        self.odom_broadcaster.sendTransform(self.laser_tfs)
        
        self.gps_tfs.header.stamp = t2
        self.gps_tfs.transform.translation.x = self.east_ant
        self.gps_tfs.transform.translation.y = self.north_ant
        self.gps_tfs.transform.rotation.x = odom_quat[0]
        self.gps_tfs.transform.rotation.y = odom_quat[1]
        self.gps_tfs.transform.rotation.z = odom_quat[2]
        self.gps_tfs.transform.rotation.w = odom_quat[3]
        self.odom_broadcaster.sendTransform(self.gps_tfs)
        #####
        
        self.prev_time = t2

def main(args=None):
    rclpy.init(args=args)
    node = GPSFusion()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
if __name__ == "__main__":
    main()
