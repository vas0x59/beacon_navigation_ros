#!/usr/bin/env python
import rospy
import numpy as np
from typing import *
# from beacon_simple_nav.utils.Test import HelloWorld
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Pose, PoseStamped, Twist, TwistStamped
from nav_msgs.msg import Odometry

from beacon_gazebo_sim.msg import ReceiverIn

rospy.init_node("simple_nav_node")
rospy.sleep(0.1)

class IMUData(NamedTuple):
    ax: float = 0
    ay: float = 0
    az: float = 0

    wx: float = 0
    wy: float = 0
    wz: float = 0


class CMDData(NamedTuple):
    vx: float = 0
    wz: float = 0


imu = IMUData()
cmd = CMDData()

def imu_clb(msg: Imu):
    global imu
    imu = IMUData(ax=msg.linear_acceleration.x, ay=msg.linear_acceleration.y, az=msg.linear_acceleration.z,
                  wx=msg.angular_velocity.x, wy=msg.angular_velocity.y, wz=msg.angular_velocity.z)

def cmd_vel_clb(msg: Twist):
    global cmd
    cmd = CMDData(vx=msg.linear.x, wz=msg.angular.z)

def receiver_clb(msg: ReceiverIn):
    pass

robot_name = rospy.get_namespace().replace("/", "")
print("robot_name: ", robot_name)

sub_imu = rospy.Subscriber("imu", Imu, imu_clb)
sub_cmd_vel = rospy.Subscriber("cmd_vel", Twist, cmd_vel_clb)
sub_receiver_msgs = rospy.Subscriber(f"receiver__{robot_name}/receiver_in_msgs", ReceiverIn, receiver_clb)

r = rospy.Rate(10)

while not rospy.is_shutdown():
    t = rospy.Time.now().to_sec()
    print(f"t: {t}  imu: {imu}  cmd: {cmd}")
    r.sleep()





