#!/usr/bin/env python
import rospy
import numpy as np
import math
from typing import *
# from beacon_simple_nav.utils.Test import HelloWorld
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Pose, PoseStamped, Twist, TwistStamped
from nav_msgs.msg import Odometry
from dataclasses import dataclass, field, fields
from beacon_gazebo_sim.msg import ReceiverIn

import matplotlib
matplotlib.use('TkAgg')
import matplotlib.pyplot as plt

rospy.init_node("simple_nav_node")
rospy.sleep(0.1)
robot_name = rospy.get_namespace().replace("/", "")
print("robot_name: ", robot_name)

def rssi_to_distance(rssi: float, m_power: float, a: float = 10, b: float = 1, c: float = 0) -> float:  # rssi, m_power -> distance
    return b * ((rssi / m_power) ** a) + c

def solve_position_2d(b_positions: np.ndarray, b_distances: np.ndarray) -> np.ndarray:
    """
    simple 2d pose estimation
    :param b_positions: np.ndarray - 2d pose
    :param b_distances: np.ndarray - rssi vals
    :return: np.ndarray - 2d pose
    """
    b_p = b_positions[1:, :]
    b_p_1 = b_positions[0]
    # print(b_p_1, b_p)
    A = (b_p_1 - b_p)
    # print("A", A)
    # print("b_p**2", np.expand_dims(np.sum(-b_p**2, axis=1), axis=1), np.sum(b_p_1**2), b_distances[0]**2, b_distances[1:]**2, np.expand_dims(b_distances[1:]**2, axis=1))
    b = np.expand_dims(np.sum(-b_p**2, axis=1), axis=1) + np.sum(b_p_1**2) - b_distances[0]**2 + np.expand_dims(b_distances[1:]**2, axis=1)
    b = b*0.5
    # print("b", b)
    # print("------", A.T @ A)
    m = np.linalg.inv(A.T @ A) @ (A.T @ b)
    # print(m)
    return np.reshape(m, (2))


class TData:
    def __str__(self) -> str:
        return str("IMUData: " + "\n\t".join([""] + [str(f"{_f.name}: {getattr(self, _f.name)}") for _f in fields(self)]))


@dataclass
class IMUData(TData):
    time_stamp: float = -1

    ax: float = 0
    ay: float = 0
    az: float = 0

    wx: float = 0
    wy: float = 0
    wz: float = 0

    def to_np_array_a(self) -> np.ndarray:
        return np.array([getattr(self, k) for k in ["ax", "ay", "az"]])

    def to_np_array_w(self) -> np.ndarray:
        return np.array([getattr(self, k) for k in ["wx", "wy", "wz"]])


@dataclass
class CMDData(TData):
    time_stamp: float = -1

    vx: float = 0
    wz: float = 0

    def to_np_array(self) -> np.ndarray:
        return np.array([getattr(self, k) for k in ["vx", "wz"]])


@dataclass
class DistsData(TData):
    time_stamp: float = -1

    beacon_names: List[str] = field(default_factory=list)
    beacon_distances: Dict[str, float] = field(default_factory=dict)

    beacon_rssi: Dict[str, float] = field(default_factory=dict)
    beacon_ts: Dict[str, float] = field(default_factory=dict)
    beacon_m_rssi: Dict[str, float] = field(default_factory=dict)

    def to_np_array(self) -> np.ndarray:
        return np.array([el for k, el in self.beacon_distances.items()])


imu = IMUData()
cmd = CMDData()
dists = DistsData()


def imu_clb(msg: Imu):
    global imu
    imu = IMUData(ax=msg.linear_acceleration.x, ay=msg.linear_acceleration.y, az=msg.linear_acceleration.z,
                  wx=msg.angular_velocity.x, wy=msg.angular_velocity.y, wz=msg.angular_velocity.z,
                  time_stamp=msg.header.stamp.to_sec())


def cmd_vel_clb(msg: Twist):
    global cmd
    cmd = CMDData(vx=msg.linear.x, wz=msg.angular.z,
                  time_stamp=rospy.Time.now().to_sec())


def receiver_clb(msg: ReceiverIn):
    global dists
    # print("asdasdasdad: ")
    if "beacon__beacon" in msg.id:
        # _f_d = lambda _d: {_k:_el for _k, _el in _d.items() if _k in dists.beacon_names}
        dists.beacon_rssi[msg.id] = msg.rssi; dists.beacon_ts[msg.id] = msg.time_stamp.to_sec(); dists.beacon_m_rssi[msg.id] = msg.m_rssi
        beacons_ids = sorted(list(dists.beacon_rssi.keys()))
        # dists.beacon_names = beacons_ids
        # print("beacons_ids: ", beacons_ids)
        dists.beacon_names = beacons_ids
        # if (lambda ids, ts: (len(ids) != 0) and (min(ts, key=lambda x: x[1]) == max(ts, key=lambda x: x[1]))) \
        #             (beacons_ids, [el for k, el in dists.beacon_ts.items() if k in beacons_ids]):
        dists.beacon_distances = {_k: rssi_to_distance(_r, _mr) for _k, _r, _mr in
                                  (lambda _d1, _d2, _ks: map(lambda _k: (_k, _d1[_k], _d2[_k]), _ks))
                                  (dists.beacon_rssi, dists.beacon_m_rssi, beacons_ids)}
        dists.time_stamp = rospy.Time.now().to_sec()

sub_imu = rospy.Subscriber("imu", Imu, imu_clb)
sub_cmd_vel = rospy.Subscriber("cmd_vel", Twist, cmd_vel_clb)
sub_receiver_msgs = rospy.Subscriber(f"receiver__{robot_name}/receiver_in_msgs", ReceiverIn, receiver_clb)

HZ = 15

DT = 1/HZ

r = rospy.Rate(HZ)

good_beacon = ["beacon__beacon_1", "beacon__beacon_2", "beacon__beacon_3"]
good_beacon_pose = np.array([[1, 1], [0, 4], [-1, -1]])

# https://github.com/AtsushiSakai/PythonRobotics/blob/master/Localization/extended_kalman_filter/extended_kalman_filter.py
Q = np.diag([
    0.1,  # variance of location on x-axis
    0.1,  # variance of location on y-axis
    np.deg2rad(2.0),  # variance of yaw angle
    1.0  # variance of velocity
]) ** 2  # predict state covariance
R = np.diag([2, 2, 0.1]) ** 2  # Observation x,y position covariance

# ws = 0.32

def motion_model(x, u):
    F = np.array([[1.0, 0, 0, 0],
                  [0, 1.0, 0, 0],
                  [0, 0, 1.0, 0],
                  [0, 0, 0, 0]])

    B = np.array([[DT * math.sin(x[2, 0]), 0],
                  [DT * math.cos(x[2, 0]), 0],
                  [0.0, DT],
                  [1.0, 0.0]])

    x = F @ x + B @ u

    return x


def observation_model(x):
    H = np.array([
        [1, 0, 0, 0],
        [0, 1, 0, 0],
        [0, 0, 1, 0]
    ])

    z = H @ x

    return z


def jacob_f(x, u):
    """
    Jacobian of Motion Model

    motion model
    x_{t+1} = x_t+v*dt*cos(yaw)
    y_{t+1} = y_t+v*dt*sin(yaw)
    yaw_{t+1} = yaw_t+omega*dt
    v_{t+1} = v{t}
    so
    dx/dyaw = -v*dt*sin(yaw)
    dx/dv = dt*cos(yaw)
    dy/dyaw = v*dt*cos(yaw)
    dy/dv = dt*sin(yaw)
    """
    yaw = x[2, 0]
    v = u[0, 0]
    jF = np.array([
        [1.0, 0.0, -DT * v * math.cos(yaw), DT * math.sin(yaw)],
        [0.0, 1.0, DT * v * math.sin(yaw), DT * math.cos(yaw)],
        [0.0, 0.0, 1.0, 0.0],
        [0.0, 0.0, 0.0, 1.0]])

    return jF


def jacob_h():
    # Jacobian of Observation Model
    jH = np.array([
        [1, 0, 0, 0],
        [0, 1, 0, 0],
        [0, 0, 1, 0]
    ])

    return jH


def ekf_estimation(xEst, PEst, z, u):
    #  Predict
    xPred = motion_model(xEst, u)
    jF = jacob_f(xEst, u)
    PPred = jF @ PEst @ jF.T + Q

    #  Update
    jH = jacob_h()
    zPred = observation_model(xPred)
    y = z - zPred
    S = jH @ PPred @ jH.T + R
    K = PPred @ jH.T @ np.linalg.inv(S)
    xEst = xPred + K @ y
    PEst = (np.eye(len(xEst)) - K @ jH) @ PPred
    return xEst, PEst

xEst = np.zeros((4, 1))
xDR = np.zeros((4, 1))
PEst = np.eye(4)

hxEst = np.zeros((4, 1))
hz = np.zeros((3, 1))
hxDR = np.zeros((4, 1))

show_animation = True
ys = 0

while not rospy.is_shutdown():
    t = rospy.Time.now().to_sec()
    # print(f"t: {t}\n  imu: {imu}\n  cmd: {cmd}\n  dists: {dists}")

    if set(dists.beacon_names) == set(good_beacon) and imu.time_stamp != -1:
        d_ = dists.to_np_array()
        imu_a_ = imu.to_np_array_a()
        imu_w_ = imu.to_np_array_w()
        cmd_ = cmd.to_np_array()
        print(f"d_: {d_}\nimu_a_: {imu_a_}\nimu_w_: {imu_w_}\ncmd_: {cmd_}")

        # Simple solve
        b_xy_ = solve_position_2d(good_beacon_pose, d_)
        print(f"b_xy_: {b_xy_}")

        # extended kalman filter with b_xy_ and cmd_
        z = np.zeros((3, 1))
        z[0:2, 0] = b_xy_
        ys += -imu_w_[2] * DT
        z[2, 0] = ys

        ud = cmd_.reshape(2, 1)

        xEst, PEst = ekf_estimation(xEst, PEst, z, ud)

        print(f"xEst: {xEst}")

        hxEst = np.hstack((hxEst, xEst))
        hz = np.hstack((hz, z))

        xDR = motion_model(xDR, ud)
        hxDR = np.hstack((hxDR, xDR))

        if show_animation:
            plt.cla()
            # for stopping simulation with the esc key.
            plt.gcf().canvas.mpl_connect('key_release_event',
                                         lambda event: [exit(0) if event.key == 'escape' else None])
            plt.plot(-hz[1, :], hz[0, :], ".g")
            plt.plot(-hxEst[1, :].flatten(),
                     hxEst[0, :].flatten(), "-r")
            plt.plot(hxDR[0, :].flatten(),
                     hxDR[1, :].flatten(), "-k")
            # plot_covariance_ellipse(xEst, PEst)
            plt.axis("equal")
            plt.grid(True)
            plt.pause(0.001)


    r.sleep()





