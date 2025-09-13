import logging
import sys
import time
from threading import Event

import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.log import LogConfig
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.positioning.motion_commander import MotionCommander
from cflib.utils import uri_helper

import numpy as np

from pid_controller.PIDController import PIDController2D, PIDController3D
from apriltag_beacon.april_test import HttpAprilResolver

URI = uri_helper.uri_from_env(default='radio://0/80/2M/E7E7E7E7E7')

DEFAULT_HEIGHT = 0.5

deck_attached_event = Event()

logging.basicConfig(level=logging.ERROR)

position_estimate = [0, 0, 0]

rel_beacon_postion = None

state = 'ALIGNING'

def aligning(mc: MotionCommander, pid: PIDController2D):

    if rel_beacon_postion is None:
        return
    
    vel_x, vel_y = pid.compute_velocity(rel_beacon_postion[0], rel_beacon_postion[1])

    print(f' m: {rel_beacon_postion[0]}, y: {rel_beacon_postion[1]}, vel_x: {vel_x}, vel_y: {vel_y}')

    mc.start_linear_motion(vel_x, vel_y, 0)
    
    print(pid.get_stability())
    if pid.get_stability() > 0.3:
        
        global state
        state = 'DESCENDING'

def descending(mc: MotionCommander, pid: PIDController3D):

    if rel_beacon_postion is None:
        return

    x,y,z = rel_beacon_postion[0], rel_beacon_postion[1], position_estimate[2]

    vel_x, vel_y, vel_z = pid.compute_velocity(x,y,z)

    print(f'x: {x}, y: {y}, z: {z}, vel_x: {vel_x}, vel_y: {vel_y}, vel_z: {vel_z}')

    mc.start_linear_motion(vel_x, vel_y, vel_z)

    print(pid.get_stability())
    if pid.get_stability() > 0.3:
        
        global state
        state = 'LANDING'

def beacon_landing(scf):
    pid_aligning = PIDController2D(target_point=(0.0,0.0), kp=1.0, ki=0.3, kd=0.0, output_limit=0.2)
    pid_descending = PIDController3D(target_point=(0.0,0.0,0.25), kp=0.5, ki=0.2, kd=0.0, output_limit=0.2)
    with MotionCommander(scf, default_height=DEFAULT_HEIGHT) as mc:
        time.sleep(5.0)
        while True:
            if state == 'ALIGNING':
                aligning(mc, pid_aligning)
            elif state == "DESCENDING":
                descending(mc, pid_descending)
            elif state == 'LANDING':
                mc.land()

            time.sleep(0.05)

def log_pos_callback(timestamp, data, logconf):
    # print(data)
    global position_estimate
    position_estimate[0] = data['stateEstimate.x']
    position_estimate[1] = data['stateEstimate.y']
    position_estimate[2] = data['stateEstimate.z']

def beacon_resolver_callback(center):
    target_pixel_position = center
    center_pixel_position = (160,120)
    rel_pixel_position = np.array(target_pixel_position) - np.array(center_pixel_position)
    # resolution = [320 240]
    # cx = 146.35090
    # cy = 125.99543
    # fx = 316.92703
    # fy = 326.38666
    alpha = 320 # 相机标定的结果
    z = position_estimate[2]
    rel_position = tuple(rel_pixel_position / alpha * z)
    rel_position[1] = -rel_position[1] # y轴翻转
    global rel_beacon_postion
    rel_beacon_postion = rel_position

def param_deck_flow(_, value_str):
    value = int(value_str)
    print(value)
    if value:
        deck_attached_event.set()
        print('Deck is attached!')
    else:
        print('Deck is NOT attached!')


if __name__ == '__main__':
    ip = "10.201.171.40"
    april_beacon = HttpAprilResolver(ip, callback=beacon_resolver_callback)

    cflib.crtp.init_drivers()

    with SyncCrazyflie(URI, cf=Crazyflie(rw_cache='./cache')) as scf:

        scf.cf.param.add_update_callback(group='deck', name='bcFlow2',
                                         cb=param_deck_flow)
        time.sleep(1)

        logconf = LogConfig(name='Position', period_in_ms=10)
        logconf.add_variable('stateEstimate.x', 'float')
        logconf.add_variable('stateEstimate.y', 'float')
        logconf.add_variable('stateEstimate.z', 'float')
        scf.cf.log.add_config(logconf)
        logconf.data_received_cb.add_callback(log_pos_callback)


        if not deck_attached_event.wait(timeout=5):
            print('No flow deck detected!')
            sys.exit(1)

        # Arm the Crazyflie
        scf.cf.platform.send_arming_request(True)
        time.sleep(1.0)

        logconf.start()

        beacon_landing(scf)

        logconf.stop()