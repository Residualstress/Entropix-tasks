import logging
import sys
import time
from threading import Event
import threading
import traceback
import requests

import numpy as np
import math

import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.log import LogConfig
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.positioning.motion_commander import MotionCommander
from cflib.utils import uri_helper

from pid_controller.PIDController import PIDController2D, PIDController1D


URI = uri_helper.uri_from_env(default='radio://0/80/2M/E7E7E7E7E7')

deck_attached_event = Event()

logging.basicConfig(level=logging.ERROR)

position_estimate = [0,0,0]
yaw_estimate = None
multirange_estimate = None

on_logging = True

DEFAULT_HEIGHT = 1.8
WALL_MARGIN = 1.0

def wall_following(scf):
    pid_wall = PIDController1D(target_point=WALL_MARGIN, kp=0.8, ki=0.2, kd=0.0, output_limit=0.1)

    with MotionCommander(scf, default_height=DEFAULT_HEIGHT) as mc:
        print('start forwarding', flush=True)
        while multirange_estimate['front'] > WALL_MARGIN:
            # print(f'front: {multirange_estimate["front"]}', flush=True)
            mc.start_linear_motion(0.8, 0.0, 0.0)
            time.sleep(0.1)
        mc.start_linear_motion(0.0, 0.0, 0.0)
        mc.turn_left(90,rate=36)

        for i in range(6):
            print(f'start following wall: {i+1}', flush=True)
            while multirange_estimate['front'] > WALL_MARGIN:
                # print(f'front: {multirange_estimate["front"]}, right: {multirange_estimate["right"]}', flush=True)
                vy = pid_wall.compute_velocity(multirange_estimate['right'])
                mc.start_linear_motion(0.8, vy, 0.0)
                time.sleep(0.1)
            mc.start_linear_motion(0.0, 0.0, 0.0)
            mc.turn_left(90,rate=36)


def log_position_file(filename):
    with open(filename, 'w') as f:
        while on_logging:
            f.write(f'{position_estimate[0]},{position_estimate[1]}\n')
            time.sleep(0.1)

def log_pos_callback(timestamp, data, logconf):
    global position_estimate, yaw_estimate, multirange_estimate
    position_estimate[0] = data['stateEstimate.x']
    position_estimate[1] = data['stateEstimate.y']
    position_estimate[2] = data['stateEstimate.z']
    yaw_estimate = data['stateEstimate.yaw']
    multirange_estimate = {'front': data['range.front']/1000.0, 
                           'back': data['range.back']/1000.0, 
                           'left': data['range.left']/1000.0, 
                           'right': data['range.right']/1000.0}

def param_deck_flow(_, value_str):
    value = int(value_str)
    print(value)
    if value:
        deck_attached_event.set()
        print('Deck is attached!')
    else:
        print('Deck is NOT attached!')

if __name__ == '__main__':

    cflib.crtp.init_drivers()

    with SyncCrazyflie(URI, cf=Crazyflie(rw_cache='./cache')) as scf:

        scf.cf.param.add_update_callback(group='deck', name='bcFlow2',
                                         cb=param_deck_flow)
        time.sleep(1)

        logconf = LogConfig(name='Position', period_in_ms=10)
        logconf.add_variable('stateEstimate.x', 'float')
        logconf.add_variable('stateEstimate.y', 'float')
        logconf.add_variable('stateEstimate.z', 'float')
        logconf.add_variable('stateEstimate.yaw', 'float')
        logconf.add_variable('range.front', 'uint16_t')
        logconf.add_variable('range.back', 'uint16_t')
        logconf.add_variable('range.left', 'uint16_t')
        logconf.add_variable('range.right', 'uint16_t')

        scf.cf.log.add_config(logconf)
        logconf.data_received_cb.add_callback(log_pos_callback)


        if not deck_attached_event.wait(timeout=5):
            print('No flow deck detected!')
            sys.exit(1)

        # Arm the Crazyflie
        scf.cf.platform.send_arming_request(True)
        time.sleep(2.0)

        logconf.start()
        file_logging_thread = threading.Thread(target=log_position_file, args=('position_log.csv',))
        file_logging_thread.start()
        wall_following(scf)
        on_logging = False
        file_logging_thread.join()
        logconf.stop()