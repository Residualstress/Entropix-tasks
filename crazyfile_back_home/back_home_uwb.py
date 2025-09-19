import logging
import sys
import time
from threading import Event
import warnings


import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.log import LogConfig
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.positioning.motion_commander import MotionCommander
from cflib.utils import uri_helper

from uwb_localization.uwb_receiver import UWBReceiver
from uwb_localization.ukf_filter import PositionUKF
from pid_controller.PIDController import PIDController2D

URI = uri_helper.uri_from_env(default='radio://0/80/2M/E7E7E7E7E7')

DEFAULT_HEIGHT = 0.5
BOX_LIMIT = 0.5

deck_attached_event = Event()

logging.basicConfig(level=logging.ERROR)

position_estimate = [0, 0]

def back_home(scf, uwb: UWBReceiver, pid_controller: PIDController2D, home_point):
    with MotionCommander(scf, default_height=DEFAULT_HEIGHT) as mc:
        pid_controller.set_target(home_point)
        while True:
            while uwb.has_data():
                _,_,x, y = uwb.pop_data()

            vel_x, vel_y = pid_controller.compute_velocity(x, y)

            print(f'x: {x}, y: {y}, vel_x: {vel_x}, vel_y: {vel_y}')

            mc.start_linear_motion(vel_x, vel_y, 0)

            time.sleep(0.05)



def move_linear_simple(scf):
    ...

def take_off_simple(scf):
    ...

def log_pos_callback(timestamp, data, logconf):
    # print(data)
    global position_estimate
    position_estimate[0] = data['stateEstimate.x']
    position_estimate[1] = data['stateEstimate.y']


def param_deck_flow(_, value_str):
    value = int(value_str)
    print(value)
    if value:
        deck_attached_event.set()
        print('Deck is attached!')
    else:
        print('Deck is NOT attached!')


if __name__ == '__main__':
    warnings.filterwarnings("ignore", category=DeprecationWarning)

    pid_controller = PIDController2D(target_point=(0.5, -1.0), kp=1.0, ki=0.3, kd=0.0, output_limit=0.15)
    ukf_filter = PositionUKF(dt=0.02, win_size=6) # 50Hz data rate
    uwb = UWBReceiver("COM4", 115200, ukf_filter)
    uwb.start()

    cflib.crtp.init_drivers()

    with SyncCrazyflie(URI, cf=Crazyflie(rw_cache='./cache')) as scf:

        scf.cf.param.add_update_callback(group='deck', name='bcFlow2',
                                         cb=param_deck_flow)
        time.sleep(1)

        logconf = LogConfig(name='Position', period_in_ms=10)
        logconf.add_variable('stateEstimate.x', 'float')
        logconf.add_variable('stateEstimate.y', 'float')
        scf.cf.log.add_config(logconf)
        logconf.data_received_cb.add_callback(log_pos_callback)


        if not deck_attached_event.wait(timeout=5):
            print('No flow deck detected!')
            sys.exit(1)

        # Arm the Crazyflie
        scf.cf.platform.send_arming_request(True)
        time.sleep(1.0)

        logconf.start()
        # move_box_limit(scf)
        back_home(scf, uwb, pid_controller, (0.5, -1.0))
        logconf.stop()