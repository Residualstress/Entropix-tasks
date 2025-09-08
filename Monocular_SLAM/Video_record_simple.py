import logging
import sys
import time
from threading import Event
import pandas as pd
from datetime import datetime

import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.positioning.motion_commander import MotionCommander
from cflib.utils import uri_helper
from cflib.crazyflie.log import LogConfig

URI = uri_helper.uri_from_env(default='radio://0/80/2M/E7E7E7E7E7')
DEFAULT_HEIGHT = 0.5

deck_attached_event = Event()

# 只输出错误级别日志（可改成 INFO 看更详细输出）
logging.basicConfig(level=logging.ERROR)
log_data=[]

def param_deck_flow(_, value_str):
    value = int(value_str)
    print(value)
    if value:
        deck_attached_event.set()
        print('Deck is attached!')
    else:
        print('Deck is NOT attached!')


def log_callback(timestamp, data, logconf):
    row = {
        "timestamp": timestamp,
        "group": logconf.name,
    }
    row.update(data)
    log_data.append(row)


def start_logging(cf):
    logs = []

    # 一个块里把 Stabilizer 和 StateEstimate 都采集
    lg_state = LogConfig(name='State', period_in_ms=50)
    # 姿态
    lg_state.add_variable('stabilizer.roll', 'float')
    lg_state.add_variable('stabilizer.pitch', 'float')
    lg_state.add_variable('stabilizer.yaw', 'float')
    # 位置
    lg_state.add_variable('stateEstimate.x', 'float')
    lg_state.add_variable('stateEstimate.y', 'float')
    lg_state.add_variable('stateEstimate.z', 'float')

    cf.log.add_config(lg_state)
    lg_state.data_received_cb.add_callback(log_callback)
    lg_state.start()
    logs.append(lg_state)

    return logs


def stop_logging(log_confs):
    # 按启动的顺序停掉
    for lg in log_confs:
        try:
            lg.stop()
        except Exception:
            pass

def save_logs_to_excel():
    if not log_data:
        print("No log data collected")
        return
    df = pd.DataFrame(log_data)

    filename = datetime.now().strftime("crazyflie_log_%Y%m%d_%H%M%S.xlsx")
    df.to_excel(filename, index=False)
    print(f"Logs saved to {filename}")
def move_linear_simple(scf):
    with MotionCommander(scf, default_height=DEFAULT_HEIGHT) as mc:
        # ===== 关键点：飞行前启动日志 =====
        log_confs = start_logging(scf.cf)
        time.sleep(1.0)
        mc.forward(0.5)   # 向前 0.5 m
        time.sleep(1.0)
        mc.back(0.5)      # 向后 0.5 m
        time.sleep(1.0)
        stop_logging(log_confs)
        save_logs_to_excel()

if __name__ == '__main__':
    # 初始化底层驱动
    cflib.crtp.init_drivers()

    with SyncCrazyflie(URI, cf=Crazyflie(rw_cache='./cache')) as scf:
        # 监听 Flow deck
        scf.cf.param.add_update_callback(group='deck', name='bcFlow2', cb=param_deck_flow)
        time.sleep(1.0)

        if not deck_attached_event.wait(timeout=5):
            print('No flow deck detected!')
            sys.exit(1)

        # 起桨（如为刷驱版需要；若报错可去掉）
        scf.cf.platform.send_arming_request(True)
        time.sleep(1.0)


        move_linear_simple(scf)
