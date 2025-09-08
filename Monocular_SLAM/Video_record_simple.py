import logging
import sys
import time
from threading import Event

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


def param_deck_flow(_, value_str):
    value = int(value_str)
    print(value)
    if value:
        deck_attached_event.set()
        print('Deck is attached!')
    else:
        print('Deck is NOT attached!')


def log_callback(timestamp, data, logconf):
    print(f'[{timestamp}][{logconf.name}]: {data}')


def start_logging(cf):
    """
    配置并启动多个日志组。返回它们以便后续停止。
    你可以按需增删变量；周期单位 ms。
    """
    logs = []

    # 姿态
    lg_stab = LogConfig(name='Stabilizer', period_in_ms=10)
    lg_stab.add_variable('stabilizer.roll', 'float')
    lg_stab.add_variable('stabilizer.pitch', 'float')
    lg_stab.add_variable('stabilizer.yaw', 'float')
    cf.log.add_config(lg_stab)
    lg_stab.data_received_cb.add_callback(log_callback)
    lg_stab.start()
    logs.append(lg_stab)

    # 位置估计（如果启用 stateEstimate）
    lg_pos = LogConfig(name='StateEst', period_in_ms=50)
    lg_pos.add_variable('stateEstimate.x', 'float')
    lg_pos.add_variable('stateEstimate.y', 'float')
    lg_pos.add_variable('stateEstimate.z', 'float')
    cf.log.add_config(lg_pos)
    lg_pos.data_received_cb.add_callback(log_callback)
    lg_pos.start()
    logs.append(lg_pos)
    return logs


def stop_logging(log_confs):
    # 按启动的顺序停掉
    for lg in log_confs:
        try:
            lg.stop()
        except Exception:
            pass


def move_linear_simple(scf):
    with MotionCommander(scf, default_height=DEFAULT_HEIGHT) as mc:
        time.sleep(1.0)
        mc.forward(0.5)   # 向前 0.5 m
        time.sleep(1.0)
        mc.back(0.5)      # 向后 0.5 m
        time.sleep(1.0)


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

        # ===== 关键点：飞行前启动日志 =====
        log_confs = start_logging(scf.cf)

        try:
            move_linear_simple(scf)
        finally:
            # 飞行后停止日志
            stop_logging(log_confs)
            # 如需落地后的额外等待以收尾日志，可再 sleep
            # time.sleep(0.2)
