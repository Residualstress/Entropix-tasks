import logging
import time
from pathlib import Path

import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.positioning.motion_commander import MotionCommander as _LibMotionCommander

# ================== 配置 ==================
URI = 'radio://0/80/2M/E7E7E7E7E7'
PARAM_FILE = Path(__file__).with_name("parameters.txt")
POLL_SEC = 0.1          # 轮询参数文件频率
TAKEOFF_HEIGHT = 0.5    # 起飞高度（米）
# =========================================

def read_flag(path: Path) -> int:
    """读取 parameters.txt：以'1'开头算 1，其它算 0；异常→0"""
    try:
        with path.open("r", encoding="utf-8", errors="ignore") as f:
            s = f.read(8).strip()
        return 1 if s.startswith("1") else 0
    except FileNotFoundError:
        return 0
    except Exception as e:
        logging.debug("read_flag error: %r", e)
        return 0

# —— 关键：禁用“with 进入即起飞”的封装 —— #
class MotionCommander(_LibMotionCommander):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self._flying = False

    def __enter__(self):
        # 不自动起飞，按你的指令起飞
        return self

    # 记住飞行状态，方便 __exit__ 收尾
    def take_off(self, *args, **kwargs):
        super().take_off(*args, **kwargs)
        self._flying = True

    def land(self, *args, **kwargs):
        super().land(*args, **kwargs)
        self._flying = False

    def __exit__(self, exc_type, exc, tb):
        # 退出时若还在飞，先落地，再停
        try:
            if self._flying:
                try:
                    super().land()
                except Exception:
                    pass
        finally:
            try:
                super().stop()
            except Exception:
                pass
        return False

if __name__ == '__main__':
    logging.basicConfig(level=logging.INFO, format='[%(asctime)s] %(message)s')
    cflib.crtp.init_drivers(enable_debug_driver=False)

    print(f"已启动：监听 {PARAM_FILE.name}（1=起飞，0=降落）。"
          f"启动时即使是1也不会起飞，必须经历 0→1 才会起飞。")

    while True:
        try:
            with SyncCrazyflie(URI, cf=Crazyflie(rw_cache='./cache')) as scf:
                # 使用“禁自动起飞”的 MotionCommander
                with MotionCommander(scf) as mc:
                    last_flag = read_flag(PARAM_FILE)  # 启动时取一次，作为“上一状态”
                    is_flying = False

                    print(f"已连接。文件初值={last_flag}；等待 0→1 起飞、1→0 降落。")

                    while True:
                        flag = read_flag(PARAM_FILE)

                        # 0 -> 1：起飞
                        if last_flag == 0 and flag == 1 and not is_flying:
                            print("检测到 0→1：起飞中…")
                            mc.take_off(height=TAKEOFF_HEIGHT)  # 速度用库默认值
                            is_flying = True
                            print("Yeah, I'm connected! :D（已起飞）")

                        # 1 -> 0：降落
                        if last_flag == 1 and flag == 0 and is_flying:
                            print("检测到 1→0：降落中…")
                            mc.land()  # 速度用库默认值
                            is_flying = False
                            print("Now I will disconnect :'(（已降落）")

                        last_flag = flag
                        time.sleep(POLL_SEC)

        except KeyboardInterrupt:
            print("\n收到中断信号，退出。")
            break
        except Exception as e:
            logging.exception("连接/通信异常，将自动重连：%s", e)
            time.sleep(1.0)
