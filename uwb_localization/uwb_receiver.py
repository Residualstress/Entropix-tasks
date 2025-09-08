import serial
import struct
import threading
from queue import Queue, Empty
import math

class UWBReceiver:
    def __init__(self, port="COM7", baudrate=115200, custom_filter=None):
        """
        Parameters:
            port (str): serial port name (e.g. "COM7")
            baudrate (int): serial baudrate (default: 115200)
            custom_filter (UKF): custom filter (default: None)
        """
        self.ser = serial.Serial(port, baudrate, timeout=0.1)
        self.running = False
        # 状态机变量
        self.state = "WAIT_HEAD"
        self.buffer = bytearray()
        self.cmd_len = 0
        self.expected_len = 0
        # 最新解析数据（线程安全访问）
        self.data_queue = Queue(maxsize=100) # th_meas, r_meas, th_filt, r_filt
        # 滤波器配置
        self.filter = custom_filter

    def start(self):
        """
        Start the serial reading loop.

        This method starts a new daemon thread to read the serial data
        and parse it. The parsed data will be stored in the internal queue
        and can be retrieved with `has_data()` and `pop_data()` methods.
        """
        self.running = True
        self.thread = threading.Thread(target=self.read_loop, daemon=True)
        self.thread.start()
        print('start')

    def stop(self):
        self.running = False
        if self.ser.is_open:
            self.ser.close()

    def read_loop(self):
        """
        Read serial data loop.

        This method reads the serial data and parses it. If a complete packet
        is received, it will be parsed and stored in the internal queue. The
        parsed data can be retrieved with `has_data()` and `pop_data()` methods.

        The state machine is as follows:

        * WAIT_HEAD: wait for the head byte (0x2A)
        * READ_LEN: read the cmd_len field (2 bytes) and the cmd_len content
        * READ_PAYLOAD: read the payload (cmd_len bytes)
        * READ_CHECK: read the check byte (1 byte)
        * WAIT_FOOT: wait for the foot byte (0x23)

        The state machine will be reset to WAIT_HEAD if an error occurs or
        a complete packet is received.

        :return: None
        """
        while self.running:
            byte = self.ser.read(1)
            if not byte:
                continue
            b = byte[0]

            if self.state == "WAIT_HEAD":
                if b == 0x2A:
                    self.buffer = bytearray([b])
                    self.state = "READ_LEN"

            elif self.state == "READ_LEN":
                self.buffer.append(b)
                if len(self.buffer) == 3:
                    # 解析长度字段 (cmd_len)
                    self.cmd_len = struct.unpack_from("<H", self.buffer, 1)[0]
                    # 总长度 = head(1) + cmd_len字段(2) + cmd_len内容 + check(1) + foot(1)
                    self.expected_len = 1 + 2 + self.cmd_len + 1 + 1
                    if self.expected_len < 81:  # 协议最小长度保护
                        self.state = "WAIT_HEAD"
                    else:
                        self.state = "READ_PAYLOAD"

            elif self.state == "READ_PAYLOAD":
                self.buffer.append(b)
                if len(self.buffer) == self.expected_len - 2:  # 留下 check + foot
                    self.state = "READ_CHECK"

            elif self.state == "READ_CHECK":
                self.buffer.append(b)
                self.state = "WAIT_FOOT"

            elif self.state == "WAIT_FOOT":
                self.buffer.append(b)
                if b == 0x23:
                    # 收到完整包
                    parsed = self.parse_packet(self.buffer)
                    if parsed:
                        th_meas, r_meas = parsed['self_raw_angle'], parsed['self_raw_range']
                        x_meas, y_meas =   r_meas * math.sin(math.radians(th_meas)), - r_meas * math.cos(math.radians(th_meas))
                        if self.filter is not None:
                            x_filt, y_filt = self.filter.update(x_meas, y_meas)
                        else:
                            th_filt, r_filt = parsed['tag_a0_filter_angle'], parsed['tag_a0_filter_range']
                            x_filt, y_filt = r_filt * math.sin(math.radians(th_filt)), - r_filt * math.cos(math.radians(th_filt))
                        self.data_queue.put((x_meas, y_meas, x_filt, y_filt))
                # 无论对不对，都重置状态机
                self.state = "WAIT_HEAD"

    def checksum(self, payload: bytes) -> int:
        """ 校验和: cmd_saddr 到 reserve 的异或结果 """
        xor_crc = 0
        for b in payload:
            xor_crc ^= b
        return xor_crc


    def parse_packet(self, packet: bytes):
        if len(packet) < 81:
            return None
        if packet[0] != 0x2A or packet[-1] != 0x23:
            return None

        # 按协议解析
        try:
            cmd_len = struct.unpack_from("<H", packet, 1)[0]
            cmd_saddr = struct.unpack_from("<Q", packet, 3)[0]
            cmd_daddr = struct.unpack_from("<Q", packet, 11)[0]
            cmd_type = packet[19]
            cmd_direct = packet[20]
            tag_time = struct.unpack_from("<I", packet, 21)[0]
            anc_addr16 = struct.unpack_from("<H", packet, 25)[0]
            tag_addr16 = struct.unpack_from("<H", packet, 27)[0]
            tag_sn = packet[29]
            tag_mask = packet[30]
            tag_a0_filter_angle = struct.unpack_from("<h", packet, 31)[0]
            tag_a0_filter_range = struct.unpack_from("<H", packet, 33)[0]
            tag_a1_filter_angle = struct.unpack_from("<h", packet, 35)[0]
            tag_a1_filter_range = struct.unpack_from("<H", packet, 37)[0]
            tag_a2_filter_angle = struct.unpack_from("<h", packet, 39)[0]
            tag_a2_filter_range = struct.unpack_from("<H", packet, 41)[0]
            tag_a3_filter_angle = struct.unpack_from("<h", packet, 43)[0]
            tag_a3_filter_range = struct.unpack_from("<H", packet, 45)[0]
            tag_detail_para = struct.unpack_from("<I", packet, 47)[0]
            tag_acc_x = struct.unpack_from("<H", packet, 51)[0]
            tag_acc_y = struct.unpack_from("<H", packet, 53)[0]
            tag_acc_z = struct.unpack_from("<H", packet, 55)[0]
            tag_gcc_x = struct.unpack_from("<H", packet, 57)[0]
            tag_gcc_y = struct.unpack_from("<H", packet, 59)[0]
            tag_gcc_z = struct.unpack_from("<H", packet, 61)[0]
            self_raw_angle = struct.unpack_from("<h", packet, 63)[0]
            self_raw_range = struct.unpack_from("<H", packet, 65)[0]
            self_raw_degree = struct.unpack_from("<i", packet, 67)[0]
            angle_360 = struct.unpack_from("<H", packet, 71)[0]
            angle_dir = packet[73]
            reserve = packet[74:79]
            check = packet[79]

            # 校验
            payload = packet[3:79]  # 从 cmd_saddr 到 reserve
            if check != self.checksum(payload):
                print("Checksum error!")
                return None

            return {
                "cmd_len": cmd_len,
                "cmd_saddr": cmd_saddr,
                "cmd_daddr": cmd_daddr,
                "cmd_type": cmd_type,
                "cmd_direct": cmd_direct,
                "tag_time": tag_time,
                "anc_addr16": anc_addr16,
                "tag_addr16": tag_addr16,
                "tag_sn": tag_sn,
                "tag_mask": tag_mask,
                "tag_a0_filter_angle": tag_a0_filter_angle,
                "tag_a0_filter_range": tag_a0_filter_range,
                "tag_a1_filter_angle": tag_a1_filter_angle,
                "tag_a1_filter_range": tag_a1_filter_range,
                "tag_a2_filter_angle": tag_a2_filter_angle,
                "tag_a2_filter_range": tag_a2_filter_range,
                "tag_a3_filter_angle": tag_a3_filter_angle,
                "tag_a3_filter_range": tag_a3_filter_range,
                "tag_detail_para": tag_detail_para,
                "tag_acc_x": tag_acc_x,
                "tag_acc_y": tag_acc_y,
                "tag_acc_z": tag_acc_z,
                "tag_gcc_x": tag_gcc_x,
                "tag_gcc_y": tag_gcc_y,
                "tag_gcc_z": tag_gcc_z,
                "self_raw_angle": self_raw_angle,
                "self_raw_range": self_raw_range,
                "self_raw_degree": self_raw_degree,
                "angle_360": angle_360,
                "angle_dir": angle_dir,
                "reserve": reserve,
            }
        except Exception as e:
            print("Parse error:", e)
            return None
        
    def has_data(self) -> bool:
        """
        Check if there is any data in the internal queue.

        Returns:
            bool: True if the queue is not empty, False otherwise.
        """
        return not self.data_queue.empty() 
    
    def pop_data(self):
        """
        Try to pop a data packet from the internal queue.

        If the queue is empty, return None.

        Returns:
            Tuple(x_meas, y_meas, x_filt, y_filt) or None: A data packet, or None if the queue is empty.
        """
        try:
            return self.data_queue.get_nowait()
        except Empty:
            return None
